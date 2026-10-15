"""
Centerlock Controller — Raspberry Pi 5
=======================================
Rewrite of the ESP32 centerlock_controller.cpp/.h for Pi + Canable 2.0 + ODrive Pro.

Hardware:
  CAN  : Canable 2.0 (candleLight firmware) on can0
  GPIO : gpiozero + lgpio backend (Pi 5 compatible)
    PIN_BUTTON_IN       (GPIO17) — "Shift In"  button,  active-low with internal pullup
    PIN_BUTTON_OUT      (GPIO27) — "Shift Out" button,  active-low with internal pullup
    PIN_LIMIT_INBOUND   (GPIO22) — Inbound  limit switch, active-low with internal pullup
    PIN_LIMIT_OUTBOUND  (GPIO23) — Outbound limit switch, active-low with internal pullup
    PIN_LED             (GPIO24) — Status LED

Dependencies:
    pip3 install python-can gpiozero lgpio

CAN interface is brought up automatically at startup via subprocess + sudo.
To avoid needing sudo every run, add your user to the netdev group:
    sudo usermod -aG netdev $USER
"""

import subprocess
import sys
import can
import struct
import time
import threading
import logging

from gpiozero import Button, LED
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# Force lgpio backend — required for Raspberry Pi 5
Device.pin_factory = LGPIOFactory()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
log = logging.getLogger("centerlock")

# ── GPIO pin assignments (BCM numbering) ──────────────────────────────────────

PIN_BUTTON_IN      = 17  # "Shift In"  button
PIN_BUTTON_OUT     = 27  # "Shift Out" button
PIN_LED            = 4  # Status LED

PIN_LIMIT_INBOUND  = 22  # Inbound  limit switch
PIN_LIMIT_OUTBOUND = 23  # Outbound limit switch

# ── CAN / ODrive configuration ────────────────────────────────────────────────

SLCAN_PORT     = "/dev/canable"  # Symlink set by udev rule
CAN_INTERFACE  = "can0"
CAN_BITRATE    = 250000
ODRIVE_NODE_ID = 0x03            # Match odrv0.axis0.config.can.node_id

# ── Centerlock motion parameters ──────────────────────────────────────────────

CENTERLOCK_DIR      =  -1      # Flip to -1 if motor direction is reversed
CENTERLOCK_VEL      = 10.0    # Normal running velocity (rev/s)
CENTERLOCK_HOME_VEL =  6.0    # Slower homing velocity  (rev/s)
HOME_TIMEOUT_S      = 20.0    # Abort homing if limit not hit within this time

# ── ODrive CAN protocol constants ─────────────────────────────────────────────

CMD_HEARTBEAT          = 0x01
CMD_GET_ENCODER        = 0x09
CMD_SET_AXIS_STATE     = 0x07
CMD_SET_CTRL_MODES     = 0x0B
CMD_SET_INPUT_VEL      = 0x0D

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8
CTRL_MODE_VELOCITY     = 2
INPUT_MODE_PASSTHROUGH = 1
# ── State enum ────────────────────────────────────────────────────────────────

class State:
    IN      = "IN"
    OUT     = "OUT"
    UNKNOWN = "UNKNOWN"

# ── CAN interface setup ───────────────────────────────────────────────────────

def setup_can_interface():
    """
    Bring up the CAN interface via slcand (slcan firmware).
    Equivalent to:
        sudo slcand -o -c -s5 /dev/canable can0
        sudo ip link set can0 up
        sudo ip link set can0 txqueuelen 1000

    Bitrate is controlled by the -s flag:
        s4 = 125k, s5 = 250k, s6 = 500k, s8 = 1M
    """
    # log.info("Configuring CAN interface via slcand (%s -> %s) ...",
    #          SLCAN_PORT, CAN_INTERFACE)

    # Kill any existing slcand and bring interface down (ignore errors)
    subprocess.run(["sudo", "killall", "slcand"], capture_output=True)
    subprocess.run(["sudo", "ip", "link", "set", CAN_INTERFACE, "down"],
                   capture_output=True)

    # Start slcand: -o = open device, -c = close on exit, -s5 = 250kbps
    result = subprocess.run(
        ["sudo", "slcand", "-o", "-c", "-s5", SLCAN_PORT, CAN_INTERFACE],
        capture_output=True, text=True
    )
    if result.returncode != 0:
    #     log.error("slcand failed: %s", result.stderr.strip())
          sys.exit(1)

    time.sleep(0.2)  # give slcand a moment to initialise

    # Bring interface up
    result = subprocess.run(
        ["sudo", "ip", "link", "set", CAN_INTERFACE, "up"],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        log.error("Failed to bring up %s: %s", CAN_INTERFACE, result.stderr.strip())
        sys.exit(1)

    # Set TX queue length (recommended for slcan)
    subprocess.run(
        ["sudo", "ip", "link", "set", CAN_INTERFACE, "txqueuelen", "1000"],
        capture_output=True
    )

    log.info("CAN interface %s is up.", CAN_INTERFACE)

# ── Low-level CAN helpers ─────────────────────────────────────────────────────

def _arb(node_id, cmd_id):
    return (node_id << 5) | cmd_id


def _send(bus, cmd_id, payload=b""):
    data = payload.ljust(8, b"\x00")
    msg = can.Message(
        arbitration_id=_arb(ODRIVE_NODE_ID, cmd_id),
        data=data,
        is_extended_id=False
    )
    bus.send(msg)


def odrive_set_axis_state(bus, state):
    _send(bus, CMD_SET_AXIS_STATE, struct.pack("<I", state))


def odrive_set_velocity_mode(bus):
    _send(bus, CMD_SET_CTRL_MODES,
          struct.pack("<II", CTRL_MODE_VELOCITY, INPUT_MODE_PASSTHROUGH))


def odrive_set_vel(bus, vel_rev_s):
    _send(bus, CMD_SET_INPUT_VEL, struct.pack("<ff", vel_rev_s, 0.0))


def odrive_idle(bus):
    odrive_set_axis_state(bus, AXIS_STATE_IDLE)


def odrive_arm_velocity(bus, vel_rev_s):
    """Set velocity mode, enter closed loop, send velocity command."""
    odrive_set_velocity_mode(bus)
    odrive_set_axis_state(bus, AXIS_STATE_CLOSED_LOOP)
    time.sleep(0.05)
    odrive_set_vel(bus, vel_rev_s)

# ── CenterlockController ──────────────────────────────────────────────────────

class CenterlockController:

    CONTROL_LOOP_HZ = 100  

    def __init__(self):
        self.bus         = None
        self.state       = State.UNKNOWN
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()

        # gpiozero devices (assigned in init)
        self._btn_in    = None
        self._btn_out   = None
        self._limit_in  = None
        self._limit_out = None
        self._led       = None

    # ── Startup ───────────────────────────────────────────────────────────────

    def init(self):
        """Equivalent to CenterlockController::init() in the ESP code."""

        # ── Bring up CAN interface ────────────────────────────────────────────
        setup_can_interface()

        # ── GPIO setup ────────────────────────────────────────────────────────
        # pull_up=True + active-low wiring means is_pressed=True when triggered
        # bounce_time in seconds
        self._btn_in    = Button(PIN_BUTTON_IN,      pull_up=True, bounce_time=0.01)
        self._btn_out   = Button(PIN_BUTTON_OUT,     pull_up=True, bounce_time=0.01)
        self._limit_in  = Button(PIN_LIMIT_INBOUND,  pull_up=True, bounce_time=0.01)
        self._limit_out = Button(PIN_LIMIT_OUTBOUND, pull_up=True, bounce_time=0.01)
        self._led       = LED(PIN_LED)

        # Wire button press callbacks (fires on falling edge)
        self._btn_in.when_pressed  = self._button_in_cb
        self._btn_out.when_pressed = self._button_out_cb

        # ── Open CAN bus ──────────────────────────────────────────────────────
        # log.info("Opening CAN bus %s ...", CAN_INTERFACE)
        self.bus = can.Bus(interface="socketcan",
                           channel=CAN_INTERFACE,
                           bitrate=CAN_BITRATE)

        # ── Wait for ODrive heartbeat (LED blinks while waiting) ──────────────
        log.info("Waiting for ODrive heartbeat (node 0x%02x) ...", ODRIVE_NODE_ID)
        result = self._wait_for_heartbeat(timeout=60.0)
        while not result:
            result = self._wait_for_heartbeat(timeout=60.0)
        # ── Home ──────────────────────────────────────────────────────────────
        homed = self._home()
        if homed:
            log.info("Centerlock homed successfully.")
        else:
            log.warning("Homing failed or timed out — state is UNKNOWN.")

        # ── Start control loop thread ─────────────────────────────────────────
        self._thread = threading.Thread(
            target=self._control_loop,
            name="centerlock_ctrl",
            daemon=True
        )
        self._thread.start()
        # log.info("Control loop running at %d Hz.", self.CONTROL_LOOP_HZ)

    def stop(self):
        """Cleanly shut down the controller."""
        self._stop_event.set()
        if self.bus:
            try:
                odrive_idle(self.bus)
            except Exception:
                pass
            self.bus.shutdown()
        for dev in [self._btn_in, self._btn_out,
                    self._limit_in, self._limit_out, self._led]:
            if dev:
                dev.close()
        # log.info("Centerlock stopped.")

    # ── GPIO helpers ──────────────────────────────────────────────────────────

    def _get_limit_inbound(self):
        """True when inbound limit switch is triggered (active-low)."""
        return self._limit_in.is_pressed

    def _get_limit_outbound(self):
        """True when outbound limit switch is triggered (active-low)."""
        return self._limit_out.is_pressed

    # ── Heartbeat wait ────────────────────────────────────────────────────────

    def _wait_for_heartbeat(self, timeout=10.0):
        """
        Block until ODrive heartbeat is seen.
        LED blinks at ~5 Hz while waiting, goes solid on success,
        goes off on timeout (then raises).
        """
        led_state = False
        expected   = _arb(ODRIVE_NODE_ID, CMD_HEARTBEAT)
        deadline   = time.time() + timeout
        last_blink = time.time()

        while time.time() < deadline:
            # Blink LED at ~5 Hz
            if time.time() - last_blink >= 0.1:
                led_state = not led_state
                self._led.on() if led_state else self._led.off()
                last_blink = time.time()

            msg = self.bus.recv(timeout=0.05)
            if msg and msg.arbitration_id == expected:
                self._led.on()  # Solid = good
                # log.info("Heartbeat received — ODrive is alive.")
                return True

        self._led.off()
        return False
        raise RuntimeError(
            f"No heartbeat from ODrive node 0x{ODRIVE_NODE_ID:02x} within {timeout}s. "
            "Check wiring, bitrate, and node ID."
        )

    # ── Homing ────────────────────────────────────────────────────────────────

    def _home(self):
        """
        Mirror of CenterlockController::home().
        Drives outbound until the outbound limit switch triggers, then idles.
        If already at inbound limit, sets state to IN immediately.
        """
        led_state = False
        # log.info("Homing: driving to outbound limit ...")
        time.sleep(0.5)

        if self._get_limit_inbound():
            odrive_idle(self.bus)
            with self._lock:
                self.state = State.IN
            # log.info("Already at inbound limit — homed to IN.")
            self._led.on()
            return True

        odrive_arm_velocity(self.bus, -CENTERLOCK_DIR * CENTERLOCK_HOME_VEL)

        start = time.time()
        last_blink = time.time()
        while not self._get_limit_outbound():
            if time.time() - last_blink >= 0.1:
                led_state = not led_state
                self._led.on() if led_state else self._led.off()
                last_blink = time.time()
            if time.time() - start > HOME_TIMEOUT_S:
                log.error("Homing timeout after %.1fs.", HOME_TIMEOUT_S)
                odrive_idle(self.bus)
                return False
            time.sleep(0.01)
        self._led.off()

        odrive_idle(self.bus)
        with self._lock:
            self.state = State.OUT
        log.info("Homing complete — at OUT limit.")
        return True

    # ── Button callbacks ──────────────────────────────────────────────────────

    def _button_in_cb(self):
        """Fires on falling edge of PIN_BUTTON_IN. Mirror of shift_in_button_isr."""
        #log.info("Button IN pressed → state = IN")
        with self._lock:
            self.state = State.IN

    def _button_out_cb(self):
        """Fires on falling edge of PIN_BUTTON_OUT. Mirror of shift_out_button_isr."""
        #log.info("Button OUT pressed → state = OUT")
        with self._lock:
            self.state = State.OUT

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        """
        Mirror of CenterlockController::control_loop().
        Runs at CONTROL_LOOP_HZ. Drives motor toward target state,
        idles when the corresponding limit switch is hit.
        """
        led_state = False
        period = 1.0 / self.CONTROL_LOOP_HZ
        cycle  = 0

        while not self._stop_event.is_set():
            t_start = time.time()
            cycle  += 1

            with self._lock:
                current_state = self.state

            #try:
            if current_state == State.IN:
                if not self._get_limit_inbound():
                    odrive_arm_velocity(self.bus,
                                        CENTERLOCK_DIR * CENTERLOCK_VEL)
                    if cycle % 3 == 0:
                        led_state = not led_state 
                        self._led.on() if led_state else self._led.off()
                else:
                    self._led.on()
                    odrive_idle(self.bus)

            elif current_state == State.OUT:
                if not self._get_limit_outbound():
                    odrive_arm_velocity(self.bus,
                                        -CENTERLOCK_DIR * CENTERLOCK_VEL)
                    if cycle % 3 == 0:
                        led_state = not led_state 
                        self._led.on() if led_state else self._led.off()
                else:
                    self._led.off()
                    odrive_idle(self.bus)

                # UNKNOWN: do nothing, motor stays idle

            # except Exception as e:
            #     log.error("Control loop error: %s", e)

            # Log at ~2 Hz
            # if cycle % (self.CONTROL_LOOP_HZ // 2) == 0:
            #     log.info(
            #         "state=%-7s  inbound=%d  outbound=%d",
            #         current_state,
            #         self._get_limit_inbound(),
            #         self._get_limit_outbound()
            #     )

            elapsed   = time.time() - t_start
            sleep_for = period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    controller = CenterlockController()
    try:
        controller.init()
        #log.info("Running — press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    #except KeyboardInterrupt:
        #log.info("Interrupted.")
    #except Exception as e:
        #log.exception("Fatal error: %s", e)
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
