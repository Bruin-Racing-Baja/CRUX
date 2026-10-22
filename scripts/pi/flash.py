#!/home/main/esp-flash-venv/bin/python3
"""
Runs on the Pi. Flashes ESP32s from a build dir that was rsynced over.

  ./flash.py list                      what serial ports does the Pi see?
  ./flash.py check <name>              talk to the chip, read its ID (writes nothing)
  ./flash.py flash <name> <build_dir>  flash bootloader + partitions + app
  ./flash.py flash all <build_dir>     flash every device in devices.json

Setup on the Pi:  pip install esptool pyserial gpiozero lgpio
(gpiozero/lgpio only needed if a device uses en_gpio/boot_gpio)
"""
import glob, json, os, shlex, subprocess, sys, time, signal
from contextlib import contextmanager

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = json.load(open(os.path.join(HERE, "devices.json")))
ESPTOOL = [sys.executable, "-m", "esptool"]


def cmd_list():
    print("Serial devices the Pi sees:")
    found = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    print("  " + ("\n  ".join(found) if found else "(none: ESPs are probably not on USB)"))
    print("\nStable names (use these in devices.json so order never swaps):")
    byid = sorted(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/serial/by-path/*"))
    for p in byid:
        print(f"  {p} -> {os.path.realpath(p)}")
    if not byid:
        print("  (none)")
    print("\nOn-board UARTs (if wired to Pi header pins):")
    for p in sorted(glob.glob("/dev/serial*") + glob.glob("/dev/ttyAMA*")):
        print("  " + p)


class BootCtl:
    """Optional: Pi GPIOs wired to EN and IO0 to force download mode."""

    def __init__(self, dev):
        self.on = "en_gpio" in dev and "boot_gpio" in dev
        if self.on:
            from gpiozero import OutputDevice
            self.en = OutputDevice(dev["en_gpio"], initial_value=True)
            self.boot = OutputDevice(dev["boot_gpio"], initial_value=True)

    def enter_bootloader(self):
        if not self.on:
            return
        self.boot.off()          # IO0 low = download mode
        self.en.off(); time.sleep(0.1)
        self.en.on();  time.sleep(0.3)   # chip samples IO0 as it leaves reset
        self.boot.on()

    def reset_to_app(self):
        if not self.on:
            return
        self.boot.on()
        self.en.off(); time.sleep(0.1)
        self.en.on()


def run(args, cwd=None):
    print("$", " ".join(shlex.quote(a) for a in args))
    return subprocess.run(args, cwd=cwd).returncode


def base_args(dev):
    a = ESPTOOL + ["--chip", dev["chip"], "-p", dev["port"], "-b", str(dev.get("baud", 115200))]
    if "en_gpio" in dev:  # we handle reset ourselves
        a += ["--before", "no_reset", "--after", "no_reset"]
    return a


def cmd_check(name):
    dev = CFG[name]
    with port_free(dev):
        ctl = BootCtl(dev)
        ctl.enter_bootloader()
        rc = run(base_args(dev) + ["chip_id"])
        ctl.reset_to_app()
    return rc


def cmd_flash(name, build_dir):
    dev = CFG[name]
    flash_args = os.path.join(build_dir, "flash_args")
    if not os.path.exists(flash_args):
        sys.exit(f"no flash_args in {build_dir}, did the rsync work?")
    # flash_args: first line = options, other lines = "<offset> <file>"
    lines = [l.strip() for l in open(flash_args) if l.strip()]
    opts = shlex.split(lines[0]) if lines[0].startswith("--") else []
    pairs = [x for l in lines if not l.startswith("--") for x in l.split()]
    for f in pairs[1::2]:
        if not os.path.exists(os.path.join(build_dir, f)):
            sys.exit(f"missing file listed in flash_args: {f}")
    with port_free(dev):
        ctl = BootCtl(dev)
        ctl.enter_bootloader()
        rc = run(base_args(dev) + ["write_flash"] + opts + pairs, cwd=build_dir)
        ctl.reset_to_app()
    print(f"[{name}] {'OK' if rc == 0 else 'FAILED'}")
    return rc

def port_holders(port):
    """Processes (other than us) that have this serial port open."""
    real = os.path.realpath(port)
    out = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == os.getpid():
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                if os.path.realpath(f"/proc/{pid}/fd/{fd}") == real:
                    raw = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")[:-1]
                    out.append({"pid": int(pid),
                                "cmd": [c.decode() for c in raw],
                                "cwd": os.readlink(f"/proc/{pid}/cwd")})
                    break
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
    return out


def stop_holders(holders):
    for h in holders:
        print(f"stopping {' '.join(h['cmd'])} (pid {h['pid']})")
        os.kill(h["pid"], signal.SIGTERM)
    for h in holders:                      # give each up to 5 s to exit cleanly
        for _ in range(50):
            if not os.path.exists(f"/proc/{h['pid']}"):
                break
            time.sleep(0.1)
        else:
            os.kill(h["pid"], signal.SIGKILL)


def wait_for_port(port, timeout=15):
    """After a flash the board reboots and its USB port vanishes briefly."""
    time.sleep(1.5)   # let the board actually reset first
    end = time.time() + timeout
    while time.time() < end and not os.path.exists(port):
        time.sleep(0.2)
    time.sleep(1)


def restart_holders(holders):
    log = open(os.path.expanduser("~/flash-tool/relaunch.log"), "a")
    for h in holders:
        print(f"restarting {' '.join(h['cmd'])}")
        subprocess.Popen(h["cmd"], cwd=h["cwd"], stdin=subprocess.DEVNULL,
                         stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True)   # survives the ssh session ending


@contextmanager
def port_free(dev):
    """Use as:  with port_free(dev): ...flash...   Logger is paused inside."""
    if "stop_cmd" in dev:                  # Way 2 (see below)
        subprocess.run(dev["stop_cmd"], shell=True)
        try:
            yield
        finally:
            wait_for_port(dev["port"])
            subprocess.run(dev["start_cmd"], shell=True)
        return
    holders = port_holders(dev["port"])
    stop_holders(holders)
    try:
        yield
    finally:
        if holders:
            wait_for_port(dev["port"])
            restart_holders(holders)


def main():
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    if a[0] == "list":
        return cmd_list()
    if a[0] in ("check", "flash") and len(a) >= 2:
        names = list(CFG) if a[1] == "all" else [a[1]]
        for n in names:
            if n not in CFG:
                sys.exit(f"unknown device '{n}'. known: {', '.join(CFG)}")
        if a[0] == "check":
            sys.exit(max(cmd_check(n) for n in names))
        if len(a) < 3:
            sys.exit("flash needs a build dir")
        sys.exit(max(cmd_flash(n, a[2]) for n in names))
    sys.exit(__doc__)


if __name__ == "__main__":
    main()