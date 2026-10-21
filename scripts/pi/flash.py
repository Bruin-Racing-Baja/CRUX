#!/usr/bin/env python3
"""
Runs on the Pi. Flashes ESP32s from a build dir that was rsynced over.

  ./flash.py list                      what serial ports does the Pi see?
  ./flash.py check <name>              talk to the chip, read its ID (writes nothing)
  ./flash.py flash <name> <build_dir>  flash bootloader + partitions + app
  ./flash.py flash all <build_dir>     flash every device in devices.json

Setup on the Pi:  pip install esptool pyserial gpiozero lgpio
(gpiozero/lgpio only needed if a device uses en_gpio/boot_gpio)
"""
import glob, json, os, shlex, subprocess, sys, time

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
    ctl = BootCtl(dev)
    ctl.enter_bootloader()
    rc = run(base_args(dev) + ["write_flash"] + opts + pairs, cwd=build_dir)
    ctl.reset_to_app()
    print(f"[{name}] {'OK' if rc == 0 else 'FAILED'}")
    return rc


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