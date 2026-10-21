#!/usr/bin/env python3
"""
Runs on your laptop. Builds an ESP-IDF project, ships the binaries to the Pi,
and tells the Pi to flash them.

Repo layout this expects:
    controls/   (ESP-IDF project)
    daq/        (ESP-IDF project)
    scripts/    (this file + pi/flash.py + pi/devices.json)

Usage (run from anywhere; set PI if your Pi isn't pi@raspberrypi.local):
    ./scripts/deploy.py sync                 # copy scripts/pi/ -> ~/flash-tool on the Pi
    ./scripts/deploy.py daq                  # build + send + flash daq
    ./scripts/deploy.py controls             # build + send + flash controls
    ./scripts/deploy.py all                  # both
    ./scripts/deploy.py daq --no-flash       # build + send only
    ./scripts/deploy.py daq --no-build       # reuse the last build

Needs the ESP-IDF environment active in your shell (export.sh) so `idf.py` works.
The name (controls / daq) is the link between: project folder -> folder on the
Pi (~/firmware/<name>) -> entry in devices.json -> serial port.
"""
import argparse, os, shlex, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECTS = ["controls", "daq"]
PI = os.environ.get("PI", "main@10.42.0.1")


def run(cmd):
    print("$", " ".join(shlex.quote(c) for c in cmd))
    subprocess.run(cmd, check=True)


def send_dir(local_dir, files, remote_dir, wipe=True):
    """tar | ssh | tar: copies files (relative to local_dir), keeping subfolders.
    Uses only tar + ssh, so no rsync needed (works on Windows 10+ too)."""
    prep = f"mkdir -p {remote_dir}"
    if wipe:
        prep = f"rm -rf {remote_dir} && {prep}"
    tar = subprocess.Popen(["tar", "-cf", "-", "-C", local_dir] + files,
                           stdout=subprocess.PIPE)
    ssh = subprocess.Popen(["ssh", PI, f"{prep} && tar -xf - -C {remote_dir}"],
                           stdin=tar.stdout)
    tar.stdout.close()
    if ssh.wait() != 0 or tar.wait() != 0:
        sys.exit("transfer to Pi failed")


def sync_tool():
    pi_dir = os.path.join(ROOT, "scripts", "pi")
    send_dir(pi_dir, ["flash.py", "devices.json"], "~/flash-tool", wipe=False)
    run(["ssh", PI, "chmod +x ~/flash-tool/flash.py"])


def deploy(name, build=True, flash=True):
    proj = os.path.join(ROOT, name)
    if not os.path.isdir(proj):
        sys.exit(f"no project folder: {proj}")
    if build:
        run(["idf.py", "-C", proj, "build"])

    bdir = os.path.join(proj, "build")
    flash_args = os.path.join(bdir, "flash_args")
    if not os.path.exists(flash_args):
        sys.exit(f"{flash_args} not found: build first (drop --no-build)")
    # flash_args lines look like "0x10000 daq.bin"; the first line is options
    files = [l.split()[1] for l in open(flash_args) if l.startswith("0x")]
    send_dir(bdir, ["flash_args"] + files, f"~/firmware/{name}")

    if flash:
        run(["ssh", PI, f"~/flash-tool/flash.py flash {name} ~/firmware/{name}"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", choices=["sync", "all"] + PROJECTS)
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--no-flash", action="store_true")
    a = ap.parse_args()

    if a.target == "sync":
        return sync_tool()
    for name in (PROJECTS if a.target == "all" else [a.target]):
        print(f"\n=== {name} ===")
        deploy(name, build=not a.no_build, flash=not a.no_flash)


if __name__ == "__main__":
    main()