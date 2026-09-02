#!/usr/bin/env python3
"""Boot-mode pty validation harness for me7eeprom.

Spins up a pseudo-terminal that performs K-line loopback echo only (no ECU
responder), runs the tool against it in boot mode, and asserts that the
per-stage transcript prints live. With no ECU answering the bootstrap loader,
the run must reach the uC-ID read timeout and report "FAIL ... No ECU
response" — exercising the boot-mode serial path with no hardware.

  writeByteEcho(0x00)  -> echo arrives  -> OK
  readN(ucId,1)       -> no response   -> read timeout -> "FAIL ... No ECU response"

Usage:
    python3 re/scripts/pty_boot_validate.py [tool-path]
"""
import os
import pty
import queue
import subprocess
import sys
import threading
import time

TOOL = sys.argv[1] if len(sys.argv) > 1 else "build/me7eeprom"
BAUD = "9600"


def main():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print(f"[harness]  pty slave = {slave_name}", flush=True)

    def echo_loop():
        try:
            while True:
                try:
                    data = os.read(master, 4096)
                except OSError:
                    break
                if not data:
                    break
                os.write(master, data)   # K-line loopback echo only
        except Exception as e:           # pragma: no cover
            print(f"[harness] echo thread: {e!r}")

    t_echo = threading.Thread(target=echo_loop, daemon=True)
    t_echo.start()

    env = dict(os.environ, ME7_DEVICE=slave_name)
    cmd = [TOOL, "--bootmode", "95040", "--CSpin", "P4.7",
           "-r", "-p", "1", "-b", BAUD, "/tmp/me7_boot_pty.bin"]
    print(f"[harness]  running: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, env=env, text=True, capture_output=True, timeout=60)

    try:
        os.close(slave)
    except OSError:
        pass
    t_echo.join(timeout=5)
    try:
        os.close(master)
    except OSError:
        pass

    print("=== tool stdout ===")
    print(proc.stdout)
    if proc.stderr:
        print("=== tool stderr ===")
        print(proc.stderr)
    print(f"=== exit code: {proc.returncode} ===")

    # The fix: the stage prefix must now print live (it was buffered+overwritten
    # before, so "Starting Boot_mode ... " was missing entirely).
    has_prefix = "Starting Boot_mode ..." in proc.stdout
    has_fail = "FAIL ... No ECU response" in proc.stdout
    print(f"has 'Starting Boot_mode ...' : {has_prefix}")
    print(f"has 'FAIL ... No ECU response': {has_fail}")
    ok = has_prefix and has_fail
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
