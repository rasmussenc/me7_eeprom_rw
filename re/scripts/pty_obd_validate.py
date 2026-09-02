#!/usr/bin/env python3
"""End-to-end OBD-read validation of the native me7eeprom against a pty.

Creates a pty, plays the ME7 ECU on the master (K-line loopback echo plus the
KWPi-ish OBD responses the tool expects), and drives the tool against the slave
through the real libserialport backend via ME7_DEVICE. This exercises the full
serial stack + protocol framing with no hardware, and writes the resulting
EEPROM image so it can be inspected.

The response bytes come straight from src/core/protocol.cpp (obdInit/obdRead);
see the header comments there for the framing. baud is pinned to 9600 because
the non-standard K-line 10400 generally can't be set on a pty.

Usage:
    python3 re/scripts/pty_obd_validate.py [tool-path]
"""

import os
import pty
import queue
import subprocess
import sys
import threading
import time

TOOL = "build/me7eeprom"
OUT = "/tmp/me7_obd_pty.bin"
BAUD = "9600"

# Optional: serve a real immo-3 image (env ME7_IMAGE) so --immo's checksum
# verifier sees valid pages instead of the synthetic pattern.
_IMG_PATH = os.environ.get("ME7_IMAGE")
IMG = bytearray(open(_IMG_PATH, "rb").read()) if _IMG_PATH else None

# --- OBD init: identification frames --------------------------------
# Wire frame read by obdRxFrame = [len][b1 b2(=F6) id-bytes...][03].
# The tool's ID string is ASCII7 of frame[3 .. len-1]; frame[2] is the SID.
#   -> frames 0..2 carry ECU-ID strings; frame 3 is the SoftCod/WSC frame
#      (frame[0]==5, i.e. len==8); frame 4 is the end frame (SID 9).
ID_FRAMES = [
    bytes([0x06, 0x00, 0xF6]) + b"ME7" + bytes([0x03]),   # ID "ME7"
    bytes([0x06, 0x00, 0xF6]) + b"BOS" + bytes([0x03]),   # ID "BOS"
    bytes([0x06, 0x00, 0xF6]) + b"C16" + bytes([0x03]),   # ID "C16"
    bytes([0x08, 0x00, 0xF6, 0x00, 0x00, 0x00, 0x34, 0x12, 0x03]),  # SoftCod/WSC
    bytes([0x03, 0x00, 0x09, 0x03]),                      # end (SID 9)
]
# Tool acks each ID frame with exactly this (see obdInit -> obdTxFrame).
TOOL_ACK = b"\x03\x01\x09\x03"


def block_data(addr):
    """Per-block 16 bytes served as Phase-1 page payload.

    By default a deterministic pattern (data[i] == addr+i) -- what the harness
    uses to verify a byte-faithful read. When ME7_IMAGE points at a real
    immo-3 .bin, block bytes are taken straight from that image so the tool's
    --immo verifier (per-page checksums) sees a *valid* EEPROM and proceeds to
    build an immo-off image -- exercising the success path end to end.
    """
    if IMG:
        start = min(addr, len(IMG) - 16)
        return IMG[start:start + 16]
    return bytes((addr + i) & 0xFF for i in range(16))


def main():
    tool = sys.argv[1] if len(sys.argv) > 1 else TOOL
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print(f"[harness]  pty slave = {slave_name}", flush=True)

    t0 = time.monotonic()
    def now():
        return (time.monotonic() - t0) * 1000.0

    # Bytes the tool transmitted (the echo thread enqueues them).
    q = queue.Queue()

    def echo_loop():
        try:
            while True:
                try:
                    data = os.read(master, 4096)
                except OSError:
                    break
                if not data:
                    break
                os.write(master, data)   # K-line loopback echo
                q.put(data)
        except Exception as e:           # pragma: no cover
            print(f"[harness] echo thread: {e!r}")

    # ------------------------------------------------------------------
    # ECU responder: a byte-level KWP-on-K-line peer.
    #
    # The wire discipline the tool expects (see obdTxFrame / obdRxFrame in
    # protocol.cpp) is byte-by-byte with a complement ack: every byte a side
    # transmits is answered by the other side with ~byte. So the two directions
    # serialize each other one byte at a time, and there is no buffering of whole
    # frames — a burst would put response bytes into the tool's read buffer ahead
    # of its own echo and corrupt writeByteEcho. Model exactly that discipline:
    #
    #   send_frame(data): send each byte except the 0x03 terminator, and after each
    #     non-terminator byte consume the tool's ~byte ack from q.
    #   recv_frame(): read the tool's bytes from q one at a time, and after each
    #     non-terminator byte write ~byte back (the ack the tool's obdTxFrame
    #     reads via its second readNTimeout). The terminator (0x03) is NOT acked.
    #
    # The pty's two independent channels keep this clean: writing the master
    # delivers bytes to the tool (slave reader) only; the echo_loop reads the
    # master, so it sees only what the *tool* writes to the slave (the tool's own
    # transmitted bytes), echoes those back (giving writeByteEcho its local echo)
    # and enqueues them in q for us. Our acks (master writes) never reach q.
    # ------------------------------------------------------------------
    stop = threading.Event()
    init_at = now() + 2300.0   # after the ~2 s 5-baud fast-init + purge

    def recv_byte(timeout=2.0):
        """One byte the tool transmitted (from q). Raises queue.Empty on timeout."""
        b = q.get(timeout=timeout)
        return b[0] if isinstance(b, (bytes, bytearray)) else b

    def ack(b):
        """Send the ECU's complement ack ~b for a tool-transmitted byte."""
        os.write(master, bytes([(~b) & 0xFF]))

    def recv_frame():
        """Receive a tool-transmitted KWP frame byte-by-byte, acking each byte
        except the 0x03 terminator. Returns the raw frame bytes [len][...][03]."""
        n = recv_byte()                 # length byte (index 0)
        ack(n)
        frame = bytearray([n])
        for _ in range(1, n):           # indices 1 .. n-1
            b = recv_byte()
            ack(b)
            frame.append(b)
        term = recv_byte()              # index n = 0x03 terminator (no ack)
        frame.append(term)
        return bytes(frame)

    def send_frame(data):
        """Send an ECU frame byte-by-byte, consuming the tool's ~byte ack after
        each byte except the 0x03 terminator. `data` = [len][...][03]."""
        last = len(data) - 1
        for i, b in enumerate(data):
            os.write(master, bytes([b]))
            if i < last:
                recv_byte()             # consume the tool's ~b ack

    def responder():
        try:
            # Wait for the tool's 5-baud fast-init (10 bits * 200 ms ~= 2 s) +
            # input purge to finish, then emit the keybyte wakeup. The tool's
            # first readNTimeout(.,1,500) sits in a 500 ms window after the purge.
            while now() < init_at:
                if stop.is_set():
                    return
                time.sleep(0.02)
            os.write(master, b"\x55\x01\x8a")
            print(f"[sim]     {now():8.0f}  TX 55018a   (init 55 01 8a)", flush=True)

            # Tool replies 0x75 (writeByteEcho, single echo — no ack of it).
            b = recv_byte()
            if b != 0x75:
                print(f"[sim]     expected 0x75 handshake, got 0x{b:02x}")
                return

            # ID frames: send each, then receive the tool's per-frame ack
            # (obdTxFrame `03 01 09 03`). The last ID frame is the SID-9 end
            # frame; the tool returns from obdInit on it WITHOUT acking, so we
            # do not expect an ack after it.
            for k, frm in enumerate(ID_FRAMES):
                send_frame(frm)
                print(f"[sim]     {now():8.0f}  TX {frm.hex()}   (id[{k}])",
                      flush=True)
                if k == len(ID_FRAMES) - 1:
                    break               # end frame: no ack follows
                rack = recv_frame()
                if rack != TOOL_ACK:
                    print(f"[sim]     id[{k}] ack mismatch: {rack.hex()}")
                    return

            # obdRead cycles: serve Phase 1 (32 blocks) + Phase 2 (scan) as an
            # indefinite loop, so callers that read more than once (e.g. --immo's
            # two-read verify) each get a full, identical response.
            nblocks = 0x200 // 0x10
            cycle = 0
            while not stop.is_set():
                cycle += 1
                # Phase 1: 32 sixteen-byte block requests (SID 0x19).
                for _ in range(nblocks):
                    req = recv_frame()          # 06 01 19 10 AH AL 03
                    addr = (req[4] << 8) | req[5]
                    resp = bytes([0x13, 0x00, 0xEF]) + block_data(addr) + bytes([0x03])
                    send_frame(resp)
                    nblocks_done = _ + 1
                    if nblocks_done <= 2 or nblocks_done >= nblocks - 1 or cycle > 1:
                        pass  # (logging kept quiet on repeats)
                    else:
                        print(f"[sim]     {now():8.0f}  TX {resp.hex()}   "
                              f"(block 0x{addr:04x} [c{cycle} {nblocks_done}/"
                              f"{nblocks}])", flush=True)

                # Phase 2: mirror scan (SID 0x01). Reply with block 0's data so
                # findBlock() hits valid[0] & valid[1] and the tool breaks out.
                req = recv_frame()              # 06 01 01 10 AH AL 03
                addr = (req[4] << 8) | req[5]
                resp = bytes([0x13, 0x00, 0xFE]) + block_data(0) + bytes([0x03])
                send_frame(resp)
                if cycle == 1:
                    print(f"[sim]     {now():8.0f}  TX {resp.hex()}   "
                          f"(scan 0x{addr:04x})", flush=True)
        except queue.Empty:
            print(f"[sim]     responder: timed out waiting for a tool byte")
        except Exception as e:           # pragma: no cover
            print(f"[sim]     responder error: {e!r}")

    t_echo = threading.Thread(target=echo_loop, daemon=True)
    t_resp = threading.Thread(target=responder, daemon=True)
    t_echo.start()
    t_resp.start()

    env = dict(os.environ, ME7_DEVICE=slave_name)
    extra = os.environ.get("ME7_EXTRA", "").split()   # extra tool args, e.g. "--immo"
    cmd = [tool, "--OBD", "-r", "-p", "1", "-b", BAUD, OUT] + extra
    print(f"[harness]  running: {' '.join(cmd)}   (ME7_DEVICE={slave_name})", flush=True)
    # stdin pipes ME7_INPUT (default empty) so a confirmation prompt (e.g.
    # --immo's y/N) reads that rather than hanging on the terminal.
    proc = subprocess.run(cmd, env=env, text=True, capture_output=True, timeout=90,
                           input=os.environ.get("ME7_INPUT", ""))

    stop.set()
    # Close the slave FIRST. The echo thread is blocked in os.read(master); on
    # macOS closing the master while that read is outstanding deadlocks the
    # close (uninterruptible). Closing the slave EOFs the master read (EIO),
    # the echo loop breaks, and the subsequent master close is clean.
    try:
        os.close(slave)
    except OSError:
        pass
    t_echo.join(timeout=5)
    t_resp.join(timeout=5)
    try:
        os.close(master)
    except OSError:
        pass

    print("=== tool stdout ===")
    print(proc.stdout)
    if proc.stderr:
        print("=== tool stderr ===")
        print(proc.stderr)

    print(f"=== exit code: {proc.returncode} ===", flush=True)
    if proc.returncode != 0:
        print("FAIL: tool did not exit 0")
        return 1

    data = open(OUT, "rb").read()
    print(f"out file: {len(data)} bytes")
    ok = (data == b"".join(block_data(a) for a in range(0, 0x200, 0x10)))
    print("image == expected per-block pattern: " + ("MATCH" if ok else "MISMATCH"))
    if not ok:
        print("  expected first 32B:", b"".join(block_data(a) for a in (0, 0x10)).hex())
        print("  actual   first 32B:", data[:32].hex())
    print("RESULT:", "PASS" if (proc.returncode == 0 and ok) else "FAIL")
    return 0 if (proc.returncode == 0 and ok) else 1


if __name__ == "__main__":
    sys.exit(main())
