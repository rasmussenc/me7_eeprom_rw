#!/usr/bin/env python3
"""ME7 immo-3 EEPROM patcher / inspector.

Reads an ME7 immo-3 EEPROM dump (.bin), reports the immobiliser state, SKC,
VIN, soft-coding and per-page checksums, and can build a surgical
immobiliser-off (or on) image ready to write back to the ECU.

EEPROM layout (512-byte ME7 immo-3 ECU dump, 32 pages x 16 bytes):
  page 0x00          : header  (no checksum)
  page 0x01, 0x02    : immobiliser flag at byte[2]:  0x01 = ON, 0x02 = OFF
                       (both pages must agree)
  page 0x03, 0x04    : SKC / cluster code / softcoding
  page 0x07          : softcoding, flash counters region
  page 0x0B, 0x0D    : VIN (17 ASCII chars)
  page 0x08/0x0A/0x0C/0x0E/0x10/0x1F : redundant backup copies of the
                       preceding page (kept byte-identical by the ECU)
  pages 0x11..0x14   : no checksum (erased / wear region)
  bytes 0x0E, 0x0F of each checksummed page : low / high byte of
       calc = 0xFFFF - (page - minus) - sum(bytes[0..13])
       where minus = 2 for backup pages, else 1

Write-back workflow:
  1. read the ECU :   build/me7eeprom --OBD -r -p 1 -b 9600 dump.bin
  2. patch immo    :   python3 re/scripts/me7_patch.py --immo off dump.bin -o patched.bin
  3. write back   :   build/me7eeprom --bootmode <memtype> -w patched.bin

Safety: the tool is lossless by default -- it backs up any output it overwrites,
refuses an inconsistent immo state without --force, and (by default) only ever
touches the immobiliser pages and their checksums, leaving every other byte
exactly as the ECU wrote it.
"""

import argparse
import os
import sys

PAGE = 16
NO_CHECKSUM = {0x00, 0x11, 0x12, 0x13, 0x14}
BACKUP = {0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x1F}
IMMO_ON = 0x01
IMMO_OFF = 0x02
IMMO_PAGES = (0x01, 0x02)
IMMO_BYTE = 2


def load(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) == 0:
        sys.exit("error: empty file")
    if len(data) % PAGE != 0:
        sys.exit(f"error: size {len(data)} not a multiple of {PAGE}")
    return data


def pages(data):
    return [bytearray(data[i:i + PAGE]) for i in range(0, len(data), PAGE)]


def flatten(pg):
    return bytes(b for p in pg for b in p)


# --- immo version + field decoding -------------------------------------------

def immo_version(pg):
    # immo3: first 5 bytes of page 0 are identical; immo2 otherwise.
    return 2 if any(pg[0][i] != pg[0][0] for i in range(1, 5)) else 3


def decode_vin(pg):
    s = bytearray()
    for i in range(0x05, 0x0A):
        s.append(pg[0x0B][i])
    for i in range(0x00, 0x0C):
        s.append(pg[0x0D][i])
    try:
        return s.decode("ascii")
    except UnicodeDecodeError:
        return s.hex()


def decode_skc(pg):
    raw = (pg[0x03][0x03] << 8) | pg[0x03][0x02]
    if raw == 0xFFFF:
        return "none (uncoded)"
    return "0%04d" % raw


def decode_softcoding(pg):
    raw = (pg[0x07][0x0B] << 8) | pg[0x07][0x0A]
    return "%04d" % raw


def decode_immo_id(pg):
    s = bytearray([pg[0x0D][0x0C]])
    s.extend(pg[0x0F][0x00:0x0D])
    try:
        return s.decode("ascii")
    except UnicodeDecodeError:
        return s.hex()


# --- checksum ----------------------------------------------------------------

def page_checksum(pg, pageno):
    """Compute the stored checksum bytes for a page. Does NOT writeback."""
    minus = 2 if pageno in BACKUP else 1
    bytesum = sum(pg[pageno][0:14])
    calc = (0xFFFF - (pageno - minus) - bytesum) & 0xFFFF
    return calc


def checksum_ok(pg, pageno):
    if pageno in NO_CHECKSUM:
        return None  # not checksummed
    calc = page_checksum(pg, pageno)
    saved = (pg[pageno][0x0F] << 8) | pg[pageno][0x0E]
    return calc == saved


def write_checksum(pg, pageno):
    calc = page_checksum(pg, pageno)
    pg[pageno][0x0E] = calc & 0xFF
    pg[pageno][0x0F] = (calc >> 8) & 0xFF


def validate_all(pg):
    bad = []
    for p in range(len(pg)):
        ok = checksum_ok(pg, p)
        if ok is False:
            bad.append(p)
    return bad


# --- report -------------------------------------------------------------------

def report(pg, label, out=sys.stdout):
    out.write(f"=== {label} ===\n")
    i1, i2 = pg[0x01][IMMO_BYTE], pg[0x02][IMMO_BYTE]
    state = {IMMO_ON: "ON", IMMO_OFF: "OFF"}.get(i1, f"UNKNOWN(0x{i1:02x})")
    out.write(f"  immo version : {immo_version(pg)}\n")
    out.write(f"  immobiliser  : {state}  (page1[2]=0x{i1:02x}, page2[2]=0x{i2:02x}")
    out.write(")\n" if i1 == i2 else ")  ** MISMATCH **\n")
    out.write(f"  SKC          : {decode_skc(pg)}\n")
    out.write(f"  softcoding   : {decode_softcoding(pg)}\n")
    out.write(f"  VIN          : {decode_vin(pg)!r}\n")
    out.write(f"  immo ID      : {decode_immo_id(pg)!r}\n")
    bad = validate_all(pg)
    if not bad:
        out.write("  checksums    : all valid\n")
    else:
        out.write("  checksums    : INVALID on pages " +
                  ", ".join(f"0x{p:02x}" for p in bad) + "\n")
        out.write("                 (pages 0x03/0x04 are often 'errors' on ECUs\n"
                  "                 with uncoded SKC -- erased pages the tool\n"
                  "                 expects to be structured. Left untouched by\n"
                  "                 the surgical patch.)\n")


# --- operations ---------------------------------------------------------------

def op_immo(pg, setting):
    """Surgically flip the immo flag and recompute ONLY the affected pages'
    checksums. Pages 1 and 2 are not backup pages (minus=1), so this is a
    4-byte change at most (2 flag bytes + 2 checksum low-bytes)."""
    target = IMMO_ON if setting else IMMO_OFF
    flag_name = "ON" if setting else "OFF"
    i1, i2 = pg[0x01][IMMO_BYTE], pg[0x02][IMMO_BYTE]
    if i1 != i2:
        sys.exit(f"error: immo flag mismatch (page1=0x{i1:02x}, page2=0x{i2:02x})\n"
                 f"       the two immo pages disagree; refusing without --force.")
    if {i1, i2} == {target}:
        print(f"immo already {flag_name}; nothing to change.")
        return []
    # base sanity: the immo pages must have been self-consistent to edit safely
    for p in IMMO_PAGES:
        if checksum_ok(pg, p) is False:
            print(f"warning: page 0x{p:02x} checksum was invalid before edit.",
                  file=sys.stderr)
    deltas = []
    for p in IMMO_PAGES:
        old_flag = pg[p][IMMO_BYTE]
        pg[p][IMMO_BYTE] = target
        deltas.append((p * PAGE + IMMO_BYTE, old_flag, target))
        # recompute this page's checksum
        old_lo, old_hi = pg[p][0x0E], pg[p][0x0F]
        write_checksum(pg, p)
        if pg[p][0x0E] != old_lo:
            deltas.append((p * PAGE + 0x0E, old_lo, pg[p][0x0E]))
        if pg[p][0x0F] != old_hi:
            deltas.append((p * PAGE + 0x0F, old_hi, pg[p][0x0F]))
    return deltas


def op_fix_all(pg):
    """Full fixChecksum pass: mirror backup pages and recompute every
    non-skipped page's checksum. Opt-in via --fix-checksums."""
    deltas = []
    for p in BACKUP:
        src = pg[p - 1]
        for b in range(PAGE):
            if pg[p][b] != src[b]:
                deltas.append((p * PAGE + b, pg[p][b], src[b]))
                pg[p][b] = src[b]
    for p in range(len(pg)):
        if p in NO_CHECKSUM:
            continue
        old_lo, old_hi = pg[p][0x0E], pg[p][0x0F]
        write_checksum(pg, p)
        if pg[p][0x0E] != old_lo:
            deltas.append((p * PAGE + 0x0E, old_lo, pg[p][0x0E]))
        if pg[p][0x0F] != old_hi:
            deltas.append((p * PAGE + 0x0F, old_hi, pg[p][0x0F]))
    return deltas


# --- main ---------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="EEPROM dump (.bin)")
    ap.add_argument("-o", "--output", help="write patched image to this path")
    ap.add_argument("--immo", choices=["on", "off"],
                    help="set immobiliser on/off (surgical flag flip)")
    ap.add_argument("--fix-checksums", action="store_true",
                    help="also run the full fixChecksum pass "
                         "(mirror backup pages + recompute all checksums). "
                         "Changes more bytes; opt-in.")
    ap.add_argument("--force", action="store_true",
                    help="proceed despite warnings / overwrite existing output")
    args = ap.parse_args()

    data = load(args.input)
    pg = pages(data)

    report(pg, f"input: {args.input}")
    if not args.immo and not args.fix_checksums:
        print("\n(no --immo / --fix-checksums given; report only, no changes)")
        return 0

    deltas = []
    if args.immo:
        deltas += op_immo(pg, args.immo == "on")
    if args.fix_checksums:
        deltas += op_fix_all(pg)

    # verify the immo pages are now self-consistent after editing
    if args.immo:
        for p in IMMO_PAGES:
            if checksum_ok(pg, p) is False:
                sys.exit("internal error: page 0x%02x checksum invalid after edit" % p)

    print("\n--- changes ---")
    if not deltas:
        print("no bytes changed.")
    else:
        for off, old, new in sorted(deltas):
            p = off // PAGE
            print(f"  0x{off:04x} (page 0x{p:02x}): 0x{old:02x} -> 0x{new:02x}")
        print(f"  total: {len(deltas)} byte(s) changed.")

    report(pg, "after patch")

    if not args.output:
        print("\nno -o given; not writing a file.")
        return 0

    if os.path.exists(args.output) and not args.force:
        sys.exit(f"\nrefusing to overwrite {args.output} (pass --force)")

    # Always back up an existing output before clobbering it.
    bak = args.output + ".bak"
    if os.path.exists(args.output):
        os.replace(args.output, bak)
        print(f"moved existing {args.output} to {bak}")
    elif not os.path.exists(bak):
        # keep a copy of the pristine source next to the output too
        with open(bak, "wb") as f:
            f.write(data)
        print(f"backup of original written to {bak}")

    out = flatten(pg)
    with open(args.output, "wb") as f:
        f.write(out)
    print(f"\npatched image written to {args.output} ({len(out)} bytes)")
    print("write back with: build/me7eeprom --bootmode <memtype> -w "
          f"{os.path.basename(args.output)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
