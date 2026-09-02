# me7eeprom

A cross-platform (Linux and macOS) C++17 tool that reads and writes the
serial EEPROM (95040 / 95080 / 95160 / 95P08) on Bosch ME7.x engine ECUs over
a K-line serial adapter. It runs on modern Unix hosts with no Windows
dependencies.

This project is a clean-room re-implementation **inspired by the [me7eeprom
tool by ArgDub](http://nefariousmotorsports.com/forum/index.php?topic=1168.0title=)**,
a Windows-only utility for the same job.

It also has a convenience mode that builds an **immobiliser-off ("immo-off")**
EEPROM image from an OBD read, and a standalone Python patcher/inspector.

> ## ⚠️ Disclaimer
> Modifying your ECU's EEPROM can **brick the ECU** and may affect the
> immobilizer / anti-theft system. You are responsible for complying with all
> applicable local laws. **Always back up the original EEPROM before writing.**
> Use this software at your own risk.

## Supported ECUs / hardware

- **Bosch ME7.x** ECUs, e.g. **ME7.1** with the Infineon SAK-C167CR CPU, a 95040
  512-byte SPI EEPROM, and chip-select on **P4.7**.
- A **K-line USB-serial adapter** (a "KKL" cable, e.g. an FTDI or CH340-based
  USB-KKL). On Linux/macOS the `-p` COM-number argument is kept only for CLI
  compatibility — you point the tool at the real device with the `ME7_DEVICE`
  environment variable (e.g. `ME7_DEVICE=/dev/cu.usbserial-XXXX`).

> A photo locating the 95040 EEPROM chip and pin 24 on the ECU board is in
> [docs/TDyO5.jpg](docs/TDyO5.jpg). Pin 24 is the flash-chip boot strap
> (P0L.4) grounded at power-up to enter boot mode — see
> [docs/bench-setup.md](docs/bench-setup.md) for the full procedure. (The
> chip-select pin selected by `--CSpin`, e.g. `P4.7`, is a separate CPU port
> pin, not pin 24.)

## Prerequisites

- **CMake ≥ 3.20** and a **C++17** compiler.
- The **default build needs no third-party libraries** — it uses the native
  POSIX termios serial backend and only the C++17 standard library.
- (Optional) **libserialport** — only required for a true 10400 baud custom
  divisor on real K-line hardware (the termios backend cannot express
  non-standard baud rates). Leave it off for OBD / standard-baud work or the
  pty validation harnesses.
- The Python scripts (`me7_patch.py` and the pty validation harnesses) need
  **Python 3** and its standard library only — no third-party packages.

Install the build tools per platform:

| OS | Command |
|---|---|
| macOS | `xcode-select --install` then `brew install cmake` |
| Debian/Ubuntu | `sudo apt install build-essential cmake` |

Add libserialport when you want the optional backend:

| OS | Command |
|---|---|
| macOS | `brew install libserialport` |
| Debian/Ubuntu | `sudo apt install libserialport-dev` |

## Build

The default POSIX termios build needs no extra dependencies:

```sh
cmake -B build && cmake --build build
```

For a **true 10400 baud** custom divisor on real K-line hardware, build with
the libserialport backend:

```sh
cmake -B build -DME7_USE_LIBSERIALPORT=ON
cmake --build build
```

Run the unit tests:

```sh
ctest --test-dir build
```

The built tool is `build/me7eeprom`.

## Usage

```sh
./build/me7eeprom --help
```

The tool has two modes:

### OBD mode (`--OBD`) — read-only over the OBD port

Reads the EEPROM using KWP over the OBD port. OBD mode is **read-only**.

```sh
ME7_DEVICE=/dev/cu.usbserial-XXXX ./build/me7eeprom --OBD -p1 -r dump.bin
```

> Note: over OBD the ECU shadows some EEPROM regions (returns `0xFF` for
> `0x30`–`0x4F` and `0x153`–`0x16F`). For a byte-faithful image of the raw
> silicon, use **boot mode** instead.

### Boot mode (`--bootmode <memtype>`) — read or write the raw EEPROM

Boot-straps the ECU's C166 bootstrap loader, uploads a monitor and EEPROM
driver, then reads or writes the raw EEPROM.

```sh
# read (do this first)
ME7_DEVICE=/dev/cu.usbserial-XXXX ./build/me7eeprom --bootmode 95040 -p1 -r dump.bin

# write
ME7_DEVICE=/dev/cu.usbserial-XXXX ./build/me7eeprom --bootmode 95040 -p1 -w image.bin
```

The `--CSpin Px.y` option selects the CPU chip-select pin (e.g. `--CSpin P4.7`).
Omit it to let the tool auto-detect.

## Immo-off workflow

`--immo` (with `--OBD -r`) reads the EEPROM **twice**, verifies the per-page
checksums, and — after a `y/N` prompt — writes a surgical immo-off image to
`dump.bin.immooff.bin`. The patch flips the immobiliser flag on pages 1 and 2
and recomputes only those two pages' checksums (**4 bytes changed**).

```sh
# 1. back up the original + build the immo-off image
ME7_DEVICE=/dev/cu.usbserial-XXXX ./build/me7eeprom --OBD -p1 --immo -r dump.bin

# 2. write the immo-off image back in boot mode
ME7_DEVICE=/dev/cu.usbserial-XXXX ./build/me7eeprom --bootmode 95040 -p1 -w dump.bin.immooff.bin
```

**Always back up the original read first.** The standalone patcher
(`re/scripts/me7_patch.py`) can inspect a dump and build the same image without
talking to the ECU.

## Notes

- Default baud **10400** (allowed: 9600, 10400, 19200, 57600).
- **Boot mode is required for writes.** OBD write is intentionally disabled, as
  in the original tool.
- A diagnostic raw-byte dump of the boot handshake is available with
  `ME7_DEBUG_BOOT=1`.
- A bench wiring/boot-strap guide is in `docs/bench-setup.md`; an ECM pinout
  reference is available
  [on the Nefarious Motorsports forum](http://nefariousmotorsports.com/forum/index.php?topic=5068.msg49080#msg49080).

## Repository layout

- `src/core/` — protocol, boot uploader, EEPROM read/write, immobiliser model,
  CLI options, serial engine, timing.
- `src/pal/` — serial-port backends (native POSIX termios; libserialport
  optional).
- `src/firmware/` — the bundled C166 boot-mode blobs, included as opaque data.
- `re/scripts/me7_patch.py` — standalone immo-off patcher / dump inspector.
- `tests/` — unit tests for the options parser and EEPROM logic.

## License

License: MIT.
