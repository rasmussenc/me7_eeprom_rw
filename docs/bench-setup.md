# ME7 ECU Benchtop Setup — Boot Mode + EEPROM Read/Write

This guide describes how to wire and power a Bosch ME7 ECU on a bench and put
it into **boot mode** so `me7eeprom` can read/write the 95040 serial EEPROM
over the K-line. It is the physical/harness companion to the software in this
repo.

> **Two concepts — don't confuse them.** Getting the ECU on the bench and
> talking over K-line, and entering boot mode, are *separate* steps:
>
> | Step | What it does | Open the ECU? |
> |------|--------------|---------------|
> | **Bench harness** | supplies +12V, ground, and K-line to the ECU via its **connector** | **No** |
> | **Boot strap** | pulls the C166 boot pin (P0L.4) low at power-up so the on-chip bootstrap loader runs instead of the normal firmware | **Yes, for ME7.1 / 1.1** (ground flash pin 24 / PCB pad) |
>
> Connecting the ECU to the bench does not require opening it — the harness
> wires to the external connector. But to actually *enter boot mode* on an
> ME7.1 / ME7.1.1 you must open the case and ground the boot strap (flash chip
> pin 24, or the PCB test pad soldered to it) at power-up. The boot strap is
> not brought out to a connector pin on the ME7.1/1.1 harness.

> **"Pin 24" = pin 24 of the flash *chip*, not a connector pin.**
> Forum posts can read ambiguously because they mix connector-pin and
> chip-pin language. When someone writes "I grounded pin 24 then connected
> power to pin 3", the tell is *pin 3* — that's the **ECU connector's**
> switched-12V (terminal 15), while the *pin 24* in the same sentence is the
> **Am29F800BB flash chip's** pin 24 inside the case (different numbering
> space entirely). S4wiki states the mechanism plainly: *"Grounding pin 24 on
> the flash also grounds the P0L.4 pin (Port 0, bit 4) on the CPU. The CPU
> samples this pin on power up. If it sees low, it will go into boot mode."*

> **Why boot mode?** VW service departments add or match a key over the **OBD
> port**, using the normal KWP2000 diagnostic protocol and the SKC — no
> removal, no opening, no boot mode. Boot mode is a **factory/recovery back
> door**: it is how you reach a bricked ECU that no longer answers the normal
> protocol, or how you bypass the protocol layer to touch memory directly.
>
> Reading the 95040 EEPROM over OBD works fine with this tool (`--OBD -r`).
> Boot mode enters the workflow specifically for the **write-back** of an
> immo-off image, because the original tool intentionally disables OBD writes.
>
> | Task | Path | Open ECU? |
> |------|------|-----------|
> | Read EEPROM / get SKC | OBD `--OBD -r`, in-car | No |
> | VW matches a new key (service dept) | OBD KWP2000 + SKC | No |
> | **Write back an immo-off image** | boot mode (OBD write disabled) | **Yes** (strap) |

> **Read before you write.** A boot-mode *read* cannot brick the ECU. Do a read
> first, confirm the dump looks sane, and only then write back an immo-off
> image. Boot-mode writes that fail mid-transfer have bricked ECUs that then
> need bench recovery. Always keep a backup of the original read.

---

## 1. Parts

| Item | Notes |
|------|-------|
| **KKL (dumb K-line) cable** | e.g. the blue "409.1" FTDI USB-KKL. This is what `me7eeprom` uses. A Galletto 1260 in dumb mode or a HEX-CAN in VCP mode also works. |
| **Bench DC power supply** | Set to ~13.5V, capable of ≥1A. Do **not** run the ECU off a weak 12V wall wart or a car battery with dirty charging — flaky power causes read/write failures. |
| **Bench harness wires** | to reach the ECU connector pins (see pinout below). |
| **Boot-strap jumper** | a short wire with an alligator clip / probe on each end, to momentarily ground flash pin 24 (or the PCB pad) at power-up. |
| **Ground reference** | the jumper grounds to the ECU's own ground (any GND pin / the case metalwork). |
| **ECU connector breakout** | optional but strongly recommended: a 121-pin (or 80-pin) breakout / "dummy plug" so you can clip onto individual pins. |

---

## 2. ECU connector pinout (the harness — no opening needed)

> **Fill in your ECU part number** (§6) to confirm which of these two pinouts
> applies. The connector pinout lists **power, ground, K-line — and no boot
> pin**, which is why the boot strap must be reached inside the case on ME7.1/1.1.

### 121-pin ECU — ME7.1, ME7.1.1, ME7.5 (VAG)

| Signal | Connector pin(s) | Wire to |
|--------|------------------|---------|
| **GND** | 1 | PSU ground (-) |
| **VCC (+12V)** | 3, 21, 62 | PSU +12V (switched + battery) — wire all three to +12V |
| **K-line** | 43 | KKL cable K-line |
| CAN-H | 60 | not used by this tool |
| CAN-L | 58 | not used by this tool |

### 80-pin ECU — ME7.5B, ME7.5.10 (VAG)

| Signal | Connector pin(s) | Wire to |
|--------|------------------|---------|
| **GND** | 2 | PSU ground (-) |
| **VCC (+12V)** | 15, 27 | PSU +12V — wire both |
| **K(W)-line** | 29 | KKL cable K-line |
| CAN-L | 31 or 17 | not used |
| CAN-H | 32 or 18 | not used |

Terminals: model **battery (terminal 30)** and **ignition (terminal 15)** separately
if you want the real power-up sequence — but for bench EEPROM work it's fine to
just switch all +12V pins together (they're all "ECU powered"). The boot strap
needs power to be **applied** while pin 24 is held low, so a single switch on
the +12V line is what matters.

---

## 3. Benchtop wiring diagram

```
                      13.5V DC BENCH PSU
                      +-----------------+
                      |  + (red)     - (blk) |
                      +----|----------|-----+
                           |          |
                           |          |
                  +12V (T30/T15)      GND
                           |          |
        +------------------|----------|--------------------+
        |                  |          |                    |
        |        +12V pins (3,21,62)  GND pin(s)           |
        |        |             |         |                 |
        |   +----+-------------+    +-----+                 |
        |   |                          |                    |
        |  [ 121-PIN ECU CONNECTOR ]   |                    |
        |   |  pin43 = K-line          |                    |
        |   |   |                      |                    |
        |   |   +---- K-line ----------+                    |
        |   |                             |                 |
        |   +-------- GND ----------------+                 |
        |                                                  |
        |        BOSCH ME7 ECU (case illustrated)          |
        |   +-----------------------------------------+     |
        |   |                                         |     |
        |   |   (open the lid to reach the boot strap) |     |
        |   |      ........ Am29F800BB flash ........  |     |
        |   |      :  . . . . . . . . . . . . . .  :  |     |
        |   |      :  . . . . . . . . . . . . . .  :  |     |
        |   |      :  [24] . . . . . . . . . . . .  :  |<-- boot strap
        |   |      :  . . . . . . . . . . . . . .  :  |     (pin 24,
        |   |      '''''''''''''''''''''''''''''''''  |      2nd from
        |   |       ^dot = pin 1 (top-right)          |      left, bottom
        |   +-----------------------------------------+     row)
        |                                                  |
        +----------------------|---------------------------+
                               |
                          GND / case metalwork
                               ^
                               |
                     boot-strap jumper (clip to pin 24
                     or the PCB solder pad beside it),
                     held to GND only at power-up, ~2-3 s
                     then REMOVED before the tool reads.

          KKL cable
          +---------------------------------+
          | USB ----- laptop (me7eeprom)    |
          | K-line --> ECU connector pin 43 |
          | GND ..... ECU GND (common)      |
          +---------------------------------+
```

---

## 4. Boot strap location (inside the ECU)

For **ME7.1 (C167CR CPU)**, the boot strap is **P0L.4 of the C166**, which is
wired to **pin 24 of the Am29F800BB 44-pin PSOP flash chip**. Grounding it at
power-up makes the C166 run its on-chip bootstrap loader (which is what answers
`me7eeprom`'s `0x00` byte with the uC-ID).

> **This mapping is documented for ME7.1 / C167 and is ECU-family-dependent.**
> A forum user with a **24V VR6 ME7.1.1 (ST10-class CPU, p/n 022 906 032 CS)**
> grounded flash pin 24 exactly as the write-up says and still got
> *"BOOT MODE INACTIVE"* — the ST10 variants (uC-ID 0xD5 / C167CS) appear to
> use a different strap node. Worse, the stock ME7EEPROM v1.40 does **not**
> support ST10 at all (the author says so; it fails at the driver stage), so
> for those ECUs the flash-pin-24 strap is moot. **Confirm your part number
> (§8) before spending effort on the strap** — if it's ST10, this tool won't
> talk to it regardless of how perfectly you strap.

Chip orientation (as you'll read it on the NefMoto/S4wiki write-ups):

```
        Am29F800BB  (44-pin PSOP flash)  -- look for the dot
        pin1 dot in the TOP-RIGHT, ECU connector plugs facing RIGHT.

        top row    . . . . . . . . . . . . .  .   (pin 1 is here, far right)
        bottom row . . . . . . . . . . . . .  .
                          ^^
                          pin 25 ... pin 24 is the 2nd pin FROM THE LEFT
                          on the bottom row. (count from the left edge
                          of the bottom row: pin 24 = 2nd from left.)

            [24]
             |
            GND  (only during power-up; release before programming)
```

- Some builders skip the chip leg and instead ground the **PCB test pad /
  solder point** that pin 24 traces to (the NefMoto thread mentions the
  "solder point right next to it") — easier to clip onto and less risk of
  shorting adjacent legs.
- **Do NOT leave pin 24 grounded while the tool reads/writes.** It must be
  grounded *only* during the power-up moment; grounding it during the
  EEPROM operation causes the read/write to fail. Hold ~2–3 seconds after
  applying power, then release.

---

## 5. Step-by-step procedure

1. **Wire the bench harness** (connector only — ECU lid still on or off
   doesn't matter for this step):
   - PSU **+13.5V** → ECU +12V pins (121-pin: 3, 21, 62 — wire all three).
   - PSU **GND** → ECU GND pin(s) (121-pin: pin 1) and the KKL cable's GND.
   - KKL cable **K-line** → ECU K-line pin (121-pin: pin 43).
   - Leave the **PSU off** for now.
2. **Open the ECU lid** (4 screws / clips) to expose the PCB and the flash
   chip. (The boot strap is not on the connector for ME7.1/1.1; see §4 for
   the strap location and §8 to confirm this is the right path for your ECU.)
3. **Attach the boot-strap jumper** to flash **pin 24** (or the PCB pad beside
   it) on one end, and to **ECU ground** (a GND pin / case metalwork) on the
   other — **but do not yet leave it connected if you can't hold it**; the
   idea is it's grounded *at* power-up and released ~2–3 s after. A practical
   method: clip one end to GND, hold the probe tip to pin 24, apply power,
   count "2... 3", lift the probe off pin 24.
4. **With pin 24 grounded, turn the PSU on** (apply +12V). The C166 samples
   P0L.4 at reset, sees low, and enters the bootstrap loader.
5. **Hold ~2–3 seconds, then remove the ground** from pin 24. You are now in
   boot mode. (Leaving it grounded = the next step fails.)
6. **Run the tool within a few seconds.** From another terminal:
   ```sh
   ME7_DEVICE=/dev/cu.usbserial-AG0KI7TE build/me7eeprom \
       --bootmode 95040 --CSpin P4.7 -r -p 2 boot_read.bin
   ```
   - `--bootmode 95040`  — memory type string (adjust to your chip; see §8)
   - `--CSpin P4.7`      — chip-select pin (omit it to let the tool auto-search)
   - `-r`                — **read** (safe; do this first)
   - `-p 2`              — COM port number (ignored when ME7_DEVICE is set)
   - `boot_read.bin`     — output file (your backup)
7. **Back up `boot_read.bin` somewhere safe** before doing anything else.

Only **after** a clean read, repeat for a write with an immo-off image:
```sh
ME7_DEVICE=/dev/cu.usbserial-AG0KI7TE build/me7eeprom \
    --bootmode 95040 --CSpin P4.7 -w -p 2 immo_off.bin
```

---

## 6. Power-up sequence timing (the part people get wrong)

```
time -->
                    t0            t0+2s        t0+3s        t0+5s
                     |              |            |            |
PSU +12V  -----------+              |            |            |
                     ON             |            |            |
pin24 -> GND  -------================+ OFF      |            |
                     [ground holds through power-up, ~2-3 s, then release]
                                                       |
                                          run me7eeprom (read/write) ---->
```

- Ground **first** (or at the same instant as) applying power; the C166
  samples the strap at reset. Grounding *after* power-up may not take.
- Release the strap **before** the tool reads/writes.
- Between attempts: **power the ECU fully off and wait for the capacitors
  to discharge** (a few seconds) before re-strapping. Repeated attempts on
  a stale powered state fail reliably — this comes up often on the forum.

---

## 7. What success and each failure look like

With the live (per-stage) output now in the port, you'll see exactly which
stage you reached:

```
SUCCESS (ME7.1, uC = C167CR, ID 0xC5):
  Opening COM2 ... OK
  Starting Boot_mode ... uC ID response 0xC5: C167CR ... OK
  Sending Loader + MonitorCore ... MonitorCore successfully launched
  Initializing registers ... OK
  Sending EEPROM driver ... OK
  Configuring SPI Interface ... OK        (or "Searching Chip_Select pin ... P4.7")
  Checking EEPROM Status Register ... 0x00F0
  Reading EEPROM ... OK
  File saved

"FAIL ... No ECU response" (error=0x20F07):
  -> The 0x00 was sent (cable OK) but no uC-ID byte came back.
  -> The C166 bootstrap loader is NOT running = boot strap not grounded at
     power-up, or ECU has no power, or wrong baud. Fix the strap sequence
     first; only try -b 9600 if you're sure the strap is correct.

fails later, e.g. at "Sending Loader + MonitorCore" / "Initializing registers":
  -> The strap worked (you got a uC ID) but comms are flaky. Power-cycle,
     discharge caps, retry. Different baud (9600) sometimes helps.

write: "Verifying EEPROM write ... FAIL (error=0x70101)":
  -> commonly a FALSE alarm on this tool; the write often actually succeeded.
     Re-read the EEPROM and compare to confirm rather than re-writing blindly.
```

### `--CSpin`: specify or auto-search?
- If you know the chip-select pin (often **P4.7** for a 95040), pass
  `--CSpin P4.7` to skip the auto-search.
- If you're unsure, **omit** `--CSpin` and the tool prints
  `Searching Chip_Select pin ... Px.y` once it detects it. Use the found
  value for subsequent reads/writes.

---

## 8. Confirmed ECU reference: 022 906 032 F (Bosch ME7.1, C167CR)

This row was validated on a **2001 VW Eurovan (T4), 2.8L VR6** ECU, read and
written successfully with this tool.
  - VAG part number: **`022 906 032 F`** → Bosch ME7.1
  - Bosch hardware number: **`0 261 206 736`** → ME7.1 hardware family
    (SAK-C167CR-4RM)

`022 906 032 F` is a **Bosch ME7.1**, the Bosch `0 261 206 736` hardware is the
ME7.1 family (Infineon SAK-C167CR-4RM CPU → uC-ID **0xC5 (C167CR)**). ME7.1 /
C167CR is the family this tool **supports** (the ST10/0xD5 incompatibility is
ME7.1.1 — see below). The flash-pin-24 = P0L.4 strap is documented for this
ME7.1 family, so the strap location is reliable.

| Field | **022906032F / 0261206736 / ME7.1 / C167CR** |
|-------|---------------------------------------------------------|
| ECU family | Bosch **ME7.1** (not 1.1) |
| CPU | Infineon SAK-C167CR-4RM |
| expected uC ID | **0xC5** C167CR → `"uC ID response 0xC5: C167CR ... OK"` |
| supported by v1.40? | **Yes** |
| connector +12V pins | 3, 21, 62 (wire all three) |
| connector GND | 1 |
| K-line pin | 43 |
| boot strap | open case, **flash pin 24** (P0L.4) — reliable for this family |
| EEPROM chip | **95040** (512 bytes) → `--bootmode 95040` |
| `--CSpin` | **P4.7** (or omit to auto-search) |
| Read command | `ME7_DEVICE=/dev/cu.usbserial-... build/me7eeprom --bootmode 95040 --CSpin P4.7 -r -p 2 boot_read.bin` |

**Both write-back routes are viable for this ECU:**
- **Boot mode** (this tool, K-line + strap).
- **Direct SPI clip** (TL866/T48 or CH341A+flashrom → SOIC8 clip on the 95040)
  — CPU-independent fallback; see the "Direct SPI clip access" section below.

Generic reference (other ME7 families, for completeness):

| Field | ME7.1 (1.8T/2.7T, C167CR) | ME7.1.1 (C167 or ST10) | ME7.5 (VAG, 121) | ME7.5B/5.10 (80-pin) |
|-------|---------------------------|------------------------|------------------|----------------------|
| connector +12V pins | 3,21,62 | 3,21,62 | 3,21,62 | 15,27 |
| connector GND | 1 | 1 | 1 | 2 |
| K-line pin | 43 | 43 | 43 | 29 |
| boot strap | open case, flash pin 24 | open case, flash pin 24 * | open case, flash pin 24 | open case, flash pin 24 |
| expected uC ID | **0xC5** C167CR | 0xC5 C167 **or** 0xD5 C167CS/ST10 | 0xC5 C167CR | 0xC5 C167CR |
| supported by v1.40? | **Yes** | C167 yes / **ST10 No** | Yes | Yes |
| EEPROM chip | 95040 (512B) | 95040 / 95160 | 95040 / 95160 / 95P08 | varies |
| `--CSpin` typical | P4.7 | P4.7 (or P6.3 for 95160) | P4.7 | varies / auto-search |

\* **ME7.1.1 ST10-class variants** return uC-ID **0xD5 (C167CS)** and then
read IDCHIP `0x0C43`; the documented flash-pin-24 strap did **not** put a
24V VR6 ST10 ECU into boot mode in a documented forum attempt. The stock
ME7EEPROM v1.40 does **not** support ST10 at all; this port matches the
original's behavior.

> To fill in the row for your ECU, note (1) the exact part number on the case
> label and (2) the engine code from the sticker/cowl, then confirm the
> memtype, chip-select pin, expected uC-ID, and whether boot mode is the right
> path for it.

---

## 9. Safety / bricking notes

- **Read first, always.** A read is non-destructive and gives you a recovery
  image.
- Keep `boot_read.bin` backed up in more than one place.
- Boot-mode writes can brick the ECU if interrupted. Have a bench recovery
  plan (the same boot-mode path can often re-write a bricked ECU, which is
  exactly what boot mode is for — but only if the bootstrap loader still
  answers).
- If a write fails, **do not power-cycle in a panic** until you've confirmed
  whether the verify-fail was a false alarm (re-read first).
- Never leave flash pin 24 grounded during the actual read/write.

---

## 10. Direct SPI clip access (alternate write path)

Boot mode is the K-line path. There is a second, more CPU-independent route:
**read/write the 95xxx SPI EEPROM directly with a programmer + SOIC-8 test
clip**, bypassing the C166, the K-line, and boot mode entirely. Because you
talk to the memory chip rather than the CPU, this works regardless of the ECU's
CPU family (C167 vs ST10) — though on your `022906032F` (C167CR) boot mode is
also viable, so this is a *fallback / alternative*, not a requirement.

> **The 95xxx is a SPI serial EEPROM, not 25-series NOR flash.** Use a
> programmer whose device database knows the ST/Microchip 95xxx family
> (M95040 / ST95040 etc.). "SPI flasher" clips built only for 25-series flash
> (W25/Q25 etc.) won't have the right EEPROM instruction/addressing.

### Hardware options
- **XGecu T48 / T56 (or TL866II Plus)** universal programmer — 95xxx EEPROMs in
  its device list; supports in-circuit clamping via ICSP lead + SOIC-8 test
  clip. "Select the chip, click read." ~$50–70 + a SOIC-8 test clip.
- **CH341A + flashrom** — the ~$5 CH341A board talks SPI over USB; `flashrom`
  supports ST M95xxx-family EEPROMs. Cheaper but more hands-on, and the CH341A
  supplies more VCC current than ideal for in-circuit (see back-power caveat).

### In-circuit rules (the part that bites)
The EEPROM is still soldered onto the ECU's SPI bus, so:

- **ECU must be UNPOWERED.** The C166 is the SPI master when the ECU is on; if
  it drives MOSI/SCK/CS while your programmer also drives them, you get a
  corrupted read and can stress drivers. With the ECU's 12V off, the CPU's SPI
  pins go high-Z and the programmer owns the bus. **Never clamp-read while the
  ECU is powered.**
- **Back-powering.** The programmer's VCC can leak back through the rail and
  partially wake adjacent 5V silicon. Usually fine at the ~2 mA an EEPROM
  draws (the TL866/T48 ICSP path current-limits VCC); a hot CH341A can backfeed
  more. Symptom: reads come back all `0xFF` or are intermittent → suspect
  back-power; reduce VCC or use the ICSP current-limited path.
- **WP (write-protect) pin.** Writing needs `WP#/STATUS` high during the
  write; in-circuit it may be tied. If a write fails/verifies as unwritten,
  check this pin.
- **Clip seating.** SOIC-8 clips are finicky — a slightly-off clip shorts
  adjacent legs or misses a contact, giving garbage (all `0xFF` or partial).
  **Verify by reading twice and comparing**; if they don't match byte-for-byte,
  reseat the clip. The bulletproof fallback is to **desolder** the SOIC-8,
  read/write it in a programmer socket, and resolder — zero bus contention at
  the cost of more invasive soldering.

### Where it plugs into our flow
Our immo-off image is **transport-agnostic** — it's just the 512 bytes of the
95040. You already have the *content* (your in-car `--OBD -r` gave a real
immo-3 dump; our `--immo` path builds the patched `.bin`). So the clip is only
needed for the **write-back** of the immo-off image:

```
OBD read (done, in-car)  ->  me7eeprom --immo  ->  immo_off.bin
   ->  open ECU, clip the 95xxx SPI EEPROM
   ->  write immo_off.bin with the programmer
   ->  re-read to verify  ->  done
```

No strap timing, no CPU handshake, no K-line, no bricking-from-mid-write risk.
The only downside vs boot mode is the clip/programmer hardware and its
in-circuit caveats above.

---

## Sources

- S4wiki, *Boot mode* — chip pin 24 = P0L.4 strap, power-up procedure, and the
  clarification that grounding flash pin 24 grounds P0L.4 on the CPU.
  https://s4wiki.com/wiki/Boot_mode
- Nefmoto wiki, *ECU Boot Mode* — open casing, ground pin 24 of 800BB at power-up.
  http://www.nefariousmotorsports.com/wiki/index.php/ECU_Boot_Mode
- NefMoto forum, *ME7EEPROM* thread (topic 1168) — tool usage, baud,
  error codes, write-verify false alarms, and the author's statement that OBD
  writes were deliberately disabled / immo-off over OBD won't be allowed.
- NefMoto forum, topic 5068 — the "grounded pin 24 then connected power to pin 3"
  post that mixes connector-pin (pin 3 = connector +12V) and chip-pin (pin 24 =
  flash chip) language in one sentence.
- NefMoto forum, topic 115 — 24V VR6 ME7.1.1 (022906032CS) where grounding
  flash pin 24 gave "BOOT MODE INACTIVE"; evidence the strap mapping is
  ECU-family-dependent (C167 vs ST10).
- NefMoto forum, *ME7EEPROM* author — ST10 is not supported by the tool.
- S4wiki, *Bosch ME7.1* — CPU (SAK-C167CR-4RM), Am29F800BB flash.
- transpondery.com VAG ME7xx pinout — connector +12V / GND / K-line pins.
