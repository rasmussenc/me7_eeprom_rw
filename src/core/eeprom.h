#pragma once

// ME7_EEPROM — boot-mode EEPROM logic layer (read / write / verify + memory map).
//
// C++17 port of the original ME7EEPROM v1.40 boot-mode memory drivers:
//   boot read   "Reading EEPROM ..."
//   boot write  "Writing EEPROM " (erase/enable + program + re-read verify)
//   erase/write-enable helper (raw K-line exchange)
//   post-process checksum verify helper
// plus the memory-size table.
//
// Only boot-mode K-line operation is handled here. The OBD read path belongs to
// the protocol layer and is not duplicated.
//
// Serial / protocol ownership:
//   - The high-level cmd/memory exchanges go through me7::Protocol
//     (callDriverCmd + memoryRead).
//   - The erase/write-enable step is a raw K-line exchange that the write
//     driver performs directly against the serial engine, so bootWriteEEPROM
//     additionally takes a me7::SerialEngine&.

#include "core/serial_engine.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace me7 {
class Protocol;  // defined in core/protocol.h against the documented API.

// ---------------------------------------------------------------------------
// Error / status codes (matching the original's values).
// ---------------------------------------------------------------------------
inline constexpr uint32_t kErrEepromVerify   = 0x70101;  // re-read != written (verify mismatch)
inline constexpr uint32_t kErrEepromReadChk  = 0x70501;  // read: driver status word[7] != 0
inline constexpr uint32_t kErrEepromWriteEn  = 0x70601;  // write: enable status word[7] != 0
inline constexpr uint32_t kErrEepromAlloc    = 0x70b05;  // write: buffer allocation failed
inline constexpr uint32_t kErrEepromEraseAck = 0x40701;  // erase: bad 0xAA after 0x84
inline constexpr uint32_t kErrEepromEraseEa  = 0x40801;  // erase: bad 0xEA
inline constexpr uint32_t kErrEepromProcAck  = 0x40b01;  // post: bad 0xAA after 0x33
inline constexpr uint32_t kErrEepromProcEa   = 0x40c01;  // post: bad 0xEA after 0x33
inline constexpr uint32_t kErrEepromProcXor  = 0x4100b;  // post: checksum mismatch

// ---------------------------------------------------------------------------
// Memory map.
// ---------------------------------------------------------------------------
// EEPROM capacity (bytes) keyed by memory-type index, matching the original
// tool's type/size tables:
//   { "95040", "95080", "95P08", "95160" } -> { 0x200, 0x400, 0x400, 0x800 }.
// Default 0x200 when no memory type is given / index out of range.
size_t eepromSize(size_t memIndex);

// ---------------------------------------------------------------------------
// Boot-mode drivers.
// ---------------------------------------------------------------------------
// Read the whole EEPROM into `out` (resized to `byteSize`).
// Reads in 0x200-byte blocks: callDriverCmd(0xF610, set-read-area) then
// memoryRead(0xFC00, block). Returns 0 on success or a status code.
//
// `memIndex` is the driver memory-type field (the quirked value: 95160's raw
// index 3 maps to driver memtype 1); `byteSize` is the total buffer size (so
// 95160 still reads 0x800 bytes while the driver sees memtype 1).
uint32_t bootReadEEPROM(Protocol& proto, size_t memIndex, size_t byteSize,
                        std::vector<uint8_t>& out);

// Erase/enable + program (per 16-byte page) + re-read verify. The erase step
// and its checksum post-process run directly on the serial engine (see header
// note). `memIndex`/`byteSize` split is the same as bootReadEEPROM. Returns 0
// on success, else a status code (e.g. kErrEepromVerify on mismatch).
uint32_t bootWriteEEPROM(Protocol& proto, SerialEngine& eng, size_t memIndex,
                         size_t byteSize, const std::vector<uint8_t>& data);

}  // namespace me7
