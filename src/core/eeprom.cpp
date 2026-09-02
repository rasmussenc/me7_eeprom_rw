// EEPROM logic layer — port of the ME7EEPROM boot-mode memory drivers. See
// eeprom.h for ownership notes and the memory map.
//
// Helpers in this file:
//   bootReadEEPROM   — read the whole EEPROM
//   bootWriteEEPROM  — erase/enable + program + re-read verify
//   eraseBlock       — erase/write-enable a 0x200-byte block (raw K-line)
//   postProcess      — checksum verify (raw K-line)
//   kSizeTable       — EEPROM capacity by memory-type index
//
// Serial primitives used by the helpers (per src/core/serial_engine.h):
//   writeByteEcho  — write one byte and consume its local echo
//   readN          — read N bytes
//   writeBufferEcho— write a buffer and consume its local echo

#include "core/eeprom.h"
#include "core/protocol.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace me7 {

// EEPROM capacity table:
//   index 0 (95040) = 0x200, index 1 (95080) = 0x400,
//   index 2 (95P08) = 0x400, index 3 (95160) = 0x800.
namespace {
constexpr size_t kSizeTable[4] = {0x200, 0x400, 0x400, 0x800};
}

size_t eepromSize(size_t memIndex) {
  if (memIndex >= 4) return 0x200;  // default (no memtype given / out of range)
  return kSizeTable[memIndex];
}

// ---------------------------------------------------------------------------
// post-process / checksum verify.
//   send '3' -> expect 0xAA : read 2 bytes [check, 0xEA]
//   XOR-fold the data block against the returned check byte; 0 means OK.
// ---------------------------------------------------------------------------
namespace {

uint32_t postProcess(SerialEngine& eng, const uint8_t* data, size_t len) {
  uint32_t r = eng.writeByteEcho(0x33);  // '3'
  if (r != 0) return r;

  uint8_t b0;
  r = eng.readN(&b0, 1);  // ack
  if (r != 0) return r;
  if (b0 != 0xaa) return 0x40b01;

  uint8_t pair[2];
  r = eng.readN(pair, 2);  // [check, terminator]
  if (r != 0) return r;
  if (pair[1] != 0xea) return 0x40c01;

  uint32_t acc = pair[0];  // uVar2 = echoed check byte
  size_t u3 = 0;
  if (len > 0) {
    // Leading unaligned bytes to reach 4-byte alignment (min(0 - %data & 3, len)).
    size_t u5 = static_cast<size_t>((-(intptr_t)data) & 3);
    if (len < u5) u5 = len;
    if (u5 != 0) {
      do {
        acc = static_cast<uint8_t>(acc) ^ data[u3];
        u3++;
      } while (u3 < u5);
      if (len == u5) goto done;
    }
    // Fold 32-bit words.
    size_t u4 = (len - u5) >> 2;
    if (u4 != 0) {
      size_t u6 = 0;
      uint32_t u7 = 0;
      do {
        uint32_t w;
        std::memcpy(&w, data + u5 + u6 * 4, 4);  // may be unaligned
        u7 ^= w;
        u6++;
      } while (u6 < u4);
      acc = acc ^ ((u7 >> 8) & 0xff) ^ u7 ^ (u7 >> 16) ^ (u7 >> 24);
      acc = static_cast<uint8_t>(acc);
      u3 += u4 * 4;
      if (len - u5 == u4 * 4) goto done;
    }
    // Remaining tail bytes.
    do {
      acc = static_cast<uint8_t>(acc) ^ data[u3];
      u3++;
    } while (u3 < len);
  }
done:
  if (static_cast<uint8_t>(acc) == 0) return 0;
  return 0x4100b;
}

// ---------------------------------------------------------------------------
// erase / write-enable a 0x200-byte block, then checksum-verify.
//   send 0x84 -> expect 0xAA : send addr(3 LE)+len(2 LE) header : expect 0xEA
//   -> postProcess(data, len)
// ---------------------------------------------------------------------------
uint32_t eraseBlock(SerialEngine& eng, uint32_t addr, const uint8_t* data,
                    size_t len) {
  uint32_t r = eng.writeByteEcho(0x84);  // erase/write-enable command
  if (r != 0) return r;

  uint8_t b;
  r = eng.readN(&b, 1);  // ack
  if (r != 0) return r;
  if (b != 0xaa) return 0x40701;

  uint8_t frame[5];
  frame[0] = static_cast<uint8_t>(addr);
  frame[1] = static_cast<uint8_t>(addr >> 8);
  frame[2] = static_cast<uint8_t>(addr >> 16);
  frame[3] = static_cast<uint8_t>(len);
  frame[4] = static_cast<uint8_t>(len >> 8);

  r = eng.writeBufferEcho(frame, 5);     // header {addr,len}
  if (r != 0) return r;
  r = eng.writeBufferEcho(data, len);    // the 0x200-byte payload
  if (r != 0) return r;
  // NOTE: the write sends the 5-byte header THEN the 0x200-byte data payload as
  // two separate writes — not the header twice. Sending the header twice never
  // transmits the block, so the monitor waits for data that never arrives, the
  // 0xEA terminator read times out (0x20F07), and every boot WRITE fails at
  // "Writing EEPROM" (read never exercises this path). Same two-distinct-buffers
  // class as the loader/monitor upload and the 19-byte callDriverCmd frame.

  r = eng.readN(&b, 1);  // terminator
  if (r != 0) return r;
  if (b != 0xea) return 0x40801;

  return postProcess(eng, data, len);  // checksum verify
}

}  // namespace

// ---------------------------------------------------------------------------
// boot read driver.
// ---------------------------------------------------------------------------
uint32_t bootReadEEPROM(Protocol& proto, size_t memIndex, size_t byteSize,
                        std::vector<uint8_t>& out) {
  const size_t size = byteSize;
  if (size < 1) return 0;  // nothing to read
  out.resize(size);

  uint32_t r;
  size_t off = 0;
  for (;;) {
    std::array<uint16_t, 8> params = {};
    params[0] = static_cast<uint16_t>(6);              // opcode: set read area
    params[1] = 0;
    params[2] = 0;
    params[3] = static_cast<uint16_t>(off);            // block offset
    params[4] = static_cast<uint16_t>(0x200);          // block length
    params[5] = static_cast<uint16_t>(0xfc00);         // address
    params[6] = static_cast<uint16_t>(3);
    params[7] = static_cast<uint16_t>(memIndex);       // memory type

    r = proto.callDriverCmd(0xf610, params, 300);
    if (r != 0) break;
    if (params[7] != 0) return 0x70501;  // driver status != 0

    std::vector<uint8_t> blk(0x200);
    r = proto.memoryRead(0xfc00, blk);  // read 0x200-byte block at 0xfc00
    if (r != 0) break;
    std::memcpy(out.data() + off, blk.data(), 0x200);

    off += 0x200;
    if (size <= off) return 0;  // all blocks read
  }
  return r;
}

// ---------------------------------------------------------------------------
// boot write driver (erase/enable + program + re-read verify).
// ---------------------------------------------------------------------------
uint32_t bootWriteEEPROM(Protocol& proto, SerialEngine& eng, size_t memIndex,
                         size_t byteSize, const std::vector<uint8_t>& data) {
  const size_t size = byteSize;
  uint32_t r = 0;

  if (size > 0) {
    size_t blkOff = 0;
    do {
      r = eraseBlock(eng, 0xfc00, data.data() + blkOff, 0x200);  // erase/write-enable
      if (r != 0) return r;

      uint32_t addr = 0xfc00;
      do {
        // Write status / enable.
        std::array<uint16_t, 8> params = {};
        params[0] = static_cast<uint16_t>(2);  // opcode: write status / enable
        r = proto.callDriverCmd(0xf610, params, 100);
        if (r != 0) return r;
        if (params[7] != 0) return 0x70601;  // enable status != 0

        // Program one 16-byte page.
        params = {};
        params[0] = static_cast<uint16_t>(7);            // opcode: program 16-byte page
        params[1] = 0;
        params[2] = 0;
        params[3] = static_cast<uint16_t>(addr - 0xfc00 + blkOff);  // page offset
        params[4] = static_cast<uint16_t>(0x10);        // page length
        params[5] = static_cast<uint16_t>(addr);        // address
        params[6] = static_cast<uint16_t>(3);
        params[7] = static_cast<uint16_t>(memIndex);    // memory type

        r = proto.callDriverCmd(0xf610, params, 300);
        // The original swallows errors on the program step (returns 0 success):
        if (r != 0) return 0;
        if (params[7] != 0) return 0;

        std::fputc('.', stdout);  // per-page progress dot
        addr += 0x10;
      } while (addr != 0xfe00);

      blkOff += 0x200;
    } while (blkOff < size);
  }

  std::puts(" OK");
  std::printf("Verifying EEPROM write ... ");

  // Re-read and compare (the write driver verifies inline, not via a separate call).
  std::vector<uint8_t> reread(size);
  r = bootReadEEPROM(proto, memIndex, byteSize, reread);
  if (r != 0) return r;

  if (std::memcmp(data.data(), reread.data(), size) != 0) return 0x70101;  // verify mismatch
  return 0;
}

}  // namespace me7
