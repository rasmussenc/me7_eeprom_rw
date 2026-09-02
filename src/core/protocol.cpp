// Protocol layer implementation. Each function's wire-critical framing is
// documented inline; behavior matches the original ME7EEPROM v1.40 tool.

#include "core/protocol.h"

#include <cstring>

namespace me7 {

void Protocol::pDelay(double ms) const {
  // Frame-spacing gate: sleep in ~1 ms slices until `ms` has elapsed (mirrors
  // the original's clock()-based spin-wait, which Sleep(1)-polls a deadline).
  const double deadline = nowMs() + ms;
  while (nowMs() < deadline) {
    sleepMs(1);
  }
}

// ---------------------------------------------------------------------------
// call_driver_cmd
// ---------------------------------------------------------------------------
//
// Wire exchange:
//   write 0x9F (+ local echo) -> read 1 byte -> must be 0xAA, else 0x40301
//   build frame: cmdWord (3 LE bytes) + params[0..7] (16 LE bytes) = 19 bytes
//   writeBufferEcho(frame, 19)   // ALL 8 words, incl. params[7]
//   retries = retries ? retries : 200
//   read 17-byte response -> byte[16] must be 0xEA, else 0x40401
//   unpack response words 0..7 (LE) back into params[]
//
// NOTE: the request frame is 19 bytes — cmd(3) + params[0..7](16). The last
// word, params[7], is part of the frame and MUST be transmitted. Sending only
// 17 bytes (dropping params[7]) leaves the monitor 2 bytes short of a full
// frame, so it never executes the command and never responds — surfacing as a
// readNTimeout (0x21007) at the first callDriverCmd ("Sending EEPROM driver").
// The 17-byte response READ and the 8-word unpack are correct as-is.
uint32_t Protocol::callDriverCmd(uint16_t cmdWord,
                                 std::array<std::uint16_t, 8>& params, int retries) {
  uint8_t frame[19];
  uint32_t r = eng_.writeByteEcho(0x9F);      // wake byte (+ echo)
  if (r != kErrNone) return r;
  r = eng_.readN(frame, 1);                   // ack byte
  if (r != kErrNone) return r;
  if (frame[0] != 0xAA) return kErrDriverNoAck;

  frame[0] = static_cast<uint8_t>(cmdWord);
  frame[1] = static_cast<uint8_t>(cmdWord >> 8);
  frame[2] = static_cast<uint8_t>(cmdWord >> 16);
  for (int i = 0; i < 8; i++) {               // all 8 words incl. params[7]
    frame[3 + i * 2] = static_cast<uint8_t>(params[i]);
    frame[3 + i * 2 + 1] = static_cast<uint8_t>(params[i] >> 8);
  }

  r = eng_.writeBufferEcho(frame, 19);        // 19-byte request (+ echo)
  if (r != kErrNone) return r;
  if (retries == 0) retries = 200;
  r = eng_.readNTimeout(frame, 17, static_cast<double>(retries));  // 17-byte response
  if (r != kErrNone) return r;
  if (frame[16] != 0xEA) return kErrDriverEa;
  for (int i = 0; i < 8; i++) {
    params[i] = static_cast<uint16_t>(frame[i * 2] | (frame[i * 2 + 1] << 8));
  }
  return kErrNone;
}

// ---------------------------------------------------------------------------
// Post-process / checksum verify
// ---------------------------------------------------------------------------
//
// Wire exchange:
//   write 0x33 ('3') -> read 1 byte -> must be 0xAA, else 0x40b01
//   read 2 bytes [check, term] -> term must be 0xEA, else 0x40c01
//   accumulate: check ^ XOR(buf) ; 0 means OK, else 0x4100b
//
// The original folds the buffer through 32-bit words (XOR reduced by
// >>8 ^ >>16 ^ >>24), which equals a plain byte-wise XOR of the whole buffer,
// so this reduces to: chk ^ XOR(buf) == 0.
uint32_t Protocol::memoryReadPost(const uint8_t* buf, size_t len) {
  uint8_t b[2];
  uint32_t r = eng_.writeByteEcho('3');       // '3' == 0x33
  if (r != kErrNone) return r;
  r = eng_.readN(b, 1);
  if (r != kErrNone) return r;
  if (b[0] != 0xAA) return kErrPostAck;
  r = eng_.readN(b, 2);
  if (r != kErrNone) return r;
  if (b[1] != 0xEA) return kErrPostEa;

  uint8_t acc = b[0];
  for (size_t i = 0; i < len; i++) acc = static_cast<uint8_t>(acc ^ buf[i]);
  return (acc == 0) ? kErrNone : kErrChecksum;
}

// ---------------------------------------------------------------------------
// Boot-mode memory read
// ---------------------------------------------------------------------------
//
// Wire exchange:
//   write 0x85 -> read 1 byte -> must be 0xAA, else 0x40501
//   write 5-byte frame: addr(3 LE) + len(2 LE)
//   read `len` payload bytes
//   read 1 byte terminator -> must be 0xEA, else 0x40601
//   memoryReadPost(buf, len) checksum verify
uint32_t Protocol::memoryRead(uint32_t addr, std::vector<uint8_t>& out) {
  const size_t len = out.size();
  uint8_t frame[5];
  uint32_t r = eng_.writeByteEcho(0x85);    // read-area command
  if (r != kErrNone) return r;
  r = eng_.readN(frame, 1);
  if (r != kErrNone) return r;
  if (frame[0] != 0xAA) return kErrMemAddrAck;

  frame[0] = static_cast<uint8_t>(addr);
  frame[1] = static_cast<uint8_t>(addr >> 8);
  frame[2] = static_cast<uint8_t>(addr >> 16);
  frame[3] = static_cast<uint8_t>(len);
  frame[4] = static_cast<uint8_t>(len >> 8);
  r = eng_.writeBufferEcho(frame, 5);       // addr+len header
  if (r != kErrNone) return r;

  r = eng_.readN(out.data(), len);          // payload
  if (r != kErrNone) return r;

  uint8_t term;
  r = eng_.readN(&term, 1);                 // terminator
  if (r != kErrNone) return r;
  if (term != 0xEA) return kErrMemEa;
  return memoryReadPost(out.data(), len);   // checksum verify
}

// ---------------------------------------------------------------------------
// OBD fast frame transmit
// ---------------------------------------------------------------------------
//
// Frame build: len += 3, SID/ctrl byte += 1, terminator 0x03 written at index
// len. Sends indexes 0..len (len+1 bytes), echoing each byte.
//
// After each writeByteEcho the ECU replies with the byte's complement (~byte)
// as its ack; this is consumed by a per-byte read (skipped for the last byte,
// the 0x03 terminator, which the ECU does not ack). On a single-wire K-line,
// writeByteEcho consumes the *local echo* of the byte we sent, and this second
// read consumes the ECU's ack. Without it the acks would pile up in the receive
// buffer and corrupt the next frame. The RX side (obdRxFrame) is symmetric: it
// reads one ECU byte then acks it via writeByteEcho(~byte); the ECU does not
// re-ack the ack, so no second read there.
uint32_t Protocol::obdTxFrame(uint8_t* frame) {
  const uint8_t n = frame[0];
  frame[0] = static_cast<uint8_t>(n + 3);
  frame[1] = static_cast<uint8_t>(frame[1] + 1);
  frame[frame[0]] = 3;
  pDelay(kFrameGateMs);
  int i = 0;
  for (;;) {
    pDelay(kFrameGateMs);
    uint32_t r = eng_.writeByteEcho(frame[i]);            // send + local echo
    if (r != kErrNone) return r;
    if (i < static_cast<int>(frame[0])) {                 // consume ECU's ~byte ack
      std::uint8_t ack = 0;
      r = eng_.readNTimeout(&ack, 1, 100);
      if (r != kErrNone) return r;
    }
    i++;
    if (static_cast<int>(frame[0]) < i) return kErrNone;
  }
}

// ---------------------------------------------------------------------------
// OBD fast frame receive
// ---------------------------------------------------------------------------
//
// Read the len byte (first-byte timeout kRxByteMs), then for each of bytes
// 1..len: read it, gate the frame spacing, and ack it with its complement
// (~byte) — except the last byte, which is not acked. The final byte must be
// the 0x03 terminator (else 0x50501); on success len is adjusted by -3 and an
// end-of-frame gap is waited.
uint32_t Protocol::obdRxFrame(uint8_t* frame) {
  uint32_t r = eng_.readNTimeout(frame, 1, kRxByteMs);  // len byte
  if (r != kErrNone) return r;

  pDelay(kFrameGateMs);
  const uint8_t n = frame[0];
  r = eng_.writeByteEcho(static_cast<uint8_t>(~frame[0]));  // ack ~len
  if (r != kErrNone) return r;
  for (int i = 1; i <= static_cast<int>(n); i++) {
    r = eng_.readNTimeout(frame + i, 1, 100);
    if (r != kErrNone) return r;
    pDelay(kFrameGateMs);
    if (i < static_cast<int>(n)) {
      r = eng_.writeByteEcho(static_cast<uint8_t>(~frame[i]));  // ack ~byte
      if (r != kErrNone) return r;
    }
  }
  if (frame[n] == 3) {
    frame[0] = static_cast<uint8_t>(n - 3);
    pDelay(kFrameEofMs);
    return kErrNone;
  }
  return kErrObdFrame;
}

// ---------------------------------------------------------------------------
// OBD fast init (5-baud wake-up)
// ---------------------------------------------------------------------------
//
// Bit-encodes wakeByte into a 10-bit sequence and drives it onto the K-line by
// toggling the UART break (break = bit 0, clear = bit 1), holding each bit for
// kFastInitBitMs (~200 ms/bit => ~2 s total at 5 baud), then flushes the line.
// For wakeByte == 1 the generated 10-bit sequence is [0,1,0,0,0,0,0,0,0,1].
uint32_t Protocol::fastInitLine(uint8_t wakeByte) {
  uint8_t b2 = static_cast<uint8_t>(
      (static_cast<uint8_t>(wakeByte * 8) & 0x80) ^
      (static_cast<uint8_t>(wakeByte * 4) & 0x80) ^
      (static_cast<uint8_t>(wakeByte * 2) & 0x80) ^
      (wakeByte | 0x80));
  b2 = static_cast<uint8_t>(((b2 & 0xf8) << 4) ^ b2);
  b2 = static_cast<uint8_t>(((b2 & 0xfc) << 5) ^ b2);
  const uint32_t encoded =
      static_cast<uint32_t>((wakeByte << 7) ^ ((b2 & 0xfe) << 6) ^ b2);

  bool broken = false;
  for (int i = 0; i < 10; i++) {
    const int bit = static_cast<int>(((encoded | 0x100) << 1) >> (i & 0x1f)) & 1;
    if (bit == 0) {
      if (!broken) {
        uint32_t r = eng_.setBreak(true);
        if (r != kErrNone) return r;
        broken = true;
      }
    } else if (broken) {
      uint32_t r = eng_.setBreak(false);
      if (r != kErrNone) return r;
      broken = false;
    }
    pDelay(kFastInitBitMs);
  }
  return eng_.flush();   // purge the line after the wake-up
}

// ---------------------------------------------------------------------------
// OBD init (KWP fast init)
// ---------------------------------------------------------------------------
//
//   fastInitLine(1)                      -> 5-baud wake
//   read 1 byte -> must be 0x55, else 0x50001
//   read 2 bytes -> [1]==1 && [2]==0x8A, else 0x50101
//   gap, then send ~0x8A == 0x75
//   loop: receive a frame
//     frame[2] == 9     -> init complete
//     frame[2] != 0xF6  -> 0x50201
//     frame[2] == 0xF6, frame[0]==5 and 4th frame -> SoftCod/WSC decode
//     otherwise        -> ASCII7-stripped ECU-ID string collected into ecuIds_
//     ack the frame, repeat
//
// The original's heap-backed handle/string-list has no portable analogue; the
// ID strings are collected directly into ecuIds_ and the alloc-fail status
// codes 0x50301/0x50401 cannot occur.
uint32_t Protocol::obdInit(bool fastInit) {
  uint8_t frame[64] = {0};
  ecuIds_.clear();
  softcod_ = 0;
  wsc_ = 0;

  uint32_t r = fastInitLine(fastInit ? 1 : 0);   // 5-baud wake
  if (r != kErrNone) return r;
  r = eng_.readNTimeout(frame, 1, 500);          // sync byte
  if (r != kErrNone) return r;
  if (frame[0] != 0x55) return kErrObdInit1;
  r = eng_.readNTimeout(frame + 1, 2, 100);      // key bytes
  if (r != kErrNone) return r;
  if (!(frame[1] == 1 && frame[2] == 0x8A)) return kErrObdInit2;

  pDelay(kFrameGateMs);                          // wake-up gap
  r = eng_.writeByteEcho(static_cast<uint8_t>(~frame[2]));  // send ~0x8A -> 0x75
  if (r != kErrNone) return r;

  frame[1] = 0;
  int ids = 0;
  for (;;) {
    r = obdRxFrame(frame);                       // receive frame
    if (r != kErrNone) return r;
    if (frame[2] == 9) return kErrNone;          // init complete
    if (frame[2] != 0xF6) return kErrObdInitEnd; // 0x50201

    if (ids == 3 && frame[0] == 5) {
      // SoftCod / WSC decode
      wsc_ = static_cast<uint32_t>(frame[6] | (frame[7] << 8));
      softcod_ = static_cast<uint32_t>((frame[4] << 7) | (frame[5] >> 1) |
                                        (frame[3] << 15));
    } else {
      std::string s;
      s.reserve(frame[0]);
      for (uint8_t i = 0; i < frame[0]; i++) {
        s.push_back(static_cast<char>(frame[3 + i] & 0x7f));   // ASCII7 strip
      }
      ecuIds_.push_back(std::move(s));
    }

    frame[0] = 0;      // reset length
    frame[2] = 9;      // end marker
    r = obdTxFrame(frame);   // ack frame
    if (r != kErrNone) return r;
    ids++;
  }
}

// Byte-block search: returns the first offset in `hay` of a byte-exact match
// of the `len`-byte `needle`, or nullptr.
const uint8_t* Protocol::findBlock(const uint8_t* hay, size_t hayLen,
                                   const uint8_t* needle, size_t len) const {
  if (len == 0 || len > hayLen) return nullptr;
  for (size_t i = 0; i + len <= hayLen; i++) {
    if (std::memcmp(hay + i, needle, len) == 0) return hay + i;
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// OBD EEPROM read
// ---------------------------------------------------------------------------
//
// Phase 1 — 16-byte-block requests across the whole EEPROM.
//   for each 16-byte block at addr:
//     tx frame {3, 0x19, 0x10, addr>>8, addr}, rx response
//     frame[2] == 0xEF -> payload present: copy 16 B (frame+3) to out+addr
//                         (frame[0] must be 0x10, else 0x800ef01)
//     frame[2] == 0x0A -> block missing: fill out+addr with 0xFF
//     otherwise        -> (frame[2]<<8) | 0x8000001
//
// Phase 2 — scan/mirror region to locate the EEPROM's mirror base.
//   for (addr = 0x3F80; addr != 0x2FC0; addr -= 0x40):
//     tx frame {3, 0x01, 0x10, addr>>8, addr}, rx
//     frame[2] must be 0xFE and frame[0] 0x10, else 0x80101
//     findBlock(out, frame+3, 0x10); stop when both flanking blocks are valid
//
// The original's final mirror merge/repair pass rewrites blocks of `out` from a
// mirrored region. Phase 1 already yields the complete EEPROM image (every block
// either payload or 0xFF), so this port returns it directly; the mirror
// merge/repair pass is intentionally not reproduced.
uint32_t Protocol::obdRead(std::vector<uint8_t>& out) {
  const uint32_t size = eepromSize_;
  out.assign(size, 0);
  const size_t blocks = (size + 15) / 16;
  std::vector<uint8_t> valid(blocks + 2, 0);
  uint8_t frame[64];

  // Phase 1 -----------------------------------------------------------------
  uint32_t addr = 0;
  while (static_cast<int>(addr) < static_cast<int>(size)) {
    frame[0] = 3;
    frame[1] = 0;
    frame[2] = 0x19;
    frame[3] = 0x10;
    frame[4] = static_cast<uint8_t>(addr >> 8);
    frame[5] = static_cast<uint8_t>(addr);
    uint32_t r = obdTxFrame(frame);
    if (r != kErrNone) return r;
    r = obdRxFrame(frame);
    if (r != kErrNone) return r;

    const size_t bi = addr >> 4;
    if (frame[2] == 0xEF) {
      if (frame[0] != 0x10) return kErrObdReadLen;
      valid[bi] = 1;
      std::memcpy(out.data() + addr, frame + 3, 16);
    } else if (frame[2] == 0x0A) {
      valid[bi] = 0;
      std::memset(out.data() + addr, 0xFF, 16);
    } else {
      return (static_cast<uint32_t>(frame[2]) << 8) | kErrObdReadSid;
    }
    addr += 0x10;
  }

  // Phase 2 (scan) ----------------------------------------------------------
  int32_t scanAddr = 0x3F80;
  for (;;) {
    frame[0] = 3;
    frame[1] = 0;
    frame[2] = 0x01;
    frame[3] = 0x10;
    frame[4] = static_cast<uint8_t>(scanAddr >> 8);
    frame[5] = static_cast<uint8_t>(scanAddr);
    uint32_t r = obdTxFrame(frame);
    if (r != kErrNone) return r;
    r = obdRxFrame(frame);
    if (r != kErrNone) return r;
    if (frame[2] != 0xFE || frame[0] != 0x10) return kErrObdScan;

    const uint8_t* hit = findBlock(out.data(), out.size(), frame + 3, 0x10);
    if (hit != nullptr) {
      ptrdiff_t didx = hit - out.data();
      if (didx < 0) didx += 0xf;
      const size_t dbi = static_cast<size_t>(didx >> 4);
      if (valid[dbi] != 0 && valid[dbi + 1] != 0) break;
    }
    scanAddr -= 0x40;
    if (scanAddr == 0x2FC0) break;
  }

  return kErrNone;
}

}  // namespace me7
