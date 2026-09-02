// Boot-mode uploader implementation — drives the ME7 ECU's C166 monitor and
// EEPROM driver upload sequence over the K-line. Function comments describe the
// wire-level framing and byte orderings a maintainer needs; see uploader.h for
// the public API. Behavior matches the original ME7EEPROM v1.40 tool.

#include "core/uploader.h"
#include "core/protocol.h"
#include "core/timing.h"
#include "firmware/embedded.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace me7 {

namespace {

// Stream a printf-style fragment to stdout and flush, so the boot-mode
// transcript prints live (matching the original's interleaved printf()/puts()).
// The original prints each stage prefix with printf("... ") BEFORE the blocking
// serial exchange, then the result with puts()/printf() afterwards; emitting
// live (instead of buffering into a string) is what lets you see *which* stage
// the tool is sitting in when an ECU is slow or absent.
void out(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  std::fputs(buf, stdout);
  std::fflush(stdout);
}

}  // namespace

// ---------------------------------------------------------------------------
// BootUploader
// ---------------------------------------------------------------------------

BootUploader::BootUploader(SerialEngine& serial, Protocol& protocol)
    : serial_(serial), protocol_(protocol) {}

int BootUploader::memTypeIndex(const std::string& memType) {
  // main(): strcmp("95040")->0, "95080"->1, "95P08"->2, "95160"->3.
  if (memType == "95040") return 0;
  if (memType == "95080") return 1;
  if (memType == "95P08") return 2;
  if (memType == "95160") return 3;
  return -1;
}

const char* BootUploader::csPortName(std::uint8_t portId) {
  // port-id encoding used by the monitor: 0xd1=P5, 0xe0=P2, 0xe2=P3,
  // 0xe4=P4, 0xe6=P6, 0xe8=P7, 0xea=P8.
  switch (portId) {
    case 0xd1: return "P5";
    case 0xe0: return "P2";
    case 0xe2: return "P3";
    case 0xe4: return "P4";
    case 0xe6: return "P6";
    case 0xe8: return "P7";
    case 0xea: return "P8";
    default: return nullptr;
  }
}

bool BootUploader::parseCsPin(const std::string& pin, std::uint16_t& portReg,
                              std::uint8_t& dirByte, std::uint8_t& bit) {
  // --CSpin "Px.y" -> the monitor's chip-select config uses a PORT-ID
  // encoding, NOT the physical SFR addresses the --help text prints (P4 =
  // PortReg 0xFFC8 / DirControlReg 0xFFCA) — those are for human reference only
  // and are NOT what goes on the wire. The encoding is:
  //   P2=0xe0/0xe1  P3=0xe2/0xe3  P4=0xe4/0xe5  P6=0xe6/0xe7
  //   P7=0xe8/0xe9  P8=0xea/0xeb   (dirctrl == portId + 1)
  // The driver-blob write puts (byte)portId into bytes 1/5 and (byte)dirctrl
  // into byte 9. Sending the SFR address (0xC8) or a bit mask (0x80) instead
  // makes the driver mis-configure the CS pin and hang on the 0xf610 init,
  // surfacing as a readN timeout (0x21007) at "Sending EEPROM driver".
  struct { char port; std::uint8_t id; std::uint8_t maxbit; } kTbl[] = {
      {'2', 0xe0, 15}, {'3', 0xe2, 15}, {'4', 0xe4, 7},
      {'6', 0xe6, 7},  {'7', 0xe8, 7},  {'8', 0xea, 7},
  };
  if (pin.size() < 4 || pin[0] != 'P' || pin[2] != '.') return false;
  std::uint8_t portId = 0;
  std::uint8_t maxbit = 0;
  bool found = false;
  for (const auto& e : kTbl) {
    if (pin[1] == e.port) { portId = e.id; maxbit = e.maxbit; found = true; break; }
  }
  if (!found) return false;
  const int b = pin[3] - '0';
  if (b < 0 || b > maxbit) return false;
  portReg = portId;                 // driver-blob bytes 1/5 = (byte)portId
  dirByte = static_cast<std::uint8_t>(portId + 1);  // blob byte 9 = (byte)dirctrl
  bit = static_cast<std::uint8_t>(b);
  return true;
}

// ---------------------------------------------------------------------------
// Raw byte helpers (low-level K-line, via SerialEngine)
// ---------------------------------------------------------------------------

// 5-byte raw command {cmd, data}: send 0x82, read ack 0xAA, send cmd (3 bytes
// LE) + data (2 bytes LE), read terminator 0xEA.
//   writeByteEcho(0x82) -> readN(1) -> expect 0xAA else 0x40901
//   writeBufferEcho(frame,5) -> readN(1) -> expect 0xEA else 0x40a01
uint32_t BootUploader::sendCmdWord(std::uint16_t cmd, std::uint16_t data) {
  uint8_t frame[5];
  uint32_t r = serial_.writeByteEcho(0x82);      // command lead-in
  if (r != kErrNone) return r;
  r = serial_.readN(frame, 1);                   // ack
  if (r != kErrNone) return r;
  if (frame[0] != 0xAA) return kErrCmdNoAck;

  frame[0] = static_cast<uint8_t>(cmd);
  frame[1] = static_cast<uint8_t>(cmd >> 8);
  frame[2] = static_cast<uint8_t>(cmd >> 16);
  frame[3] = static_cast<uint8_t>(data);
  frame[4] = static_cast<uint8_t>(data >> 8);
  r = serial_.writeBufferEcho(frame, 5);         // {cmd LE3, data LE2}
  if (r != kErrNone) return r;
  r = serial_.readN(frame, 1);                   // terminator
  if (r != kErrNone) return r;
  if (frame[0] != 0xEA) return kErrCmdEa;
  return kErrNone;
}

// Read a 16-bit value from a driver/register address.
//   writeByteEcho(0xCD) -> readN(1) -> expect 0xAA else 0x40d01
//   writeBufferEcho({reg LE2, reg>>16}, 3) -> readN(3)
//   terminator (frame[2]) must be 0xEA else 0x40e01; value = frame[0]|frame[1]<<8.
uint32_t BootUploader::readIdchip(std::uint16_t reg, std::uint16_t& value) {
  uint8_t frame[3];
  uint32_t r = serial_.writeByteEcho(0xCD);      // read lead-in
  if (r != kErrNone) return r;
  r = serial_.readN(frame, 1);                   // ack
  if (r != kErrNone) return r;
  if (frame[0] != 0xAA) return kErrReadNoAck;

  frame[0] = static_cast<uint8_t>(reg);
  frame[1] = static_cast<uint8_t>(reg >> 8);
  frame[2] = static_cast<uint8_t>(reg >> 16);    // 0 for 16-bit regs
  r = serial_.writeBufferEcho(frame, 3);         // {reg LE2, reg>>16}
  if (r != kErrNone) return r;
  r = serial_.readN(frame, 3);                   // {value LE2, terminator}
  if (r != kErrNone) return r;
  if (frame[2] != 0xEA) return kErrReadEa;       // terminator check
  value = static_cast<uint16_t>(frame[0] | (frame[1] << 8));
  return kErrNone;
}

// Write a value to a driver address and read it back to verify.
//   sendCmdWord(reg, value) -> readIdchip(reg, readback) -> mismatch -> 0x41101.
uint32_t BootUploader::dcc(std::uint16_t reg, std::uint16_t value) {
  uint32_t r = sendCmdWord(reg, value);
  if (r != kErrNone) return r;
  std::uint16_t readback = 0;
  r = readIdchip(reg, readback);
  if (r != kErrNone) return r;
  if (readback != value) return kErrVerify;
  return kErrNone;
}

// Post-process / checksum verify of a transmitted payload: ask the device for
// its accumulated XOR, then XOR that against the local buffer.
//   writeByteEcho('3'=0x33) -> readN(1) -> expect 0xAA else 0x40b01
//   readN(2) -> terminator (b[1]) must be 0xEA else 0x40c01
//   acc = b[0] ^ buf[...] over all bytes; acc==0 -> ok, else 0x4100b.
uint32_t BootUploader::checksumPost(const std::uint8_t* buf, size_t len) {
  uint8_t b[2];
  uint32_t r = serial_.writeByteEcho('3');       // '3' == 0x33
  if (r != kErrNone) return r;
  r = serial_.readN(b, 1);
  if (r != kErrNone) return r;
  if (b[0] != 0xAA) return kErrPostAckB;
  r = serial_.readN(b, 2);
  if (r != kErrNone) return r;
  if (b[1] != 0xEA) return kErrPostEaB;
  uint8_t acc = b[0];
  for (size_t i = 0; i < len; i++) {
    acc = static_cast<uint8_t>(acc ^ buf[i]);
  }
  return (acc == 0) ? kErrNone : kErrChkSum;
}

// Send the EEPROM driver blob (0x35c bytes): send 0x84, read ack 0xAA, send a
// 5-byte header {cmd 0xf600 LE3, len LE2}, then the payload itself, read the
// 0xEA terminator, then run the checksum post-process.
//   writeByteEcho(0x84) -> readN(1) -> expect 0xAA else 0x40701
//   writeBufferEcho(header,5) -> writeBufferEcho(payload,len)
//   readN(1) -> expect 0xEA else 0x40801 -> checksumPost(payload, len)
uint32_t BootUploader::sendBlob(const std::vector<std::uint8_t>& blob) {
  const size_t len = blob.size();
  uint8_t frame[5];
  uint32_t r = serial_.writeByteEcho(0x84);      // blob lead-in
  if (r != kErrNone) return r;
  r = serial_.readN(frame, 1);                   // ack
  if (r != kErrNone) return r;
  if (frame[0] != 0xAA) return kErrBlobNoAck;

  frame[0] = 0x00;                               // cmd word 0xf600, 3 bytes LE
  frame[1] = 0xF6;
  frame[2] = 0x00;
  frame[3] = static_cast<uint8_t>(len);
  frame[4] = static_cast<uint8_t>(len >> 8);
  r = serial_.writeBufferEcho(frame, 5);         // {cmd LE3, len LE2}
  if (r != kErrNone) return r;
  r = serial_.writeBufferEcho(blob.data(), len); // payload
  if (r != kErrNone) return r;
  r = serial_.readN(frame, 1);                   // terminator
  if (r != kErrNone) return r;
  if (frame[0] != 0xEA) return kErrBlobEa;
  return checksumPost(blob.data(), len);
}

// ---------------------------------------------------------------------------
// The boot-mode step functions
// ---------------------------------------------------------------------------

// "Starting Boot_mode" init: send 0x00, read the uC-ID byte, accept/describe
// it. The uC-ID result line is streamed live (prefix before the blocking read,
// result after), matching the original tool's interleaved output. On read
// timeout the original overwrites its banner with "FAIL ... No ECU response";
// because the prefix is printed live we emit the fail text after it.
// Return codes: 0 (0xd5/0xc5), 0x40201 (unsupported/unknown), serial timeout.
uint32_t BootUploader::bootInit(std::uint8_t& ucId) {
  // ME7_DEBUG_BOOT diagnostic: when set, bypass the strict echo+1-byte-ID read
  // and instead raw-send the 0x00 wake byte, then drain EVERY byte received over
  // a 400 ms window printing hex + ms offsets. The normal path reads the uC-ID
  // inside a ~1 ms window anchored right after the echo read; on a single-wire
  // K-line the C167 BSL can reply before that window is set up (echo/response
  // race) or mis-frame, so the strict read hides what is really on the wire. The
  // drain shows the full picture (local-echo 0x00, the BSL ID 0xC5, retries,
  // silence) and is what disambiguates "not in BSL" from "timing race".
  if (const char* dbg = std::getenv("ME7_DEBUG_BOOT")) {
    if (dbg[0] != '\0') {
      // Purge any stale RX-buffer bytes (framing-error zeros left from prior
      // attempts) so the drain shows only bytes the ECU sends in response to
      // THIS wake. Report residue first so we can confirm the purge worked.
      serial_.flush();
      std::vector<std::uint8_t> pre;
      const double tp = nowMs();
      while (nowMs() < tp + 30.0) {
        std::uint8_t b = 0;
        if (serial_.port().read(&b, 1, 0)) pre.push_back(b);
        else sleepMs(1);
      }
      out("DBG post-purge residue in 30ms: %zu byte(s)\n", pre.size());
      for (size_t i = 0; i < pre.size(); ++i)
        out("DBG stale 0x%02X\n", pre[i]);

      std::uint8_t wake = 0x00;
      serial_.port().write(&wake, 1);
      out("DBG sent wake 0x00, draining 400ms...\n");
      const double t0 = nowMs();
      const double drainUntil = t0 + 400.0;
      std::vector<std::uint8_t> rx;
      while (nowMs() < drainUntil) {
        std::uint8_t b = 0;
        size_t got = serial_.port().read(&b, 1, 0);
        if (got) {
          rx.push_back(b);
          out("DBG +%.0fms 0x%02X\n", nowMs() - t0, b);
        } else {
          sleepMs(1);
        }
      }
      out("DBG drain done: %zu byte(s)\n", rx.size());
      return kErrUcId;   // stop here; the dump is the point
    }
  }

  uint32_t r = serial_.writeByteEcho(0x00);      // BSL wake byte
  if (r == kErrNone) {
    r = serial_.readN(&ucId, 1);                 // read uC-ID
    if (r == kErrNone) {
      out("uC ID response 0x%02X", ucId);        // streamed result line
      const uint8_t b = ucId;
      if (b == 0xb5) {                           // (Previous versions of the C165)
        out(": (Previous versions of the C165). Not supported");
        return kErrUcId;
      }
      if (b < 0xb6) {
        if (b == 0x55) { out(": (8xC166). Not supported"); return kErrUcId; }
        if (b == 0xa5) {
          out(": (Previous versions of the C167). Not supported");
          return kErrUcId;
        }
      } else {
        if (b == 0xc5) { out(": C167CR ... OK"); return kErrNone; }
        if (b == 0xd5) { out(" ... OK"); return kErrNone; }
      }
      out(". Unknown ID");
      return kErrUcId;
    }
    if ((static_cast<char>(r) == '\a'))           // serial timeout (0x2x07)
      out("FAIL ... No ECU response");
  } else if ((r & 0xff) == 7) {
    out("?FAIL ... No echo, check cable");
  } else if ((r & 0xff) == 8) {
    out("FAIL ... Invalid echo, check cable");
  }
  return r;
}

// "Sending Loader + MonitorCore". Two echo-sends of DISTINCT C166 blobs, each
// acknowledged with a handshake byte (0x01 then 0x03):
//   writeBufferEcho(loader, 0x20 bytes)   -> read 1 byte; if != 0x01 return 0x40001
//   writeBufferEcho(monitor, 0x18A bytes)-> read 1 byte; if != 0x03 return 0x40101
//
// These are two distinct blobs sent in two phases, NOT one blob sent twice: the
// first is the 32-byte C166 on-chip bootstrap loader image (the bootstrap only
// accepts 0x20 bytes in phase 1); the second is the monitor it hands off to.
uint32_t BootUploader::sendLoaderMonitor() {
  const firmware::Blob ldr = firmware::loader_core();   // 0x20 bytes -> ack 0x01
  const firmware::Blob mon = firmware::monitor_core();  // 0x18A bytes -> ack 0x03

  uint8_t ack = 0;
  uint32_t r = serial_.writeBufferEcho(ldr.data, ldr.size);   // loader
  if (r != kErrNone) return r;
  r = serial_.readN(&ack, 1);                // ack 0x01
  if (r != kErrNone) return r;
  if (ack != 0x01) return kErrLdrId0;

  r = serial_.writeBufferEcho(mon.data, mon.size);            // monitor
  if (r != kErrNone) return r;
  r = serial_.readN(&ack, 1);                // ack 0x03
  if (r != kErrNone) return r;
  if (ack != 0x03) return kErrLdrId1;
  return kErrNone;
}

// "Testing monitor communication": send 0x93, expect the two-byte {0xAA, 0xEA}
// acknowledgement from the running monitor. Else 0x40f01.
uint32_t BootUploader::testMonitorComm() {
  uint32_t r = serial_.writeByteEcho(0x93);      // comm-test lead-in
  if (r != kErrNone) return r;
  uint8_t buf[2];
  r = serial_.readN(buf, 2);                     // {0xAA, 0xEA}
  if (r != kErrNone) return r;
  if (buf[0] != 0xAA || buf[1] != 0xEA) return kErrMonComm;
  return kErrNone;
}

// "Initializing registers". Writes a small register table {0xFF12:0x04E6,
// 0xFF0C:0x04AE} via dcc() and verifies each readback. Mismatch -> 0x60001.
uint32_t BootUploader::initRegisters() {
  // Register-init table (entries {reg, value}).
  static const struct { std::uint16_t reg, value; } kRegs[] = {
      {0xFF12, 0x04E6},   // SYSCON etc.
      {0xFF0C, 0x04AE},
  };
  for (const auto& e : kRegs) {
    uint32_t r = dcc(e.reg, e.value);
    if (r != kErrNone) return r;
    std::uint16_t readback = 0;
    r = readIdchip(e.reg, readback);
    if (r != kErrNone) return r;
    if (readback != e.value) return kErrRegMismatch;  // 0x60001
  }
  return kErrNone;
}

// "Sending EEPROM driver" (optionally with a CS pin configuration baked into
// the blob bytes 0/1/4/5/8/9).
uint32_t BootUploader::sendEepromDriver(std::uint16_t csPort,
                                        std::uint8_t dirByte, std::uint8_t bit) {
  // Copy the 0x35c-byte driver blob into a working buffer; optionally patch the
  // CS bytes; send via sendBlob, then callDriverCmd(0xf610, {0x93,...}, 100)
  // expecting the status word (params[7]) to be 0x55 (else 0x70901).
  const firmware::Blob drv = firmware::eeprom_driver();
  std::vector<std::uint8_t> mem(0x35c, 0);       // driver blob + zero padding
  const size_t n = drv.size < mem.size() ? drv.size : mem.size();
  std::memcpy(mem.data(), drv.data, n);

  if (csPort != 0) {                             // CS config requested
    mem[0] = static_cast<std::uint8_t>((bit << 4) | 0x0E);
    mem[1] = static_cast<std::uint8_t>(csPort);
    const std::uint8_t bVar1 = static_cast<std::uint8_t>((bit << 4) | 0x0F);
    mem[4] = bVar1;
    mem[5] = static_cast<std::uint8_t>(csPort);
    mem[8] = bVar1;
    mem[9] = dirByte;
  }

  uint32_t r = sendBlob(mem);
  if (r != kErrNone) return r;

  std::array<std::uint16_t, 8> params = {0x93, 0, 0, 0, 0, 0, 0, 0};
  r = protocol_.callDriverCmd(0xf610, params, 100);
  if (r != kErrNone) return r;
  if (params[7] != 0x55) return kErrDrvStatus;        // status word != 0x55 -> 0x70901
  return kErrNone;
}

// Read the chip-select / GPIO identification; returns the port id + bit
// pattern via out params. Helper for searchChipSelect.
uint32_t BootUploader::detectChipSelect(std::uint8_t& portId, std::uint8_t& bit) {
  //   callDriverCmd(0xf610, {5,0,...}, 6000);
  //   if (params[7] != 0) return 0x70301;
  //   addr = params[6]*0x4000 + params[5] - 10   (wrapped, matches int arithmetic)
  //   memoryRead(addr & 0xFFFFFF, buf, 10);
  //   if (buf[4..5] != {0xDB,0x00} || (buf[6]&0xF)!=0xF || (buf[8]&0xF)!=0xF)
  //     return 0x70c01;
  //   portId = buf[7]; bit = buf[6] >> 4;
  //   dcc(0xf600, word@6 & 0xFFFE); dcc(0xf604, word@6); dcc(0xf608, word@8);
  std::array<std::uint16_t, 8> params = {5, 0, 0, 0, 0, 0, 0, 0};
  uint32_t r = protocol_.callDriverCmd(0xf610, params, 6000);
  if (r != kErrNone) return r;
  if (params[7] != 0) return kErrCsBusy;         // busy -> 0x70301

  const std::uint32_t addr =
      static_cast<std::uint32_t>(params[6] * 0x4000) + params[5] + 0xFFFFFFF6u;
        // params[6]*0x4000 + params[5] - 10  (wrapped, matches int arithmetic)

  std::vector<std::uint8_t> buf(10, 0);          // 10-byte GPIO read
  r = protocol_.memoryRead(addr & 0xFFFFFF, buf);
  if (r != kErrNone) return r;
  // byte layout of the 10 read bytes:
  //   r[4..5] = GPIO signature {0xDB,0x00}; r[6..7] = port-id word LE;
  //   r[8..9] = config word LE.
  if (!(buf[4] == 0xDB && buf[5] == 0x00 &&      // signature {0xDB,0x00}
        (buf[6] & 0x0F) == 0x0F &&               // port-id low nibble == 0xF
        (buf[8] & 0x0F) == 0x0F)) {              // config low nibble == 0xF
    return kErrCsVerify;                          // 0x70c01
  }
  const std::uint16_t portIdWord = static_cast<std::uint16_t>(buf[6] | (buf[7] << 8));
  const std::uint16_t configWord = static_cast<std::uint16_t>(buf[8] | (buf[9] << 8));
  portId = buf[7];                               // high byte of port-id word
  bit = static_cast<std::uint8_t>(buf[6] >> 4);  // bit = high nibble of low byte
  r = dcc(0xf600, static_cast<std::uint16_t>(portIdWord & 0xFFFE));
  if (r != kErrNone) return r;
  r = dcc(0xf604, portIdWord);
  if (r != kErrNone) return r;
  return dcc(0xf608, configWord);
}

// "Searching Chip_Select pin" (auto-detect).
uint32_t BootUploader::searchChipSelect(std::uint8_t& portId, std::uint8_t& bit) {
  uint32_t r = detectChipSelect(portId, bit);
  if (r != kErrNone) return r;
  // Map the detected port id to a name; unknown id -> 0x70001.
  const char* name = csPortName(portId);
  if (!name) return kErrCsUnknown;
  std::printf("%s.%d\n", name, bit);             // prints "P5.%d".."P8.%d"
  return kErrNone;
}

// "Configuring SPI Interface". callDriverCmd with opcode 1 (retries default
// 200); the status word (params[7]) must be 0 (else 0x70201).
uint32_t BootUploader::configSpi() {
  std::array<std::uint16_t, 8> params = {1, 0, 0, 0, 0, 0, 0, 0};
  uint32_t r = protocol_.callDriverCmd(0xf610, params, 0);  // 0 -> default 200
  if (r != kErrNone) return r;
  if (params[7] != 0) return kErrSpiStatus;      // status word != 0 -> 0x70201
  return kErrNone;
}

// "Checking EEPROM Status Register". callDriverCmd with opcode 4; the status
// register value is params[5]; the status word (params[7]) must be 0 (else
// 0x70801).
uint32_t BootUploader::checkStatus(std::uint16_t& status) {
  std::array<std::uint16_t, 8> params = {4, 0, 0, 0, 0, 0, 0, 0};
  uint32_t r = protocol_.callDriverCmd(0xf610, params, 100);
  if (r != kErrNone) return r;
  if (params[7] != 0) return kErrStatus;         // status word != 0 -> 0x70801
  status = params[5];                            // status register value
  return kErrNone;
}

// ---------------------------------------------------------------------------
// run() — the boot-mode orchestration (the sequence main() drives).
// ---------------------------------------------------------------------------
uint32_t BootUploader::run(const Options& opts) {
  memType_ = memTypeIndex(opts.memType);

  // "Starting Boot_mode" — prefix emitted BEFORE the blocking uC-ID read so it
  // appears live.
  out("Starting Boot_mode ... ");
  std::uint8_t ucId = 0;
  uint32_t r = bootInit(ucId);
  out("\n");
  if (r != kErrNone) return r;

  // "Sending Loader + MonitorCore"
  out("Sending Loader + MonitorCore ... ");
  r = sendLoaderMonitor();
  if (r != kErrNone) return r;

  // "Testing monitor communication"
  r = testMonitorComm();
  if (r != kErrNone) return r;
  out("MonitorCore successfully launched\n");

  if (static_cast<std::int8_t>(ucId) == -0x2b) {     // uC-ID == 0xD5
    // "Reading IDCHIP"
    out("Reading IDCHIP ... ");
    r = readIdchip(0xf07c, idchip_);
    if (r != kErrNone) return r;
    if ((idchip_ & 0xFF00u) == 0x0C00u) {
      out("0x%04X: C167CS ... OK\n", idchip_);
      // fall through to "Initializing registers"
    } else {
      out("0x%04X. Not supported\n", idchip_);
      // "Not supported" -> stop before the driver steps (as the original does).
      return kErrNone;
    }
  }
  // (else uC-ID != 0xD5, e.g. C167CR 0xC5: skip the IDCHIP read)

  // "Initializing registers"
  out("Initializing registers ... ");
  r = initRegisters();
  if (r != kErrNone) return r;
  out("OK\n");

  // "Sending EEPROM driver"
  out("Sending EEPROM driver ... ");
  const bool hasCs = !opts.csPin.empty();
  std::uint16_t csPort = 0; std::uint8_t dirByte = 0, csBit = 0;
  if (hasCs && !parseCsPin(opts.csPin, csPort, dirByte, csBit)) {
    return kErrCsUnknown;                          // invalid --CSpin
  }
  if (hasCs) {
    r = sendEepromDriver(csPort, dirByte, csBit);
  } else {
    r = sendEepromDriver(0, 0, 0);
  }
  if (r != kErrNone) return r;
  out("OK\n");

  if (!hasCs) {                                    // no --CSpin -> auto-detect
    // "Searching Chip_Select pin"
    out("Searching Chip_Select pin ... ");
    std::uint8_t portId = 0, bit = 0;
    r = searchChipSelect(portId, bit);
    if (r != kErrNone) return r;
  }

  // "Configuring SPI Interface"
  out("Configuring SPI Interface ... ");
  r = configSpi();
  if (r != kErrNone) return r;
  out("OK\n");

  // "Checking EEPROM Status Register"
  out("Checking EEPROM Status Register ... ");
  r = checkStatus(status_);
  if (r != kErrNone) return r;
  out("0x%04X\n", status_);

  // 95160 -> 95080 quirk: memory index 3 is treated as 1 by the driver.
  if (memType_ == 3) memType_ = 1;

  return kErrNone;
}

}  // namespace me7
