#pragma once

// K-line / OBD protocol layer for the ME7 EEPROM programmer — a C++17 port of
// the boot-mode driver-command and OBD (fast-init / KWP read) protocol of the
// original ME7EEPROM v1.40 tool. Behavior matches the original's byte values,
// byte orderings, and status codes.
//
// Wire I/O is routed exclusively through me7::SerialEngine (src/core/serial_engine.h);
// timing via me7::nowMs/sleepMs (src/core/timing.h).

#include "core/serial_engine.h"
#include "core/timing.h"

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace me7 {

// ---------------------------------------------------------------------------
// Protocol status codes.
// ---------------------------------------------------------------------------

// call_driver_cmd
inline constexpr uint32_t kErrDriverNoAck = 0x40301;  // ack after 0x9F != 0xAA
inline constexpr uint32_t kErrDriverEa    = 0x40401;  // response[16] != 0xEA

// memory read
inline constexpr uint32_t kErrMemAddrAck  = 0x40501;  // ack after 0x85 != 0xAA
inline constexpr uint32_t kErrMemEa       = 0x40601;  // terminator after payload != 0xEA

// post-process / checksum
inline constexpr uint32_t kErrPostAck     = 0x40b01;  // ack after '3' != 0xAA
inline constexpr uint32_t kErrPostEa      = 0x40c01;  // checksum frame[1] != 0xEA
inline constexpr uint32_t kErrChecksum    = 0x4100b;  // accumulated XOR != 0

// OBD init
inline constexpr uint32_t kErrObdInit1    = 0x50001;  // first byte != 0x55
inline constexpr uint32_t kErrObdInit2    = 0x50101;  // [1]==1 && [2]==0x8A failed
inline constexpr uint32_t kErrObdInitEnd  = 0x50201;  // frame SID neither 9 nor 0xF6
inline constexpr uint32_t kErrObdInitSoft = 0x50301;  // (malloc-fail path; unportable)
inline constexpr uint32_t kErrObdInitId   = 0x50401;  // (malloc-fail path; unportable)
inline constexpr uint32_t kErrObdFrame    = 0x50501;  // rx frame terminator != 3

// OBD read
inline constexpr uint32_t kErrObdReadSid  = 0x8000001; // base | (sid<<8) for bad SID
inline constexpr uint32_t kErrObdReadLen  = 0x800ef01; // payload frame[0] != 0x10
inline constexpr uint32_t kErrObdScan     = 0x80101;   // scan response not 0xFE/0x10

// Wire timing constants (milliseconds).
inline constexpr double kFrameGateMs   = 2.0;   // per-byte frame spacing gate
inline constexpr double kFrameEofMs    = 100.0; // end-of-frame gap
inline constexpr double kFastInitBitMs  = 200.0; // 5-baud wake bit time (~2s for 10 bits)
inline constexpr double kRxByteMs       = 250.0; // first OBD-byte timeout (0xfa)

class Protocol {
 public:
  explicit Protocol(SerialEngine& eng) : eng_(eng) {}

  // --- Driver-command primitive (call_driver_cmd) --------------------------
  // Sends a 19-byte request frame: 0x9F wake -> 0xAA ack -> cmdWord(3 LE bytes)
  // + params[0..7] (16 LE bytes) -> 0xEA terminator. Receives the 8-word
  // (17-byte) response back into `params`. `retries` == 0 selects the default
  // of 200.
  uint32_t callDriverCmd(uint16_t cmdWord, std::array<uint16_t, 8>& params,
                         int retries = 200);

  // --- Boot-mode memory read + post-process checksum verify ------------------
  // `out` is the payload buffer; its size is the read length (sent as the
  // 2-byte LE length). Reads `out.size()` bytes starting at `addr` and
  // verifies the trailing checksum.
  uint32_t memoryRead(uint32_t addr, std::vector<uint8_t>& out);

  // --- OBD fast frame helpers -----------------------------------------------
  uint32_t obdTxFrame(uint8_t* frame);  // transmit + echo ack
  uint32_t obdRxFrame(uint8_t* frame);  // receive

  // --- OBD init -------------------------------------------------------------
  uint32_t fastInitLine(uint8_t wakeByte);  // 5-baud break wake-up
  uint32_t obdInit(bool fastInit = true);   // full KWP init

  const std::vector<std::string>& ecuIds() const { return ecuIds_; }
  uint32_t softCod() const { return softcod_; }
  uint32_t wsc() const { return wsc_; }

  // --- OBD EEPROM read ------------------------------------------------------
  // Reads every 16-byte block of the EEPROM into `out` (resized to eepromSize()).
  uint32_t obdRead(std::vector<uint8_t>& out);

  // --- EEPROM geometry ------------------------------------------------------
  void setEepromSize(uint32_t bytes) { eepromSize_ = bytes; }
  uint32_t eepromSize() const { return eepromSize_; }

 private:
  uint32_t memoryReadPost(const uint8_t* buf, size_t len);  // checksum verify
  const uint8_t* findBlock(const uint8_t* hay, size_t hayLen,
                           const uint8_t* needle, size_t len) const;  // byte-block search
  // Per-frame byte-spacing gate; a per-site delay in place of the original's
  // shared spin-wait timer.
  void pDelay(double ms) const;

  SerialEngine& eng_;
  uint32_t eepromSize_ = 0x200;  // default EEPROM size
  std::vector<std::string> ecuIds_;
  uint32_t softcod_ = 0;
  uint32_t wsc_ = 0;
};

}  // namespace me7
