#pragma once

// Boot-mode (C166 monitor / EEPROM driver upload) orchestration for the ME7
// EEPROM programmer — a C++17 port of the boot-mode flow in the original
// ME7EEPROM v1.40 tool. The run() sequence drives the ECU through these stages:
//
//     "Starting Boot_mode ..."            wake + uC-ID handshake
//     "Sending Loader + MonitorCore"      upload C166 loader + monitor blobs
//     "Testing monitor communication"      {0xAA,0xEA} ack + IDCHIP check
//     "Initializing registers ... OK"     write/verify register table
//     "Sending EEPROM driver"             upload the EEPROM driver blob
//     "Searching Chip_Select pin"         auto-detect the CS pin (if not given)
//     "Configuring SPI Interface ... OK"  bring up the SPI interface
//     "Checking EEPROM Status Register"   read status -> 0x%04X
//
// The EEPROM read/write drivers themselves (boot read / boot write + verify)
// live in the eeprom module and are not part of the uploader.
//
// Wire I/O uses me7::SerialEngine (low-level K-line) for the raw byte helpers
// and me7::Protocol for the driver-command primitive and the boot-mode memory
// read. C166 firmware blobs are opaque data supplied by me7::firmware
// (see src/firmware/embedded.h).

#include "core/serial_engine.h"
#include "core/options.h"

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace me7 {

class Protocol;

// ---------------------------------------------------------------------------
// Boot-mode status codes.
// (This header is the single point of truth for the code constants.)
// ---------------------------------------------------------------------------

// loader/monitor handshake
inline constexpr uint32_t kErrLdrId0   = 0x40001;  // first handshake byte != 0x01
inline constexpr uint32_t kErrLdrId1   = 0x40101;  // second handshake byte != 0x03

// uC ID acceptance
inline constexpr uint32_t kErrUcId     = 0x40201;  // unknown / unsupported uC ID

// 5-byte raw command
inline constexpr uint32_t kErrCmdNoAck = 0x40901;  // ack after 0x82 != 0xAA
inline constexpr uint32_t kErrCmdEa    = 0x40a01;  // terminator != 0xEA

// send blob
inline constexpr uint32_t kErrBlobNoAck= 0x40701;  // ack after 0x84 != 0xAA
inline constexpr uint32_t kErrBlobEa   = 0x40801;  // terminator != 0xEA

// read 16-bit value
inline constexpr uint32_t kErrReadNoAck= 0x40d01;  // ack after 0xCD != 0xAA
inline constexpr uint32_t kErrReadEa   = 0x40e01;  // terminator != 0xEA

// test monitor communication
inline constexpr uint32_t kErrMonComm  = 0x40f01;  // response != {0xAA,0xEA}

// post-process checksum (also in protocol.h)
inline constexpr uint32_t kErrPostAckB = 0x40b01;  // ack after '3' != 0xAA
inline constexpr uint32_t kErrPostEaB  = 0x40c01;  // checksum frame[1] != 0xEA
inline constexpr uint32_t kErrChkSum   = 0x4100b;  // accumulated XOR != 0

// write + readback verify
inline constexpr uint32_t kErrVerify   = 0x41101;  // readback != written value

// register init
inline constexpr uint32_t kErrRegMismatch = 0x60001;  // readback != expected

// Boot-mode EEPROM driver errors
inline constexpr uint32_t kErrCsUnknown = 0x70001;   // unknown CS port
inline constexpr uint32_t kErrSpiStatus = 0x70201;   // SPI config status != 0
inline constexpr uint32_t kErrCsBusy    = 0x70301;   // CS detect status != 0
inline constexpr uint32_t kErrDrvStatus = 0x70901;   // driver status != 0x55
inline constexpr uint32_t kErrStatus    = 0x70801;   // status register read status != 0
inline constexpr uint32_t kErrCsVerify  = 0x70c01;   // CS ID pattern bad

// ---------------------------------------------------------------------------
// Chip-select pin table ("Sending EEPROM driver" with a CS pin, and the
// --CSpin option). The monitor's chip-select config uses a PORT-ID encoding —
// NOT the physical SFR addresses the --help text prints. P4 = portId 0x00e4,
// dirctrl 0x00e5, maxbit 7. The driver-blob write puts (byte)portId into bytes
// 1/5 and (byte)dirctrl into byte 9. The default no-CS path passes
// (csPort=0, dirByte=0, bit=0), which the driver accepts as-is.
// ---------------------------------------------------------------------------
struct ChipSelConfig {
  const char* name;      // e.g. "P4"
  std::uint16_t portReg; // PortReg (physical SFR addr, help-text only)
  std::uint16_t dirReg;  // DirControlReg (physical SFR addr, help-text only)
  std::uint8_t  dirByte; // dirctrl port-id (0xe5 for P4) written into blob byte 9
};

class BootUploader {
 public:
  BootUploader(SerialEngine& serial, Protocol& protocol);

  // Run the full boot-mode init sequence (wake → ... → status register) exactly
  // as main() dispatches it, streaming the human-readable transcript to stdout
  // live (each stage prefix printed before its blocking serial exchange). Returns
  // 0 when the boot reaches (and reports) the EEPROM status register, else the
  // first error code encountered. If the uC/IDCHIP is unsupported the run stops
  // before the driver steps and returns 0 (the "Not supported" line is already
  // printed).
  uint32_t run(const Options& opts);

  // ---- Per-step primitives -------------------------------------------------

  uint32_t bootInit(std::uint8_t& ucId);                        // wake + uC-ID handshake
  uint32_t sendLoaderMonitor();                                 // upload loader + monitor blobs
  uint32_t testMonitorComm();                                   // {0xAA,0xEA} comm test
  uint32_t readIdchip(std::uint16_t reg, std::uint16_t& value); // read a 16-bit reg
  uint32_t dcc(std::uint16_t reg, std::uint16_t value);         // write a reg + verify
  uint32_t initRegisters();                                     // write/verify register table
  uint32_t sendEepromDriver(std::uint16_t csPort, std::uint8_t dirByte,
                            std::uint8_t bit);                  // upload EEPROM driver blob
  uint32_t searchChipSelect(std::uint8_t& portId,
                            std::uint8_t& bit);                 // auto-detect CS pin
  uint32_t configSpi();                                         // bring up SPI interface
  uint32_t checkStatus(std::uint16_t& status);                  // read EEPROM status register

  // Accessors for values recovered during run().
  std::uint16_t idChip() const { return idchip_; }
  std::uint16_t statusRegister() const { return status_; }
  int effectiveMemType() const { return memType_; }   // mem-type index (95160→1 quirk applied)

  // Maps an Options.memType string to the memory-type index (95040=0, 95080=1,
  // 95P08=2, 95160=3) or -1 if unknown.
  static int memTypeIndex(const std::string& memType);

  // CS name for a port id returned by searchChipSelect (0xe0=P2 ... 0xea=P8).
  static const char* csPortName(std::uint8_t portId);

 private:
  uint32_t sendCmdWord(std::uint16_t cmd, std::uint16_t data);      // 5-byte raw command
  uint32_t sendBlob(const std::vector<std::uint8_t>& blob);         // upload a blob + checksum
  uint32_t checksumPost(const std::uint8_t* buf, size_t len);       // XOR checksum verify
  uint32_t detectChipSelect(std::uint8_t& portId, std::uint8_t& bit); // CS/GPIO id read

  static bool parseCsPin(const std::string& pin, std::uint16_t& portReg,
                         std::uint8_t& dirByte, std::uint8_t& bit);

  SerialEngine& serial_;
  Protocol& protocol_;

  std::uint16_t idchip_ = 0;
  std::uint16_t status_ = 0;
  int memType_ = 0;
};

}  // namespace me7
