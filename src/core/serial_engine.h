#pragma once

// Low-level K-line serial engine — a portable reimplementation of the original
// ME7EEPROM v1.40 Win32 serial layer (open comport, flush, set/clear break,
// write byte + echo, write buffer + echo, read).
//
// Every method returns the original's exact status code (0 == success, or the
// documented error constant such as 0x21007 "read timeout"). The engine talks
// only to the abstract SerialPort (src/pal/serial_pal.h) so the protocol core
// stays byte-identical regardless of the host serial backend.

#include "pal/serial_pal.h"
#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace me7 {

// K-line error codes (low nibble = stage, high = base), matching the original.
inline constexpr uint32_t kErrNone = 0;
inline constexpr uint32_t kErrIlegalBaud = 0x20001;   // illegal baudrate
inline constexpr uint32_t kErrIlegalComport = 0x20101;// illegal comport
inline constexpr uint32_t kErrOpen = 0x20201;         // unable to open comport
inline constexpr uint32_t kErrDcb = 0x20301;          // unable to set DCB
inline constexpr uint32_t kErrCfg = 0x20401;          // unable to set cfg settings
inline constexpr uint32_t kErrTimeouts = 0x20501;     // unable to set time-out settings
inline constexpr uint32_t kErrPurge = 0x20609;        // PurgeComm failed
inline constexpr uint32_t kErrSetBreak = 0x20709;     // SetCommBreak failed
inline constexpr uint32_t kErrClearBreak = 0x20809;   // ClearCommBreak failed
inline constexpr uint32_t kErrWrite = 0x20909;        // WriteFile failed / 0 bytes
inline constexpr uint32_t kErrEcho = 0x20a08;         // byte echo mismatch
inline constexpr uint32_t kErrNoEcho = 0x20b07;       // byte echo timeout
inline constexpr uint32_t kErrWriteCount = 0x20c09;   // write buffer count mismatch
inline constexpr uint32_t kErrWriteTimeout = 0x20d07; // write buffer echo timeout
inline constexpr uint32_t kErrBufMismatch = 0x20e08;  // buffer echo mismatch
inline constexpr uint32_t kErrReadTimeout = 0x20f07;  // readN timeout
inline constexpr uint32_t kErrReadTimeout2 = 0x21007; // readNwithTimeout timeout

class SerialEngine {
 public:
  explicit SerialEngine(std::unique_ptr<SerialPort> port) : port_(std::move(port)) {}

  // Baud-to-value mapping for the Win32 BuildCommDCB string. Returns the
  // value to feed BuildCommDCBA or -1 if not one of the allowed rates.
  static int normalizeBaud(int baud);

  // Open COM<n> at `baud`. Sets error message on failure.
  uint32_t open(int comport, int baud, std::string* errMsg = nullptr);

  void close();

  // PurgeComm(ALL). 0 on success.
  uint32_t flush();
  // Set/clear the UART break line.
  uint32_t setBreak(bool on);

  // Transmit one byte and, within the echo window, require the same byte to
  // come back (K-line local-echo). 0 on match.
  uint32_t writeByteEcho(std::uint8_t b);

  // Transmit a buffer and read back the identical bytes (echo).
  uint32_t writeBufferEcho(const std::uint8_t* buf, size_t n);

  // Read exactly n bytes; timeout derived from n/baud. 0x20f07 on timeout.
  uint32_t readN(std::uint8_t* buf, size_t n);
  // Read exactly n bytes with an explicit extra timeout (ms).
  uint32_t readNTimeout(std::uint8_t* buf, size_t n, double extraMs);

  // Access to the underlying port (for protocol-level needs).
  SerialPort& port() { return *port_; }

 private:
  std::unique_ptr<SerialPort> port_;
  int baud_ = 10400;
  std::vector<std::uint8_t> buf_out_;
};

}  // namespace me7
