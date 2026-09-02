#pragma once

// Cross-platform serial-port abstraction.
//
// The original ME7EEPROM tool drives a K-line serial adapter straight through
// the Win32 API (CreateFileA("\\\\.\\COM%d"), BuildCommDCBA, SetCommState,
// SetCommTimeouts, PurgeComm, SetCommBreak, ReadFile/WriteFile). To run on
// Linux and macOS this layer swaps that for an abstract SerialPort interface,
// implemented by per-platform backends (src/pal/serial_posix.cpp by default,
// src/pal/serial_libserialport.cpp for a true 10400 baud divisor). The rest of
// the program only ever talks to this interface, so the K-line protocol core
// stays identical to the original.
//
// Backends may throw SerialError on hard failures; the caller is responsible
// for mapping that to the tool's error messages.

#include <cstdint>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

namespace me7 {

struct SerialError : std::runtime_error {
  explicit SerialError(const std::string& m) : std::runtime_error(m) {}
};

// Settings map 1:1 onto the original tool's BuildCommDCBA string
// ("baud=%d data=8 parity=N stop=1") plus the CS pin control.
struct SerialParams {
  int baudrate = 10400;   // 9600 / 10400 / 19200 / 57600
  int dataBits = 8;
  char parity  = 'N';     // 'N', 'E', 'O'
  int stopBits = 1;
};

class SerialPort {
 public:
  virtual ~SerialPort() = default;

  virtual void open(const std::string& device, const SerialParams& params) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  // Read up to count bytes. Returns bytes read (0 on timeout/no data).
  virtual size_t read(std::uint8_t* buf, size_t count, int timeoutMs) = 0;
  // Write all bytes; throws SerialError on failure.
  virtual void write(const std::uint8_t* buf, size_t count) = 0;

  // Set/clear the UART break condition (used by the original's SetCommBreak to
  // implement the K-line 5-baud wake-up). Optional; default no-op.
  virtual void setBreak(bool) {}

  // Drop pending input/output buffers (PurgeComm).
  virtual void purge() {}
};

// Returns a backend-specific SerialPort instance. Defined per-backend; each
// backend lives in its own .cpp under src/pal/.
std::unique_ptr<SerialPort> makeSerialPort();

}  // namespace me7
