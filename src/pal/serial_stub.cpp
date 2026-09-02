// Stub serial backend used when libserialport is unavailable at build time.
// Lets the project compile and the CLI run on Linux/macOS before the real
// serial layer is wired up. Throws on every open.

#include "pal/serial_pal.h"

#include <cstdio>

namespace me7 {

class StubSerial : public SerialPort {
 public:
  void open(const std::string& device, const SerialParams& params) override {
    std::fprintf(stderr,
                 "me7eeprom: serial backend not built (no libserialport). "
                 "Cannot open '%s'.\n"
                 "    Install libserialport:  brew install libserialport\n",
                 device.c_str());
    throw SerialError("serial backend unavailable");
  }
  void close() override {}
  bool isOpen() const override { return false; }
  size_t read(std::uint8_t*, size_t, int) override { return 0; }
  void write(const std::uint8_t*, size_t) override {}
};

std::unique_ptr<SerialPort> makeSerialPort() {
  return std::make_unique<StubSerial>();
}

}  // namespace me7
