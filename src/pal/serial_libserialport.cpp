// libserialport backend for Linux/macOS.
//
// Translates the abstract SerialPort operations onto libserialport, providing
// the same behavior the original tool got from the Win32 serial API.
// libserialport exposes COM-like ports as /dev/tty* on Unix and hides platform
// differences, so the core protocol code is untouched.
//
// NOTE: break control (the original's SetCommBreak/ClearCommBreak, used for the
// K-line 5-baud fast-init wake-up) is provided via libserialport's
// sp_start_break/sp_end_break. The POSIX termios backend instead uses
// TIOCSBRK/TIOCCBRK; either drives a real UART break on supported hardware and
// is a no-op where the driver doesn't (e.g. some USB-serial chips).

#include "pal/serial_pal.h"

#include <libserialport.h>

#include <cstdlib>
#include <thread>
#include <chrono>

namespace me7 {

namespace {

// Maps a libserialport return code onto a SerialError, preserving an
// "error=0x%02X"-style message like the original.
void check(sp_return r, const char* op) {
  if (r != SP_OK) {
    const char* detail = sp_last_error_message();
    throw SerialError(std::string(op) + " failed" +
                      (detail ? std::string(": ") + detail : std::string()));
  }
}

enum sp_parity toParity(char p) {
  if (p == 'E') return SP_PARITY_EVEN;
  if (p == 'O') return SP_PARITY_ODD;
  return SP_PARITY_NONE;
}

int toStopBits(int s) {
  // libserialport 0.1.x sp_set_stopbits() takes the number of stop bits (1/2),
  // not the SP_STOPBITS_* enum from newer API generations.
  return s == 2 ? 2 : 1;
}

}  // namespace

class LibserialportSerial : public SerialPort {
 public:
  ~LibserialportSerial() override { close(); }

  void open(const std::string& device, const SerialParams& params) override {
    close();

    // The core always hands us the original tool's Win32-style name "\\.\COM%d".
    // On Linux/macOS that maps to a real /dev/tty* node, which the original
    // never had to choose. Allow an explicit override via ME7_DEVICE so the
    // K-line COM%d CLI contract is preserved while the user points at their
    // adapter (e.g. ME7_DEVICE=/dev/cu.usbserial-1420 ./me7eeprom -p1 ...).
    std::string path = device;
    if (const char* env = std::getenv("ME7_DEVICE"); env && *env) path = env;

    struct sp_port* p = nullptr;
    if (sp_get_port_by_name(path.c_str(), &p) != SP_OK || !p)
      throw SerialError("Unable to open port: no such port");
    port_ = p;

    check(sp_open(port_, SP_MODE_READ_WRITE), "Unable to open comport");
    check(sp_set_baudrate(port_, params.baudrate), "SetCommState");
    check(sp_set_bits(port_, params.dataBits), "SetCommState");
    check(sp_set_parity(port_, toParity(params.parity)), "SetCommState");
    check(sp_set_stopbits(port_, toStopBits(params.stopBits)), "SetCommState");
    check(sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE), "SetCommState");
  }

  void close() override {
    if (port_) {
      sp_close(port_);
      sp_free_port(port_);
      port_ = nullptr;
    }
  }

  bool isOpen() const override { return port_ != nullptr; }

  size_t read(std::uint8_t* buf, size_t count, int timeoutMs) override {
    if (!port_) return 0;
    // NOTE: in libserialport, a timeout of 0 means "block indefinitely" (NOT
    // non-blocking, unlike POSIX poll(,0)). The SerialEngine drives its own
    // bounded loop with nowMs()>=deadline and passes timeoutMs=0 expecting an
    // immediate "no data" return. Translate that intent: a 0 ms engine request
    // becomes a short polling read so we never block forever on a silent ECU.
    int eff = (timeoutMs <= 0) ? 1 : timeoutMs;
    int r = sp_blocking_read(port_, buf, count, eff);
    return r > 0 ? static_cast<size_t>(r) : 0;
  }

  void write(const std::uint8_t* buf, size_t count) override {
    if (!port_) throw SerialError("port not open");
    size_t done = 0;
    while (done < count) {
      int r = sp_blocking_write(port_, buf + done, count - done, 1000);
      if (r < 0) {
        const char* detail = sp_last_error_message();
        throw SerialError(std::string("WriteFile failed") +
                          (detail ? std::string(": ") + detail : std::string()));
      }
      if (r == 0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
      else done += static_cast<size_t>(r);
    }
  }

  void purge() override {
    if (port_) sp_flush(port_, SP_BUF_BOTH);
  }

  // SetCommBreak / ClearCommBreak — drives the K-line 5-baud wake-up on hardware.
  void setBreak(bool on) override {
    if (!port_) return;
    check(on ? sp_start_break(port_) : sp_end_break(port_),
          on ? "SetCommBreak" : "ClearCommBreak");
  }

 private:
  struct sp_port* port_ = nullptr;
};

std::unique_ptr<SerialPort> makeSerialPort() {
  return std::make_unique<LibserialportSerial>();
}

}  // namespace me7
