// Native POSIX termios serial backend for Linux/macOS.
//
// This replaces libserialport as the default backend. libserialport refuses to
// open a pseudo-terminal (sp_open runs modem-control ioctls that return ENOTTY
// on a pty), which blocks the loopback validation harness. Plain termios opens a
// pty *and* a real /dev/cu.* / /dev/ttyUSB* adapter equally well, so the protocol
// core can be exercised end-to-end on a pty sim and against standard-baud hardware.
//
// The interface maps 1:1 onto the original tool's BuildCommDCBA + SetCommTimeouts
// + ReadFile/WriteFile/SetCommBreak/PurgeComm:
//   - ReadIntervalTimeout = MAXDWORD + 0/0 (SetCommTimeouts) = ReadFile returns
//     immediately with whatever bytes are buffered, so read() polls with
//     timeoutMs=0 and returns available bytes (0 if none).
//   - SetCommBreak/ClearCommBreak (the 5-baud wake-up) -> TIOCSBRK/TIOCCBRK
//     (real UART break on hardware; no-op on a pty where the sim time-injects the
//     init response instead).
//
// Note on baud 10400: it is non-standard and not representable in termios speed_t
// constants, so on a pty (where baud is ignored) and for standard-baud hardware
// this backend maps it to B9600. For a real K-line adapter at a true 10400 baud,
// build with -DME7_USE_LIBSERIALPORT=ON to use the libserialport backend, which
// supports the custom divisor.

#include "pal/serial_pal.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace me7 {

namespace {

speed_t baudToSpeed(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 57600:  return B57600;
    case 10400:  return B9600;   // not representable in termios; harmless on a pty
    default:     return B9600;
  }
}

class PosixSerial : public SerialPort {
 public:
  ~PosixSerial() override { close(); }

  void open(const std::string& device, const SerialParams& params) override {
    close();

    // ME7_DEVICE points the original tool's Win32 "COM<n>" contract at a real node.
    std::string path = device;
    if (const char* env = std::getenv("ME7_DEVICE"); env && *env) path = env;

    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
      throw SerialError(std::string("Unable to open comport: ") +
                        std::strerror(errno));

    struct termios t{};
    if (tcgetattr(fd_, &t) != 0) {
      int e = errno; ::close(fd_); fd_ = -1;
      throw SerialError(std::string("Unable to set comport cfg settings: ") +
                        std::strerror(e));
    }
    cfmakeraw(&t);
    t.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    t.c_iflag = 0;       // no input processing (matches BuildCommDCB parity=N)
    t.c_oflag = 0;
    t.c_lflag = 0;       // raw; no ECHO (so tool writes don't self-echo in-kernel)
    const speed_t sp = baudToSpeed(params.baudrate);
    cfsetispeed(&t, sp);
    cfsetospeed(&t, sp);
    // VMIN=0/VTIME=0: non-blocking reads; read() below gates on poll() instead.
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &t) != 0) {
      int e = errno; ::close(fd_); fd_ = -1;
      throw SerialError(std::string("Unable to set comport cfg settings: ") +
                        std::strerror(e));
    }
    tcflush(fd_, TCIOFLUSH);
  }

  void close() override {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  bool isOpen() const override { return fd_ >= 0; }

  size_t read(std::uint8_t* buf, size_t count, int timeoutMs) override {
    if (fd_ < 0 || count == 0) return 0;
    struct pollfd pfd{fd_, POLLIN, 0};
    int pr = ::poll(&pfd, 1, timeoutMs);
    if (pr <= 0) return 0;            // timeout or error -> no data
    ssize_t n = ::read(fd_, buf, count);
    return n > 0 ? static_cast<size_t>(n) : 0;
  }

  void write(const std::uint8_t* buf, size_t count) override {
    if (fd_ < 0) throw SerialError("port not open");
    size_t done = 0;
    while (done < count) {
      ssize_t n = ::write(fd_, buf + done, count - done);
      if (n < 0) {
        if (errno == EINTR || errno == EAGAIN) continue;
        throw SerialError(std::string("WriteFile failed: ") +
                          std::strerror(errno));
      }
      done += static_cast<size_t>(n);
    }
  }

  void setBreak(bool on) override {
    if (fd_ < 0) return;
    // SetCommBreak / ClearCommBreak. On hardware these drive the UART break
    // line (used by the 5-baud fast-init bit-bang). On a pty these ioctls are
    // no-ops.
    if (on) ::ioctl(fd_, TIOCSBRK, 0);
    else    ::ioctl(fd_, TIOCCBRK, 0);
  }

  void purge() override {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
  }

 private:
  int fd_ = -1;
};

}  // namespace

std::unique_ptr<SerialPort> makeSerialPort() {
  return std::make_unique<PosixSerial>();
}

}  // namespace me7
