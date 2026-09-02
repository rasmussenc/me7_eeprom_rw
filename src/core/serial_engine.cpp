// Serial engine — portable reimplementation of the original ME7EEPROM v1.40
// Win32 serial layer.
//
// The timing constants below reproduce the original's byte/response timeout
// arithmetic (all values in ms):
//   bits per byte         = 10.0   (8 data + start + stop)
//   seconds -> milliseconds = 1000.0
//   small slack            = 1.0
//
// Timeout formulas (in ms):
//   one byte echo window  = (10.0/baud + ~0.0) * 1000.0        ~ 10000/baud
//   send deadline         = 10000.0/baud + 1.0 + now           ~ 10000/baud + 1
//   readN(xN)             = (10.0*xN/baud + ~0.0) * 1000.0     ~ 10000*xN/baud
//   readNwithTimeout(xN,e)= 1000.0*(e/1000.0 + 10.0*xN/baud)    = e + 10000*xN/baud

#include "core/serial_engine.h"
#include "core/timing.h"

#include <cstdio>
#include <cstring>

namespace me7 {

// Timing constants (see the formulas at the top of this file).
static constexpr float kA284 = 10.0f;
static constexpr float kA288 = 0.0f;    // negligible (~1.1e-23)
static constexpr float kA290 = 1000.0f;
static constexpr float kA294 = 10000.0f;
static constexpr float kA298 = 1.0f;
static constexpr float kA2A0 = 0.0f;    // negligible

// Read deadline for the K-line local echo after a byte (or buffer) is clocked out.
// The original reads the echo within one byte-time anchored at the clock sampled
// *before* the write (see writeByteEcho); on real hardware the echo is already
// buffered the instant the send completes, so it returns on the first poll. That
// window is far too tight for a software loopback (pty: another thread's wake-up
// latency). We re-anchor a small window at send completion and give it a margin
// that comfortably covers loopback jitter while staying negligible next to the
// 100-500 ms command-response timeouts. This is a *read* deadline for our own
// echo, NOT wire pacing — inter-byte/inter-frame delays are unchanged.
static constexpr double kEchoWindowMs = 25.0;

// Read-latency accommodation for readN(). The original computes a read deadline
// of one byte-time * n (e.g. ~1 ms for a single byte, ~492 ms for 512 bytes at
// 10400 baud) and relies on the bytes ALREADY being in the host RX buffer when
// readN is called -- which holds for the low-latency direct/K-line COM ports the
// tool was written for, where the echo read that precedes a response spans the
// response's arrival so it is buffered by the time readN runs. USB-serial
// adapters (FTDI etc.) add a "latency timer" (commonly 16 ms) that batches
// received chars before delivering them, so:
//   - a single response byte can land 0-16 ms AFTER the preceding echo was
//     consumed, far outside the original's ~1 ms window (uC-ID read back as 0x00
//     / "No ECU response" intermittently), and
//   - the LAST chunk of a multi-byte read arrives up to one latency-timer AFTER
//     the final wire byte, so a 512-byte read needs ~492 ms of wire time PLUS
//     ~16 ms of tail delivery, but the original's 492 ms window has no margin --
//     the read times out just short of the last chunk (the post-write verify
//     "Read timeout 0x20F07" is exactly this: a 512-byte read right after the
//     32-page program loop, where the first byte is also delayed by driver
//     re-arm, leaving no slack at all).
// We both floor small reads (so a 1-byte response clears the latency timer) and
// add this margin ON TOP of the byte-time for every read, so the first byte's
// arrival delay and the last chunk's tail delivery both fit inside the window.
// This is a host-side read-latency accommodation only -- wire pacing and
// inter-byte timing are unchanged. readNTimeout() takes an explicit extra-timeout
// from its caller (typically 100-6000 ms), so it already has generous margin and
// does not need this.
static constexpr double kReadFloorMs = 30.0;
static constexpr double kReadLatencyMs = 40.0;  // > 2x a 16 ms FTDI latency timer

int SerialEngine::normalizeBaud(int baud) {
  switch (baud) {
    case 9600: return 0x2580;
    case 10400: return 0x28a0;
    case 19200: return 0x4b00;
    case 57600: return 0xe100;
    default: return -1;
  }
}

uint32_t SerialEngine::open(int comport, int baud, std::string* errMsg) {
  const int dcbaud = normalizeBaud(baud);
  if (dcbaud < 0) {
    if (errMsg) {
      char b[64];
      std::snprintf(b, sizeof b, "Error: ilegal baudrate argument `%d'", baud);
      *errMsg = b;
    }
    return kErrIlegalBaud;
  }
  baud_ = baud;

  // Port must be in 1..16 (param_1 - 1U < 0xf).
  if (comport < 1 || comport > 16) {
    if (errMsg) {
      char b[64];
      std::snprintf(b, sizeof b, "Error: ilegal comport argument `%d'", comport);
      *errMsg = b;
    }
    return kErrIlegalComport;
  }

  char dcbbuf[64];
  std::snprintf(dcbbuf, sizeof dcbbuf, "baud=%d data=8 parity=N stop=1", dcbaud);
  char dev[32];
  std::snprintf(dev, sizeof dev, "\\\\.\\COM%d", comport);

  try {
    SerialParams p;
    p.baudrate = baud;  // host backend wants the lifted baud, not the DCB value
    p.dataBits = 8;
    p.parity = 'N';
    p.stopBits = 1;
    port_->open(dev, p);
  } catch (SerialError&) {
    if (errMsg) *errMsg = "Unable to open comport";
    return kErrOpen;
  }
  // libserialport/stub backends handle DCB + timeouts internally; there is no
  // separable failure to report, so open() is the single gate.
  if (errMsg) errMsg->clear();

  // The original ran on Windows, where opening a COM port (or the subsequent
  // BuildCommDCB/SetCommState sequence) effectively starts the driver with a
  // fresh RX buffer. The Unix backends do NOT discard buffered input on open,
  // so bytes left over from a prior run -- or K-line framing-error zeros that
  // accumulated while the ECU was being strapped -- survive in the FTDI FIFO
  // and are read first on the next attempt, poisoning the 1-byte BSL uC-ID
  // read with 0x00. Purge both buffers here to match the Windows behavior the
  // original implicitly relied on.
  flush();
  return kErrNone;
}

void SerialEngine::close() { port_->close(); }

uint32_t SerialEngine::flush() {
  try {
    port_->purge();
  } catch (...) {
    return kErrPurge;
  }
  return kErrNone;
}

uint32_t SerialEngine::setBreak(bool on) {
  try {
    port_->setBreak(on);
  } catch (...) {
    return on ? kErrSetBreak : kErrClearBreak;
  }
  return kErrNone;
}

uint32_t SerialEngine::writeByteEcho(std::uint8_t b) {
  // Transmit one byte, wait for it to clock out, then read back the K-line local
  // echo and verify it matches. Byte time = 10 bits / baud (start + 8 data +
  // stop); send deadline = byte time + 1 ms slack.
  std::uint8_t buf[1] = {b};
  port_->write(buf, 1);
  const double start = nowMs();
  const double sendDeadline = kA294 / baud_ + kA298 + start;      // ~10000/baud + 1 ms
  while (nowMs() < sendDeadline) sleepMs(1);

  // Read the echo. The original anchored its one-byte-time window at the clock
  // sampled before the write, which expired during the send spin (so the read
  // never ran and every byte reported "no echo"). Re-anchor at send completion;
  // real adapters echo within a byte-time, the margin only guards loopback latency.
  const double echoDeadline = nowMs() + kEchoWindowMs;
  while (nowMs() < echoDeadline) {
    std::uint8_t in = 0;
    size_t got = port_->read(&in, 1, 0);
    if (got) {
      if (in != b) return kErrEcho;
      return kErrNone;
    }
    sleepMs(1);
  }
  return kErrNoEcho;
}

uint32_t SerialEngine::writeBufferEcho(const std::uint8_t* buf, size_t n) {
  // Transmit n bytes, wait for the whole buffer to clock out, then read back the
  // n-byte local echo and compare it (byte-time*n for the send, echoed
  // one-byte-time*n window re-anchored at send completion, see above).
  port_->write(buf, n);  // write all n bytes; the backend throws on failure
  buf_out_.resize(n);
  const double start = nowMs();
  const double sendDone = (kA284 * static_cast<double>(n) / baud_ + kA288) *
                              kA290 + start;  // ~10000*n/baud ms
  while (nowMs() < sendDone) sleepMs(1);

  const double echoDeadline =
      nowMs() + (kA284 * static_cast<double>(n) / baud_) * kA290 + kEchoWindowMs;
  size_t got = 0;
  while (got < n) {
    if (nowMs() >= echoDeadline) return kErrWriteTimeout;
    size_t r = port_->read(buf_out_.data() + got, n - got, 0);
    got += r;
    if (r == 0) sleepMs(1);
  }
  if (std::memcmp(buf_out_.data(), buf, n) != 0) return kErrBufMismatch;
  return kErrNone;
}

uint32_t SerialEngine::readN(std::uint8_t* buf, size_t n) {
  const double start = nowMs();
  const double byteTime = (kA284 * static_cast<double>(n) / baud_ + kA2A0) * kA290;
  // Window = byte-time + adapter-latency margin, floored for small reads. The
  // margin covers both the first byte's delivery delay (driver re-arm + FTDI
  // latency timer) and the last chunk's tail delivery, which the original's
  // pure byte-time window lacks on USB-serial adapters. See kReadLatencyMs.
  const double window = byteTime + kReadLatencyMs;
  const double deadline = start + (window > kReadFloorMs ? window : kReadFloorMs);
  size_t got = 0;
  while (got < n) {
    if (nowMs() >= deadline) return kErrReadTimeout;  // 0x20f07
    size_t r = port_->read(buf + got, n - got, 0);
    got += r;
    if (r == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return kErrNone;
}

uint32_t SerialEngine::readNTimeout(std::uint8_t* buf, size_t n, double extraMs) {
  const double start = nowMs();
  const double deadline =
      kA290 * (extraMs / kA290 + kA284 * static_cast<double>(n) / baud_) + start;
  size_t got = 0;
  while (got < n) {
    if (nowMs() >= deadline) return kErrReadTimeout2;  // 0x21007
    size_t r = port_->read(buf + got, n - got, 0);
    got += r;
    if (r == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return kErrNone;
}

}  // namespace me7
