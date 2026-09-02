#pragma once

// Opaque C166 firmware blobs the tool uploads to the ECU over the K-line in
// boot mode. The original tool ships the ME7 boot-mode monitor/loader and
// EEPROM driver as compiled C166 code; we reuse those bytes as opaque data and
// bundle them so the port is self-contained.

#include <cstddef>
#include <cstdint>

namespace me7::firmware {

struct Blob {
  const std::uint8_t* data;
  std::size_t size;
  const char* name;      // blob identifier, for logging
};

// Boot-mode bootstrap uploaded to the ECU's C166 in two phases:
//   1. loader_core()  -> exactly 0x20 bytes, the C166 on-chip bootstrap loader
//      image (acked 0x01 by the running-once-it-loads stub);
//   2. monitor_core() -> the monitor (acked 0x03 once the loader hands off).
// Distinct buffers, NOT one blob sent twice.
Blob loader_core();
Blob monitor_core();

// EEPROM byte-trigger driver (0x35C-byte blob).
Blob eeprom_driver();

}  // namespace me7::firmware
