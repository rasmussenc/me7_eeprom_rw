// ME7 immobiliser EEPROM model -- implementation. See immo.h for the layout
// and the source of the checksum formula (the reference immo-3 EEPROM editor).

#include "core/immo.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace me7 {
namespace immo {

namespace {

// Pages that carry no checksum (header + wear region).
constexpr std::array kNoChecksum = {0x00u, 0x11u, 0x12u, 0x13u, 0x14u};
// Pages that are backup copies of the preceding page.
constexpr std::array kBackup = {0x08u, 0x0Au, 0x0Cu, 0x0Eu, 0x10u, 0x1Fu};

}  // namespace

bool isBackupPage(size_t pageno) {
  return std::find(kBackup.begin(), kBackup.end(), pageno) != kBackup.end();
}

bool isNoChecksumPage(size_t pageno) {
  return std::find(kNoChecksum.begin(), kNoChecksum.end(), pageno) !=
         kNoChecksum.end();
}

uint16_t pageChecksum(const uint8_t* data, size_t size, size_t pageno) {
  const size_t base = pageno * kPageSize;
  if (base + kPageSize > size) return 0xFFFF;  // truncated -- treat as mismatch
  const size_t minus = isBackupPage(pageno) ? 2 : 1;
  uint32_t bytesum = 0;
  for (size_t i = 0; i < 14; ++i) bytesum += data[base + i];
  // 0xFFFF - (pageno - minus) - bytesum, reduced to 16 bits.
  const uint32_t calc =
      (0xFFFFu - static_cast<uint32_t>(pageno - minus) - bytesum) & 0xFFFFu;
  return static_cast<uint16_t>(calc);
}

std::vector<size_t> invalidPages(const uint8_t* data, size_t size) {
  std::vector<size_t> bad;
  const size_t pages = size / kPageSize;
  for (size_t p = 0; p < pages; ++p) {
    if (isNoChecksumPage(p)) continue;
    const uint16_t calc = pageChecksum(data, size, p);
    const size_t base = p * kPageSize;
    const uint16_t saved = static_cast<uint16_t>(
        (data[base + 0x0F] << 8) | data[base + 0x0E]);
    if (calc != saved) bad.push_back(p);
  }
  return bad;
}

bool isImmoPatchPage(size_t pageno) {
  for (size_t p : kImmoPatchPages)
    if (p == pageno) return true;
  return false;
}

ImmoState immoState(const uint8_t* data, size_t size) {
  if (size < (kImmoPageIndex1 + 1) * kPageSize) return ImmoState::Error;
  const uint8_t a = data[kImmoPageIndex0 * kPageSize + kImmoByteIndex];
  const uint8_t b = data[kImmoPageIndex1 * kPageSize + kImmoByteIndex];
  if (a != b) return ImmoState::Error;
  if (a == kImmoOn) return ImmoState::On;
  if (a == kImmoOff) return ImmoState::Off;
  return ImmoState::Error;
}

std::vector<Delta> setImmo(uint8_t* data, size_t size, ImmoState target) {
  const uint8_t flag = (target == ImmoState::On) ? kImmoOn : kImmoOff;
  std::vector<Delta> deltas;
  const size_t pages[2] = {kImmoPageIndex0, kImmoPageIndex1};

  for (size_t p : pages) {
    const size_t base = p * kPageSize;
    if (base + kPageSize > size) return {};  // truncated: refuse silently
    if (data[base + kImmoByteIndex] != flag) {
      deltas.push_back({base + kImmoByteIndex, data[base + kImmoByteIndex], flag});
      data[base + kImmoByteIndex] = flag;
    }
    // Recompute this page's checksum (pages 1/2 are not backup pages -> minus 1).
    const uint16_t calc = pageChecksum(data, size, p);
    const uint8_t lo = static_cast<uint8_t>(calc & 0xFFu);
    const uint8_t hi = static_cast<uint8_t>((calc >> 8) & 0xFFu);
    if (data[base + 0x0E] != lo) {
      deltas.push_back({base + 0x0E, data[base + 0x0E], lo});
      data[base + 0x0E] = lo;
    }
    if (data[base + 0x0F] != hi) {
      deltas.push_back({base + 0x0F, data[base + 0x0F], hi});
      data[base + 0x0F] = hi;
    }
  }
  // Sort deltas by offset for a clean diff display.
  std::sort(deltas.begin(), deltas.end(),
            [](const Delta& x, const Delta& y) { return x.offset < y.offset; });
  return deltas;
}

}  // namespace immo
}  // namespace me7
