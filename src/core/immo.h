// ME7 immobiliser EEPROM model: 512-byte page layout, per-page checksum, and
// the surgical immobiliser flag patch.
//
// This is a C++17 reimplementation of the layout/checksum/patch logic from a
// known ME7 immo-3 EEPROM editor, kept faithful to that tool's field offsets
// and checksum formula. It does NOT reproduce the original ME7EEPROM.exe
// binary (which is a generic block reader/writer with no immo concept) --
// this module is an added convenience layer for building immobiliser-off
// images from an OBD read.
//
// EEPROM layout (immo-3, 512 bytes = 32 pages x 16 bytes):
//   page 0x00          : header  (no checksum)
//   page 0x01, 0x02    : immobiliser flag at byte[2]: 0x01 = ON, 0x02 = OFF
//                        (both pages must agree)
//   page 0x08/0x0A/0x0C/0x0E/0x10/0x1F : backup copies of the preceding page
//   pages 0x11..0x14   : wear region (no checksum)
//   bytes 0x0E, 0x0F of each checksummed page : low / high byte of
//       calc = 0xFFFF - (pageno - minus) - sum(bytes[0..13])
//       where minus = 2 for backup pages, else 1.

#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace me7 {
namespace immo {

inline constexpr size_t kPageSize = 16;

// Immobiliser flag values at page[1][2] / page[2][2].
inline constexpr uint8_t kImmoOn = 0x01;
inline constexpr uint8_t kImmoOff = 0x02;
inline constexpr size_t kImmoPageIndex0 = 0x01;
inline constexpr size_t kImmoPageIndex1 = 0x02;
inline constexpr size_t kImmoByteIndex = 2;

// Compute the 16-bit checksum a page *should* carry, given its number and
// whether it is a backup page (backup pages use (pageno - 2) as the page term).
uint16_t pageChecksum(const uint8_t* data, size_t size, size_t pageno);

// True if `pageno` is a backup copy page (must mirror page pageno-1).
bool isBackupPage(size_t pageno);
// True if `pageno` carries no checksum (header / wear region).
bool isNoChecksumPage(size_t pageno);

// Returns the numbers of all checksummed pages whose stored checksum is wrong.
// (Pages in the no-checksum set are skipped; an all-0xFF erased page that is
// *expected* to be structured -- e.g. an uncoded SKC page -- will show up here,
// which is normal for an ECU that never had that field coded.)
std::vector<size_t> invalidPages(const uint8_t* data, size_t size);

// The pages the surgical immo patch actually modifies (the immo-flag pages).
// invalidPages restricted to these is the integrity gate the patch needs: if a
// touched page's checksum is already bad, the image is corrupt in the exact
// region we'd edit and we must not patch. Pages outside this set may legitimately
// be invalid (e.g. an erased/uncoded SKC page) since the patch never touches them.
inline constexpr size_t kImmoPatchPages[] = {0x01, 0x02};
bool isImmoPatchPage(size_t pageno);

enum class ImmoState { On, Off, Error };
// Reads the flag at page[1][2] and page[2][2]; Error if they disagree or hold
// a value other than 0x01/0x02.
ImmoState immoState(const uint8_t* data, size_t size);

// A per-byte change produced by a patch.
struct Delta {
  size_t offset;
  uint8_t oldVal;
  uint8_t newVal;
};

// Surgically set the immobiliser flag to `target` (On/Off): flips byte[2] of
// pages 1 and 2 and recomputes ONLY those two pages' checksums. Leaves every
// other page (backup pages, erased SKC pages, wear region) byte-for-byte
// unchanged. Returns the offsets changed so the caller can show the diff.
// If the two immo pages already hold `target` and are self-consistent, no
// change is made (empty deltas). Throws nothing; caller checks beforehand.
std::vector<Delta> setImmo(uint8_t* data, size_t size, ImmoState target);

}  // namespace immo
}  // namespace me7
