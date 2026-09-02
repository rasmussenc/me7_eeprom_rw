// ME7_EEPROM — cross-platform port, faithful to the original ME7EEPROM v1.40
// tool's main() flow.
//
// The CLI option model is implemented (and tested) in src/core/options.cpp;
// this file is the dispatch layer that reproduces the original's per-mode flow:
//   - informational header prints (COM / memory type / CS pin),
//   - opening the COM port,
//   - OBD mode: init + read (read-only, save/screen),
//   - bootmode: BootUploader::run() boot sequence, then read / write / screen,
//   - file write via a faithful port of the EEPROM file writer.
//
// No hardware is touched at parse time; serial access is only attempted after
// the user requests a real operation.

#include "core/eeprom.h"
#include "core/immo.h"
#include "core/options.h"
#include "core/protocol.h"
#include "core/serial_engine.h"
#include "core/uploader.h"
#include "pal/serial_pal.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// Status code -> human message table.
//
// The original builds a per-layer message into its banner and main() prints
// "<msg>. (error=0x%02X)". Most protocol/upload layers only ever set the
// banner to "FAIL", so that is the fallback. The strings below come from the
// serial layer and the general error banners; where a code is documented but
// has no recoverable string we fall through to the generic "FAIL" line.
// ---------------------------------------------------------------------------
const char* statusMessage(uint32_t code) {
  using namespace me7;
  switch (code) {
    case kErrIlegalBaud:   return "ilegal baudrate";
    case kErrIlegalComport:return "ilegal comport";
    case kErrOpen:         return "Unable to open comport";
    case kErrDcb:          return "Unable to set comport DCB settings";
    case kErrCfg:          return "Unable to set comport cfg settings";
    case kErrTimeouts:     return "Unable to set time-out settings";
    case kErrPurge:        return "PurgeComm failed";
    case kErrSetBreak:     return "SetCommBreak failed";
    case kErrClearBreak:   return "ClearCommBreak failed";
    case kErrWrite:        return "WriteFile failed";
    case kErrEcho:         return "Echo mismatch";
    case kErrNoEcho:       return "Echo timeout";
    case kErrWriteCount:   return "Write buffer count mismatch";
    case kErrWriteTimeout: return "Write buffer echo timeout";
    case kErrBufMismatch:  return "Buffer echo mismatch";
    case kErrReadTimeout:  return "Read timeout";
    case kErrReadTimeout2: return "Read timeout";
    default:               return "FAIL";
  }
}

// Reproduces main's error exit: prints the banner text followed by
// "(error=0x%02X)". Never returns.
int reportError(uint32_t code) {
  std::printf("%s. (error=0x%02X)\n", statusMessage(code), code);
  return EXIT_FAILURE;
}

// ---------------------------------------------------------------------------
// Read a whole EEPROM-sized file for a bootmode write.
// ---------------------------------------------------------------------------
uint32_t readEepromFile(const std::string& name, size_t size,
                        std::vector<uint8_t>& data) {
  std::FILE* f = std::fopen(name.c_str(), "rb");
  if (!f) {
    std::printf("Unable to open file `%s'\n", name.c_str());
    return 0x30004;
  }
  data.assign(size, 0);
  const size_t got = std::fread(data.data(), 1, size, f);
  const int c = std::fgetc(f);
  const bool eof = (c == EOF) && (std::feof(f) != 0);
  std::fclose(f);
  if (got == size && eof) return 0;
  std::printf("Invalid file size: `%s'\n", name.c_str());
  return 0x3010c;
}

// ---------------------------------------------------------------------------
// Write the EEPROM image to a file. Reproduced exactly (return codes 0,
// 0x30204, 0x3030d, 0x30402 with the same messages).
// ---------------------------------------------------------------------------
uint32_t writeEepromFile(const std::string& name, const uint8_t* buf,
                         size_t size) {
  std::FILE* f = std::fopen(name.c_str(), "wb");
  if (!f) {
    std::printf("Unable to open file `%s'\n", name.c_str());
    return 0x30204;
  }
  const size_t w = std::fwrite(buf, 1, size, f);
  if (w != size) {
    std::printf("Error writing to file `%s'\n", name.c_str());
    std::fclose(f);
    return 0x3030d;
  }
  if (std::fclose(f) != 0) {
    std::printf("File close failed: Inconsistent write\n");
    return 0x30402;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Common "display to screen" block (the screen action).
// ---------------------------------------------------------------------------
void printScreen(const std::vector<uint8_t>& data) {
  std::puts("Printing to screen");
  std::printf("\nOffset  0  1  2  3  4  5  6  7   8  9  A  B  C  D  E  F");
  for (size_t i = 0; i < data.size(); ++i) {
    if ((i & 7) == 0) std::putchar(' ');
    if ((i & 0xf) == 0) std::printf("\n%4X   ", static_cast<unsigned>(i));
    std::printf("%02X ", data[i]);
  }
  std::puts("\n");
}

// Open the COM port and, on success, print the matching banner line.
// SerialEngine::open returns the error text in errMsg on failure.
std::unique_ptr<me7::SerialEngine> openSerial(const me7::Options& opts) {
  std::unique_ptr<me7::SerialPort> port;
  try {
    port = me7::makeSerialPort();
  } catch (const me7::SerialError& e) {
    std::printf("%s. (error=0x%02X)\n", e.what(), 0u);
    std::exit(EXIT_FAILURE);
  }

  auto eng = std::make_unique<me7::SerialEngine>(std::move(port));
  std::printf("Opening COM%d ... ", opts.comport);
  std::string errMsg;
  const uint32_t st = eng->open(opts.comport, opts.baudrate, &errMsg);
  if (st != 0) {
    // The layer already left a human string; use it (or "FAIL").
    std::printf("%s. (error=0x%02X)\n",
                errMsg.empty() ? statusMessage(st) : errMsg.c_str(), st);
    std::exit(EXIT_FAILURE);
  }
  std::puts("OK");
  return eng;
}

// ---------------------------------------------------------------------------
// OBD mode: init (fast init + KWP) then read. Read-only, as the CLI already
// rejects write-over-OBD. (The tool prints "Initiating communication ... " ...)
//
// With --immo the read is run twice and the two captures compared, the
// per-page immo checksums are verified, and only if both pass is an
// immobiliser-off image produced (behind a y/N prompt). No ECU write is ever
// performed here. See src/core/immo.h for the layout/checksum source.
// ---------------------------------------------------------------------------
int runOBD(me7::SerialEngine& eng, const me7::Options& opts) {
  me7::Protocol proto(eng);
  proto.setEepromSize(0x200);  // EEPROM size for OBD

  std::printf("Initiating communication ... ");
  uint32_t st = proto.obdInit();
  if (st != 0) return reportError(st);

  std::printf("OK\nECU ID response: ");
  const std::vector<std::string>& ids = proto.ecuIds();
  for (size_t i = 0; i < ids.size(); ++i)
    std::printf(i == 0 ? "%s\n" : "                 %s\n", ids[i].c_str());

  // ---- read -------------------------------------------------------------
  std::printf("Reading EEPROM ");
  std::vector<uint8_t> data;
  st = proto.obdRead(data);
  if (st != 0) return reportError(st);
  std::puts(" OK");

  // ---- verification pass (only when building an immo image) -------------
  bool checksOk = true;
  if (opts.immo) {
    // Second read + compare: guards against a flaky K-line corrupting one pass.
    std::printf("Re-reading EEPROM for verification ... ");
    std::vector<uint8_t> data2;
    st = proto.obdRead(data2);
    if (st != 0) return reportError(st);
    std::puts("OK");
    if (data != data2) {
      std::puts("Two reads differ -- EEPROM read is inconsistent; not patching.");
      checksOk = false;
    } else {
      std::puts("Two reads match.");
    }

    if (checksOk) {
      const auto bad = me7::immo::invalidPages(data.data(), data.size());
      // Split the invalid pages into the ones the immo patch touches vs the
      // rest. The patch only ever modifies the immo-flag pages (pages 1/2), so
      // those must be valid -- a bad checksum there means the region we'd edit
      // is corrupt and we must not patch. Other pages (e.g. an erased/uncoded
      // SKC page 3/4) may legitimately be invalid since we never touch them;
      // those are reported as a warning but do not block the patch.
      std::vector<size_t> badTouched, badOther;
      for (size_t p : bad) {
        if (me7::immo::isImmoPatchPage(p)) badTouched.push_back(p);
        else badOther.push_back(p);
      }
      if (!badOther.empty()) {
        std::printf("WARNING: per-page checksum invalid on non-patched pages ");
        for (size_t i = 0; i < badOther.size(); ++i)
          std::printf(i == 0 ? "0x%02zx" : ", 0x%02zx", badOther[i]);
        std::puts(" -- these are left byte-for-byte as read (commonly an erased");
        std::puts("uncoded SKC page); the immo patch does not touch them.");
      }
      if (!badTouched.empty()) {
        std::printf("Per-page checksum invalid on immo pages ");
        for (size_t i = 0; i < badTouched.size(); ++i)
          std::printf(i == 0 ? "0x%02zx" : ", 0x%02zx", badTouched[i]);
        std::puts(" -- the immobiliser flag pages are corrupt; not patching.");
        checksOk = false;
      } else {
        std::puts("Immo-flag pages (1/2) checksums valid.");
      }
    }
  }

  if (opts.action == me7::Action::Screen) {
    printScreen(data);
    return EXIT_SUCCESS;
  }
  // Read: save to file (always -- so a baseline is preserved even if checks fail).
  if (opts.fileName.empty()) {           // defensive: parser normally requires it
    std::printf("FAIL. (error=0x%02X)\n", 0x30204u);
    return EXIT_FAILURE;
  }
  st = writeEepromFile(opts.fileName, data.data(), data.size());
  if (st != 0) {
    std::printf("FAIL. (error=0x%02X)\n", st);
    return EXIT_FAILURE;
  }
  std::puts("File saved");

  // ---- immo-off image generation (checks must have passed) --------------
  if (opts.immo) {
    if (!checksOk) {
      std::puts("Verification failed -- raw read saved above; no immo image built.");
      return EXIT_FAILURE;
    }
    me7::immo::ImmoState state =
        me7::immo::immoState(data.data(), data.size());
    if (state == me7::immo::ImmoState::Error) {
      std::puts("Immobiliser flag is inconsistent (pages 1/2 disagree or hold an");
      std::puts("unexpected value) -- not patching. Inspect the dump first.");
      return EXIT_FAILURE;
    }
    std::printf("Immobiliser: %s\n",
                state == me7::immo::ImmoState::On ? "ON" : "OFF");
    if (state == me7::immo::ImmoState::Off) {
      std::puts("Already immo-off; no image built.");
      return EXIT_SUCCESS;
    }

    // Build the patched image and show the surgical diff.
    std::vector<uint8_t> patched = data;
    const auto deltas = me7::immo::setImmo(
        patched.data(), patched.size(), me7::immo::ImmoState::Off);
    std::puts("Immo-off patch:");
    for (const auto& d : deltas)
      std::printf("  0x%04zx: 0x%02x -> 0x%02x\n", d.offset, d.oldVal, d.newVal);

    // y/N confirmation before writing the immo-off file.
    std::printf("Write immo-off image to %s.immooff.bin? [y/N] ",
                opts.fileName.c_str());
    std::cout.flush();
    std::string resp;
    std::getline(std::cin, resp);
    if (resp != "y" && resp != "Y") {
      std::puts("Aborted -- no immo-off image written.");
      return EXIT_SUCCESS;
    }
    const std::string outName = opts.fileName + ".immooff.bin";
    st = writeEepromFile(outName, patched.data(), patched.size());
    if (st != 0) {
      std::printf("FAIL. (error=0x%02X)\n", st);
      return EXIT_FAILURE;
    }
    std::printf("Immo-off image written to %s\n", outName.c_str());
    std::puts("Write back to the ECU later with (boot mode, separate entry):");
    std::printf("  me7eeprom --bootmode <memtype> -w %s\n", outName.c_str());
  }
  return EXIT_SUCCESS;
}

// ---------------------------------------------------------------------------
// Bootmode: BootUploader::run() boots the ECU, then dispatch read / write /
// screen and save. Reproduces the original main() exactly, including the
// 95160 -> 95080 quirk (applied inside BootUploader::run; here we use the
// resulting effectiveMemType() as the driver's memory-index argument while the
// buffer/file sizing uses the raw index, mirroring the original's two
// memory-type values).
// ---------------------------------------------------------------------------
int runBootmode(me7::SerialEngine& eng, const me7::Options& opts,
                size_t configSize, const std::vector<uint8_t>& fileData) {
  me7::Protocol proto(eng);
  me7::BootUploader uploader(eng, proto);

  // The uploader streams the boot-mode transcript live (mirroring the original's
  // per-stage printf/puts), so there is no buffered banner to print afterwards.
  uint32_t st = uploader.run(opts);
  if (st != 0) return reportError(st);

  const int driverIndex = uploader.effectiveMemType();  // quirked (95160 -> 1)

  if (opts.action == me7::Action::Write) {
    std::printf("Writing EEPROM ");
    st = me7::bootWriteEEPROM(proto, eng,
                              static_cast<size_t>(driverIndex),
                              configSize, fileData);
    if (st != 0) return reportError(st);
    std::puts("OK");   // main: puts("OK") after the driver returns
    return EXIT_SUCCESS;
  }

  // Read (and screen) both read the EEPROM first.
  std::printf("Reading EEPROM ... ");
  std::vector<uint8_t> data;
  st = me7::bootReadEEPROM(proto, static_cast<size_t>(driverIndex),
                           configSize, data);
  if (st != 0) return reportError(st);
  std::puts("OK");

  if (opts.action == me7::Action::Screen) {
    printScreen(data);
    return EXIT_SUCCESS;
  }
  // Read: save to file.
  if (opts.fileName.empty()) {
    std::printf("FAIL. (error=0x%02X)\n", 0x30204u);
    return EXIT_FAILURE;
  }
  st = writeEepromFile(opts.fileName, data.data(), data.size());
  if (st != 0) {
    std::printf("FAIL. (error=0x%02X)\n", st);
    return EXIT_FAILURE;
  }
  std::puts("File saved");
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  std::puts("ME7_EEPROM v1.40");

  me7::Options opts;
  auto err = me7::parseOptions(argc, argv, opts);
  if (err) {
    std::printf("%s\n", err->c_str());
    std::puts("try 'me7eeprom --help' for more information.");
    return EXIT_FAILURE;
  }
  if (opts.help) {
    me7::printHelp();
    return EXIT_SUCCESS;
  }

  // Informational header — printed before the COM is opened (mirrors the
  // original main's post-option success block).
  std::printf("COM: %d, Baud Rate: %d\n", opts.comport, opts.baudrate);

  std::vector<uint8_t> fileData;
  size_t cfgSize = 0x200;  // default EEPROM size; set from the mem type in bootmode
  if (opts.mode == me7::Mode::Bootmode) {
    const int rawIndex = me7::BootUploader::memTypeIndex(opts.memType);
    cfgSize = me7::eepromSize(static_cast<size_t>(rawIndex));
    std::printf("Memory type: %s, size: %d\n", opts.memType.c_str(),
                static_cast<int>(cfgSize));
    if (!opts.csPin.empty())
      std::printf("Chip Select pin: %s\n", opts.csPin.c_str());

    if (opts.action == me7::Action::Write) {
      // Read the whole file before opening the COM port.
      const uint32_t st = readEepromFile(opts.fileName, cfgSize, fileData);
      if (st != 0) return reportError(st);
    }
  }

  auto eng = openSerial(opts);
  if (opts.mode == me7::Mode::OBD) return runOBD(*eng, opts);
  return runBootmode(*eng, opts, cfgSize, fileData);
}
