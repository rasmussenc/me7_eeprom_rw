#include "core/options.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

namespace me7 {

namespace {

bool isBaudRate(int b) {
  for (int allowed : kBaudRates)
    if (b == allowed) return true;
  return false;
}

bool isMemoryType(const std::string& t) {
  for (const char* allowed : kMemoryTypes)
    if (t == allowed) return true;
  return false;
}

bool isAllowedPort(int p) {
  // Original checks the port against an internal port table; a sane default is
  // COM1..COM16.
  return p >= 1 && p <= 16;
}

}  // namespace

void printUsage() {
  std::puts("Usage: me7EEPROM [-p <comport>] [-b <baudrate>] [--OBD | --bootmode <mem_type>] [--CSpin Px.x] [-r | -w | -s] [<file name>]");
}

void printHelp() {
  printUsage();
  std::puts("Usage example:");
  std::puts("   Write file to EEPROM in bootmode:  $ ME7_EEPROM --bootmode 95040 -wp1 95040.bin");
  std::puts("   Read EEPROM over OBD port:         $ ME7_EEPROM --OBD -r -p 1 95040.bin");
  std::puts("   Print EEPROM contents to srcreen:  $ ME7_EEPROM --OBD -p1 --screen");
  std::puts("   ");
  std::puts(" -p, --comport  COMPORT    Set COMPORT.");
  std::puts(" -b, --baudrate BAUDRATE   Set BAUDRATE, default: 10400. ");
  std::puts(" -r, --read                Read EEPROM contents and save it to file.");
  std::puts(" -s, --screen              Displays EEPROM contents on the screen.");
  std::puts(" -w, --write               Write a file to EEPROM.");
  std::puts("     --bootmode MEM_TYPE   Use this option to program the EEPROM in boot mode.");
  std::puts("     --OBD                 Use this option to read the EEPROM over OBD port.");
  std::puts("     --immo                With --OBD -r: read twice & verify, then build an");
  std::puts("                            immobiliser-off image (<file>.immooff.bin). No ECU");
  std::puts("                            write is performed; write back later via bootmode.");
  std::puts("     --CSpin               Set the CPU chip select (CS) output. eg: P4.7");
  std::puts("     --help                Display this help and exit.");
  std::puts(" ");
  std::puts("                           Allowed baud rates: 9600, 10400, 19200, 57600.");
  std::puts("                           Allowed memory types: [95040 | 95080 | 95160 | 95P08].");
  std::puts("                           Currently, only read is supported in OBD mode.");
  std::puts(" ");
  std::puts("  On Linux/macOS the -p COMPORT is the COM-number only; point the");
  std::puts("  adapter at a real device with ME7_DEVICE (e.g. /dev/cu.usbserial-1420).");
}

// ---------------------------------------------------------------------------
// Narrow getopt-style short-option scan. The original bundles shorts like
// "-wp1" (write + port 1) and "-p1", so we handle attached values too.
// ---------------------------------------------------------------------------
std::optional<std::string> parseOptions(int argc, char** argv, Options& out) {
  out = Options{};  // always start from a clean slate
  std::vector<std::string> positional;
  int modeCount = 0;  // # of --OBD / --bootmode seen

  auto bail = [](const char* msg) -> std::optional<std::string> {
    return std::string("Error: ") + msg;
  };

  auto needValue = [&](const auto& haveNext) -> std::optional<std::string> {
    (void)haveNext;
    return std::nullopt;  // unused fallback
  };
  (void)needValue;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];

    // ---- long options ----
    if (a == "--help") { out.help = true; continue; }
    if (a == "--OBD")  { out.mode = Mode::OBD; modeCount++; continue; }
    if (a == "--immo") { out.immo = true; continue; }
    if (a == "--screen") {
      if (out.action != Action::None) return bail("ilegal read/write options");
      out.action = Action::Screen;
      continue;
    }

    if (a.rfind("--bootmode", 0) == 0) {
      std::string v;
      if (a.find('=') != std::string::npos) v = a.substr(a.find('=') + 1);
      else if (i + 1 < argc) v = argv[++i];
      else return bail("option `bootmode' requires an argument");
      if (!isMemoryType(v)) return bail(("unknown memory type " + v).c_str());
      out.mode = Mode::Bootmode;
      out.memType = v;
      modeCount++;
      continue;
    }

    if (a.rfind("--baudrate", 0) == 0) {
      std::string v;
      if (a.find('=') != std::string::npos) v = a.substr(a.find('=') + 1);
      else if (i + 1 < argc) v = argv[++i];
      else return bail("option `baudrate' requires an argument");
      int b = std::atoi(v.c_str());
      if (v.empty() || !isBaudRate(b))
        return bail(("invalid baudrate argument `" + v + "'").c_str());
      out.baudrate = b;
      continue;
    }

    if (a.rfind("--comport", 0) == 0) {
      std::string v;
      if (a.find('=') != std::string::npos) v = a.substr(a.find('=') + 1);
      else if (i + 1 < argc) v = argv[++i];
      else return bail("option `com_port' requires an argument");
      int p = std::atoi(v.c_str());
      if (v.empty() || !isAllowedPort(p))
        return bail(("invalid com_port argument `" + v + "'").c_str());
      out.comport = p;
      continue;
    }

    if (a.rfind("--CSpin", 0) == 0) {
      std::string v;
      if (a.find('=') != std::string::npos) v = a.substr(a.find('=') + 1);
      else if (i + 1 < argc) v = argv[++i];
      else return bail("option `CSpin' requires an argument");
      // Original validates with a port table; light check here.
      out.csPin = v;
      continue;
    }

    // ---- short options, possibly bundled with attached values ----
    if (a.size() >= 2 && a[0] == '-' && a[1] != '-') {
      for (size_t j = 1; j < a.size(); ++j) {
        char c = a[j];
        switch (c) {
          case 'p': {
            // -p with attached value, e.g. "-p1"
            std::string v = a.substr(j + 1);
            if (v.empty()) {
              if (i + 1 < argc) v = argv[++i];
              else return bail("option `com_port' requires an argument");
            }
            int p = std::atoi(v.c_str());
            if (v.empty() || !isAllowedPort(p))
              return bail(("invalid com_port argument `" + v + "'").c_str());
            out.comport = p;
            j = a.size();  // consumed rest
            break;
          }
          case 'b': {
            std::string v = a.substr(j + 1);
            if (v.empty()) {
              if (i + 1 < argc) v = argv[++i];
              else return bail("option `baudrate' requires an argument");
            }
            int b = std::atoi(v.c_str());
            if (v.empty() || !isBaudRate(b))
              return bail(("invalid baudrate argument `" + v + "'").c_str());
            out.baudrate = b;
            j = a.size();
            break;
          }
          case 'r': if (out.action == Action::None) out.action = Action::Read; else return bail("ilegal read/write options"); break;
          case 'w': if (out.action == Action::None) out.action = Action::Write; else return bail("ilegal read/write options"); break;
          case 's': if (out.action == Action::None) out.action = Action::Screen; else return bail("ilegal read/write options"); break;
          case 'h': out.help = true; break;
          default:
            return bail((std::string("unrecognised option -") + c).c_str());
        }
      }
      continue;
    }

    positional.push_back(a);
  }

  // ---- post-parse validation, mirroring the original ordering ----
  if (out.help)
    return std::nullopt;  // --help needs no mode/action/port
  if (out.mode == Mode::None)
    return bail("'OBD' or 'bootmode' option not specified");
  if (modeCount > 1)
    return bail("'OBD' and 'bootmode' are exclusive options");
  if (out.action == Action::None)
    return bail("read/write option not specified");
  if (out.comport == 0)
    return bail("com_port option not specified");
  if (out.mode == Mode::OBD && out.action == Action::Write)
    return bail("EEPROM write over OBD port not supported");
  if (out.immo && out.mode != Mode::OBD)
    return bail("--immo is only supported in OBD mode");
  if (out.immo && out.action != Action::Read)
    return bail("--immo requires -r (read mode)");
  if (positional.size() > 1)
    return bail("too many arguments specified");
  if (positional.size() == 1)
    out.fileName = positional[0];

  return std::nullopt;
}

}  // namespace me7
