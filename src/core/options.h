#pragma once

// ME7_EEPROM command-line option model.
//
// CLI option model matching the original ME7EEPROM v1.40 tool's options,
// defaults, error strings, and exit paths so behavior stays byte-identical.

#include <string>
#include <optional>

namespace me7 {

enum class Mode { None, OBD, Bootmode };
enum class Action { None, Read, Write, Screen };

// Allowed serial EEPROM memory types (from the original help text).
inline constexpr const char* kMemoryTypes[] = {"95040", "95080", "95160", "95P08"};

// Allowed baud rates (from the original help text).
inline constexpr int kBaudRates[] = {9600, 10400, 19200, 57600};

struct Options {
  Mode   mode        = Mode::None;
  Action action      = Action::None;
  std::string memType;   // required for --bootmode
  std::string csPin;     // --CSpin, eg "P4.7"
  int    comport    = 0;         // 0 = unspecified; COM1 => 1
  int    baudrate   = 10400;     // default from original
  std::string fileName;
  bool   help       = false;
  bool   immo       = false;   // --immo: build an immo-off image from an OBD read
};

// Parses argc/argv into `out`. Returns nullopt on success; on failure returns
// the exact error string the original prints (excluding any leading "Error: ").
std::optional<std::string> parseOptions(int argc, char** argv, Options& out);

// Help/usage text reproduced verbatim from the original binary.
void printUsage();
void printHelp();

}  // namespace me7
