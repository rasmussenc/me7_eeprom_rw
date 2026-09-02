// Unit tests for the CLI option parser.
// Minimal harness — zero external deps. Returns non-zero on any failure.

#include "core/options.h"

#include <cstdio>
#include <cstring>
#include <vector>

namespace {
int failures = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
      ++failures;                                                            \
    }                                                                        \
  } while (0)

// Parse from a vector of args. Returns the optional error string.
// NOTE: parseOptions treats argv[0] as the program name (like real OS argv),
// so we must prepend a placeholder before the user-supplied options.
std::optional<std::string> parse(std::vector<std::string> args, me7::Options& out) {
  std::vector<std::string> full;
  full.reserve(args.size() + 1);
  full.push_back("me7eeprom");  // argv[0]
  for (auto& a : args) full.push_back(a);
  std::vector<char*> ptrs;
  for (auto& a : full) ptrs.push_back(a.data());
  return me7::parseOptions(static_cast<int>(ptrs.size()), ptrs.data(), out);
}
}  // namespace

int main() {
  me7::Options o;

  // Untouched defaults.
  CHECK(parse({"--bootmode", "95040", "-r", "-p", "1", "out.bin"}, o) == std::nullopt);
  CHECK(o.mode == me7::Mode::Bootmode);
  CHECK(o.memType == "95040");
  CHECK(o.action == me7::Action::Read);
  CHECK(o.comport == 1);
  CHECK(o.baudrate == 10400);   // default
  CHECK(o.fileName == "out.bin");

  // Bundled short option: -wp1 => write + port 1.
  CHECK(parse({"--bootmode", "95160", "-wp1", "f.bin"}, o) == std::nullopt);
  CHECK(o.action == me7::Action::Write);
  CHECK(o.comport == 1);

  // OBD read with screen.
  CHECK(parse({"--OBD", "-p1", "--screen"}, o) == std::nullopt);
  CHECK(o.mode == me7::Mode::OBD);
  CHECK(o.action == me7::Action::Screen);

  // Baud override.
  CHECK(parse({"--OBD", "-r", "-p", "2", "-b", "19200", "x.bin"}, o) == std::nullopt);
  CHECK(o.baudrate == 19200);

  // Errors : mode/action/port required.
  CHECK(parse({"-r", "-p", "1"}, o) != std::nullopt);               // no mode
  CHECK(parse({"--OBD", "-p", "1"}, o) != std::nullopt);            // no action
  CHECK(parse({"--OBD", "-r"}, o) != std::nullopt);                 // no port
  CHECK(parse({"--OBD", "-w", "-p", "1", "f.bin"}, o) != std::nullopt);  // write over OBD
  CHECK(parse({"--OBD", "--bootmode", "1", "-r", "-p", "1"}, o) != std::nullopt);  // exclusive modes
  CHECK(parse({"--bootmode", "9999", "-r", "-p", "1"}, o) != std::nullopt);  // bad mem type
  CHECK(parse({"--OBD", "-r", "-p", "1", "-b", "12345", "f.bin"}, o) != std::nullopt);  // bad baud

  // Help.
  CHECK(parse({"--help"}, o) == std::nullopt);
  CHECK(o.help);

  if (failures == 0) {
    std::puts("test_options: all checks passed");
    return 0;
  }
  std::printf("test_options: %d failure(s)\n", failures);
  return 1;
}
