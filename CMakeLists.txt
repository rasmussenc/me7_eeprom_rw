cmake_minimum_required(VERSION 3.20)
project(me7eeprom VERSION 1.40 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# ---------------------------------------------------------------------------
# core: portable protocol / domain logic (no OS dependencies)
# ---------------------------------------------------------------------------
add_library(me7_core STATIC
  src/core/options.cpp
  src/core/serial_engine.cpp
  src/core/protocol.cpp
  src/core/uploader.cpp
  src/core/eeprom.cpp
  src/core/immo.cpp
)
target_include_directories(me7_core PUBLIC src)

# ---------------------------------------------------------------------------
# pal: platform abstraction layer (serial ports)
#   - serial_pal.h defines the abstract SerialPort interface (portable)
#   - concrete backends live in src/pal/ and are selected here
#
# Backend selection. The POSIX termios backend is the default: it opens a pty
# (so the loopback validation harness works) and any standard-baud /dev/cu.* or
# /dev/ttyUSB* adapter with no external dependencies. libserialport remains
# available for the one thing termios can't express — a true 10400 baud custom
# divisor on real K-line hardware.
# ---------------------------------------------------------------------------
option(ME7_USE_LIBSERIALPORT OFF "Use the libserialport backend instead of native POSIX termios")

if(ME7_USE_LIBSERIALPORT)
  find_package(PkgConfig QUIET)
  find_package(libserialport QUIET)
  if(NOT (libserialport_FOUND OR PKG_CONFIG_FOUND))
    message(FATAL_ERROR "ME7_USE_LIBSERIALPORT=ON but libserialport was not found. "
                        "Install it (brew install libserialport) or leave the option OFF "
                        "to use the native POSIX termios backend.")
  endif()
  if(NOT libserialport_FOUND)
    pkg_check_modules(SERIALPORT libserialport)
  endif()
  add_library(me7_pal STATIC src/pal/serial_libserialport.cpp)
  target_include_directories(me7_pal PUBLIC src)
  if(TARGET PkgConfig::SERIALPORT)
    target_link_libraries(me7_pal PUBLIC PkgConfig::SERIALPORT)
  else()
    target_include_directories(me7_pal PUBLIC ${SERIALPORT_INCLUDE_DIRS})
    target_link_directories(me7_pal PUBLIC ${SERIALPORT_LIBRARY_DIRS})
    target_link_libraries(me7_pal PUBLIC ${SERIALPORT_LIBRARIES})
  endif()
  message(STATUS "me7_pal: libserialport backend (ME7_USE_LIBSERIALPORT=ON)")
else()
  add_library(me7_pal STATIC src/pal/serial_posix.cpp)
  target_include_directories(me7_pal PUBLIC src)
  message(STATUS "me7_pal: native POSIX termios backend (default)")
endif()

# ---------------------------------------------------------------------------
# executable
# ---------------------------------------------------------------------------
add_executable(me7eeprom
  src/main.cpp
  src/firmware/embedded.cpp
)
target_link_libraries(me7eeprom PRIVATE me7_core me7_pal)

# Make the firmware blobs available at build time.
target_include_directories(me7eeprom PRIVATE src)

enable_testing()
add_subdirectory(tests)
