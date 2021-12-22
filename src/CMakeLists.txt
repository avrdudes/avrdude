#
# CMakeLists.txt - CMake project for AVRDUDE
# Copyright (C) 2021 Marius Greuel
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

# Typical usage:
#     cmake -B build
#     cmake --build build

cmake_minimum_required(VERSION 3.12)
project(avrdude VERSION 6.5.0.20211213)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

option(BUILD_DOC "Enable building documents" OFF)
option(HAVE_LINUXGPIO "Enable Linux sysfs GPIO support" OFF)
option(HAVE_LINUXSPI "Enable Linux SPI support" OFF)
option(HAVE_PARPORT "Enable parallel port support" OFF)
option(USE_EXTERNAL "Use local code from the 'external' folder" OFF)
option(USE_LIBUSBWIN32 "Prefer libusb-win32 over libusb" OFF)
option(DEBUG_CMAKE "Enable debugging output for this CMake project" OFF)

include(CheckIncludeFile)
include(CheckFunctionExists)
include(GNUInstallDirs)

set(CONFIG_DIR "${CMAKE_INSTALL_FULL_SYSCONFDIR}")
set(AVRDUDE_EXTERNAL_PATH "${PROJECT_SOURCE_DIR}/external")

# =====================================
# Detect Flex and Bison
# =====================================

find_package(FLEX)
if(FLEX_FOUND)
    FLEX_TARGET(Parser lexer.l "${PROJECT_BINARY_DIR}/lexer.c")
else()
    message(SEND_ERROR "This CMake project requires flex, which is not installed on your system." )
endif()

find_package(BISON)
if(BISON_FOUND)
    BISON_TARGET(Parser config_gram.y "${PROJECT_BINARY_DIR}/config_gram.c" DEFINES_FILE "${PROJECT_BINARY_DIR}/config_gram.h")
else()
    message(SEND_ERROR "This CMake project requires bison, which is not installed on your system." )
endif()

# =====================================
# Detect headers
# =====================================

check_include_file(stdint.h HAVE_STDINT_H)
check_include_file(stdlib.h HAVE_STDLIB_H)
check_include_file(inttypes.h HAVE_INTTYPES_H)
check_include_file(netinet/in.h HAVE_NETINET_IN_H)

# =====================================
# Detect functions
# =====================================

check_function_exists(usleep HAVE_USLEEP)
check_function_exists(getaddrinfo HAVE_GETADDRINFO)
check_function_exists(gettimeofday HAVE_GETTIMEOFDAY)
check_function_exists(strcasecmp HAVE_STRCASECMP)

# =====================================
# Detect installed libraries
# =====================================

# Prefer static libraries over DLLs on Windows
if(WIN32)
    set(PREFERRED_LIBELF libelf.a elf)
    set(PREFERRED_LIBUSB libusb.a usb)
    set(PREFERRED_LIBUSB_1_0 libusb-1.0.a usb-1.0)
    set(PREFERRED_LIBHIDAPI libhidapi.a libhidapi-libusb.a libhidapi-hidraw.a hidapi hidapi-libusb hidapi-hidraw)
    set(PREFERRED_LIBFTDI libftdi.a ftdi)
    set(PREFERRED_LIBFTDI1 libftdi1.a ftdi1)
else()
    set(PREFERRED_LIBELF elf)
    set(PREFERRED_LIBUSB usb)
    set(PREFERRED_LIBUSB_1_0 usb-1.0)
    set(PREFERRED_LIBHIDAPI hidapi hidapi-libusb hidapi-hidraw)
    set(PREFERRED_LIBFTDI ftdi)
    set(PREFERRED_LIBFTDI1 ftdi1)
endif()

# -------------------------------------
# Find libelf

find_library(HAVE_LIBELF NAMES ${PREFERRED_LIBELF})
if(HAVE_LIBELF)
    set(LIB_LIBELF ${HAVE_LIBELF})
    check_include_file(libelf.h HAVE_LIBELF_H)
    check_include_file(libelf/libelf.h HAVE_LIBELF_LIBELF_H)
endif()

# -------------------------------------
# Find libusb

find_library(HAVE_LIBUSB NAMES ${PREFERRED_LIBUSB})
if(HAVE_LIBUSB)
    set(LIB_LIBUSB ${HAVE_LIBUSB})
endif()

find_library(HAVE_LIBUSB_1_0 NAMES ${PREFERRED_LIBUSB_1_0})
if(HAVE_LIBUSB_1_0)
    set(LIB_LIBUSB_1_0 ${HAVE_LIBUSB_1_0})
endif()

find_library(HAVE_LIBUSB_WIN32 NAMES libusb0.a usb0)

if(HAVE_LIBUSB OR HAVE_LIBUSB_1_0 OR HAVE_LIBUSB_WIN32)
    check_include_file(usb.h HAVE_USB_H)
    check_include_file(lusb0_usb.h HAVE_LUSB0_USB_H)
    check_include_file(libusb.h HAVE_LIBUSB_H)
    check_include_file(libusb-1.0/libusb.h HAVE_LIBUSB_1_0_LIBUSB_H)

    if((USE_LIBUSBWIN32 OR NOT HAVE_LIBUSB) AND HAVE_LIBUSB_WIN32)
        set(HAVE_LIBUSB ${HAVE_LIBUSB_WIN32})
        set(LIB_LIBUSB ${HAVE_LIBUSB_WIN32})
        unset(HAVE_USB_H CACHE)
    elseif(NOT HAVE_USB_H)
        find_path(LIBUSB_COMPAT_DIR libusb-compat/usb.h)
        if(LIBUSB_COMPAT_DIR)
            set(LIBUSB_COMPAT_DIR ${LIBUSB_COMPAT_DIR}/libusb-compat)
            set(HAVE_USB_H 1)
        else()
            unset(LIBUSB_COMPAT_DIR CACHE)
        endif()
    endif()
endif()

# -------------------------------------
# Find libhidapi

find_library(HAVE_LIBHIDAPI NAMES ${PREFERRED_LIBHIDAPI})
if(HAVE_LIBHIDAPI)
    set(LIB_LIBHIDAPI ${HAVE_LIBHIDAPI})
    check_include_file(hidapi/hidapi.h HAVE_HIDAPI_HIDAPI_H)
endif()

find_library(HAVE_LIBHID NAMES hid)

# -------------------------------------
# Find libftdi

find_library(HAVE_LIBFTDI NAMES ${PREFERRED_LIBFTDI})
if(HAVE_LIBFTDI)
    set(LIB_LIBFTDI ${HAVE_LIBFTDI})
    set(HAVE_LIBFTDI_TYPE_232H 1)
endif()

find_library(HAVE_LIBFTDI1 NAMES ${PREFERRED_LIBFTDI1})
if(HAVE_LIBFTDI1)
    set(LIB_LIBFTDI1 ${HAVE_LIBFTDI1})
    set(HAVE_LIBFTDI_TYPE_232H 1)
endif()

# -------------------------------------
# Find libreadline

find_library(HAVE_LIBREADLINE NAMES readline)
if(HAVE_LIBREADLINE)
    set(LIB_LIBREADLINE ${HAVE_LIBREADLINE})
endif()

# =====================================
# Setup target specific options
# =====================================

if(WIN32)
    set(HAVE_LIBWS2_32 1)
    set(EXTRA_WINDOWS_SOURCES "${PROJECT_BINARY_DIR}/windows.rc")
    set(EXTRA_WINDOWS_LIBRARIES setupapi ws2_32)
    add_compile_definitions(WIN32NATIVE=1)
endif()

if(NOT WIN32)
    set(LIB_MATH m)
    #add_compile_options(-Wall -Wextra -pedantic)
endif()

# =====================================
# Setup default port names
# =====================================

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(DEFAULT_PAR_PORT "/dev/parport0")
    set(DEFAULT_SER_PORT "/dev/ttyS0")
elseif (CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
    set(DEFAULT_PAR_PORT "/dev/ppi0")
    set(DEFAULT_SER_PORT "/dev/cuad0")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Solaris")
    set(DEFAULT_PAR_PORT "/dev/printers/0")
    set(DEFAULT_SER_PORT "/dev/term/a")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(DEFAULT_PAR_PORT "lpt1")
    set(DEFAULT_SER_PORT "com1")
else()
    set(DEFAULT_PAR_PORT "unknown")
    set(DEFAULT_SER_PORT "unknown")
endif()

# =====================================
# Configuration
# =====================================

message(STATUS "Configuration summary:")
message(STATUS "----------------------")

if (DEBUG_CMAKE)
    message(STATUS "CMAKE_HOST_SYSTEM: ${CMAKE_HOST_SYSTEM}")
    message(STATUS "CMAKE_SYSTEM: ${CMAKE_SYSTEM}")
    message(STATUS "CONFIG_DIR: ${CONFIG_DIR}")
    message(STATUS "USE_EXTERNAL: ${USE_EXTERNAL}")
    message(STATUS "USE_LIBUSBWIN32: ${USE_LIBUSBWIN32}")
    message(STATUS "HAVE_LIBELF: ${HAVE_LIBELF}")
    message(STATUS "HAVE_LIBUSB: ${HAVE_LIBUSB}")
    message(STATUS "HAVE_LIBUSB_1_0: ${HAVE_LIBUSB_1_0}")
    message(STATUS "HAVE_LIBUSB_WIN32: ${HAVE_LIBUSB_WIN32}")
    message(STATUS "HAVE_LIBHIDAPI: ${HAVE_LIBHIDAPI}")
    message(STATUS "HAVE_LIBFTDI: ${HAVE_LIBFTDI}")
    message(STATUS "HAVE_LIBFTDI1: ${HAVE_LIBFTDI1}")
    message(STATUS "HAVE_LIBREADLINE: ${HAVE_LIBREADLINE}")
    message(STATUS "HAVE_LIBWS2_32: ${HAVE_LIBWS2_32}")
    message(STATUS "HAVE_LIBELF_H: ${HAVE_LIBELF_H}")
    message(STATUS "HAVE_LIBELF_LIBELF_H: ${HAVE_LIBELF_LIBELF_H}")
    message(STATUS "HAVE_USB_H: ${HAVE_USB_H}")
    message(STATUS "HAVE_LUSB0_USB_H: ${HAVE_LUSB0_USB_H}")
    message(STATUS "HAVE_LIBUSB_H: ${HAVE_LIBUSB_H}")
    message(STATUS "HAVE_LIBUSB_1_0_LIBUSB_H: ${HAVE_LIBUSB_1_0_LIBUSB_H}")
    message(STATUS "HAVE_HIDAPI_HIDAPI_H: ${HAVE_HIDAPI_HIDAPI_H}")
    message(STATUS "LIBUSB_COMPAT_DIR: ${LIBUSB_COMPAT_DIR}")
    message(STATUS "----------------------")
endif()

if(HAVE_LIBELF)
    message(STATUS "DO HAVE    libelf")
else()
    message(STATUS "DON'T HAVE libelf")
endif()

if(HAVE_LIBUSB)
    message(STATUS "DO HAVE    libusb")
else()
    message(STATUS "DON'T HAVE libusb")
endif()

if(HAVE_LIBUSB_1_0)
    message(STATUS "DO HAVE    libusb_1_0")
else()
    message(STATUS "DON'T HAVE libusb_1_0")
endif()

if(HAVE_LIBHIDAPI)
    message(STATUS "DO HAVE    libhidapi")
else()
    message(STATUS "DON'T HAVE libhidapi")
endif()

if(HAVE_LIBFTDI)
    if(HAVE_LIBFTDI1)
        message(STATUS "DO HAVE    libftdi (but prefer to use libftdi1)")
    else()
        message(STATUS "DO HAVE    libftdi")
    endif()
else()
    message(STATUS "DON'T HAVE libftdi")
endif()

if(HAVE_LIBFTDI1)
    message(STATUS "DO HAVE    libftdi1")
else()
    message(STATUS "DON'T HAVE libftdi1")
endif()

if(BUILD_DOC)
	message(STATUS "ENABLED    doc")
else()
	message(STATUS "DISABLED   doc")
endif()

if(HAVE_PARPORT)
    message(STATUS "ENABLED    parport")
else()
    message(STATUS "DISABLED   parport")
endif()

if(HAVE_LINUXGPIO)
    message(STATUS "ENABLED    linuxgpio")
else()
    message(STATUS "DISABLED   linuxgpio")
endif()

if(HAVE_LINUXSPI)
    message(STATUS "ENABLED    linuxspi")
else()
    message(STATUS "DISABLED   linuxspi")
endif()

message(STATUS "----------------------")

# =====================================
# Configure files
# =====================================

configure_file(cmake_config.h.in ac_cfg.h)
configure_file(avrdude.conf.in avrdude.conf)
configure_file(avrdude.spec.in avrdude.spec)
if(WIN32)
    configure_file(windows.rc.in windows.rc)
endif()

# =====================================
# Project
# =====================================

add_library(libavrdude STATIC
    ac_cfg.h
    arduino.h
    arduino.c
    avr.c
    avr910.c
    avr910.h
    avrdude.h
    avrftdi.c
    avrftdi.h
    avrftdi_private.h
    avrftdi_tpi.c
    avrftdi_tpi.h
    avrpart.c
    bitbang.c
    bitbang.h
    buspirate.c
    buspirate.h
    butterfly.c
    butterfly.h
    config.c
    config.h
    confwin.c
    crc16.c
    crc16.h
    dfu.c
    dfu.h
    fileio.c
    flip1.c
    flip1.h
    flip2.c
    flip2.h
    freebsd_ppi.h
    ft245r.c
    ft245r.h
    jtagmkI.c
    jtagmkI.h
    jtagmkI_private.h
    jtagmkII.c
    jtagmkII.h
    jtagmkII_private.h
    jtag3.c
    jtag3.h
    jtag3_private.h
    libavrdude.h
    linuxgpio.c
    linuxgpio.h
    linuxspi.c
    linuxspi.h
    linux_ppdev.h
    lists.c
    my_ddk_hidsdi.h
    par.c
    par.h
    pgm.c
    pgm_type.c
    pickit2.c
    pickit2.h
    pindefs.c
    ppi.c
    ppi.h
    ppiwin.c
    safemode.c
    serbb.h
    serbb_posix.c
    serbb_win32.c
    ser_avrdoper.c
    ser_posix.c
    ser_win32.c
    solaris_ecpp.h
    stk500.c
    stk500.h
    stk500_private.h
    stk500v2.c
    stk500v2.h
    stk500v2_private.h
    stk500generic.c
    stk500generic.h
    tpi.h
    usbasp.c
    usbasp.h
    usbdevs.h
    usb_hidapi.c
    usb_libusb.c
    usbtiny.h
    usbtiny.c
    update.c
    wiring.h
    wiring.c
    xbee.h
    xbee.c
    ${FLEX_Parser_OUTPUTS}
    ${BISON_Parser_OUTPUTS}
    )

target_include_directories(libavrdude
    PUBLIC
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_BINARY_DIR}"
    "${LIBUSB_COMPAT_DIR}"
    "${EXTRA_WINDOWS_INCLUDES}"
    )

target_link_libraries(libavrdude
    PUBLIC
    ${LIB_MATH}
    ${LIB_LIBELF}
    ${LIB_LIBUSB}
    ${LIB_LIBUSB_1_0}
    ${LIB_LIBHIDAPI}
    ${LIB_LIBFTDI}
    ${LIB_LIBFTDI1}
    ${LIB_LIBREADLINE}
    ${EXTRA_WINDOWS_LIBRARIES}
    )

add_executable(avrdude
    main.c
    term.c
    term.h
    "${EXTRA_WINDOWS_SOURCES}"
    )

target_link_libraries(avrdude PUBLIC libavrdude)

# =====================================
# Install
# =====================================

install(TARGETS avrdude DESTINATION bin)
install(FILES "${PROJECT_BINARY_DIR}/avrdude.conf" TYPE SYSCONF)
install(FILES avrdude.1 TYPE MAN)
