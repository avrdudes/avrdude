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
project(avrdude VERSION 7.0 LANGUAGES C)

# moved here from src/configure.ac
set(PROJ_VERSION_DATE "20220508")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

option(BUILD_DOC "Enable building documents" OFF)
option(HAVE_LINUXGPIO "Enable Linux sysfs GPIO support" OFF)
option(HAVE_LINUXSPI "Enable Linux SPI support" OFF)
option(HAVE_PARPORT "Enable parallel port support" OFF)
option(USE_EXTERNAL "Use external libraries from AVRDUDE GitHub repositories" OFF)
option(USE_LIBUSBWIN32 "Prefer libusb-win32 over libusb" OFF)
option(DEBUG_CMAKE "Enable debugging output for this CMake project" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

if(WIN32)
    # Prefer static libraries over DLLs on Windows
    option(USE_STATIC_LIBS "Use static libraries" ON)
else()
    option(USE_STATIC_LIBS "Use static libraries" OFF)
endif()

include(CheckIncludeFile)
include(CheckSymbolExists)
include(FetchContent)
include(FindPackageMessage)
include(GNUInstallDirs)

set(CONFIG_DIR "${CMAKE_INSTALL_FULL_SYSCONFDIR}")
set(AVRDUDE_FULL_VERSION ${CMAKE_PROJECT_VERSION})

# =====================================
# Get Git commit info
# =====================================

# GIT_COMMIT_HASH -> hash of latest commit, e.g. b8b859f5
# GIT_COMMIT_DATE -> date of latest commit, e.g. 20201231
# GIT_COMMIT_YEAR -> year of latest commit, e.g. 2020

find_package(Git)
if(Git_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" log -1 --format=%h
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
        )

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" log -1 --format=%ad --date=format:%Y%m%d
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
        )

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" log -1 --format=%ad --date=format:%Y
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_YEAR
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
        )

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" log -1 --tags --format=%h
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_TAG_HASH
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
        )

    # If the commit is not tagged, include the date and commit hash in the full version string.
    if(NOT GIT_COMMIT_HASH STREQUAL GIT_TAG_HASH)
        set(AVRDUDE_FULL_VERSION "${CMAKE_PROJECT_VERSION}-${GIT_COMMIT_DATE} (${GIT_COMMIT_HASH})")
    endif()
endif()

find_package(FLEX)
if(NOT FLEX_FOUND)
    message(SEND_ERROR "This CMake project requires 'flex', which is not installed on your system." )
endif()

find_package(BISON QUIET)
if(BISON_FOUND)
    find_package_message(BISON "Found BISON: ${BISON_EXECUTABLE} (found version \"${BISON_VERSION}\")" "[${BISON_EXECUTABLE}][${BISON_VERSION}]")
else()
    find_program(YACC_EXECUTABLE NAMES yacc byacc DOC "path to the yacc executable")
    mark_as_advanced(YACC_EXECUTABLE)
    if(YACC_EXECUTABLE)
        find_package_message(YACC "Found YACC: ${YACC_EXECUTABLE}" "[${YACC_EXECUTABLE}]")
    else()
        message(SEND_ERROR "This CMake project requires 'bison', 'yacc', or 'byacc', which is not installed on your system." )
    endif()
endif()

# =====================================
# Detect installed libraries
# =====================================

if(USE_STATIC_LIBS)
    set(PREFERRED_LIBELF libelf.a elf)
    set(PREFERRED_LIBUSB libusb.a usb)
    set(PREFERRED_LIBUSB_1_0 libusb-1.0.a usb-1.0)
    set(PREFERRED_LIBHIDAPI libhidapi.a libhidapi-libusb.a libhidapi-hidraw.a hidapi hidapi-libusb hidapi-hidraw)
    set(PREFERRED_LIBFTDI libftdi.a ftdi)
    set(PREFERRED_LIBFTDI1 libftdi1.a ftdi1)
    set(PREFERRED_LIBREADLINE libreadline.a)
else()
    set(PREFERRED_LIBELF elf)
    set(PREFERRED_LIBUSB usb)
    set(PREFERRED_LIBUSB_1_0 usb-1.0)
    set(PREFERRED_LIBHIDAPI hidapi hidapi-libusb hidapi-hidraw)
    set(PREFERRED_LIBFTDI ftdi)
    set(PREFERRED_LIBFTDI1 ftdi1)
    set(PREFERRED_LIBREADLINE readline)
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

# FreeBSD's library 'libusb' supports both the libusb-0.1 and libusb-1.0 API.
if (HAVE_LIBUSB AND CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
    set(HAVE_LIBUSB_1_0 ${HAVE_LIBUSB})
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

find_library(HAVE_LIBHID NAMES hid)
if(HAVE_LIBHID)
    set(LIB_LIBHID ${HAVE_LIBHID})
endif()

find_library(HAVE_LIBHIDAPI NAMES ${PREFERRED_LIBHIDAPI})
if(HAVE_LIBHIDAPI)
    set(LIB_LIBHIDAPI ${HAVE_LIBHIDAPI})
    check_include_file(hidapi/hidapi.h HAVE_HIDAPI_HIDAPI_H)
endif()

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
    set(CMAKE_REQUIRED_LIBRARIES ${LIB_LIBFTDI1} ${LIB_LIBUSB} ${LIB_LIBUSB_1_0})
    check_symbol_exists(ftdi_tcioflush "libftdi1/ftdi.h" HAVE_FTDI_TCIOFLUSH)
endif()

# -------------------------------------
# Find libreadline

find_library(HAVE_LIBREADLINE NAMES ${PREFERRED_LIBREADLINE})
if(HAVE_LIBREADLINE)
    set(LIB_LIBREADLINE ${HAVE_LIBREADLINE})
endif()

# =====================================
# Use external libraries if requested
# =====================================

if(USE_EXTERNAL)
    FetchContent_Declare(libelf
        GIT_REPOSITORY https://github.com/avrdudes/libelf.git
        GIT_TAG e5a39bf19bd6598c42e09172be5a78ceec2a065c
        )

    FetchContent_Declare(libusb
        GIT_REPOSITORY https://github.com/avrdudes/libusb.git
        GIT_TAG 632bc25d04eff563cc00de29435b9a7ed6f4654c
        )

    FetchContent_Declare(libhidapi
        GIT_REPOSITORY https://github.com/avrdudes/libhidapi.git
        GIT_TAG e3700e951f762ef92871ff4fc94586e4d1c042a6
        )

    FetchContent_Declare(libftdi
        GIT_REPOSITORY https://github.com/avrdudes/libftdi.git
        GIT_TAG f3a54da710002a7d25a32a69e667a69ef84cc120
        )

    message(STATUS "Fetching external libraries, please wait...")
    FetchContent_MakeAvailable(
        libelf
        libusb
        libhidapi
        libftdi
        )

    message(STATUS "Using external library 'libelf'")
    set(LIB_LIBELF libelf)
    set(HAVE_LIBELF 1)
    set(HAVE_LIBELF_H 1)

    message(STATUS "Using external library 'libusb'")
    set(LIB_LIBUSB libusb)
    set(HAVE_LIBUSB 1)
    set(HAVE_LUSB0_USB_H 1)

    message(STATUS "Using external library 'libhidapi'")
    set(LIB_LIBHIDAPI libhidapi)
    set(HAVE_LIBHIDAPI 1)
    set(HAVE_HIDAPI_HIDAPI_H 1)

    message(STATUS "Using external library 'libftdi'")
    set(LIB_LIBFTDI libftdi)
    set(HAVE_LIBFTDI 1)
    set(HAVE_LIBFTDI_TYPE_232H 1)
endif()

add_subdirectory(src)

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
    message(STATUS "CMAKE_FIND_ROOT_PATH: ${CMAKE_FIND_ROOT_PATH}")
    message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
    message(STATUS "CONFIG_DIR: ${CONFIG_DIR}")
    message(STATUS "AVRDUDE_FULL_VERSION: ${AVRDUDE_FULL_VERSION}")
    message(STATUS "USE_EXTERNAL: ${USE_EXTERNAL}")
    message(STATUS "USE_LIBUSBWIN32: ${USE_LIBUSBWIN32}")
    message(STATUS "HAVE_LIBELF: ${HAVE_LIBELF}")
    message(STATUS "HAVE_LIBUSB: ${HAVE_LIBUSB}")
    message(STATUS "HAVE_LIBUSB_1_0: ${HAVE_LIBUSB_1_0}")
    message(STATUS "HAVE_LIBUSB_WIN32: ${HAVE_LIBUSB_WIN32}")
    message(STATUS "HAVE_LIBHID: ${HAVE_LIBHID}")
    message(STATUS "HAVE_LIBHIDAPI: ${HAVE_LIBHIDAPI}")
    message(STATUS "HAVE_LIBFTDI: ${HAVE_LIBFTDI}")
    message(STATUS "HAVE_LIBFTDI1: ${HAVE_LIBFTDI1}")
    message(STATUS "HAVE_LIBREADLINE: ${HAVE_LIBREADLINE}")
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
