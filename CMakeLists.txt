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

# CMakeLists.txt - Modernized Configuration for AVRDUDE
cmake_minimum_required(VERSION 3.14)
project(avrdude VERSION 8.1 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

option(BUILD_DOC "Enable building documents" OFF)
option(HAVE_LINUXGPIO "Enable Linux sysfs and libgpiod GPIO support" OFF)
option(HAVE_LINUXSPI "Enable Linux SPI support" OFF)
option(HAVE_PARPORT "Enable parallel port support" OFF)
option(USE_EXTERNAL_LIBS "Use external libraries from AVRDUDE GitHub repositories" OFF)
option(USE_LIBUSBWIN32 "Prefer libusb-win32 over libusb" OFF)
option(DEBUG_CMAKE "Enable debugging output for this CMake project" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(FORCE_DISABLE_PYTHON_SUPPORT "Option to explicitly disable Python support" OFF)

if(WIN32)
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
set(AVRDUDE_FULL_VERSION "${CMAKE_PROJECT_VERSION}${EXTRA_VERSION_SUFFIX}")

# Global configuration directory preprocessor macro definition
add_compile_definitions(CONFIG_DIR="${CONFIG_DIR}")

# =====================================
# Get Git commit info Safely
# =====================================
find_package(Git)
if(Git_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" log -1 --format=%h
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" log -1 --format=%cd --date=format:%Y%m%d
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" log -1 --tags --format=%h
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_TAG_HASH
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(GIT_COMMIT_HASH AND NOT GIT_COMMIT_HASH STREQUAL GIT_TAG_HASH)
        set(AVRDUDE_FULL_VERSION "${CMAKE_PROJECT_VERSION}-${GIT_COMMIT_DATE}${EXTRA_VERSION_SUFFIX} (${GIT_COMMIT_HASH})")
    endif()
endif()

# =====================================
# Core Build Tools
# =====================================
find_package(FLEX REQUIRED)
find_package(BISON QUIET)
if(NOT BISON_FOUND)
    find_program(YACC_EXECUTABLE NAMES yacc byacc DOC "path to the yacc executable")
    mark_as_advanced(YACC_EXECUTABLE)
    if(NOT YACC_EXECUTABLE)
        message(SEND_ERROR "AVRDUDE requires 'bison', 'yacc', or 'byacc' to build the configuration parser.")
    endif()
endif()

# =====================================
# Smart Dependency Discovery Pattern
# =====================================
find_package(PkgConfig QUIET)

if(USE_STATIC_LIBS)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a .lib)
endif()

# Unified finder macro
macro(find_avrdude_lib prefix pkg_name fallback_lib)
    set(HAVE_${prefix} FALSE)
    if(PKG_CONFIG_FOUND AND NOT USE_EXTERNAL_LIBS)
        pkg_check_modules(${prefix} IMPORTED_TARGET GLOBAL QUIET ${pkg_name})
        if(${prefix}_FOUND)
            set(HAVE_${prefix} TRUE)
            set(TARGET_${prefix} PkgConfig::${prefix})
        endif()
    endif()

    if(NOT HAVE_${prefix} AND NOT USE_EXTERNAL_LIBS)
        find_library(LIB_${prefix} NAMES ${fallback_lib})
        if(LIB_${prefix})
            set(HAVE_${prefix} TRUE)
            add_library(Fallback::${prefix} INTERFACE IMPORTED GLOBAL)
            target_link_libraries(Fallback::${prefix} INTERFACE ${LIB_${prefix}})
            set(TARGET_${prefix} Fallback::${prefix})
        endif()
    endif()
endmacro()

# Resolve core libelf system targets
find_avrdude_lib(LIBELF "libelf" "elf")

# Handle setups where libelf headers reside inside a subpath
if(HAVE_LIBELF)
    check_include_file(elf.h HAVE_ELF_H)
    check_include_file(libelf.h HAVE_LIBELF_H)
    check_include_file(libelf/libelf.h HAVE_LIBELF_LIBELF_H)

    if(NOT HAVE_LIBELF_H AND NOT HAVE_ELF_H)
        find_path(LIBELF_INCLUDE_DIR NAMES libelf.h PATH_SUFFIXES libelf)
        if(LIBELF_INCLUDE_DIR)
            set(HAVE_LIBELF_H TRUE)
            set(LIBELF_CUSTOM_DIR "${LIBELF_INCLUDE_DIR}")
        endif()
    endif()
endif()

# 1. Probing for modern libusb-1.0 engine
find_avrdude_lib(LIBUSB_1_0 "libusb-1.0>=1.0.16" "usb-1.0")

# 2. Probing for legacy libusb-0.1 engine / libusb-compat-0.1
find_avrdude_lib(LIBUSB "libusb" "usb")

# FreeBSD Native fallback mapping
if(NOT HAVE_LIBUSB AND CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
    find_library(FREEBSD_USB NAMES usb)
    if(FREEBSD_USB)
        set(HAVE_LIBUSB TRUE)
        add_library(Fallback::LIBUSB INTERFACE IMPORTED GLOBAL)
        target_link_libraries(Fallback::LIBUSB INTERFACE ${FREEBSD_USB})
        set(TARGET_LIBUSB Fallback::LIBUSB)
    endif()
endif()

# =====================================
# Dual-API Headers Check Layer
# =====================================
if(HAVE_LIBUSB)
    check_include_file(usb.h HAVE_USB_H)
    if(NOT HAVE_USB_H)
        find_path(LIBUSB_COMPAT_INCLUDE_DIR NAMES usb.h PATH_SUFFIXES libusb-compat libusb)
        if(LIBUSB_COMPAT_INCLUDE_DIR)
            set(HAVE_USB_H TRUE)
            set(LIBUSB_COMPAT_DIR "${LIBUSB_COMPAT_INCLUDE_DIR}")
        endif()
    endif()
endif()

if(HAVE_LIBUSB_1_0)
    check_include_file(libusb.h HAVE_LIBUSB_H)
    check_include_file(libusb-1.0/libusb.h HAVE_LIBUSB_1_0_LIBUSB_H)
endif()

# Resolve remaining peripheral libraries
find_avrdude_lib(LIBHIDAPI "hidapi-libusb" "hidapi-libusb;hidapi")
find_avrdude_lib(LIBFTDI1 "libftdi1" "ftdi1")
find_avrdude_lib(LIBSERIALPORT "libserialport" "serialport")
find_avrdude_lib(LIBREADLINE "readline" "readline")

if(HAVE_LIBREADLINE)
    find_library(LIB_NCURSES NAMES ncurses pdcurses)
endif()

# =====================================
# Linux GPIO / libgpiod Logic
# =====================================
if(HAVE_LINUXGPIO AND PKG_CONFIG_FOUND)
    pkg_check_modules(LIBGPIOD IMPORTED_TARGET GLOBAL QUIET libgpiod>=2.0)
    if(LIBGPIOD_FOUND)
        set(HAVE_LIBGPIOD_V2 TRUE)
    else()
        pkg_check_modules(LIBGPIOD IMPORTED_TARGET GLOBAL QUIET libgpiod>=1.6)
        if(LIBGPIOD_FOUND)
            set(HAVE_LIBGPIOD_V1_6 TRUE)
        else()
            pkg_check_modules(LIBGPIOD IMPORTED_TARGET GLOBAL QUIET libgpiod)
        endif()
    endif()
endif()

# =====================================
# SWIG / Python3 Environment
# =====================================
if(NOT FORCE_DISABLE_PYTHON_SUPPORT)
    find_package(SWIG 4.0 COMPONENTS python)
    if(SWIG_FOUND)
        set(Python3_FIND_STRATEGY LOCATION)
        set(Python3_FIND_UNVERSIONED_NAMES FIRST)
        find_package(Python3 COMPONENTS Interpreter Development)
        if(Python3_FOUND)
            set(HAVE_SWIG TRUE)
            execute_process(
                COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])" 
                OUTPUT_VARIABLE PYTHON_SITE_PACKAGES 
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
        endif()
    endif()
endif()

# =====================================
# FetchContent Fallback Infrastructure
# =====================================
if(USE_EXTERNAL_LIBS)
    message(STATUS "Fetching external dependencies via FetchContent...")
    
    macro(fetch_avrdude_fork name repo tag)
        FetchContent_Declare(${name} GIT_REPOSITORY ${repo} GIT_TAG ${tag})
        FetchContent_MakeAvailable(${name})
        
        # Match the exact casing passed to the macro (e.g., ${LIBUSB_SOURCE_DIR})
        set(FETCHED_${name}_BASE "${${name}_SOURCE_DIR}" CACHE INTERNAL "Fetched source path for ${name}")
    endmacro()

    fetch_avrdude_fork(LIBELF "https://github.com/avrdudes/libelf.git" "e5a39bf19bd6598c42e09172be5a78ceec2a065c")
    fetch_avrdude_fork(LIBUSB "https://github.com/avrdudes/libusb.git" "632bc25d04eff563cc00de29435b9a7ed6f4654c")
    fetch_avrdude_fork(LIBHIDAPI "https://github.com/avrdudes/libhidapi.git" "d1307487973c857bc158e27ecc99644b2f5e68ea")
    fetch_avrdude_fork(LIBFTDI "https://github.com/avrdudes/libftdi.git" "f9fe6e96b97c3a08efd081632c1859cb83aa14e3")

    # Map variables uniformly based on the /include design pattern across Marius's forks
    set(FETCHED_LIBELF_DIR  "${FETCHED_LIBELF_BASE}/include"  CACHE INTERNAL "")
    set(FETCHED_LIBUSB_DIR  "${FETCHED_LIBUSB_BASE}/include"  CACHE INTERNAL "")
    set(FETCHED_HIDAPI_DIR  "${FETCHED_LIBHIDAPI_BASE}/include" CA INTERNAL "")
    set(FETCHED_LIBFTDI_DIR "${FETCHED_LIBFTDI_BASE}/include" CACHE INTERNAL "")
endif()

add_subdirectory(src)

if(BUILD_DOC)
    add_subdirectory(src/doc)
endif()

# =====================================
# Configuration Summary
# =====================================
string(APPEND SUMMARY "\n------------------------------------------------------\n"
                     " AVRDUDE Configuration Summary (Coexistence Layout):\n"
                     "------------------------------------------------------\n"
                     "  Version       : ${AVRDUDE_FULL_VERSION}\n"
                     "  libelf        : ${HAVE_LIBELF} (HAVE_ELF_H: ${HAVE_ELF_H} / HAVE_LIBELF_H: ${HAVE_LIBELF_H})\n"
                     "  libusb-1.0    : ${HAVE_LIBUSB_1_0}\n"
                     "  libusb-0.1    : ${HAVE_LIBUSB} (HAVE_USB_H: ${HAVE_USB_H})\n"
                     "  libhidapi     : ${HAVE_LIBHIDAPI}\n"
                     "  libftdi1      : ${HAVE_LIBFTDI1}\n"
                     "  libserialport : ${HAVE_LIBSERIALPORT}\n"
                     "  libreadline   : ${HAVE_LIBREADLINE}\n"
                     "  linuxgpio     : ${HAVE_LINUXGPIO} (libgpiod: ${LIBGPIOD_FOUND})\n"
                     "  Python API    : ${HAVE_SWIG}\n"
                     "------------------------------------------------------\n")
message(STATUS "${SUMMARY}")
