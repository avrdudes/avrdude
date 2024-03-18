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

# =====================================
# Set up flex target
# =====================================

FLEX_TARGET(Parser "lexer.l" "${PROJECT_BINARY_DIR}/lexer.c")

if (FLEX_VERSION VERSION_GREATER_EQUAL 2.5.9)
    # print the version of flex
    message(STATUS "Flex version: ${FLEX_VERSION}")
    set(HAVE_YYLEX_DESTROY 1)
endif()

# =====================================
# Set up yacc/bison target
# =====================================

if(BISON_FOUND)
    BISON_TARGET(Parser config_gram.y "${PROJECT_BINARY_DIR}/config_gram.cpp" DEFINES_FILE "${PROJECT_BINARY_DIR}/config_gram.h")
else()
    set(YACC_TARGET_outputs "${PROJECT_BINARY_DIR}/config_gram.c")
    add_custom_command(OUTPUT ${YACC_TARGET_outputs}
            COMMAND ${YACC_EXECUTABLE} -d -o ${YACC_TARGET_outputs} config_gram.y
            VERBATIM
            COMMENT "[YACC][Parser] Building parser with yacc"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    set(BISON_Parser_OUTPUTS ${YACC_TARGET_outputs})
endif()

# =====================================
# Setup target specific options
# =====================================

include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})
add_compile_definitions(CONFIG_DIR=\"${CONFIG_DIR}\")

set(EXTRA_WINDOWS_INCLUDES)
set(EXTRA_WINDOWS_RESOURCES)
set(EXTRA_WINDOWS_LIBRARIES)

if(WIN32 OR MINGW)
    list(APPEND EXTRA_WINDOWS_RESOURCES "${PROJECT_BINARY_DIR}/src/windows.rc")
    list(APPEND EXTRA_WINDOWS_LIBRARIES setupapi hid ws2_32)
endif()

if(MSVC)
    enable_language(CXX)

    add_compile_definitions(_CRT_SECURE_NO_WARNINGS=1)
    add_compile_definitions(_CRT_NONSTDC_NO_WARNINGS=1)
    add_compile_definitions(_WINSOCK_DEPRECATED_NO_WARNINGS=1)
    add_compile_options(/W3)
    add_compile_options(/wd4018) # warning C4018: signed/unsigned mismatch
    add_compile_options(/wd4244) # warning C4244: conversion from '...' to '...', possible loss of data
    add_compile_options(/wd4267) # warning C4267: conversion from '...' to '...', possible loss of data

    list(APPEND EXTRA_WINDOWS_SOURCES
            "msvc/getopt.c"
            "msvc/gettimeofday.c"
            "msvc/usleep.cpp"
            "msvc/readline.cpp"
    )
    list(APPEND EXTRA_WINDOWS_INCLUDES "msvc")
else()
    set(LIB_MATH m)
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# =====================================
# Setup default port names
# =====================================

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(DEFAULT_PAR_PORT "/dev/parport0")
    set(DEFAULT_SER_PORT "/dev/ttyS0")
    set(DEFAULT_LINUXGPIO_PORT "gpiochip0")
elseif (CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
    set(DEFAULT_PAR_PORT "/dev/ppi0")
    set(DEFAULT_SER_PORT "/dev/cuad0")
    set(DEFAULT_LINUXGPIO_PORT "unknown")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Solaris")
    set(DEFAULT_PAR_PORT "/dev/printers/0")
    set(DEFAULT_SER_PORT "/dev/term/a")
    set(DEFAULT_LINUXGPIO_PORT "unknown")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(DEFAULT_PAR_PORT "lpt1")
    set(DEFAULT_SER_PORT "com1")
    set(DEFAULT_LINUXGPIO_PORT "unknown")
else()
    set(DEFAULT_PAR_PORT "unknown")
    set(DEFAULT_SER_PORT "unknown")
    set(DEFAULT_LINUXGPIO_PORT "unknown")
endif()

# =====================================
# Configure files
# =====================================

configure_file(cmake_config.h.in ac_cfg.h)
configure_file(avrdude.spec.in avrdude.spec)
if(WIN32 OR MINGW)
    configure_file(windows.rc.in windows.rc)
endif()

add_custom_command(
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/avrdude.conf
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/avrdude.conf.in" avrdude.conf.in
        COMMAND ${CMAKE_COMMAND}
        -D "AVRDUDE_FULL_VERSION=${AVRDUDE_FULL_VERSION}"
        -D HAVE_PARPORT=$<BOOL:${HAVE_PARPORT}>
        -D HAVE_LINUXSPI=$<BOOL:${HAVE_LINUXSPI}>
        -D HAVE_LINUXGPIO=$<BOOL:${HAVE_LINUXGPIO}>
        -D DEFAULT_PAR_PORT=${DEFAULT_PAR_PORT}
        -D DEFAULT_SER_PORT=${DEFAULT_SER_PORT}
        -D DEFAULT_LINUXGPIO_PORT=${DEFAULT_LINUXGPIO_PORT}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/configure.cmake"
        DEPENDS avrdude.conf.in
        VERBATIM
)

add_custom_target(conf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/avrdude.conf)

# =====================================
# Project
# =====================================
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC --bind -fpermissive")


message("FLEX_Parser_OUTPUTS: ${FLEX_Parser_OUTPUTS}")
add_library(libavrdude
        ac_cfg.h
        arduino.h
        arduino.cpp
        avr.cpp
        avr910.cpp
        avr910.h
        avrcache.cpp
        avrdude.h
        avrftdi.cpp
        avrftdi.h
        avrftdi_private.h
        avrftdi_tpi.c
        avrftdi_tpi.h
        avrintel.cpp
        libavrdude-avrintel.h
        avrpart.cpp
        bitbang.cpp
        bitbang.h
        buspirate.cpp
        buspirate.h
        butterfly.cpp
        butterfly.h
        ch341a.cpp
        ch341a.h
        config.cpp
        config.h
        confwin.c
        crc16.c
        crc16.h
        dfu.c
        dfu.h
        dryrun.cpp
        dryrun.h
        dryrun_private.h
        fileio.cpp
        flip1.cpp
        flip1.h
        flip2.cpp
        flip2.h
        freebsd_ppi.h
        ft245r.cpp
        ft245r.h
        jtagmkI.cpp
        jtagmkI.h
        jtagmkI_private.h
        jtagmkII.cpp
        jtagmkII.h
        jtagmkII_private.h
        jtag3.cpp
        jtag3.h
        jtag3_private.h
        leds.cpp
        libavrdude.h
        linuxgpio.cpp
        linuxgpio.h
        linuxspi.cpp
        linuxspi.h
        linux_ppdev.h
        lists.cpp
        micronucleus.cpp
        micronucleus.h
        par.cpp
        par.h
        pgm.cpp
        pgm_type.cpp
        pickit2.cpp
        pickit2.h
        pindefs.cpp
        ppi.cpp
        ppi.h
        ppiwin.c
        serbb.h
        serbb_posix.cpp
        ser_avrdoper.cpp
        ser_posix.cpp
        serialadapter.cpp
        serialupdi.cpp
        serialupdi.h
        solaris_ecpp.h
        stk500.cpp
        stk500.h
        stk500_private.h
        stk500v2.cpp
        stk500v2.h
        stk500v2_private.h
        stk500generic.cpp
        stk500generic.h
        strutil.cpp
        teensy.cpp
        teensy.h
        term.cpp
        tpi.h
        updi_constants.h
        updi_link.cpp
        updi_link.h
        updi_nvm.cpp
        updi_nvm.h
        updi_nvm_v0.cpp
        updi_nvm_v0.h
        updi_nvm_v2.cpp
        updi_nvm_v2.h
        updi_nvm_v3.cpp
        updi_nvm_v3.h
        updi_nvm_v4.cpp
        updi_nvm_v4.h
        updi_nvm_v5.cpp
        updi_nvm_v5.h
        updi_readwrite.cpp
        updi_readwrite.h
        updi_state.c
        updi_state.h
        urclock.cpp
        urclock.h
        urclock_hash.h
        urclock_private.h
        usbasp.cpp
        usbasp.h
        usbdevs.h
        usb_hidapi.c
        usb_libusb.c
        usbtiny.h
        usbtiny.cpp
        update.cpp
        wiring.h
        wiring.cpp
        xbee.h
        xbee.cpp
        lexer.cpp
        ${BISON_Parser_OUTPUTS}
        ${EXTRA_WINDOWS_SOURCES}
)

# require the Parser target to be built before libavrdude

target_link_options(libavrdude
        PUBLIC
        --bind
        -fPIC
)

target_compile_options(libavrdude
        PUBLIC
        -fPIC
        --bind
)

set_target_properties(libavrdude PROPERTIES
        PREFIX ""
        PUBLIC_HEADER "libavrdude.h;libavrdude-avrintel.h"
        VERSION 2.0.0
        SOVERSION 2
)

target_include_directories(libavrdude
        PUBLIC
        "${PROJECT_SOURCE_DIR}"
        "${PROJECT_BINARY_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${LIBUSB_COMPAT_DIR}"
        ${EXTRA_WINDOWS_INCLUDES}
        ../libserial
)

target_link_libraries(libavrdude
        PUBLIC
        ${LIB_MATH}
        ${LIB_LIBELF}
        ${LIB_LIBUSB}
        ${LIB_LIBUSB_1_0}
        ${LIB_LIBHID}
        ${LIB_LIBHIDAPI}
        ${LIB_LIBFTDI}
        ${LIB_LIBFTDI1}
        ${LIB_LIBREADLINE}
        #${LIB_LIBSERIALPORT}
        ${LIB_NCURSES}
        ${LIB_LIBGPIOD}
        ${EXTRA_WINDOWS_LIBRARIES}
)

add_dependencies(libavrdude conf)

