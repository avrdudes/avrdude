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
    set(HAVE_YYLEX_DESTROY 1)
endif()

# =====================================
# Set up yacc/bison target
# =====================================

if(BISON_FOUND)
    BISON_TARGET(Parser config_gram.y "${PROJECT_BINARY_DIR}/config_gram.c" DEFINES_FILE "${PROJECT_BINARY_DIR}/config_gram.h")
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

set(EXTRA_WINDOWS_SOURCES)
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
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-format-zero-length)
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

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/ac_cfg.h"
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/avrdude.spec.in"
  "${CMAKE_CURRENT_BINARY_DIR}/avrdude.spec"
)
if(WIN32 OR MINGW)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/windows.rc.in"
    "${CMAKE_CURRENT_BINARY_DIR}/windows.rc"
  )
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/python/adgui.bat.in"
    "${CMAKE_CURRENT_BINARY_DIR}/avrdude-gui.bat"
  )
else()
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/python/adgui.sh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/avrdude-gui"
  )
endif()

add_custom_command(
    OUTPUT avrdude.conf
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

add_custom_target(conf ALL DEPENDS avrdude.conf)

# =====================================
# Project
# =====================================

set(SOURCES
    ${CMAKE_CURRENT_BINARY_DIR}/ac_cfg.h
    arduino.h
    arduino.c
    avr.c
    avr910.c
    avr910.h
    avrcache.c
    avrdude.h
    avrftdi.c
    avrftdi.h
    avrftdi_private.h
    avrftdi_tpi.c
    avrftdi_tpi.h
    avrintel.c
    libavrdude-avrintel.h
    avr_opcodes.c
    avrpart.c
    bitbang.c
    bitbang.h
    buspirate.c
    buspirate.h
    butterfly.c
    butterfly.h
    ch341a.c
    ch341a.h
    config.c
    config.h
    confwin.c
    crc16.c
    crc16.h
    disasm.c
    dfu.c
    dfu.h
    dryrun.c
    dryrun.h
    dryrun_private.h
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
    leds.c
    libavrdude.h
    linuxgpio.c
    linuxgpio.h
    linuxspi.c
    linuxspi.h
    linux_ppdev.h
    lists.c
    micronucleus.c
    micronucleus.h
    par.c
    par.h
    pgm.c
    pgm_type.c
    pickit2.c
    pickit2.h
    pickit5_lut_dw.c
    pickit5_lut_isp.c
    pickit5_lut_jtag.c
    pickit5_lut_pdi.c
    pickit5_lut_tpi.c
    pickit5_lut_updi.c
    pickit5_lut.h
    pickit5.c
    pickit5.h
    pindefs.c
    ppi.c
    ppi.h
    serbb.h
    serbb_posix.c
    serbb_win32.c
    ser_avrdoper.c
    ser_posix.c
    ser_win32.c
    serialadapter.c
    serialupdi.c
    serialupdi.h
    serprog.c
    serprog.h
    solaris_ecpp.h
    stk500.c
    stk500.h
    stk500_private.h
    stk500v2.c
    stk500v2.h
    stk500v2_private.h
    stk500generic.c
    stk500generic.h
    strutil.c
    teensy.c
    teensy.h
    term.c
    tpi.h
    updi_constants.h
    updi_link.c
    updi_link.h
    updi_nvm.c
    updi_nvm.h
    updi_nvm_v0.c
    updi_nvm_v0.h
    updi_nvm_v2.c
    updi_nvm_v2.h
    updi_nvm_v3.c
    updi_nvm_v3.h
    updi_nvm_v4.c
    updi_nvm_v4.h
    updi_nvm_v5.c
    updi_nvm_v5.h
    updi_readwrite.c
    updi_readwrite.h
    updi_state.c
    updi_state.h
    urclock.c
    urclock.h
    urclock_hash.h
    urclock_private.h
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
    ${EXTRA_WINDOWS_SOURCES}
    )

add_library(libavrdude
    ${SOURCES}
    )

set_target_properties(libavrdude PROPERTIES
    PREFIX ""
    PUBLIC_HEADER "libavrdude.h;libavrdude-avrintel.h"
    VERSION 3.0.0
    SOVERSION 3
    )

target_include_directories(libavrdude
    PUBLIC
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_BINARY_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${LIBUSB_COMPAT_DIR}"
    ${EXTRA_WINDOWS_INCLUDES}
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
    ${LIB_LIBSERIALPORT}
    ${LIB_NCURSES}
    ${LIB_LIBGPIOD}
    ${EXTRA_WINDOWS_LIBRARIES}
    )

add_executable(avrdude
    main.c
    developer_opts.c
    developer_opts.h
    developer_opts_private.h
    whereami.c
    whereami.h
    ${CMAKE_CURRENT_BINARY_DIR}/ac_cfg.h
    ${EXTRA_WINDOWS_RESOURCES}
    )

target_link_libraries(avrdude PUBLIC libavrdude)

if(MINGW)
    target_link_options(avrdude PRIVATE -static)
endif()

if(HAVE_SWIG)
  include (UseSWIG)
  swig_add_library(swig_avrdude LANGUAGE Python SOURCES libavrdude.i ${SOURCES})
  target_include_directories(swig_avrdude
    PUBLIC
    "${Python3_INCLUDE_DIRS}"
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_BINARY_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${LIBUSB_COMPAT_DIR}"
    ${EXTRA_WINDOWS_INCLUDES}
  )
  target_link_libraries(swig_avrdude
    PRIVATE
    ${LIB_MATH}
    ${LIB_LIBELF}
    ${LIB_LIBUSB}
    ${LIB_LIBUSB_1_0}
    ${LIB_LIBHID}
    ${LIB_LIBHIDAPI}
    ${LIB_LIBFTDI}
    ${LIB_LIBFTDI1}
    ${LIB_LIBREADLINE}
    ${LIB_LIBSERIALPORT}
    ${LIB_NCURSES}
    ${LIB_LIBGPIOD}
    ${Python3_LIBRARIES}
    ${EXTRA_WINDOWS_LIBRARIES}
    )
endif()

# =====================================
# Install
# =====================================

install(TARGETS avrdude DESTINATION bin)
install(PROGRAMS "elf2tag" DESTINATION bin)
install(TARGETS libavrdude
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION include COMPONENT dev
    )
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/avrdude.conf" TYPE SYSCONF)
install(FILES "avrdude.1"
	DESTINATION "${CMAKE_INSTALL_MANDIR}/man1"
	)

if(HAVE_SWIG)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
                    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
    install(TARGETS swig_avrdude DESTINATION ${PYTHON_SITE_PACKAGES})
    install(FILES ${CMAKE_BINARY_DIR}/src/swig_avrdude.py DESTINATION ${PYTHON_SITE_PACKAGES})
    install(DIRECTORY python/ DESTINATION ${CMAKE_INSTALL_DATADIR}/avrdude FILES_MATCHING PATTERN "*.ui")
    install(DIRECTORY python/ DESTINATION ${CMAKE_INSTALL_DATADIR}/avrdude FILES_MATCHING PATTERN "*.py")
  if(WIN32 OR MINGW)
    install(PROGRAMS ${CMAKE_BINARY_DIR}/src/avrdude-gui.bat TYPE BIN)
  else()
    install(PROGRAMS ${CMAKE_BINARY_DIR}/src/avrdude-gui TYPE BIN)
  endif()
endif()
