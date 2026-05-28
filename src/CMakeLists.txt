#
# src/CMakeLists.txt - CMake project for AVRDUDE
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

# src/CMakeLists.txt - Modernized Inner-Source Targets

# =====================================
# Parser Rules (Flex & Bison/Yacc)
# =====================================
FLEX_TARGET(Parser "lexer.l" "${PROJECT_BINARY_DIR}/lexer.c")
if (FLEX_VERSION VERSION_GREATER_EQUAL 2.5.9)
    set(HAVE_YYLEX_DESTROY 1)
endif()

if(BISON_FOUND)
    BISON_TARGET(Parser config_gram.y "${PROJECT_BINARY_DIR}/config_gram.c" DEFINES_FILE "${PROJECT_BINARY_DIR}/config_gram.h")
    set(PARSER_SOURCES ${BISON_Parser_OUTPUTS})
else()
    set(YACC_OUT "${PROJECT_BINARY_DIR}/config_gram.c")
    add_custom_command(
        OUTPUT ${YACC_OUT}
        COMMAND ${YACC_EXECUTABLE} -d -o ${YACC_OUT} config_gram.y
        DEPENDS config_gram.y
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "[YACC] Building parser with legacy yacc replacement"
        VERBATIM
    )
    set(PARSER_SOURCES ${YACC_OUT})
endif()

# =====================================
# Target Windows Platforms Properties
# =====================================
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
    list(APPEND EXTRA_WINDOWS_SOURCES
        "msvc/getopt.c"
        "msvc/gettimeofday.c"
        "msvc/usleep.cpp"
        "msvc/readline.cpp"
    )
    list(APPEND EXTRA_WINDOWS_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/msvc")
else()
    set(LIB_MATH m)
endif()

# =====================================
# Define OS Default Target Ports
# =====================================
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(DEFAULT_PAR_PORT "/dev/parport0")
    set(DEFAULT_SER_PORT "/dev/ttyS0")
    set(DEFAULT_SPI_PORT "/dev/spidev0.0:/dev/gpiochip0")
    set(DEFAULT_LINUXGPIO_PORT "gpiochip0")
elseif (CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
    set(DEFAULT_PAR_PORT "/dev/ppi0")
    set(DEFAULT_SER_PORT "/dev/cuad0")
    set(DEFAULT_SPI_PORT "unknown")
    set(DEFAULT_LINUXGPIO_PORT "unknown")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(DEFAULT_PAR_PORT "lpt1")
    set(DEFAULT_SER_PORT "com1")
    set(DEFAULT_SPI_PORT "unknown")
    set(DEFAULT_LINUXGPIO_PORT "unknown")
else()
    set(DEFAULT_PAR_PORT "unknown")
    set(DEFAULT_SER_PORT "unknown")
    set(DEFAULT_SPI_PORT "unknown")
    set(DEFAULT_LINUXGPIO_PORT "unknown")
endif()

# =====================================
# Generate Configuration Files
# =====================================
configure_file("cmake_config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ac_cfg.h")
configure_file("avrdude.spec.in" "${CMAKE_CURRENT_BINARY_DIR}/avrdude.spec")

if(WIN32 OR MINGW)
    configure_file("windows.rc.in" "${CMAKE_CURRENT_BINARY_DIR}/windows.rc")
    configure_file("python/adgui.bat.in" "${CMAKE_CURRENT_BINARY_DIR}/avrdude-gui.bat")
else()
    configure_file("python/adgui.sh.in" "${CMAKE_CURRENT_BINARY_DIR}/avrdude-gui")
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
        -D DEFAULT_SPI_PORT=${DEFAULT_SPI_PORT}
        -D DEFAULT_LINUXGPIO_PORT=${DEFAULT_LINUXGPIO_PORT}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/configure.cmake"
    DEPENDS avrdude.conf.in
    VERBATIM
)
add_custom_target(conf ALL DEPENDS avrdude.conf)

# =====================================
# Build Targets: libavrdude Core
# =====================================
set(SOURCES
    arduino.c avr.c avr910.c avrcache.c avrftdi.c avrftdi_tpi.c avrintel.c
    avr_opcodes.c avrpart.c bitbang.c buspirate.c butterfly.c ch341a.c
    config.c confwin.c crc16.c disasm.c dfu.c dryrun.c fileio.c flip1.c
    flip2.c ft245r.c jtagmkI.c jtagmkII.c jtag3.c leds.c linuxgpio.c
    linuxspi.c lists.c micronucleus.c par.c pgm.c pgm_type.c pickit2.c
    pickit5_lut_dw.c pickit5_lut_isp.c pickit5_lut_jtag.c pickit5_lut_pdi.c
    pickit5_lut_tpi.c pickit5_lut_updi.c pickit5.c pindefs.c ppi.c
    serbb_posix.c serbb_win32.c ser_avrdoper.c ser_posix.c ser_win32.c
    serialadapter.c serialupdi.c serprog.c stk500.c stk500v2.c stk500generic.c
    strutil.c teensy.c term.c updi_link.c updi_nvm.c updi_nvm_v0.c updi_nvm_v2.c
    updi_nvm_v3.c updi_nvm_v4.c updi_nvm_v5.c updi_nvm_v6.c updi_readwrite.c
    updi_state.c urbootlist.c urbootautogen.c urclock.c usbasp.c usb_hidapi.c
    usb_libusb.c usbtiny.c update.c wiring.c xbee.c
    ${FLEX_Parser_OUTPUTS}
    ${PARSER_SOURCES}
    ${EXTRA_WINDOWS_SOURCES}
)

add_library(libavrdude ${SOURCES})

set_target_properties(libavrdude PROPERTIES
    PREFIX ""
    PUBLIC_HEADER "libavrdude.h;libavrdude-avrintel.h"
    VERSION 4.0.0
    SOVERSION 4
    POSITION_INDEPENDENT_CODE ON
)

# Apply global feature macro overrides directly to target compilation definitions
if(HAVE_LIBELF)
    target_compile_definitions(libavrdude PRIVATE HAVE_LIBELF=1)
endif()
if(HAVE_ELF_H)
    target_compile_definitions(libavrdude PRIVATE HAVE_ELF_H=1)
endif()
if(HAVE_LIBELF_H)
    target_compile_definitions(libavrdude PRIVATE HAVE_LIBELF_H=1)
endif()
if(HAVE_LIBELF_LIBELF_H)
    target_compile_definitions(libavrdude PRIVATE HAVE_LIBELF_LIBELF_H=1)
endif()

# Force macro assignments down to files like avrftdi.c
if(HAVE_LIBUSB)
    target_compile_definitions(libavrdude PRIVATE HAVE_LIBUSB=1)
endif()
if(HAVE_USB_H)
    target_compile_definitions(libavrdude PRIVATE HAVE_USB_H=1)
endif()
if(HAVE_LIBUSB_1_0)
    target_compile_definitions(libavrdude PRIVATE HAVE_LIBUSB_1_0=1)
endif()

if(NOT MSVC)
    target_compile_options(libavrdude PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-format-zero-length)
else()
    target_compile_definitions(libavrdude PRIVATE _CRT_SECURE_NO_WARNINGS=1 _CRT_NONSTDC_NO_WARNINGS=1 _WINSOCK_DEPRECATED_NO_WARNINGS=1)
    target_compile_options(libavrdude PRIVATE /W3 /wd4018 /wd4244 /wd4267)
endif()

if(USE_EXTERNAL_LIBS AND FETCHED_LIBUSB_DIR)
    target_include_directories(libavrdude SYSTEM PRIVATE "${FETCHED_LIBUSB_DIR}")
endif()

target_include_directories(libavrdude
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
        ${PROJECT_BINARY_DIR}
        ${EXTRA_WINDOWS_INCLUDES}
        ${LIBUSB_COMPAT_DIR}
        ${LIBELF_CUSTOM_DIR}
        ${FETCHED_LIBELF_DIR}
        ${FETCHED_HIDAPI_DIR}
        ${FETCHED_LIBFTDI_DIR}
)

target_link_libraries(libavrdude
    PRIVATE
        ${LIB_MATH}
        ${TARGET_LIBELF}
        ${TARGET_LIBUSB_1_0}
        ${TARGET_LIBUSB}
        ${TARGET_LIBHIDAPI}
        ${TARGET_LIBFTDI1}
        ${TARGET_LIBSERIALPORT}
        ${TARGET_LIBREADLINE}
        ${LIB_NCURSES}
        ${TARGET_LIBGPIOD}
        ${EXTRA_WINDOWS_LIBRARIES}
)

# =====================================
# Build Targets: Main Binary Executable
# =====================================
add_executable(avrdude main.c developer_opts.c whereami.c ${EXTRA_WINDOWS_RESOURCES})
target_link_libraries(avrdude PRIVATE libavrdude)
target_include_directories(avrdude PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

if(MINGW)
    target_link_options(avrdude PRIVATE -static)
endif()

# =====================================
# Clean SWIG Integration Target
# =====================================
if(HAVE_SWIG)
    include(UseSWIG)
    set_property(SOURCE libavrdude.i PROPERTY CPLUSPLUS OFF)
    
    swig_add_library(swig_avrdude LANGUAGE Python SOURCES libavrdude.i)
    
    target_link_libraries(swig_avrdude PRIVATE libavrdude Python3::Python)
    target_include_directories(swig_avrdude PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${LIBUSB_COMPAT_DIR})
endif()

# =====================================
# Installation Specifications
# =====================================
install(TARGETS avrdude DESTINATION ${CMAKE_INSTALL_BINDIR})
install(PROGRAMS "elf2tag" DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS libavrdude
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT dev
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/avrdude.conf" DESTINATION ${CMAKE_INSTALL_SYSCONFDIR})
install(FILES "avrdude.1" DESTINATION "${CMAKE_INSTALL_MANDIR}/man1")

if(HAVE_SWIG)
    install(TARGETS swig_avrdude DESTINATION ${PYTHON_SITE_PACKAGES})
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/swig_avrdude.py DESTINATION ${PYTHON_SITE_PACKAGES})
    install(DIRECTORY python/ DESTINATION ${CMAKE_INSTALL_DATADIR}/avrdude FILES_MATCHING PATTERN "*.ui")
    install(DIRECTORY python/ DESTINATION ${CMAKE_INSTALL_DATADIR}/avrdude FILES_MATCHING PATTERN "*.py")
    if(WIN32 OR MINGW)
        install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/avrdude-gui.bat DESTINATION ${CMAKE_INSTALL_BINDIR})
    else()
        install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/avrdude-gui DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
endif()
