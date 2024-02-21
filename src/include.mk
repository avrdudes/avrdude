# -*- makefile-automake -*-
#
# avrdude - A Downloader/Uploader for AVR device programmers
# Copyright (C) 2003, 2004  Theodore A. Roth  <troth@openavr.org>
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


BUILT_SOURCES += %reldir%/config_gram.c
CLEANFILES    += %reldir%/config_gram.c

BUILT_SOURCES += %reldir%/config_gram.h
CLEANFILES    += %reldir%/config_gram.h

BUILT_SOURCES += %reldir%/lexer.c
CLEANFILES    += %reldir%/lexer.c

AM_YFLAGS    = -d

avrdude_conf      = %reldir%/avrdude.conf
avrdude_exe       = %reldir%/avrdude$(EXEEXT)

bin_PROGRAMS     += %reldir%/avrdude
noinst_LIBRARIES += %reldir%/libavrdude.a
lib_LTLIBRARIES  += %reldir%/libavrdude.la

common_cppflags            =
common_cppflags           += -I$(srcdir)/%reldir%
common_cppflags           += '-DCONFIG_DIR="$(sysconfdir)"'
%C%_avrdude_CPPFLAGS       = $(common_cppflags)
%C%_libavrdude_a_CPPFLAGS  = $(common_cppflags)
%C%_libavrdude_la_CPPFLAGS = $(common_cppflags)

common_cflags              = @ENABLE_WARNINGS@
%C%_avrdude_CFLAGS         = $(common_cflags)
%C%_libavrdude_a_CFLAGS    = $(common_cflags)
%C%_libavrdude_la_CFLAGS   = $(common_cflags)

%C%_avrdude_LDADD  =
%C%_avrdude_LDADD += $(top_builddir)/$(noinst_LIBRARIES)
%C%_avrdude_LDADD += @LIBUSB_1_0@
%C%_avrdude_LDADD += @LIBHIDAPI@
%C%_avrdude_LDADD += @LIBUSB@
%C%_avrdude_LDADD += @LIBFTDI1@
%C%_avrdude_LDADD += @LIBFTDI@
%C%_avrdude_LDADD += @LIBHID@
%C%_avrdude_LDADD += @LIBELF@
%C%_avrdude_LDADD += @LIBPTHREAD@
%C%_avrdude_LDADD += @LIBSERIALPORT@
%C%_avrdude_LDADD += -lm

libavrdude_sources  =
libavrdude_sources += %reldir%/arduino.c
libavrdude_sources += %reldir%/arduino.h
libavrdude_sources += %reldir%/avr.c
libavrdude_sources += %reldir%/avr910.c
libavrdude_sources += %reldir%/avr910.h
libavrdude_sources += %reldir%/avrcache.c
libavrdude_sources += %reldir%/avrdude.h
libavrdude_sources += %reldir%/avrftdi.c
libavrdude_sources += %reldir%/avrftdi.h
libavrdude_sources += %reldir%/avrftdi_private.h
libavrdude_sources += %reldir%/avrftdi_tpi.c
libavrdude_sources += %reldir%/avrftdi_tpi.h
libavrdude_sources += %reldir%/avrintel.c
libavrdude_sources += %reldir%/avrpart.c
libavrdude_sources += %reldir%/bitbang.c
libavrdude_sources += %reldir%/bitbang.h
libavrdude_sources += %reldir%/buildinfo.c
libavrdude_sources += %reldir%/buspirate.c
libavrdude_sources += %reldir%/buspirate.h
libavrdude_sources += %reldir%/butterfly.c
libavrdude_sources += %reldir%/butterfly.h
libavrdude_sources += %reldir%/ch341a.c
libavrdude_sources += %reldir%/ch341a.h
libavrdude_sources += %reldir%/config.c
libavrdude_sources += %reldir%/config.h
libavrdude_sources += %reldir%/config_gram.y
libavrdude_sources += %reldir%/confwin.c
libavrdude_sources += %reldir%/crc16.c
libavrdude_sources += %reldir%/crc16.h
libavrdude_sources += %reldir%/dfu.c
libavrdude_sources += %reldir%/dfu.h
libavrdude_sources += %reldir%/dryrun.c
libavrdude_sources += %reldir%/dryrun.h
libavrdude_sources += %reldir%/dryrun_private.h
libavrdude_sources += %reldir%/fileio.c
libavrdude_sources += %reldir%/flip1.c
libavrdude_sources += %reldir%/flip1.h
libavrdude_sources += %reldir%/flip2.c
libavrdude_sources += %reldir%/flip2.h
libavrdude_sources += %reldir%/freebsd_ppi.h
libavrdude_sources += %reldir%/ft245r.c
libavrdude_sources += %reldir%/ft245r.h
libavrdude_sources += %reldir%/jtag3.c
libavrdude_sources += %reldir%/jtag3.h
libavrdude_sources += %reldir%/jtag3_private.h
libavrdude_sources += %reldir%/jtagmkI.c
libavrdude_sources += %reldir%/jtagmkI.h
libavrdude_sources += %reldir%/jtagmkII.c
libavrdude_sources += %reldir%/jtagmkII.h
libavrdude_sources += %reldir%/jtagmkII_private.h
libavrdude_sources += %reldir%/jtagmkI_private.h
libavrdude_sources += %reldir%/leds.c
libavrdude_sources += %reldir%/lexer.l
libavrdude_sources += %reldir%/libavrdude-avrintel.h
libavrdude_sources += %reldir%/libavrdude.h
libavrdude_sources += %reldir%/linux_ppdev.h
libavrdude_sources += %reldir%/linuxgpio.c
libavrdude_sources += %reldir%/linuxgpio.h
libavrdude_sources += %reldir%/linuxspi.c
libavrdude_sources += %reldir%/linuxspi.h
libavrdude_sources += %reldir%/lists.c
libavrdude_sources += %reldir%/micronucleus.c
libavrdude_sources += %reldir%/micronucleus.h
libavrdude_sources += %reldir%/par.c
libavrdude_sources += %reldir%/par.h
libavrdude_sources += %reldir%/pgm.c
libavrdude_sources += %reldir%/pgm_type.c
libavrdude_sources += %reldir%/pickit2.c
libavrdude_sources += %reldir%/pickit2.h
libavrdude_sources += %reldir%/pindefs.c
libavrdude_sources += %reldir%/ppi.c
libavrdude_sources += %reldir%/ppi.h
libavrdude_sources += %reldir%/ppiwin.c
libavrdude_sources += %reldir%/ser_avrdoper.c
libavrdude_sources += %reldir%/ser_posix.c
libavrdude_sources += %reldir%/ser_win32.c
libavrdude_sources += %reldir%/serbb.h
libavrdude_sources += %reldir%/serbb_posix.c
libavrdude_sources += %reldir%/serbb_win32.c
libavrdude_sources += %reldir%/serialadapter.c
libavrdude_sources += %reldir%/serialupdi.c
libavrdude_sources += %reldir%/serialupdi.h
libavrdude_sources += %reldir%/solaris_ecpp.h
libavrdude_sources += %reldir%/stk500.c
libavrdude_sources += %reldir%/stk500.h
libavrdude_sources += %reldir%/stk500_private.h
libavrdude_sources += %reldir%/stk500generic.c
libavrdude_sources += %reldir%/stk500generic.h
libavrdude_sources += %reldir%/stk500v2.c
libavrdude_sources += %reldir%/stk500v2.h
libavrdude_sources += %reldir%/stk500v2_private.h
libavrdude_sources += %reldir%/strutil.c
libavrdude_sources += %reldir%/teensy.c
libavrdude_sources += %reldir%/teensy.h
libavrdude_sources += %reldir%/term.c
libavrdude_sources += %reldir%/tpi.h
libavrdude_sources += %reldir%/update.c
libavrdude_sources += %reldir%/updi_constants.h
libavrdude_sources += %reldir%/updi_link.c
libavrdude_sources += %reldir%/updi_link.h
libavrdude_sources += %reldir%/updi_nvm.c
libavrdude_sources += %reldir%/updi_nvm.h
libavrdude_sources += %reldir%/updi_nvm_v0.c
libavrdude_sources += %reldir%/updi_nvm_v0.h
libavrdude_sources += %reldir%/updi_nvm_v2.c
libavrdude_sources += %reldir%/updi_nvm_v2.h
libavrdude_sources += %reldir%/updi_nvm_v3.c
libavrdude_sources += %reldir%/updi_nvm_v3.h
libavrdude_sources += %reldir%/updi_nvm_v4.c
libavrdude_sources += %reldir%/updi_nvm_v4.h
libavrdude_sources += %reldir%/updi_nvm_v5.c
libavrdude_sources += %reldir%/updi_nvm_v5.h
libavrdude_sources += %reldir%/updi_readwrite.c
libavrdude_sources += %reldir%/updi_readwrite.h
libavrdude_sources += %reldir%/updi_state.c
libavrdude_sources += %reldir%/updi_state.h
libavrdude_sources += %reldir%/urclock.c
libavrdude_sources += %reldir%/urclock.h
libavrdude_sources += %reldir%/urclock_hash.h
libavrdude_sources += %reldir%/urclock_private.h
libavrdude_sources += %reldir%/usb_hidapi.c
libavrdude_sources += %reldir%/usb_libusb.c
libavrdude_sources += %reldir%/usbasp.c
libavrdude_sources += %reldir%/usbasp.h
libavrdude_sources += %reldir%/usbdevs.h
libavrdude_sources += %reldir%/usbtiny.c
libavrdude_sources += %reldir%/usbtiny.h
libavrdude_sources += %reldir%/wiring.c
libavrdude_sources += %reldir%/wiring.h
libavrdude_sources += %reldir%/xbee.c
libavrdude_sources += %reldir%/xbee.h

%C%_libavrdude_a_SOURCES  = $(libavrdude_sources)
%C%_libavrdude_la_SOURCES = $(libavrdude_sources)
%C%_libavrdude_la_LDFLAGS = -version-info 2:0

include_HEADERS  = %reldir%/libavrdude.h
include_HEADERS += %reldir%/libavrdude-avrintel.h

%C%_avrdude_SOURCES  =
%C%_avrdude_SOURCES += %reldir%/main.c
%C%_avrdude_SOURCES += %reldir%/whereami.c
%C%_avrdude_SOURCES += %reldir%/whereami.h
%C%_avrdude_SOURCES += %reldir%/developer_opts.c
%C%_avrdude_SOURCES += %reldir%/developer_opts.h
%C%_avrdude_SOURCES += %reldir%/developer_opts_private.h

dist_man_MANS = %reldir%/avrdude.1

EXTRA_DIST += %reldir%/CMakeLists.txt
EXTRA_DIST += %reldir%/cmake_config.h.in
EXTRA_DIST += %reldir%/configure.cmake
EXTRA_DIST += %reldir%/windows.rc.in
EXTRA_DIST += %reldir%/msvc/getopt.c
EXTRA_DIST += %reldir%/msvc/getopt.h
EXTRA_DIST += %reldir%/msvc/gettimeofday.c
EXTRA_DIST += %reldir%/msvc/msvc_compat.h
EXTRA_DIST += %reldir%/msvc/readline.cpp
EXTRA_DIST += %reldir%/msvc/readline/history.h
EXTRA_DIST += %reldir%/msvc/readline/readline.h
EXTRA_DIST += %reldir%/msvc/sys/time.h
EXTRA_DIST += %reldir%/msvc/unistd.h
EXTRA_DIST += %reldir%/msvc/usleep.cpp

sysconf_DATA = %reldir%/avrdude.conf

install-exec-local: backup-avrdude-conf

distclean-local:
	rm -f %reldir%/avrdude.conf

# This will get run before the config file is installed.
backup-avrdude-conf:
	@echo "Backing up avrdude.conf in ${DESTDIR}${sysconfdir}"
	@if test -e ${DESTDIR}${sysconfdir}/avrdude.conf; then \
		cp -pR ${DESTDIR}${sysconfdir}/avrdude.conf \
			${DESTDIR}${sysconfdir}/avrdude.conf.bak; \
	fi
