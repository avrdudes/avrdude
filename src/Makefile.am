
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

# In order to get verbose build messages rather than a summary line
# only, either use
#
# ./configure --disable-silent-rules
#
# or run make like
#
# make V=1

#
# $Id$
#

BUILT_SOURCES =
CLEANFILES =
EXTRA_DIST =
bin_PROGRAMS =
lib_LIBRARIES =
lib_LTLIBRARIES =
noinst_LIBRARIES =
noinst_LTLIBRARIES =


EXTRA_DIST += avrdude.spec
EXTRA_DIST += bootstrap


include build-helpers/versioninfo.mk

BUILT_SOURCES += config_gram.c
CLEANFILES    += config_gram.c

BUILT_SOURCES += config_gram.h
CLEANFILES    += config_gram.h

BUILT_SOURCES += lexer.c
CLEANFILES    += lexer.c


# . lets build this directory before the following in SUBDIRS
SUBDIRS = .
# doc comes here, and we want to use the built avrdude to generate the parts list
SUBDIRS += doc

AM_YFLAGS    = -d

bin_PROGRAMS     += avrdude
noinst_LIBRARIES += libavrdude.a
lib_LTLIBRARIES  += libavrdude.la

common_cppflags        = '-DCONFIG_DIR="$(sysconfdir)"'
avrdude_CPPFLAGS       = $(common_cppflags)
libavrdude_a_CPPFLAGS  = $(common_cppflags)
libavrdude_la_CPPFLAGS = $(common_cppflags)

common_cflags          = @ENABLE_WARNINGS@
avrdude_CFLAGS         = $(common_cflags)
libavrdude_a_CFLAGS    = $(common_cflags)
libavrdude_la_CFLAGS   = $(common_cflags)

avrdude_LDADD  =
avrdude_LDADD += $(top_builddir)/$(noinst_LIBRARIES)
avrdude_LDADD += @LIBUSB_1_0@
avrdude_LDADD += @LIBHIDAPI@
avrdude_LDADD += @LIBUSB@
avrdude_LDADD += @LIBFTDI1@
avrdude_LDADD += @LIBFTDI@
avrdude_LDADD += @LIBHID@
avrdude_LDADD += @LIBELF@
avrdude_LDADD += @LIBPTHREAD@
avrdude_LDADD += @LIBSERIALPORT@
avrdude_LDADD += -lm

libavrdude_sources  =
libavrdude_sources += arduino.c
libavrdude_sources += arduino.h
libavrdude_sources += avr.c
libavrdude_sources += avr910.c
libavrdude_sources += avr910.h
libavrdude_sources += avrcache.c
libavrdude_sources += avrdude.h
libavrdude_sources += avrftdi.c
libavrdude_sources += avrftdi.h
libavrdude_sources += avrftdi_private.h
libavrdude_sources += avrftdi_tpi.c
libavrdude_sources += avrftdi_tpi.h
libavrdude_sources += avrintel.c
libavrdude_sources += avrpart.c
libavrdude_sources += bitbang.c
libavrdude_sources += bitbang.h
libavrdude_sources += buspirate.c
libavrdude_sources += buspirate.h
libavrdude_sources += butterfly.c
libavrdude_sources += butterfly.h
libavrdude_sources += ch341a.c
libavrdude_sources += ch341a.h
libavrdude_sources += config.c
libavrdude_sources += config.h
libavrdude_sources += config_gram.y
libavrdude_sources += confwin.c
libavrdude_sources += crc16.c
libavrdude_sources += crc16.h
libavrdude_sources += dfu.c
libavrdude_sources += dfu.h
libavrdude_sources += dryrun.c
libavrdude_sources += dryrun.h
libavrdude_sources += dryrun_private.h
libavrdude_sources += fileio.c
libavrdude_sources += flip1.c
libavrdude_sources += flip1.h
libavrdude_sources += flip2.c
libavrdude_sources += flip2.h
libavrdude_sources += freebsd_ppi.h
libavrdude_sources += ft245r.c
libavrdude_sources += ft245r.h
libavrdude_sources += jtag3.c
libavrdude_sources += jtag3.h
libavrdude_sources += jtag3_private.h
libavrdude_sources += jtagmkI.c
libavrdude_sources += jtagmkI.h
libavrdude_sources += jtagmkII.c
libavrdude_sources += jtagmkII.h
libavrdude_sources += jtagmkII_private.h
libavrdude_sources += jtagmkI_private.h
libavrdude_sources += leds.c
libavrdude_sources += lexer.l
libavrdude_sources += libavrdude-avrintel.h
libavrdude_sources += libavrdude.h
libavrdude_sources += linux_ppdev.h
libavrdude_sources += linuxgpio.c
libavrdude_sources += linuxgpio.h
libavrdude_sources += linuxspi.c
libavrdude_sources += linuxspi.h
libavrdude_sources += lists.c
libavrdude_sources += micronucleus.c
libavrdude_sources += micronucleus.h
libavrdude_sources += par.c
libavrdude_sources += par.h
libavrdude_sources += pgm.c
libavrdude_sources += pgm_type.c
libavrdude_sources += pickit2.c
libavrdude_sources += pickit2.h
libavrdude_sources += pindefs.c
libavrdude_sources += ppi.c
libavrdude_sources += ppi.h
libavrdude_sources += ppiwin.c
libavrdude_sources += ser_avrdoper.c
libavrdude_sources += ser_posix.c
libavrdude_sources += ser_win32.c
libavrdude_sources += serbb.h
libavrdude_sources += serbb_posix.c
libavrdude_sources += serbb_win32.c
libavrdude_sources += serialadapter.c
libavrdude_sources += serialupdi.c
libavrdude_sources += serialupdi.h
libavrdude_sources += solaris_ecpp.h
libavrdude_sources += stk500.c
libavrdude_sources += stk500.h
libavrdude_sources += stk500_private.h
libavrdude_sources += stk500generic.c
libavrdude_sources += stk500generic.h
libavrdude_sources += stk500v2.c
libavrdude_sources += stk500v2.h
libavrdude_sources += stk500v2_private.h
libavrdude_sources += strutil.c
libavrdude_sources += teensy.c
libavrdude_sources += teensy.h
libavrdude_sources += term.c
libavrdude_sources += tpi.h
libavrdude_sources += update.c
libavrdude_sources += updi_constants.h
libavrdude_sources += updi_link.c
libavrdude_sources += updi_link.h
libavrdude_sources += updi_nvm.c
libavrdude_sources += updi_nvm.h
libavrdude_sources += updi_nvm_v0.c
libavrdude_sources += updi_nvm_v0.h
libavrdude_sources += updi_nvm_v2.c
libavrdude_sources += updi_nvm_v2.h
libavrdude_sources += updi_nvm_v3.c
libavrdude_sources += updi_nvm_v3.h
libavrdude_sources += updi_nvm_v4.c
libavrdude_sources += updi_nvm_v4.h
libavrdude_sources += updi_nvm_v5.c
libavrdude_sources += updi_nvm_v5.h
libavrdude_sources += updi_readwrite.c
libavrdude_sources += updi_readwrite.h
libavrdude_sources += updi_state.c
libavrdude_sources += updi_state.h
libavrdude_sources += urclock.c
libavrdude_sources += urclock.h
libavrdude_sources += urclock_hash.h
libavrdude_sources += urclock_private.h
libavrdude_sources += usb_hidapi.c
libavrdude_sources += usb_libusb.c
libavrdude_sources += usbasp.c
libavrdude_sources += usbasp.h
libavrdude_sources += usbdevs.h
libavrdude_sources += usbtiny.c
libavrdude_sources += usbtiny.h
libavrdude_sources += wiring.c
libavrdude_sources += wiring.h
libavrdude_sources += xbee.c
libavrdude_sources += xbee.h

libavrdude_a_SOURCES  = $(libavrdude_sources)
libavrdude_la_SOURCES = $(libavrdude_sources)
libavrdude_la_LDFLAGS = -version-info 2:0

include_HEADERS = libavrdude.h
include_HEADERS += libavrdude-avrintel.h

avrdude_SOURCES  =
avrdude_SOURCES += main.c
avrdude_SOURCES += whereami.c
avrdude_SOURCES += whereami.h
avrdude_SOURCES += developer_opts.c
avrdude_SOURCES += developer_opts.h
avrdude_SOURCES += developer_opts_private.h

dist_man_MANS = avrdude.1

sysconf_DATA = avrdude.conf

install-exec-local: backup-avrdude-conf

distclean-local:
	rm -f avrdude.conf

# This will get run before the config file is installed.
backup-avrdude-conf:
	@echo "Backing up avrdude.conf in ${DESTDIR}${sysconfdir}"
	@if test -e ${DESTDIR}${sysconfdir}/avrdude.conf; then \
		cp -pR ${DESTDIR}${sysconfdir}/avrdude.conf \
			${DESTDIR}${sysconfdir}/avrdude.conf.bak; \
	fi

ACLOCAL_AMFLAGS = -I auto-m4 -I build-helpers
