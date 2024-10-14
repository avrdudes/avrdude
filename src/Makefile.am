
#
# avrdude - A Downloader/Uploader for AVR device programmers
# Copyright (C) 2003, 2004 Theodore A. Roth <troth@openavr.org>
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

EXTRA_DIST   = \
	avrdude.1 \
	avrdude.spec \
	bootstrap

BUILT_SOURCES  =
CLEANFILES     =

built_sources  =
built_sources += config_gram.c
built_sources += config_gram.h
built_sources += lexer.c

BUILT_SOURCES += $(built_sources)
CLEANFILES    += $(built_sources)

include build-helpers/versioninfo.mk

#SUBDIRS      = doc
#DIST_SUBDIRS = doc

# . lets build this directory before the following in SUBDIRS
SUBDIRS = .
# doc comes here, and we want to use the built avrdude to generate the parts list
SUBDIRS += @SUBDIRS_AC@
DIST_SUBDIRS = @DIST_SUBDIRS_AC@

AM_YFLAGS    = -d

avrdude_CPPFLAGS = -DCONFIG_DIR=\"$(sysconfdir)\"

libavrdude_la_CPPFLAGS = -DCONFIG_DIR=\"$(sysconfdir)\"

avrdude_CFLAGS   = @ENABLE_WARNINGS@

libavrdude_la_CFLAGS   = @ENABLE_WARNINGS@ $(LIBGPIOD_CFLAGS)

avrdude_LDADD  = libavrdude.la @LIBUSB_1_0@ @LIBHIDAPI@ @LIBUSB@ @LIBFTDI1@ @LIBFTDI@ @LIBHID@ @LIBELF@ @LIBPTHREAD@ @LIBSERIALPORT@ $(LIBGPIOD_LIBS) -lm

bin_PROGRAMS = avrdude

lib_LTLIBRARIES = libavrdude.la

# automake thinks these generated files should be in the distribution,
# but this might cause trouble for some users, so we rather don't want
# to have them there.
#
# See
#
# https://savannah.nongnu.org/bugs/index.php?func=detailitem&item_id=15536
#
# for why we don't want to have them.
#
# We could avoid this dist-hook altogether if we could require
# Automake >= 1.16.4 and just use its no-dist-built-sources flag.
dist-hook: dist-hook-no-dist-built-sources-workaround
dist-hook-no-dist-built-sources-workaround:
	cd "$(distdir)" && rm -f $(built_sources)

libavrdude_la_SOURCES = \
	config_gram.y \
	lexer.l \
	arduino.h \
	arduino.c \
	avr.c \
	avr910.c \
	avr910.h \
	avrcache.c \
	avrdude.h \
	avrftdi.c \
	avrftdi.h \
	avrftdi_private.h \
	avrftdi_tpi.c \
	avrftdi_tpi.h \
	avrintel.c \
	libavrdude-avrintel.h \
	avrpart.c \
	avr_opcodes.c \
	bitbang.c \
	bitbang.h \
	buspirate.c \
	buspirate.h \
	butterfly.c \
	butterfly.h \
	ch341a.c \
	ch341a.h \
	config.c \
	config.h \
	confwin.c \
	crc16.c \
	crc16.h \
	disasm.c \
	dfu.c \
	dfu.h \
	dryrun.c \
	dryrun.h \
	dryrun_private.h \
	fileio.c \
	flip1.c \
	flip1.h \
	flip2.c \
	flip2.h \
	freebsd_ppi.h \
	ft245r.c \
	ft245r.h \
	jtagmkI.c \
	jtagmkI.h \
	jtagmkI_private.h \
	jtagmkII.c \
	jtagmkII.h \
	jtagmkII_private.h \
	jtag3.c \
	jtag3.h \
	jtag3_private.h \
	leds.c \
	libavrdude.h \
	linuxgpio.c \
	linuxgpio.h \
	linuxspi.c \
	linuxspi.h \
	linux_ppdev.h \
	lists.c \
	micronucleus.c \
	micronucleus.h \
	par.c \
	par.h \
	pgm.c \
	pgm_type.c \
	pickit2.c \
	pickit2.h \
	pickit5_lut_dw.c \
	pickit5_lut_isp.c \
	pickit5_lut_jtag.c \
	pickit5_lut_pdi.c \
	pickit5_lut_tpi.c \
	pickit5_lut_updi.c \
	pickit5_lut.h \
	pickit5.c \
	pickit5.h \
	pindefs.c \
	ppi.c \
	ppi.h \
	serbb.h \
	serbb_posix.c \
	serbb_win32.c \
	ser_avrdoper.c \
	ser_posix.c \
	ser_win32.c \
	serialadapter.c \
	solaris_ecpp.h \
	stk500.c \
	stk500.h \
	stk500_private.h \
	stk500v2.c \
	stk500v2.h \
	stk500v2_private.h \
	stk500generic.c \
	stk500generic.h \
	strutil.c \
	teensy.c \
	teensy.h \
	term.c \
	tpi.h \
	usbasp.c \
	usbasp.h \
	serialupdi.c \
	serialupdi.h \
	serprog.c \
	serprog.h \
	updi_constants.h \
	updi_link.c \
	updi_link.h \
	updi_state.c \
	updi_state.h \
	updi_readwrite.c \
	updi_readwrite.h \
	updi_nvm.c \
	updi_nvm.h \
	updi_nvm_v0.c \
	updi_nvm_v0.h \
	updi_nvm_v2.c \
	updi_nvm_v2.h \
	updi_nvm_v3.c \
	updi_nvm_v3.h \
	updi_nvm_v4.c \
	updi_nvm_v4.h \
	updi_nvm_v5.c \
	updi_nvm_v5.h \
	urclock.c \
	urclock.h \
	urclock_hash.h \
	urclock_private.h \
	usbdevs.h \
	usb_hidapi.c \
	usb_libusb.c \
	usbtiny.h \
	usbtiny.c \
	update.c \
	wiring.h \
	wiring.c \
	xbee.h \
	xbee.c
libavrdude_la_LDFLAGS = -version-info @LIBAVRDUDE_VERSION_INFO@

include_HEADERS = libavrdude.h
include_HEADERS += libavrdude-avrintel.h

avrdude_SOURCES = \
	main.c \
	whereami.c \
	whereami.h \
	developer_opts.c \
	developer_opts.h \
	developer_opts_private.h

dist_bin_SCRIPTS = elf2tag

man_MANS = avrdude.1

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
