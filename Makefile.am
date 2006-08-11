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
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

#
# $Id$
#

EXTRA_DIST   = \
	ChangeLog \
	ChangeLog-2001 \
	ChangeLog-2002 \
	ChangeLog-2003 \
	avrdude.1 \
	avrdude.spec \
	bootstrap

CLEANFILES = \
	config_gram.c \
	config_gram.h \
	lexer.c

#SUBDIRS      = doc @WINDOWS_DIRS@
#DIST_SUBDIRS = doc windows

SUBDIRS = @SUBDIRS_AC@
DIST_SUBDIRS = @DIST_SUBDIRS_AC@

AM_YFLAGS    = -d

avrdude_CPPFLAGS = -DCONFIG_DIR=\"$(sysconfdir)\"

avrdude_CFLAGS   = @ENABLE_WARNINGS@  

avrdude_LDADD  = @LIBUSB@

bin_PROGRAMS = avrdude

avrdude_SOURCES = \
	config_gram.y \
	lexer.l \
	avr.c \
	avr.h \
	avr910.c \
	avr910.h \
	avrpart.c \
	avrpart.h \
	bitbang.c \
	bitbang.h \
	butterfly.c \
	butterfly.h \
	config.c \
	config.h \
	confwin.c \
	confwin.h \
	crc16.c \
	crc16.h \
	fileio.c \
	fileio.h \
	freebsd_ppi.h \
	jtagmkI.c \
	jtagmkI.h \
	jtagmkI_private.h \
	jtagmkII.c \
	jtagmkII.h \
	jtagmkII_private.h \
	linux_ppdev.h \
	lists.c \
	lists.h \
	main.c \
	par.c \
	par.h \
	pgm.c \
	pgm.h \
	pindefs.h \
	ppi.c \
	ppi.h \
	ppiwin.c \
	safemode.c \
	safemode.h \
	serial.h \
	serbb.h \
	serbb_posix.c \
	serbb_win32.c \
	ser_posix.c \
	ser_win32.c \
	solaris_ecpp.h \
	stk500.c \
	stk500.h \
	stk500_private.h \
	stk500v2.c \
	stk500v2.h \
	stk500v2_private.h \
	term.c \
	term.h \
	usbdevs.h \
	usb_libusb.c

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
