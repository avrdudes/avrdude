#
# avrdude - A Downloader/Uploader for AVR device programmers
# Copyright (C) 2003  Theodore A. Roth  <troth@openavr.org>
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

EXTRA_DIST   = avrdude.1 avrdude.pdf avrdude.conf.sample bootstrap

AM_YFLAGS    = -d

AM_CPPFLAGS     = -DCONFIG_DIR=\"$(sysconfdir)\"

bin_PROGRAMS = avrdude

avrdude_SOURCES = \
	config_gram.y \
	lexer.l \
	avr.c \
	avr.h \
	avrpart.h \
	config.c \
	config.h \
	fileio.c \
	fileio.h \
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
	stk500.c \
	stk500.h \
	stk500_private.h \
	term.c \
	term.h

man_MANS = avrdude.1

sysconf_DATA = avrdude.conf

avrdude.conf: avrdude.conf.sample
	cp $(srcdir)/avrdude.conf.sample $@

distclean-local:
	rm -rf avrdude.conf
