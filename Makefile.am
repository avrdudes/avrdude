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

AM_MAKEINFOFLAGS =
AM_MAKEINFOHTMLFLAGS =
BUILT_SOURCES =
CLEANFILES =
EXTRA_DIST =
bin_PROGRAMS =
lib_LIBRARIES =
lib_LTLIBRARIES =
noinst_LIBRARIES =
noinst_LTLIBRARIES =

include build-helpers/versioninfo.mk
include include.mk
include src/include.mk
include src/doc/include.mk
include tools/include.mk

all-local: html pdf ps

install-data-local: install-html install-pdf install-ps

ACLOCAL_AMFLAGS = -I auto-m4 -I build-helpers
