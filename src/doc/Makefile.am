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
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

#
# $Id$
#

AM_MAKEINFOFLAGS =
AM_MAKEINFOHTMLFLAGS =
CLEANFILES =
EXTRA_DIST =

EXTRA_DIST      += avrdude.texi
info_TEXINFOS    = avrdude.texi
CLEANFILES      += avrdude.info
avrdude_TEXINFOS =

AM_MAKEINFOHTMLFLAGS += --split=node

EXTRA_DIST           += avrdude.css
AM_MAKEINFOHTMLFLAGS += --css-include=$(srcdir)/avrdude.css

EXTRA_DIST += parts_comments.txt

# if it does not exist make this first
../avrdude$(EXEEXT):
	cd .. && $(MAKE) avrdude$(EXEEXT)

../avrdude.conf:
	cd .. && $(MAKE) avrdude.conf

CLEANFILES       += $(builddir)/programmers.texi
avrdude_TEXINFOS += $(builddir)/programmers.texi
$(builddir)/programmers.texi: ../avrdude$(EXEEXT) ../avrdude.conf Makefile
	../avrdude$(EXEEXT) -C ../avrdude.conf -c '?' 2>&1 \
	| $(AWK) '$$2 ~ /^=$$/ {printf("@item @code{%s} @tab %s\n",$$1,gensub("[^=]+=[ \t]*","",1))}' \
	| sed "s# *,\? *<\?\(http://[^ \t>]*\)>\?#,@*\n@url{\1}#g" \
	>programmers.texi

CLEANFILES       += $(builddir)/programmer_types.texi
avrdude_TEXINFOS += $(builddir)/programmer_types.texi
$(builddir)/programmer_types.texi: ../avrdude$(EXEEXT) ../avrdude.conf Makefile
	../avrdude$(EXEEXT) -C ../avrdude.conf -c '?type' 2>&1 \
	| $(AWK) '$$2 ~ /^=$$/ {printf("@item @code{%s} @tab %s\n",$$1,gensub("[^=]+=[ \t]*","",1))}' \
	| sed "s#<\?\(http://[^ \t,>]*\)>\?#@url{\1}#g" \
	>programmer_types.texi

CLEANFILES       += $(builddir)/parts.texi
avrdude_TEXINFOS += $(builddir)/parts.texi
$(builddir)/parts.texi: ../avrdude$(EXEEXT) ../avrdude.conf parts_comments.txt Makefile
	../avrdude$(EXEEXT) -C ../avrdude.conf -p '?' 2>&1 \
	| $(AWK) '$$2 ~ /^=$$/ {printf("@item @code{%s} @tab %s\n",$$1,$$3)}' \
	| sed -e "`sed 's:\([^ \t]*\)[ \t]*\(.*\):s/\1$$/\1 \2/g:g' <parts_comments.txt`" \
	>parts.texi

all-local: html pdf ps

install-data-local: install-html install-pdf install-ps
