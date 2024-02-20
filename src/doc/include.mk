# -*- makefile-automake -*-
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

EXTRA_DIST          += %reldir%/CMakeLists.txt
EXTRA_DIST          += %reldir%/parts.cmake
EXTRA_DIST          += %reldir%/programmer_types.cmake
EXTRA_DIST          += %reldir%/programmers.cmake

EXTRA_DIST          += %reldir%/avrdude.texi
info_TEXINFOS        = %reldir%/avrdude.texi
CLEANFILES          += %reldir%/avrdude.info
%C%_avrdude_TEXINFOS =

AM_MAKEINFOFLAGS     += -I$(builddir)/%reldir%
AM_MAKEINFOHTMLFLAGS += -I$(builddir)/%reldir%
AM_MAKEINFOFLAGS     += -I$(srcdir)/%reldir%
AM_MAKEINFOHTMLFLAGS += -I$(srcdir)/%reldir%

AM_MAKEINFOHTMLFLAGS += --split=node

EXTRA_DIST           += %reldir%/avrdude.css
AM_MAKEINFOHTMLFLAGS += --css-include=$(srcdir)/%reldir%/avrdude.css

EXTRA_DIST += %reldir%/parts_comments.txt

# FIXME: Do not run avrdude_exe if cross-compiling.

CLEANFILES           += %reldir%/programmers.texi
%C%_avrdude_TEXINFOS += %reldir%/programmers.texi
%reldir%/programmers.texi: $(avrdude_exe) $(avrdude_conf) Makefile
	@$(MKDIR_P) %reldir%
	$(avrdude_exe) -C $(avrdude_conf) -c '?' 2>&1 \
	| $(AWK) '$$2 ~ /^=$$/ {printf("@item @code{%s} @tab %s\n",$$1,gensub("[^=]+=[ \t]*","",1))}' \
	| sed "s# *,\? *<\?\(http://[^ \t>]*\)>\?#,@*\n@url{\1}#g" \
	>%reldir%/programmers.texi

CLEANFILES           += %reldir%/programmer_types.texi
%C%_avrdude_TEXINFOS += %reldir%/programmer_types.texi
%reldir%/programmer_types.texi: $(avrdude_exe) $(avrdude_conf) Makefile
	@$(MKDIR_P) %reldir%
	$(avrdude_exe) -C $(avrdude_conf) -c '?type' 2>&1 \
	| $(AWK) '$$2 ~ /^=$$/ {printf("@item @code{%s} @tab %s\n",$$1,gensub("[^=]+=[ \t]*","",1))}' \
	| sed "s#<\?\(http://[^ \t,>]*\)>\?#@url{\1}#g" \
	>%reldir%/programmer_types.texi

# FIXME: srcdir or builddir for parts_comments.txt?
CLEANFILES           += %reldir%/parts.texi
%C%_avrdude_TEXINFOS += %reldir%/parts.texi
%reldir%/parts.texi: $(avrdude_exe) $(avrdude_conf) $(srcdir)/%reldir%/parts_comments.txt Makefile
	@$(MKDIR_P) %reldir%
	$(avrdude_exe) -C $(avrdude_conf) -p '?' 2>&1 \
	| $(AWK) '$$2 ~ /^=$$/ {printf("@item @code{%s} @tab %s\n",$$1,$$3)}' \
	| sed -e "`sed 's:\([^ \t]*\)[ \t]*\(.*\):s/\1$$/\1 \2/g:g' < $(srcdir)/%reldir%/parts_comments.txt`" \
	>%reldir%/parts.texi
