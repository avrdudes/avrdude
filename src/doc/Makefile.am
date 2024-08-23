#
# avrdude - A Downloader/Uploader for AVR device programmers
# Copyright (C) 2003 Theodore A. Roth <troth@openavr.org>
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

GENERATED_TEXINFOS = \
	$(builddir)/programmers.texi \
	$(builddir)/parts.texi \
	$(builddir)/programmer_types.texi \
	$(builddir)/avrstats.texi \
	$(builddir)/version.texi

CLEANFILES = \
	$(GENERATED_TEXINFOS) \
	$(builddir)/stamp-vti

info_TEXINFOS = avrdude.texi

EXTRA_DIST = \
	parts.sed \
	programmers.sed

all-local: info html pdf

html: avrdude-html/avrdude.html

avrdude-html/avrdude.html: $(srcdir)/$(info_TEXINFOS) $(GENERATED_TEXINFOS)
	texi2html --split=node --css-include=$(srcdir)/avrdude.css $(srcdir)/$(info_TEXINFOS)
	if [ -e ./avrdude.html -o -e ./avrdude_1.html ]; then \
	 mkdir -p avrdude-html ; \
	 mv -f *.html avrdude-html ; \
	else \
	 rm -rf avrdude-html; \
	 mv -f avrdude avrdude-html; \
	fi;

$(builddir)/avrdude.info: $(GENERATED_TEXINFOS)
$(builddir)/avrdude.dvi: $(GENERATED_TEXINFOS)
$(builddir)/avrdude.pdf: $(GENERATED_TEXINFOS)

# if it does not exist make this first
../avrdude$(EXEEXT):
	$(MAKE) -C .. avrdude$(EXEEXT)

$(builddir)/programmers.texi: ../avrdude$(EXEEXT) ../avrdude.conf $(srcdir)/programmers.sed Makefile
	../avrdude$(EXEEXT) -C ../avrdude.conf -c \? 2>&1 \
	|  grep = | sed -f $(srcdir)/programmers.sed \
	>programmers.texi

$(builddir)/programmer_types.texi: ../avrdude$(EXEEXT) ../avrdude.conf Makefile
	../avrdude$(EXEEXT) -C ../avrdude.conf -c \?type 2>&1 \
	| $(AWK) '$$2 ~ /^=$$/ {printf("@item @code{%s} @tab %s\n",$$1,gensub("[^=]+=[ \t]*","",1))}' \
	| sed "s#<\?\(http://[^ \t,>]*\)>\?#@url{\1}#g" \
	>programmer_types.texi

$(builddir)/parts.texi: ../avrdude$(EXEEXT) ../avrdude.conf $(srcdir)/parts.sed Makefile
	../avrdude$(EXEEXT) -C ../avrdude.conf -p \? 2>&1 \
	| grep = | sed -f $(srcdir)/parts.sed \
	>parts.texi

$(builddir)/avrstats.texi: ../avrdude$(EXEEXT) ../avrdude.conf Makefile
	echo -n "@set NUMPARTS " >> avrstats.texi
	../avrdude$(EXEEXT) -C ../avrdude.conf -p \? 2>&1 | grep = | wc -l >> avrstats.texi
	echo -n "@set NUMPROGRAMMERS " >> avrstats.texi
	../avrdude$(EXEEXT) -C ../avrdude.conf -c \? 2>&1 | grep = | wc -l >> avrstats.texi

clean-local:
	rm -rf avrdude-html *.info

install-data-local: install-docs

install-docs: html pdf
	$(mkinstalldirs) $(DOC_INST_DIR)
	$(INSTALL_DATA) avrdude.pdf $(DOC_INST_DIR)/avrdude.pdf
	$(mkinstalldirs) $(DOC_INST_DIR)/avrdude-html
	@list=`echo avrdude-html/*.html`; \
		for file in $$list; \
		do \
			$(INSTALL_DATA) $$file $(DOC_INST_DIR)/$$file; \
		done

uninstall-local:
	rm -rf $(DOC_INST_DIR)
