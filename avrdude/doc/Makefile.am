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

CLEANFILES = \
	version.texi \
	stamp-vti

info_TEXINFOS = avrdude.texi

all-local: info html ps pdf

html: 
	mkdir -p avrdude-html
	texi2html -split_node $(srcdir)/$(info_TEXINFOS)
	mv *.html avrdude-html

clean-local:
	rm -rf avrdude-html *.info

install-data-local: install-docs

install-docs: html ps pdf
	$(mkinstalldirs) $(DOC_INST_DIR)
	$(INSTALL_DATA) avrdude.ps $(DOC_INST_DIR)/avrdude.ps
	$(INSTALL_DATA) avrdude.pdf $(DOC_INST_DIR)/avrdude.pdf
	$(mkinstalldirs) $(DOC_INST_DIR)/avrdude-html
	@list=`echo avrdude-html/*.html`; \
		for file in $$list; \
		do \
			$(INSTALL_DATA) $$file $(DOC_INST_DIR)/$$file; \
		done

uninstall-local:
	rm -rf $(DOC_INST_DIR)
