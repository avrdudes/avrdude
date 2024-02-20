# -*- makefile-automake -*-
#
# FIXME: Move top-level include.mk into Makefile.am

EXTRA_DIST += .editorconfig
EXTRA_DIST += CMakeLists.txt
EXTRA_DIST += CMakeSettings.json
EXTRA_DIST += bootstrap
EXTRA_DIST += autotools-build.sh
EXTRA_DIST += build.sh

EXTRA_DIST += AUTHORS
EXTRA_DIST += COPYING
EXTRA_DIST += INSTALL
EXTRA_DIST += NEWS
EXTRA_DIST += README.md

# Ship spec file to make "rpmbuild -ta avrdude-*.tar.*" command work
EXTRA_DIST += avrdude.spec

GIT_ARCHIVE_IGNORE =
GIT_ARCHIVE_IGNORE += .github/
GIT_ARCHIVE_IGNORE += .gitattributes
GIT_ARCHIVE_IGNORE += .gitignore
GIT_ARCHIVE_IGNORE += atmel-docs/
GIT_ARCHIVE_IGNORE += auto-aux/
GIT_ARCHIVE_IGNORE += auto-m4/
GIT_ARCHIVE_IGNORE += src/doc/.gitignore
GIT_ARCHIVE_IGNORE += supporting-docs/

DIST_ARCHIVE_IGNORE =
DIST_ARCHIVE_IGNORE += @VERSIONINFO_STAMPFILE@
DIST_ARCHIVE_IGNORE += Makefile.in
DIST_ARCHIVE_IGNORE += aclocal.m4
DIST_ARCHIVE_IGNORE += auto-aux/
DIST_ARCHIVE_IGNORE += auto-m4/
DIST_ARCHIVE_IGNORE += avrdude.spec
DIST_ARCHIVE_IGNORE += configure
DIST_ARCHIVE_IGNORE += src/doc/stamp-vti

# FIXME: What to do about these?
DIST_ARCHIVE_IGNORE += src/ac_cfg.h.in
DIST_ARCHIVE_IGNORE += src/config_gram.c
DIST_ARCHIVE_IGNORE += src/config_gram.h
DIST_ARCHIVE_IGNORE += src/doc/avrdude.info
DIST_ARCHIVE_IGNORE += src/doc/parts.texi
DIST_ARCHIVE_IGNORE += src/doc/programmer_types.texi
DIST_ARCHIVE_IGNORE += src/doc/programmers.texi
DIST_ARCHIVE_IGNORE += src/doc/version.texi
DIST_ARCHIVE_IGNORE += src/lexer.c

distcheck-hook:
if HAVE_GIT
if HAVE_DIFF
	set -ex; if test -d "$(top_srcdir)/.git/refs/heads"; then \
	  $(GIT) archive --prefix=$(distdir)/ -o $(distdir).tar.gz HEAD; \
	  rm -rf git-archive dist-archive; \
	  mkdir -p git-archive dist-archive; \
	  (cd git-archive && tar xf ../$(distdir).tar.gz); \
	  (cd dist-archive && tar xf ../$(distdir).tar.xz); \
	  (cd git-archive/$(distdir) && for i in $(GIT_ARCHIVE_IGNORE); do \
	     rm -rf "$$i"; \
	   done); \
	  (cd dist-archive/$(distdir) && for i in $(DIST_ARCHIVE_IGNORE); do \
	     rm -rf "$$i"; \
	   done); \
	  export LC_ALL=C; \
	  echo "diff -u GIT-ARCHIVE DIST-ARCHIVE"; \
	  diff -u <(cd git-archive/ && find $(distdir) | sort) <(cd dist-archive/ && find $(distdir) | sort); \
	  diff -ruN git-archive/$(distdir)/ dist-archive/$(distdir)/; \
	fi
endif
endif
