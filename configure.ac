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

# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.57)
AC_INIT(avrdude, 4.4.0cvs, avrdude-dev@nongnu.org)

AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_CANONICAL_TARGET

AC_CONFIG_SRCDIR([main.c])
AM_INIT_AUTOMAKE
AM_CONFIG_HEADER([ac_cfg.h]) 

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_YACC
AM_PROG_LEX

# Checks for libraries.
AC_CHECK_LIB([termcap], [tputs])
AC_CHECK_LIB([ncurses], [tputs])
AC_CHECK_LIB([readline], [readline])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([fcntl.h limits.h stdlib.h string.h sys/ioctl.h sys/time.h termios.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_HEADER_TIME

# Checks for library functions.
AC_PROG_GCC_TRADITIONAL
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset select strcasecmp strdup strerror strncasecmp strtol strtoul])

# Checks for misc stuff.

AC_ARG_ENABLE(
	[versioned-doc],
	AC_HELP_STRING(
		[--enable-versioned-doc],
		[install docs in directory with version name (default)]),
	[case "${enableval}" in
		yes) versioned_doc=yes ;;
		no)  versioned_doc=no ;;
		*)   AC_MSG_ERROR(bad value ${enableval} for versioned-doc option) ;;
		esac],
	[versioned_doc=yes])

if test "$versioned_doc" = "yes"; then
    DOC_INST_DIR='$(DESTDIR)$(datadir)/doc/avrdude-$(VERSION)'
else
    DOC_INST_DIR='$(DESTDIR)$(datadir)/doc/avrdude'
fi

AC_ARG_ENABLE(
	[doc],
	AC_HELP_STRING(
		[--enable-doc],
		[Enable building documents(default)]),
	[case "${enableval}" in
		yes) enabled_doc=yes ;;
		no)  enabled_doc=no ;;
		*)   AC_MSG_ERROR(bad value ${enableval} for disable-doc option) ;;
		esac],
	[enabled_doc=no])

if test "$enabled_doc" = "yes"; then

SUBDIRS_AC='doc @WINDOWS_DIRS@'
DIST_SUBDIRS_AC='doc windows'
else

SUBDIRS_AC='@WINDOWS_DIRS@'
DIST_SUBDIRS_AC='windows'
fi

AC_SUBST(DOC_INST_DIR, $DOC_INST_DIR)
AC_SUBST(SUBDIRS_AC, $SUBDIRS_AC)	
AC_SUBST(DIST_SUBDIRS_AC, $DIST_SUBDIRS_AC)


# Find the parallel serial device files based on target system
# If a system doesn't have a PC style parallel, mark it as unknown.
case $target in
	i[[3456]]86-*-linux*)
		DEFAULT_PAR_PORT="/dev/parport0"
		DEFAULT_SER_PORT="/dev/ttyS0"
		;;
	*-*-linux*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="/dev/ttyS0"
		;;
	i[[3456]]86-*-freebsd*)
		DEFAULT_PAR_PORT="/dev/ppi0"
		DEFAULT_SER_PORT="/dev/cuaa0"
		;;
	*-*-freebsd*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="/dev/cuaa0"
		;;
	*-*-msdos* | *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		DEFAULT_PAR_PORT="lpt1"
		DEFAULT_SER_PORT="com1"
		;;
	*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="unknown"
		;;
esac

AC_MSG_CHECKING([for parallel device])
AC_MSG_RESULT([$DEFAULT_PAR_PORT])
AC_SUBST(DEFAULT_PAR_PORT, $DEFAULT_PAR_PORT)

AC_MSG_CHECKING([for serial device])
AC_MSG_RESULT([$DEFAULT_SER_PORT])
AC_SUBST(DEFAULT_SER_PORT, $DEFAULT_SER_PORT)

# See if we need to drop into the windows subdir.
case $target in
        *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		WINDOWS_DIRS="windows"
		CFLAGS="-mno-cygwin -DWIN32NATIVE"
		LDFLAGS="-static"
		;;
esac
AC_SUBST(WINDOWS_DIRS,$WINDOWS_DIRS)

# If we are compiling with gcc, enable all warning and make warnings errors.
if test "$GCC" = yes; then
    ENABLE_WARNINGS="-Wall"
fi
AC_SUBST(ENABLE_WARNINGS,$ENABLE_WARNINGS)

AC_CONFIG_FILES([
	doc/Makefile
	windows/Makefile
	avrdude.spec
	avrdude.conf
	Makefile
])

AC_OUTPUT

