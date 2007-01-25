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
AC_INIT(avrdude, 5.3cvs, avrdude-dev@nongnu.org)

AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_CANONICAL_TARGET

AC_CONFIG_SRCDIR([main.c])
AM_INIT_AUTOMAKE
AM_CONFIG_HEADER(ac_cfg.h)

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_YACC
AM_PROG_LEX
AC_PROG_RANLIB

# Checks for libraries.
AC_CHECK_LIB([termcap], [tputs])
AC_CHECK_LIB([ncurses], [tputs])
AC_CHECK_LIB([readline], [readline])
AH_TEMPLATE([HAVE_LIBUSB],
            [Define if USB support is enabled via libusb])
AC_CHECK_LIB([usb], [usb_get_string_simple], [have_libusb=yes])
if test x$have_libusb = xyes; then
   case $target in
       *-*-darwin*)
               LIBUSB="-lusb -framework CoreFoundation -framework IOKit"
               ;;
       *)
               LIBUSB="-lusb"
               ;;
   esac
   AC_DEFINE([HAVE_LIBUSB])
fi
AC_SUBST(LIBUSB, $LIBUSB)

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([limits.h stdlib.h string.h])
AC_CHECK_HEADERS([fcntl.h sys/ioctl.h sys/time.h termios.h unistd.h])
AC_CHECK_HEADERS([ddk/hidsdi.h],,,[#include <windows.h>
#include <setupapi.h>])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_HEADER_TIME

# Checks for library functions.
AC_PROG_GCC_TRADITIONAL
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset select strcasecmp strdup strerror strncasecmp strtol strtoul gettimeofday])

AC_MSG_CHECKING([for a Win32 HID libray])
SAVED_LIBS="${LIBS}"
case $target in
        *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		LIBHID="-lhid -lsetupapi"
		if test $ac_cv_header_ddk_hidsdi_h = yes
		then
			HIDINCLUDE="#include <ddk/hidsdi.h>"
		else
			HIDINCLUDE="#include \"my_ddk_hidsdi.h\""
		fi
		;;
	*)
		LIBHID=""
		;;
esac
LIBS="${LIBS} ${LIBHID}"

AH_TEMPLATE([HAVE_LIBHID],
            [Define if HID support is enabled via the Win32 DDK])
AC_TRY_RUN([#include <windows.h>
#include <setupapi.h>
$HIDINCLUDE

int
main(void)
{
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);

    return 0;
}
], [have_libhid=yes], [have_libhid=no], [have_libhid=no])
AC_MSG_RESULT([$have_libhid])
if test x$have_libhid = xyes; then
   AC_DEFINE([HAVE_LIBHID])
else
   LIBHID=""
fi
LIBS="${SAVED_LIBS}"
AC_SUBST(LIBHID, $LIBHID)

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
		[Enable building documents]),
	[case "${enableval}" in
		yes) enabled_doc=yes ;;
		no)  enabled_doc=no ;;
		*)   AC_MSG_ERROR(bad value ${enableval} for disable-doc option) ;;
		esac],
	[enabled_doc=no])

AC_ARG_ENABLE(
	[parport],
	AC_HELP_STRING(
		[--enable-parport],
		[Enable accessing parallel ports(default)]),
	[case "${enableval}" in
		yes) enabled_parport=yes ;;
		no)  enabled_parport=no ;;
		*)   AC_MSG_ERROR(bad value ${enableval} for enable-parport option) ;;
		esac],
	[enabled_parport=yes])

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
	i[[3456]]86-*-linux*|x86_64-*-linux*)
		DEFAULT_PAR_PORT="/dev/parport0"
		DEFAULT_SER_PORT="/dev/ttyS0"
		;;
	*-*-linux*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="/dev/ttyS0"
		;;
	i[[3456]]86-*-freebsd*|amd64-*-freebsd*)
		DEFAULT_PAR_PORT="/dev/ppi0"
		DEFAULT_SER_PORT="/dev/cuaa0"
		;;
	*-*-freebsd*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="/dev/cuaa0"
		;;
	*-*-solaris*)
		DEFAULT_PAR_PORT="/dev/printers/0"
		DEFAULT_SER_PORT="/dev/term/a"
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

if test "$enabled_parport" = "yes"; then
	AC_MSG_CHECKING([for parallel device])
	if test "$DEFAULT_PAR_PORT" = "unknown"; then
		AC_MSG_NOTICE([parallel port access disabled for this system])
		enabled_parport=no
	else
		AC_MSG_RESULT([$DEFAULT_PAR_PORT])
	fi
	AC_SUBST(DEFAULT_PAR_PORT, $DEFAULT_PAR_PORT)
fi

AC_MSG_CHECKING([for serial device])
AC_MSG_RESULT([$DEFAULT_SER_PORT])
AC_SUBST(DEFAULT_SER_PORT, $DEFAULT_SER_PORT)

if test "$enabled_parport" = "yes"; then
	AC_DEFINE(HAVE_PARPORT, 1, [parallel port access enabled])
	confsubst="-e /^@HAVE_PARPORT_/d"
else
	confsubst="-e /^@HAVE_PARPORT_BEGIN@/,/^@HAVE_PARPORT_END@/d"
fi
export confsubst

# See if we need to drop into the windows subdir.
case $target in
        *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		WINDOWS_DIRS="windows"
		CFLAGS="${CFLAGS} -mno-cygwin -DWIN32NATIVE"
		LDFLAGS="${LDFLAGS} -static"
		;;
esac
AC_SUBST(WINDOWS_DIRS,$WINDOWS_DIRS)

# If we are compiling with gcc, enable all warning and make warnings errors.
if test "$GCC" = yes; then
    ENABLE_WARNINGS="-Wall"
fi
AC_SUBST(ENABLE_WARNINGS,$ENABLE_WARNINGS)

if test "$enabled_doc" = "yes"; then
AC_CONFIG_FILES([doc/Makefile])
fi

AC_CONFIG_FILES([
       windows/Makefile
       avrdude.spec
       Makefile
])

# The procedure to create avrdude.conf involves two steps.  First,
# normal autoconf substitution will be applied, resulting in
# avrdude.conf.tmp.  Finally, a sed command will be applied to filter
# out unwanted parts (currently the parallel port programmer types)
# based on previous configuration results, thereby producing the final
# avrdude.conf file.

AC_CONFIG_FILES([avrdude.conf.tmp:avrdude.conf.in],
		[sed $confsubst avrdude.conf.tmp > avrdude.conf])

AC_OUTPUT
