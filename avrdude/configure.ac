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
AC_INIT(avrdude, 5.11cvs, avrdude-dev@nongnu.org)

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
AC_PROG_LEX
AC_PROG_RANLIB

dnl Makefile.am:77: compiling `config_gram.c' with per-target flags requires `AM_PROG_CC_C_O' in `configure.ac'
AM_PROG_CC_C_O

# Checks for libraries.
AC_CHECK_LIB([termcap], [tputs])
AC_CHECK_LIB([ncurses], [tputs])
AC_CHECK_LIB([readline], [readline])
AC_SEARCH_LIBS([gethostent], [nsl])
AC_SEARCH_LIBS([setsockopt], [socket])
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

AH_TEMPLATE([HAVE_LIBUSB_1_0],
            [Define if USB support is enabled via libusb 1.0])
AC_CHECK_LIB([usb-1.0], [libusb_init], [have_libusb_1_0=yes])
if test x$have_libusb_1_0 = xyes; then
   case $target in
       *-*-darwin*)
               LIBUSB_1_0="-lusb-1.0 -framework CoreFoundation -framework IOKit"
               ;;
       *)
               LIBUSB_1_0="-lusb-1.0"
               ;;
   esac
   AC_DEFINE([HAVE_LIBUSB_1_0])
   AC_CHECK_HEADERS([libusb-1.0/libusb.h])
   AC_CHECK_HEADERS([libusb.h])
fi
AH_TEMPLATE([HAVE_LIBUSB_1_0],
            [Define if USB support is enabled via a libusb-1.0 compatible libusb])
AC_CHECK_LIB([usb], [libusb_init], [have_libusb_1_0=yes])
if test x$have_libusb_1_0 = xyes; then
   case $target in
       *-*-freebsd*)
               # FreeBSD 8+ has a native libusb-1.0 API compatible
               # library offered by -lusb (which is also libusb-0.1
               # compatible).  FreeBSD <8 does not have a libusb-1.0
               # at all so probing will fail but we do not have to
               # special-case that.
               LIBUSB_1_0="-lusb"
               ;;
       *)
               LIBUSB_1_0="-lusb-1.0"
               ;;
   esac
   AC_DEFINE([HAVE_LIBUSB_1_0])
   AC_CHECK_HEADERS([libusb.h])
fi
AC_SUBST(LIBUSB_1_0, $LIBUSB_1_0)
AH_TEMPLATE([HAVE_LIBFTDI],
            [Define if FTDI support is enabled via libftdi])
AC_CHECK_LIB([ftdi], [ftdi_init], [have_libftdi=yes], [], [-lusb])
if test x$have_libftdi = xyes; then
   LIBFTDI="-lftdi -lusb"
   AC_DEFINE([HAVE_LIBFTDI])
fi
AC_SUBST(LIBFTDI, $LIBFTDI)
# Checks for header files.
AC_CHECK_HEADERS([limits.h stdlib.h string.h])
AC_CHECK_HEADERS([fcntl.h sys/ioctl.h sys/time.h termios.h unistd.h])
AC_CHECK_HEADERS([ddk/hidsdi.h],,,[#include <windows.h>
#include <setupapi.h>])


# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_HEADER_TIME

# Checks for library functions.
AC_CHECK_FUNCS([memset select strcasecmp strdup strerror strncasecmp strtol strtoul gettimeofday usleep])

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

# Check for types

# Solaris has uint_t and ulong_t typedefs in <sys/types.h>, avoid
# the redeclaration in usbtiny.c.
AC_CHECK_TYPES([uint_t], [], [], [#include <sys/types.h>])
AC_CHECK_TYPES([ulong_t], [], [], [#include <sys/types.h>])

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

DIST_SUBDIRS_AC='doc windows'

if test "$enabled_doc" = "yes"; then
	SUBDIRS_AC='doc'
else
	SUBDIRS_AC=''
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
	i[[3456]]86-*-*freebsd*|amd64-*-*freebsd*)
		DEFAULT_PAR_PORT="/dev/ppi0"
		DEFAULT_SER_PORT="/dev/cuad0"
		;;
	*-*-*freebsd*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="/dev/cuad0"
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

# If we are compiling with gcc, enable all warning and make warnings errors.
if test "$GCC" = yes; then
    ENABLE_WARNINGS="-Wall"

	# does this compiler support -Wno-pointer-sign ?
	AC_MSG_CHECKING([if gcc accepts -Wno-pointer-sign ])

	safe_CFLAGS=$CFLAGS
	CFLAGS="$ENABLE_WARNINGS -Wno-pointer-sign"

	AC_TRY_COMPILE(, [ int main () { return 0 ; } ], [
		no_pointer_sign=yes
		AC_MSG_RESULT([yes]) 
		], [
		no_pointer_sign=no
		AC_MSG_RESULT([no]) 
		])
	CFLAGS=$safe_CFLAGS

	if test x$no_pointer_sign = xyes; then
		ENABLE_WARNINGS="$ENABLE_WARNINGS -Wno-pointer-sign"
	fi
fi
AC_SUBST(ENABLE_WARNINGS,$ENABLE_WARNINGS)

# See if we need to drop into the windows subdir.
case $target in
        *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		if test "$GCC" = yes -a \( "$CC" = "cc" -o "$CC" = "gcc" \); then
			# does this compiler support -mno-cygwin?
			AC_MSG_CHECKING([if $CC accepts -mno-cygwin])

			safe_CFLAGS="$CFLAGS"
			CFLAGS="$ENABLE_WARNINGS -mno-cygwin"

			AC_TRY_COMPILE(, [ int main () { return 0 ; } ], [
				no_cygwin=yes
				AC_MSG_RESULT([yes])
				], [
				no_cygwin=no
				AC_MSG_RESULT([no])
				])
			CFLAGS="$safe_CFLAGS"

			if test x$no_cygwin = xyes; then
				CFLAGS="${CFLAGS} -mno-cygwin"
			else
				AC_MSG_NOTICE([Your compiler does not understand the -mno-cygwin option.])
				AC_MSG_NOTICE([You might want to select an alternative compiler, like])
				AC_MSG_NOTICE([])
				AC_MSG_NOTICE([CC=mingw32-gcc ./configure])
				AC_MSG_NOTICE([])
			fi
		fi

		AC_MSG_CHECKING([if linker accepts -static])

		safe_LDFLAGS="$LDFLAGS"
		LDFLAGS="${LDFLAGS} -static"
		AC_TRY_LINK(, [ int main () { return 0 ; } ], [
			can_link_static=yes
			AC_MSG_RESULT([yes])
			], [
			can_link_static_cygwin=no
			AC_MSG_RESULT([no])
			])
		LDFLAGS="$safe_LDFLAGS"

		if test x$can_link_static = xyes; then
			LDFLAGS="${LDFLAGS} -static"
		fi

		WINDOWS_DIRS="windows"
		CFLAGS="${CFLAGS} -DWIN32NATIVE"
		;;
esac
AC_SUBST(WINDOWS_DIRS,$WINDOWS_DIRS)

AC_CONFIG_FILES([
       doc/Makefile
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
