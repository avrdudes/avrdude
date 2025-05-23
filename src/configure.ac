#
# avrdude - A Downloader/Uploader for AVR device programmers
# Copyright (C) 2003, 2004 Theodore A. Roth <troth@openavr.org>
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

# Process this file with autoreconf to produce a configure script.

dnl A few tool releases with release dates for orientation which tool
dnl requirements can make sense:
dnl
dnl 2019-03-14 cmake    3.13
dnl
dnl 2006-10-23 autoconf 2.60    used to be avrdude's requirement
dnl 2012-04-24 autoconf 2.69
dnl 2021-01-28 autoconf 2.71
dnl
dnl 2013-06-20 automake 1.14
dnl 2018-02-25 automake 1.16
dnl 2018-03-11 automake 1.16.1  install-sh symlink fix


dnl Define the dist tarball name as
dnl   * avrdude-7.3 for releases
dnl   * avrdude-7.3-${COMMIT_DATE} for snapshots
dnl just like CMakeLists.txt does.
m4_define([versioninfo_AVRDUDE_PACKAGE_VERSION],
          m4_case(m4_defn([versioninfo_GIT_COMMIT_HASH]),
	          m4_defn([versioninfo_GIT_TAG_HASH]),
	          m4_defn([versioninfo_CMAKE_PROJECT_VERSION]),
		  m4_defn([versioninfo_CMAKE_PROJECT_VERSION])-m4_defn([versioninfo_GIT_COMMIT_DATE])))

AC_PREREQ([2.69])
AC_INIT([avrdude],
        m4_defn([versioninfo_AVRDUDE_PACKAGE_VERSION]),
        [https://github.com/avrdudes/avrdude/issues])

AC_CONFIG_SRCDIR([main.c])
AC_CONFIG_AUX_DIR([auto-aux])
AC_CONFIG_HEADERS([ac_cfg.h])
AC_CONFIG_MACRO_DIR([auto-m4])

AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_CANONICAL_TARGET

AM_INIT_AUTOMAKE([
  1.14
  foreign
  subdir-objects
])

AM_SILENT_RULES([yes])

dnl Set up the macro definitions for versioninfo_* which we hand to AC_INIT.
dnl
dnl Yes, it does work to expand the VERSIONINFO_SETUP macro *after* AC_INIT:
dnl The presence of VERSIONINFO_SETUP pulls in the definitions at the top.
dnl
dnl For details, see the build-helpers/versioninfo.{md,m4,mk,sh} files.
m4_pattern_forbid([VERSIONINFO_SETUP])dnl
VERSIONINFO_SETUP()

dnl Inform about the derived PACKAGE_VERSION
AC_MSG_CHECKING([versioninfo derived PACKAGE_VERSION])
AC_MSG_RESULT([$PACKAGE_VERSION])

dnl Compose the full version message mirroring the cmake one, inform
dnl about it and pass it on to avrdude.conf and the "avrdude -?"
dnl version message
AC_ARG_VAR([EXTRA_VERSION_SUFFIX],
           [extra version information to be appended to the version])
AC_MSG_CHECKING([versioninfo derived AVRDUDE_FULL_VERSION])
if test "x$GIT_COMMIT_HASH" = "x$GIT_TAG_HASH"; then
	AVRDUDE_FULL_VERSION="$PACKAGE_VERSION$EXTRA_VERSION_SUFFIX"
else
	AVRDUDE_FULL_VERSION="$PACKAGE_VERSION$EXTRA_VERSION_SUFFIX ($GIT_COMMIT_HASH)"
fi
AC_MSG_RESULT([$AVRDUDE_FULL_VERSION])
AC_DEFINE_UNQUOTED([AVRDUDE_FULL_VERSION], ["$AVRDUDE_FULL_VERSION"],
                   [The full avrdude version as displayed in -? and avrdude.conf])
AC_SUBST([AVRDUDE_FULL_VERSION])


# Define libavrdude libtool version from cmake libavrdude information
dnl
dnl This is a very simple consistency check. If this check ever fails,
dnl the library versioning policy and its technical implementation
dnl must be adapted.
m4_case(m4_defn([versioninfo_CMAKE_LIBAVRDUDE_SOVERSION])[.0.0],
        m4_defn([versioninfo_CMAKE_LIBAVRDUDE_VERSION]),
	[],
	[m4_fatal([
The cmake libavrdude VERSION should be SOVERSION.0.0 ("]m4_defn([versioninfo_CMAKE_LIBAVRDUDE_SOVERSION])[.0.0"),
but it is actually "]m4_defn([versioninfo_CMAKE_LIBAVRDUDE_VERSION])[".

This is an internal error in the logic which transfers library version
information from the cmake buildsystem to the automake buildsystem.

Please file a GitHub issue for avrdude and mention @ndim.
	])])dnl
dnl
AC_MSG_CHECKING([versioninfo derived libtool -version-info for libavrdude])
libavrdude_lt_cur=$CMAKE_LIBAVRDUDE_SOVERSION
libavrdude_lt_rev=0
libavrdude_lt_age=0
AC_SUBST([LIBAVRDUDE_VERSION_INFO],
         [${libavrdude_lt_cur}:${libavrdude_lt_rev}:${libavrdude_lt_age}])
AC_MSG_RESULT([$LIBAVRDUDE_VERSION_INFO])


# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_AWK
AC_PROG_SED
AC_PROG_YACC
AC_PROG_LEX([noyywrap])
AM_PROG_AR

m4_pattern_forbid([LT_INIT])dnl
LT_INIT()

dnl If macro PKG_PROG_PKG_CONFIG is not available, Autoconf generates
dnl a misleading error message, so check for existence first, and
dnl otherwise provide helpful advice.
m4_ifndef([PKG_PROG_PKG_CONFIG], [m4_fatal(m4_normalize([
  Macro PKG_PROG_PKG_CONFIG is not available.
  It is usually defined in file pkg.m4 provided by package pkg-config.]))])
m4_pattern_forbid([PKG_PROG_PKG_CONFIG])dnl
PKG_PROG_PKG_CONFIG([0.23])

AH_TEMPLATE([HAVE_YYLEX_DESTROY],
            [Define if lex/flex has yylex_destroy])
# flex should have this
if test "x$LEX" = xflex; then
   AC_MSG_CHECKING([whether yylex_destroy is generated by flex])
   flex_version=`$LEX -V -v --version 2>/dev/null | $SED -e 's/^.* //'`
   case $flex_version in
       [[0-1].*)]
           AC_MSG_RESULT([version $flex_version => no])
           ;;
       [2.[0-4].*)]
           AC_MSG_RESULT([version $flex_version => no])
           ;;
       [2.5.[0-8])]
           AC_MSG_RESULT([version $flex_version => no])
           ;;
       [2.5.[0-8][A-Za-z]*)]
           AC_MSG_RESULT([version $flex_version => no])
           ;;
       *)
           AC_MSG_RESULT([version $flex_version => yes])
           AC_DEFINE([HAVE_YYLEX_DESTROY])
           ;;
   esac
fi

dnl Makefile.am:77: compiling `config_gram.c' with per-target flags requires `AM_PROG_CC_C_O' in `configure.ac'
AM_PROG_CC_C_O

# Checks for libraries.
# For MinGW.
AC_CHECK_LIB([ws2_32], [WSAStartup])

AC_CHECK_LIB([termcap], [tputs])
AC_CHECK_LIB([ncurses], [tputs])

AC_CHECK_LIB([readline], [readline], [dnl
	LIBS="-lreadline $LIBS"
	AC_DEFINE([HAVE_LIBREADLINE], [1],
	          [Whether we have libreadline with the readline function])
	have_libreadline=yes
])

AH_TEMPLATE([HAVE_LIBELF],
            [Define if ELF support is enabled via libelf])
AC_CHECK_LIB([elf], [elf_getshdrstrndx], [have_libelf=yes])
LIBELF=""
if test "x$have_libelf" = xyes; then
   case $target in
       *)
               LIBELF="-lelf"
               ;;
   esac
   AC_DEFINE([HAVE_LIBELF])
   AC_CHECK_HEADERS([libelf.h libelf/libelf.h])
fi
AC_SUBST([LIBELF])

AC_SEARCH_LIBS([gethostent], [nsl])
AC_SEARCH_LIBS([setsockopt], [socket])
AH_TEMPLATE([HAVE_LIBUSB],
            [Define if USB support is enabled via libusb])
AC_CHECK_LIB([usb], [usb_get_string_simple], [have_libusb=yes])
LIBUSB=""
if test "x$have_libusb" = xyes; then
   case $target in
       *-*-darwin*)
               LIBUSB="-lusb -framework CoreFoundation -framework IOKit"
               ;;
       *)
               LIBUSB="-lusb"
               ;;
   esac
   AC_DEFINE([HAVE_LIBUSB])
   AC_CHECK_HEADERS([usb.h])
   AC_CHECK_HEADERS([lusb0_usb.h])
fi
AC_SUBST([LIBUSB])

AC_ARG_ENABLE(
	[libusb_1_0],
	AS_HELP_STRING([--disable-libusb_1_0],
	               [Allow to build with libusb instead of libusb_1_0 if both are present]),
	[case "${enableval}" in
		yes) enabled_libusb_1_0=yes ;;
		no)  enabled_libusb_1_0=no ;;
		*)   AC_MSG_ERROR([bad value ${enableval} for enable-libusb_1_0 option]) ;;
		esac],
	[enabled_libusb_1_0=yes])	

have_libusb_1_0=no
AH_TEMPLATE([HAVE_LIBUSB_1_0],
            [Define if USB support is enabled via libusb 1.0])
AC_CHECK_LIB([usb-1.0], [libusb_init], [have_libusb_1_0=yes])
LIBUSB_1_0=""
if test "x$have_libusb_1_0" = xyes && test "x$enabled_libusb_1_0" = xyes; then
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
if test "x$have_libusb_1_0" = xyes && test "x$enabled_libusb_1_0" = xyes; then
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
AC_SUBST([LIBUSB_1_0])

AH_TEMPLATE([HAVE_LIBHIDAPI],
            [Define if HID support is enabled via libhidapi])
AC_SEARCH_LIBS([hid_init], [hidapi hidapi-libusb hidapi-hidraw],
               [have_libhidapi=yes], [have_libhidapi=no])
LIBHIDAPI=""
if test "x$have_libhidapi" = xyes; then
   case $target in
       *-*-darwin*)
               LIBHIDAPI="-lhidapi -iframework CoreFoundation -framework IOKit"
               ;;
       *)
               LIBHIDAPI="$ac_cv_lib_hid_init"
               ;;
   esac
   AC_DEFINE([HAVE_LIBHIDAPI])
   AC_CHECK_HEADERS([hidapi/hidapi.h])
fi
AC_SUBST([LIBHIDAPI])

AH_TEMPLATE([HAVE_LIBSERIALPORT],
            [Define if libserialport is found])
AC_CHECK_LIB([serialport], [sp_open],
             [have_libserialport=yes], [have_libserialport=no])
LIBSERIALPORT=""
if test "x$have_libserialport" = xyes; then
   case $target in
       *)
               LIBSERIALPORT="-lserialport"
               ;;
   esac
   AC_DEFINE([HAVE_LIBSERIALPORT])
   AC_CHECK_HEADERS([libserialport.h])
fi
AC_SUBST([LIBSERIALPORT])

AH_TEMPLATE([HAVE_LIBFTDI1],
            [Define if FTDI support is enabled via libftdi1])
AH_TEMPLATE([HAVE_LIBFTDI],
            [Define if FTDI support is enabled via libftdi])
AH_TEMPLATE([HAVE_LIBFTDI_TYPE_232H],
            [Define if libftdi supports FT232H, libftdi version >= 0.20])
AC_CHECK_LIB([ftdi1], [ftdi_new],
             [have_libftdi1=yes], [have_libftdi1=no], [$LIBUSB_1_0])
AC_CHECK_LIB([ftdi], [ftdi_usb_get_strings],
             [have_libftdi=yes], [have_libftdi=no], [-lusb])
LIBFTDI=""
LIBFTDI1=""
if test "x$have_libftdi1" = xyes; then
   LIBFTDI1="-lftdi1"
   AC_DEFINE([HAVE_LIBFTDI1])
   AC_SUBST([LIBFTDI1])
   LIBS="${LIBS} ${LIBFTDI1}"
   AC_CHECK_FUNCS([ftdi_tcioflush])
else
   if test "x$have_libftdi" = xyes; then
      LIBFTDI="-lftdi -lusb"
      AC_DEFINE([HAVE_LIBFTDI])
      AC_SUBST([LIBFTDI])
      AC_CHECK_DECL([TYPE_232H],
        [have_libftdi_FT232H=yes],
        [have_libftdi_FT232H=no],
        [[#include <ftdi.h>]])
      if test "x$have_libftdi_FT232H" = xyes; then
         AC_DEFINE([HAVE_LIBFTDI_TYPE_232H])
      fi
   fi
fi

AC_CHECK_HEADERS([pthread.h])
# as there exist header file only pthread implementations for Windows, check if we have a library
AC_CHECK_LIB([pthread], [pthread_create], [have_pthread=yes], [have_pthread=no])
LIBPTHREAD=""
if test "x$have_pthread" = xyes; then
   LIBPTHREAD="-lpthread"
fi
AC_SUBST([LIBPTHREAD])

# Checks for header files.
AC_CHECK_HEADERS([limits.h stdlib.h string.h])
AC_CHECK_HEADERS([fcntl.h sys/ioctl.h sys/time.h termios.h unistd.h])
AC_CHECK_HEADERS([ddk/hidsdi.h],,,[#include <windows.h>
#include <setupapi.h>])


# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST


AC_CHECK_HEADERS([netinet/in.h])

# Checks for library functions.
AC_CHECK_FUNCS([memset select strcasecmp strdup strerror strncasecmp strtol strtoul gettimeofday usleep getaddrinfo])

AC_MSG_CHECKING([for a Win32 HID library])
SAVED_LIBS="${LIBS}"
LIBHID=""
HIDINCLUDE=""
case $target in
        *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		LIBHID="-lhid -lsetupapi"
		HIDINCLUDE="#include <hidsdi.h>"
		;;
	*)
		LIBHID=""
		;;
esac
LIBS="${LIBS} ${LIBHID}"

AH_TEMPLATE([HAVE_LIBHID],
            [Define if HID support is enabled via the Win32 DDK])
AC_LINK_IFELSE([AC_LANG_PROGRAM([[#include <windows.h>
#include <setupapi.h>
$HIDINCLUDE]], [[int
main(void)
{
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);

    return 0;
}
]])], [have_libhid=yes], [have_libhid=no])
AC_MSG_RESULT([$have_libhid])
if test "x$have_libhid" = xyes; then
   AC_DEFINE([HAVE_LIBHID])
else
   LIBHID=""
fi
LIBS="${SAVED_LIBS}"
AC_SUBST([LIBHID])

# Check for types

# Solaris has uint_t and ulong_t typedefs in <sys/types.h>, avoid
# the redeclaration in usbtiny.c.
AC_CHECK_TYPES([uint_t], [], [], [#include <sys/types.h>])
AC_CHECK_TYPES([ulong_t], [], [], [#include <sys/types.h>])

# Checks for misc stuff.

AC_ARG_ENABLE(
	[versioned-doc],
	AS_HELP_STRING([--enable-versioned-doc],
	               [install docs in directory with version name (default)]),
	[case "${enableval}" in
		yes) versioned_doc=yes ;;
		no)  versioned_doc=no ;;
		*)   AC_MSG_ERROR([bad value ${enableval} for versioned-doc option]) ;;
		esac],
	[versioned_doc=yes])

if test "x$versioned_doc" = "xyes"; then
    DOC_INST_DIR='$(DESTDIR)$(datadir)/doc/avrdude-$(VERSION)'
else
    DOC_INST_DIR='$(DESTDIR)$(datadir)/doc/avrdude'
fi
AC_SUBST([DOC_INST_DIR])

AC_ARG_ENABLE(
	[doc],
	AS_HELP_STRING([--enable-doc],
	               [Enable building documents]),
	[case "${enableval}" in
		yes) enabled_doc=yes ;;
		no)  enabled_doc=no ;;
		*)   AC_MSG_ERROR([bad value ${enableval} for disable-doc option]) ;;
		esac],
	[enabled_doc=no])

AC_ARG_ENABLE(
	[parport],
	AS_HELP_STRING([--enable-parport],
	               [Enable accessing parallel ports(default)]),
	[case "${enableval}" in
		yes) enabled_parport=yes ;;
		no)  enabled_parport=no ;;
		*)   AC_MSG_ERROR([bad value ${enableval} for enable-parport option]) ;;
		esac],
	[enabled_parport=no])

AS_IF([test "x$enabled_parport" = xyes], [dnl
  case $target in
        *-*-mingw32* | *-*-cygwin* | *-*-windows*)
	  AC_MSG_ERROR([avrdude does not support parallel port on Windows])
	;;
        *-*-darwin*)
	  AC_MSG_ERROR([avrdude does not support parallel port on MacOS])
	;;
  esac
])

AC_ARG_ENABLE(
	[linuxgpio],
	AS_HELP_STRING([--enable-linuxgpio],
	               [Enable the Linux GPIO interface programmer type]),
	[case "${enableval}" in
		yes) enabled_linuxgpio=yes ;;
		no)  enabled_linuxgpio=no ;;
		*)   AC_MSG_ERROR([bad value ${enableval} for enable-linuxgpio option]) ;;
		esac],
	[enabled_linuxgpio=no])	

AC_ARG_ENABLE(
	[linuxspi],
	AS_HELP_STRING([--enable-linuxspi],
	               [Enable the Linux SPIDEV interface programmer type]),
	[case "${enableval}" in
		yes) enabled_linuxspi=yes ;;
		no)  enabled_linuxspi=no ;;
		*)   AC_MSG_ERROR([bad value ${enableval} for enable-linuxspi option]) ;;
		esac],
	[enabled_linuxspi=no])

AC_SUBST([DIST_SUBDIRS_AC], [doc])

if test "x$enabled_doc" = xyes; then
	SUBDIRS_AC='doc'
else
	SUBDIRS_AC=''
fi
AC_SUBST([SUBDIRS_AC])


# Find the parallel serial device files based on target system
# If a system doesn't have a PC style parallel, mark it as unknown.
case $target in
	i[[3456]]86-*-linux*|x86_64-*-linux*)
		DEFAULT_PAR_PORT="/dev/parport0"
		DEFAULT_SER_PORT="/dev/ttyS0"
		DEFAULT_SPI_PORT="/dev/spidev0.0:/dev/gpiochip0"
		DEFAULT_LINUXGPIO_PORT="gpiochip0"
		;;
	*-*-linux*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="/dev/ttyS0"
		DEFAULT_SPI_PORT="/dev/spidev0.0:/dev/gpiochip0"
		DEFAULT_LINUXGPIO_PORT="gpiochip0"
		;;
	i[[3456]]86-*-*freebsd*|amd64-*-*freebsd*)
		DEFAULT_PAR_PORT="/dev/ppi0"
		DEFAULT_SER_PORT="/dev/cuad0"
		DEFAULT_SPI_PORT="unknown"
		DEFAULT_LINUXGPIO_PORT="unknown"
		;;
	*-*-*freebsd*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="/dev/cuad0"
		DEFAULT_SPI_PORT="unknown"
		DEFAULT_LINUXGPIO_PORT="unknown"
		;;
	*-*-solaris*)
		DEFAULT_PAR_PORT="/dev/printers/0"
		DEFAULT_SER_PORT="/dev/term/a"
		DEFAULT_SPI_PORT="unknown"
		DEFAULT_LINUXGPIO_PORT="unknown"
		;;
	*-*-msdos* | *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		DEFAULT_PAR_PORT="lpt1"
		DEFAULT_SER_PORT="com1"
		DEFAULT_SPI_PORT="unknown"
		DEFAULT_LINUXGPIO_PORT="unknown"
		;;
	*)
		DEFAULT_PAR_PORT="unknown"
		DEFAULT_SER_PORT="unknown"
		DEFAULT_SPI_PORT="unknown"
		DEFAULT_LINUXGPIO_PORT="unknown"
		;;
esac

if test "x$enabled_parport" = xyes; then
	AC_MSG_CHECKING([for parallel device])
	if test "x$DEFAULT_PAR_PORT" = xunknown; then
		AC_MSG_NOTICE([parallel port access disabled for this system])
		enabled_parport=no
	else
		AC_MSG_RESULT([$DEFAULT_PAR_PORT])
	fi
	AC_SUBST([DEFAULT_PAR_PORT])
fi

AC_MSG_CHECKING([for serial device])
AC_MSG_RESULT([$DEFAULT_SER_PORT])
AC_SUBST([DEFAULT_SER_PORT])

if test "x$enabled_parport" = xyes; then
	AC_DEFINE([HAVE_PARPORT], [1], [parallel port access enabled])
	confsubst="-e /^@HAVE_PARPORT_/d"
else
	confsubst="-e /^@HAVE_PARPORT_BEGIN@/,/^@HAVE_PARPORT_END@/d"
fi

AC_MSG_CHECKING([for SPI device])
AC_MSG_RESULT([$DEFAULT_SPI_PORT])
AC_SUBST([DEFAULT_SPI_PORT])

if test "x$enabled_linuxgpio" = xyes; then
	AC_MSG_CHECKING([for default linuxgpio port])
	if test "x$DEFAULT_LINUXGPIO_PORT" = xunknown; then
		AC_MSG_NOTICE([linuxgpio port access disabled for this system])
		enabled_linuxgpio=no
	else
		AC_MSG_RESULT([$DEFAULT_LINUXGPIO_PORT])
	fi
	AC_SUBST([DEFAULT_LINUXGPIO_PORT])
fi

if test "x$enabled_linuxgpio" = xyes; then
	AC_DEFINE([HAVE_LINUXGPIO], [1], [Linux sysfs GPIO support enabled])
	confsubst="$confsubst -e /^@HAVE_LINUXGPIO_/d"

	PKG_CHECK_MODULES([LIBGPIOD], [libgpiod >= 2.0], [
		have_libgpiod=yes
		have_libgpiodv2=yes
		AC_DEFINE([HAVE_LIBGPIOD], [1], [Linux libgpiod available])
		AC_DEFINE([HAVE_LIBGPIOD_V1_6], [0], [Linux libgpiod >= v1.6 but < 2.0 available])
		AC_DEFINE([HAVE_LIBGPIOD_V2], [1], [Linux libgpiod >= 2.0 available])
	], [
		PKG_CHECK_MODULES([LIBGPIOD], [libgpiod >= 1.6], [
			have_libgpiod=yes
			have_libgpiodv1_6=yes
			AC_DEFINE([HAVE_LIBGPIOD], [1], [Linux libgpiod available])
			AC_DEFINE([HAVE_LIBGPIOD_V1_6], [1], [Linux libgpiod >= v1.6 but < 2.0 available])
			AC_DEFINE([HAVE_LIBGPIOD_V2], [0], [Linux libgpiod >= 2.0 available])
		], [
			PKG_CHECK_MODULES([LIBGPIOD], [libgpiod], [
				have_libgpiod=yes
				AC_DEFINE([HAVE_LIBGPIOD], [1], [Linux libgpiod available])
				AC_DEFINE([HAVE_LIBGPIOD_V1_6], [0], [Linux libgpiod >= v1.6 but < 2.0 available])
				AC_DEFINE([HAVE_LIBGPIOD_V2], [0], [Linux libgpiod >= 2.0 available])
			], [
				have_libgpiod=no
			])
		])
	])

else
	confsubst="$confsubst -e /^@HAVE_LINUXGPIO_BEGIN@/,/^@HAVE_LINUXGPIO_END@/d"
fi


if test "x$enabled_linuxspi" = xyes; then
	AC_DEFINE([HAVE_LINUXSPI], [1], [Linux SPI support enabled])
	confsubst="$confsubst -e /^@HAVE_LINUXSPI_/d"
else
	confsubst="$confsubst -e /^@HAVE_LINUXSPI_BEGIN@/,/^@HAVE_LINUXSPI_END@/d"
fi


# If we are compiling with gcc, enable all warnings and make warnings errors.
ENABLE_WARNINGS=""
if test "x$GCC" = xyes; then
	ENABLE_WARNINGS="-Wall -Wextra -Wno-unused-parameter -Wno-format-zero-length"

	# does this compiler support -Wno-pointer-sign ?
	AC_MSG_CHECKING([if gcc accepts -Wno-pointer-sign ])

	safe_CFLAGS=$CFLAGS
	CFLAGS="$ENABLE_WARNINGS -Wno-pointer-sign"

	AC_COMPILE_IFELSE([
		AC_LANG_PROGRAM([[]], [[ int main () { return 0 ; } ]])
	], [dnl
		no_pointer_sign=yes
		AC_MSG_RESULT([yes])
	], [dnl
		no_pointer_sign=no
		AC_MSG_RESULT([no])
	])
	CFLAGS=$safe_CFLAGS

	if test "x$no_pointer_sign" = xyes; then
		ENABLE_WARNINGS="$ENABLE_WARNINGS -Wno-pointer-sign"
	fi
fi
AC_SUBST([ENABLE_WARNINGS])

# See if we need to drop into the windows subdir.
case $target in
        *-*-mingw32* | *-*-cygwin* | *-*-windows*)
		if test "x$GCC" = xyes && test "x$CC" = xcc -o "x$CC" = xgcc; then
			# does this compiler support -mno-cygwin?
			AC_MSG_CHECKING([if $CC accepts -mno-cygwin])

			safe_CFLAGS="$CFLAGS"
			CFLAGS="$ENABLE_WARNINGS -mno-cygwin"

			AC_COMPILE_IFELSE([
				AC_LANG_PROGRAM([[]], [[ int main () { return 0 ; } ]])
			], [dnl
				no_cygwin=yes
				AC_MSG_RESULT([yes])
			], [dnl
				no_cygwin=no
				AC_MSG_RESULT([no])
			])
			CFLAGS="$safe_CFLAGS"

			if test "x$no_cygwin" = xyes; then
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
		AC_LINK_IFELSE([
			AC_LANG_PROGRAM([[]], [[ int main () { return 0 ; } ]])
		], [dnl
			can_link_static=yes
			AC_MSG_RESULT([yes])
		], [dnl
			can_link_static=no
			AC_MSG_RESULT([no])
		])
		LDFLAGS="$safe_LDFLAGS"

		if test "x$can_link_static" = xyes; then
			LDFLAGS="${LDFLAGS} -static"
		fi

		CFLAGS="${CFLAGS} -DWIN32NATIVE"
		;;
esac

AC_CONFIG_FILES([
       doc/Makefile
       avrdude.spec
       Makefile
])

AC_CONFIG_FILES([
       GNUmakefile
])


# The procedure to create avrdude.conf involves two steps.  First,
# normal autoconf substitution will be applied, resulting in
# avrdude.conf.tmp. Finally, a sed command will be applied to filter
# out unwanted parts (currently the parallel port programmer types)
# based on previous configuration results, thereby producing the final
# avrdude.conf file.

AC_CONFIG_FILES([avrdude.conf.tmp:avrdude.conf.in],
		[sed $confsubst avrdude.conf.tmp > avrdude.conf],
		[confsubst="$confsubst"])

AC_OUTPUT

echo ""
echo ""
echo "Configuration summary:"
echo "----------------------"

if test "x$have_libelf" = xyes; then
   echo "DO HAVE    libelf"
else
   echo "DON'T HAVE libelf"
fi

if test "x$have_libusb" = xyes; then
   echo "DO HAVE    libusb"
else
   echo "DON'T HAVE libusb"
fi

if test "x$have_libusb_1_0" = xyes; then
   if test "x$enabled_libusb_1_0" = xyes; then
       echo "DO HAVE    libusb_1_0"
   else
       echo "DISABLED   libusb_1_0"
   fi
else
   echo "DON'T HAVE libusb_1_0"
fi

if test "x$have_libftdi1" = xyes; then
   echo "DO HAVE    libftdi1"
else
   echo "DON'T HAVE libftdi1"
fi

if test "x$have_libftdi" = xyes; then
   if test x$have_libftdi1 = xyes; then
       echo "DO HAVE    libftdi (but prefer to use libftdi1)"
   else
       echo "DO HAVE    libftdi"
   fi
else
   echo "DON'T HAVE libftdi"
fi

if test "x$have_libhid" = xyes; then
   echo "DO HAVE    libhid"
else
   echo "DON'T HAVE libhid"
fi

if test "x$have_libhidapi" = xyes; then
   echo "DO HAVE    libhidapi"
else
   echo "DON'T HAVE libhidapi"
fi

if test "x$have_libreadline" = xyes; then
   echo "DO HAVE    libreadline"
else
   echo "DON'T HAVE libreadline"
fi

if test "x$have_libserialport" = xyes; then
   echo "DO HAVE    libserialport"
else
   echo "DON'T HAVE libserialport"
fi

if test "x$have_pthread" = xyes; then
   echo "DO HAVE    pthread"
else
   echo "DON'T HAVE pthread"
fi

if test "x$enabled_doc" = xyes; then
   echo "ENABLED    doc"
else
   echo "DISABLED   doc"
fi

if test "x$enabled_parport" = xyes; then
   echo "ENABLED    parport"
else
   echo "DISABLED   parport"
fi

if test "x$enabled_linuxgpio" = xyes; then
   echo "ENABLED    linuxgpio"
   if test "x$have_libgpiodv2" = xyes; then
       echo "DO HAVE    libgpiod (>= 2.0)"
   elif test "x$have_libgpiodv1_6" = xyes; then
       echo "DO HAVE    libgpiod (>= 1.6 < 2.0)"
   elif test "x$have_libgpiod" = xyes; then
       echo "DO HAVE    libgpiod (< 1.6)"
   else
       echo "DON'T HAVE libgpiod"
   fi
else
   echo "DISABLED   linuxgpio"
fi

if test "x$enabled_linuxspi" = xyes; then
   echo "ENABLED    linuxspi"
else
   echo "DISABLED   linuxspi"
fi

