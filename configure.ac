# Process this file with autoconf to produce a configure script.
AC_INIT(avrdude, 3.1.0cvs, avrdude-dev@nongnu.org)
AC_PREREQ(2.53)
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

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
