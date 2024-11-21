# buildinfo.m4 - print build info at build time and at runtime -*- Autoconf -*-
# serial 1
dnl | Increment the above serial number every time you edit this file.
dnl | When it finds multiple m4 files with the same name,
dnl | aclocal will use the one with the highest serial.
dnl
dnl
dnl ========================================================================
dnl BUILDINFO_SETUP()
dnl ========================================================================
dnl
m4_pattern_forbid([^BUILDINFO_SETUP])dnl
AC_DEFUN([BUILDINFO_SETUP], [dnl
AC_MSG_NOTICE([BEGIN ][[$0]])

AC_MSG_NOTICE([END ][[$0]])
])dnl
dnl
dnl
dnl ========================================================================
dnl BUILDINFO_OUTPUT()
dnl ========================================================================
dnl
m4_pattern_forbid([^BUILDINFO_OUTPUT])dnl
AC_DEFUN([BUILDINFO_OUTPUT], [dnl
AC_MSG_NOTICE([BEGIN ][[$0]])
AC_REQUIRE([BUILDINFO_SETUP])

AC_MSG_NOTICE([END ][[$0]])
])dnl
dnl
dnl ####################################################################
dnl
dnl Local Variables:
dnl mode: autoconf
dnl End:
