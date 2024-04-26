# versioninfo.m4 - find avrdude version information for msg  -*- Autoconf -*-
# serial 1
dnl | Increment the above serial number every time you edit this file.
dnl | When it finds multiple m4 files with the same name,
dnl | aclocal will use the one with the highest serial.
dnl
dnl The sequence of version info items to store in the version-stamp file.
dnl This must be the same sequence as the versioninfo script writes.
m4_pattern_forbid([versioninfo_items])
m4_define([versioninfo_items], [
  [CMAKE_PROJECT_VERSION],
  [GIT_COMMIT_DATE],
  [GIT_COMMIT_HASH],
  [GIT_TAG_HASH]
])
dnl
dnl Initialize version info from the script command given as macro argument.
m4_pattern_forbid([versioninfo_init])
m4_define([versioninfo_init], [
  m4_pushdef([versioninfo_split], m4_split(m4_esyscmd($1),m4_newline))
  m4_case(m4_sysval, [0], [
    m4_for([N], [1], m4_count(versioninfo_split), [1], [
      m4_define([versioninfo_]m4_normalize(m4_argn(N, versioninfo_items)),
                m4_normalize(m4_argn(N, versioninfo_split)))
    ])
  ], [
    m4_fatal([versioninfo script returned non-0])
  ])
])
dnl
m4_define([versioninfo_stampfile], [versioninfo-stamp])
dnl
versioninfo_init([./build-helpers/versioninfo.sh . ]m4_defn([versioninfo_stampfile]))
dnl
dnl
dnl
dnl ========================================================================
dnl VERSIONINFO_SETUP()
dnl ========================================================================
dnl
AC_DEFUN([VERSIONINFO_SETUP], [dnl
m4_foreach([ITEM], [versioninfo_items], [dnl
  AC_SUBST(m4_normalize(ITEM), [m4_defn([versioninfo_]m4_normalize(ITEM))])
  AC_MSG_CHECKING([versioninfo item ]m4_normalize(ITEM))
  AC_MSG_RESULT([$]m4_normalize(ITEM))
])
dnl
dnl Define helper substitutions containing shell code for use inside
dnl make recipes in build-helpers/versioninfo.mk and $(shell ...)
dnl calls in GNUmakefile.in dealing with version info.
dnl
AC_SUBST([VERSIONINFO_STAMPFILE], [m4_defn([versioninfo_stampfile])])
AM_SUBST_NOTMAKE([VERSIONINFO_WRITE])
dnl
AC_SUBST([VERSIONINFO_READ],
         ['m4_foreach([ITEM], [versioninfo_items], [read m4_normalize(ITEM); ]):'])
AM_SUBST_NOTMAKE([VERSIONINFO_READ])
dnl
AC_SUBST([VERSIONINFO_WRITE],
         ['printf "%s\n"m4_foreach([ITEM], [versioninfo_items], [ "$(m4_normalize(ITEM))"])'])
AM_SUBST_NOTMAKE([VERSIONINFO_WRITE])
dnl
AC_SUBST([VERSIONINFO_IS_UNCHANGED],
         ['false; then :; m4_foreach([ITEM], [versioninfo_items], [elif test "x$(m4_normalize(ITEM))" != "x$$m4_normalize(ITEM)"; then printf "%s has changed from %s to %s\n" "m4_normalize(ITEM)" "$(m4_normalize(ITEM))" "$$m4_normalize(ITEM)" >&2; ]) else '])
AM_SUBST_NOTMAKE([VERSIONINFO_IS_UNCHANGED])
dnl
])dnl
dnl
dnl ####################################################################
dnl
dnl Local Variables:
dnl mode: autoconf
dnl End:
