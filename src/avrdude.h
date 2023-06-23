/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2007 Joerg Wunsch
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

 /* $Id$ */

#ifndef avrdude_h
#define avrdude_h

#include <stdio.h>

#define SYSTEM_CONF_FILE "avrdude.conf"
#if defined(WIN32)
#define USER_CONF_FILE "avrdude.rc"
#else
#define USER_CONF_FILE ".avrduderc"
#define XDG_USER_CONF_FILE "avrdude/avrdude.rc"
#endif

extern char *progname;       // Name of program, for messages
extern char progbuf[];       // Spaces same length as progname

extern int ovsigck;          // Override signature check (-F)
extern int verbose;          // Verbosity level (-v, -vv, ...)
extern int quell_progress;   // Quell progress report -q, reduce effective verbosity level (-qq, -qqq)
extern const char *partdesc; // Part -p string
extern const char *pgmid;    // Programmer -c string

int avrdude_message(int msglvl, const char *format, ...);
int avrdude_message2(FILE *fp, int lno, const char *file, const char *func, int msgmode, int msglvl, const char *format, ...);

#define MSG_EXT_ERROR   (-3) // OS-type error, no -v option, can be suppressed with -qqqqq
#define MSG_ERROR       (-2) // Avrdude error, no -v option, can be suppressed with -qqqq
#define MSG_WARNING     (-1) // Warning, no -v option, can be suppressed with -qqq
#define MSG_INFO           0 // Commentary, no -v option, can be suppressed with -qq
#define MSG_NOTICE         1 // Displayed with -v
#define MSG_NOTICE2        2 // Displayed with -vv
#define MSG_DEBUG          3 // Displayed with -vvv
#define MSG_TRACE          4 // Displayed with -vvvv, show trace communication
#define MSG_TRACE2         5 // Displayed with -vvvvv

#define MSG2_PROGNAME      1 // Start by printing progname
#define MSG2_FUNCTION      2 // Print calling function (1st arg) after progname if >= notice
#define MSG2_FILELINE      4 // Print source file and line number after function if >= debug
#define MSG2_TYPE          8 // Print message type after function or progname
#define MSG2_INDENT1      16 // Start by printing indentation of progname+1 blanks
#define MSG2_INDENT2      32 // Start by printing indentation of progname+2 blanks
#define MSG2_FLUSH        64 // Flush before and after printing

// Shortcuts
#define msg_ext_error(...)  avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_EXT_ERROR, __VA_ARGS__)
#define msg_error(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_ERROR, __VA_ARGS__)
#define msg_warning(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_WARNING, __VA_ARGS__)
#define msg_info(...)       avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_INFO, __VA_ARGS__)
#define msg_notice(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_NOTICE, __VA_ARGS__)
#define msg_notice2(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_NOTICE2, __VA_ARGS__)
#define msg_debug(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_DEBUG, __VA_ARGS__)
#define msg_trace(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_TRACE, __VA_ARGS__)
#define msg_trace2(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, 0, MSG_TRACE2, __VA_ARGS__)

#define pmsg_ext_error(...) avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FUNCTION|MSG2_FILELINE|MSG2_TYPE|MSG2_FLUSH, MSG_EXT_ERROR, __VA_ARGS__)
#define pmsg_error(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FUNCTION|MSG2_FILELINE|MSG2_TYPE|MSG2_FLUSH, MSG_ERROR, __VA_ARGS__)
#define pmsg_warning(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FUNCTION|MSG2_FILELINE|MSG2_TYPE|MSG2_FLUSH, MSG_WARNING, __VA_ARGS__)
#define pmsg_info(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FLUSH, MSG_INFO, __VA_ARGS__)
#define pmsg_notice(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FLUSH, MSG_NOTICE, __VA_ARGS__)
#define pmsg_notice2(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FLUSH, MSG_NOTICE2, __VA_ARGS__)
#define pmsg_debug(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FLUSH, MSG_DEBUG, __VA_ARGS__)
#define pmsg_trace(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FLUSH, MSG_TRACE, __VA_ARGS__)
#define pmsg_trace2(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_PROGNAME|MSG2_FLUSH, MSG_TRACE2, __VA_ARGS__)

#define imsg_ext_error(...) avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT1|MSG2_FLUSH, MSG_EXT_ERROR, __VA_ARGS__)
#define imsg_error(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT1|MSG2_FLUSH, MSG_ERROR, __VA_ARGS__)
#define imsg_warning(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT1|MSG2_FLUSH, MSG_WARNING, __VA_ARGS__)
#define imsg_info(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT2|MSG2_FLUSH, MSG_INFO, __VA_ARGS__)
#define imsg_notice(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT2|MSG2_FLUSH, MSG_NOTICE, __VA_ARGS__)
#define imsg_notice2(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT2|MSG2_FLUSH, MSG_NOTICE2, __VA_ARGS__)
#define imsg_debug(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT2|MSG2_FLUSH, MSG_DEBUG, __VA_ARGS__)
#define imsg_trace(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT2|MSG2_FLUSH, MSG_TRACE, __VA_ARGS__)
#define imsg_trace2(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_INDENT2|MSG2_FLUSH, MSG_TRACE2, __VA_ARGS__)

#define term_out(...)       avrdude_message2(stdout, __LINE__, __FILE__, __func__, MSG2_FLUSH, MSG_INFO, __VA_ARGS__)
#define fmsg_out(fp, ...)   avrdude_message2(fp, __LINE__, __FILE__, __func__, MSG2_FLUSH, MSG_INFO, __VA_ARGS__)

#endif
