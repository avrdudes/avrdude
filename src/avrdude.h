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

#ifndef avrdude_h
#define avrdude_h

#include <stdio.h>
#include <stdlib.h>

#define SYSTEM_CONF_FILE "avrdude.conf"

#if defined(WIN32)
#define USER_CONF_FILE "avrdude.rc"
#else
#define USER_CONF_FILE ".avrduderc"
#define XDG_USER_CONF_FILE "avrdude/avrdude.rc"
#endif

#define progbuf ""              // Used to be for indenting continuation below "avrdude: msg"
extern char *progname;          // Name of program, for messages
extern int ovsigck;             // Override signature check (-F)
extern int verbose;             // Verbosity level (-v, -vv, ...)
extern int quell_progress;      // Quell progress report -q, reduce effective verbosity level (-qq, -qqq)
extern const char *partdesc;    // Part -p string
extern const char *pgmid;       // Programmer -c string

// Magic memory tree: these functions succeed or exit()
#define mmt_strdup(s) cfg_strdup(__func__, s)
#define mmt_malloc(n) cfg_malloc(__func__, n)
#define mmt_realloc(p, n) cfg_realloc(__func__, p, n)
#define mmt_sprintf(...) str_sprintf(__VA_ARGS__)
#define mmt_free(p) free(p)

int avrdude_message2(FILE *fp, int lno, const char *file, const char *func, int msgmode, int msglvl, const char *format, ...)
#if defined(__GNUC__)           // Ask gcc to check whether format and parameters match
  __attribute__((format(printf, 7, 8)))
#endif
  ;

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

#define pmsg_ext_error(...) avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FUNCTION|MSG2_FILELINE|MSG2_TYPE|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_EXT_ERROR, __VA_ARGS__)
#define pmsg_error(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FUNCTION|MSG2_FILELINE|MSG2_TYPE|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_ERROR, __VA_ARGS__)
#define pmsg_warning(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FUNCTION|MSG2_FILELINE|MSG2_TYPE|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_WARNING, __VA_ARGS__)
#define pmsg_info(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_INFO, __VA_ARGS__)
#define pmsg_notice(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_NOTICE, __VA_ARGS__)
#define pmsg_notice2(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_NOTICE2, __VA_ARGS__)
#define pmsg_debug(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_DEBUG, __VA_ARGS__)
#define pmsg_trace(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_TRACE, __VA_ARGS__)
#define pmsg_trace2(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_UCFIRST|MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_TRACE2, __VA_ARGS__)

#define imsg_ext_error(...) avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_EXT_ERROR, __VA_ARGS__)
#define imsg_error(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_ERROR, __VA_ARGS__)
#define imsg_warning(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_WARNING, __VA_ARGS__)
#define imsg_info(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_INFO, __VA_ARGS__)
#define imsg_notice(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_NOTICE, __VA_ARGS__)
#define imsg_notice2(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_NOTICE2, __VA_ARGS__)
#define imsg_debug(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_DEBUG, __VA_ARGS__)
#define imsg_trace(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_TRACE, __VA_ARGS__)
#define imsg_trace2(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_TRACE2, __VA_ARGS__)

#define lmsg_ext_error(...) avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_EXT_ERROR, __VA_ARGS__)
#define lmsg_error(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_ERROR, __VA_ARGS__)
#define lmsg_warning(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_WARNING, __VA_ARGS__)
#define lmsg_info(...)      avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_INFO, __VA_ARGS__)
#define lmsg_notice(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_NOTICE, __VA_ARGS__)
#define lmsg_notice2(...)   avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_NOTICE2, __VA_ARGS__)
#define lmsg_debug(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_DEBUG, __VA_ARGS__)
#define lmsg_trace(...)     avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_TRACE, __VA_ARGS__)
#define lmsg_trace2(...)    avrdude_message2(stderr, __LINE__, __FILE__, __func__, MSG2_LEFT_MARGIN, MSG_TRACE2, __VA_ARGS__)

#define term_out(...)       avrdude_message2(stdout, __LINE__, __FILE__, __func__, MSG2_FLUSH, MSG_INFO, __VA_ARGS__)
#define lterm_out(...)      avrdude_message2(stdout, __LINE__, __FILE__, __func__, MSG2_FLUSH|MSG2_LEFT_MARGIN, MSG_INFO, __VA_ARGS__)

#define fmsg_out(fp, ...)   avrdude_message2(fp, __LINE__, __FILE__, __func__, MSG2_FLUSH, MSG_INFO, __VA_ARGS__)
#endif
