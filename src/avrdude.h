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

#define SYSTEM_CONF_FILE "avrdude.conf"
#if defined(WIN32)
#define USER_CONF_FILE "avrdude.rc"
#else
#define USER_CONF_FILE ".avrduderc"
#endif

extern char *progname;          // name of program, for messages
extern char progbuf[];          // spaces same length as progname

extern int ovsigck;             // override signature check (-F)
extern int verbose;             // verbosity level (-v, -vv, ...)
extern int quell_progress;      // quell progress report -q, reduce effective verbosity level (-qq, -qqq)

int avrdude_message(const int msglvl, const char *format, ...);

#define MSG_EXT_ERROR      (-3) // no -v option, can be suppressed with -qqqqq
#define MSG_ERROR          (-2) // no -v option, can be suppressed with -qqqq
#define MSG_WARNING        (-1) // no -v option, can be suppressed with -qqq
#define MSG_INFO              0 // no -v option, can be suppressed with -qq
#define MSG_NOTICE            1 // displayed with -v
#define MSG_NOTICE2           2 // displayed with -vv, used rarely
#define MSG_DEBUG             3 // displayed with -vvv
#define MSG_TRACE             4 // displayed with -vvvv, show trace communication
#define MSG_TRACE2            5 // displayed with -vvvvv

// Shortcuts
#define msg_ext_error(...) avrdude_message(MSG_ERROR, __VA_ARGS__)
#define msg_error(...)     avrdude_message(MSG_ERROR, __VA_ARGS__)
#define msg_warning(...)   avrdude_message(MSG_WARNING, __VA_ARGS__)
#define msg_info(...)      avrdude_message(MSG_INFO, __VA_ARGS__)
#define msg_notice(...)    avrdude_message(MSG_NOTICE, __VA_ARGS__)
#define msg_notice2(...)   avrdude_message(MSG_NOTICE2, __VA_ARGS__)
#define msg_debug(...)     avrdude_message(MSG_DEBUG, __VA_ARGS__)
#define msg_trace(...)     avrdude_message(MSG_TRACE, __VA_ARGS__)
#define msg_trace2(...)    avrdude_message(MSG_TRACE2, __VA_ARGS__)

#endif
