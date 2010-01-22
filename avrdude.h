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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */

#ifndef avrdude_h
#define avrdude_h

extern char * progname;		/* name of program, for messages */
extern char progbuf[];		/* spaces same length as progname */

extern int do_cycles;		/* track erase-rewrite cycles (-y) */
extern int ovsigck;		/* override signature check (-F) */
extern int verbose;		/* verbosity level (-v, -vv, ...) */
extern int quell_progress;	/* quiteness level (-q, -qq) */

#if defined(WIN32NATIVE)

#include "ac_cfg.h"
#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(HAVE_USLEEP)
int usleep(unsigned int us);
#endif

#if !defined(HAVE_GETTIMEOFDAY)
struct timezone;
int gettimeofday(struct timeval *tv, struct timezone *tz);
#ifdef __cplusplus
}
#endif
#endif /* HAVE_GETTIMEOFDAY */

#endif /* defined(WIN32NATIVE) */

#endif
