/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef term_h
#define term_h

#include "libavrdude.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  WRITE_MODE_STANDARD = 0,
  WRITE_MODE_FILL     = 1,
} mode;

int terminal_mode(const PROGRAMMER * pgm, const AVRPART * p);
char * terminal_get_input(const char *prompt);
void terminal_setup_update_progress();

#ifdef __cplusplus
}
#endif

#endif
