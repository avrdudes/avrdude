/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2011  Darell Tan <darell.tan@gmail.com>
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

#ifndef hvsp_h
#define hvsp_h

#include <stdbool.h>

#include "libavrdude.h"

#ifdef __cplusplus
extern "C" {
#endif

bool hvsp_is_hvsp_mode(const PROGRAMMER *pgm, const AVRPART *p);

#ifdef __cplusplus
}
#endif

#endif
