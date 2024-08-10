/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2024 Stefan Rueger <stefan.rueger@urclocks.com>
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


#include <ac_cfg.h>

#include "avrdude.h"
#include "libavrdude.h"

#if defined(WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// Store the full path of a file using a registry-dependent system search path
int win_set_path(char *path, int n, const char *file) {
  *path = 0;
  return SearchPath(NULL, file, NULL, n, path, NULL);
}

#endif
