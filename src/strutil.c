/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2023, Hans Eirik Bull
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

#include "ac_cfg.h"
#include <string.h>
#include "libavrdude.h"

int str_starts(const char *str, const char *starts) {
  return strncmp(str, starts, strlen(starts)) == 0;
}

int str_eq(const char *str1, const char *str2) {
  return strcmp(str1, str2) == 0;
}

int str_contains(const char *str, const char *substr) {
  return !!strstr(str, substr);
}

int str_ends(const char *str, const char *ends) {
  size_t str_len  = strlen(str);
  size_t ends_len = strlen(ends);

  if (ends_len > str_len)
    return 0;

  return str_eq(str + str_len - ends_len, ends);
}
