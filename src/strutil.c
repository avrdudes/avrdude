/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2023, Hans Eirik Bull
 * Copyright (C) 2023, Stefan Rueger <stefan.rueger@urclocks.com>
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

int str_casestarts(const char *str, const char *starts) {
  return strncasecmp(str, starts, strlen(starts)) == 0;
}

int str_caseeq(const char *str1, const char *str2) {
  return strcasecmp(str1, str2) == 0;
}

int str_caseends(const char *str, const char *ends) {
  size_t str_len  = strlen(str);
  size_t ends_len = strlen(ends);

  if (ends_len > str_len)
    return 0;

  return str_caseeq(str + str_len - ends_len, ends);
}


/*
 * Match STRING against the partname pattern PATTERN, returning 1 if it matches,
 * 0 if not. Note: str_casematch() is a modified old copy of !fnmatch() from the
 * GNU C Library (published under GLP v2). Used for portability.
 */

inline static int fold(int c) {
  return (c >= 'A' && c <= 'Z')? c+('a'-'A'): c;
}

int str_casematch(const char *pattern, const char *string) {
  unsigned char c;
  const char *p = pattern, *n = string;

  if(!*n)                       // AVRDUDE specialty: empty string never matches
    return 0;

  while((c = fold(*p++))) {
    switch(c) {
    case '?':
      if(*n == 0)
        return 0;
      break;

    case '\\':
      c = fold(*p++);
      if(fold(*n) != c)
        return 0;
      break;

    case '*':
      for(c = *p++; c == '?' || c == '*'; c = *p++)
        if(c == '?' && *n++ == 0)
          return 0;

      if(c == 0)
        return 1;

      {
        unsigned char c1 = fold(c == '\\'? *p : c); // This char

        for(--p; *n; ++n)       // Recursively check reminder of string for *
          if((c == '[' || fold(*n) == c1) && str_casematch(p, n) == 1)
            return 1;
        return 0;
      }

    case '[':
      {
        int negate;

        if(*n == 0)
          return 0;

        negate = (*p == '!' || *p == '^');
        if(negate)
          ++p;

        c = *p++;
        for(;;) {
          unsigned char cstart = c, cend = c;

          if(c == '\\')
            cstart = cend = *p++;

          cstart = cend = fold(cstart);

          if(c == 0)            // [ (unterminated)
            return 0;

          c = *p++;
          c = fold(c);

          if(c == '-' && *p != ']') {
            cend = *p++;
            if(cend == '\\')
              cend = *p++;
            if(cend == 0)
              return 0;
            cend = fold(cend);

            c = *p++;
          }

          if(fold(*n) >= cstart && fold(*n) <= cend)
            goto matched;

          if(c == ']')
            break;
        }
        if(!negate)
          return 0;
        break;

      matched:;
        while(c != ']') {       // Skip the rest of the [...] that already matched

          if(c == 0)            // [... (unterminated)
            return 0;

          c = *p++;
          if(c == '\\')
            ++p;
        }
        if(negate)
          return 0;
      }
      break;

    default:
      if(c != fold(*n))
        return 0;
    }

    ++n;
  }

  return *n == 0;
}
