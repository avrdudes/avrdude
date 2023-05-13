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
#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include "libavrdude.h"

// Return 1 if str starts with starts, 0 otherwise
int str_starts(const char *str, const char *starts) {
  return strncmp(str, starts, strlen(starts)) == 0;
}

// Return 1 if str1 and str2 are the same, 0 otherwise
int str_eq(const char *str1, const char *str2) {
  return strcmp(str1, str2) == 0;
}

// Return 1 if str contains substr as substring, 0 otherwise
int str_contains(const char *str, const char *substr) {
  return !!strstr(str, substr);
}

// Return 1 if str ends in ends, 0 otherwise
int str_ends(const char *str, const char *ends) {
  size_t str_len  = strlen(str);
  size_t ends_len = strlen(ends);

  if (ends_len > str_len)
    return 0;

  return str_eq(str + str_len - ends_len, ends);
}

// Return 1 if str starts with starts irrespective of case, 0 otherwise
int str_casestarts(const char *str, const char *starts) {
  return strncasecmp(str, starts, strlen(starts)) == 0;
}

// Return 1 if str1 and str2 are the same irrespective of case, 0 otherwise
int str_caseeq(const char *str1, const char *str2) {
  return strcasecmp(str1, str2) == 0;
}

// Return 1 if str ends in ends irrespective of case, 0 otherwise
int str_caseends(const char *str, const char *ends) {
  size_t str_len  = strlen(str);
  size_t ends_len = strlen(ends);

  if (ends_len > str_len)
    return 0;

  return str_caseeq(str + str_len - ends_len, ends);
}


/*
 * Match string against the partname pattern, returning 1 if it matches, 0 if
 * not. Note: str_casematch() is a modified old copy of !fnmatch() from the
 * GNU C Library (published under GLP v2), which uses shell wildcards for
 * constructing patterns, ie, *, ? and single character classes, eg, [^0-6].
 * Used for portability.
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


// Return a malloc'd string with the sprintf() result
char *str_sprintf(const char *fmt, ...) {
  int size = 0;
  va_list ap;

  // Compute size
  va_start(ap, fmt);
  size = vsnprintf(NULL, size, fmt, ap);
  va_end(ap);

  if(size < 0)
    return cfg_strdup(__func__, "");

  size++;                       // For terminating '\0'
  char *p = cfg_malloc(__func__, size);

  va_start(ap, fmt);
  size = vsnprintf(p, size, fmt, ap);
  va_end(ap);

  if(size < 0)
    *p = 0;

  return p;
}



// Change data item p of size bytes from big endian to little endian and vice versa
static void change_endian(void *p, int size) {
  uint8_t tmp, *w = p;

  for(int i=0; i<size/2; i++)
    tmp = w[i], w[i] = w[size-i-1], w[size-i-1] = tmp;
}


// Looks like a double mantissa in hex or dec notation?
static int is_mantissa_only(char *p) {
  char *digs;

  if(*p == '+' || *p == '-')
    p++;

  if(*p == '0' && (p[1] == 'x' || p[1] == 'X')) {
    p += 2;
    digs = "0123456789abcdefABCDEF";
  } else
    digs = "0123456789";

  if(!*p)
    return 0;

  while(*p)
    if(!strchr(digs, *p++))
      return 0;

  return 1;
}

// Return 1 if all n bytes in memory pointed to by p are c, 0 otherwise
int memall(const void *p, char c, size_t n) {
  const char *q = (const char *) p;
  return n <= 0 || (*q == c && memcmp(q, q+1, n-1) == 0);
}

// Like strtoull but knows binary, too
unsigned long long int str_ull(const char *str, char **endptr, int base) {
  const char *nptr = (char *) str;
  unsigned long long int ret = 0;
  int neg = 0;

  while(isspace(*nptr & 0xff))
    nptr++;

  // Check explicit sign for benefit of 0b...
  if(*nptr == '-' || *nptr == '+') {
    if(*nptr == '-')
      neg = 1;
    nptr++;
    // Don't allow double signs
    if(*nptr == '-' || *nptr == '+') {
      if(endptr)
        *endptr = (char *) nptr;
      return ret;
    }
  }

  if((base == 0 || base == 2) && *nptr && (nptr[1] == 'b' || nptr[1] == 'B'))
    base = 2, nptr+=2;
  else if((base == 0 || base == 16) && *nptr && (nptr[1] == 'x' || nptr[1] == 'X'))
    base = 16, nptr+=2;

  errno = 0;
  ret = strtoull(nptr, endptr, base);
  if(endptr && *endptr == nptr)
    *endptr = (char *) str;

  if(neg && errno == 0)
    ret = -ret;

  return ret;
}


/*
 * str_todata() is the workhorse for generic string to data conversion for the terminal write
 * function, but is also used for generic string to integer conversions in str_int() below. Both
 * routines define the "character" of how avrdude understands strings in (most) of its dealings.
 * The granularity of type is an bitwise-or combination of bits making up STR_INTEGER; STR_FLOAT;
 * STR_DOUBLE or STR_STRING.
 */

#define Return(...) do { \
  sd->errstr = str_sprintf(__VA_ARGS__); \
  sd->type = 0; \
  free(str); \
  return sd; \
} while (0)

#define Warning(...) do { \
  if(sd->warnstr) \
    free(sd->warnstr); \
   sd->warnstr = str_sprintf(__VA_ARGS__); \
} while (0)

#define sizeforsigned(ll) ( \
  (ll) < INT32_MIN || (ll) > INT32_MAX? 8: \
  (ll) < INT16_MIN || (ll) > INT16_MAX? 4: \
  (ll) < INT8_MIN  || (ll) > INT8_MAX? 2: 1)

Str2data *str_todata(const char *s, int type) {
  char *end_ptr;
  Str2data *sd = cfg_malloc(__func__, sizeof *sd);
  char *str = cfg_strdup(__func__, s);

  size_t arglen = strlen(str);
  // Remove trailing comma to allow cut and paste of lists
  if(arglen > 0 && str[arglen-1] == ',')
    str[--arglen] = 0;

  if(arglen == 0)
    Return("no data to convert");

  // Try integers and assign data size
  if(type & STR_INTEGER) {
    bool is_big_endian, is_signed = 0, is_outside_int64_t = 0, is_out_of_range = 0;
    char *stri = str;

    while(isspace(*stri & 0xff))
      stri++;

    sd->ull = 1;
    if(sizeof(long long) != sizeof(int64_t) || (sd->a[0]^sd->a[7]) != 1)
      Return("assumption on data types not met? Check source and recompile\n");
    is_big_endian = sd->a[7];

    sd->sigsz = sd->size = 0;
    errno = 0;
    sd->ull = str_ull(stri, &end_ptr, 0);

    if(!(end_ptr == stri || errno)) {
      unsigned int nu=0, nl=0, nh=0, ns=0, nx=0;

      // Parse suffixes: ULL, LL, UL, L ... UHH, HH
      for(char *p=end_ptr; *p; p++) {
        switch(toupper(*p)) {
        case 'U': nu++; break;
        case 'L': nl++; break;
        case 'H': nh++; break;
        case 'S': ns++; break;
        default: nx++;
        }
      }

      if(nx==0 && nu<2 && nl<3 && nh<3 && ns<2) { // Could be valid integer suffix
        // If U, then must be at start or end
        if(nu==0 || toupper(*end_ptr) == 'U' || toupper(str[arglen-1]) == 'U') {
          bool is_hex, is_bin;
          int ndigits;

          is_hex = str_casestarts(stri, "0x");    // Ordinary hex without explicit +/- sign
          is_bin = str_casestarts(stri, "0b");    // Ordinary bin without explicit +/- sign
          ndigits = end_ptr - stri - 2;           // Used for is_hex and is_bin
          is_signed = !(nu || is_hex || is_bin);  // Neither explicitly unsigned nor 0x/0b

          if(is_signed) {       // Is input in range for int64_t?
            if(*stri == '-' && (sd->ull == ~(~0ULL>>1) || sd->ll > 0))
              is_outside_int64_t = 1;
            if(*stri != '-' && sd->ll < 0)
              is_outside_int64_t = 1;
          }

          // Set size
          if(nl==0 && ns==0 && nh==0) { // No explicit data size
            // Ordinary hex/bin get implicit size by number of digits, including leading zeros
            if(is_hex) {
              sd->size = ndigits > 8? 8: ndigits > 4? 4: ndigits > 2? 2: 1;
            } else if(is_bin) {
              sd->size = ndigits > 32? 8: ndigits > 16? 4: ndigits > 8? 2: 1;
            } else if(is_signed) {
              // Smallest size that fits signed or unsigned (asymmetric to meet user expectation)
              sd->size =
                is_outside_int64_t? 8:
                sd->ll < INT32_MIN || sd->ll > (long long) UINT32_MAX? 8:
                sd->ll < INT16_MIN || sd->ll > (long long) UINT16_MAX? 4:
                sd->ll < INT8_MIN  || sd->ll > (long long) UINT8_MAX? 2: 1;
              if(sd->size < 8)  // sigsz is the one needed for signed int
                sd->sigsz = sizeforsigned(sd->ll);
            } else {
              // Smallest size that fits unsigned representation
              sd->size =
                sd->ull > UINT32_MAX? 8:
                sd->ull > UINT16_MAX? 4:
                sd->ull > UINT8_MAX? 2: 1;
            }
          } else if(nl==0 && nh==2 && ns==0) { // HH
            sd->size = 1;
            if(is_signed && (sd->ll < INT8_MIN  || sd->ll > INT8_MAX))
              is_out_of_range = 1;                    // out of range if uint64 and -uint64 are
            else if(!is_signed && sd->ull > UINT8_MAX && -sd->ull > UINT8_MAX)
              is_out_of_range = 1;
            if(is_signed)
              sd->sigsz = sizeforsigned(sd->ll);
          } else if(nl==0 && ((nh==1 && ns==0) || (nh==0 && ns==1))) { // H or S
            sd->size = 2;
            if(is_signed && (sd->ll < INT16_MIN  || sd->ll > INT16_MAX))
              is_out_of_range = 1;
            else if(!is_signed && sd->ull > UINT16_MAX && -sd->ull > UINT16_MAX)
              is_out_of_range = 1;
            if(is_signed)
              sd->sigsz = sizeforsigned(sd->ll);
          } else if(nl==1 && nh==0 && ns==0) { // L
            sd->size = 4;
            if(is_signed && (sd->ll < INT32_MIN  || sd->ll > INT32_MAX))
              is_out_of_range = 1;
            else if(!is_signed && sd->ull > UINT32_MAX && -sd->ull > UINT32_MAX)
              is_out_of_range = 1;
            if(is_signed)
              sd->sigsz = sizeforsigned(sd->ll);
          } else if(nl==2 && nh==0 && ns==0) { // LL
            sd->size = 8;
          }
        }
      }
    }
    if(sd->size) {
      if(sd->sigsz < sd->size)
        sd->sigsz = sd->size;
      if(is_big_endian && sd->size > 1) // Ensure little endian format
        change_endian(sd->a, sd->size);
      if(sd->sigsz < 8)                 // Curtail and sign extend the number
        memset(sd->a+sd->sigsz, is_signed && (sd->a[sd->sigsz-1] & 0x80)? 0xff: 0, 8-sd->sigsz);

      if(is_signed && is_out_of_range)
        Warning("%s out of int%d range, interpreted as %d-byte %lld%sU",
          stri, sd->size*8, sd->size, sd->ll, sd->size == 4? "L": sd->size==2? "H": "HH");
      else if(is_out_of_range)
        Warning("%s out of uint%d range, interpreted as %d-byte %llu",
          stri, sd->size*8, sd->size, sd->ull);
      else if(is_outside_int64_t)
        Warning("%s out of int64 range (consider U suffix)", stri);

      sd->type = STR_INTEGER;
      free(str);
      return sd;
    }
  }

  if(type & STR_DOUBLE) {       // Try double next, must have D suffix
    sd->d = strtod(str, &end_ptr);
    if (end_ptr != str && toupper(*end_ptr) == 'D' && end_ptr[1] == 0) {
      sd->size = 8;
      sd->type = STR_DOUBLE;
      free(str);
      return sd;
    }
  }

  if(type & STR_FLOAT) {        // Try float next
    sd->size = 0;
    sd->f = strtof(str, &end_ptr);
    if (end_ptr != str && toupper(*end_ptr) == 'F' && end_ptr[1] == 0)
      sd->size = 4;
    // Do not accept valid mantissa-only floats that are integer rejects (eg, 078 or ULL overflows)
    if(end_ptr != str && *end_ptr == 0 && !is_mantissa_only(str))
      sd->size = 4;
    if(sd->size) {
      sd->type = STR_FLOAT;
      free(str);
      return sd;
    }
  }

  if(type & STR_STRING) {       // Try C-style string or single character
    if((*str == '\'' && str[arglen-1] == '\'') || (*str == '\"' && str[arglen-1] == '\"')) {
      char *s = calloc(arglen-1, 1);
      if (s == NULL)
        Return("out of memory");

      // Strip start and end quotes, and unescape C string
      strncpy(s, str+1, arglen-2);
      cfg_unescape(s, s);
      if (*str == '\'') {         // Single C-style character
        if(*s && s[1])
          Warning("only using first character of %s", str);
        sd->a[0] = *s;
        memset(sd->a+1, 0, 7);
        sd->sigsz = sd->size = 1;
        sd->type = STR_INTEGER;
        free(s);
      } else {                    // C-style string
        sd->str_ptr = s;
        sd->type = STR_STRING;
      }
      free(str);
      return sd;
    }
  }

  Return("cannot parse");
}

// Free the data structure returned by str_todata()
void str_freedata(Str2data *sd) {
  if(sd) {
    if(sd->warnstr)
      free(sd->warnstr);
    if(sd->errstr)
      free(sd->errstr);
    free(sd);
  }
}
