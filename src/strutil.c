/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2023 Hans Eirik Bull
 * Copyright (C) 2023 Stefan Rueger <stefan.rueger@urclocks.com>
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
#include <math.h>

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
 * not. Note: str_match_core() is a modified old copy of !fnmatch() from the
 * GNU C Library (published under GLP v2), which uses shell wildcards for
 * constructing patterns, ie, *, ? and single character classes, eg, [^0-6].
 * Used for portability.
 */

inline static int fold(int c) {
  return (c >= 'A' && c <= 'Z')? c+('a'-'A'): c;
}

inline static int nofold(int c) {
  return c;
}

static int str_match_core(const char *pattern, const char *string, int (*fold)(int c)) {
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
          if((c == '[' || fold(*n) == c1) && str_match_core(p, n, fold) == 1)
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

int str_match(const char *pattern, const char *string) {
  return str_match_core(pattern, string, nofold);
}

int str_casematch(const char *pattern, const char *string) {
  return str_match_core(pattern, string, fold);
}

int str_matched_by(const char *string, const char *pattern) {
  return str_match_core(pattern, string, nofold);
}

int str_casematched_by(const char *string, const char *pattern) {
  return str_match_core(pattern, string, fold);
}

// Does the string contain wildcard pattern matching elements?
int str_is_pattern(const char *str) {
  for(;;)
    switch(*str++) {
      case 0: return 0;
      case '*': case '?': case '[': case '\\': return 1;
    }
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


// Reads a potentially long line and returns it in a malloc'd buffer
char *str_fgets(FILE *fp, const char **errpp) {
  int bs = 1023;                // Must be 2^n - 1
  char *ret = (char *) cfg_malloc(__func__, bs);

  ret[bs-2] = 0;
  if(!fgets(ret, bs, fp)) {
    free(ret);
    if(errpp)
      *errpp = ferror(fp) && !feof(fp)? "I/O error": NULL;
    return NULL;
  }

  while(ret[bs-2] != 0 && ret[bs-2] != '\n' && ret[bs-2] != '\r') {
    if(bs >= INT_MAX/2) {
      free(ret);
      if(errpp)
        *errpp = "cannot cope with lines longer than INT_MAX/2 bytes";
      return NULL;
    }
    int was = bs;
    bs = 2*bs+1;
    ret = cfg_realloc(__func__, ret, bs);
    ret[was-1] = ret[bs-2] = 0;
    if(!fgets(ret+was-1, bs-(was-1), fp)) { // EOF? Error?
      if(ferror(fp)) {
        free(ret);
        if(errpp)
          *errpp = "I/O error";
        return NULL;
      }
      break;
    }
  }

  if(errpp)
    *errpp = NULL;
  return ret;
}


// Changes string to be all lowercase and returns original pointer
char *str_lc(char *s) {
  for(char *t = s; *t; t++)
    *t = tolower(*t & 0xff);
  return s;
}

// Changes string to be all uppercase and returns original pointer
char *str_uc(char *s) {
  for(char *t = s; *t; t++)
    *t = toupper(*t & 0xff);
  return s;
}

// Changes first character in a string to be lowercase and returns original pointer
char *str_lcfirst(char *s) {
  *s = tolower(*s & 0xff);
  return s;
}

// Changes first character in a string to be uppercase and returns original pointer
char *str_ucfirst(char *s) {
  *s = toupper(*s & 0xff);
  return s;
}


// Convert unsigned to ASCII string; caller needs to allocate enough space for buf
char *str_utoa(unsigned n, char *buf, int base) {
  unsigned q;
  char *cp;

  if(base == 'r') {
    const char *units = "IVXLCDMFTYHSNabcdefghijkl";
    const char *rep[10] = {"", "a", "aa", "aaa", "ab", "b", "ba", "baa", "baaa", "ac"};

    if(n == 0) {
      strcpy(buf, "0");
      return buf;
    }

    int i = 0;
    for(unsigned u = n; u; u /= 10)
      i++;
    for(*buf = 0; i > 0; i--) {
      unsigned u = n;
      for(int j=1; j<i; j++)
        u /= 10;
      char *q = buf+strlen(buf);
      for(const char *p = rep[u%10], *d = units + (i-1)*2; *p; p++)
        *q++ = d[*p-'a'];
      *q = 0;
    }
    return buf;
  }

  if(base < 2 || base > 36) {
    *buf = 0;
    return buf;
  }

  cp = buf;
  /*
   * Divide by base until the number disappeared, but ensure at least
   * one digit will be emitted.
   */
  do {
    q = n % base;
    n /= base;
    *cp++ = q < 10? q + '0': q + 'a'-10;
  } while(n);

  // Terminate the string
  *cp-- = 0;

  // Revert the result string
  for(char *cp2 = buf; cp > cp2; ) {
    char c = *cp;
    *cp-- = *cp2;
    *cp2++ = c;
  }

  return buf;
}

// Returns a pointer to the start of a trailing number in the string or NULL if not there
char *str_endnumber(const char *str) {
  const char *ret = NULL;

  for(const char *end = str + strlen(str)-1; end >= str; end--)
    if(isdigit((unsigned char) *end))
      ret = end;
    else
      break;

  return (char *) ret;
}


// Convenience functions for printing
const char *str_plural(int x) {
  return x==1? "": "s";
}

const char *str_inname(const char *fn) {
  return !fn? "???": strcmp(fn, "-")? fn: "<stdin>";
}

const char *str_outname(const char *fn) {
  return !fn? "???": strcmp(fn, "-")? fn: "<stdout>";
}

// Return sth like "[0, 0x1ff]"
const char *str_interval(int a, int b) {
  // Cyclic buffer for 20+ temporary interval strings each max 41 bytes at 64-bit int
  static char space[20*41 + 80], *sp;
  if(!sp || sp-space > (int) sizeof space - 80)
    sp = space;

  char *ret = sp;

  sprintf(sp, a<16? "[%d": "[0x%x", a);
  sp += strlen(sp);
  sprintf(sp, b<16? ", %d]": ", 0x%x]", b);

  // Advance beyond return string in temporary ring buffer
  sp += strlen(sp)+1;

  return ret;
}


bool is_bigendian() {
  union {char a[2]; int16_t i;} u = {.i = 1};
  return u.a[1] == 1;
}

// Change data item p of size bytes from big endian to little endian and vice versa
void change_endian(void *p, int size) {
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


// https://en.wikipedia.org/wiki/Easter_egg_(media)#Software
unsigned long long int easteregg(const char *str, const char **endpp) {
  unsigned long long int ret = 0;
  struct {
    char chr[3];
    unsigned lim, nxt, val;
  } eet[] = {
    { "M",  3, 1, 1000 },
    { "CM", 1, 4,  900 },
    { "D",  1, 2,  500 },
    { "CD", 1, 2,  400 },
    { "C",  3, 1,  100 },
    { "XC", 1, 4,   90 },
    { "L",  1, 2,   50 },
    { "XL", 1, 2,   40 },
    { "X",  3, 1,   10 },
    { "IX", 1, 4,    9 },
    { "V",  1, 2,    5 },
    { "IV", 1, 2,    4 },
    { "I",  3, 1,    1 },
  }, *dig;

  for(size_t i = 0; i < sizeof eet/sizeof*eet; ) {
    dig = eet+i;
    unsigned lim = dig->lim;
    size_t ni = i+1;
    for(unsigned j=0; j<lim; j++) {
      if(!str_starts(str, dig->chr))
        break;
      ret += dig->val;
      if(ret < dig->val) {
        if(endpp)
          *endpp = str;
        return 0;
      }
      str += strlen(dig->chr);
      ni = i + dig->nxt;
    }
    if(!*str)
      break;
    i = ni;
  }
  if(endpp)
    *endpp = str;

  return ret;
}


// Like strtoull but knows binary, too
unsigned long long int str_ull(const char *str, char **endptr, int base) {
  const char *nptr = str, *ep;
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

  if((base == 0 || base == 2) && *nptr == '0' && (nptr[1] == 'b' || nptr[1] == 'B'))
    base = 2, nptr+=2;
  else if((base == 0 || base == 16) && *nptr == '0' && (nptr[1] == 'x' || nptr[1] == 'X'))
    base = 16, nptr+=2;

  errno = 0;
  if((base == 0 || base == 'r') && (ret = easteregg(nptr, &ep)) && ep != nptr && !*ep) {
    if(endptr)
      *endptr = (char *) ep;
  } else {
    ret = strtoull(nptr, endptr, base);
    if(endptr && *endptr == nptr)
       *endptr = (char *) str;
  }


  if(neg && errno == 0)
    ret = ~ret+1;               // Same as -ret but silences overzealous compiler warnings

  return ret;
}


/*
 * str_todata() is the workhorse for generic string to data conversion for the terminal write
 * function, but is also used for generic string to integer conversions in str_int() below. Both
 * routines define the "character" of how avrdude understands strings in (most) of its dealings.
 * The granularity of type is an bitwise-or combination of bits making up STR_INTEGER; STR_FLOAT;
 * STR_DOUBLE or STR_STRING. The arguments part and memstr are only needed for input from files.
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

Str2data *str_todata(const char *s, int type, const AVRPART *part, const char *memstr) {
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
      Return("assumption on data types not met? Check source and recompile");
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
            else if(!is_signed && sd->ull > UINT8_MAX && ~sd->ull+1 > UINT8_MAX)
              is_out_of_range = 1;
            if(is_signed)
              sd->sigsz = sizeforsigned(sd->ll);
          } else if(nl==0 && ((nh==1 && ns==0) || (nh==0 && ns==1))) { // H or S
            sd->size = 2;
            if(is_signed && (sd->ll < INT16_MIN  || sd->ll > INT16_MAX))
              is_out_of_range = 1;
            else if(!is_signed && sd->ull > UINT16_MAX && ~sd->ull+1 > UINT16_MAX)
              is_out_of_range = 1;
            if(is_signed)
              sd->sigsz = sizeforsigned(sd->ll);
          } else if(nl==1 && nh==0 && ns==0) { // L
            sd->size = 4;
            if(is_signed && (sd->ll < INT32_MIN  || sd->ll > INT32_MAX))
              is_out_of_range = 1;
            else if(!is_signed && sd->ull > UINT32_MAX && ~sd->ull+1 > UINT32_MAX)
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
      if(sd->sigsz < 8) {               // Curtail and sign extend the number
        if(is_big_endian && sd->size > 1)
          change_endian(sd->a, sd->size);
        memset(sd->a+sd->sigsz, is_signed && (sd->a[sd->sigsz-1] & 0x80)? 0xff: 0, 8-sd->sigsz);
        if(is_big_endian)
          change_endian(sd->a, sizeof sd->a);
      }

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

  if(type & STR_STRING && arglen > 1) { // Try C-style string or single character
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

  if(type & STR_FILE && part && memstr) { // File name containing data to be loaded
    int format = FMT_AUTO;
    FILE *f;
    char fmtstr[4] = { 0 };

    if(arglen > 2 && str[arglen-2] == ':') {
      fmtstr[0] = ' '; strcpy(fmtstr+1, str+arglen-2);
      format = fileio_format(str[arglen-1]);
      if(format == FMT_ERROR)
        Return("unknown format%s suffix of file name", fmtstr);
       str[arglen-=2] = 0;
    }
    if(format == FMT_AUTO) {
      f = fileio_fopenr(str);
      if(f == NULL)
        Return("unable to read the%s file: %s", fmtstr, strerror(errno));
      format = fileio_fmt_autodetect_fp(f);
      fclose(f);

      if(format < 0)
        Return("cannot determine format for the file, specify explicitly");
    }
    // Obtain a copy of the part incl all memories
    AVRPART *dp = avr_dup_part(part);
    AVRMEM *mem = avr_locate_mem(dp, memstr);
    if(!mem) {
      avr_free_part(dp);
      Return("memory %s not configured for device %s", memstr, part->desc);
    }
    int rc = fileio(FIO_READ_FOR_VERIFY, str, format, dp, memstr, -1);
    if(rc < 0) {
      avr_free_part(dp);
      Return("unable to read the%s %s file", fmtstr, fileio_fmtstr(format));
    }
    sd->mem = avr_dup_mem(mem);
    sd->size = rc;
    avr_free_part(dp);
    sd->type = STR_FILE;
    free(str);
    return sd;
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
    if(sd->mem)
      avr_free_mem(sd->mem);
    free(sd);
  }
}


/*
 * Generic string to integer routine that conforms to avrdude terminal syntax
 *
 * Str points to a string that contains an integer terminal data item. Type can be STR_INTEGER or
 * a non-zero bitwise-or combination of integer size designators STR_1, STR_2, STR_4 and STR_8 and
 * sign type STR_SIGNED or, independently, STR_UNSIGNED. A corresponding range check will be done
 * for the numbers that are encoded in the string. If neither or both of STR_UNSIGNED and
 * STR_SIGNED was given then the admitted integer can be in either the signed or the unsigned
 * range of the given size, otherwise only numbers of the requested signedness range will be
 * admitted. Either way, if the string itself restricts the size through a size suffix (see below)
 * then any overflow in there will trigger a range error, too. As in C, a sign-changed unsigned
 * number will yield another unsigned number greater than or equal to zero. As such, the numbers
 * 0, -255U, -254U, ..., -1U are all valid unsigned representations of one-byte integers 0, 1, ...
 * 255. At the same time -256U and 256U are not in the one-byte unsigned range [0, 255]. Finally,
 * in the case of  success str_int() will set the character pointer pointed to by errptr to NULL
 * and return an integer in the range of the requested type as unsigned long long. In the case of
 * a conversion error, the pointer pointed to by errptr will be set to a human-readable error
 * message whilst the returned value has no meaning.
 *
 * Integer terminal data items are either a literal C-like character such as '\t' or an integer
 * string with optional leading white space; optional + or - sign; a binary (leading 0b), octal
 * (leading 0), decimal or hexadecimal number (leading 0x); optional size suffix LL/L/S/H/HH;
 * optional unsigned U suffix. All terminal data items can have an optional trailing comma to
 * allow cutting and pasting lists, and will undergo automated data size and base detection. The
 * known integer sizes are either 1 (suffix HH), 2 (suffix H or S), 4 (suffix L) or 8 bytes
 * (suffix LL).
 *
 * Usage example:
 *
 * #include "libavrdude.h"
 *
 * const char *errptr;
 * int addr = str_int(string, STR_INT32, &errptr);
 * if(errptr) {
 *   pmsg_error("(dump) address %s: %s\n", string, errptr);
 *   return -1;
 * }
 */

unsigned long long int str_int(const char *str, int type, const char **errpp) {
  char *tofree;
  const char *err = NULL;
  Str2data *sd = NULL;
  unsigned long long int ret = 0ULL;

  type &= STR_INTEGER;
  if(type == 0) {
    err = "no integral type requested in str_int()";
    goto finished;
  }

  sd = str_todata(str, type | STR_STRING, NULL, NULL);
  // 1<<lds is number of expected bytes
  int lds = type&STR_8? 3: type&STR_4? 2: type&STR_2? 1: type&STR_1? 0: 3;

  if(sd->type != STR_INTEGER || sd->errstr) {
    err = sd->errstr? cache_string(sd->errstr): "not an integral type";
    goto finished;
  }

  if(sd->warnstr && strstr(sd->warnstr, " out of ")) { // Convert out of range warning into error
    char *p = strstr(sd->warnstr, "out of ");
    if(p) {
      p = cfg_strdup(__func__, p);
      if(strchr(p, ','))
       *strchr(p, ',') = 0;
      err = cache_string(p);
      free(p);
    } else {
      err = "out of range";
    }
    goto finished;
  }

  if(sd->sigsz > (1<<lds)) {    // Check for range if returned size bigger than requested
    int signd = type & (STR_SIGNED|STR_UNSIGNED);
    long long int smin[4] = { INT8_MIN, INT16_MIN, INT32_MIN, INT64_MIN };
    long long int smax[4] = { INT8_MAX, INT16_MAX, INT32_MAX, INT64_MAX };
    unsigned long long int umax[4] = { UINT8_MAX, UINT16_MAX, UINT32_MAX, UINT64_MAX };

    if(signd == STR_SIGNED) {   // Strictly signed
      if(sd->ll < smin[lds] || sd->ll > smax[lds]) {
        err = cache_string(tofree=str_sprintf("out of int%d range", 1<<(3+lds)));
        free(tofree);
        goto finished;
      }
    } else if(signd == STR_UNSIGNED) { // Strictly unsigned are out of range if u and -u are
      if(sd->ull > umax[lds] && ~sd->ull+1 > umax[lds]) {
        err = cache_string(tofree=str_sprintf("out of uint%d range", 1<<(3+lds)));
        free(tofree);
        goto finished;
      }
    } else {                    // Neither strictly signed or unsigned
      if((sd->ll < smin[lds] || sd->ll > smax[lds]) && sd->ull > umax[lds] && ~sd->ull+1 > umax[lds]) {
        err = cache_string(tofree=str_sprintf("out of int%d and uint%d range", 1<<(3+lds), 1<<(3+lds)));
        free(tofree);
        goto finished;
      }
    }
  }

  ret = sd->ull;

finished:
  if(errpp)
    *errpp = err;
  str_freedata(sd);

  return ret;
}


// Convert a data string (except STR_FILE) to a memory buffer suitable for AVRMEM use
int str_membuf(const char *str, int type, unsigned char *buf, int size, const char **errpp) {
  int n = 0;
  const char *err = NULL;
  Str2data *sd = NULL;

  type &= ~STR_FILE;
  if(type == 0)                 // Nothing requested, nothing gained
    goto finished;

  sd = str_todata(str, type, NULL, NULL);
  if(!sd->type || sd->errstr) {
    err = cache_string(sd->errstr);
    n = -1;
    goto finished;
  }

  if(sd->type == STR_STRING && sd->str_ptr) {
    size_t len = strlen(sd->str_ptr);
    for(size_t j = 0; j < len && n < size; j++)
      buf[n++] = (uint8_t) sd->str_ptr[j];
    if(n < size)                // Terminating nul
      buf[n++] = 0;
  } else if(sd->size > 0 && (sd->type & STR_NUMBER)) {
     // Always write little endian to AVR memory
    if(is_bigendian() && sd->size > 0 && (sd->type & STR_NUMBER))
      change_endian(sd->a, sd->size);
    for(int k = 0; k < sd->size && n < size; k++)
      buf[n++] = sd->a[k];
  }


finished:
  if(errpp)
    *errpp = err;
  str_freedata(sd);

  return n;
}


/*
 * Returns the next space separated token in buf (terminating it) and
 * places start of next token into pointer pointed to by next. Keeps
 * single or double quoted strings together and changes backslash-space
 * sequences to space whilst keeping other backslashed characters.
 * Used for terminal line parsing and reading files with ASCII numbers.
 */
char *str_nexttok(char *buf, const char *delim, char **next) {
  unsigned char *q, *r, *w, inquote;

  q = (unsigned char *) buf;
  while(*q && strchr(delim, *q))
    q++;

  // Isolate first token
  for(inquote = 0, w = r = q; *r && !(strchr(delim, *r) && !inquote); *w++ = *r++) {
    // Poor man's quote and escape processing
    if(*r == '"' || *r == '\'')
      inquote = inquote && *r == inquote? 0: inquote? inquote: *r;
    else if(*r == '\\' && r[1] && strchr(delim, r[1])) // Remove \ before space for file names
      r++;
    else if(*r == '\\' && r[1])          // Leave other \ to keep C-style, eg, '\n'
      *w++ = *r++;
  }
  if(*r)
    r++;
  *w = 0;

  // Find start of next token
  while(*r && strchr(delim, *r))
    r++;

  if(next)
    *next = (char *) r;

  return (char *) q;
}

/*
 * From https://github.com/git/git/blob/master/levenshtein.c
 *
 * This function implements the Damerau-Levenshtein algorithm to
 * calculate a distance between strings.
 *
 * Basically, it says how many letters need to be swapped, substituted,
 * deleted from, or added to str1, at least, to get str2.
 *
 * The idea is to build a distance matrix for the substrings of both
 * strings.  To avoid a large space complexity, only the last three rows
 * are kept in memory (if swaps had the same or higher cost as one deletion
 * plus one insertion, only two rows would be needed).
 *
 * At any stage, "i + 1" denotes the length of the current substring of
 * str1 that the distance is calculated for.
 *
 * row2 holds the current row, row1 the previous row (i.e. for the substring
 * of str1 of length "i"), and row0 the row before that.
 *
 * In other words, at the start of the big loop, row2[j + 1] contains the
 * Damerau-Levenshtein distance between the substring of str1 of length
 * "i" and the substring of str2 of length "j + 1".
 *
 * All the big loop does is determine the partial minimum-cost paths.
 *
 * It does so by calculating the costs of the path ending in characters
 * i (in str1) and j (in str2), respectively, given that the last
 * operation is a substitution, a swap, a deletion, or an insertion.
 *
 * This implementation allows the costs to be weighted:
 *
 * - swap
 * - subst (as in "Substitution")
 * - add (for insertion, AKA "Add")
 * - del (as in "Deletion")
 *
 * Note that this algorithm calculates a distance _iff_ del == add.
 */

int str_levenshtein(const char *str1, const char *str2,
  int swap, int subst, int add, int del) {

  int i, j, len1 = strlen(str1), len2 = strlen(str2);
  int *row0 = cfg_malloc(__func__, (len2+1)*sizeof*row0);
  int *row1 = cfg_malloc(__func__, (len2+1)*sizeof*row1);
  int *row2 = cfg_malloc(__func__, (len2+1)*sizeof*row2);

  for (j = 0; j <= len2; j++)
    row1[j] = j * add;
  for (i = 0; i < len1; i++) {
    row2[0] = (i+1) * del;
    for (j = 0; j < len2; j++) {
      // Substitution
      row2[j+1] = row1[j] + subst * (str1[i] != str2[j]);
      // Swap
      if (i > 0 && j > 0 && str1[i-1] == str2[j] &&
        str1[i] == str2[j-1] && row2[j+1] > row0[j-1] + swap)
        row2[j+1] = row0[j-1] + swap;
      // Deletion
      if (row2[j+1] > row1[j+1] + del)
        row2[j+1] = row1[j+1] + del;
      // Insertion
      if (row2[j+1] > row2[j] + add)
        row2[j+1] = row2[j] + add;
    }
    int *temp = row0;
    row0 = row1;
    row1 = row2;
    row2 = temp;
  }
  i = row1[len2];
  free(row0);
  free(row1);
  free(row2);
  return i;
}


// Alphanumeric chars get the full weight, all others such as hyphen or underscore get less
static size_t wchr(size_t w, unsigned char c) {
  return isascii(c) && isalnum(c)? w: w >= 8? w/8: 1;
}

// Index of character in string or -1 of not found
static int chridx(char *str, char c) {
  char *e = strchr(str, c);
  return e? e-str: -1;
}

// (x, y) position of key on keyboard 1 being the centre
static void xypos(char c, double *x, double *y) {
  int num = chridx("1234567890", c);
  int upp = chridx("qwertyuiop", c);
  int mid = chridx("asdfghjkl", c);
  int low = chridx("zxcvbnm", c);

  // My laptop's keyboard layout: your mileage may vary (smr)
  *x = num >= 0? num: upp >= 0? upp + 0.5: mid >= 0? mid + 0.75: low >= 0? low + 1.25: -3.0;
  *y = num >= 0? 0.0: upp >= 0? 1.0:       mid >= 0? 2.0:        low >= 0? 3.0:        -3.0;
}


// Weight by keyboard distance
static size_t qwertydist(size_t w, unsigned char c1, unsigned char c2) {
  if(c1 == c2)
    return 0;

  double x1, y1, x2, y2;
  xypos(tolower(c1), &x1, &y1);
  xypos(tolower(c2), &x2, &y2);

  if(x1 == x2 && y1 == y2)
    return w;

  size_t ret = isalpha(c1) && isalpha(c2) && isupper(c1) != isupper(c2)? w/8: 0;
  ret += sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))/2.5 * w;

  return ret > w? w: ret > 0? ret: 1;
}

// Substitution cost considering qwerty keyboard typos and case
static size_t csubs(size_t w, unsigned char c1, unsigned char c2) {
  if(c1 >= 128 || c2 >= 128)
    return c1 != c2? w: 0;

  if(w < 8)
    w = 8;

  static size_t wmat[128][128];
  if(!wmat[0][1])               // Initialize weight matrix
    for(size_t k1 = 0; k1 < 128; k1++)
      for(size_t k2 = 0; k2 < 128; k2++)
        wmat[k1][k2] =
          k1 == k2? 0:
          !isalnum(k1) && !isalnum(k2)? w/8:
          !isalnum(k1) || !isalnum(k2)? w:
          isalpha(k1) && isalpha(k2) && tolower(k1) == tolower(k2)? w/8:
          qwertydist(w, k1, k2);

  return wmat[c1][c2];
}

// Cost of morphing s1 to s2 modelling typos and mix-up of non-alphanumeric letters
size_t str_weighted_damerau_levenshtein(const char *s1, const char *s2) {
  const size_t swap = 3;        // Transposing neighbouring letters is an easy mistake to make
  const size_t subst = 32, add = 32, del = 32; // Must be multiples of 8
  size_t i, j, len1 = strlen(s1), len2 = strlen(s2);
  size_t *row0 = cfg_malloc(__func__, (len2+1)*sizeof*row0);
  size_t *row1 = cfg_malloc(__func__, (len2+1)*sizeof*row1);
  size_t *row2 = cfg_malloc(__func__, (len2+1)*sizeof*row2);
  unsigned char *str1 = (unsigned char *) s1, *str2 = (unsigned char *) s2;

  for(j = 0; j < len2; j++)
    row1[j+1] = row1[j]+ wchr(add, str2[j]);
  for(i = 0; i < len1; i++) {
    row2[0] = 0;
    for(size_t k = 0; k <= i; k++)
      row2[0] += wchr(del, str1[k]);
    for(j = 0; j < len2; j++) {
      // Substitution of str1[i] with str2[j]
      row2[j+1] = row1[j] + (str1[i] != str2[j]? csubs(subst, str1[i], str2[j]): 0);
      // Swap: str1[i-1]str1[i] is same as str2[j]str2[j-1]
      if(i > 0 && j > 0 && str1[i-1] == str2[j] && str1[i] == str2[j-1] && row2[j+1] > row0[j-1] + swap)
        row2[j+1] = row0[j-1] + swap;
      // Deletion of str1[i]
      size_t wdel = wchr(del, str1[i]);
      if(row2[j+1] > row1[j+1] + wdel)
        row2[j+1] = row1[j+1] + wdel;
      // Insertion of str2[j]
      size_t wadd = wchr(add, str2[j]);
      if(row2[j+1] > row2[j] + wadd)
        row2[j+1] = row2[j] + wadd;
      // Todo: fat finger, eg, typing test as tesdt or tedst
    }
    size_t *temp = row0;
    row0 = row1;
    row1 = row2;
    row2 = temp;
  }
  i = row1[len2];               // Last row2[len2]
  free(row0);
  free(row1);
  free(row2);
  return i;
}
