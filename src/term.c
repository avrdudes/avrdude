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

#include "ac_cfg.h"

#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>

#if defined(HAVE_LIBREADLINE)
#  include <readline/readline.h>
#  include <readline/history.h>
#endif

#include "avrdude.h"
#include "term.h"

struct command {
  char * name;
  int (*func)(PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);
  char * desc;
};


static int cmd_dump  (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_write (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_erase (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_sig   (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_part  (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_help  (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_quit  (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_send  (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_parms (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_vtarg (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_varef (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_fosc  (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_sck   (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_spi   (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_pgm   (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_verbose (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

static int cmd_quell (PROGRAMMER * pgm, struct avrpart * p,
		      int argc, char *argv[]);

struct command cmd[] = {
  { "dump",  cmd_dump,  "%s <memory> [<addr> <len> | <addr> ... | <addr> | ...]" },
  { "read",  cmd_dump,  "alias for dump" },
  { "write", cmd_write, "%s <memory> <addr> [<data>[,] {<data>[,]} | <len> <data>[,] {<data>[,]} ...]" },
  { "erase", cmd_erase, "perform a chip erase" },
  { "sig",   cmd_sig,   "display device signature bytes" },
  { "part",  cmd_part,  "display the current part information" },
  { "send",  cmd_send,  "send a raw command: %s <b1> <b2> <b3> <b4>" },
  { "parms", cmd_parms, "display adjustable parameters (STK500 and Curiosity Nano only)" },
  { "vtarg", cmd_vtarg, "set <V[target]> (STK500 and Curiosity Nano only)" },
  { "varef", cmd_varef, "set <V[aref]> (STK500 only)" },
  { "fosc",  cmd_fosc,  "set <oscillator frequency> (STK500 only)" },
  { "sck",   cmd_sck,   "set <SCK period> (STK500 only)" },
  { "spi",   cmd_spi,   "enter direct SPI mode" },
  { "pgm",   cmd_pgm,   "return to programming mode" },
  { "verbose", cmd_verbose, "change verbosity" },
  { "quell", cmd_quell, "set quell level for progress bars" },
  { "help",  cmd_help,  "help" },
  { "?",     cmd_help,  "help" },
  { "quit",  cmd_quit,  "quit" }
};

#define NCMDS ((int)(sizeof(cmd)/sizeof(struct command)))



static int spi_mode = 0;

static int nexttok(char * buf, char ** tok, char ** next)
{
  unsigned char *q, *n;

  q = (unsigned char *) buf;
  while (isspace(*q))
    q++;

  /* isolate first token */
  n = q;
  uint8_t quotes = 0;
  while (*n && (!isspace(*n) || quotes)) {
    // poor man's quote and escape processing
    if (*n == '"' || *n == '\'')
      quotes++;
    else if(*n == '\\' && n[1])
      n++;
    else if (isspace(*n) && (n > q+1) && (n[-1] == '"' || n[-1] == '\''))
      break;
    n++;
  }

  if (*n) {
    *n = 0;
    n++;
  }

  /* find start of next token */
  while (isspace(*n))
    n++;

  *tok  = (char *) q;
  *next = (char *) n;

  return 0;
}


static int hexdump_line(char * buffer, unsigned char * p, int n, int pad)
{
  char * hexdata = "0123456789abcdef";
  char * b = buffer;
  int i = 0;
  int j = 0;

  for (i=0; i<n; i++) {
    if (i && ((i % 8) == 0))
      b[j++] = ' ';
    b[j++] = hexdata[(p[i] & 0xf0) >> 4];
    b[j++] = hexdata[(p[i] & 0x0f)];
    if (i < 15)
      b[j++] = ' ';
  }

  for (i=j; i<pad; i++)
    b[i] = ' ';

  b[i] = 0;

  for (i=0; i<pad; i++) {
    if (!((b[i] == '0') || (b[i] == ' ')))
      return 0;
  }

  return 1;
}


static int chardump_line(char * buffer, unsigned char * p, int n, int pad)
{
  int i;
  unsigned char b[128];

  // sanity check
  n = n < 1? 1: n > sizeof b? sizeof b: n;

  memcpy(b, p, n);
  for (int i = 0; i < n; i++)
    buffer[i] = isascii(b[i]) && isspace(b[i])? ' ':
      isascii(b[i]) && isgraph(b[i])? b[i]: '.';

  for (i = n; i < pad; i++)
    buffer[i] = ' ';

  buffer[i] = 0;

  return 0;
}


static int hexdump_buf(FILE * f, int startaddr, unsigned char * buf, int len)
{
  char dst1[80];
  char dst2[80];

  int addr = startaddr;
  unsigned char * p = (unsigned char *)buf;
  while (len) {
    int n = 16;
    if (n > len)
      n = len;
    hexdump_line(dst1, p, n, 48);
    chardump_line(dst2, p, n, 16);
    fprintf(stdout, "%04x  %s  |%s|\n", addr, dst1, dst2);
    len -= n;
    addr += n;
    p += n;
  }

  return 0;
}


static int cmd_dump(PROGRAMMER * pgm, struct avrpart * p,
		    int argc, char * argv[])
{
  if (argc < 2 || argc > 4) {
    terminal_message(MSG_INFO,
      "Usage: %s <memory> <addr> <len>\n"
      "       %s <memory> <addr> ...\n"
      "       %s <memory> <addr>\n"
      "       %s <memory> ...\n"
      "       %s <memory>\n",
      argv[0], argv[0], argv[0], argv[0], argv[0]);
    return -1;
  }

  enum { read_size = 256 };
  static char prevmem[AVR_MEMDESCLEN] = {0x00};
  char * memtype = argv[1];
  AVRMEM * mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    terminal_message(MSG_INFO, "%s (dump): %s memory type not defined for part %s\n",
      progname, memtype, p->desc);
    return -1;
  }
  int maxsize = mem->size;

  // Get start address if present
  char * end_ptr;
  static int addr = 0;

  if (argc >= 3 && strcmp(argv[2], "...") != 0) {
    addr = strtoul(argv[2], &end_ptr, 0);
    if (*end_ptr || (end_ptr == argv[2])) {
      terminal_message(MSG_INFO, "%s (dump): can't parse address %s\n",
        progname, argv[2]);
      return -1;
    } else if (addr >= maxsize) {
      terminal_message(MSG_INFO, "%s (dump): address 0x%05lx is out of range for %s memory\n",
        progname, (long) addr, mem->desc);
      return -1;
    }
  }

  // Get no. bytes to read if present
  static int len = read_size;
  if (argc >= 3) {
    memset(prevmem, 0x00, sizeof(prevmem));
    if (strcmp(argv[argc - 1], "...") == 0) {
      if (argc == 3)
        addr = 0;
      len = maxsize - addr;
    } else if (argc == 4) {
      len = strtol(argv[3], &end_ptr, 0);
      if (*end_ptr || (end_ptr == argv[3])) {
        terminal_message(MSG_INFO, "%s (dump): can't parse length %s\n",
          progname, argv[3]);
        return -1;
      }
    } else {
      len = read_size;
    }
  }
  // No address or length specified
  else if (argc == 2) {
    if (strncmp(prevmem, memtype, strlen(memtype)) != 0) {
      addr = 0;
      len  = read_size;
      strncpy(prevmem, memtype, sizeof(prevmem) - 1);
      prevmem[sizeof(prevmem) - 1] = 0;
    }
    if (addr >= maxsize)
      addr = 0; // Wrap around
  }

  // Trim len if nessary to not read past the end of memory
  if ((addr + len) > maxsize)
    len = maxsize - addr;

  uint8_t * buf = malloc(len);
  if (buf == NULL) {
    terminal_message(MSG_INFO, "%s (dump): out of memory\n", progname);
    return -1;
  }

  report_progress(0, 1, "Reading");
  for (int i = 0; i < len; i++) {
    int rc = pgm->read_byte(pgm, p, mem, addr + i, &buf[i]);
    if (rc != 0) {
      terminal_message(MSG_INFO, "%s (dump): error reading %s address 0x%05lx of part %s\n",
        progname, mem->desc, (long) addr + i, p->desc);
      if (rc == -1)
        terminal_message(MSG_INFO, "%*sread operation not supported on memory type %s\n",
          (int) strlen(progname)+9, "", mem->desc);
      return -1;
    }
    report_progress(i, len, NULL);
  }
  report_progress(1, 1, NULL);

  hexdump_buf(stdout, addr, buf, len);
  fprintf(stdout, "\n");

  free(buf);

  addr = addr + len;

  return 0;
}


static size_t maxstrlen(int argc, char **argv) {
  size_t max = 0;

  for(int i=0; i<argc; i++)
    if(strlen(argv[i]) > max)
      max = strlen(argv[i]);

  return max;
}


// Change data item p of size bytes from big endian to little endian and vice versa
static void change_endian(void *p, int size) {
  uint8_t tmp, *w = p;

  for(int i=0; i<size/2; i++)
    tmp = w[i], w[i] = w[size-i-1], w[size-i-1] = tmp;
}


// Looks like a double mantissa in hex or dec notation
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


static int cmd_write(PROGRAMMER * pgm, struct avrpart * p,
		     int argc, char * argv[])
{
  if (argc < 4) {
    terminal_message(MSG_INFO,
      "Usage: write <memory> <addr> <data>[,] {<data>[,]} \n"
      "       write <memory> <addr> <len> <data>[,] {<data>[,]} ...\n"
      "\n"
      "Ellipsis ... writes <len> bytes padded by repeating the last <data> item.\n"
      "\n"
      "<data> can be hexadecimal, octal or decimal integers, floating point numbers\n"
      "or C-style strings and characters. For integers, an optional case-insensitive\n"
      "suffix specifies the data size: HH 8 bit, H/S 16 bit, L 32 bit, LL 64 bit.\n"
      "Suffix D indicates a 64-bit double, F a 32-bit float, whilst a floating point\n"
      "number without suffix  defaults to 32-bit float. Hexadecimal floating point\n"
      "notation is supported. An ambiguous trailing suffix, eg, 0x1.8D, is read as\n"
      "no-suffix float where D is part of the mantissa; use a zero exponent 0x1.8p0D\n"
      "to clarify.\n"
      "\n"
      "An optional U suffix makes integers unsigned. Ordinary 0x hex integers are\n"
      "always treated as unsigned. +0x or -0x hex numbers are treated as signed\n"
      "unless they have a U suffix. Unsigned integers cannot be larger than 2^64-1.\n"
      "If n is an unsigned integer then -n is also a valid unsigned integer as in C.\n"
      "Signed integers must fall into the [-2^63, 2^63-1] range or a correspondingly\n"
      "smaller range when a suffix specifies a smaller type. Out of range signed\n"
      "numbers trigger a warning.\n"
      "\n"
      "Ordinary 0x hex integers with n hex digits (counting leading zeros) use the\n"
      "smallest size of 1, 2, 4 and 8 bytes that can accommodate any n-digit hex\n"
      "integer. If an integer suffix specifies a size explicitly the corresponding\n"
      "number of least significant bytes are written. Otherwise, signed and unsigned\n"
      "integers alike occupy the smallest of 1, 2, 4, or 8 bytes needed to\n"
      "accommodate them in their respective representation.\n"
    );
    return -1;
  }

  int i;
  uint8_t write_mode;       // Operation mode, "standard" or "fill"
  uint8_t start_offset;     // Which argc argument
  int len;                  // Number of bytes to write to memory
  char * memtype = argv[1]; // Memory name string
  AVRMEM * mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    terminal_message(MSG_INFO, "%s (write): %s memory type not defined for part %s\n",
      progname, memtype, p->desc);
    return -1;
  }
  int maxsize = mem->size;

  char * end_ptr;
  int addr = strtoul(argv[2], &end_ptr, 0);
  if (*end_ptr || (end_ptr == argv[2])) {
    terminal_message(MSG_INFO, "%s (write): can't parse address %s\n",
      progname, argv[2]);
    return -1;
  }

  if (addr > maxsize) {
    terminal_message(MSG_INFO, "%s (write): address 0x%05lx is out of range for %s memory\n",
      progname, (long) addr, memtype);
    return -1;
  }

  // Allocate a buffer guaranteed to be large enough
  uint8_t * buf = calloc(mem->size + 8 + maxstrlen(argc-3, argv+3)+1, sizeof(uint8_t));
  if (buf == NULL) {
    terminal_message(MSG_INFO, "%s (write): out of memory\n", progname);
    return -1;
  }

  // Find the first argument to write to flash and how many arguments to parse and write
  if (strcmp(argv[argc - 1], "...") == 0) {
    write_mode = WRITE_MODE_FILL;
    start_offset = 4;
    len = strtoul(argv[3], &end_ptr, 0);
    if (*end_ptr || (end_ptr == argv[3])) {
      terminal_message(MSG_INFO, "%s (write ...): can't parse length %s\n",
        progname, argv[3]);
      free(buf);
      return -1;
    }
  } else {
    write_mode = WRITE_MODE_STANDARD;
    start_offset = 3;
    len = argc - start_offset;
  }

  // Structure related to data that is being written to memory
  struct Data {
    // Data info
    int bytes_grown;
    uint8_t size;
    char *str_ptr;
    // Data union
    union {
      float f;
      double d;
      int64_t ll;
      uint64_t ull;
      uint8_t a[8];
    };
  } data = {
    .bytes_grown = 0,
    .size        = 0,
    .str_ptr     = NULL,
    .ull         = 1
  };

  if(sizeof(long long) != sizeof(int64_t) || (data.a[0]^data.a[7]) != 1)
    terminal_message(MSG_INFO, "%s (write): assumption on data types not met? "
      "Check source and recompile\n", progname);
  bool is_big_endian = data.a[7];

  for (i = start_offset; i < len + start_offset; i++) {
    // Handle the next argument
    if (i < argc - start_offset + 3) {
      char *argi = argv[i];
      size_t arglen = strlen(argi);

      data.size = 0;

      // Free string pointer if already allocated
      if(data.str_ptr) {
        free(data.str_ptr);
        data.str_ptr = NULL;
      }

      // Remove trailing comma to allow cut and paste of lists
      if(arglen > 0 && argi[arglen-1] == ',')
        argi[--arglen] = 0;

      // Try integers and assign data size
      errno = 0;
      data.ull = strtoull(argi, &end_ptr, 0);
      if (!(end_ptr == argi || errno)) {
        unsigned int nu=0, nl=0, nh=0, ns=0, nx=0;
        char *p;

        // Parse suffixes: ULL, LL, UL, L ... UHH, HH
        for(p=end_ptr; *p; p++)
          switch(toupper(*p)) {
          case 'U': nu++; break;
          case 'L': nl++; break;
          case 'H': nh++; break;
          case 'S': ns++; break;
          default: nx++;
          }

        if(nx==0 && nu<2 && nl<3 && nh<3 && ns<2) { // Could be valid integer suffix
          if(nu==0 || toupper(*end_ptr) == 'U' || toupper(p[-1]) == 'U') { // If U, then must be at start or end
            bool is_hex = strncasecmp(argi, "0x", 2) == 0; // Ordinary hex: 0x... without explicit +/- sign
            bool is_signed = !(nu || is_hex);              // Neither explicitly unsigned nor ordinary hex
            bool is_outside_int64_t = 0;
            bool is_out_of_range = 0;
            int nhexdigs = p-argi-2;

            if(is_signed) {   // Is input in range for int64_t?
              errno = 0; (void) strtoll(argi, NULL, 0);
              is_outside_int64_t = errno == ERANGE;
            }

            if(nl==0 && ns==0 && nh==0) { // No explicit data size
              // Ordinary hex numbers have implicit size given by number of hex digits, including leading zeros
              if(is_hex) {
                data.size = nhexdigs > 8? 8: nhexdigs > 4? 4: nhexdigs > 2? 2: 1;

              } else if(is_signed) {
                // Smallest size that fits signed representation
                data.size =
                  is_outside_int64_t? 8:
                  data.ll < INT32_MIN || data.ll > INT32_MAX? 8:
                  data.ll < INT16_MIN || data.ll > INT16_MAX? 4:
                  data.ll < INT8_MIN  || data.ll > INT8_MAX? 2: 1;

              } else {
                // Smallest size that fits unsigned representation
                data.size =
                  data.ull > UINT32_MAX? 8:
                  data.ull > UINT16_MAX? 4:
                  data.ull > UINT8_MAX? 2: 1;
              }
            } else if(nl==0 && nh==2 && ns==0) { // HH
              data.size = 1;
              if(is_outside_int64_t || (is_signed && (data.ll < INT8_MIN  || data.ll > INT8_MAX))) {
                is_out_of_range = 1;
                data.ll = (int8_t) data.ll;
              }
            } else if(nl==0 && ((nh==1 && ns==0) || (nh==0 && ns==1))) { // H or S
              data.size = 2;
              if(is_outside_int64_t || (is_signed && (data.ll < INT16_MIN  || data.ll > INT16_MAX))) {
                is_out_of_range = 1;
                data.ll = (int16_t) data.ll;
              }
            } else if(nl==1 && nh==0 && ns==0) { // L
              data.size = 4;
              if(is_outside_int64_t || (is_signed && (data.ll < INT32_MIN  || data.ll > INT32_MAX))) {
                is_out_of_range = 1;
                data.ll = (int32_t) data.ll;
              }
            } else if(nl==2 && nh==0 && ns==0) { // LL
              data.size = 8;
            }

            if(is_outside_int64_t || is_out_of_range)
              terminal_message(MSG_INFO, "%s (write): %s out of int%d_t range, "
                "interpreted as %d-byte %lld; consider 'U' suffix\n",
                progname, argi, data.size*8, data.size, data.ll);
          }
        }
      }

      if(!data.size) {          // Try double now that input was rejected as integer
        data.d = strtod(argi, &end_ptr);
        if (end_ptr != argi && toupper(*end_ptr) == 'D' && end_ptr[1] == 0)
          data.size = 8;
      }

      if(!data.size) {          // Try float
        data.f = strtof(argi, &end_ptr);
        if (end_ptr != argi && toupper(*end_ptr) == 'F' && end_ptr[1] == 0)
          data.size = 4;
        if (end_ptr != argi && *end_ptr == 0) // no suffix defaults to float but ...
        // ... do not accept valid mantissa-only floats that are integer rejects (eg, 078 or ULL overflows)
          if (!is_mantissa_only(argi))
            data.size = 4;
      }

      if(!data.size && arglen > 1) { // Try C-style string or single character
        if ((*argi == '\'' && argi[arglen-1] == '\'') || (*argi == '\"' && argi[arglen-1] == '\"')) {
          char *s = calloc(arglen-1, 1);
          if (s == NULL) {
            terminal_message(MSG_INFO, "%s (write str): out of memory\n", progname);
            free(buf);
            return -1;
          }
          // Strip start and end quotes, and unescape C string
          strncpy(s, argi+1, arglen-2);
          cfg_unescape(s, s);
          if (*argi == '\'') { // Single C-style character
            if(*s && s[1])
              terminal_message(MSG_INFO, "%s (write): only using first character of %s\n",
                progname, argi);
            data.ll = *s;
            data.size = 1;
            free(s);
          } else {             // C-style string
            data.str_ptr = s;
          }
        }
      }

      if(!data.size && !data.str_ptr) {
        terminal_message(MSG_INFO, "%s (write): can't parse data %s\n",
          progname, argi);
        free(buf);
        return -1;
      }

      // Assume endianness is the same for double and int, and ensure little endian representation
      if(is_big_endian && data.size > 1)
        change_endian(data.a, data.size);
    }

    if(data.str_ptr) {
      for(size_t j = 0; j < strlen(data.str_ptr); j++)
        buf[i - start_offset + data.bytes_grown++] = (uint8_t)data.str_ptr[j];
    } else if(data.size > 0) {
      for(int k=0; k<data.size; k++)
        buf[i - start_offset + data.bytes_grown + k] = data.a[k];
      data.bytes_grown += data.size-1;
    }

    // Make sure buf does not overflow
    if (i - start_offset + data.bytes_grown > maxsize)
      break;
  }

  // When in "fill" mode, the maximum size is already predefined
  if (write_mode == WRITE_MODE_FILL)
    data.bytes_grown = 0;

  if ((addr + len + data.bytes_grown) > maxsize) {
    terminal_message(MSG_INFO, "%s (write): selected address and # bytes exceed "
      "range for %s memory\n", progname, memtype);
    free(buf);
    return -1;
  }

  if(data.str_ptr)
    free(data.str_ptr);

  terminal_message(MSG_NOTICE, "Info: writing %d bytes starting from address 0x%02lx",
    len + data.bytes_grown, (long) addr);
  if (write_mode == WRITE_MODE_FILL)
    terminal_message(MSG_NOTICE, "; remaining space filled with %s", argv[argc - 2]);
  terminal_message(MSG_NOTICE, "\n");

  pgm->err_led(pgm, OFF);
  bool werror = false;
  report_progress(0, 1, "Writing");
  for (i = 0; i < (len + data.bytes_grown); i++) {
    int rc = avr_write_byte(pgm, p, mem, addr+i, buf[i]);
    if (rc) {
      terminal_message(MSG_INFO, "%s (write): error writing 0x%02x at 0x%05lx, rc=%d\n",
        progname, buf[i], (long) addr+i, (int) rc);
      if (rc == -1)
        terminal_message(MSG_INFO, "%*swrite operation not supported on memory type %s\n",
          (int) strlen(progname)+10, "", mem->desc);
      werror = true;
    }

    uint8_t b;
    rc = pgm->read_byte(pgm, p, mem, addr+i, &b);
    if (b != buf[i]) {
      terminal_message(MSG_INFO, "%s (write): error writing 0x%02x at 0x%05lx cell=0x%02x\n",
        progname, buf[i], (long) addr+i, b);
      werror = true;
    }

    if (werror) {
      pgm->err_led(pgm, ON);
    }

    report_progress(i, (len + data.bytes_grown), NULL);
  }
  report_progress(1, 1, NULL);

  free(buf);

  return 0;
}


static int cmd_send(PROGRAMMER * pgm, struct avrpart * p,
		    int argc, char * argv[])
{
  unsigned char cmd[4], res[4];
  char * e;
  int i;
  int len;

  if (pgm->cmd == NULL) {
    terminal_message(MSG_INFO, "%s (send): the %s programmer does not support direct ISP commands\n",
      progname, pgm->type);
    return -1;
  }

  if (spi_mode && (pgm->spi == NULL)) {
    terminal_message(MSG_INFO, "%s (send): the %s programmer does not support direct SPI transfers\n",
      progname, pgm->type);
    return -1;
  }


  if ((argc > 5) || ((argc < 5) && (!spi_mode))) {
    terminal_message(MSG_INFO, spi_mode?
      "Usage: send <byte1> [<byte2> [<byte3> [<byte4>]]]\n":
      "Usage: send <byte1> <byte2> <byte3> <byte4>\n");
    return -1;
  }

  /* number of bytes to write at the specified address */
  len = argc - 1;

  /* load command bytes */
  for (i=1; i<argc; i++) {
    cmd[i-1] = strtoul(argv[i], &e, 0);
    if (*e || (e == argv[i])) {
      terminal_message(MSG_INFO, "%s (send): can't parse byte %s\n",
        progname, argv[i]);
      return -1;
    }
  }

  pgm->err_led(pgm, OFF);

  if (spi_mode)
    pgm->spi(pgm, cmd, res, argc-1);
  else
    pgm->cmd(pgm, cmd, res);

  /*
   * display results
   */
  terminal_message(MSG_INFO, "results:");
  for (i=0; i<len; i++)
    terminal_message(MSG_INFO, " %02x", res[i]);
  terminal_message(MSG_INFO, "\n");

  fprintf(stdout, "\n");

  return 0;
}


static int cmd_erase(PROGRAMMER * pgm, struct avrpart * p,
		     int argc, char * argv[])
{
  terminal_message(MSG_INFO, "%s: erasing chip\n", progname);
  pgm->chip_erase(pgm, p);
  return 0;
}


static int cmd_part(PROGRAMMER * pgm, struct avrpart * p,
		    int argc, char * argv[])
{
  fprintf(stdout, "\n");
  avr_display(stdout, p, "", 0);
  fprintf(stdout, "\n");

  return 0;
}


static int cmd_sig(PROGRAMMER * pgm, struct avrpart * p,
		   int argc, char * argv[])
{
  int i;
  int rc;
  AVRMEM * m;

  rc = avr_signature(pgm, p);
  if (rc != 0) {
    terminal_message(MSG_INFO, "%s (sig): error reading signature data, rc=%d\n",
      progname, rc);
  }

  m = avr_locate_mem(p, "signature");
  if (m == NULL) {
    terminal_message(MSG_INFO, "%s (sig): signature data not defined for device %s\n",
      progname, p->desc);
  }
  else {
    fprintf(stdout, "Device signature = 0x");
    for (i=0; i<m->size; i++)
      fprintf(stdout, "%02x", m->buf[i]);
    fprintf(stdout, "\n\n");
  }

  return 0;
}


static int cmd_quit(PROGRAMMER * pgm, struct avrpart * p,
		    int argc, char * argv[])
{
  /* FUSE bit verify will fail if left in SPI mode */
  if (spi_mode) {
    cmd_pgm(pgm, p, 0, NULL);
  }
  return 1;
}


static int cmd_parms(PROGRAMMER * pgm, struct avrpart * p,
		     int argc, char * argv[])
{
  if (pgm->print_parms == NULL) {
    terminal_message(MSG_INFO, "%s (parms): the %s programmer does not support "
      "adjustable parameters\n", progname, pgm->type);
    return -1;
  }
  pgm->print_parms(pgm);

  return 0;
}


static int cmd_vtarg(PROGRAMMER * pgm, struct avrpart * p,
		     int argc, char * argv[])
{
  int rc;
  double v;
  char *endp;

  if (argc != 2) {
    terminal_message(MSG_INFO, "Usage: vtarg <value>\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    terminal_message(MSG_INFO, "%s (vtarg): can't parse voltage %s\n",
      progname, argv[1]);
    return -1;
  }
  if (pgm->set_vtarget == NULL) {
    terminal_message(MSG_INFO, "%s (vtarg): the %s programmer cannot set V[target]\n",
      progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_vtarget(pgm, v)) != 0) {
    terminal_message(MSG_INFO, "%s (vtarg): failed to set V[target] (rc = %d)\n",
      progname, rc);
    return -3;
  }
  return 0;
}


static int cmd_fosc(PROGRAMMER * pgm, struct avrpart * p,
		    int argc, char * argv[])
{
  int rc;
  double v;
  char *endp;

  if (argc != 2) {
    terminal_message(MSG_INFO, "Usage: fosc <value>[M|k] | off\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    if (strcmp(argv[1], "off") == 0)
      v = 0.0;
    else {
      terminal_message(MSG_INFO, "%s (fosc): can't parse frequency %s\n",
        progname, argv[1]);
      return -1;
    }
  }
  if (*endp == 'm' || *endp == 'M')
    v *= 1e6;
  else if (*endp == 'k' || *endp == 'K')
    v *= 1e3;
  if (pgm->set_fosc == NULL) {
    terminal_message(MSG_INFO, "%s (fosc): the %s programmer cannot set oscillator frequency\n",
      progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_fosc(pgm, v)) != 0) {
    terminal_message(MSG_INFO, "%s (fosc): failed to set oscillator frequency (rc = %d)\n",
      progname, rc);
    return -3;
  }
  return 0;
}


static int cmd_sck(PROGRAMMER * pgm, struct avrpart * p,
		   int argc, char * argv[])
{
  int rc;
  double v;
  char *endp;

  if (argc != 2) {
    terminal_message(MSG_INFO, "Usage: sck <value>\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    terminal_message(MSG_INFO, "%s (sck): can't parse period %s\n",
      progname, argv[1]);
    return -1;
  }
  v *= 1e-6;			/* Convert from microseconds to seconds. */
  if (pgm->set_sck_period == NULL) {
    terminal_message(MSG_INFO, "%s (sck): the %s programmer cannot set SCK period\n",
      progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_sck_period(pgm, v)) != 0) {
    terminal_message(MSG_INFO, "%s (sck): failed to set SCK period (rc = %d)\n",
      progname, rc);
    return -3;
  }
  return 0;
}


static int cmd_varef(PROGRAMMER * pgm, struct avrpart * p,
		     int argc, char * argv[])
{
  int rc;
  unsigned int chan;
  double v;
  char *endp;

  if (argc != 2 && argc != 3) {
    terminal_message(MSG_INFO, "Usage: varef [channel] <value>\n");
    return -1;
  }
  if (argc == 2) {
    chan = 0;
    v = strtod(argv[1], &endp);
    if (endp == argv[1]) {
      terminal_message(MSG_INFO, "%s (varef): can't parse voltage %s\n",
        progname, argv[1]);
      return -1;
    }
  } else {
    chan = strtoul(argv[1], &endp, 10);
    if (endp == argv[1]) {
      terminal_message(MSG_INFO, "%s (varef): can't parse channel %s\n",
        progname, argv[1]);
      return -1;
    }
    v = strtod(argv[2], &endp);
    if (endp == argv[2]) {
      terminal_message(MSG_INFO, "%s (varef): can't parse voltage %s\n",
        progname, argv[2]);
      return -1;
    }
  }
  if (pgm->set_varef == NULL) {
    terminal_message(MSG_INFO, "%s (varef): the %s programmer cannot set V[aref]\n",
      progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_varef(pgm, chan, v)) != 0) {
    terminal_message(MSG_INFO, "%s (varef): failed to set V[aref] (rc = %d)\n",
      progname, rc);
    return -3;
  }
  return 0;
}


static int cmd_help(PROGRAMMER * pgm, struct avrpart * p,
		    int argc, char * argv[])
{
  int i;

  fprintf(stdout, "Valid commands:\n");
  for (i=0; i<NCMDS; i++) {
    fprintf(stdout, "  %-7s : ", cmd[i].name);
    fprintf(stdout, cmd[i].desc, cmd[i].name);
    fprintf(stdout, "\n");
  }
  fprintf(stdout,
          "\nUse the 'part' command to display valid memory types for use with the\n"
          "'dump' and 'write' commands.\n\n");

  return 0;
}

static int cmd_spi(PROGRAMMER * pgm, struct avrpart * p,
        int argc, char * argv[])
{
  if (pgm->setpin != NULL) {
    pgm->setpin(pgm, PIN_AVR_RESET, 1);
    spi_mode = 1;
    return 0;
  }
  terminal_message(MSG_INFO, "%s: spi command unavailable for this programmer type\n",
    progname);
  return -1;
}

static int cmd_pgm(PROGRAMMER * pgm, struct avrpart * p,
        int argc, char * argv[])
{
  if (pgm->setpin != NULL) {
    pgm->setpin(pgm, PIN_AVR_RESET, 0);
    spi_mode = 0;
    pgm->initialize(pgm, p);
    return 0;
  }
  terminal_message(MSG_INFO, "%s: pgm command unavailable for this programmer type\n",
    progname);
  return -1;
}

static int cmd_verbose(PROGRAMMER * pgm, struct avrpart * p,
		       int argc, char * argv[])
{
  int nverb;
  char *endp;

  if (argc != 1 && argc != 2) {
    terminal_message(MSG_INFO, "Usage: verbose [<value>]\n");
    return -1;
  }
  if (argc == 1) {
    terminal_message(MSG_INFO, "Verbosity level: %d\n", verbose);
    return 0;
  }
  nverb = strtol(argv[1], &endp, 0);
  if (endp == argv[1] || *endp) {
    terminal_message(MSG_INFO, "%s (verbose): can't parse verbosity level %s\n",
      progname, argv[1]);
    return -1;
  }
  if (nverb < 0) {
    terminal_message(MSG_INFO, "%s: verbosity level must not be negative: %d\n",
      progname, nverb);
    return -1;
  }
  verbose = nverb;
  terminal_message(MSG_INFO, "New verbosity level: %d\n", verbose);

  return 0;
}

static int cmd_quell(PROGRAMMER * pgm, struct avrpart * p,
		       int argc, char * argv[])
{
  int nquell;
  char *endp;

  if (argc != 1 && argc != 2) {
    terminal_message(MSG_INFO, "Usage: quell [<value>]\n");
    return -1;
  }
  if (argc == 1) {
    terminal_message(MSG_INFO, "Quell level: %d\n", quell_progress);
    return 0;
  }
  nquell = strtol(argv[1], &endp, 0);
  if (endp == argv[1] || *endp) {
    terminal_message(MSG_INFO, "%s (quell): can't parse quell level %s\n",
      progname, argv[1]);
    return -1;
  }
  if (nquell < 0) {
    terminal_message(MSG_INFO, "%s: quell level must not be negative: %d\n",
      progname, nquell);
    return -1;
  }
  quell_progress = nquell;
  terminal_message(MSG_INFO, "New quell level: %d\n", quell_progress);

  if(quell_progress > 0)
    update_progress = NULL;
  else
    terminal_setup_update_progress();

  return 0;
}

static int tokenize(char * s, char *** argv)
{
  int     i, n, l, k, nargs, offset;
  int     len, slen;
  char  * buf;
  int     bufsize;
  char ** bufv;
  char  * bufp;
  char  * q, * r;
  char  * nbuf;
  char ** av;

  slen = strlen(s);

  /*
   * initialize allow for 20 arguments, use realloc to grow this if
   * necessary
   */
  nargs   = 20;
  bufsize = slen + 20;
  buf     = malloc(bufsize);
  bufv    = (char **) malloc(nargs*sizeof(char *));
  for (i=0; i<nargs; i++) {
    bufv[i] = NULL;
  }
  buf[0] = 0;

  n    = 0;
  l    = 0;
  nbuf = buf;
  r    = s;
  while (*r) {
    nexttok(r, &q, &r);
    strcpy(nbuf, q);
    bufv[n]  = nbuf;
    len      = strlen(q);
    l       += len + 1;
    nbuf    += len + 1;
    nbuf[0]  = 0;
    n++;
    if ((n % 20) == 0) {
      char *buf_tmp;
      char **bufv_tmp;
      /* realloc space for another 20 args */
      bufsize += 20;
      nargs   += 20;
      bufp     = buf;
      buf_tmp  = realloc(buf, bufsize);
      if (buf_tmp == NULL) {
        free(buf);
        free(bufv);
        return -1;
      }
      buf = buf_tmp;
      bufv_tmp = realloc(bufv, nargs*sizeof(char *));
      if (bufv_tmp == NULL) {
        free(buf);
        free(bufv);
        return -1;
      }
      bufv = bufv_tmp;
      nbuf     = &buf[l];
      /* correct bufv pointers */
      k = buf - bufp;
      for (i=0; i<n; i++) {
          bufv[i] = bufv[i] + k;
      }
      for (i=n; i<nargs; i++)
        bufv[i] = NULL;
    }
  }

  /*
   * We have parsed all the args, n == argc, bufv contains an array of
   * pointers to each arg, and buf points to one memory block that
   * contains all the args, back to back, seperated by a nul
   * terminator.  Consilidate bufv and buf into one big memory block
   * so that the code that calls us, will have an easy job of freeing
   * this memory.
   */
  av = (char **) malloc(slen + n + (n+1)*sizeof(char *));
  q  = (char *)&av[n+1];
  memcpy(q, buf, l);
  for (i=0; i<n; i++) {
    offset = bufv[i] - buf;
    av[i] = q + offset;
  }
  av[i] = NULL;

  free(buf);
  free(bufv);

  *argv = av;

  return n;
}


static int do_cmd(PROGRAMMER * pgm, struct avrpart * p,
		  int argc, char * argv[])
{
  int i;
  int hold;
  int len;

  len = strlen(argv[0]);
  hold = -1;
  for (i=0; i<NCMDS; i++) {
    if (strcasecmp(argv[0], cmd[i].name) == 0) {
      return cmd[i].func(pgm, p, argc, argv);
    }
    else if (strncasecmp(argv[0], cmd[i].name, len)==0) {
      if (hold != -1) {
        terminal_message(MSG_INFO, "%s (cmd): command %s is ambiguous\n",
          progname, argv[0]);
        return -1;
      }
      hold = i;
    }
  }

  if (hold != -1)
    return cmd[hold].func(pgm, p, argc, argv);

  terminal_message(MSG_INFO, "%s (cmd): invalid command %s\n",
    progname, argv[0]);

  return -1;
}


char * terminal_get_input(const char *prompt)
{
#if defined(HAVE_LIBREADLINE) && !defined(WIN32)
  char *input;
  input = readline(prompt);
  if ((input != NULL) && (strlen(input) >= 1))
    add_history(input);

  return input;
#else
  char input[256];
  printf("%s", prompt);
  if (fgets(input, sizeof(input), stdin))
  {
    /* FIXME: readline strips the '\n', should this too? */
    return strdup(input);
  }
  else
    return NULL;
#endif
}


int terminal_mode(PROGRAMMER * pgm, struct avrpart * p)
{
  char  * cmdbuf;
  char  * q;
  int     rc;
  int     argc;
  char ** argv;

  rc = 0;
  while ((cmdbuf = terminal_get_input("avrdude> ")) != NULL) {
    /*
     * find the start of the command, skipping any white space
     */
    q = cmdbuf;
    while (*q && isspace((unsigned char) *q))
      q++;

    /* skip blank lines and comments */
    if (!*q || (*q == '#'))
      continue;

    /* tokenize command line */
    argc = tokenize(q, &argv);
    if (argc < 0) {
      free(cmdbuf);
      return argc;
    }

#if !defined(HAVE_LIBREADLINE) || defined(WIN32) || defined(__APPLE__)
    fprintf(stdout, ">>> ");
    for (int i=0; i<argc; i++)
      fprintf(stdout, "%s ", argv[i]);
    fprintf(stdout, "\n");
#endif

    /* run the command */
    rc = do_cmd(pgm, p, argc, argv);
    free(argv);
    if (rc > 0) {
      rc = 0;
      break;
    }
    free(cmdbuf);
  }

  return rc;
}


int terminal_message(const int msglvl, const char *format, ...) {
  int rc = 0;
  va_list ap;

  fflush(stdout); fflush(stderr);
  if (verbose >= msglvl) {
    va_start(ap, format);
    rc = vfprintf(stderr, format, ap);
    va_end(ap);
  }
  fflush(stderr);

  return rc;
}


static void update_progress_tty (int percent, double etime, char *hdr)
{
  static char hashes[51];
  static char *header;
  static int last = 0;
  int i;

  setvbuf(stderr, (char*)NULL, _IONBF, 0);

  hashes[50] = 0;

  memset (hashes, ' ', 50);
  for (i=0; i<percent; i+=2) {
    hashes[i/2] = '#';
  }

  if (hdr) {
    avrdude_message(MSG_INFO, "\n");
    last = 0;
    header = hdr;
  }

  if (last == 0) {
    avrdude_message(MSG_INFO, "\r%s | %s | %d%% %0.2fs",
            header, hashes, percent, etime);
  }

  if (percent == 100) {
    if (!last) avrdude_message(MSG_INFO, "\n\n");
    last = 1;
  }

  setvbuf(stderr, (char*)NULL, _IOLBF, 0);
}

static void update_progress_no_tty (int percent, double etime, char *hdr)
{
  static int done = 0;
  static int last = 0;
  int cnt = (percent>>1)*2;

  setvbuf(stderr, (char*)NULL, _IONBF, 0);

  if (hdr) {
    avrdude_message(MSG_INFO, "\n%s | ", hdr);
    last = 0;
    done = 0;
  }
  else {
    while ((cnt > last) && (done == 0)) {
      avrdude_message(MSG_INFO, "#");
      cnt -=  2;
    }
  }

  if ((percent == 100) && (done == 0)) {
    avrdude_message(MSG_INFO, " | 100%% %0.2fs\n\n", etime);
    last = 0;
    done = 1;
  }
  else
    last = (percent>>1)*2;    /* Make last a multiple of 2. */

  setvbuf(stderr, (char*)NULL, _IOLBF, 0);
}

void terminal_setup_update_progress() {
  if (isatty (STDERR_FILENO))
    update_progress = update_progress_tty;
  else {
    update_progress = update_progress_no_tty;
    /* disable all buffering of stderr for compatibility with
       software that captures and redirects output to a GUI
       i.e. Programmers Notepad */
    setvbuf( stderr, NULL, _IONBF, 0 );
    setvbuf( stdout, NULL, _IONBF, 0 );
  }
}
