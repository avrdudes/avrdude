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
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdarg.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>

#if defined(HAVE_LIBREADLINE)
#include <readline/readline.h>
#include <readline/history.h>

#ifdef _MSC_VER
#include "msvc/unistd.h"
#else
#include <unistd.h>
#endif

#ifdef WIN32
#include <windows.h>
#else
#include <sys/select.h>
#endif
#endif


#include "avrdude.h"
#include "term.h"

struct command {
  char *name;
  int (*func)(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
  size_t fnoff;
  char *desc;
};


static int cmd_dump   (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_write  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_flush  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_abort  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_erase  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_pgerase(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_sig    (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_part   (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_help   (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_quit   (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_send   (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_parms  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_vtarg  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_varef  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_fosc   (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_sck    (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_spi    (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_pgm    (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_verbose(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);
static int cmd_quell  (PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]);

#define _fo(x) offsetof(PROGRAMMER, x)

struct command cmd[] = {
  { "dump",  cmd_dump,  _fo(read_byte_cached),  "%s <memory> [<addr> <len> | <addr> ... | <addr> | ...]" },
  { "read",  cmd_dump,  _fo(read_byte_cached),  "alias for dump" },
  { "write", cmd_write, _fo(write_byte_cached), "write <memory> <addr> <data>[,] {<data>[,]}" },
  { "",      cmd_write, _fo(write_byte_cached), "write <memory> <addr> <len> <data>[,] {<data>[,]} ..." },
  { "flush", cmd_flush, _fo(flush_cache),       "synchronise flash & EEPROM writes with the device" },
  { "abort", cmd_abort, _fo(reset_cache),       "abort flash & EEPROM writes (reset the r/w cache)" },
  { "erase", cmd_erase, _fo(chip_erase_cached), "perform a chip erase" },
  { "pgerase", cmd_pgerase, _fo(page_erase),    "pgerase <memory> <addr>" },
  { "sig",   cmd_sig,   _fo(open),              "display device signature bytes" },
  { "part",  cmd_part,  _fo(open),              "display the current part information" },
  { "send",  cmd_send,  _fo(cmd),               "send a raw command: %s <b1> <b2> <b3> <b4>" },
  { "parms", cmd_parms, _fo(print_parms),       "display adjustable parameters" },
  { "vtarg", cmd_vtarg, _fo(set_vtarget),       "set <V[target]>" },
  { "varef", cmd_varef, _fo(set_varef),         "set <V[aref]>" },
  { "fosc",  cmd_fosc,  _fo(set_fosc),          "set <oscillator frequency>" },
  { "sck",   cmd_sck,   _fo(set_sck_period),    "set <SCK period>" },
  { "spi",   cmd_spi,   _fo(setpin),            "enter direct SPI mode" },
  { "pgm",   cmd_pgm,   _fo(setpin),            "return to programming mode" },
  { "verbose", cmd_verbose, _fo(open),          "change verbosity" },
  { "quell", cmd_quell, _fo(open),              "set quell level for progress bars" },
  { "help",  cmd_help,  _fo(open),              "show help message" },
  { "?",     cmd_help,  _fo(open),              "same as help" },
  { "quit",  cmd_quit,  _fo(open),              "quit after writing out cache for flash & EEPROM" }
};

#define NCMDS ((int)(sizeof(cmd)/sizeof(struct command)))



static int spi_mode = 0;

static int nexttok(char *buf, char **tok, char **next) {
  unsigned char *q, *n;

  q = (unsigned char *) buf;
  while (isspace(*q))
    q++;

  /* isolate first token */
  n = q;
  uint8_t quotes = 0;
  while (*n && (!isspace(*n) || quotes)) {
    // Poor man's quote and escape processing
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


static int hexdump_line(char *buffer, unsigned char *p, int n, int pad) {
  char *hexdata = "0123456789abcdef";
  char *b = buffer;
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


static int chardump_line(char *buffer, unsigned char *p, int n, int pad) {
  int i;
  unsigned char b[128];

  // Sanity check
  n = n < 1? 1: n > (int) sizeof b? (int) sizeof b: n;

  memcpy(b, p, n);
  for (int i = 0; i < n; i++)
    buffer[i] = isascii(b[i]) && isspace(b[i])? ' ':
      isascii(b[i]) && isgraph(b[i])? b[i]: '.';

  for (i = n; i < pad; i++)
    buffer[i] = ' ';

  buffer[i] = 0;

  return 0;
}


static int hexdump_buf(FILE *f, AVRMEM *m, int startaddr, unsigned char *buf, int len) {
  char dst1[80];
  char dst2[80];

  int addr = startaddr;
  unsigned char *p = (unsigned char *) buf;
  while (len) {
    int n = 16;
    if (n > len)
      n = len;
    if(addr + n > m->size)
      n = m->size - addr;

    hexdump_line(dst1, p, n, 48);
    chardump_line(dst2, p, n, 16);
    term_out("%0*x  %s  |%s|\n", m->size > 0x10000 ? 5: 4, addr, dst1, dst2);
    len -= n;
    addr += n;
    if (addr >= m->size)
      addr = 0;
    p += n;
  }

  return 0;
}


static int cmd_dump(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  static struct mem_addr_len {
    int addr;
    int len;
    AVRMEM *mem;
  } read_mem[32];
  static int i;

  if ((argc < 2 && read_mem[0].mem == NULL) || argc > 4) {
    msg_error(
      "Usage: %s <memory> <addr> <len>\n"
      "       %s <memory> <addr> ...\n"
      "       %s <memory> <addr>\n"
      "       %s <memory> ...\n"
      "       %s <memory>\n",
      argv[0], argv[0], argv[0], argv[0], argv[0]);
    return -1;
  }

  enum { read_size = 256 };
  char *memtype;
  if(argc > 1)
    memtype = argv[1];
  else
    memtype = (char*)read_mem[i].mem->desc;
  AVRMEM *mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    pmsg_error("(dump) %s memory type not defined for part %s\n", memtype, p->desc);
    return -1;
  }

  int maxsize = mem->size;
  if(maxsize <= 0) { // Sanity check
    pmsg_error("cannot read memory %s of size %d\n", mem->desc, maxsize);
    return -1;
  }

  // Iterate through the read_mem structs to find relevant address and length info
  for(i = 0; i < 32; i++) {
    if(read_mem[i].mem == NULL)
      read_mem[i].mem = mem;
    if(read_mem[i].mem == mem) {
      if(read_mem[i].len == 0)
        read_mem[i].len = maxsize > read_size? read_size: maxsize;
      break;
    }
  }

  if(i >= 32) { // Catch highly unlikely case
    pmsg_error("read_mem[] under-dimensioned; increase and recompile\n");
    return -1;
  }

  // Get start address if present
  char *end_ptr;
  if (argc >= 3 && strcmp(argv[2], "...") != 0) {
    unsigned long ul = strtoul(argv[2], &end_ptr, 0);
    if(*end_ptr || (end_ptr == argv[2])) {
      pmsg_error("(dump) cannot parse address %s\n", argv[2]);
      return -1;
    }
    if(ul > INT_MAX || ul >= (unsigned long) maxsize) {
      pmsg_error("(dump) %s address 0x%lx is out of range [0, 0x%0*x]\n", mem->desc, ul,
        mem->size > 0x10000? 5: 4, maxsize-1);
      return -1;
    }
    read_mem[i].addr = (int) ul;
  }

  // Get no. bytes to read if present
  if (argc >= 3) {
    if (strcmp(argv[argc - 1], "...") == 0) {
      if (argc == 3)
        read_mem[i].addr = 0;
      read_mem[i].len = maxsize - read_mem[i].addr;
    } else if (argc == 4) {
      unsigned long ul = strtoul(argv[3], &end_ptr, 0);
      if (*end_ptr || (end_ptr == argv[3])) {
        pmsg_error("(dump) cannot parse length %s\n", argv[3]);
        return -1;
      }
      if (ul == 0 || ul > INT_MAX) // Not positive if used as int, make it 1
        ul = 1;
      read_mem[i].len = (int) ul;
    }
  }
  // Wrap around if the memory address is greater than the maximum size
  if(read_mem[i].addr >= maxsize)
    read_mem[i].addr = 0; // Wrap around

  // Trim len if nessary to prevent reading from the same memory address twice
  if (read_mem[i].len > maxsize)
    read_mem[i].len = maxsize;

  uint8_t *buf = malloc(read_mem[i].len);
  if (buf == NULL) {
    pmsg_error("(dump) out of memory\n");
    return -1;
  }

  if(argc < 4 && verbose)
    term_out(">>> %s %s 0x%x 0x%x\n", argv[0], read_mem[i].mem->desc, read_mem[i].addr, read_mem[i].len);

  report_progress(0, 1, "Reading");
  for (int j = 0; j < read_mem[i].len; j++) {
    int addr = (read_mem[i].addr + j) % mem->size;
    int rc = pgm->read_byte_cached(pgm, p, read_mem[i].mem, addr, &buf[j]);
    if (rc != 0) {
      report_progress(1, -1, NULL);
      pmsg_error("(dump) error reading %s address 0x%05lx of part %s\n", mem->desc, (long) read_mem[i].addr + j, p->desc);
      if (rc == -1)
        imsg_error("%*sread operation not supported on memory type %s\n", 7, "", mem->desc);
      free(buf);
      return -1;
    }
    report_progress(j, read_mem[i].len, NULL);
  }
  report_progress(1, 1, NULL);

  hexdump_buf(stdout, mem, read_mem[i].addr, buf, read_mem[i].len);
  term_out("\n");

  free(buf);

  read_mem[i].addr = (read_mem[i].addr + read_mem[i].len) % maxsize;

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


static int cmd_write(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  if (argc < 4) {
    msg_error(
      "Usage: write <memory> <addr> <data>[,] {<data>[,]}\n"
      "       write <memory> <addr> <len> <data>[,] {<data>[,]} ...\n"
      "\n"
      "Ellipsis ... writes <len> bytes padded by repeating the last <data> item.\n"
      "\n"
      "<data> can be hexadecimal, octal or decimal integers, floating point numbers\n"
      "or C-style strings and characters. For integers, an optional case-insensitive\n"
      "suffix specifies the data size: HH 8 bit, H/S 16 bit, L 32 bit, LL 64 bit.\n"
      "Suffix D indicates a 64-bit double, F a 32-bit float, whilst a floating point\n"
      "number without suffix defaults to 32-bit float. Hexadecimal floating point\n"
      "notation is supported. An ambiguous trailing suffix, eg, 0x1.8D, is read as\n"
      "no-suffix float where D is part of the mantissa; use a zero exponent 0x1.8p0D\n"
      "to clarify.\n"
      "\n"
      "An optional U suffix makes integers unsigned. Ordinary 0x hex integers are\n"
      "always treated as unsigned. +0x or -0x hex numbers are treated as signed\n"
      "unless they have a U suffix. Unsigned integers cannot be larger than 2^64-1.\n"
      "If n is an unsigned integer then -n is also a valid unsigned integer as in C.\n"
      "Signed integers must fall into the [-2^63, 2^63-1] range or a correspondingly\n"
      "smaller range when a suffix specifies a smaller type.\n"
      "\n"
      "Ordinary 0x hex integers with n hex digits (counting leading zeros) use the\n"
      "smallest size of one, two, four and eight bytes that can accommodate any\n"
      "n-digit hex integer. If an integer suffix specifies a size explicitly the\n"
      "corresponding number of least significant bytes are written, and a warning\n"
      "shown if the number does not fit into the desired representation. Otherwise,\n"
      "unsigned integers occupy the smallest of one, two, four or eight bytes\n"
      "needed. Signed numbers are allowed to fit into the smallest signed or\n"
      "smallest unsigned representation: For example, 255 is stored as one byte as\n"
      "255U would fit in one byte, though as a signed number it would not fit into a\n"
      "one-byte interval [-128, 127]. The number -1 is stored in one byte whilst -1U\n"
      "needs eight bytes as it is the same as 0xFFFFffffFFFFffffU.\n"
    );
    return -1;
  }

  int i;
  uint8_t write_mode;           // Operation mode, "standard" or "fill"
  uint8_t start_offset;         // Which argc argument
  int len;                      // Number of bytes to write to memory
  char *memtype = argv[1];      // Memory name string
  AVRMEM *mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    pmsg_error("(write) %s memory type not defined for part %s\n", memtype, p->desc);
    return -1;
  }
  int maxsize = mem->size;

  char *end_ptr;
  int addr = strtoul(argv[2], &end_ptr, 0);
  if (*end_ptr || (end_ptr == argv[2])) {
    pmsg_error("(write) cannot parse address %s\n", argv[2]);
    return -1;
  }

  if (addr < 0 || addr >= maxsize) {
    pmsg_error("(write) %s address 0x%05x is out of range [0, 0x%05x]\n", mem->desc, addr, maxsize-1);
    return -1;
  }

  // Allocate a buffer guaranteed to be large enough
  uint8_t *buf = calloc(mem->size + 8 + maxstrlen(argc-3, argv+3)+1, sizeof(uint8_t));
  if (buf == NULL) {
    pmsg_error("(write) out of memory\n");
    return -1;
  }

  // Find the first argument to write to flash and how many arguments to parse and write
  if (strcmp(argv[argc - 1], "...") == 0) {
    write_mode = WRITE_MODE_FILL;
    start_offset = 4;
    len = strtoul(argv[3], &end_ptr, 0);
    if (*end_ptr || (end_ptr == argv[3])) {
      pmsg_error("(write ...) cannot parse length %s\n", argv[3]);
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
    pmsg_error("(write) assumption on data types not met? "
      "Check source and recompile\n");
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

            if(is_signed) {     // Is input in range for int64_t?
              errno = 0; (void) strtoll(argi, NULL, 0);
              is_outside_int64_t = errno == ERANGE;
            }

            if(nl==0 && ns==0 && nh==0) { // No explicit data size
              // Ordinary hex numbers have implicit size given by number of hex digits, including leading zeros
              if(is_hex) {
                data.size = nhexdigs > 8? 8: nhexdigs > 4? 4: nhexdigs > 2? 2: 1;

              } else if(is_signed) {
                // Smallest size that fits signed or unsigned (asymmetric to meet user expectation)
                data.size =
                  is_outside_int64_t? 8:
                  data.ll < INT32_MIN || data.ll > (long long) UINT32_MAX? 8:
                  data.ll < INT16_MIN || data.ll > (long long) UINT16_MAX? 4:
                  data.ll < INT8_MIN  || data.ll > (long long) UINT8_MAX? 2: 1;

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
              if(is_outside_int64_t || is_signed)
                is_out_of_range = 1;
            }

            if(is_out_of_range)
              pmsg_error("(write) %s out of int%d_t range, "
                "interpreted as %d-byte %lld; consider 'U' suffix\n", argi, data.size*8, data.size, (long long int) data.ll);
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
        if (end_ptr != argi && *end_ptr == 0) // No suffix defaults to float but ...
          // ... do not accept valid mantissa-only floats that are integer rejects (eg, 078 or ULL overflows)
          if (!is_mantissa_only(argi))
            data.size = 4;
      }

      if(!data.size && arglen > 1) { // Try C-style string or single character
        if ((*argi == '\'' && argi[arglen-1] == '\'') || (*argi == '\"' && argi[arglen-1] == '\"')) {
          char *s = calloc(arglen-1, 1);
          if (s == NULL) {
            pmsg_error("(write str) out of memory\n");
            free(buf);
            return -1;
          }
          // Strip start and end quotes, and unescape C string
          strncpy(s, argi+1, arglen-2);
          cfg_unescape(s, s);
          if (*argi == '\'') {  // Single C-style character
            if(*s && s[1])
              pmsg_error("(write) only using first character of %s\n", argi);
            data.ll = *s;
            data.size = 1;
            free(s);
          } else {              // C-style string
            data.str_ptr = s;
          }
        }
      }

      if(!data.size && !data.str_ptr) {
        pmsg_error("(write) cannot parse data %s\n", argi);
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
    pmsg_error("(write) selected address and # bytes exceed "
      "range for %s memory\n", memtype);
    free(buf);
    return -1;
  }

  if(data.str_ptr)
    free(data.str_ptr);

  pmsg_notice2("(write) writing %d byte%s starting from address 0x%02lx",
    len + data.bytes_grown, update_plural(len + data.bytes_grown), (long) addr);
  if (write_mode == WRITE_MODE_FILL)
    msg_notice2("; remaining space filled with %s", argv[argc - 2]);
  msg_notice2("\n");

  pgm->err_led(pgm, OFF);
  bool werror = false;
  report_progress(0, 1, avr_has_paged_access(pgm, mem)? "Caching": "Writing");
  for (i = 0; i < len + data.bytes_grown; i++) {
    int rc = pgm->write_byte_cached(pgm, p, mem, addr+i, buf[i]);
    if (rc == LIBAVRDUDE_SOFTFAIL) {
      pmsg_warning("(write) programmer write protects %s address 0x%04x\n", mem->desc, addr+i);
    } else if(rc) {
      pmsg_error("(write) error writing 0x%02x at 0x%05lx, rc=%d\n", buf[i], (long) addr+i, (int) rc);
      if (rc == -1)
        imsg_error("%*swrite operation not supported on memory type %s\n", 8, "", mem->desc);
      werror = true;
    } else {
      uint8_t b;
      rc = pgm->read_byte_cached(pgm, p, mem, addr+i, &b);
      if (b != buf[i]) {
        pmsg_error("(write) verification error writing 0x%02x at 0x%05lx cell=0x%02x\n", buf[i], (long) addr+i, b);
        werror = true;
      }
   }

    if (werror)
      pgm->err_led(pgm, ON);

    report_progress(i, len + data.bytes_grown, NULL);
  }
  report_progress(1, 1, NULL);

  free(buf);

  return 0;
}


static int cmd_flush(PROGRAMMER *pgm, AVRPART *p, int ac, char *av[]) {
  pgm->flush_cache(pgm, p);
  return 0;
}


static int cmd_abort(PROGRAMMER *pgm, AVRPART *p, int ac, char *av[]) {
  pgm->reset_cache(pgm, p);
  return 0;
}


static int cmd_send(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  unsigned char cmd[4], res[4];
  char *e;
  int i;
  int len;

  if (spi_mode && (pgm->spi == NULL)) {
    pmsg_error("(send) the %s programmer does not support direct SPI transfers\n", pgm->type);
    return -1;
  }


  if ((argc > 5) || ((argc < 5) && (!spi_mode))) {
    msg_error(spi_mode?
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
      pmsg_error("(send) cannot parse byte %s\n", argv[i]);
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
  term_out("results:");
  for (i=0; i<len; i++)
    term_out(" %02x", res[i]);
  term_out("\n\n");

  return 0;
}


static int cmd_erase(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  term_out("erasing chip ...\n");

  // Erase chip and clear cache
  int rc = pgm->chip_erase_cached(pgm, p);

  if(rc == LIBAVRDUDE_SOFTFAIL) {
    pmsg_info("(erase) emulating chip erase by writing 0xff to flash ");
    AVRMEM *flm = avr_locate_mem(p, "flash");
    if(!flm) {
      msg_error("but flash not defined for part %s?\n", p->desc);
      return -1;
    }
    int addr, beg = 0, end = flm->size-1;
    if(pgm->readonly) {
      for(addr=beg; addr < flm->size; addr++)
        if(!pgm->readonly(pgm, p, flm, addr)) {
          beg = addr;
          break;
        }
      if(addr >= flm->size) {
        msg_info("but all flash is write protected\n");
        return 0;
      }
      for(addr=end; addr >= 0; addr--)
        if(!pgm->readonly(pgm, p, flm, addr)) {
          end = addr;
          break;
        }
    }

    msg_info("[0x%04x, 0x%04x]; undo with abort\n", beg, end);
    for(int addr=beg; addr <= end; addr++)
      if(!pgm->readonly || !pgm->readonly(pgm, p, flm, addr))
        if(pgm->write_byte_cached(pgm, p, flm, addr, 0xff) == -1)
          return -1;
    return 0;
  }

  if(rc) {
    pmsg_error("(erase) programmer %s failed erasing the chip\n", (char *) ldata(lfirst(pgm->id)));
    return -1;
  }

  return 0;
}


static int cmd_pgerase(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  if(argc < 3) {
    msg_error("Usage: pgerase <memory> <addr>\n");
    return -1;
  }

  char *memtype = argv[1];
  AVRMEM *mem = avr_locate_mem(p, memtype);
  if(!mem) {
    pmsg_error("(pgerase) %s memory type not defined for part %s\n", memtype, p->desc);
    return -1;
  }
  if(!avr_has_paged_access(pgm, mem)) {
    pmsg_error("(pgerase) %s memory cannot be paged addressed by %s\n", memtype, (char *) ldata(lfirst(pgm->id)));
    return -1;
  }

  int maxsize = mem->size;

  char *end_ptr;
  int addr = strtoul(argv[2], &end_ptr, 0);
  if(*end_ptr || (end_ptr == argv[2])) {
    pmsg_error("(pgerase) cannot parse address %s\n", argv[2]);
    return -1;
  }

  if (addr < 0 || addr >= maxsize) {
    pmsg_error("(pgerase) %s address 0x%05x is out of range [0, 0x%05x]\n", mem->desc, addr, maxsize-1);
    return -1;
  }

  if(pgm->page_erase_cached(pgm, p, mem, (unsigned int) addr) < 0) {
    pmsg_error("(pgerase) unable to erase %s page at 0x%05x\n", mem->desc, addr);
    return -1;
  }

  return 0;
}


static int cmd_part(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  term_out("\n");
  avr_display(stdout, p, "", 0);
  term_out("\n");

  return 0;
}


static int cmd_sig(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int i;
  int rc;
  AVRMEM *m;

  rc = avr_signature(pgm, p);
  if (rc != 0) {
    pmsg_error("(sig) error reading signature data, rc=%d\n", rc);
  }

  m = avr_locate_mem(p, "signature");
  if (m == NULL) {
    pmsg_error("(sig) signature data not defined for device %s\n", p->desc);
  }
  else {
    term_out("Device signature = 0x");
    for (i=0; i<m->size; i++)
      term_out("%02x", m->buf[i]);
    term_out("\n\n");
  }

  return 0;
}


static int cmd_quit(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  /* FUSE bit verify will fail if left in SPI mode */
  if (spi_mode) {
    cmd_pgm(pgm, p, 0, NULL);
  }
  return 1;
}


static int cmd_parms(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  pgm->print_parms(pgm, stdout);
  term_out("\n");
  return 0;
}


static int cmd_vtarg(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int rc;
  double v;
  char *endp;

  if (argc != 2) {
    msg_error("Usage: vtarg <value>\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    pmsg_error("(vtarg) cannot parse voltage %s\n", argv[1]);
    return -1;
  }
  if ((rc = pgm->set_vtarget(pgm, v)) != 0) {
    pmsg_error("(vtarg) unable to set V[target] (rc = %d)\n", rc);
    return -3;
  }
  return 0;
}


static int cmd_fosc(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int rc;
  double v;
  char *endp;

  if (argc != 2) {
    msg_error("Usage: fosc <value>[M|k] | off\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    if (strcmp(argv[1], "off") == 0)
      v = 0.0;
    else {
      pmsg_error("(fosc) cannot parse frequency %s\n", argv[1]);
      return -1;
    }
  }
  if (*endp == 'm' || *endp == 'M')
    v *= 1e6;
  else if (*endp == 'k' || *endp == 'K')
    v *= 1e3;
  if ((rc = pgm->set_fosc(pgm, v)) != 0) {
    pmsg_error("(fosc) unable to set oscillator frequency (rc = %d)\n", rc);
    return -3;
  }
  return 0;
}


static int cmd_sck(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int rc;
  double v;
  char *endp;

  if (argc != 2) {
    msg_error("Usage: sck <value>\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    pmsg_error("(sck) cannot parse period %s\n", argv[1]);
    return -1;
  }
  v *= 1e-6;                    // Convert from microseconds to seconds
  if ((rc = pgm->set_sck_period(pgm, v)) != 0) {
    pmsg_error("(sck) unable to set SCK period (rc = %d)\n", rc);
    return -3;
  }
  return 0;
}


static int cmd_varef(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int rc;
  unsigned int chan;
  double v;
  char *endp;

  if (argc != 2 && argc != 3) {
    msg_error("Usage: varef [channel] <value>\n");
    return -1;
  }
  if (argc == 2) {
    chan = 0;
    v = strtod(argv[1], &endp);
    if (endp == argv[1]) {
      pmsg_error("(varef) cannot parse voltage %s\n", argv[1]);
      return -1;
    }
  } else {
    chan = strtoul(argv[1], &endp, 10);
    if (endp == argv[1]) {
      pmsg_error("(varef) cannot parse channel %s\n", argv[1]);
      return -1;
    }
    v = strtod(argv[2], &endp);
    if (endp == argv[2]) {
      pmsg_error("(varef) cannot parse voltage %s\n", argv[2]);
      return -1;
    }
  }
  if ((rc = pgm->set_varef(pgm, chan, v)) != 0) {
    pmsg_error("(varef) unable to set V[aref] (rc = %d)\n", rc);
    return -3;
  }
  return 0;
}


static int cmd_help(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int i;

  term_out("Valid commands:\n");
  for (i=0; i<NCMDS; i++) {
    if(!*(void (**)(void)) ((char *) pgm + cmd[i].fnoff))
      continue;
    term_out("  %-7s : ", cmd[i].name);
    term_out(cmd[i].desc, cmd[i].name);
    term_out("\n");
  }
  term_out("\n"
    "Note that not all programmer derivatives support all commands. Flash and\n"
    "EEPROM type memories are normally read and written using a cache via paged\n"
    "read and write access; the cache is synchronised on quit or flush commands.\n"
    "The part command displays valid memory types for use with dump and write.\n\n");
  return 0;
}

static int cmd_spi(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  pgm->setpin(pgm, PIN_AVR_RESET, 1);
  spi_mode = 1;
  return 0;
}

static int cmd_pgm(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  pgm->setpin(pgm, PIN_AVR_RESET, 0);
  spi_mode = 0;
  pgm->initialize(pgm, p);
  return 0;
}

static int cmd_verbose(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int nverb;
  char *endp;

  if (argc != 1 && argc != 2) {
    msg_error("Usage: verbose [<value>]\n");
    return -1;
  }
  if (argc == 1) {
    msg_error("Verbosity level: %d\n", verbose);
    return 0;
  }
  nverb = strtol(argv[1], &endp, 0);
  if (endp == argv[1] || *endp) {
    pmsg_error("(verbose) cannot parse verbosity level %s\n", argv[1]);
    return -1;
  }
  if (nverb < 0) {
    pmsg_error("(verbose) level must not be negative: %d\n", nverb);
    return -1;
  }
  verbose = nverb;
  term_out("New verbosity level: %d\n", verbose);

  return 0;
}

static int cmd_quell(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int nquell;
  char *endp;

  if (argc != 1 && argc != 2) {
    msg_error("Usage: quell [<value>]\n");
    return -1;
  }
  if (argc == 1) {
    msg_error("Quell level: %d\n", quell_progress);
    return 0;
  }
  nquell = strtol(argv[1], &endp, 0);
  if (endp == argv[1] || *endp) {
    pmsg_error("(quell) cannot parse quell level %s\n", argv[1]);
    return -1;
  }
  if (nquell < 0) {
    pmsg_error("(quell) level must not be negative: %d\n", nquell);
    return -1;
  }
  quell_progress = nquell;
  term_out("New quell level: %d\n", quell_progress);

  if(quell_progress > 0)
    update_progress = NULL;
  else
    terminal_setup_update_progress();

  return 0;
}

static int tokenize(char *s, char ***argv) {
  int     i, n, l, nargs;
  int     len, slen;
  char  *buf;
  int     bufsize;
  char **bufv;
  char  *bufp;
  char  *q, *r;
  char  *nbuf;
  char **av;

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
      ptrdiff_t k = buf - bufp;
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
    ptrdiff_t offset = bufv[i] - buf;
    av[i] = q + offset;
  }
  av[i] = NULL;

  free(buf);
  free(bufv);

  *argv = av;

  return n;
}


static int do_cmd(PROGRAMMER *pgm, AVRPART *p, int argc, char *argv[]) {
  int i;
  int hold;
  int len;

  len = strlen(argv[0]);
  hold = -1;
  for (i=0; i<NCMDS; i++) {
    if(!*(void (**)(void)) ((char *) pgm + cmd[i].fnoff))
      continue;
    if (len && strcasecmp(argv[0], cmd[i].name) == 0)
      return cmd[i].func(pgm, p, argc, argv);
    if (len && strncasecmp(argv[0], cmd[i].name, len)==0) {
      if (hold != -1) {
        pmsg_error("(cmd) command %s is ambiguous\n", argv[0]);
        return -1;
      }
      hold = i;
    }
  }

  if (hold != -1)
    return cmd[hold].func(pgm, p, argc, argv);

  pmsg_error("(cmd) invalid command %s\n", argv[0]);

  return -1;
}


char *terminal_get_input(const char *prompt) {
  char input[256];

  term_out("%s", prompt);
  if(fgets(input, sizeof(input), stdin)) {
    int len = strlen(input);
    if(len > 0 && input[len-1] == '\n')
      input[len-1] = 0;
    return cfg_strdup(__func__, input);
  }

  return NULL;
}


static int process_line(char *cmdbuf, PROGRAMMER *pgm, struct avrpart *p) {
  int argc, rc;
  char **argv = NULL, *q;

  // Find the start of the command, skipping any white space
  q = cmdbuf;
  while(*q && isspace((unsigned char) *q))
    q++;

  // Skip blank lines and comments
  if (!*q || (*q == '#'))
    return 0;

  // Tokenize command line
  argc = tokenize(q, &argv);

  if(!argv)
    return -1;

  // Run the command
  rc = do_cmd(pgm, p, argc, argv);
  free(argv);

  return rc;
}



#if defined(HAVE_LIBREADLINE)

static PROGRAMMER *term_pgm;
static struct avrpart *term_p;

static int term_running;

// Any character in standard input available (without sleeping)?
static int readytoread() {
#ifdef WIN32
  HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);

  while(1) {
    INPUT_RECORD input[1] = { 0 };
    DWORD dwNumberOfEventsRead = 0;

    if(!PeekConsoleInputA(hStdin, input, ARRAYSIZE(input), &dwNumberOfEventsRead)) {
      DWORD dwError = GetLastError();

      // Stdin redirected from a pipe or file (FIXME: reading from a pipe may sleep)
      if(dwError == ERROR_INVALID_HANDLE)
        return 1;

      pmsg_warning("PeekConsoleInputA() failed with error code %u\n", (unsigned int) dwError);
      return -1;
    }

    if(dwNumberOfEventsRead <= 0) // Nothing in the input buffer
      return 0;

    // Filter out all the events that readline does not handle ...
    if((input[0].EventType & KEY_EVENT) != 0 && input[0].Event.KeyEvent.bKeyDown)
      return 1;

    // Drain other events not handled by readline
    if(!ReadConsoleInputA(hStdin, input, ARRAYSIZE(input), &dwNumberOfEventsRead)) {
      pmsg_warning("ReadConsoleInputA() failed with error code %u\n", (unsigned int) GetLastError());
      return -1;
    }
  }
#else
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);

    return select(1, &fds, NULL, NULL, &tv) > 0;
#endif
}

// Callback processes commands whenever readline() has finished
void term_gotline(char *cmdstr) {
  if(cmdstr) {
    if(*cmdstr) {
      add_history(cmdstr);
      // only quit/abort returns a value > 0
      if(process_line(cmdstr, term_pgm, term_p) > 0)
        term_running = 0;
    }
    free(cmdstr);
    /*
     * This is a workaround for a bug apparently present in the
     * readline compat layer of libedit which is natively present in
     * NetBSD and MacOS.
     *
     * see https://github.com/avrdudes/avrdude/issues/1173
     */
    rl_callback_handler_remove();
    rl_callback_handler_install("avrdude> ", term_gotline);
  } else {
    // call quit at end of file or terminal ^D
    term_out("\n");
    cmd_quit(term_pgm, term_p, 0, NULL);
    term_running = 0;
  }
}


int terminal_mode_interactive(PROGRAMMER *pgm, struct avrpart *p) {
  term_pgm = pgm;               // For callback routine
  term_p = p;

  rl_callback_handler_install("avrdude> ", term_gotline);

  term_running = 1;
  for(int n=1; term_running; n++) {
    if(n%16 == 0) {             // Every 100 ms (16*6.25 us) reset bootloader watchdog timer
      if(pgm->term_keep_alive)
        pgm->term_keep_alive(pgm, NULL);
    }
    usleep(6250);
    if(readytoread() > 0 && term_running)
      rl_callback_read_char();
  }

  rl_callback_handler_remove();

  return pgm->flush_cache(pgm, p);
}

#endif


int terminal_mode_noninteractive(PROGRAMMER *pgm, struct avrpart *p) {
  char *cmdbuf;
  int rc = 0;

  while((cmdbuf = terminal_get_input("avrdude> "))) {
    int rc = process_line(cmdbuf, pgm, p);
    free(cmdbuf);
    if(rc > 0)
      break;
  }

  if(rc <= 0)
    cmd_quit(pgm, p, 0, NULL);
  return pgm->flush_cache(pgm, p);
}

int terminal_mode(PROGRAMMER *pgm, struct avrpart *p) {
#if defined(HAVE_LIBREADLINE)
  // GNU libreadline can also work if input is a pipe.
  // EditLine (NetBSD, MacOS) has issues with that, so only use it when
  // running interactively.
  // EditLine uses version 4.2 (0x0402).
  if (isatty(fileno(stdin)) || (rl_readline_version >= 0x0500))
    return terminal_mode_interactive(pgm, p);
#endif
  return terminal_mode_noninteractive(pgm, p);
}

static void update_progress_tty(int percent, double etime, const char *hdr, int finish) {
  static char *header;
  static int last, done = 1;
  int i;

  setvbuf(stderr, (char *) NULL, _IONBF, 0);

  if(hdr) {
    msg_info("\n");
    last = done = 0;
    if(header)
      free(header);
    header = cfg_strdup("update_progress_tty()",  hdr);
  }

  percent = percent > 100? 100: percent < 0? 0: percent;

  if(!done) {
    if(!header)
      header = cfg_strdup("update_progress_tty()", "report");

    int showperc = finish >= 0? percent: last;

    char hashes[51];
    memset(hashes, finish >= 0? ' ': '-', 50);
    for(i=0; i<showperc; i+=2)
      hashes[i/2] = '#';
    hashes[50] = 0;

    msg_info("\r%s | %s | %d%% %0.2f s ", header, hashes, showperc, etime);
    if(percent == 100) {
      if(finish)
        msg_info("\n\n");
      done = 1;
    }
  }
  last = percent;

  setvbuf(stderr, (char *) NULL, _IOLBF, 0);
}

static void update_progress_no_tty(int percent, double etime, const char *hdr, int finish) {
  static int last, done = 1;

  setvbuf(stderr, (char *) NULL, _IONBF, 0);

  percent = percent > 100? 100: percent < 0? 0: percent;

  if(hdr) {
    msg_info("\n%s | ", hdr);
    last = done = 0;
  }

  if(!done) {
    for(int cnt = percent/2; cnt > last/2; cnt--)
      msg_info(finish >= 0? "#": "-");

    if(percent == 100) {
      msg_info(" | %d%% %0.2fs", finish >= 0? 100: last, etime);
      if(finish)
        msg_info("\n\n");
      done = 1;
    }
  }
  last = percent;

  setvbuf(stderr, (char *) NULL, _IOLBF, 0);
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
