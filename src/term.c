/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2021-2023 Hans Eirik Bull
 * Copyright (C) 2022-2023 Stefan Rueger <stefan.rueger@urclocks.com>
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

#include "libavrdude.h"
#include "avrintel.h"

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
  int (*func)(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
  size_t fnoff;
  char *desc;
};


static int cmd_dump   (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_write  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_flush  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_abort  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_erase  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_pgerase(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_config (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_sig    (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_part   (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_help   (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_quit   (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_send   (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_parms  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_vtarg  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_varef  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_fosc   (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_sck    (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_spi    (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_pgm    (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_verbose(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);
static int cmd_quell  (const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]);

#define _fo(x) offsetof(PROGRAMMER, x)

struct command cmd[] = {
  { "dump",  cmd_dump,  _fo(read_byte_cached),  "display a memory section as hex dump" },
  { "read",  cmd_dump,  _fo(read_byte_cached),  "alias for dump" },
  { "write", cmd_write, _fo(write_byte_cached), "write data to memory; flash and EEPROM are cached" },
  { "flush", cmd_flush, _fo(flush_cache),       "synchronise flash and EEPROM cache with the device" },
  { "abort", cmd_abort, _fo(reset_cache),       "abort flash and EEPROM writes, ie, reset the r/w cache" },
  { "erase", cmd_erase, _fo(chip_erase_cached), "perform a chip or memory erase" },
  { "pgerase", cmd_pgerase, _fo(page_erase),    "erase one page of flash or EEPROM memory" },
  { "config", cmd_config, _fo(open),            "change or show configuration properties of the part" },
  { "sig",   cmd_sig,   _fo(open),              "display device signature bytes" },
  { "part",  cmd_part,  _fo(open),              "display the current part information" },
  { "send",  cmd_send,  _fo(cmd),               "send a raw command to the programmer" },
  { "parms", cmd_parms, _fo(print_parms),       "display adjustable parameters" },
  { "vtarg", cmd_vtarg, _fo(set_vtarget),       "set the target voltage" },
  { "varef", cmd_varef, _fo(set_varef),         "set the analog reference voltage" },
  { "fosc",  cmd_fosc,  _fo(set_fosc),          "set the oscillator frequency" },
  { "sck",   cmd_sck,   _fo(set_sck_period),    "set the SCK period" },
  { "spi",   cmd_spi,   _fo(setpin),            "enter direct SPI mode" },
  { "pgm",   cmd_pgm,   _fo(setpin),            "return to programming mode" },
  { "verbose", cmd_verbose, _fo(open),          "display or set -v verbosity level" },
  { "quell", cmd_quell, _fo(open),              "display or set -q quell level for progress bars" },
  { "help",  cmd_help,  _fo(open),              "show help message" },
  { "?",     cmd_help,  _fo(open),              "same as help" },
  { "quit",  cmd_quit,  _fo(open),              "synchronise flash/EEPROM cache with device and quit" },
  { "q",     cmd_quit,  _fo(open),              "abbreviation for quit" },
};

#define NCMDS ((int)(sizeof(cmd)/sizeof(struct command)))


static int spi_mode = 0;

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


static int hexdump_buf(const FILE *f, const AVRMEM *m, int startaddr, const unsigned char *buf, int len) {
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


static int cmd_dump(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  static struct mem_addr_len {
    int addr;
    int len;
    const AVRMEM *mem;
  } read_mem[32];
  static int i;
  const char *cmd = tolower(**argv) == 'd'? "dump": "read";

  if ((argc < 2 && read_mem[0].mem == NULL) || argc > 4 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: %s <mem> <addr> <len> # display entire region\n"
      "        %s <mem> <addr>       # start at <addr>\n"
      "        %s <mem>              # Continue displaying memory where left off\n"
      "        %s                    # Continue displaying most recently shown <mem>\n"
      "Function: display memory section as hex dump\n"
      "\n"
      "Both the <addr> and <len> can be negative numbers; a negative <addr> starts\n"
      "an interval from that many bytes below the memory size; a negative <len> ends\n"
      "the interval at that many bytes below the memory size.\n"
      "\n"
      "The latter two versions of the command page through the memory with a page\n"
      "size of the last used effective length (256 bytes default)\n",
      cmd, cmd, cmd, cmd
    );
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
    pmsg_error("(%s) %s memory type not defined for part %s\n", cmd, memtype, p->desc);
    return -1;
  }

  int maxsize = mem->size;
  if(maxsize <= 0) { // Sanity check
    pmsg_error("(%s) cannot read memory %s of size %d\n", cmd, mem->desc, maxsize);
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
    pmsg_error("(%s) read_mem[] under-dimensioned; increase and recompile\n", cmd);
    return -1;
  }

  // Get start address if present
  const char *errptr;
  if(argc >= 3 && !str_eq(argv[2], "...")) {
    int addr = str_int(argv[2], STR_INT32, &errptr);
    if(errptr) {
      pmsg_error("(%s) address %s: %s\n", cmd, argv[2], errptr);
      return -1;
    }

    // Turn negative addr value (counting from top and down) into an actual memory address
    if (addr < 0)
      addr = maxsize + addr;

    if (addr < 0 || addr >= maxsize) {
      int digits = mem->size > 0x10000? 5: 4;
      pmsg_error("(%s) %s address %s is out of range [-0x%0*x, 0x%0*x]\n",
        cmd, mem->desc, argv[2], digits, maxsize, digits, maxsize-1);
      return -1;
    }
    read_mem[i].addr = addr;
  }

  // Get number of bytes to read if present
  if (argc >= 3) {
    if(str_eq(argv[argc - 1], "...")) {
      if (argc == 3)
        read_mem[i].addr = 0;
      read_mem[i].len = maxsize - read_mem[i].addr;
    } else if (argc == 4) {
      int len = str_int(argv[3], STR_INT32, &errptr);
      if(errptr) {
        pmsg_error("(%s) length %s: %s\n", cmd, argv[3], errptr);
        return -1;
      }

      // Turn negative len value (number of bytes from top of memory) into an actual length
      if (len < 0)
        len = maxsize + len + 1 - read_mem[i].addr;

      if (len == 0)
        return 0;
      if (len < 0) {
        pmsg_error("(%s) invalid effective length %d\n", cmd, len);
        return -1;
      }
      read_mem[i].len = len;
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
    pmsg_error("(%s) out of memory\n", cmd);
    return -1;
  }

  if(argc < 4 && verbose)
    term_out(">>> %s %s 0x%x 0x%x\n", cmd, read_mem[i].mem->desc, read_mem[i].addr, read_mem[i].len);

  report_progress(0, 1, "Reading");
  for (int j = 0; j < read_mem[i].len; j++) {
    int addr = (read_mem[i].addr + j) % mem->size;
    int rc = pgm->read_byte_cached(pgm, p, read_mem[i].mem, addr, &buf[j]);
    if (rc != 0) {
      report_progress(1, -1, NULL);
      pmsg_error("(%s) error reading %s address 0x%05lx of part %s\n", cmd, mem->desc, (long) read_mem[i].addr + j, p->desc);
      if (rc == -1)
        imsg_error("%*sread operation not supported on memory type %s\n", 7, "", mem->desc);
      free(buf);
      return -1;
    }
    report_progress(j, read_mem[i].len, NULL);
  }
  report_progress(1, 1, NULL);

  hexdump_buf(stdout, mem, read_mem[i].addr, buf, read_mem[i].len);
  term_out("\v");

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


static int cmd_write(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if (argc < 3 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: write <mem> <addr> <data>[,] {<data>[,]}\n"
      "        write <mem> <addr> <len> <data>[,] {<data>[,]} ... # Fill, see below\n"
      "        write <mem> <data> # Any <data> incl file if memory has only 1 byte\n"
      "        write <mem> <file> # Must be file if memory has more than 1 byte\n"
      "Function: write data to memory; flash and EEPROM are normally cached\n"
      "\n"
      "Ellipsis ... writes <len> bytes padded by repeating the last <data> item.\n"
      "\n"
      "Both the <addr> and <len> can be negative numbers; a negative <addr> starts\n"
      "an interval from that many bytes below the memory size; a negative <len> ends\n"
      "the interval at that many bytes below the memory size.\n"
      "\n"
      "<data> can be binary, octal, decimal or hexadecimal integers, floating point\n"
      "numbers or C-style strings and characters. If nothing matches, <data> will be\n"
      "interpreted as name of a file containing data. In absence of a :<f> format\n"
      "suffix, the terminal will try to auto-detect the file format.\n"
      "\n"
      "For integers, an optional case-insensitive suffix specifies the data size: HH\n"
      "8 bit, H/S 16 bit, L 32 bit, LL 64 bit. Suffix D indicates a 64-bit double, F\n"
      "a 32-bit float, whilst a floating point number without suffix defaults to\n"
      "32-bit float. Hexadecimal floating point notation is supported. An ambiguous\n"
      "trailing suffix, eg, 0x1.8D, is read as no-suffix float where D is part of\n"
      "the mantissa; use a zero exponent 0x1.8p0D to clarify.\n"
      "\n"
      "An optional U suffix makes integers unsigned. Ordinary 0x hex and 0b binary\n"
      "integers are always treated as unsigned. +0x, -0x, +0b and -0b numbers with\n"
      "an explicit sign are treated as signed unless they have a U suffix. Unsigned\n"
      "integers cannot be larger than 2^64-1. If n is an unsigned integer then -n is\n"
      "also a valid unsigned integer as in C. Signed integers must fall into the\n"
      "[-2^63, 2^63-1] range or a correspondingly smaller range when a suffix\n"
      "specifies a smaller type.\n"
      "\n"
      "Ordinary 0x hex and 0b binary integers with n digits (counting leading zeros)\n"
      "use the smallest size of one, two, four and eight bytes that can accommodate\n"
      "any n-digit hex/bin integer. If an integer suffix specifies a size explicitly\n"
      "the corresponding number of least significant bytes are written, and a\n"
      "warning shown if the number does not fit into the desired representation.\n"
      "Otherwise, unsigned integers occupy the smallest of one, two, four or eight\n"
      "bytes needed. Signed numbers are allowed to fit into the smallest signed or\n"
      "smallest unsigned representation: For example, 255 is stored as one byte as\n"
      "255U would fit in one byte, though as a signed number it would not fit into a\n"
      "one-byte interval [-128, 127]. The number -1 is stored in one byte whilst -1U\n"
      "needs eight bytes as it is the same as 0xFFFFffffFFFFffffU.\n"
    );
    return -1;
  }

  int i;
  int write_mode;               // Operation mode, standard or fill
  int start_offset;             // Which argc argument
  int len;                      // Number of bytes to write to memory
  char *memtype = argv[1];      // Memory name string
  AVRMEM *mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    pmsg_error("(write) %s memory type not defined for part %s\n", memtype, p->desc);
    return -1;
  }
  int maxsize = mem->size;

  if (argc == 3 && maxsize > 1) {
    // Check whether argv[2] might be anything other than a file
    Str2data *sd = str_todata(argv[2], STR_ANY & ~STR_FILE, NULL, NULL);
    if(sd && sd->type) {
      if(sd->type & STR_INTEGER && sd->ll >= -maxsize && sd->ll < maxsize)
        pmsg_error("(write) no data specified for %s address %s\n", mem->desc, argv[2]);
      else
        pmsg_error("(write) no address specified for %s data %s\n", mem->desc, argv[2]);
      str_freedata(sd);
      return -1;
    }
    str_freedata(sd);
    // Argv[2] might be a file --- keep it in the running for address 0
  }

  const char *errptr;
  int addr = 0;
  if(argc >= 4) {
    addr = str_int(argv[2], STR_INT32, &errptr);
    if(errptr) {
      pmsg_error("(write) address %s: %s\n", argv[2], errptr);
      return -1;
    }
  }

  // Turn negative addr value (counting from top and down) into an actual memory address
  if (addr < 0)
    addr = maxsize + addr;

  if (addr < 0 || addr >= maxsize) {
    int digits = maxsize > 0x10000? 5: 4;
    pmsg_error("(write) %s address 0x%0*x is out of range [-0x%0*x, 0x%0*x]\n",
      mem->desc, digits, addr, digits, maxsize, digits, maxsize-1);
    return -1;
  }

  // Allocate large enough data and allocation tags space
  size_t bufsz = mem->size + 8 + maxstrlen(argc-3, argv+3)+1;
  if(bufsz > INT_MAX) {
    pmsg_error("(write) too large memory request (%zu)\n", bufsz);
    return -1;
  }
  unsigned char *buf = calloc(bufsz, 1), *tags = calloc(bufsz, 1);
  if(buf == NULL || tags == NULL) {
    pmsg_error("(write) out of memory\n");
    return -1;
  }

  // Find the first argument to write to flash and how many arguments to parse and write
  if(str_eq(argv[argc - 1], "...")) {
    write_mode = WRITE_MODE_FILL;
    start_offset = 4;
    len = str_int(argv[3], STR_INT32, &errptr);
    if(errptr) {
      pmsg_error("(write ...) length %s: %s\n", argv[3], errptr);
      free(buf); free(tags);
      return -1;
    }

    // Turn negative len value (number of bytes from top of memory) into an actual length number
    if (len < 0)
      len = maxsize + len - addr + 1;
    if (len == 0)
      return 0;
    if (len < 0 || len > maxsize - addr) {
      pmsg_error("(write ...) effective %s start address 0x%0*x and effective length %d not compatible with memory size %d\n",
        mem->desc, maxsize > 0x10000? 5: 4, addr, len, maxsize);
      return -1;
    }
  } else {
    write_mode = WRITE_MODE_STANDARD;
    // With no user specified start address, data starts at argv[2]
    // With user specified start address, data starts at a argv[3]
    if(argc == 3)
      start_offset = 2;
    else
      start_offset = 3;
    len = argc - start_offset;
  }

  int bytes_grown = 0, filling = 0, recorded = 0, maxneeded = maxsize-addr;
  Str2data *sd = NULL;

  for (i = start_offset; i < len + start_offset; i++) {
    // Handle the next argument
    if (i < argc - start_offset + 3) {
      str_freedata(sd);
      sd = str_todata(argv[i], STR_ANY, p, mem->desc);
      if(!sd->type || sd->errstr) {
        pmsg_error("(write) data %s: %s\n", argv[i], sd->errstr? sd->errstr: "str_todata");
        free(buf); free(tags);
        str_freedata(sd);
        return -1;
      }
      if(sd->warnstr)
        pmsg_warning("(write) %s\n", sd->warnstr);
      // Always write little endian (assume double and int have same endianess)
      if(is_bigendian() && sd->size > 0 && (sd->type & STR_NUMBER))
        change_endian(sd->a, sd->size);
    } else {
      filling = 1;
      if(!sd)
        break;
    }
    int n = i - start_offset + bytes_grown;
    if(sd->type == STR_STRING && sd->str_ptr) {
      size_t len = strlen(sd->str_ptr);
      for(size_t j = 0; j < len; j++, n++) {
        buf[n] = (uint8_t) sd->str_ptr[j];
        tags[n] = TAG_ALLOCATED;
      }
      buf[n] = 0;               // Terminating nul
      tags[n] = TAG_ALLOCATED;
      bytes_grown += (int) len; // Sic: one less than written
    } else if(sd->type == STR_FILE && sd->mem && sd->size > 0) {
      int end = bufsz - n;      // Available buffer size
      if(sd->size < end)
        end = sd->size;
      for(int j = 0; j < end; j++, n++) {
        if(sd->mem->tags[j]) {
          buf[n] = sd->mem->buf[j];
          tags[n] = TAG_ALLOCATED;
        }
      }
      if(end > 0)               // Should always be true
        bytes_grown += end-1;
    } else if(sd->size > 0 && (sd->type & STR_NUMBER)) {
      for(int k = 0; k < sd->size; k++, n++) {
        buf[n] = sd->a[k];
        tags[n] = TAG_ALLOCATED;
      }
      bytes_grown += sd->size-1;
    } else {                    // Nothing written
      bytes_grown--;            // Sic: stay stagnat as i increases, but break when filling
      if(write_mode == WRITE_MODE_FILL && filling) {
        filling = 0;
        break;
      }
    }
    recorded = i - start_offset + bytes_grown + 1;
    if(recorded >= maxneeded)
      break;
  }
  str_freedata(sd);

  // When in fill mode, the maximum size is already predefined
  if(write_mode == WRITE_MODE_FILL) {
    if(recorded < len) {
      pmsg_warning("(write ...) can only fill %d < %d byte%s as last item has zero bytes\n",
        recorded, len, update_plural(recorded));
      len = recorded;
    }
    bytes_grown = 0;
  } else if(addr + len + bytes_grown > maxsize) {
    bytes_grown = maxsize - addr - len;
    pmsg_warning("(write) clipping data to fit into %s %s memory\n", p->desc, mem->desc);
  }

  pmsg_notice2("(write) writing %d byte%s starting from address 0x%02x",
    len + bytes_grown, update_plural(len + bytes_grown), addr);
  if (write_mode == WRITE_MODE_FILL && filling)
    msg_notice2("; remaining space filled with %s", argv[argc - 2]);
  msg_notice2("\v");

  pgm->err_led(pgm, OFF);
  bool werror = false;
  report_progress(0, 1, avr_has_paged_access(pgm, mem)? "Caching": "Writing");
  for (i = 0; i < len + bytes_grown; i++) {
    report_progress(i, len + bytes_grown, NULL);
    if(!tags[i])
      continue;

    uint8_t b;
    int rc = pgm->write_byte_cached(pgm, p, mem, addr+i, buf[i]);
    if (rc == LIBAVRDUDE_SOFTFAIL) {
      pmsg_warning("(write) programmer write protects %s address 0x%04x\n", mem->desc, addr+i);
    } else if(rc) {
      pmsg_error("(write) error writing 0x%02x at 0x%05x, rc=%d\n", buf[i], addr+i, (int) rc);
      if (rc == -1)
        imsg_error("%*swrite operation not supported on memory type %s\n", 8, "", mem->desc);
      werror = true;
    } else if(pgm->read_byte_cached(pgm, p, mem, addr+i, &b) < 0) {
      imsg_error("%*sreadback from %s failed\n", 8, "", mem->desc);
      werror = true;
    } else {                    // Read back byte b is now set
      int bitmask = avr_mem_bitmask(p, mem, addr+i);
      if((b & bitmask) != (buf[i] & bitmask)) {
        pmsg_error("(write) verification error writing 0x%02x at 0x%05x cell=0x%02x", buf[i], addr+i, b);
        if(bitmask != 0xff)
          msg_error(" using bit mask 0x%02x", bitmask);
        msg_error("\n");
        werror = true;
      }
   }

    if (werror)
      pgm->err_led(pgm, ON);
  }
  report_progress(1, 1, NULL);

  free(buf);

  return 0;
}


static int cmd_flush(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: flush\n"
      "Function: synchronise flash and EEPROM cache with the device\n"
    );
    return -1;
  }

  pgm->flush_cache(pgm, p);
  return 0;
}


static int cmd_abort(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: abort\n"
      "Function: abort flash and EEPROM writes, ie, reset the r/w cache\n"
    );
    return -1;
  }

  pgm->reset_cache(pgm, p);
  return 0;
}


static int cmd_send(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  unsigned char cmd[4], res[4];
  const char *errptr;
  int i;
  int len;

  if(argc > 5 || (argc < 5 && !spi_mode) || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(spi_mode?
      "Syntax: send <byte1> [<byte2> [<byte3> [<byte4>]]]\n":
      "Syntax: send <byte1> <byte2> <byte3> <byte4>\n"
    );
    msg_error(
      "Function: send a raw command to the programmer\n"
    );
    return -1;
  }

  if (spi_mode && (pgm->spi == NULL)) {
    pmsg_error("(send) the %s programmer does not support direct SPI transfers\n", pgm->type);
    return -1;
  }

  /* number of bytes to write at the specified address */
  len = argc - 1;

  /* load command bytes */
  for (i=1; i<argc; i++) {
    cmd[i-1] = str_int(argv[i], STR_UINT8, &errptr);
    if(errptr) {
      pmsg_error("(send) byte %s: %s\n", argv[i], errptr);
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
  term_out("\n");

  return 0;
}


static int cmd_erase(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if (argc > 4 || argc == 3 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: erase <mem> <addr> <len> # Fill section with 0xff values\n"
      "        erase <mem>              # Fill with 0xff values\n"
      "        erase                    # Chip erase (no chache, immediate effect)\n"
      "Function: perform a chip or memory erase; flash or EEPROM erase is cached\n"
    );
    return -1;
  }

  if (argc > 1) {
    char *memtype = argv[1];
    AVRMEM *mem = avr_locate_mem(p, memtype);
    if (mem == NULL) {
      pmsg_error("(erase) %s memory type not defined for part %s\n", argv[1], p->desc);
      return -1;
    }
    char *args[] = {"write", memtype, "", "", "0xff", "...", NULL};
    // erase <mem>
    if (argc == 2) {
      args[2] = "0";
      args[3] = "-1";
    }
    // erase <mem> <addr> <len>
    else {
      args[2] = argv[2];
      args[3] = argv[3];
    }
    return cmd_write(pgm, p, 6, args);
  }

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


static int cmd_pgerase(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc != 3 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: pgerase <mem> <addr>\n"
      "Function: erase one page of flash or EEPROM memory\n"
    );
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

  const char *errptr;
  int addr = str_int(argv[2], STR_INT32, &errptr);
  if(errptr) {
    pmsg_error("(pgerase) address %s: %s\n", argv[2], errptr);
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


// Config command

const int MAX_PAD = 10;         // Align value labels if difference between their lengths is less than this

typedef union {                 // Lock memory can be 1 or 4 bytes
  uint8_t b[4];
  uint32_t i;
} fl_t;

typedef struct {                // Fuses and lock bits
  uint8_t fuses[16];
  uint32_t lock;
  int fread[16], lread;
  int islock;
  uint32_t current;
} Fusel_t;

typedef struct {
  const Configitem_t *t;        // Configuration bitfield table
  const char *memstr;           // Could be "lockbits"
  const char *alt;              // Set when memstr is an alias
  int match;                    // Matched by user request
  int ok, val, initval;         // Has value val been read OK? Initval == -1 if not known
} Cfg_t;

typedef struct {                // Context parameters to be passed to functions
  int verb, allscript, flheaders, allv, vmax, printfactory;
} Cfg_opts_t;

// Cache the contents of the fuse and lock bits memories that a particular Configitem is involved in
static int getfusel(const PROGRAMMER *pgm, const AVRPART *p, Fusel_t *fl, const Cfg_t *cci, const char **errpp) {
  const char *err = NULL;
  char *tofree;
  int islock;

  islock = str_starts(cci->memstr, "lock");
  if((islock && cci->t->memoffset != 0) || (!islock && (cci->t->memoffset < 0 || cci->t->memoffset >= (int) sizeof fl->fuses))) {
    err = cache_string(tofree = str_sprintf("%s's %s has invalid memoffset %d", p->desc, cci->memstr, cci->t->memoffset));
    free(tofree);
    goto back;
  }

  if(islock && fl->lread) {    // Cached lock OK
    fl->current = fl->lock;
    fl->islock = 1;
    goto back;
  }

  if(!islock && fl->fread[cci->t->memoffset])  { // Cached fuse OK
    fl->current = fl->fuses[cci->t->memoffset];
    fl->islock = 0;
    goto back;
  }

  AVRMEM *mem = avr_locate_mem(p, cci->memstr);
  if(!mem) {
    err = cache_string(tofree = str_sprintf("%s memory type not defined for part %s", cci->memstr, p->desc));
    free(tofree);
    goto back;
  }

  if((islock && mem->size != 4 && mem->size != 1) || (!islock && mem->size != 1)) {
    err = cache_string(tofree = str_sprintf("%s's %s memory has unexpected size %d", p->desc, mem->desc, mem->size));
    free(tofree);
    goto back;
  }

  fl_t m = {.i = 0};
  for(int i=0; i<mem->size; i++)
    if(pgm->read_byte(pgm, p, mem, i, m.b+i) < 0) {
      err = cache_string(tofree = str_sprintf("cannot read %s's %s memory", p->desc, mem->desc));
      free(tofree);
      goto back;
    }

  if(islock) {
    fl->lock = m.i;
    fl->lread = 1;
  } else {
    fl->fread[cci->t->memoffset] = 1;
    fl->fuses[cci->t->memoffset] = *m.b;
  }
  fl->islock = islock;
  fl->current = m.i;

back:
  if(err && errpp)
    *errpp = err;
  return err? -1: 0;
}

static int setmatches(const char *str, int n, Cfg_t *cc) {
  int matches = 0;

  if(!*str)
    return 0;

  for(int i=0; i<n; i++) {
    cc[i].match = 0;
    if(!cc[i].ok)
      continue;
    if(str_starts(cc[i].t->name, str) || str_match(str, cc[i].t->name)) {
      cc[i].match = 1;
      matches++;
      if(str_eq(cc[i].t->name, str)) {
        for(int j=0; j<i; j++)
          cc[j].match = 0;
        matches = 1;
        break;
      }
    }
  }

  return matches;
}

static int getvalidx(const char *str, int n, const Valueitem_t *vt) {
  int hold, matches = 0;

  if(!*str)
    return 0;

  for(int i=0; i<n; i++) {
    if(str_starts(vt[i].label, str) || str_match(str, vt[i].label)) {
      hold = i;
      matches++;
      if(str_eq(vt[i].label, str)) {
        matches = 1;
        break;
      }
    }
  }

  return matches == 1? hold: matches == 0? -1: -2;
}

typedef struct {                // Fuse/lock properties of the part
  const char *memstr;
  int mask;
  int value;
} Flock_t;


// Fill in cc record with the actual value of the relevant fuse
static int gatherval(const PROGRAMMER *pgm, const AVRPART *p, Cfg_t *cc, int i,
  Fusel_t *fuselp, Flock_t *fc, int nf) {

  // Load current value of this config item
  const char *errstr = NULL;
  getfusel(pgm, p, fuselp, cc+i, &errstr);
  if(errstr) {
    cc[i].ok = 0;
    if(!str_contains(errstr, "cannot read "))
      pmsg_error("(config) cannot handle %s in %s: %s\n", cc[i].t->name, cc[i].memstr, errstr);
    return -1;
  }
  // Update fuse intell
  for(int fj=0; fj<nf; fj++)
    if(str_eq(cc[i].memstr, fc[fj].memstr))
      fc[fj].value = fuselp->current;

  cc[i].val = (cc->t[i].mask & fuselp->current) >> cc->t[i].lsh;

  return 0;
}

// Comment printed next to symbolic value
static char *valuecomment(const Configitem_t *cti, const Valueitem_t *vp, int value, Cfg_opts_t o) {
  static char buf[512], bin[129];
  unsigned u = value, m = cti->mask >> cti->lsh;
  int lsh = cti->lsh;

  if(!vp && cti->vlist)         // No symbolic value despite symbol list?
    strcpy(buf, "reserved");    // Enter reserved instead of the number
  else if (m > 255)             // 4-byte lock
    sprintf(buf, "0x%08x", value);
  else
    sprintf(buf, "%*d", o.vmax >= 100? 3: o.vmax >= 10? 2: 1, value);

  // Show as binary with leading bitmask zeros if 2 or more bits in bitmask
  if(u < 256 && (m & (m-1)) && (o.allscript || o.verb > 0))
    if(cti->mask != 0xff && (unsigned) cti->mask != 0xffffffff)
      sprintf(buf+strlen(buf), " = 0b%s", str_utoa(u | (1<<(intlog2(m)+1)), bin, 2)+1);

  if(o.allscript || o.verb > 1) // Fuse mask visible: print shift pattern
    sprintf(buf+strlen(buf), " = 0x%02x>>%d", u<<lsh, lsh);

  // Print value comment and/or factory setting
  int prvcom = (vp || !cti->vlist) && o.verb > 1;
  int prfact = value >= 0 && value == cti->initval && (o.allv || o.printfactory);
  if(prvcom || prfact) {
    strcat(buf+strlen(buf), " (");
    if(prvcom)
      strncat(buf+strlen(buf), !cti->vlist? "arbitrary": vp->vcomment, sizeof buf-strlen(buf)-32);
    if(prvcom && prfact)
      strcat(buf+strlen(buf), ", ");
    if(prfact)
      strcat(buf+strlen(buf), "factory");
    strcat(buf+strlen(buf), ")");
  }
  return buf;
}

// How a single property is printed
static void printoneproperty(Cfg_t *cc, int ii, const Valueitem_t *vp, int llen, const char *vstr, Cfg_opts_t o) {
  int value = vp? vp->value: cc[ii].val;
  term_out("%s %s=%-*s # %s\n", vp && cc[ii].val != vp->value? "# conf": "config",
    cc[ii].t->name, llen, vstr, valuecomment(cc[ii].t, vp, value, o));
}

// Prints a list of all possible values (o.allv) or just the one proporty cc[ii]
static void printproperty(Cfg_t *cc, int ii, Cfg_opts_t o) {
  const Valueitem_t *vt = cc[ii].t->vlist, *vp;
  int nv = cc[ii].t->nvalues;
  const char *ccom = cc->t[ii].ccomment, *col = strchr(ccom, ':');
  char buf[32];

  // Scan value list for symbolic label and update it
  vp = NULL;
  const char *vstr = NULL;
  if(vt)
    for(int j=0; j<nv; j++)
      if(vt[j].value == cc[ii].val) {
        vstr = vt[j].label;
        vp = vt+j;
        break;
      }
  if(!vstr) {
    sprintf(buf, (unsigned ) cc[ii].t->mask > 255? "0x%08x": "%d", cc[ii].val);
    vstr = buf;
  }

  int lmin, lmax, llen;
  lmin = lmax = strlen(vstr);

  if(o.verb > 0) {
    const char *vcom = !cc[ii].t->vlist? "arbitrary": vp? vp->vcomment: "";
    // Remove some redundancy in explanations
    int cclen = col && str_ends(vcom, col+1)? (int) (col-ccom-1): (int) strlen(ccom);

    if(o.verb > 1)
      term_out("# Mask 0x%02x of %s: %.*s\n", cc->t[ii].mask, cc[ii].memstr, cclen, ccom);
    else if(*cc[ii].t->ccomment)
      term_out("# %c%.*s\n", toupper(*cc[ii].t->ccomment), cclen-1, cc[ii].t->ccomment+1);
    else
      term_out("# %s\n", cc[ii].t->name);
  }

  int done = 0;
  o.vmax = cc[ii].val;
  if(o.allv && vt) {
    for(int j=0; j<nv; j++) {
      if(vt[j].value > o.vmax)
        o.vmax = vt[j].value;
      llen = strlen(vt[j].label);
      lmin = llen < lmin? llen: lmin;
      lmax = llen > lmax? llen: lmax;
    }
  }
  llen = lmax <= lmin+MAX_PAD? lmax: 1; // Align label width if max and min length are similar

  if(o.allv && vt) {
    for(int j=0; j<nv; j++) {
      printoneproperty(cc, ii, vt+j, llen, vt[j].label, o);
      if(cc[ii].val == vt[j].value)
        done = 1;
    }
  }

  if(done)
    return;

  printoneproperty(cc, ii, vp, llen, vstr, o);
}

// Print the fuse/lock bits header (-f, o.flheaders)
static void printfuse(Cfg_t *cc, int ii, Flock_t *fc, int nf, int printed, Cfg_opts_t o) {
  char buf[512];
  int fj;
  for(fj=0; fj<nf; fj++)
    if(str_eq(cc[ii].memstr, fc[fj].memstr))
      break;
  if(fj == nf) {
    pmsg_error("(config) unexpected failure to find fuse %s\n", cc[ii].memstr);
    return;
  }
  if(printed)
    term_out("\n");
  sprintf(buf, "%s %s", str_starts(fc[fj].memstr, "lock")? "Lock bits": "Fuse", fc[fj].memstr);
  if(cc[ii].alt)
    sprintf(buf+strlen(buf), "/%s", cc[ii].alt);
  sprintf(buf+strlen(buf), " value 0x%02x", fc[fj].value);
  if(cc[ii].initval != -1)
    sprintf(buf+strlen(buf), " (factory 0x%02x)", cc[ii].initval);
  if(fc[fj].mask != 0xff && (unsigned) fc[fj].mask != 0xffffffff)
    sprintf(buf+strlen(buf), " mask 0x%02x", fc[fj].mask);
  for(int n = strlen(buf)+2; n; n--)
    term_out("#");
  term_out("\n# %s\n", buf);
  if(o.flheaders && !o.allscript)
    term_out("#\n");
}

static int cmd_config(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  Cfg_opts_t o = { 0 };
  int help = 0, invalid = 0, itemac=1;

  for(int ai = 0; --argc > 0; ) { // Simple option parsing
    const char *q;
    if(*(q=argv[++ai]) != '-' || !q[1])
      argv[itemac++] = argv[ai];
    else {
      while(*++q) {
        switch(*q) {
        case '?':
        case 'h':
          help++;
          break;
        case 'v':
          o.verb++;
          break;
        case 'a':
          o.allscript++;        // Fall through
        case 'f':
          o.flheaders++;
          break;
        default:
          if(!invalid++)
            pmsg_error("(config) invalid option %c, see usage:\n", *q);
          q = "x";
        }
      }
    }
  }
  argc = itemac;                // (arg,c argv) still valid but options have been removed

  if(o.allscript && argc > 1)
    pmsg_error("(config) -a does not allow any further arguments\n");

  if(argc > 2 || help || invalid || (argc >1 && o.allscript)) {
    msg_error(
      "Syntax: config [<opts>] [<property>[=<value>]] [<opts>]\n"
      "Function: Show or change configuration properties of the part\n"
      "Options:\n"
      "    -f show value of fuse and lock bit memories as well\n"
      "    -a output an initialisation script with all possible assignments\n"
      "    -v increase verbosity, show explanations alongside output\n"
      "    -h show this help message\n"
      "\n"
      "Config alone shows all property names and current settings of the part's\n"
      "hardware configuration in terms of symbolic mnemonics or values. Use\n"
      "\n"
      "avrdude> config <property>\n"
      "\n"
      "to show that of <property>. Wildcards or initial strings are permitted (but\n"
      "not both), in which case all settings of matching properties are displayed.\n"
      "\n"
      "avrdude> config <property>=\n"
      "\n"
      "shows all possible values that <property> can take on with the currently\n"
      "set one being the only that is not commented out. Assignments\n"
      "\n"
      "avrdude> config <property>=<value>\n"
      "\n"
      "modify the corresponding fuses or lock bits immediately but will normally only\n"
      "take effect the next time the part is reset. Value can be a valid integer or\n"
      "one of the symbolic mnemonics, if known. Wildcards or initial strings are\n"
      "permitted for both the property and the mnemonic, but an assignment only\n"
      "happens if both the property and the name can be uniquely resolved.\n"
      "\n"
      "It is quite possible, as is with direct writing to the underlying fuses and\n"
      "lock bits, to brick a part, i.e., make it unresponsive to further programming\n"
      "with the chosen programmer: here be dragons.\n"
    );
    return !help || invalid? -1: 0;
  }

  int idx = -1;                 // Index in uP_table[]
  const Configitem_t *ct;       // Configuration bitfield table
  int nc;                       // Number of config properties, some may not be available
  Fusel_t fusel;                // Copy of fuses and lock bits
  const Valueitem_t *vt;        // Pointer to symbolic labels and associated values
  int nv;                       // Number of symbolic labels
  Cfg_t *cc;                    // Current configuration; cc[] and ct[] are parallel arrays
  Flock_t *fc;                  // Current fuse and lock bits memories
  int nf = 0;                   // Number of involved fuse and lock bits memories

  memset(&fusel, 0, sizeof fusel);

  if(p->mcuid >= 0)
    idx = upidxmcuid(p->mcuid);
  if(idx < 0 && p->desc && *p->desc)
    idx = upidxname(p->desc);
  if(idx < 0) {
    pmsg_error("(config) uP_table neither knows mcuid %d nor part %s\n",
      p->mcuid, p->desc && *p->desc? p->desc: "???");
    return -1;
  }
  nc = uP_table[idx].nconfigs;
  ct = uP_table[idx].cfgtable;
  if(nc <= 0 || !ct) {
    pmsg_error("(config) part %s does not have a configuration table\n", p->desc);
    return -1;
  }

  int ret = 0;
  cc = cfg_malloc(__func__, sizeof *cc*nc);
  fc = cfg_malloc(__func__, sizeof *fc*nc);

  char *locktype = "lock";
  if(!avr_locate_mem(p, "lock") && avr_locate_mem(p, "lockbits"))
    locktype = "lockbits";
  for(int i=0; i<nc; i++) {
    cc[i].t = ct+i;
    const char *mt = str_starts(ct[i].memtype, "lock")? locktype: ct[i].memtype;
    cc[i].memstr = mt;
    AVRMEM *mem = avr_locate_mem(p, mt);
    if(!mem) {
      pmsg_warning("(config) %s unavailable as memory %s is not defined for %s\n", ct[i].name, mt, p->desc);
      continue;
    }
    cc[i].ok = 1;
    cc[i].alt = str_eq(mem->desc, mt)? NULL: mem->desc;
    cc[i].initval = mem->initval;
    if(!nf || !str_eq(fc[nf-1].memstr, mt))
      fc[nf++].memstr = mt;
    if(fc[nf-1].mask & ct[i].mask) { // This should not happen
      pmsg_error("(config) overlapping bit values of %s mask 0x%02x in %s's %s\n", cc[i].t->name, ct[i].mask, p->desc, cc[i].memstr);
      ret = -1;
      goto finished;
    }
    fc[nf-1].mask |= ct[i].mask;
  }

  const char *item = argc < 2? "*": argv[1];

  char *rhs = strchr(item, '=');
  if(rhs)                       // Right-hand side of assignment
    *rhs++ = 0;                 // Terminate lhs

  int nm = setmatches(item, nc, cc);
  if(nm == 0) {
    pmsg_warning("(config) non-matching %s; known config items are:\n", item);
    for(int i=0; i<nc; i++)
      if(cc[i].ok)
        msg_warning(" - %s\n", cc[i].t->name);
    ret = -1;
    goto finished;
  }

  if(!rhs || !*rhs || o.allscript) { // Show (all possible) values
    const char *lastfuse = "not a fuse";
    for(int printed = 0, i = 0; i < nc; i++) {
      if(!cc[i].match || !cc[i].ok)
        continue;
      if(gatherval(pgm, p, cc, i, &fusel, fc, nf) < 0) {
        for(int ii=i+1; ii<nc; ii++)
          if(str_eq(cc[i].memstr, cc[ii].memstr))
            cc[ii].ok = 0;
        continue;
      }
      if(!str_eq(lastfuse, cc[i].memstr)) {
        lastfuse = cc[i].memstr;
        if(o.flheaders)
          printfuse(cc, i, fc, nf, printed, o);
      }
      if(o.allscript)
        term_out("#\n# Mask 0x%02x %s\n", ct[i].mask, ct[i].ccomment);
      else if(printed && (rhs || o.verb > 1))
        term_out("\n");
      o.allv = (rhs && !*rhs) || o.allscript; // Print list of all values
      printproperty(cc, i, o);
      printed = 1;
    }
    goto finished;
  }

  // Non-empty rhs: attempt assignment

  if(nm > 1) {
    pmsg_warning("(config) ambiguous; known %s=... config items are:\n", item);
    for(int i=0; i<nc; i++)
      if(cc[i].match)
        msg_warning(" - %s\n", cc[i].t->name);
    ret = -1;
    goto finished;
  }

  int ci;
  for(ci = 0; ci < nc; ci++)
    if(cc[ci].match)
      break;

  if(ci == nc) {
    pmsg_error("(config) unexpected failure to find match index\n");
    ret = -1;
    goto finished;
  }

  // ci is fixed now: save what we have for sanity check
  Cfg_t safecc = cc[ci];

  nv = ct[ci].nvalues;
  vt = ct[ci].vlist;

  // Assignment can be an integer or symbolic value
  const char *errptr;
  int toassign = str_int(rhs, STR_UINT32, &errptr);
  if(errptr) {                  // Cannot parse as int? Match against symbols, if possible
    if(!vt) {
      pmsg_error("(config) no symbols known: assign an appropriate number\n");
      ret = -1;
      goto finished;
    }
    int vj = getvalidx(rhs, nv, vt);
    if(vj < 0) {
      if(vj == -1)
        pmsg_warning("(config) non-matching %s; known %s symbols are:\n", rhs, cc[ci].t->name);
      else
        pmsg_warning("(config) ambiguous; known %s %s symbols are:\n", cc[ci].t->name, rhs);
      o.vmax = 0;
      o.printfactory = 1;
      int llen, lmin = 9999, lmax = 0;
      for(int j=0; j<nv; j++) {
        if(vj == -1 || (str_starts(vt[j].label, rhs) || str_match(rhs, vt[j].label))) {
          llen = strlen(vt[j].label);
          lmin = llen < lmin? llen: lmin;
          lmax = llen > lmax? llen: lmax;
          o.vmax = vt[j].value > o.vmax? vt[j].value: o.vmax;
        }
      }
      llen = lmax <= lmin+MAX_PAD? lmax: 1; // Align label width if max and min length are similar
      for(int j=0; j<nv; j++)
        if(vj == -1 || (str_starts(vt[j].label, rhs) || str_match(rhs, vt[j].label)))
          msg_warning(" - %s=%-*s # %s\n", cc[ci].t->name, llen, vt[j].label,
            valuecomment(ct+ci, vt+j, vt[j].value, o));
      ret = -1;
      goto finished;
    }
    toassign = vt[vj].value;
  }

  if((toassign<<ct[ci].lsh) & ~ct[ci].mask) {
    pmsg_error("(config) attempt to assign bits in 0x%02x outside mask 0x%02x; max value is 0x%02x; not assigned\n",
      toassign<<ct[ci].lsh, ct[ci].mask, ct[ci].mask>>ct[ci].lsh);
    ret = -1;
    goto finished;
  }

  // Check with safe copies of ct[ci] and cc[ci]
  if(memcmp(&safecc, cc+ci, sizeof *cc)) {
    pmsg_error("(config) unexpected data changes (this should never happen)\n");
    ret = -1;
    goto finished;
  }

  if(vt) {
    int j;
    for(j=0; j<nv; j++) {
      if(vt[j].value == toassign)
        break;
    }
    if(j == nv)
      pmsg_warning("(config) assigning a reserved value (0x%02x) to %s, check data sheet\n", toassign, ct[ci].name);
  }

  // Reload current value of fuse that the property lives on
  const char *errstr = NULL;
  getfusel(pgm, p, &fusel, cc+ci, &errstr);
  if(errstr) {
    if(!str_contains(errstr, "cannot read "))
      pmsg_error("(config) cannot handle %s in %s: %s\n", cc[ci].t->name, cc[ci].memstr, errstr);
    ret = -1;
    goto finished;
  }

  fl_t towrite;
  towrite.i = (fusel.current & ~ct[ci].mask) | (toassign<<ct[ci].lsh);
  AVRMEM *mem = avr_locate_mem(p, cc[ci].memstr);
  if(!mem) {
    pmsg_error("(config) %s memory type not defined for part %s\n", cc[ci].memstr, p->desc);
    ret = -1;
    goto finished;
  }
  if((fusel.islock && mem->size != 4 && mem->size != 1) || (!fusel.islock && mem->size != 1)) {
    pmsg_error("(config) %s's %s memory has unexpected size %d\n", p->desc, mem->desc, mem->size);
    ret = -1;
    goto finished;
  }
  for(int i=0; i<mem->size; i++)
    if(pgm->write_byte(pgm, p, mem, i, towrite.b[i]) < 0) {
      pmsg_error("(config) cannot write to %s's %s memory\n", p->desc, mem->desc);
      ret = -1;
      goto finished;
    }

  const char *av[] = { "confirm", cc[ci].t->name, NULL };
  if(o.verb > 0 && !str_eq(argv[0], "confirm"))
    cmd_config(pgm, p, 2, (char **) av);

finished:
  free(cc);
  free(fc);

  return ret;
}


static int cmd_part(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: part\n"
      "Function: display the current part information\n"
    );
    return -1;
  }

  term_out("\v");
  avr_display(stdout, p, "", 0);
  term_out("\v");

  return 0;
}


static int cmd_sig(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int i;
  int rc;
  AVRMEM *m;

  if(argc > 1) {
    msg_error(
      "Syntax: sig\n"
      "Function: display device signature bytes\n"
    );
    return -1;
  }

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
    term_out("\n");
  }

  return 0;
}


static int cmd_quit(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: quit\n"
      "Function: synchronise flash/EEPROM cache with device and quit\n"
    );
    return -1;
  }

  /* FUSE bit verify will fail if left in SPI mode */
  if (spi_mode) {
    cmd_pgm(pgm, p, 0, NULL);
  }
  return 1;
}


static int cmd_parms(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: parms\n"
      "Function: display adjustable parameters\n"
    );
    return -1;
  }

  pgm->print_parms(pgm, stdout);
  term_out("\v");
  return 0;
}


static int cmd_vtarg(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int rc;
  double v;
  char *endp;

  if(argc != 2 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: vtarg <value>\n"
      "Function: set target voltage\n"
    );
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


static int cmd_fosc(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int rc;
  double v;
  char *endp;

  if(argc != 2 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: fosc <value>[M|k] | off\n"
      "Function: set the oscillator frequency\n"
    );
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    if(str_eq(argv[1], "off"))
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


static int cmd_sck(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int rc;
  double v;
  char *endp;

  if(argc != 2 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: sck <value>\n"
      "Function: set the SCK period\n"
    );
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


static int cmd_varef(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int rc;
  unsigned int chan;
  double v;
  char *endp;

  if (argc < 2 || argc > 3 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: varef [channel] <value>\n"
      "Function: set the analog reference voltage\n"
    );
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
    const char *errptr;
    chan = str_int(argv[1], STR_UINT32, &errptr);
    if(errptr) {
      pmsg_error("(varef) channel %s: %s\n", argv[1], errptr);
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


static int cmd_help(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: help\n"
      "Function: show help message for terminal commands\n"
    );
    return -1;
  }

  term_out("Valid commands:\n");
  for(int i=0; i<NCMDS; i++) {
    if(!*(void (**)(void)) ((char *) pgm + cmd[i].fnoff))
      continue;
    term_out("  %-7s : ", cmd[i].name);
    term_out(cmd[i].desc, cmd[i].name);
    term_out("\n");
  }
  term_out("\n"
    "For more details about a terminal command cmd type cmd -?\n\n"
    "Note that not all programmer derivatives support all commands. Flash and\n"
    "EEPROM type memories are normally read and written using a cache via paged\n"
    "read and write access; the cache is synchronised on quit or flush commands.\n"
    "The part command displays valid memory types for use with dump and write.\n");
  return 0;
}

static int cmd_spi(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: spi\n"
      "Function: enter direct SPI mode\n"
    );
    return -1;
  }

  pgm->setpin(pgm, PIN_AVR_RESET, 1);
  spi_mode = 1;
  return 0;
}

static int cmd_pgm(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  if(argc > 1) {
    msg_error(
      "Syntax: pgm\n"
      "Function: return to programming mode\n"
    );
    return -1;
  }

  pgm->setpin(pgm, PIN_AVR_RESET, 0);
  spi_mode = 0;
  pgm->initialize(pgm, p);
  return 0;
}


static int cmd_verbose(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int nverb;
  const char *errptr;

  if (argc > 2 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: verbose [<value>]\n"
      "Function: display or set -v verbosity level\n"
    );
    return -1;
  }

  if (argc == 1) {
    msg_error("Verbosity level: %d\n", verbose);
    return 0;
  }
  nverb = str_int(argv[1], STR_INT32, &errptr);
  if(errptr) {
    pmsg_error("(verbose) verbosity level %s: %s\n", argv[1], errptr);
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


static int cmd_quell(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int nquell;
  const char *errptr;

  if (argc > 2 || (argc > 1 && str_eq(argv[1], "-?"))) {
    msg_error(
      "Syntax: quell [<value>]\n"
      "Function: display or set -q quell level for progress bars\n"
    );
    return -1;
  }
  if (argc == 1) {
    msg_error("Quell level: %d\n", quell_progress);
    return 0;
  }
  nquell = str_int(argv[1], STR_INT32, &errptr);
  if(errptr) {
    pmsg_error("(quell) quell level %s: %s\n", argv[1], errptr);
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


/*
 * Simplified shell-like tokenising of a command line, which is broken up
 * into an (argc, argv) style pointer array until
 *   - A token ends with a semicolon, which is set to nul
 *   - A token starts with a comment character #
 *   - The end of the string is encountered
 *
 * Tokenisation takes single and double quoted strings into consideration. In
 * the second and third case a pointer to the end-of-string nul is returned
 * signifying all of the command line has been processed. In the first case a
 * pointer to the start of the next token after the semicolon is returned,
 * which can be a pointer to nul if the semicolon was at the end of the
 * command line. On error NULL is returned.
 *
 */
static char *tokenize(char *s, int *argcp, char ***argvp) {
  size_t slen;
  int n, nargs;
  char **argv, *buf, *q, *r;

  // Upper estimate of the number of arguments
  for(nargs=0, q=s; *q; nargs++) {
    while(*q && !isspace((unsigned char) *q))
      q++;
    while(*q && isspace((unsigned char) *q))
      q++;
  }
  slen = q - s;

  // Limit input line to some 186 Megabytes as max nargs is (slen+1)/2
  if(slen > 2*((INT_MAX - 2*sizeof(char *))/(sizeof(char *)+3)))
    return NULL;

  // Allocate once for pointers and contents, so caller only needs to free(argv)
  argv = cfg_malloc(__func__, (nargs+2)*sizeof(char *) + slen + nargs);
  buf  = (char *) (argv+nargs+1);

  for(n=0, r=s; *r; ) {
    q = str_nexttok(r, " \t\n\r\v\f", &r);
    if(*q == '#') {             // Inline comment: ignore rest of line
      r = q+strlen(q);
      break;
    }
    strcpy(buf, q);
    if(*buf && !str_eq(buf, ";")) // Don't record empty arguments
      argv[n++] = buf;
    buf += strlen(q) + 1;
    if(buf[-2] == ';') {        // Command separator
      buf[-2] = 0;
      break;
    }
  }

  *argcp = n;
  *argvp = argv;
  return r;
}


static int do_cmd(const PROGRAMMER *pgm, const AVRPART *p, int argc, char *argv[]) {
  int i;
  int hold, matches;
  size_t len;

  len = strlen(argv[0]);
  matches = 0;
  for (i=0; i<NCMDS; i++) {
    if(!*(void (**)(void)) ((char *) pgm + cmd[i].fnoff))
      continue;
    if(len && strncasecmp(argv[0], cmd[i].name, len)==0) { // Partial initial match
      hold = i;
      matches++;
      if(cmd[i].name[len] == 0) { // Exact match
        matches = 1;
        break;
      }
    }
  }

  if(matches == 1)
    return cmd[hold].func(pgm, p, argc, argv);

  pmsg_error("(cmd) command %s is %s\n", argv[0], matches > 1? "ambiguous": "invalid");
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


static int process_line(char *q, const PROGRAMMER *pgm, const AVRPART *p) {
  int argc, rc = 0;
  char **argv;

  // Find the start of the command, skipping any white space
  while(*q && isspace((unsigned char) *q))
    q++;

  // Skip blank lines and comments
  if (!*q || (*q == '#'))
    return 0;

  // Tokenize command line
  do {
    argc = 0; argv = NULL;
    q = tokenize(q, &argc, &argv);
    if(!q)
      return -1;
    if(argc <= 0 || !argv)
      continue;
    // Run the command
    rc = do_cmd(pgm, p, argc, argv);
    free(argv);
  } while(*q);

  return rc;
}


#if defined(HAVE_LIBREADLINE)

static const PROGRAMMER *term_pgm;
static const AVRPART *term_p;

static int term_running;

// Any character in standard input available (without sleeping)?
static int readytoread() {
#ifdef _MSC_VER
    return rl_input_available();
#elif defined(WIN32)
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
      // Only quit returns a value > 0
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
    if(term_running) {
      rl_callback_handler_remove();
      rl_callback_handler_install("avrdude> ", term_gotline);
    }
  } else {
    // End of file or terminal ^D
    term_out("\v");
    cmd_quit(term_pgm, term_p, 0, NULL);
    term_running = 0;
  }
  if(!term_running)
    rl_callback_handler_remove();
}


int terminal_mode_interactive(const PROGRAMMER *pgm, const AVRPART *p) {
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

  return pgm->flush_cache(pgm, p);
}

#endif


int terminal_mode_noninteractive(const PROGRAMMER *pgm, const AVRPART *p) {
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

int terminal_mode(const PROGRAMMER *pgm, const AVRPART *p) {
#if defined(HAVE_LIBREADLINE)
  // GNU libreadline can also work if input is a pipe.
  // EditLine (NetBSD, MacOS) has issues with that, so only use it when
  // running interactively.
  // EditLine uses version 4.2 (0x0402).
  if (isatty(fileno(stdin)) || rl_readline_version > 0x0500)
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
    msg_info("\v");
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
        msg_info("\v");
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
    msg_info("\v%s | ", hdr);
    last = done = 0;
  }

  if(!done) {
    for(int cnt = percent/2; cnt > last/2; cnt--)
      msg_info(finish >= 0? "#": "-");

    if(percent == 100) {
      msg_info(" | %d%% %0.2fs", finish >= 0? 100: last, etime);
      if(finish)
        msg_info("\v");
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
