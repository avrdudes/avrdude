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
#include <limits.h>

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

struct command cmd[] = {
  { "dump",  cmd_dump,  "dump memory  : %s <memtype> <addr> <N-Bytes>" },
  { "read",  cmd_dump,  "alias for dump" },
  { "write", cmd_write, "write memory : %s <memtype> <addr> <b1> <b2> ... <bN>" },
  { "erase", cmd_erase, "perform a chip erase" },
  { "sig",   cmd_sig,   "display device signature bytes" },
  { "part",  cmd_part,  "display the current part information" },
  { "send",  cmd_send,  "send a raw command : %s <b1> <b2> <b3> <b4>" },
  { "parms", cmd_parms, "display adjustable parameters (STK500 and Curiosity Nano only)" },
  { "vtarg", cmd_vtarg, "set <V[target]> (STK500 and Curiosity Nano only)" },
  { "varef", cmd_varef, "set <V[aref]> (STK500 only)" },
  { "fosc",  cmd_fosc,  "set <oscillator frequency> (STK500 only)" },
  { "sck",   cmd_sck,   "set <SCK period> (STK500 only)" },
  { "spi",   cmd_spi,   "enter direct SPI mode" },
  { "pgm",   cmd_pgm,   "return to programming mode" },
  { "verbose", cmd_verbose, "change verbosity" },
  { "help",  cmd_help,  "help" },
  { "?",     cmd_help,  "help" },
  { "quit",  cmd_quit,  "quit" }
};

#define NCMDS (sizeof(cmd)/sizeof(struct command))



static int spi_mode = 0;

static int nexttok(char * buf, char ** tok, char ** next)
{
  char * q, * n;

  q = buf;
  while (isspace((int)*q))
    q++;

  /* isolate first token */
  n = q+1;
  while (*n && !isspace((int)*n))
    n++;

  if (*n) {
    *n = 0;
    n++;
  }

  /* find start of next token */
  while (isspace((int)*n))
    n++;

  *tok  = q;
  *next = n;

  return 0;
}


static int hexdump_line(char * buffer, unsigned char * p, int n, int pad)
{
  char * hexdata = "0123456789abcdef";
  char * b = buffer;
  int32_t i = 0;
  int32_t j = 0;

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
  char b [ 128 ];

  for (int32_t i = 0; i < n; i++) {
    memcpy(b, p, n);
    buffer[i] = '.';
    if (isalpha((int)(b[i])) || isdigit((int)(b[i])) || ispunct((int)(b[i])))
      buffer[i] = b[i];
    else if (isspace((int)(b[i])))
      buffer[i] = ' ';
  }

  for (i = n; i < pad; i++)
    buffer[i] = ' ';

  buffer[i] = 0;

  return 0;
}


static int hexdump_buf(FILE * f, int startaddr, unsigned char * buf, int len)
{
  char dst1[80];
  char dst2[80];

  int32_t addr = startaddr;
  unsigned char * p = (unsigned char *)buf;
  while (len) {
    int32_t n = 16;
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
  if (argc < 2) {
  avrdude_message(MSG_INFO, "Usage: %s <memtype> [<start addr> <len>]\n"
                            "       %s <memtype> [<start addr> <...>]\n"
                            "       %s <memtype> <...>\n"
                            "       %s <memtype>\n",
                            argv[0], argv[0], argv[0], argv[0]);
  return -1;
  }

  enum { read_size = 256 };
  static char prevmem[128] = {0x00};
  char * memtype = argv[1];
  AVRMEM * mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    avrdude_message(MSG_INFO, "\"%s\" memory type not defined for part \"%s\"\n",
            memtype, p->desc);
    return -1;
  }
  uint32_t maxsize = mem->size;

  // Get start address if present
  char * end_ptr;
  static uint32_t addr = 0;
  if (argc == 4) {
    addr = strtoul(argv[2], &end_ptr, 0);
    if (*end_ptr || (end_ptr == argv[2])) {
      avrdude_message(MSG_INFO, "%s (%s): can't parse address \"%s\"\n",
              progname, argv[0], argv[2]);
      return -1;
    } else if (addr >= maxsize) {
      avrdude_message(MSG_INFO, "%s (%s): address 0x%05lx is out of range for %s memory\n",
                      progname, argv[0], addr, mem->desc);
      return -1;
    }
  }

  // Get no. bytes to read if present
  static int32_t len = read_size;
  if (argc >= 3) {
    memset(prevmem, 0x00, sizeof(prevmem));
    if (strcmp(argv[argc - 1], "...") == 0) {
      if (argc == 3)
        addr = 0;
      len = maxsize - addr;
    } else if (argc == 4) {
      len = strtol(argv[3], &end_ptr, 0);
      if (*end_ptr || (end_ptr == argv[3])) {
        avrdude_message(MSG_INFO, "%s (%s): can't parse length \"%s\"\n",
              progname, argv[0], argv[3]);
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
    avrdude_message(MSG_INFO, "%s (dump): out of memory\n", progname);
    return -1;
  }

  for (uint32_t i = 0; i < len; i++) {
    int32_t rc = pgm->read_byte(pgm, p, mem, addr + i, &buf[i]);
    if (rc != 0) {
      avrdude_message(MSG_INFO, "error reading %s address 0x%05lx of part %s\n",
              mem->desc, addr + i, p->desc);
      if (rc == -1)
        avrdude_message(MSG_INFO, "read operation not supported on memory type \"%s\"\n",
                mem->desc);
      return -1;
    }
  }

  hexdump_buf(stdout, addr, buf, len);
  fprintf(stdout, "\n");

  free(buf);

  addr = addr + len;

  return 0;
}


static int cmd_write(PROGRAMMER * pgm, struct avrpart * p,
		     int argc, char * argv[])
{
  if (argc < 4) {
    avrdude_message(MSG_INFO,
      "Usage: write <memtype> <start addr> <data1> <data2> <dataN>\n"
      "       write <memtype> <start addr> <no. bytes> <data1> <dataN> <...>\n\n"
      "       Add a suffix to manually specify the size for each field:\n"
      "       HH/hh: 8-bit, H/h/S/s: 16-bit, L/l: 32-bit, LL/ll: 64-bit, F/f: 32-bit float\n");
    return -1;
  }

  int32_t i;
  uint8_t write_mode;       // Operation mode, "standard" or "fill"
  uint8_t start_offset;     // Which argc argument
  int32_t len;              // Number of bytes to write to memory
  char * memtype = argv[1]; // Memory name string
  AVRMEM * mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    avrdude_message(MSG_INFO, "\"%s\" memory type not defined for part \"%s\"\n",
            memtype, p->desc);
    return -1;
  }
  uint32_t maxsize = mem->size;

  char * end_ptr;
  int32_t addr = strtoul(argv[2], &end_ptr, 0);
  if (*end_ptr || (end_ptr == argv[2])) {
    avrdude_message(MSG_INFO, "%s (write): can't parse address \"%s\"\n",
            progname, argv[2]);
    return -1;
  }

  if (addr > maxsize) {
    avrdude_message(MSG_INFO, "%s (write): address 0x%05lx is out of range for %s memory\n",
                    progname, addr, memtype);
    return -1;
  }

  uint8_t * buf = malloc(mem->size + 0x10);
  if (buf == NULL) {
    avrdude_message(MSG_INFO, "%s (write): out of memory\n", progname);
    return -1;
  }

  // Find the first argument to write to flash and how many arguments to parse and write
  if (strcmp(argv[argc - 1], "...") == 0) {
    write_mode = WRITE_MODE_FILL;
    start_offset = 4;
    len = strtoul(argv[3], &end_ptr, 0);
    if (*end_ptr || (end_ptr == argv[3])) {
      avrdude_message(MSG_INFO, "%s (write ...): can't parse address \"%s\"\n",
            progname, argv[3]);
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
    int32_t bytes_grown;
    uint8_t size;
    bool is_float;
    bool is_signed;
    // Data union
    union {
      float f;
      int64_t ll;
      uint8_t a[8];
    };
  } data = {.bytes_grown = 0, .size = 0, .is_float = false, .ll = 0, .is_signed = false};

  for (i = start_offset; i < len + start_offset; i++) {
    data.is_float = false;
    data.size = 0;

    // Handle the next argument
    if (i < argc - start_offset + 3) {
      // Get suffix if present
      char suffix  = argv[i][strlen(argv[i]) - 1];
      char lsuffix = argv[i][strlen(argv[i]) - 2];
      if ((suffix == 'L' && lsuffix == 'L') || (suffix == 'l' && lsuffix == 'l')) {
        argv[i][strlen(argv[i]) - 2] = '\0';
        data.size = 8;
      } else if (suffix == 'L' || suffix == 'l') {
        argv[i][strlen(argv[i]) - 1] = '\0';
        data.size = 4;
      } else if ((suffix == 'F' || suffix == 'f') &&
          strncmp(argv[i], "0x", 2) != 0 && strncmp(argv[i], "-0x", 3) != 0) {
        argv[i][strlen(argv[i]) - 1] = '\0';
        data.size = 4;
      } else if ((suffix == 'H' && lsuffix == 'H') || (suffix == 'h' && lsuffix == 'h')) {
        argv[i][strlen(argv[i]) - 2] = '\0';
        data.size = 1;
      } else if (suffix == 'H' || suffix == 'h' || suffix == 'S' || suffix == 's') {
        argv[i][strlen(argv[i]) - 1] = '\0';
        data.size = 2;
      } else if (suffix == '\'') {
        data.size = 1;
      }

      // Try integers
      data.ll = strtoll(argv[i], &end_ptr, 0);
      if (*end_ptr || (end_ptr == argv[i])) {
        // Try float
        data.f = strtof(argv[i], &end_ptr);
        data.is_float = true;
        if (*end_ptr || (end_ptr == argv[i])) {
          data.is_float = false;
          // Try single character
          if (argv[i][0] == '\'' && argv[i][2] == '\'') {
            data.ll = argv[i][1];
          } else {
            avrdude_message(MSG_INFO, "\n%s (write): can't parse data \"%s\"\n",
                  progname, argv[i]);
            free(buf);
            return -1;
          }
        }
      }

      // Print warning if data size might be ambiguous
      bool is_hex       = (strncmp(argv[i], "0x",  2) == 0);
      bool is_neg_hex   = (strncmp(argv[i], "-0x", 3) == 0);
      bool leading_zero = (strncmp(argv[i], "0x0", 3) == 0);
      int8_t hex_digits = (strlen(argv[i]) - 2);
      if(!data.size                                                                      // No pre-defined size
        && (is_neg_hex                                                                   // Hex with - sign in front
        || (is_hex && leading_zero && (hex_digits & (hex_digits - 1)))                   // Hex with 3, 5, 6 or 7 digits
        || (!is_hex && !data.is_float && llabs(data.ll) > 0xFF && strlen(argv[i]) > 2))) // Base10 int greater than 255
      {
        avrdude_message(MSG_INFO, "Warning: no size suffix specified for \"%s\". "
                                  "Writing %d byte(s)\n",
                                  argv[i],
                                  llabs(data.ll) > UINT32_MAX ? 8 :
                                  llabs(data.ll) > UINT16_MAX || data.is_float ? 4 : \
                                  llabs(data.ll) > UINT8_MAX ? 2 : 1);
      }
      // Flag if signed integer and adjust size
      if (data.ll < 0 && !data.is_float) {
        data.is_signed = true;
        if (data.ll < INT32_MIN)
          data.size = 8;
        else if (data.ll < INT16_MIN)
          data.size = 4;
        else if (data.ll < INT8_MIN)
          data.size = 2;
        else
          data.size = 1;
      }
    }
    buf[i - start_offset + data.bytes_grown]     = data.a[0];
    if (llabs(data.ll) > 0x000000FF || data.size >= 2 || data.is_float)
      buf[i - start_offset + ++data.bytes_grown] = data.a[1];
    if (llabs(data.ll) > 0x0000FFFF || data.size >= 4 || data.is_float) {
      buf[i - start_offset + ++data.bytes_grown] = data.a[2];
      buf[i - start_offset + ++data.bytes_grown] = data.a[3];
    }
    if (llabs(data.ll) > 0xFFFFFFFF || data.size == 8) {
      buf[i - start_offset + ++data.bytes_grown] = data.a[4];
      buf[i - start_offset + ++data.bytes_grown] = data.a[5];
      buf[i - start_offset + ++data.bytes_grown] = data.a[6];
      buf[i - start_offset + ++data.bytes_grown] = data.a[7];
    }
  }

  // When in "fill" mode, the maximum size is already predefined
  if (write_mode == WRITE_MODE_FILL)
    data.bytes_grown = 0;

  if ((addr + len + data.bytes_grown) > maxsize) {
    avrdude_message(MSG_INFO, "%s (write): selected address and # bytes exceed "
                    "range for %s memory\n",
                    progname, memtype);
    return -1;
  }

  pgm->err_led(pgm, OFF);
  bool werror = false;
  for (i = 0; i < (len + data.bytes_grown); i++) {
    int32_t rc = avr_write_byte(pgm, p, mem, addr+i, buf[i]);
    if (rc) {
      avrdude_message(MSG_INFO, "%s (write): error writing 0x%02x at 0x%05lx, rc=%d\n",
              progname, buf[i], addr+i, rc);
      if (rc == -1)
        avrdude_message(MSG_INFO, "write operation not supported on memory type \"%s\"\n",
                        mem->desc);
      werror = true;
    }

    uint8_t b;
    rc = pgm->read_byte(pgm, p, mem, addr+i, &b);
    if (b != buf[i]) {
      avrdude_message(MSG_INFO, "%s (write): error writing 0x%02x at 0x%05lx cell=0x%02x\n",
                      progname, buf[i], addr+i, b);
      werror = true;
    }

    if (werror) {
      pgm->err_led(pgm, ON);
    }
  }

  free(buf);

  fprintf(stdout, "\n");

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
    avrdude_message(MSG_INFO, "The %s programmer does not support direct ISP commands.\n",
                    pgm->type);
    return -1;
  }

  if (spi_mode && (pgm->spi == NULL)) {
    avrdude_message(MSG_INFO, "The %s programmer does not support direct SPI transfers.\n",
                    pgm->type);
    return -1;
  }


  if ((argc > 5) || ((argc < 5) && (!spi_mode))) {
    avrdude_message(MSG_INFO, spi_mode?
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
      avrdude_message(MSG_INFO, "%s (send): can't parse byte \"%s\"\n",
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
  avrdude_message(MSG_INFO, "results:");
  for (i=0; i<len; i++)
    avrdude_message(MSG_INFO, " %02x", res[i]);
  avrdude_message(MSG_INFO, "\n");

  fprintf(stdout, "\n");

  return 0;
}


static int cmd_erase(PROGRAMMER * pgm, struct avrpart * p,
		     int argc, char * argv[])
{
  avrdude_message(MSG_INFO, "%s: erasing chip\n", progname);
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
    avrdude_message(MSG_INFO, "error reading signature data, rc=%d\n",
            rc);
  }

  m = avr_locate_mem(p, "signature");
  if (m == NULL) {
    avrdude_message(MSG_INFO, "signature data not defined for device \"%s\"\n",
                    p->desc);
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
    avrdude_message(MSG_INFO, "%s (parms): the %s programmer does not support "
                    "adjustable parameters\n",
                    progname, pgm->type);
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
    avrdude_message(MSG_INFO, "Usage: vtarg <value>\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    avrdude_message(MSG_INFO, "%s (vtarg): can't parse voltage \"%s\"\n",
            progname, argv[1]);
    return -1;
  }
  if (pgm->set_vtarget == NULL) {
    avrdude_message(MSG_INFO, "%s (vtarg): the %s programmer cannot set V[target]\n",
	    progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_vtarget(pgm, v)) != 0) {
    avrdude_message(MSG_INFO, "%s (vtarg): failed to set V[target] (rc = %d)\n",
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
    avrdude_message(MSG_INFO, "Usage: fosc <value>[M|k] | off\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    if (strcmp(argv[1], "off") == 0)
      v = 0.0;
    else {
      avrdude_message(MSG_INFO, "%s (fosc): can't parse frequency \"%s\"\n",
	      progname, argv[1]);
      return -1;
    }
  }
  if (*endp == 'm' || *endp == 'M')
    v *= 1e6;
  else if (*endp == 'k' || *endp == 'K')
    v *= 1e3;
  if (pgm->set_fosc == NULL) {
    avrdude_message(MSG_INFO, "%s (fosc): the %s programmer cannot set oscillator frequency\n",
                    progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_fosc(pgm, v)) != 0) {
    avrdude_message(MSG_INFO, "%s (fosc): failed to set oscillator frequency (rc = %d)\n",
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
    avrdude_message(MSG_INFO, "Usage: sck <value>\n");
    return -1;
  }
  v = strtod(argv[1], &endp);
  if (endp == argv[1]) {
    avrdude_message(MSG_INFO, "%s (sck): can't parse period \"%s\"\n",
	    progname, argv[1]);
    return -1;
  }
  v *= 1e-6;			/* Convert from microseconds to seconds. */
  if (pgm->set_sck_period == NULL) {
    avrdude_message(MSG_INFO, "%s (sck): the %s programmer cannot set SCK period\n",
                    progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_sck_period(pgm, v)) != 0) {
    avrdude_message(MSG_INFO, "%s (sck): failed to set SCK period (rc = %d)\n",
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
    avrdude_message(MSG_INFO, "Usage: varef [channel] <value>\n");
    return -1;
  }
  if (argc == 2) {
    chan = 0;
    v = strtod(argv[1], &endp);
    if (endp == argv[1]) {
      avrdude_message(MSG_INFO, "%s (varef): can't parse voltage \"%s\"\n",
              progname, argv[1]);
      return -1;
    }
  } else {
    chan = strtoul(argv[1], &endp, 10);
    if (endp == argv[1]) {
      avrdude_message(MSG_INFO, "%s (varef): can't parse channel \"%s\"\n",
              progname, argv[1]);
      return -1;
    }
    v = strtod(argv[2], &endp);
    if (endp == argv[2]) {
      avrdude_message(MSG_INFO, "%s (varef): can't parse voltage \"%s\"\n",
              progname, argv[2]);
      return -1;
    }
  }
  if (pgm->set_varef == NULL) {
    avrdude_message(MSG_INFO, "%s (varef): the %s programmer cannot set V[aref]\n",
	    progname, pgm->type);
    return -2;
  }
  if ((rc = pgm->set_varef(pgm, chan, v)) != 0) {
    avrdude_message(MSG_INFO, "%s (varef): failed to set V[aref] (rc = %d)\n",
	    progname, rc);
    return -3;
  }
  return 0;
}


static int cmd_help(PROGRAMMER * pgm, struct avrpart * p,
		    int argc, char * argv[])
{
  int i;

  fprintf(stdout, "Valid commands:\n\n");
  for (i=0; i<NCMDS; i++) {
    fprintf(stdout, "  %-6s : ", cmd[i].name);
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
  avrdude_message(MSG_INFO, "`spi' command unavailable for this programmer type\n");
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
  avrdude_message(MSG_INFO, "`pgm' command unavailable for this programmer type\n");
  return -1;
}

static int cmd_verbose(PROGRAMMER * pgm, struct avrpart * p,
		       int argc, char * argv[])
{
  int nverb;
  char *endp;

  if (argc != 1 && argc != 2) {
    avrdude_message(MSG_INFO, "Usage: verbose [<value>]\n");
    return -1;
  }
  if (argc == 1) {
    avrdude_message(MSG_INFO, "Verbosity level: %d\n", verbose);
    return 0;
  }
  nverb = strtol(argv[1], &endp, 0);
  if (endp == argv[2]) {
    avrdude_message(MSG_INFO, "%s: can't parse verbosity level \"%s\"\n",
	    progname, argv[2]);
    return -1;
  }
  if (nverb < 0) {
    avrdude_message(MSG_INFO, "%s: verbosity level must be positive: %d\n",
	    progname, nverb);
    return -1;
  }
  verbose = nverb;
  avrdude_message(MSG_INFO, "New verbosity level: %d\n", verbose);

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
        avrdude_message(MSG_INFO, "%s: command \"%s\" is ambiguous\n",
                progname, argv[0]);
        return -1;
      }
      hold = i;
    }
  }

  if (hold != -1)
    return cmd[hold].func(pgm, p, argc, argv);

  avrdude_message(MSG_INFO, "%s: invalid command \"%s\"\n",
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
  int     i;
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
    while (*q && isspace((int)*q))
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

    fprintf(stdout, ">>> ");
    for (i=0; i<argc; i++)
      fprintf(stdout, "%s ", argv[i]);
    fprintf(stdout, "\n");

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


