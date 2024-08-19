/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004 Brian S. Dean <bsd@bdmicro.com>
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

#include <ac_cfg.h>

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <stdint.h>

#ifdef HAVE_LIBELF

#ifdef HAVE_LIBELF_H
#include <libelf.h>
#elif defined(HAVE_LIBELF_LIBELF_H)
#include <libelf/libelf.h>
#endif

#ifndef EM_AVR32
#define EM_AVR32 0x18ad         // Unofficial
#endif

#ifndef EM_AVR
#define EM_AVR 83               // OpenBSD lacks it
#endif
#endif

#include "avrdude.h"
#include "libavrdude.h"

// Common internal record structure for ihex and srec files
struct ihexsrec {
  unsigned char reclen;
  unsigned int loadofs;
  unsigned char rectyp;
  unsigned char data[256];
  unsigned char cksum;
};

char *fileio_fmtstr(FILEFMT format) {
  switch(format) {
  case FMT_AUTO:
    return "auto-detect";
  case FMT_SREC:
    return "Motorola S-Record";
  case FMT_IHEX:
    return "Intel Hex";
  case FMT_IHXC:
    return "Intel Hex with comments";
  case FMT_RBIN:
    return "raw binary";
  case FMT_ELF:
    return "ELF";
  case FMT_IMM:
    return "in-place immediate";
  case FMT_EEGG:
    return "R byte list";
  case FMT_BIN:
    return "0b-binary byte list";
  case FMT_DEC:
    return "decimal byte list";
  case FMT_HEX:
    return "0x-hexadecimal byte list";
  case FMT_OCT:
    return "octal byte list";
  default:
    return "invalid format";
  }
}

int fileio_fmtchr(FILEFMT format) {
  switch(format) {
  case FMT_AUTO:
    return 'a';
  case FMT_SREC:
    return 's';
  case FMT_IHEX:
    return 'i';
  case FMT_IHXC:
    return 'I';
  case FMT_RBIN:
    return 'r';
  case FMT_ELF:
    return 'e';
  case FMT_IMM:
    return 'm';
  case FMT_EEGG:
    return 'R';
  case FMT_BIN:
    return 'b';
  case FMT_DEC:
    return 'd';
  case FMT_HEX:
    return 'h';
  case FMT_OCT:
    return 'o';
  default:
    return '?';
  }
}

FILEFMT fileio_format(char c) {
  switch(c) {
  case 'a':
    return FMT_AUTO;
  case 's':
    return FMT_SREC;
  case 'i':
    return FMT_IHEX;
  case 'I':
    return FMT_IHXC;
  case 'r':
    return FMT_RBIN;
  case 'e':
    return FMT_ELF;
  case 'm':
    return FMT_IMM;
  case 'R':
    return FMT_EEGG;
  case 'b':
    return FMT_BIN;
  case 'd':
    return FMT_DEC;
  case 'h':
    return FMT_HEX;
  case 'o':
    return FMT_OCT;
  default:
    return FMT_ERROR;
  }
}

// Same as fileio_format(ch) but show error message with originator who and list possible formats
FILEFMT fileio_format_with_errmsg(char ch, const char *who) {
  FILEFMT format = fileio_format(ch);

  if(format == FMT_ERROR) {
    pmsg_error("%sinvalid file format :%c; known formats are\n", who? who: "", ch);
    for(int f, c, i = 0; i < 62; i++) {
      c = i < 10? '0' + i: (i & 1? 'A': 'a') + (i - 10)/2;
      f = fileio_format(c);
      if(f != FMT_ERROR)
        msg_error("  :%c %s\n", c, fileio_fmtstr(f));
    }
  }

  return format;
}

// Multi-memory file flat address space layout (also used by avr-gcc's elf)

enum {
  MULTI_FLASH, MULTI_DATA, MULTI_EEPROM,
  MULTI_FUSES, MULTI_LOCK,
  MULTI_SIGROW, MULTI_USERROW, MULTI_BOOTROW,
  MULTI_N,
};

static const struct {
  unsigned base, size;
  const char *name;
} mulmem[] = {
  {0, 0x800000, "flash"},       // rjmp/call can only address 8 MiB in AVR8 architectures
  {0x800000, 0x10000, "data"},  // IO/SRAM
  {0x810000, 0x10000, "EEPROM"},
  {0x820000, 0x10000, "fuses"},
  {0x830000, 0x10000, "lock"},
  {0x840000, 0x10000, "sigrow"},
  {0x850000, 0x10000, "userrow"},
  {0x860000, 0x10000, "bootrow"},
};

#define ANY_MEM_SIZE (mulmem[MULTI_N-1].base + mulmem[MULTI_N-1].size)

#define MBASE(n) (mulmem[MULTI_ ## n].base)
#define MSIZE(n) (mulmem[MULTI_ ## n].size)
#define MEND(n) (MBASE(n) + MSIZE(n) - 1)

// Memory that holds all possible multi-memories as laid out above
AVRMEM *fileio_any_memory(const char *name) {
  return avr_new_memory(name, ANY_MEM_SIZE);
}

#define boffset(p, basemem) baseoffset((p), avr_locate_ ## basemem(p), # basemem)

static int baseoffset(const AVRPART *p, const AVRMEM *base, const char *memname) {
  if(!base)
    pmsg_error("failed to locate %s memory in %s\n", memname, p->desc);
  return base? base->offset: 0;
}

// Extends where memory is put in flat address space of .elf files
unsigned fileio_mem_offset(const AVRPART *p, const AVRMEM *mem) {
  if(mem->type == 0 && mem->size == (int) ANY_MEM_SIZE)
    return 0;

  unsigned location =
    mem_is_in_flash(mem)? MBASE(FLASH) + mem->offset - boffset(p, flash):
    mem_is_io(mem) || mem_is_sram(mem)? MBASE(DATA) + mem->offset:
    mem_is_eeprom(mem)? MBASE(EEPROM):
    mem_is_in_fuses(mem)? MBASE(FUSES) + mem_fuse_offset(mem): mem_is_lock(mem)? MBASE(LOCK):
    // Classic parts intersperse signature and calibration bytes, this code places them together
    is_classic(p) && mem_is_signature(mem)? MBASE(SIGROW):
    is_classic(p) && mem_is_calibration(mem)? MBASE(SIGROW) + 3:
    is_classic(p) && mem_is_in_sigrow(mem)? MBASE(SIGROW) + 0x10 + mem->offset - boffset(p, sigrow):
    // XMEGA parts have signature separate from prodsig, place prodsig at +0x10 as above
    is_pdi(p) && mem_is_signature(mem)? MBASE(SIGROW):
    is_pdi(p) && mem_is_in_sigrow(mem)? MBASE(SIGROW) + 0x10 + mem->offset - boffset(p, sigrow):
    is_updi(p) && mem_is_in_sigrow(mem)? MBASE(SIGROW) + mem->offset - boffset(p, sigrow):
     mem_is_sib(mem)? MBASE(SIGROW) + 0x1000: // Arbitrary 0x1000 offset in signature section for sib
    mem_is_userrow(mem)? MBASE(USERROW): mem_is_bootrow(mem)? MBASE(BOOTROW): ~0U;

  if(location == ~0U)
    pmsg_error("unable to locate %s's %s in multi-memory address space\n", p->desc, mem->desc);
  else if(location >= ANY_MEM_SIZE || location + mem->size > ANY_MEM_SIZE) {    // Overflow
    pmsg_error("%s's %s location [0x%06x, 0x%06x] outside flat address space [0, 0x%06x]\n",
      p->desc, mem->desc, location, location + mem->size - 1, ANY_MEM_SIZE - 1);
    location = ~0U;
  } else if(location <= MEND(FLASH) && location + mem->size > MEND(FLASH) + 1) {
    pmsg_error("%s's %s location [0x%06x, 0x%06x] straddles flash section boundary 0x%06x\n",
      p->desc, mem->desc, location, location + mem->size - 1, MEND(FLASH) + 1);
    location = ~0U;
  } else if(location > MEND(FLASH) && location/0x10000 != (location + mem->size - 1)/0x10000) {
    pmsg_error("%s's %s memory location [0x%06x, 0x%06x] straddles memory section boundary 0x%02x0000\n",
      p->desc, mem->desc, location, location + mem->size - 1, 1 + location/0x10000);
    location = ~0U;
  }

  return location;
}

static const char *memlabel(const AVRPART *p, const AVRMEM *m, unsigned addr, int n) {
  if(m->size < (int) ANY_MEM_SIZE)      // Ordinary (single) memory
    return addr? NULL: m->desc;

  // Inverse lookup of which memory could have been mapped to this address
  for(LNODEID lm = lfirst(p->mem); lm; lm = lnext(lm))
    if(fileio_mem_offset(p, (m = ldata(lm))) == addr && n == m->size)
      return avr_mem_name(p, m);

  for(LNODEID lm = lfirst(p->mem); lm; lm = lnext(lm))
    if(fileio_mem_offset(p, (m = ldata(lm))) == addr)
      return avr_mem_name(p, m);

  return NULL;
}

// Tells lower level .hex/.srec routines whether to write intros/outros
typedef enum {
  FIRST_SEG = 1,
  LAST_SEG = 2,
} Segorder;

static void print_ihex_extended_addr(int n_64k, FILE *outf) {
  unsigned char hi = (n_64k >> 8);
  unsigned char lo = n_64k;
  unsigned char cksum = -(2 + 0 + 4 + hi + lo);

  fprintf(outf, ":02000004%02X%02X%02X\n", hi, lo, cksum);
}

/*
 * Binary buffer to Intel Hex, see https://en.wikipedia.org/wiki/Intel_HEX
 *
 * Given a buffer and a single segment, segp, an open file 'outf' to which to
 * write Intel Hex formatted data, the desired record size recsize, an
 * AVR32-specific memory offset startaddr and the name of the output file,
 * write a valid Intel Hex file. Where indicates whether this is the first
 * segment to be written to the file or the last segment (or both).
 *
 * Return the maximum memory address within mem->buf that was read from plus
 * one. If an error occurs, return -1.
 */
static int b2ihex(const AVRPART *p, const AVRMEM *mem, const Segment *segp, Segorder where,
  int recsize, int startaddr, const char *outfile_unused, FILE *outf, FILEFMT ffmt) {

  const unsigned char *buf = mem->buf;
  int bufsize = segp->len;
  unsigned int nextaddr;
  int n, hiaddr, n_64k;

  if(recsize < 1 || recsize > 255) {
    pmsg_error("recsize %d must be in [1, 255]\n", recsize);
    return -1;
  }

  nextaddr = (unsigned) (startaddr + segp->addr)%0x10000;
  n_64k = (unsigned) (startaddr + segp->addr)/0x10000;
  hiaddr = segp->addr;
  buf += segp->addr;

  // Give address unless it's the first segment and it would be the default 0
  if(!((where & FIRST_SEG) && n_64k == 0))
    print_ihex_extended_addr(n_64k, outf);

  while(bufsize) {
    n = recsize;
    if(n > bufsize)
      n = bufsize;

    if((nextaddr + n) > 0x10000)
      n = 0x10000 - nextaddr;

    if(n) {
      fprintf(outf, ":%02X%04X00", n, nextaddr);
      unsigned char c, cksum = n + ((nextaddr >> 8) & 0x0ff) + (nextaddr & 0x0ff);

      for(int i = 0; i < n; i++) {
        fprintf(outf, "%02X", buf[i]);
        cksum += buf[i];
      }
      cksum = -cksum;
      fprintf(outf, "%02X", cksum);

      if(ffmt == FMT_IHXC) {    // Print comment with address and ASCII dump
        const char *name = memlabel(p, mem, n_64k*0x10000 + nextaddr, n);

        for(int i = n; i < recsize; i++)
          fprintf(outf, "  ");
        fprintf(outf, " // %05x> ", n_64k*0x10000 + nextaddr);
        for(int i = 0; i < n; i++)
          if(n < 9 && name)
            fprintf(outf, "%s0x%02x", i? " ": "", buf[i]);
          else
            putc((c = buf[i] & 0x7f) < ' ' || c == 0x7f? '.': c, outf);
        if(name) {
          fprintf(outf, " %s", name);
          if((str_eq(name, "sigrow") || str_eq(name, "signature")) && !nextaddr) {
            const char *mculist = str_ccmcunames_signature(buf, PM_ALL);

            if(*mculist)
              fprintf(outf, " (%s)", mculist);
          }
        }
      }
      putc('\n', outf);

      nextaddr += n;
      hiaddr += n;
    }

    if(nextaddr >= 0x10000 && bufsize > n) {
      // Output an extended address record
      n_64k++;
      print_ihex_extended_addr(n_64k, outf);
      nextaddr = 0;
    }

    // Advance to next recsize bytes
    buf += n;
    bufsize -= n;
  }

  // Add the end of record data line if it's the last segment
  if(where & LAST_SEG)
    fprintf(outf, ":00000001FF\n");

  return hiaddr;
}

static int ihex_readrec(struct ihexsrec *ihex, char *rec) {
  int i, j;
  char buf[8];
  int offset, len;
  char *e;
  unsigned char cksum;

  len = strlen(rec);
  offset = 1;
  cksum = 0;

  // Reclen
  if(offset + 2 > len)
    return -1;
  for(i = 0; i < 2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->reclen = strtoul(buf, &e, 16);
  if(e == buf || *e != 0)
    return -1;

  // Load offset
  if(offset + 4 > len)
    return -1;
  for(i = 0; i < 4; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->loadofs = strtoul(buf, &e, 16);
  if(e == buf || *e != 0)
    return -1;

  // Record type
  if(offset + 2 > len)
    return -1;
  for(i = 0; i < 2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->rectyp = strtoul(buf, &e, 16);
  if(e == buf || *e != 0)
    return -1;

  cksum = ihex->reclen + ((ihex->loadofs >> 8) & 0x0ff) + (ihex->loadofs & 0x0ff) + ihex->rectyp;

  // Data
  for(j = 0; j < ihex->reclen; j++) {
    if(offset + 2 > len)
      return -1;
    for(i = 0; i < 2; i++)
      buf[i] = rec[offset++];
    buf[i] = 0;
    ihex->data[j] = strtoul(buf, &e, 16);
    if(e == buf || *e != 0)
      return -1;
    cksum += ihex->data[j];
  }

  // Cksum
  if(offset + 2 > len)
    return -1;
  for(i = 0; i < 2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->cksum = strtoul(buf, &e, 16);
  if(e == buf || *e != 0)
    return -1;

  pmsg_debug("read ihex record type 0x%02x at 0x%04x with %2d bytes and chksum 0x%02x (0x%02x)\n",
    ihex->rectyp, ihex->loadofs, ihex->reclen, ihex->cksum, -cksum & 0xff);

  return -cksum & 0xff;
}

// Extract correct memory from large any memory assuming multi-memory model
static int any2mem(const AVRPART *p, const AVRMEM *mem, const Segment *segp, const AVRMEM *any, unsigned maxsize) {

  // Compute location for multi-memory file input
  unsigned location = maxsize > MEND(FLASH) + 1? fileio_mem_offset(p, mem): 0;

  if(location == ~0U)
    return -1;

  unsigned ret = 0;

  // Copy over memory to right place and return highest written address plus one
  for(unsigned i = segp->addr, end = segp->addr + segp->len; i < end; i++)
    if(any->tags[location + i]) {
      mem->buf[i] = any->buf[location + i];
      mem->tags[i] = any->tags[location + i];
      ret = i + 1;
    }

  return ret;
}

/*
 * Intel Hex to binary buffer
 *
 * Given an open file 'inf' which contains Intel Hex formatted data, parse the
 * file, which potentially contains many AVR memories, and lay it out in a
 * temporary AVR "any memory". This also determines whether inf contains the
 * AVR memory mem to write to. Only the segment within mem->buf, segp, is
 * written to.
 *
 * Return 0 if nothing was written, otherwise the maximum memory address within
 * mem->buf that was written plus one. On error, return -1.
 */
static int ihex2b(const char *infile, FILE *inf, const AVRPART *p, const AVRMEM *mem,
  const Segment *segp, unsigned int fileoffset, FILEFMT ffmt) {

  const char *errstr;
  unsigned int nextaddr, baseaddr, maxaddr;
  int lineno, rc;
  struct ihexsrec ihex;

  lineno = 0;
  baseaddr = 0;
  maxaddr = 0;
  nextaddr = 0;
  rewind(inf);

  AVRMEM *any = fileio_any_memory("any");

  for(char *buffer; (buffer = str_fgets(inf, &errstr)); mmt_free(buffer)) {
    lineno++;
    int len = strlen(buffer);

    if(len > 0 && buffer[len - 1] == '\n')
      buffer[--len] = 0;
    if(len == 0 || buffer[0] != ':')
      continue;
    rc = ihex_readrec(&ihex, buffer);
    if(rc < 0) {
      pmsg_error("invalid record at line %d of %s\n", lineno, infile);
      mmt_free(buffer);
      goto error;
    }
    if(rc != ihex.cksum) {
      if(ffmt == FMT_IHEX) {
        pmsg_error("checksum mismatch at line %d of %s\n", lineno, infile);
        imsg_error("checksum=0x%02x, computed checksum=0x%02x\n", ihex.cksum, rc);
        mmt_free(buffer);
        goto error;
      }
      // Just warn with more permissive format FMT_IHXC
      pmsg_notice("checksum mismatch at line %d of %s\n", lineno, infile);
      imsg_notice("checksum=0x%02x, computed checksum=0x%02x\n", ihex.cksum, rc);
    }

    unsigned below = 0, anysize = any->size;

    switch(ihex.rectyp) {
    case 0:                    // Data record
      if(ihex.loadofs + baseaddr < fileoffset) {
        if(!ovsigck) {
          pmsg_error("address 0x%06x below memory offset 0x%x at line %d of %s;\n",
            ihex.loadofs + baseaddr, fileoffset, lineno, infile);
          imsg_error("use -F to skip this check\n");
          mmt_free(buffer);
          goto error;
        }
        pmsg_warning("address 0x%06x below memory offset 0x%x at line %d of %s: ",
          ihex.loadofs + baseaddr, fileoffset, lineno, infile);
        below = fileoffset - baseaddr - ihex.loadofs;
        if(below < ihex.reclen) {       // Clip record
          ihex.reclen -= below;
          ihex.loadofs += below;
        } else {                // Nothing to write
          ihex.reclen = 0;
        }
        msg_warning("%s record\n", ihex.reclen? "clipping": "ignoring");
      }
      nextaddr = ihex.loadofs + baseaddr - fileoffset;
      if(ihex.reclen && nextaddr + ihex.reclen > anysize) {
        if(!ovsigck) {
          pmsg_error("Intel Hex record [0x%06x, 0x%06x] out of range [0, 0x%06x]\n",
            nextaddr, nextaddr + ihex.reclen - 1, anysize - 1);
          imsg_error("at line %d of %s; use -F to skip this check\n", lineno, infile);
          mmt_free(buffer);
          goto error;
        }
        pmsg_warning("Intel Hex record [0x%06x, 0x%06x] out of range [0, 0x%06x]: ",
          nextaddr, nextaddr + ihex.reclen - 1, anysize - 1);
        if(ihex.reclen && nextaddr + ihex.reclen > anysize) {
          unsigned above = nextaddr + ihex.reclen - anysize;

          ihex.reclen = above < ihex.reclen? ihex.reclen - above: 0;  // Clip or zap
        }
        msg_warning("%s it\n", ihex.reclen? "clipping": "ignoring");
      }
      for(int i = 0; i < ihex.reclen; i++) {
        any->buf[nextaddr + i] = ihex.data[below + i];
        any->tags[nextaddr + i] = TAG_ALLOCATED;
      }
      if(!ovsigck && nextaddr == mulmem[MULTI_SIGROW].base && ihex.reclen >= 3)
        if(!avr_sig_compatible(p->signature, any->buf + nextaddr)) {
          pmsg_error("signature of %s incompatible with file's (%s);\n", p->desc,
            str_ccmcunames_signature(any->buf + nextaddr, PM_ALL));
          imsg_error("use -F to override this check\n");
          mmt_free(buffer);
          goto error;
        }
      if(ihex.reclen && nextaddr + ihex.reclen > maxaddr)
        maxaddr = nextaddr + ihex.reclen;
      break;

    case 1:                    // End of file record
      mmt_free(buffer);
      goto done;

    case 2:                    // Extended segment address record
      baseaddr = (ihex.data[0] << 8 | ihex.data[1]) << 4;
      break;

    case 3:                    // Start segment address record
      // We don't do anything with the start address
      break;

    case 4:                    // Extended linear address record
      baseaddr = (ihex.data[0] << 8 | ihex.data[1]) << 16;
      break;

    case 5:                    // Start linear address record
      // We don't do anything with the start address
      break;

    default:
      pmsg_error("do not know how to deal with rectype=%d " "at line %d of %s\n", ihex.rectyp, lineno, infile);
      mmt_free(buffer);
      goto error;
    }
  }

  if(errstr) {
    pmsg_error("read error in Intel Hex file %s: %s\n", infile, errstr);
    goto error;
  }

  if(maxaddr == 0) {
    pmsg_error("no valid record found in Intel Hex file %s\n", infile);
    goto error;
  }

  pmsg_warning("no end of file record found for Intel Hex file %s\n", infile);

done:
  rc = any2mem(p, mem, segp, any, maxaddr);
  avr_free_mem(any);
  if(!rc)
    pmsg_warning("no %s data found in Intel Hex file %s\n", mem->desc, infile);
  return rc;

error:
  avr_free_mem(any);
  return -1;
}

static unsigned int cksum_srec(const unsigned char *buf, int n, unsigned addr, int addr_width) {
  unsigned char cksum = n + addr_width + 1;

  for(int i = 0; i < addr_width; i++)
    cksum += addr, addr >>= 8;
  for(int i = 0; i < n; i++)
    cksum += buf[i];
  cksum = 0xff - cksum;

  return cksum;
}

// Binary to Motorola S-Record, see https://en.wikipedia.org/wiki/SREC_(file_format)
static int b2srec(const AVRMEM *mem, const Segment *segp, Segorder where,
  int recsize, int startaddr, const char *outfile_unused, FILE *outf) {

  const unsigned char *buf;
  unsigned int nextaddr;
  int n, hiaddr, addr_width;

  buf = mem->buf + segp->addr;
  nextaddr = startaddr + segp->addr;
  hiaddr = 0;

  unsigned highest = startaddr + mem->size - 1;

  // Assume same address width throughout, even across different segments
  char datarec, endrec;

  if(highest <= 0xffffu) {
    addr_width = 2;
    datarec = '1';
    endrec = '9';
  } else if(highest <= 0xffffffu) {
    addr_width = 3;
    datarec = '2';
    endrec = '8';
  } else {
    addr_width = 4;
    datarec = '3';
    endrec = '7';
  }

  if(recsize < 1 || recsize > 255 - 1 - addr_width) {
    pmsg_error("recsize %d must be in [1, %d]\n", recsize, 255 - 1 - addr_width);
    return -1;
  }

  if(where & FIRST_SEG) {       // Write header record
    const char *s = "https://github.com/avrdudes/avrdude";
    unsigned char len = strlen(s);

    fprintf(outf, "S0%02X0000", len + 3);
    for(int i = 0; i < len; i++)
      fprintf(outf, "%02X", s[i]);
    fprintf(outf, "%02X\n", cksum_srec((unsigned char *) s, len, 0, 2));
    cx->reccount = 0;
  }

  for(int bufsize = segp->len; bufsize; bufsize -= n) {
    n = recsize;
    if(n > bufsize)
      n = bufsize;

    fprintf(outf, "S%c%02X%0*X", datarec, n + addr_width + 1, 2*addr_width, nextaddr);
    for(int i = 0; i < n; i++)
      fprintf(outf, "%02X", buf[i]);
    fprintf(outf, "%02X\n", cksum_srec(buf, n, nextaddr, addr_width));

    buf += n;
    nextaddr += n;
    hiaddr += n;
    cx->reccount++;
  }

  // Add S5/6 record count record and S7/8/9 end of data record
  if(where & LAST_SEG) {
    if(cx->reccount >= 0 && cx->reccount <= 0xffffff) {
      int wd = cx->reccount <= 0xffff? 2: 3;

      fprintf(outf, "S%c%02X%0*X%02X\n", '5' + (wd == 3), wd + 1, 2*wd,
        cx->reccount, cksum_srec(NULL, 0, cx->reccount, wd));
    }
    fprintf(outf, "S%c%02X%0*X", endrec, addr_width + 1, 2*addr_width, startaddr);
    fprintf(outf, "%02X\n", cksum_srec(NULL, 0, startaddr, addr_width));
  }

  return hiaddr;
}

static int srec_readrec(struct ihexsrec *srec, char *rec) {
  int i, j;
  char buf[8];
  int offset, len, addr_width;
  char *e;
  unsigned char cksum;
  int rc;

  len = strlen(rec);
  offset = 1;
  cksum = 0;
  addr_width = 2;

  // Record type
  if(offset + 1 > len)
    return -1;
  srec->rectyp = rec[offset++];
  if(srec->rectyp == 0x32 || srec->rectyp == 0x38)
    addr_width = 3;             // S2 or S8-record
  else if(srec->rectyp == 0x33 || srec->rectyp == 0x37)
    addr_width = 4;             // S3 or S7-record

  // Reclen
  if(offset + 2 > len)
    return -1;
  for(i = 0; i < 2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  srec->reclen = strtoul(buf, &e, 16);
  cksum += srec->reclen;
  srec->reclen -= (addr_width + 1);
  if(e == buf || *e != 0)
    return -1;

  // Load offset
  if(offset + addr_width > len)
    return -1;
  for(i = 0; i < addr_width*2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  srec->loadofs = strtoull(buf, &e, 16);
  if(e == buf || *e != 0)
    return -1;

  for(i = addr_width; i > 0; i--)
    cksum += (srec->loadofs >> (i - 1)*8) & 0xff;

  // Data
  for(j = 0; j < srec->reclen; j++) {
    if(offset + 2 > len)
      return -1;
    for(i = 0; i < 2; i++)
      buf[i] = rec[offset++];
    buf[i] = 0;
    srec->data[j] = strtoul(buf, &e, 16);
    if(e == buf || *e != 0)
      return -1;
    cksum += srec->data[j];
  }

  // Cksum
  if(offset + 2 > len)
    return -1;
  for(i = 0; i < 2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  srec->cksum = strtoul(buf, &e, 16);
  if(e == buf || *e != 0)
    return -1;

  rc = 0xff - cksum;
  return rc;
}

// Motorola S-Record to binary
static int srec2b(const char *infile, FILE *inf, const AVRPART *p,
  const AVRMEM *mem, const Segment *segp, unsigned int fileoffset) {

  const char *errstr;
  unsigned int nextaddr, maxaddr;
  struct ihexsrec srec;
  int lineno, rc, hexdigs;
  unsigned int reccount;
  unsigned char datarec;

  lineno = 0;
  maxaddr = 0;
  reccount = 0;
  rewind(inf);

  AVRMEM *any = fileio_any_memory("any");

  for(char *buffer; (buffer = str_fgets(inf, &errstr)); mmt_free(buffer)) {
    lineno++;
    int len = strlen(buffer);

    if(len > 0 && buffer[len - 1] == '\n')
      buffer[--len] = 0;
    if(len == 0 || buffer[0] != 'S')
      continue;
    rc = srec_readrec(&srec, buffer);
    if(rc < 0) {
      pmsg_error("invalid record at line %d of %s\n", lineno, infile);
      mmt_free(buffer);
      goto error;
    }
    if(rc != srec.cksum) {
      pmsg_error("checksum mismatch at line %d of %s\n", lineno, infile);
      imsg_error("checksum=0x%02x, computed checksum=0x%02x\n", srec.cksum, rc);
      mmt_free(buffer);
      goto error;
    }

    datarec = 0;
    hexdigs = 4;
    switch(srec.rectyp) {
    case '0':                  // S0: header record, ignore
      break;

    case '1':                  // S1: 16 bit address data record
      datarec = 1;
      break;

    case '2':                  // S2: 24 bit address data record
      datarec = 1;
      hexdigs = 6;
      break;

    case '3':                  // S3: 32 bit address data record
      datarec = 1;
      hexdigs = 8;
      break;

    case '4':                  // S4: symbol record (LSI extension)
      pmsg_error("not supported record at line %d of %s\n", lineno, infile);
      mmt_free(buffer);
      goto error;

    case '5':                  // S5: count of S1, S2 and S3 records previously tx'd
      if(srec.loadofs != reccount) {
        pmsg_error("count of transmitted data records mismatch at line %d of %s\n", lineno, infile);
        imsg_error("transmitted data records= %d, expected value= %d\n", reccount, srec.loadofs);
        mmt_free(buffer);
        goto error;
      }
      break;

    case '7':                  // S7: end record for 32 bit addresses
    case '8':                  // S8: end record for 24 bit addresses
    case '9':                  // S9: end record for 16 bit addresses
      mmt_free(buffer);
      goto done;

    default:
      pmsg_error("do not know how to deal with rectype S%d at line %d of %s\n",
        srec.rectyp, lineno, infile);
      mmt_free(buffer);
      goto error;
    }

    if(datarec == 1) {
      nextaddr = srec.loadofs;
      unsigned below = 0, anysize = any->size;

      if(nextaddr < fileoffset) {
        if(!ovsigck) {
          pmsg_error("address 0x%0*x below memory offset 0x%x at line %d of %s\n",
            hexdigs, nextaddr, fileoffset, lineno, infile);
          imsg_error("use -F to skip this check\n");
          mmt_free(buffer);
          goto error;
        }
        pmsg_warning("address 0x%0*x below memory offset 0x%x at line %d of %s: ",
          hexdigs, nextaddr, fileoffset, lineno, infile);
        below = fileoffset - nextaddr;
        if(below < srec.reclen) {       // Clip record
          nextaddr += below;
          srec.reclen -= below;
        } else {                // Ignore record
          srec.reclen = 0;
        }
        msg_warning("%s record\n", srec.reclen? "clipping": "ignoring");
      }
      nextaddr -= fileoffset;
      if(srec.reclen && nextaddr + srec.reclen > anysize) {
        if(!ovsigck) {
          pmsg_error("Motorola S-Record [0x%06x, 0x%06x] out of range [0, 0x%06x]\n",
            nextaddr, nextaddr + srec.reclen - 1, anysize - 1);
          imsg_error("at line %d of %s; use -F to skip this check\n", lineno, infile);
          mmt_free(buffer);
          goto error;
        }
        pmsg_warning("Motorola S-Record [0x%06x, 0x%06x] out of range [0, 0x%06x]: ",
          nextaddr, nextaddr + srec.reclen - 1, anysize - 1);
        if(srec.reclen && nextaddr + srec.reclen > anysize) {
          unsigned above = nextaddr + srec.reclen - anysize;

          srec.reclen = above < srec.reclen? srec.reclen - above: 0;  // Clip or zap
        }
        msg_warning("%s it\n", srec.reclen? "clipping": "ignoring");
      }
      for(int i = 0; i < srec.reclen; i++) {
        any->buf[nextaddr + i] = srec.data[below + i];
        any->tags[nextaddr + i] = TAG_ALLOCATED;
      }
      if(!ovsigck && nextaddr == mulmem[MULTI_SIGROW].base && srec.reclen >= 3)
        if(!avr_sig_compatible(p->signature, any->buf + nextaddr)) {
          pmsg_error("signature of %s incompatible with file's (%s);\n", p->desc,
            str_ccmcunames_signature(any->buf + nextaddr, PM_ALL));
          imsg_error("use -F to override this check\n");
          mmt_free(buffer);
          goto error;
        }

      if(srec.reclen && nextaddr + srec.reclen > maxaddr)
        maxaddr = nextaddr + srec.reclen;
      reccount++;
    }
  }

  if(errstr) {
    pmsg_error("read error in Motorola S-Record file %s: %s\n", infile, errstr);
    goto error;
  }

  pmsg_warning("no end of file record found for Motorola S-Records file %s\n", infile);
done:
  rc = any2mem(p, mem, segp, any, maxaddr);
  avr_free_mem(any);
  if(!rc)
    pmsg_warning("no %s data found in Motorola S-Record file %s\n", mem->desc, infile);
  return rc;

error:
  avr_free_mem(any);
  return -1;
}

#ifdef HAVE_LIBELF

/*
 * Determine whether the ELF file section pointed to by `sh' fits completely
 * into the program header segment pointed to by `ph'.
 *
 * Assumes the section has been checked already before to actually contain data
 * (SHF_ALLOC, SHT_PROGBITS, sh_size > 0).
 *
 * Sometimes, program header segments might be larger than the actual file
 * sections.  On VM architectures, this is used to allow mmapping the entire
 * ELF file "as is" (including things like the program header table itself).
 */
static inline int is_section_in_segment(Elf32_Shdr *sh, Elf32_Phdr *ph) {
  if(sh->sh_offset < ph->p_offset)
    return 0;
  if(sh->sh_offset + sh->sh_size > ph->p_offset + ph->p_filesz)
    return 0;
  return 1;
}

static int elf_mem_limits(const AVRMEM *mem, const AVRPART *p,
  unsigned int *lowbound, unsigned int *highbound, unsigned int *fileoff) {
  int rv = 0;

  if(is_awire(p)) {             // AVR32
    if(mem_is_flash(mem)) {
      *lowbound = 0x80000000;
      *highbound = 0xffffffff;
      *fileoff = 0;
    } else {
      rv = -1;
    }
  } else {
    if(mem_is_in_flash(mem)) {
      *lowbound = MBASE(FLASH);
      *highbound = MEND(FLASH); // Max 8 MiB
      *fileoff = 0;
    } else if(mem_is_io(mem) || mem_is_sram(mem)) {     // IO & SRAM in data space
      *lowbound = MBASE(DATA) + mem->offset;
      *highbound = MEND(DATA);
      *fileoff = 0;
    } else if(mem_is_eeprom(mem)) {
      *lowbound = MBASE(EEPROM);
      *highbound = MEND(EEPROM);        // Max 64 KiB
      *fileoff = 0;
    } else if(mem_is_a_fuse(mem) || mem_is_fuses(mem)) {
      *lowbound = MBASE(FUSES);
      *highbound = MEND(FUSES);
      *fileoff = mem_is_a_fuse(mem)? mem_fuse_offset(mem): 0;
    } else if(mem_is_lock(mem)) {       // Lock or lockbits
      *lowbound = MBASE(LOCK);
      *highbound = MEND(LOCK);
      *fileoff = 0;
    } else if(mem_is_signature(mem)) {  // Read only
      *lowbound = MBASE(SIGROW);
      *highbound = MEND(SIGROW);
      *fileoff = 0;
    } else if(mem_is_userrow(mem)) {    // usersig or userrow
      *lowbound = MBASE(USERROW);
      *highbound = MEND(USERROW);
      *fileoff = 0;
    } else if(mem_is_bootrow(mem)) {
      *lowbound = MBASE(BOOTROW);
      *highbound = MEND(BOOTROW);
      *fileoff = 0;
    } else {
      rv = -1;
    }
  }

  return rv;
}

// ELF format to binary (the memory segment to read into is ignored)
static int elf2b(const char *infile, FILE *inf, const AVRMEM *mem,
  const AVRPART *p, const Segment *segp_unused, unsigned int fileoffset_unused) {

  Elf *e;
  int rv = 0, size = 0;
  unsigned int low, high, foff;

  if(elf_mem_limits(mem, p, &low, &high, &foff) != 0) {
    pmsg_error("cannot handle %s memory region from ELF file\n", mem->desc);
    return -1;
  }

  /*
   * The Xmega memory regions for "boot", "application", and "apptable" are
   * actually sub-regions of "flash".  Refine the applicable limits.  This
   * allows to select only the appropriate sections out of an ELF file that
   * contains section data for more than one sub-segment.
   */
  if(is_pdi(p) && mem_is_in_flash(mem) && !mem_is_flash(mem)) {
    AVRMEM *flashmem = avr_locate_flash(p);

    if(flashmem == NULL) {
      pmsg_error("no flash memory region found, cannot compute bounds of %s sub-region\n", mem->desc);
      return -1;
    }
    // The config file offsets are PDI offsets, rebase to 0
    low = mem->offset - flashmem->offset;
    high = low + mem->size - 1;
  }

  if(elf_version(EV_CURRENT) == EV_NONE) {
    pmsg_error("ELF library initialization failed: %s\n", elf_errmsg(-1));
    return -1;
  }
  if((e = elf_begin(fileno(inf), ELF_C_READ, NULL)) == NULL) {
    pmsg_error("cannot open %s as an ELF file: %s\n", infile, elf_errmsg(-1));
    return -1;
  }
  if(elf_kind(e) != ELF_K_ELF) {
    pmsg_error("cannot use %s as an ELF input file\n", infile);
    goto done;
  }

  size_t i, isize;
  const char *id = elf_getident(e, &isize);

  if(id == NULL) {
    pmsg_error("unable to read ident area of %s: %s\n", infile, elf_errmsg(-1));
    goto done;
  }

  const char *endianname;
  unsigned char endianness;

  if(is_awire(p)) {             // AVR32
    endianness = ELFDATA2MSB;
    endianname = "little";
  } else {
    endianness = ELFDATA2LSB;
    endianname = "big";
  }
  if(id[EI_CLASS] != ELFCLASS32 || id[EI_DATA] != endianness) {
    pmsg_error("ELF file %s is not a 32-bit, %s-endian file that was expected\n", infile, endianname);
    goto done;
  }

  Elf32_Ehdr *eh;

  if((eh = elf32_getehdr(e)) == NULL) {
    pmsg_error("unable to read ehdr of %s: %s\n", infile, elf_errmsg(-1));
    goto done;
  }

  if(eh->e_type != ET_EXEC) {
    pmsg_error("ELF file %s is not an executable file\n", infile);
    goto done;
  }

  const char *mname;
  uint16_t machine;

  if(is_awire(p)) {
    machine = EM_AVR32;
    mname = "AVR32";
  } else {
    machine = EM_AVR;
    mname = "AVR";
  }
  if(eh->e_machine != machine) {
    pmsg_error("ELF file %s is not for machine %s\n", infile, mname);
    goto done;
  }
  if(eh->e_phnum == 0xffff /* PN_XNUM */) {
    pmsg_error("ELF file %s uses extended program header numbers which are not expected\n", infile);
    goto done;
  }

  Elf32_Phdr *ph;

  if((ph = elf32_getphdr(e)) == NULL) {
    pmsg_error("unable to read program header table of %s: %s\n", infile, elf_errmsg(-1));
    goto done;
  }

  size_t sndx;

  if(elf_getshdrstrndx(e, &sndx) != 0) {
    pmsg_error("unable to obtain section name string table: %s\n", elf_errmsg(-1));
    sndx = 0;
  }

  /*
   * Walk the program header table, pick up entries that are of type PT_LOAD,
   * and have a non-zero p_filesz.
   */
  for(i = 0; i < eh->e_phnum; i++) {
    if(ph[i].p_type != PT_LOAD || ph[i].p_filesz == 0)
      continue;

    pmsg_debug("considering PT_LOAD program header entry #%d\n", (int) i);
    imsg_debug("p_vaddr 0x%x, p_paddr 0x%x, p_filesz %d\n", ph[i].p_vaddr, ph[i].p_paddr, ph[i].p_filesz);

    Elf_Scn *scn = NULL;

    while((scn = elf_nextscn(e, scn)) != NULL) {
      size_t ndx = elf_ndxscn(scn);
      Elf32_Shdr *sh = elf32_getshdr(scn);

      if(sh == NULL) {
        pmsg_error("unable to read section #%u header: %s\n", (unsigned int) ndx, elf_errmsg(-1));
        rv = -1;
        continue;
      }
      // Only interested in PROGBITS, ALLOC sections
      if((sh->sh_flags & SHF_ALLOC) == 0 || sh->sh_type != SHT_PROGBITS)
        continue;
      // Not interested in empty sections
      if(sh->sh_size == 0)
        continue;
      // Section must belong to this segment
      if(!is_section_in_segment(sh, ph + i))
        continue;

      const char *sname = sndx? elf_strptr(e, sndx, sh->sh_name): "*unknown*";
      unsigned int lma = ph[i].p_paddr + sh->sh_offset - ph[i].p_offset;

      pmsg_debug("found section %s, LMA 0x%x, sh_size %u\n", sname, lma, sh->sh_size);

      if(!(lma >= low && lma + sh->sh_size < high)) {
        pmsg_debug("skipping %s (inappropriate for %s)\n", sname, mem->desc);
        continue;
      }
      /*
       * 1-byte sized memory regions are special: they are used for fuse bits,
       * where multiple regions (in the config file) map to a single, larger
       * region in the ELF file (e.g. "lfuse", "hfuse", and "efuse" all map to
       * ".fuse").  We silently accept a larger ELF file region for these, and
       * extract the actual byte to write from it, using the "foff" offset
       * obtained above.
       */
      if(mem->size != 1 && sh->sh_size > (unsigned) mem->size) {
        pmsg_error("section %s of size %u does not fit into %s of size %d\n",
          sname, sh->sh_size, mem->desc, mem->size);
        rv = -1;
        continue;
      }

      Elf_Data *d = NULL;

      while((d = elf_getdata(scn, d)) != NULL) {
        pmsg_debug("data block: d_buf %p, d_off 0x%x, d_size %ld\n",
          d->d_buf, (unsigned int) d->d_off, (long) d->d_size);
        if(mem->size == 1) {
          if(d->d_off != 0) {
            pmsg_error("unexpected data block at offset != 0\n");
            rv = -1;
          } else if(foff >= d->d_size) {
            pmsg_error("ELF file section does not contain byte at offset %d\n", foff);
            rv = -1;
          } else {
            pmsg_debug("extracting one byte from file offset %d\n", foff);
            mem->buf[0] = ((unsigned char *) d->d_buf)[foff];
            mem->tags[0] = TAG_ALLOCATED;
            size = 1;
          }
        } else {
          int idx = lma - low + d->d_off;
          int end = idx + d->d_size;

          if(idx >= 0 && idx < mem->size && end >= 0 && end <= mem->size && end - idx >= 0) {
            if(end > size)
              size = end;
            pmsg_debug("writing %d bytes to mem offset 0x%x\n", end - idx, idx);
            memcpy(mem->buf + idx, d->d_buf, end - idx);
            memset(mem->tags + idx, TAG_ALLOCATED, end - idx);
          } else {
            pmsg_error("section %s [0x%04x, 0x%04x] does not fit into %s [0, 0x%04x]\n",
              sname, idx, (int) (idx + d->d_size - 1), mem->desc, mem->size - 1);
            rv = -1;
          }
        }
      }
    }
  }
done:
  (void) elf_end(e);
  return rv < 0? rv: size;
}
#endif                          // HAVE_LIBELF

// Read/write binary files and return highest memory addr set + 1
static int fileio_rbin(struct fioparms *fio, const char *filename, FILE *f, const AVRMEM *mem, const Segment *segp) {

  int rc;

  switch(fio->op) {
  case FIO_READ:
    rc = fread(mem->buf + segp->addr, 1, segp->len, f);
    if(rc > 0)
      memset(mem->tags + segp->addr, TAG_ALLOCATED, rc);
    break;
  case FIO_WRITE:
    rc = fwrite(mem->buf + segp->addr, 1, segp->len, f);
    break;
  default:
    pmsg_error("invalid fileio operation=%d\n", fio->op);
    return -1;
  }

  if(rc < 0 || (fio->op == FIO_WRITE && rc < segp->len)) {
    pmsg_ext_error("%s error %s %s: %s; %s %d of the expected %d bytes\n",
      fio->iodesc, fio->dir, filename, strerror(errno), fio->rw, rc, segp->len);
    return -1;
  }

  return segp->addr + rc;
}

static int fileio_imm(struct fioparms *fio, const char *fname, FILE *f_unused,
  const AVRMEM *mem, const Segment *segp) {

  char *tok, *p, *line;
  const char *errstr;
  int n = segp->addr, end = segp->addr + segp->len;

  p = line = mmt_strdup(fname);

  switch(fio->op) {
  case FIO_READ:
    while(*(tok = str_nexttok(p, ", \t\n\r\v\f", &p)) && n < end) {
      int set = str_membuf(tok, STR_ANY, mem->buf + n, end - n, &errstr);

      if(errstr || set < 0) {
        pmsg_error("invalid data %s in immediate mode: %s\n", tok, errstr);
        mmt_free(line);
        return -1;
      }
      memset(mem->tags + n, TAG_ALLOCATED, set);
      n += set;
    }
    break;

  case FIO_WRITE:
    pmsg_error("invalid file format 'immediate' for output\n");
    mmt_free(line);
    return -1;

  default:
    pmsg_error("invalid operation=%d\n", fio->op);
    mmt_free(line);
    return -1;
  }

  mmt_free(line);
  return n;
}

static int fileio_ihex(struct fioparms *fio, const char *filename, FILE *f,
  const AVRPART *p, const AVRMEM *mem, const Segment *segp, FILEFMT ffmt, Segorder where) {

  int rc;

  switch(fio->op) {
  case FIO_WRITE:
    rc = b2ihex(p, mem, segp, where, 32, fio->fileoffset, filename, f, ffmt);
    break;

  case FIO_READ:
    rc = ihex2b(filename, f, p, mem, segp, fio->fileoffset, ffmt);
    break;

  default:
    rc = -1;
    pmsg_error("invalid Intel Hex file I/O operation=%d\n", fio->op);
  }

  return rc < 0? -1: rc;
}

static int fileio_srec(struct fioparms *fio, const char *filename, FILE *f,
  const AVRPART *p, const AVRMEM *mem, const Segment *segp, Segorder where) {

  int rc;

  switch(fio->op) {
  case FIO_WRITE:
    rc = b2srec(mem, segp, where, 32, fio->fileoffset, filename, f);
    break;

  case FIO_READ:
    rc = srec2b(filename, f, p, mem, segp, fio->fileoffset);
    break;

  default:
    rc = -1;
    pmsg_error("invalid Motorola S-Records file I/O operation=%d\n", fio->op);
  }

  return rc < 0? -1: rc;
}

#ifdef HAVE_LIBELF
static int fileio_elf(struct fioparms *fio, const char *filename, FILE *f,
  const AVRMEM *mem, const AVRPART *p, const Segment *segp) {

  int rc;

  switch(fio->op) {
  case FIO_WRITE:
    pmsg_error("write operation not supported for ELF\n");
    return -1;
    break;

  case FIO_READ:
    rc = elf2b(filename, f, mem, p, segp, fio->fileoffset);
    return rc;

  default:
    pmsg_error("invalid ELF file I/O operation=%d\n", fio->op);
    return -1;
    break;
  }
}
#endif

static int b2num(const char *filename, FILE *f, const AVRMEM *mem, const Segment *segp, FILEFMT fmt) {
  const char *prefix;
  int base;

  switch(fmt) {
  case FMT_HEX:
    prefix = "0x";
    base = 16;
    break;

  default:
  case FMT_DEC:
    prefix = "";
    base = 10;
    break;

  case FMT_OCT:
    prefix = "0";
    base = 8;
    break;

  case FMT_BIN:
    prefix = "0b";
    base = 2;
    break;

  case FMT_EEGG:
    prefix = "";
    base = 'r';
    break;
  }

  for(int seen = 0, i = segp->addr; i < segp->addr + segp->len; i++) {
    char cbuf[81];

    if(seen++)
      if(putc(',', f) == EOF)
        goto writeerr;

    unsigned num = mem->buf[i];

    /*
     * For a base of 8 and a value < 8 to convert, don't write the prefix.  The
     * conversion will be indistinguishable from a decimal one then.
     */
    if(prefix[0] != '\0' && !(base == 8 && num < 8)) {
      if(fputs(prefix, f) == EOF)
        goto writeerr;
    }
    str_utoa(num, cbuf, base);
    if(fputs(cbuf, f) == EOF)
      goto writeerr;
  }
  if(putc('\n', f) == EOF)
    goto writeerr;

  return segp->addr + segp->len;

writeerr:
  pmsg_ext_error("unable to write to %s: %s\n", filename, strerror(errno));
  return -1;
}

static int num2b(const char *filename, FILE *f, const AVRMEM *mem, const Segment *segp) {
  const char *geterr = NULL;
  char *line;
  int n = segp->addr, end = segp->addr + segp->len;

  while(n < end && (line = str_fgets(f, &geterr))) {
    char *p = line, *tok;

    while(*p && isspace(*p & 0xff))     // Skip white space, comments and empty lines
      p++;
    if(*p && *p != '#') {
      while(*(tok = str_nexttok(p, ", \t\n\r\v\f", &p)) && n < end) {
        const char *errstr;

        if(*tok == '#')         // Ignore rest of line after #
          break;
        int set = str_membuf(tok, STR_ANY, mem->buf + n, end - n, &errstr);

        if(errstr || set < 0) {
          pmsg_error("invalid data %s in immediate mode: %s\n", tok, errstr);
          mmt_free(line);
          return -1;
        }
        memset(mem->tags + n, TAG_ALLOCATED, set);
        n += set;
      }
    }
    mmt_free(line);
  }
  if(geterr) {
    pmsg_error("fgets() errror: %s\n", geterr);
    n = -1;
  }

  return n;
}

static int fileio_num(struct fioparms *fio, const char *filename, FILE *f,
  const AVRMEM *mem, const Segment *segp, FILEFMT fmt) {

  switch(fio->op) {
  case FIO_WRITE:
    return b2num(filename, f, mem, segp, fmt);

  case FIO_READ:
    return num2b(filename, f, mem, segp);

  default:
    pmsg_error("invalid operation=%d\n", fio->op);
    return -1;
  }
}

static int fileio_setparms(int op, struct fioparms *fp, const AVRPART *p, const AVRMEM *m) {
  fp->op = op;

  switch(op) {
  case FIO_READ:
    fp->mode = "r";
    fp->iodesc = "input";
    fp->dir = "from";
    fp->rw = "read";
    break;

  case FIO_WRITE:
    fp->mode = "w";
    fp->iodesc = "output";
    fp->dir = "to";
    fp->rw = "wrote";
    break;

  default:
    pmsg_error("invalid I/O operation %d\n", op);
    return -1;
    break;
  }

  /*
   * AVR32 devices maintain their load offset within the file itself, but
   * AVRDUDE maintains all memory images 0-based.
   */
  fp->fileoffset = is_awire(p)? m->offset: 0;

  return 0;
}

FILE *fileio_fopenr(const char *fname) {

#if !defined(WIN32)
  return fopen(fname, "r");
#else
  return fopen(fname, "rb");
#endif
}

static FILEFMT couldbe(int first, unsigned char *line) {
  int found;
  unsigned long i, nxdigs, len;

  // Check for ELF file
  if(first && line[0] == 0177 && str_starts((char *) line + 1, "ELF"))
    return FMT_ELF;

  len = strlen((char *) line);
  while(len > 0 && line[len - 1] && isspace(line[len - 1])) // Cr/lf etc
    line[--len] = 0;

  // Check for binary data
  for(i = 0; i < len; i++)
    if(line[i] > 127)
      return FMT_RBIN;

  // Check for lines that look like Intel HEX
  if(line[0] == ':' && len >= 11 && isxdigit(line[1]) && isxdigit(line[2])) {
    nxdigs = sscanf((char *) line + 1, "%2lx", &nxdigs) == 1? 2*nxdigs + 8: len;
    for(found = 3 + nxdigs <= len, i = 0; found && i < nxdigs; i++)
      if(!isxdigit(line[3 + i]))
        found = 0;
    if(found)
      return FMT_IHEX;
  }

  // Check for lines that look like Motorola S-record
  if(line[0] == 'S' && len >= 10 && isdigit(line[1]) && isxdigit(line[2]) && isxdigit(line[3])) {
    nxdigs = sscanf((char *) line + 2, "%2lx", &nxdigs) == 1? 2*nxdigs: len;
    for(found = 4 + nxdigs <= len, i = 0; found && i < nxdigs; i++)
      if(!isxdigit(line[4 + i]))
        found = 0;
    if(found)
      return FMT_SREC;
  }

  // Check for terminal-type data entries
  char *p = (char *) line, *tok;
  int failed = 0, idx[4] = { 0, 0, 0, 0 };
  while(*p && isspace(*p & 0xff))
    p++;
  if(*p && *p != '#') {
    while(!failed && *(tok = str_nexttok(p, ", \t\n\r\v\f", &p))) {
      const char *errstr;
      unsigned char mem[8];

      if(*tok == '#')
        break;
      int set = str_membuf(tok, STR_ANY, mem, sizeof mem, &errstr);

      if(errstr || set < 0)
        failed++;
      else if(set > 0)
        idx[str_casestarts(tok, "0x")? 0: str_casestarts(tok, "0b")? 1:
          *tok == '0' && tok[1] && strchr("01234567", tok[1])? 2: 3]++;
    }
    if(!failed && idx[0] + idx[1] + idx[2] + idx[3]) {
      // Doesn't matter which one: they all parse numbers universally
      int i0 = idx[0] >= idx[1]? 0: 1;
      int i2 = idx[2] > idx[3]? 2: 3;
      const int fmts[4] = { FMT_HEX, FMT_BIN, FMT_OCT, FMT_DEC };

      return fmts[idx[i0] >= idx[i2]? i0: i2];
    }
  }

  return FMT_ERROR;
}

int fileio_fmt_autodetect_fp(FILE *f) {
  const char *err = NULL;
  int ret = FMT_ERROR;

  if(f) {
    unsigned char *buf;

    for(int first = 1; ret == FMT_ERROR && (buf = (unsigned char *) str_fgets(f, &err)); first = 0) {
      ret = couldbe(first, buf);
      mmt_free(buf);
    }
    if(err)
      pmsg_error("fgets() error: %s\n", err);
  }

  return ret;
}

int fileio_fmt_autodetect(const char *fname) {
  FILE *f = fileio_fopenr(fname);

  if(f == NULL) {
    pmsg_ext_error("unable to open %s: %s\n", fname, strerror(errno));
    return -1;
  }

  int format = fileio_fmt_autodetect_fp(f);

  fclose(f);

  return format;
}

int fileio_mem(int op, const char *filename, FILEFMT format, const AVRPART *p, const AVRMEM *mem, int size) {

  if(size < 0 || op == FIO_READ || op == FIO_READ_FOR_VERIFY)
    size = mem->size;

  const Segment seg = { 0, size };
  return fileio_segments(op, filename, format, p, mem, 1, &seg);
}

int fileio(int op, const char *filename, FILEFMT format, const AVRPART *p, const char *memstr, int size) {

  AVRMEM *mem = avr_locate_mem(p, memstr);

  if(mem == NULL) {
    pmsg_error("memory %s not configured for device %s\n", memstr, p->desc);
    return -1;
  }

  return fileio_mem(op, filename, format, p, mem, size);
}

// Normalise segment address and length to be non-negative
int segment_normalise(const AVRMEM *mem, Segment *segp) {
  int addr = segp->addr, len = segp->len, maxsize = mem->size;
  int digits = maxsize > 0x10000? 5: 4;

  if(addr < 0)
    addr = maxsize + addr;

  if(addr < 0 || addr >= maxsize) {
    pmsg_error("%s address 0x%0*x is out of range [-0x%0*x, 0x%0*x]\n",
      mem->desc, digits, segp->addr, digits, maxsize, digits, maxsize - 1);
    return -1;
  }

  if(len < 0)
    len = maxsize + len - addr + 1;

  if(len < 0 || len > maxsize) {
    pmsg_error("invalid segment length %d for %s address 0x%0*x\n", segp->len, mem->desc, digits, addr);
    return -1;
  }

  segp->addr = addr;
  segp->len = len;

  return 0;
}

static int fileio_segments_normalise(int oprwv, const char *filename, FILEFMT format,
  const AVRPART *p, const AVRMEM *mem, int n, Segment *seglist) {

  int op, rc;
  FILE *f;
  const char *fname;
  struct fioparms fio;
  int using_stdio;

  op = oprwv == FIO_READ_FOR_VERIFY? FIO_READ: oprwv;
  rc = fileio_setparms(op, &fio, p, mem);
  if(rc < 0)
    return -1;

  for(int i = 0; i < n; i++)
    if(segment_normalise(mem, seglist + i) < 0)
      return -1;

  using_stdio = 0;
  fname = filename;
  f = NULL;
  if(str_eq(filename, "-")) {
    using_stdio = 1;
    fname = fio.op == FIO_READ? "<stdin>": "<stdout>";
    f = fio.op == FIO_READ? stdin: stdout;
  }

  if(format == FMT_AUTO) {
    int format_detect;

    if(using_stdio) {
      pmsg_error("cannot auto detect file format when using stdin/out;\n");
      imsg_error("please specify a file format and try again\n");
      return -1;
    }

    format_detect = fileio_fmt_autodetect(fname);
    if(format_detect < 0) {
      pmsg_error("cannot determine file format for %s, specify explicitly\n", fname);
      return -1;
    }
    format = format_detect;

    if(quell_progress < 2)
      pmsg_notice("%s file %s auto detected as %s\n", fio.iodesc, fname, fileio_fmtstr(format));
  }

#if defined(WIN32)
  // Open Raw Binary and ELF format in binary mode on Windows
  if(format == FMT_RBIN || format == FMT_ELF) {
    if(fio.op == FIO_READ) {
      fio.mode = "rb";
    }
    if(fio.op == FIO_WRITE) {
      fio.mode = "wb";
    }
  }
#endif

  if(format != FMT_IMM) {
    if(!using_stdio) {
      f = fopen(fname, fio.mode);
      if(f == NULL) {
        pmsg_ext_error("cannot open %s file %s: %s\n", fio.iodesc, fname, strerror(errno));
        return -1;
      }
    }
  }

  rc = 0;
  for(int i = 0; i < n; i++) {
    int addr = seglist[i].addr, len = seglist[i].len;

    if(len == 0 && fio.op != FIO_WRITE)
      continue;

    if(fio.op == FIO_READ)      // Fill unspecified memory in segment
      memset(mem->buf + addr, 0xff, len);
    memset(mem->tags + addr, 0, len);

    Segorder where = i == 0? FIRST_SEG: 0;

    if(i + 1 == n)
      where |= LAST_SEG;

    int thisrc = 0;

    switch(format) {
    case FMT_IHEX:
    case FMT_IHXC:
      thisrc = fileio_ihex(&fio, fname, f, p, mem, seglist + i, format, where);
      break;

    case FMT_SREC:
      thisrc = fileio_srec(&fio, fname, f, p, mem, seglist + i, where);
      break;

    case FMT_RBIN:
      thisrc = fileio_rbin(&fio, fname, f, mem, seglist + i);
      break;

    case FMT_ELF:

#ifdef HAVE_LIBELF
      thisrc = fileio_elf(&fio, fname, f, mem, p, seglist + i);
      break;
#else
      pmsg_error("cannot handle ELF file %s, ELF file support was not compiled in\n", fname);
      return -1;
#endif

    case FMT_IMM:
      thisrc = fileio_imm(&fio, fname, f, mem, seglist + i);
      break;

    case FMT_HEX:
    case FMT_DEC:
    case FMT_OCT:
    case FMT_BIN:
    case FMT_EEGG:
      thisrc = fileio_num(&fio, fname, f, mem, seglist + i, format);
      break;

    default:
      pmsg_error("invalid %s file format: %d\n", fio.iodesc, format);
      return -1;
    }
    if(thisrc < 0)
      return thisrc;
    if(thisrc > rc)
      rc = thisrc;
  }

  // On reading flash other than for verify set the size to location of highest non-0xff byte
  if(rc > 0 && oprwv == FIO_READ) {
    int hiaddr = avr_mem_hiaddr(mem);   // @@@ Should check segments only, not all file

    if(hiaddr < rc)             // If trailing-0xff not disabled
      rc = hiaddr;
  }

  if(format != FMT_IMM && !using_stdio) {
    fclose(f);
  }

  return rc;
}

int fileio_segments(int oprwv, const char *filename, FILEFMT format,
  const AVRPART *p, const AVRMEM *mem, int n, const Segment *list) {

  Segment *seglist = mmt_malloc(n*sizeof *seglist);

  memcpy(seglist, list, n*sizeof *seglist);
  int ret = fileio_segments_normalise(oprwv, filename, format, p, mem, n, seglist);

  mmt_free(seglist);

  return ret;
}
