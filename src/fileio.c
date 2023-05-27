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
#  define EM_AVR32 0x18ad         /* unofficial */
#endif
#ifndef EM_AVR
#  define EM_AVR 83               /* OpenBSD lacks it */
#endif
#endif

#include "avrdude.h"
#include "libavrdude.h"


#define IHEX_MAXDATA 256

#define MAX_LINE_LEN 256  /* max line length for ASCII format input files */


struct ihexrec {
  unsigned char    reclen;
  unsigned int     loadofs;
  unsigned char    rectyp;
  unsigned char    data[IHEX_MAXDATA];
  unsigned char    cksum;
};


static int b2ihex(const unsigned char *inbuf, int bufsize,
             int recsize, int startaddr,
             const char *outfile, FILE *outf, FILEFMT ffmt);

static int ihex2b(const char *infile, FILE *inf,
             const AVRMEM *mem, int bufsize, unsigned int fileoffset,
             FILEFMT ffmt);

static int b2srec(const unsigned char *inbuf, int bufsize,
             int recsize, int startaddr,
             const char *outfile, FILE *outf);

static int srec2b(const char *infile, FILE *inf,
             const AVRMEM *mem, int bufsize, unsigned int fileoffset);

static int ihex_readrec(struct ihexrec *ihex, char *rec);

static int srec_readrec(struct ihexrec *srec, char *rec);

static int fileio_rbin(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size);

static int fileio_ihex(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size,
             FILEFMT ffmt);

static int fileio_srec(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size);

#ifdef HAVE_LIBELF
static int elf2b(const char *infile, FILE *inf,
                 const AVRMEM *mem, const AVRPART *p,
                 int bufsize, unsigned int fileoffset);

static int fileio_elf(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem,
             const AVRPART *p, int size);
#endif

static int fileio_num(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size,
             FILEFMT fmt);


char *fileio_fmtstr(FILEFMT format) {
  switch (format) {
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
 };
}


FILEFMT fileio_format(char c) {
  switch (c) {
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


static int b2ihex(const unsigned char *inbuf, int bufsize, int recsize,
  int startaddr, const char *outfile_unused, FILE *outf, FILEFMT ffmt) {

  const unsigned char *buf;
  unsigned int nextaddr;
  int n, nbytes, n_64k;
  int i;
  unsigned char cksum;

  if (recsize > 255) {
    pmsg_error("recsize=%d, must be < 256\n", recsize);
    return -1;
  }

  n_64k    = 0;
  nextaddr = startaddr;
  buf      = inbuf;
  nbytes   = 0;

  while (bufsize) {
    n = recsize;
    if (n > bufsize)
      n = bufsize;

    if ((nextaddr + n) > 0x10000)
      n = 0x10000 - nextaddr;

    if (n) {
      cksum = 0;
      fprintf(outf, ":%02X%04X00", n, nextaddr);
      cksum += n + ((nextaddr >> 8) & 0x0ff) + (nextaddr & 0x0ff);
      for (i=0; i<n; i++) {
        fprintf(outf, "%02X", buf[i]);
        cksum += buf[i];
      }
      cksum = -cksum;
      fprintf(outf, "%02X", cksum);

      if(ffmt == FMT_IHXC) { /* Print comment with address and ASCII dump */
        for(i=n; i<recsize; i++)
          fprintf(outf, "  ");
        fprintf(outf, " // %05x> ", n_64k*0x10000 + nextaddr);
        for (i=0; i<n; i++) {
          unsigned char c = buf[i] & 0x7f;
          /* Print space as _ so that line is one word */
          putc(c == ' '? '_': c < ' ' || c == 0x7f? '.': c, outf);
        }
      }
      putc('\n', outf);

      nextaddr += n;
      nbytes   += n;
    }

    if (nextaddr >= 0x10000) {
      int lo, hi;
      /* output an extended address record */
      n_64k++;
      lo = n_64k & 0xff;
      hi = (n_64k >> 8) & 0xff;
      cksum = 0;
      fprintf(outf, ":02000004%02X%02X", hi, lo);
      cksum += 2 + 0 + 4 + hi + lo;
      cksum = -cksum;
      fprintf(outf, "%02X\n", cksum);
      nextaddr = 0;
    }

    /* advance to next 'recsize' bytes */
    buf += n;
    bufsize -= n;
  }

  /*-----------------------------------------------------------------
    add the end of record data line
    -----------------------------------------------------------------*/
  cksum = 0;
  n = 0;
  nextaddr = 0;
  fprintf(outf, ":%02X%04X01", n, nextaddr);
  cksum += n + ((nextaddr >> 8) & 0x0ff) + (nextaddr & 0x0ff) + 1;
  cksum = -cksum;
  fprintf(outf, "%02X\n", cksum);

  return nbytes;
}


static int ihex_readrec(struct ihexrec * ihex, char * rec)
{
  int i, j;
  char buf[8];
  int offset, len;
  char * e;
  unsigned char cksum;
  int rc;

  len    = strlen(rec);
  offset = 1;
  cksum  = 0;

  /* reclen */
  if (offset + 2 > len)
    return -1;
  for (i=0; i<2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->reclen = strtoul(buf, &e, 16);
  if (e == buf || *e != 0)
    return -1;

  /* load offset */
  if (offset + 4 > len)
    return -1;
  for (i=0; i<4; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->loadofs = strtoul(buf, &e, 16);
  if (e == buf || *e != 0)
    return -1;

  /* record type */
  if (offset + 2 > len)
    return -1;
  for (i=0; i<2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->rectyp = strtoul(buf, &e, 16);
  if (e == buf || *e != 0)
    return -1;

  cksum = ihex->reclen + ((ihex->loadofs >> 8) & 0x0ff) + 
    (ihex->loadofs & 0x0ff) + ihex->rectyp;

  /* data */
  for (j=0; j<ihex->reclen; j++) {
    if (offset + 2 > len)
      return -1;
    for (i=0; i<2; i++)
      buf[i] = rec[offset++];
    buf[i] = 0;
    ihex->data[j] = strtoul(buf, &e, 16);
    if (e == buf || *e != 0)
      return -1;
    cksum += ihex->data[j];
  }

  /* cksum */
  if (offset + 2 > len)
    return -1;
  for (i=0; i<2; i++)
    buf[i] = rec[offset++];
  buf[i] = 0;
  ihex->cksum = strtoul(buf, &e, 16);
  if (e == buf || *e != 0)
    return -1;

  rc = -cksum & 0x000000ff;

  return rc;
}



/*
 * Intel Hex to binary buffer
 *
 * Given an open file 'inf' which contains Intel Hex formatted data,
 * parse the file and lay it out within the memory buffer pointed to
 * by outbuf. The size of outbuf, 'bufsize' is honored; if data would
 * fall outsize of the memory buffer outbuf, an error is generated.
 *
 * Return the maximum memory address within 'outbuf' that was written
 * plus one. If an error occurs, return -1.
 *
 * */
static int ihex2b(const char *infile, FILE *inf,
             const AVRMEM *mem, int bufsize, unsigned int fileoffset,
             FILEFMT ffmt)
{
  char buffer [ MAX_LINE_LEN ];
  unsigned int nextaddr, baseaddr, maxaddr;
  int i;
  int lineno;
  int len;
  struct ihexrec ihex;
  int rc;

  lineno   = 0;
  baseaddr = 0;
  maxaddr  = 0;
  nextaddr = 0;

  while (fgets((char *)buffer,MAX_LINE_LEN,inf)!=NULL) {
    lineno++;
    len = strlen(buffer);
    if (buffer[len-1] == '\n') 
      buffer[--len] = 0;
    if (buffer[0] != ':')
      continue;
    rc = ihex_readrec(&ihex, buffer);
    if (rc < 0) {
      pmsg_error("invalid record at line %d of %s\n", lineno, infile);
      return -1;
    }
    else if (rc != ihex.cksum) {
      if(ffmt == FMT_IHEX) {
        pmsg_error("checksum mismatch at line %d of %s\n", lineno, infile);
        imsg_error("checksum=0x%02x, computed checksum=0x%02x\n", ihex.cksum, rc);
        return -1;
      } else {                  /* Just warn with more permissive format FMT_IHXC */
        pmsg_notice("checksum mismatch at line %d of %s\n", lineno, infile);
        imsg_notice("checksum=0x%02x, computed checksum=0x%02x\n", ihex.cksum, rc);
      }
    }

    switch (ihex.rectyp) {
      case 0: /* data record */
        if (fileoffset != 0 && baseaddr < fileoffset) {
          pmsg_error("address 0x%04x out of range (below fileoffset 0x%x) at line %d of %s\n",
            baseaddr, fileoffset, lineno, infile);
          return -1;
        }
        nextaddr = ihex.loadofs + baseaddr - fileoffset;
        if (nextaddr + ihex.reclen > (unsigned) bufsize) {
          pmsg_error("address 0x%04x out of range at line %d of %s\n",
            nextaddr+ihex.reclen, lineno, infile);
          return -1;
        }
        for (i=0; i<ihex.reclen; i++) {
          mem->buf[nextaddr+i] = ihex.data[i];
          mem->tags[nextaddr+i] = TAG_ALLOCATED;
        }
        if (nextaddr+ihex.reclen > maxaddr)
          maxaddr = nextaddr+ihex.reclen;
        break;

      case 1: /* end of file record */
        return maxaddr;
        break;

      case 2: /* extended segment address record */
        baseaddr = (ihex.data[0] << 8 | ihex.data[1]) << 4;
        break;

      case 3: /* start segment address record */
        /* we don't do anything with the start address */
        break;

      case 4: /* extended linear address record */
        baseaddr = (ihex.data[0] << 8 | ihex.data[1]) << 16;
        break;

      case 5: /* start linear address record */
        /* we don't do anything with the start address */
        break;

      default:
        pmsg_error("do not know how to deal with rectype=%d "
          "at line %d of %s\n", ihex.rectyp, lineno, infile);
        return -1;
        break;
    }

  } /* while */

  if (maxaddr == 0) {
    pmsg_error("no valid record found in Intel Hex file %s\n", infile);

    return -1;
  }
  else {
    pmsg_warning("no end of file record found for Intel Hex file %s\n", infile);

    return maxaddr;
  }
}

static int b2srec(const unsigned char *inbuf, int bufsize, int recsize,
  int startaddr, const char *outfile_unused, FILE *outf) {

  const unsigned char *buf;
  unsigned int nextaddr;
  int n, nbytes, addr_width;
  unsigned char cksum;

  char * tmpl=0;

  if (recsize > 255) {
    pmsg_error("recsize=%d, must be < 256\n", recsize);
    return -1;
  }
  
  nextaddr = startaddr;
  buf = inbuf;
  nbytes = 0;    

  addr_width = 0;

  while (bufsize) {

    n = recsize;

    if (n > bufsize) 
      n = bufsize;

    if (n) {
      cksum = 0;
      if (nextaddr + n <= 0xffff) {
        addr_width = 2;
        tmpl="S1%02X%04X";
      }
      else if (nextaddr + n <= 0xffffff) {
        addr_width = 3;
        tmpl="S2%02X%06X";
      }
      else if (nextaddr + n <= 0xffffffff) {
        addr_width = 4;
        tmpl="S3%02X%08X";
      }
      else {
        pmsg_error("address=%d, out of range\n", nextaddr);
        return -1;
      }

      fprintf(outf, tmpl, n + addr_width + 1, nextaddr);

      cksum += n + addr_width + 1;

      for (int i=addr_width; i>0; i--)
        cksum += (nextaddr >> (i-1) * 8) & 0xff;

      for (unsigned i=nextaddr; i<nextaddr + n; i++) {
        fprintf(outf, "%02X", buf[i]);
        cksum += buf[i];
      }

      cksum = 0xff - cksum;
      fprintf(outf, "%02X\n", cksum);

      nextaddr += n;
      nbytes +=n;
    }

    /* advance to next 'recsize' bytes */
    bufsize -= n;
  }

  /*-----------------------------------------------------------------
    add the end of record data line
    -----------------------------------------------------------------*/
  cksum = 0;
  n = 0;
  nextaddr = 0;

  if (startaddr <= 0xffff) {
    addr_width = 2;
    tmpl="S9%02X%04X";
  }
  else if (startaddr <= 0xffffff) {
    addr_width = 3;
    tmpl="S9%02X%06X";
  }
  else if ((unsigned) startaddr <= 0xffffffff) {
    addr_width = 4;
    tmpl="S9%02X%08X";
  }

  fprintf(outf, tmpl, n + addr_width + 1, nextaddr);

  cksum += n + addr_width +1;
  for (int i=addr_width; i>0; i--)
    cksum += (nextaddr >> (i - 1) * 8) & 0xff;
  cksum = 0xff - cksum;
  fprintf(outf, "%02X\n", cksum);

  return nbytes; 
}


static int srec_readrec(struct ihexrec * srec, char * rec)
{
  int i, j;
  char buf[8];
  int offset, len, addr_width;
  char * e;
  unsigned char cksum;
  int rc;

  len = strlen(rec);
  offset = 1;
  cksum = 0;
  addr_width = 2;

  /* record type */
  if (offset + 1 > len) 
    return -1;
  srec->rectyp = rec[offset++];
  if (srec->rectyp == 0x32 || srec->rectyp == 0x38) 
    addr_width = 3;             /* S2,S8-record */
  else if (srec->rectyp == 0x33 || srec->rectyp == 0x37) 
    addr_width = 4;             /* S3,S7-record */

  /* reclen */
  if (offset + 2 > len) 
    return -1;
  for (i=0; i<2; i++) 
    buf[i] = rec[offset++];
  buf[i] = 0;
  srec->reclen = strtoul(buf, &e, 16);
  cksum += srec->reclen;
  srec->reclen -= (addr_width+1);
  if (e == buf || *e != 0) 
    return -1;

  /* load offset */
  if (offset + addr_width > len) 
    return -1;
  for (i=0; i<addr_width*2; i++) 
    buf[i] = rec[offset++];
  buf[i] = 0;
  srec->loadofs = strtoull(buf, &e, 16);
  if (e == buf || *e != 0) 
    return -1;

  for (i=addr_width; i>0; i--)
    cksum += (srec->loadofs >> (i - 1) * 8) & 0xff;

  /* data */
  for (j=0; j<srec->reclen; j++) {
    if (offset+2  > len) 
      return -1;
    for (i=0; i<2; i++) 
      buf[i] = rec[offset++];
    buf[i] = 0;
    srec->data[j] = strtoul(buf, &e, 16);
    if (e == buf || *e != 0) 
      return -1;
    cksum += srec->data[j];
  }

  /* cksum */
  if (offset + 2 > len) 
    return -1;
  for (i=0; i<2; i++) 
    buf[i] = rec[offset++];
  buf[i] = 0;
  srec->cksum = strtoul(buf, &e, 16);
  if (e == buf || *e != 0) 
    return -1;

  rc = 0xff - cksum;
  return rc;
}


static int srec2b(const char *infile, FILE * inf,
           const AVRMEM *mem, int bufsize, unsigned int fileoffset)
{
  char buffer [ MAX_LINE_LEN ];
  unsigned int nextaddr, maxaddr;
  int i;
  int lineno;
  int len;
  struct ihexrec srec;
  int rc;
  unsigned int reccount;
  unsigned char datarec;

  char * msg = "";

  lineno   = 0;
  maxaddr  = 0;
  reccount = 0;

  while (fgets((char *)buffer,MAX_LINE_LEN,inf)!=NULL) {
    lineno++;
    len = strlen(buffer);
    if (buffer[len-1] == '\n') 
      buffer[--len] = 0;
    if (buffer[0] != 0x53)
      continue;
    rc = srec_readrec(&srec, buffer);

    if (rc < 0) {
      pmsg_error("invalid record at line %d of %s\n", lineno, infile);
      return -1;
    }
    else if (rc != srec.cksum) {
      pmsg_error("checksum mismatch at line %d of %s\n", lineno, infile);
      imsg_error("checksum=0x%02x, computed checksum=0x%02x\n", srec.cksum, rc);
      return -1;
    }

    datarec=0; 
    switch (srec.rectyp) {
      case 0x30: /* S0 - header record*/
        /* skip */
        break;

      case 0x31: /* S1 - 16 bit address data record */
        datarec=1;
        msg="address 0x%04x out of range %sat line %d of %s\n";
        break;

      case 0x32: /* S2 - 24 bit address data record */
        datarec=1;
        msg="address 0x%06x out of range %sat line %d of %s\n";
        break;

      case 0x33: /* S3 - 32 bit address data record */
        datarec=1;
        msg="address 0x%08x out of range %sat line %d of %s\n";
        break;

      case 0x34: /* S4 - symbol record (LSI extension) */
        pmsg_error("not supported record at line %d of %s\n", lineno, infile);
        return -1;

      case 0x35: /* S5 - count of S1,S2 and S3 records previously tx'd */
        if (srec.loadofs != reccount){
          pmsg_error("count of transmitted data records mismatch at line %d of %s\n", lineno, infile);
          imsg_error("transmitted data records= %d, expected value= %d\n", reccount, srec.loadofs);
          return -1;
        }
        break;

      case 0x37: /* S7 Record - end record for 32 bit address data */
      case 0x38: /* S8 Record - end record for 24 bit address data */
      case 0x39: /* S9 Record - end record for 16 bit address data */
        return maxaddr;

      default:
        pmsg_error("do not know how to deal with rectype S%d at line %d of %s\n",
          srec.rectyp, lineno, infile);
        return -1;
    }

    if (datarec == 1) {
      nextaddr = srec.loadofs;
      if (nextaddr < fileoffset) {
        pmsg_error(msg, nextaddr, "(below fileoffset) ", lineno, infile);
        return -1;
      }
      nextaddr -= fileoffset;
      if (nextaddr + srec.reclen > (unsigned) bufsize) {
        pmsg_error(msg, nextaddr+srec.reclen, "", lineno, infile);
        return -1;
      }
      for (i=0; i<srec.reclen; i++) {
        mem->buf[nextaddr+i] = srec.data[i];
        mem->tags[nextaddr+i] = TAG_ALLOCATED;
      }
      if (nextaddr+srec.reclen > maxaddr)
        maxaddr = nextaddr+srec.reclen;
      reccount++;      
    }

  }

  pmsg_warning("no end of file record found for Motorola S-Records file %s\n", infile);

  return maxaddr;
}

#ifdef HAVE_LIBELF
/*
 * Determine whether the ELF file section pointed to by `sh' fits
 * completely into the program header segment pointed to by `ph'.
 *
 * Assumes the section has been checked already before to actually
 * contain data (SHF_ALLOC, SHT_PROGBITS, sh_size > 0).
 *
 * Sometimes, program header segments might be larger than the actual
 * file sections.  On VM architectures, this is used to allow mmapping
 * the entire ELF file "as is" (including things like the program
 * header table itself).
 */
static inline
int is_section_in_segment(Elf32_Shdr *sh, Elf32_Phdr *ph)
{
    if (sh->sh_offset < ph->p_offset)
        return 0;
    if (sh->sh_offset + sh->sh_size > ph->p_offset + ph->p_filesz)
        return 0;
    return 1;
}


static int elf_mem_limits(const AVRMEM *mem, const AVRPART *p,
                          unsigned int *lowbound,
                          unsigned int *highbound,
                          unsigned int *fileoff)
{
  int rv = 0;

  if (p->prog_modes & PM_aWire) { // AVR32
    if (strcmp(mem->desc, "flash") == 0) {
      *lowbound = 0x80000000;
      *highbound = 0xffffffff;
      *fileoff = 0;
    } else {
      rv = -1;
    }
  } else {
    if (strcmp(mem->desc, "flash") == 0 ||
        strcmp(mem->desc, "boot") == 0 ||
        strcmp(mem->desc, "application") == 0 ||
        strcmp(mem->desc, "apptable") == 0) {
      *lowbound = 0;
      *highbound = 0x7Fffff;    // Max 8 MiB
      *fileoff = 0;
    } else if (strcmp(mem->desc, "data") == 0) { // SRAM for XMEGAs
      *lowbound = 0x802000;
      *highbound = 0x80ffff;
      *fileoff = 0;
    } else if (strcmp(mem->desc, "eeprom") == 0) {
      *lowbound = 0x810000;
      *highbound = 0x81ffff;    // Max 64 KiB
      *fileoff = 0;
    } else if (strcmp(mem->desc, "lfuse") == 0 || strcmp(mem->desc, "fuses") == 0) {
      *lowbound = 0x820000;
      *highbound = 0x82ffff;
      *fileoff = 0;
    } else if (strcmp(mem->desc, "hfuse") == 0) {
      *lowbound = 0x820000;
      *highbound = 0x82ffff;
      *fileoff = 1;
    } else if (strcmp(mem->desc, "efuse") == 0) {
      *lowbound = 0x820000;
      *highbound = 0x82ffff;
      *fileoff = 2;
    } else if (strncmp(mem->desc, "fuse", 4) == 0 &&
               (mem->desc[4] >= '0' && mem->desc[4] <= '9')) {
      /* Xmega fuseN */
      *lowbound = 0x820000;
      *highbound = 0x82ffff;
      *fileoff = mem->desc[4] - '0';
    } else if (strncmp(mem->desc, "lock", 4) == 0) { // Lock or lockbits
      *lowbound = 0x830000;
      *highbound = 0x83ffff;
      *fileoff = 0;
    } else if (strcmp(mem->desc, "signature") == 0) { // Read only
      *lowbound = 0x840000;
      *highbound = 0x84ffff;
      *fileoff = 0;
    } else if (strncmp(mem->desc, "user", 4) == 0) { // Usersig or userrow
      *lowbound = 0x850000;
      *highbound = 0x85ffff;
      *fileoff = 0;
    } else {
      rv = -1;
    }
  }

  return rv;
}


static int elf2b(const char *infile, FILE *inf, const AVRMEM *mem,
  const AVRPART *p, int bufsize_unused, unsigned int fileoffset_unused) {

  Elf *e;
  int rv = 0, size = 0;
  unsigned int low, high, foff;

  if (elf_mem_limits(mem, p, &low, &high, &foff) != 0) {
    pmsg_error("cannot handle %s memory region from ELF file\n", mem->desc);
    return -1;
  }

  /*
   * The Xmega memory regions for "boot", "application", and
   * "apptable" are actually sub-regions of "flash".  Refine the
   * applicable limits.  This allows to select only the appropriate
   * sections out of an ELF file that contains section data for more
   * than one sub-segment.
   */
  if ((p->prog_modes & PM_PDI) != 0 &&
      (strcmp(mem->desc, "boot") == 0 ||
       strcmp(mem->desc, "application") == 0 ||
       strcmp(mem->desc, "apptable") == 0)) {
    AVRMEM *flashmem = avr_locate_mem(p, "flash");
    if (flashmem == NULL) {
      pmsg_error("no flash memory region found, cannot compute bounds of %s sub-region\n", mem->desc);
      return -1;
    }
    /* The config file offsets are PDI offsets, rebase to 0. */
    low = mem->offset - flashmem->offset;
    high = low + mem->size - 1;
  }

  if (elf_version(EV_CURRENT) == EV_NONE) {
    pmsg_error("ELF library initialization failed: %s\n", elf_errmsg(-1));
    return -1;
  }
  if ((e = elf_begin(fileno(inf), ELF_C_READ, NULL)) == NULL) {
    pmsg_error("cannot open %s as an ELF file: %s\n", infile, elf_errmsg(-1));
    return -1;
  }
  if (elf_kind(e) != ELF_K_ELF) {
    pmsg_error("cannot use %s as an ELF input file\n", infile);
    goto done;
  }

  size_t i, isize;
  const char *id = elf_getident(e, &isize);

  if (id == NULL) {
    pmsg_error("unable to read ident area of %s: %s\n", infile, elf_errmsg(-1));
    goto done;
  }

  const char *endianname;
  unsigned char endianess;
  if (p->prog_modes & PM_aWire) { // AVR32
    endianess = ELFDATA2MSB;
    endianname = "little";
  } else {
    endianess = ELFDATA2LSB;
    endianname = "big";
  }
  if (id[EI_CLASS] != ELFCLASS32 ||
      id[EI_DATA] != endianess) {
    pmsg_error("ELF file %s is not a 32-bit, %s-endian file that was expected\n",
      infile, endianname);
    goto done;
  }

  Elf32_Ehdr *eh;
  if ((eh = elf32_getehdr(e)) == NULL) {
    pmsg_error("unable to read ehdr of %s: %s\n", infile, elf_errmsg(-1));
    goto done;
  }

  if (eh->e_type != ET_EXEC) {
    pmsg_error("ELF file %s is not an executable file\n", infile);
    goto done;
  }

  const char *mname;
  uint16_t machine;
  if (p->prog_modes & PM_aWire) {
    machine = EM_AVR32;
    mname = "AVR32";
  } else {
    machine = EM_AVR;
    mname = "AVR";
  }
  if (eh->e_machine != machine) {
    pmsg_error("ELF file %s is not for machine %s\n", infile, mname);
    goto done;
  }
  if (eh->e_phnum == 0xffff /* PN_XNUM */) {
    pmsg_error("ELF file %s uses extended program header numbers which are not expected\n", infile);
    goto done;
  }

  Elf32_Phdr *ph;
  if ((ph = elf32_getphdr(e)) == NULL) {
    pmsg_error("unable to read program header table of %s: %s\n", infile, elf_errmsg(-1));
    goto done;
  }

  size_t sndx;
  if (elf_getshdrstrndx(e, &sndx) != 0) {
    pmsg_error("unable to obtain section name string table: %s\n", elf_errmsg(-1));
    sndx = 0;
  }

  /*
   * Walk the program header table, pick up entries that are of type
   * PT_LOAD, and have a non-zero p_filesz.
   */
  for (i = 0; i < eh->e_phnum; i++) {
    if (ph[i].p_type != PT_LOAD || ph[i].p_filesz == 0)
      continue;

    pmsg_notice2("considering PT_LOAD program header entry #%d\n", (int) i);
    imsg_notice2("p_vaddr 0x%x, p_paddr 0x%x, p_filesz %d\n", ph[i].p_vaddr, ph[i].p_paddr, ph[i].p_filesz);

    Elf_Scn *scn = NULL;
    while ((scn = elf_nextscn(e, scn)) != NULL) {
      size_t ndx = elf_ndxscn(scn);
      Elf32_Shdr *sh = elf32_getshdr(scn);

      if (sh == NULL) {
        pmsg_error("unable to read section #%u header: %s\n", (unsigned int) ndx, elf_errmsg(-1));
        rv = -1;
        continue;
      }
      // Only interested in PROGBITS, ALLOC sections
      if ((sh->sh_flags & SHF_ALLOC) == 0 || sh->sh_type != SHT_PROGBITS)
        continue;
      // Not interested in empty sections
      if (sh->sh_size == 0)
        continue;
      // Section must belong to this segment
      if (!is_section_in_segment(sh, ph+i))
        continue;

      const char *sname = sndx? elf_strptr(e, sndx, sh->sh_name): "*unknown*";
      unsigned int lma = ph[i].p_paddr + sh->sh_offset - ph[i].p_offset;

      pmsg_notice2("found section %s, LMA 0x%x, sh_size %u\n", sname, lma, sh->sh_size);

      if(!(lma >= low && lma + sh->sh_size < high)) {
        imsg_notice2("skipping %s (inappropriate for %s)\n", sname, mem->desc);
        continue;
      }
      /*
       * 1-byte sized memory regions are special: they are used for fuse
       * bits, where multiple regions (in the config file) map to a
       * single, larger region in the ELF file (e.g. "lfuse", "hfuse",
       * and "efuse" all map to ".fuse").  We silently accept a larger
       * ELF file region for these, and extract the actual byte to write
       * from it, using the "foff" offset obtained above.
       */
      if (mem->size != 1 && sh->sh_size > (unsigned) mem->size) {
        pmsg_error("section %s of size %u does not fit into %s of size %d\n",
          sname, sh->sh_size, mem->desc, mem->size);
        rv = -1;
        continue;
      }

      Elf_Data *d = NULL;
      while ((d = elf_getdata(scn, d)) != NULL) {
        imsg_notice2("data block: d_buf %p, d_off 0x%x, d_size %ld\n",
          d->d_buf, (unsigned int)d->d_off, (long) d->d_size);
        if (mem->size == 1) {
          if (d->d_off != 0) {
            pmsg_error("unexpected data block at offset != 0\n");
            rv = -1;
          } else if (foff >= d->d_size) {
            pmsg_error("ELF file section does not contain byte at offset %d\n", foff);
            rv = -1;
          } else {
            imsg_notice2("extracting one byte from file offset %d\n", foff);
            mem->buf[0] = ((unsigned char *)d->d_buf)[foff];
            mem->tags[0] = TAG_ALLOCATED;
            size = 1;
          }
        } else {
          int idx = lma-low + d->d_off;
          int end = idx + d->d_size;

          if(idx >= 0 && idx < mem->size && end >= 0 && end <= mem->size && end-idx >= 0) {
            if (end > size)
              size = end;
            imsg_debug("writing %d bytes to mem offset 0x%x\n", end-idx, idx);
            memcpy(mem->buf + idx, d->d_buf, end-idx);
            memset(mem->tags + idx, TAG_ALLOCATED, end-idx);
          } else {
            pmsg_error("section %s [0x%04x, 0x%04x] does not fit into %s [0, 0x%04x]\n",
              sname, idx, (int) (idx + d->d_size-1), mem->desc, mem->size-1);
            rv = -1;
          }
        }
      }
    }
  }
done:
  (void)elf_end(e);
  return rv<0? rv: size;
}
#endif  /* HAVE_LIBELF */


static int fileio_rbin(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size) {
  int rc;
  unsigned char *buf = mem->buf;

  switch (fio->op) {
    case FIO_READ:
      rc = fread(buf, 1, size, f);
      if (rc > 0)
        memset(mem->tags, TAG_ALLOCATED, rc);
      break;
    case FIO_WRITE:
      rc = fwrite(buf, 1, size, f);
      break;
    default:
      pmsg_error("invalid fileio operation=%d\n", fio->op);
      return -1;
  }

  if (rc < 0 || (fio->op == FIO_WRITE && rc < size)) {
    pmsg_ext_error("%s error %s %s: %s; %s %d of the expected %d bytes\n",
      fio->iodesc, fio->dir, filename, strerror(errno), fio->rw, rc, size);
    return -1;
  }

  return rc;
}


static int fileio_imm(struct fioparms *fio, const char *fname, FILE *f_unused,
 const AVRMEM *mem, int size) {

  int rc = 0;
  char *tok, *p, *line;
  const char *errstr;
  int loc;

  p = line = cfg_strdup(__func__, fname);

  switch (fio->op) {
    case FIO_READ:
      loc = 0;
      while(*(tok = str_nexttok(p, ", \t\n\r\v\f", &p)) && loc < size) {
        int set = str_membuf(tok, STR_ANY, mem->buf+loc, mem->size-loc, &errstr);
        if(errstr || set < 0) {
          pmsg_error("invalid data %s in immediate mode: %s\n", tok, errstr);
          free(line);
          return -1;
        }
        memset(mem->tags+loc, TAG_ALLOCATED, set);
        loc += set;
        rc = loc;
      }
      break;

    case FIO_WRITE:
      pmsg_error("invalid file format 'immediate' for output\n");
      free(line);
      return -1;

    default:
      pmsg_error("invalid operation=%d\n", fio->op);
      free(line);
      return -1;
  }

  if (rc < 0 || (fio->op == FIO_WRITE && rc < size)) {
    pmsg_ext_error("%s error %s %s: %s; %s %d of the expected %d bytes\n",
      fio->iodesc, fio->dir, line, strerror(errno), fio->rw, rc, size);
    free(line);
    return -1;
  }

  free(line);
  return rc;
}


static int fileio_ihex(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size,
             FILEFMT ffmt)
{
  int rc;

  switch (fio->op) {
    case FIO_WRITE:
      rc = b2ihex(mem->buf, size, 32, fio->fileoffset, filename, f, ffmt);
      if (rc < 0) {
        return -1;
      }
      break;

    case FIO_READ:
      rc = ihex2b(filename, f, mem, size, fio->fileoffset, ffmt);
      if (rc < 0)
        return -1;
      break;

    default:
      pmsg_error("invalid Intel Hex file I/O operation=%d\n", fio->op);
      return -1;
      break;
  }

  return rc;
}


static int fileio_srec(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size)
{
  int rc;

  switch (fio->op) {
    case FIO_WRITE:
      rc = b2srec(mem->buf, size, 32, fio->fileoffset, filename, f);
      if (rc < 0) {
        return -1;
      }
      break;

    case FIO_READ:
      rc = srec2b(filename, f, mem, size, fio->fileoffset);
      if (rc < 0)
        return -1;
      break;

    default:
      pmsg_error("invalid Motorola S-Records file I/O operation=%d\n", fio->op);
      return -1;
      break;
  }

  return rc;
}


#ifdef HAVE_LIBELF
static int fileio_elf(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem,
             const AVRPART *p, int size)
{
  int rc;

  switch (fio->op) {
    case FIO_WRITE:
      pmsg_error("write operation not supported for ELF\n");
      return -1;
      break;

    case FIO_READ:
      rc = elf2b(filename, f, mem, p, size, fio->fileoffset);
      return rc;

    default:
      pmsg_error("invalid ELF file I/O operation=%d\n", fio->op);
      return -1;
      break;
  }
}

#endif


static int b2num(const char *filename, FILE *f, const AVRMEM *mem, int size, FILEFMT fmt) {
  const char *prefix;
  int base;

  switch (fmt) {
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
  }

  for (int i = 0; i < size; i++) {
    char cbuf[20];

    if (i > 0) {
      if (putc(',', f) == EOF)
        goto writeerr;
    }
    unsigned num = mem->buf[i];
    /*
     * For a base of 8 and a value < 8 to convert, don't write the
     * prefix.  The conversion will be indistinguishable from a
     * decimal one then.
     */
    if (prefix[0] != '\0' && !(base == 8 && num < 8)) {
      if (fputs(prefix, f) == EOF)
        goto writeerr;
    }
   str_utoa(num, cbuf, base);
    if (fputs(cbuf, f) == EOF)
      goto writeerr;
  }
  if (putc('\n', f) == EOF)
    goto writeerr;

  return 0;

 writeerr:
  pmsg_ext_error("unable to write to %s: %s\n", filename, strerror(errno));
  return -1;
}


// Allocates sufficient memory for a line; returned pointer to be free'd
char *Nfgets(FILE *fp) {
  int bs = 1023;                // Must be 2^n - 1
  char *ret = (char *) cfg_malloc(__func__, bs);

  ret[bs-2] = 0;
  if(!fgets(ret, bs, fp)) {
    free(ret);
    return NULL;
  }

  while(ret[bs-2] != 0 && ret[bs-2] != '\n') {
    if(bs >= INT_MAX/2) {
      pmsg_error("cannot cope with lines longer than %d bytes\n", INT_MAX);
      free(ret);
      return NULL;
    }
    int was = bs;
    bs = 2*bs+1;
    ret = cfg_realloc(__func__, ret, bs);
    ret[was-1] = ret[bs-2] = 0;
    if(!fgets(ret+was-1, bs-(was-1), fp)) { // EOF? Error?
      if(ferror(fp)) {
        free(ret);
        return NULL;
      }
      break;
    }
  }

  return ret;
}


static int num2b(const char *filename, FILE *f, const AVRMEM *mem) {
  char *line;
  int n = 0;

  while(n < mem->size && (line = Nfgets(f))) {
    char *p = line, *tok;
    while(*p && isspace(*p & 0xff)) // Skip white space, comments and empty lines
      p++;
    if(*p && *p != '#') {
      while(*(tok = str_nexttok(p, ", \t\n\r\v\f", &p)) && n < mem->size) {
        const char *errstr;
        if(*tok == '#')           // Ignore rest of line after #
          break;
        int set = str_membuf(tok, STR_ANY, mem->buf+n, mem->size-n, &errstr);
        if(errstr || set < 0) {
          pmsg_error("invalid data %s in immediate mode: %s\n", tok, errstr);
          free(line);
          return -1;
        }
        memset(mem->tags+n, TAG_ALLOCATED, set);
        n += set;
      }
    }
    free(line);
  }
  return n;
}


static int fileio_num(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size,
             FILEFMT fmt)
{

  switch (fio->op) {
    case FIO_WRITE:
      return b2num(filename, f, mem, size, fmt);

    case FIO_READ:
      return num2b(filename, f, mem);

    default:
      pmsg_error("invalid operation=%d\n", fio->op);
      return -1;
  }
}


int fileio_setparms(int op, struct fioparms *fp, const AVRPART *p, const AVRMEM * m) {
  fp->op = op;

  switch (op) {
    case FIO_READ:
      fp->mode   = "r";
      fp->iodesc = "input";
      fp->dir    = "from";
      fp->rw     = "read";
      break;

    case FIO_WRITE:
      fp->mode   = "w";
      fp->iodesc = "output";
      fp->dir    = "to";
      fp->rw     = "wrote";
      break;

    default:
      pmsg_error("invalid I/O operation %d\n", op);
      return -1;
      break;
  }

  /*
   * AVR32 devices maintain their load offset within the file itself,
   * but AVRDUDE maintains all memory images 0-based.
   */
  fp->fileoffset = p->prog_modes & PM_aWire? m->offset: 0;

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
  size_t i, nxdigs, len;

  // Check for ELF file
  if(first && line[0] == 0177 && str_starts((char *) line+1, "ELF"))
    return FMT_ELF;

  len = strlen((char *) line);
  while(len > 0 && line[len-1] && isspace(line[len-1])) // cr/lf etc
    line[--len] = 0;

  // Check for binary data
  for(i=0; i<len; i++)
    if(line[i] > 127)
      return FMT_RBIN;

  // Check for lines that look like Intel HEX
  if(line[0] == ':' && len >= 11 && isxdigit(line[1]) && isxdigit(line[2])) {
    nxdigs = sscanf((char *) line+1, "%2zx", &nxdigs) == 1? 2*nxdigs + 8: len;
    for(found = 3+nxdigs <= len, i=0; found && i<nxdigs; i++)
      if(!isxdigit(line[3+i]))
        found = 0;
    if(found)
      return FMT_IHEX;
  }

  // Check for lines that look like Motorola S-record
  if(line[0] == 'S' && len >= 10 && isdigit(line[1]) && isxdigit(line[2]) && isxdigit(line[3])) {
    nxdigs = sscanf((char *) line+2, "%2zx", &nxdigs) == 1? 2*nxdigs: len;
    for(found = 4+nxdigs <= len, i=0; found && i<nxdigs; i++)
      if(!isxdigit(line[4+i]))
        found = 0;
    if(found)
      return FMT_SREC;
  }

  // Check for terminal-type data entries
  char *p = (char *) line, *tok;
  int failed = 0, idx[4] = {0, 0, 0, 0};
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
          *tok=='0' && tok[1] && strchr("01234567", tok[1])? 2: 3]++;
    }
    if(!failed && idx[0]+idx[1]+idx[2]+idx[3]) {
      // Doesn't matter which one: they all parse numbers universally
      int i0 = idx[0] >= idx[1]? 0: 1;
      int i2 = idx[2] >  idx[3]? 2: 3;
      const int fmts[4] = {FMT_HEX, FMT_BIN, FMT_OCT, FMT_DEC };

      return fmts[idx[i0] >= idx[i2]? i0: i2];
    }
  }

  return FMT_ERROR;
}

int fileio_fmt_autodetect_fp(FILE *f) {
  int ret = FMT_ERROR;

  if(f) {
    unsigned char *buf;
    for(int first = 1; ret == FMT_ERROR && (buf = (unsigned char *) Nfgets(f)); first = 0) {
      ret = couldbe(first, buf);
      free(buf);
    }
  }

  return ret;
}

int fileio_fmt_autodetect(const char *fname) {
  FILE *f = fileio_fopenr(fname);

  if (f == NULL) {
    pmsg_ext_error("unable to open %s: %s\n", fname, strerror(errno));
    return -1;
  }

  int format = fileio_fmt_autodetect_fp(f);
  fclose(f);

  return format;
}



int fileio(int oprwv, const char *filename, FILEFMT format,
      const AVRPART *p, const char *memtype, int size)
{
  int op, rc;
  FILE * f;
  const char *fname;
  struct fioparms fio;
  AVRMEM * mem;
  int using_stdio;

  op = oprwv == FIO_READ_FOR_VERIFY? FIO_READ: oprwv;
  mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    pmsg_error("memory type %s not configured for device %s\n", memtype, p->desc);
    return -1;
  }

  rc = fileio_setparms(op, &fio, p, mem);
  if (rc < 0)
    return -1;

  if (size < 0 || fio.op == FIO_READ)
    size = mem->size;

  if (fio.op == FIO_READ) {
    /* 0xff fill unspecified memory */
    memset(mem->buf, 0xff, size);
  }
  memset(mem->tags, 0, size);

  using_stdio = 0;

  if (strcmp(filename, "-")==0) {
    if (fio.op == FIO_READ) {
      fname = "<stdin>";
      f = stdin;
    }
    else {
      fname = "<stdout>";
      f = stdout;
    }
    using_stdio = 1;
  }
  else {
    fname = filename;
    f = NULL;
  }

  if (format == FMT_AUTO) {
    int format_detect;

    if (using_stdio) {
      pmsg_error("cannot auto detect file format when using stdin/out\n");
      imsg_error("please specify a file format and try again\n");
      return -1;
    }

    format_detect = fileio_fmt_autodetect(fname);
    if (format_detect < 0) {
      pmsg_error("cannot determine file format for %s, specify explicitly\n", fname);
      return -1;
    }
    format = format_detect;

    if (quell_progress < 2)
      pmsg_notice("%s file %s auto detected as %s\n",
        fio.iodesc, fname, fileio_fmtstr(format));
  }

#if defined(WIN32)
  /* Open Raw Binary and ELF format in binary mode on Windows.*/
  if(format == FMT_RBIN || format == FMT_ELF)
  {
      if(fio.op == FIO_READ)
      {
          fio.mode = "rb";
      }
      if(fio.op == FIO_WRITE)
      {
          fio.mode = "wb";
      }
  }
#endif

  if (format != FMT_IMM) {
    if (!using_stdio) {
      f = fopen(fname, fio.mode);
      if (f == NULL) {
        pmsg_ext_error("cannot open %s file %s: %s\n", fio.iodesc, fname, strerror(errno));
        return -1;
      }
    }
  }

  switch (format) {
    case FMT_IHEX:
    case FMT_IHXC:
      rc = fileio_ihex(&fio, fname, f, mem, size, format);
      break;

    case FMT_SREC:
      rc = fileio_srec(&fio, fname, f, mem, size);
      break;

    case FMT_RBIN:
      rc = fileio_rbin(&fio, fname, f, mem, size);
      break;

    case FMT_ELF:
#ifdef HAVE_LIBELF
      rc = fileio_elf(&fio, fname, f, mem, p, size);
#else
      pmsg_error("cannot handle ELF file %s, ELF file support was not compiled in\n", fname);
      rc = -1;
#endif
      break;

    case FMT_IMM:
      rc = fileio_imm(&fio, fname, f, mem, size);
      break;

    case FMT_HEX:
    case FMT_DEC:
    case FMT_OCT:
    case FMT_BIN:
      rc = fileio_num(&fio, fname, f, mem, size, format);
      break;

    default:
      pmsg_error("invalid %s file format: %d\n", fio.iodesc, format);
      return -1;
  }

  /* on reading flash other than for verify set the size to location of highest non-0xff byte */
  if (rc > 0 && oprwv == FIO_READ) {
    int hiaddr = avr_mem_hiaddr(mem);

    if(hiaddr < rc)             /* if trailing-0xff not disabled */
      rc = hiaddr;
  }

  if (format != FMT_IMM && !using_stdio) {
    fclose(f);
  }

  return rc;
}

