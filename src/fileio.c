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

#ifdef HAVE_LIBELF
#define MIN_LOCK_SIZE 1
#define MAX_LOCK_SIZE 4
#define MAX_FUSE_SIZE 10
#define MAX_CMD_SIZE  (PATH_MAX + 32)

typedef struct elf_cmd {
  int elf_fuse_size;
  unsigned char * elf_fuse_data;

  int elf_lock_size;
  unsigned char * elf_lock_data;

  int elf_eeprom_size;
  char * elf_eeprom_filename;

  int elf_all_write;
} ELF_CMD;
#endif

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


char * fileio_fmtstr(FILEFMT format)
{
  switch (format) {
    case FMT_AUTO : return "auto-detect"; break;
    case FMT_SREC : return "Motorola S-Record"; break;
    case FMT_IHEX : return "Intel Hex"; break;
    case FMT_IHXC : return "Intel Hex with comments"; break;
    case FMT_RBIN : return "raw binary"; break;
    case FMT_ELF  : return "ELF"; break;
    default       : return "invalid format"; break;
  };
}


static int b2ihex(const unsigned char *inbuf, int bufsize,
           int recsize, int startaddr,
           const char *outfile, FILE *outf, FILEFMT ffmt)
{
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
 * Return the maximum memory address within 'outbuf' that was written.
 * If an error occurs, return -1.
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

static int b2srec(const unsigned char *inbuf, int bufsize,
           int recsize, int startaddr,
           const char *outfile, FILE *outf)
{
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

/*
 * Return the ELF section descriptor that corresponds to program
 * header `ph'.  The program header is expected to be of p_type
 * PT_LOAD, and to have a nonzero p_filesz.  (PT_LOAD sections with a
 * zero p_filesz are typically RAM sections that are not initialized
 * by file data, e.g. ".bss".)
 */
static Elf_Scn *elf_get_scn(Elf *e, Elf32_Phdr *ph, Elf32_Shdr **shptr)
{
  Elf_Scn *s = NULL;

  while ((s = elf_nextscn(e, s)) != NULL) {
    Elf32_Shdr *sh;
    size_t ndx = elf_ndxscn(s);
    if ((sh = elf32_getshdr(s)) == NULL) {
      pmsg_error("unable to read section #%u header: %s\n", (unsigned int)ndx, elf_errmsg(-1));
      continue;
    }
    if ((sh->sh_flags & SHF_ALLOC) == 0 ||
        sh->sh_type != SHT_PROGBITS)
      /* we are only interested in PROGBITS, ALLOC sections */
      continue;
    if (sh->sh_size == 0)
      /* we are not interested in empty sections */
      continue;
    if (is_section_in_segment(sh, ph)) {
      /* yeah, we found it */
      *shptr = sh;
      return s;
    }
  }

  pmsg_error("cannot find a matching section for program header entry @p_vaddr 0x%x\n", ph->p_vaddr);
  return NULL;
}

static int elf_mem_limits(const AVRMEM *mem, const AVRPART *p,
                          unsigned int *lowbound,
                          unsigned int *highbound,
                          unsigned int *fileoff,
                          unsigned int *is_flash)
{
  int rv = 0;

  *is_flash = 0;
  if (p->prog_modes & PM_aWire) { // AVR32
    if (strcmp(mem->desc, "flash") == 0) {
      *lowbound = 0x80000000;
      *highbound = 0xffffffff;
      *fileoff = 0;
      *is_flash = 1;
    } else {
      rv = -1;
    }
  } else {
    if (strcmp(mem->desc, "flash") == 0 ||
        strcmp(mem->desc, "boot") == 0 ||
        strcmp(mem->desc, "application") == 0 ||
        strcmp(mem->desc, "apptable") == 0) {
      *lowbound = 0;
      *highbound = 0x7ffff;       /* max 8 MiB */
      *fileoff = 0;
      *is_flash = 1;
    } else if (strcmp(mem->desc, "eeprom") == 0) {
      *lowbound = 0x810000;
      *highbound = 0x81ffff;      /* max 64 KiB */
      *fileoff = 0;
    } else if (strcmp(mem->desc, "lfuse") == 0) {
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
    } else if (strcmp(mem->desc, "fuse") == 0) {
      /* ATtiny4/5/9/10/20/40 use section .config in ELF */
      *lowbound = 0x820000;
      *highbound = 0x82ffff;
      *fileoff = 0;
    } else if (strncmp(mem->desc, "lock", 4) == 0) {
      *lowbound = 0x830000;
      *highbound = 0x83ffff;
      *fileoff = 0;
    } else {
      rv = -1;
    }
  }

  return rv;
}

extern LISTID updates;

/*  Example for AVR C code.
For Atmega8:

FUSES = { 0xC9, 0xEF };
LOCKBITS = 0xFC;
// Fix add .eeprom section
#define _EEMEM  __attribute__((__used__, __section__ (".eeprom")))
#define _EEPROM uint8_t __eeprom[512] _EEMEM

_EEPROM = { 0x11, 0x22, };

For ATtiny4/5/9/10/20/40 (FUSES macro and .fuse section not supported by avr-libc):

#define _FUSEMEM __attribute__((__used__, __section__(".config")))
#define FUSES uint8_t __fuses _FUSEMEM

FUSES = 0xFE;
LOCKBITS = 0xFC;
*/

int file_is_elf(const char *filename)
{
  FILE * f;
  unsigned char buf[MAX_LINE_LEN];

#if defined(WIN32)
  f = fopen(filename, "rb");
#else
  f = fopen(filename, "r");
#endif
  if (f == NULL)
    return 0;

  while (fgets((char *)buf, MAX_LINE_LEN, f)!=NULL) {
    /* check for ELF file */
    if (buf[0] == 0177 && buf[1] == 'E' &&
         buf[2] == 'L' && buf[3] == 'F') {
      fclose(f);
      return 1;
    }
  }
  return 0;
}

static void cmd_from_elf(ELF_CMD *lcmd, const AVRPART *p)
{
  char * cmd;
  LNODEID ln;
  AVRMEM * m;
  unsigned int elf_prev_mem_offset = 0;
  int num = 0, elf_prev_mem_size = 0;
  UPDATE * upd = NULL;

  if (!lcmd->elf_all_write)
    return;

  cmd = cfg_malloc(__func__, MAX_CMD_SIZE);
  if (!cmd)
    return;

  if (lcmd->elf_eeprom_filename) {
    for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
      if ((m = ldata(ln)) && lcmd->elf_eeprom_size) {
        if (elf_prev_mem_offset != m->offset || elf_prev_mem_size != m->size || (strcmp(p->family_id, "") == 0)) {
          elf_prev_mem_offset = m->offset;
          elf_prev_mem_size = m->size;
          if (avr_memtype_is_eeprom_type(m->desc)) {
            if (lcmd->elf_eeprom_size <= m->size) {
              pmsg_info("found in ELF %s, size = %d\n", m->desc, lcmd->elf_eeprom_size);
              memset(cmd, 0, MAX_CMD_SIZE);
              snprintf(cmd, MAX_CMD_SIZE, "%s:w:%s:a", m->desc, lcmd->elf_eeprom_filename);
              if ((upd = parse_op(cmd))) {
                upd->elf_all_write = 1;
                ladd(updates, upd);
              }
              break;
            }
          }
        }
      }
    }
  }

  for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
    if ((m = ldata(ln)) && lcmd->elf_fuse_size) {
      if (elf_prev_mem_offset != m->offset || elf_prev_mem_size != m->size || (strcmp(p->family_id, "") == 0)) {
        elf_prev_mem_offset = m->offset;
        elf_prev_mem_size = m->size;
        if (sscanf(m->desc, "fuse%d", &num) == 1) {
          if (num <= lcmd->elf_fuse_size) {
            pmsg_info("found in ELF %s = 0x%02X\n", m->desc, lcmd->elf_fuse_data[num]);
            memset(cmd, 0, MAX_CMD_SIZE);
            snprintf(cmd, MAX_CMD_SIZE,"%s:w:0x%02X:m", m->desc, lcmd->elf_fuse_data[num]);
            if ((upd = parse_op(cmd))) {
              upd->elf_all_write = 1;
              ladd(updates, upd);
            }
          }
        } else if (strstr(m->desc, "fuse")) {
          int b = -1;
          if (!strcmp(m->desc, "fuse") || !strcmp(m->desc, "lfuse")) {
            if (lcmd->elf_fuse_size >= 1)
              b = 0;
          } else if (!strcmp(m->desc, "hfuse")) {
            if (lcmd->elf_fuse_size >= 2)
              b = 1;
          } else if (!strcmp(m->desc, "efuse")) {
            if (lcmd->elf_fuse_size == 3)
              b = 2;
          }
          if (b >= 0) {
            pmsg_info("found in ELF %s = 0x%02X\n", m->desc, lcmd->elf_fuse_data[b]);
            memset(cmd, 0, MAX_CMD_SIZE);
            snprintf(cmd, MAX_CMD_SIZE,"%s:w:0x%02X:m", m->desc, lcmd->elf_fuse_data[b]);
            if ((upd = parse_op(cmd))) {
              upd->elf_all_write = 1;
              ladd(updates, upd);
            }
          }
        }
      }
    }
  }

  if (lcmd->elf_lock_size) {
    memset(cmd, 0, MAX_CMD_SIZE);
    if (lcmd->elf_lock_size == MIN_LOCK_SIZE) {
      pmsg_info("found in ELF lockbits = 0x%02X\n", lcmd->elf_lock_data[0]);
      snprintf(cmd, MAX_CMD_SIZE,"lock:w:0x%02X:m", lcmd->elf_lock_data[0]);
      if ((upd = parse_op(cmd))) {
        upd->elf_all_write = 1;
        ladd(updates, upd);
      }
    } else if (lcmd->elf_lock_size == MAX_LOCK_SIZE) {
      pmsg_info("found in ELF lockbits = 0x%02X,0x%02X,0x%02X,0x%02X\n", lcmd->elf_lock_data[0],
                lcmd->elf_lock_data[1], lcmd->elf_lock_data[2], lcmd->elf_lock_data[3]);
      snprintf(cmd, MAX_CMD_SIZE,"lock:w:0x%02X,0x%02X,0x%02X,0x%02X:m", lcmd->elf_lock_data[0],
                lcmd->elf_lock_data[1], lcmd->elf_lock_data[2], lcmd->elf_lock_data[3]);
      if ((upd = parse_op(cmd))) {
        upd->elf_all_write = 1;
        ladd(updates, upd);
      }
    }
  }

  free(cmd);
}

static int elf2b(const char *infile, FILE *inf,
                 const AVRMEM *mem, const AVRPART *p,
                 int bufsize, unsigned int fileoffset)
{
  Elf *e;
  int ign_chk = 0, rv = -1;
  unsigned int low, high, foff, isfl;
  ELF_CMD l_cmd;
  memset(&l_cmd, 0, sizeof(l_cmd));

  if (elf_mem_limits(mem, p, &low, &high, &foff, &isfl) != 0) {
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
    isfl = 1;
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

  l_cmd.elf_fuse_data = cfg_malloc(__func__, 32);
  l_cmd.elf_lock_data = cfg_malloc(__func__, 32);
  /*
   * Walk the program header table, pick up entries that are of type
   * PT_LOAD, and have a non-zero p_filesz.
   */
  for (i = 0; i < eh->e_phnum; i++) {
    if (ph[i].p_type != PT_LOAD ||
        ph[i].p_filesz == 0)
      continue;

    pmsg_notice2("considering PT_LOAD program header entry #%d:\n"
      "    p_vaddr 0x%x, p_paddr 0x%x, p_filesz %d\n", (int) i, ph[i].p_vaddr, ph[i].p_paddr, ph[i].p_filesz);

    Elf32_Shdr *sh;
    Elf_Scn *s = elf_get_scn(e, ph + i, &sh);
    if (s == NULL)
      continue;

    if ((sh->sh_flags & SHF_ALLOC) && sh->sh_size) {
      const char *sname;

      if (sndx != 0) {
        sname = elf_strptr(e, sndx, sh->sh_name);
      } else {
        sname = "*unknown*";
      }

      if (isfl && !strcmp(sname, ".text")) {
        LNODEID ln;
        UPDATE * upd = NULL;
        for (ln=lfirst(updates); ln; ln=lnext(ln)) {
          upd = ldata(ln);
          if (avr_memtype_is_flash_type(upd->memtype)) {
            l_cmd.elf_all_write = upd->elf_all_write;
            l_cmd.elf_eeprom_filename = upd->filename;
            break;
          }
        }
      }
      ign_chk = 0;
      if (l_cmd.elf_all_write && (!strcmp(sname, ".fuse") || (!strcmp(sname, ".config") ||
          !strcmp(sname, ".lock") || !strcmp(sname, ".eeprom")))) {
        ign_chk = sh->sh_size;
      }

      unsigned int lma;
      lma = ph[i].p_paddr + sh->sh_offset - ph[i].p_offset;

      pmsg_notice2("found section %s, LMA 0x%x, sh_size %u\n", sname, lma, sh->sh_size);

      if (lma >= low &&
          lma + sh->sh_size < high) {
        /* OK */
      } else if (!ign_chk) {
        msg_notice2("    => skipping, inappropriate for %s memory region\n", mem->desc);
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
        pmsg_error("section %s does not fit into %s memory:\n"
          "    0x%x + %u > %u\n", sname, mem->desc, lma, sh->sh_size, mem->size);
        continue;
      }

      Elf_Data *d = NULL;
      while ((d = elf_getdata(s, d)) != NULL) {

        if (!l_cmd.elf_all_write) {
          if (isfl && strcmp(sname, ".text") && strcmp(sname, ".data"))
            continue;
        } else {
          if (!strcmp(sname, ".fuse") || !strcmp(sname, ".config")) {
            if ((d->d_size > 0) && (d->d_size <= MAX_FUSE_SIZE)) {
              if (!l_cmd.elf_fuse_data)
                continue;
              l_cmd.elf_fuse_size = d->d_size;
              memcpy(l_cmd.elf_fuse_data, d->d_buf, d->d_size);
            }
            continue;
          } else if (!strcmp(sname, ".lock")) {
            if ((d->d_size == MIN_LOCK_SIZE) || (d->d_size == MAX_LOCK_SIZE)) {
              if (!l_cmd.elf_lock_data)
                continue;
              l_cmd.elf_lock_size = d->d_size;
              memcpy(l_cmd.elf_lock_data, d->d_buf, d->d_size);
            }
            continue;
          } else if (!strcmp(sname, ".eeprom")) {
            if (d->d_size > 0)
              l_cmd.elf_eeprom_size = d->d_size;
            continue;
          } else if (strcmp(sname, ".text") && strcmp(sname, ".data"))
            continue;
        }

        msg_notice2("    Data block: d_buf %p, d_off 0x%x, d_size %ld\n",
                        d->d_buf, (unsigned int)d->d_off, (long) d->d_size);
        if (mem->size == 1) {
          if (d->d_off != 0) {
            pmsg_error("unexpected data block at offset != 0\n");
          } else if (foff >= d->d_size) {
            pmsg_error("ELF file section does not contain byte at offset %d\n", foff);
          } else {
            msg_notice2("    Extracting one byte from file offset %d\n",
                            foff);
            mem->buf[0] = ((unsigned char *)d->d_buf)[foff];
            mem->tags[0] = TAG_ALLOCATED;
            rv = 1;
          }
        } else {
          unsigned int idx;

          idx = lma - low + d->d_off;
          if ((int)(idx + d->d_size) > rv)
            rv = idx + d->d_size;
          msg_debug("    Writing %ld bytes to mem offset 0x%x\n",
            (long) d->d_size, idx);
          memcpy(mem->buf + idx, d->d_buf, d->d_size);
          memset(mem->tags + idx, TAG_ALLOCATED, d->d_size);
        }
      }
    }
  }
  cmd_from_elf(&l_cmd, p);
  free(l_cmd.elf_fuse_data);
  free(l_cmd.elf_lock_data);
done:
  (void)elf_end(e);
  return rv;
}
#endif  /* HAVE_LIBELF */

/*
 * Simple itoa() implementation.  Caller needs to allocate enough
 * space in buf.  Only positive integers are handled.
 */
static char *itoa_simple(int n, char *buf, int base)
{
  div_t q;
  char c, *cp, *cp2;

  cp = buf;
  /*
   * Divide by base until the number disappeared, but ensure at least
   * one digit will be emitted.
   */
  do {
    q = div(n, base);
    n = q.quot;
    if (q.rem >= 10)
      c = q.rem - 10 + 'a';
    else
      c = q.rem + '0';
    *cp++ = c;
  } while (q.quot != 0);

  /* Terminate the string. */
  *cp-- = '\0';

  /* Now revert the result string. */
  cp2 = buf;
  while (cp > cp2) {
    c = *cp;
    *cp-- = *cp2;
    *cp2++ = c;
  }

  return buf;
}



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


static int fileio_imm(struct fioparms *fio,
             const char *fname, FILE *f, const AVRMEM *mem, int size)
{
  int rc = 0;
  char *e, *p, *filename;
  unsigned long b;
  int loc;

  filename = cfg_strdup(__func__, fname);

  switch (fio->op) {
    case FIO_READ:
      loc = 0;
      p = strtok(filename, " ,");
      while (p != NULL && loc < size) {
        b = strtoul(p, &e, 0);
        /* check for binary formatted (0b10101001) strings */
        b = (strncmp (p, "0b", 2))?
            strtoul (p, &e, 0):
           strtoul (p + 2, &e, 2);
        if (*e != 0) {
          pmsg_error("invalid byte value (%s) specified for immediate mode\n", p);
          free(filename);
          return -1;
        }
        mem->buf[loc] = b;
        mem->tags[loc++] = TAG_ALLOCATED;
        p = strtok(NULL, " ,");
        rc = loc;
      }
      break;

    case FIO_WRITE:
      pmsg_error("invalid file format 'immediate' for output\n");
      free(filename);
      return -1;

    default:
      pmsg_error("invalid operation=%d\n", fio->op);
      free(filename);
      return -1;
  }

  if (rc < 0 || (fio->op == FIO_WRITE && rc < size)) {
    pmsg_ext_error("%s error %s %s: %s; %s %d of the expected %d bytes\n",
      fio->iodesc, fio->dir, filename, strerror(errno), fio->rw, rc, size);
    free(filename);
    return -1;
  }

  free(filename);
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

static int fileio_num(struct fioparms *fio,
             const char *filename, FILE *f, const AVRMEM *mem, int size,
             FILEFMT fmt)
{
  const char *prefix;
  const char *name;
  char cbuf[20];
  int base, i, num;

  switch (fmt) {
    case FMT_HEX:
      name = "hex";
      prefix = "0x";
      base = 16;
      break;

    default:
    case FMT_DEC:
      name = "decimal";
      prefix = "";
      base = 10;
      break;

    case FMT_OCT:
      name = "octal";
      prefix = "0";
      base = 8;
      break;

    case FMT_BIN:
      name = "binary";
      prefix = "0b";
      base = 2;
      break;

  }

  switch (fio->op) {
    case FIO_WRITE:
      break;

    case FIO_READ:
      pmsg_error("invalid file format '%s' for input\n", name);
      return -1;

    default:
      pmsg_error("invalid operation=%d\n", fio->op);
      return -1;
  }

  for (i = 0; i < size; i++) {
    if (i > 0) {
      if (putc(',', f) == EOF)
        goto writeerr;
    }
    num = (unsigned int)(mem->buf[i]);
    /*
     * For a base of 8 and a value < 8 to convert, don't write the
     * prefix.  The conversion will be indistinguishable from a
     * decimal one then.
     */
    if (prefix[0] != '\0' && !(base == 8 && num < 8)) {
      if (fputs(prefix, f) == EOF)
        goto writeerr;
    }
    itoa_simple(num, cbuf, base);
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



int fileio_fmt_autodetect(const char * fname)
{
  FILE * f;
  unsigned char buf[MAX_LINE_LEN];
  int i;
  int len;
  int found;
  int first = 1;

#if defined(WIN32)
  f = fopen(fname, "rb");
#else
  f = fopen(fname, "r");
#endif
  if (f == NULL) {
    pmsg_ext_error("unable to open %s: %s\n", fname, strerror(errno));
    return -1;
  }

  while (fgets((char *)buf, MAX_LINE_LEN, f)!=NULL) {
    /* check for ELF file */
    if (first &&
        (buf[0] == 0177 && buf[1] == 'E' &&
         buf[2] == 'L' && buf[3] == 'F')) {
      fclose(f);
      return FMT_ELF;
    }

    buf[MAX_LINE_LEN-1] = 0;
    len = strlen((char *)buf);
    if (buf[len-1] == '\n')
      buf[--len] = 0;

    /* check for binary data */
    found = 0;
    for (i=0; i<len; i++) {
      if (buf[i] > 127) {
        found = 1;
        break;
      }
    }
    if (found) {
      fclose(f);
      return FMT_RBIN;
    }

    /* check for lines that look like intel hex */
    if ((buf[0] == ':') && (len >= 11)) {
      found = 1;
      for (i=1; i<len; i++) {
        if (!isxdigit(buf[1])) {
          found = 0;
          break;
        }
      }
      if (found) {
        fclose(f);
        return FMT_IHEX;
      }
    }

    /* check for lines that look like motorola s-record */
    if ((buf[0] == 'S') && (len >= 10) && isdigit(buf[1])) {
      found = 1;
      for (i=1; i<len; i++) {
        if (!isxdigit(buf[1])) {
          found = 0;
          break;
        }
      }
      if (found) {
        fclose(f);
        return FMT_SREC;
      }
    }

    first = 0;
  }

  fclose(f);
  return -1;
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

