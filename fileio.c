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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */

#include "ac_cfg.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "avrdude.h"
#include "avr.h"
#include "fileio.h"


#define IHEX_MAXDATA 256

#define MAX_LINE_LEN 256  /* max line length for ASCII format input files */


struct ihexrec {
  unsigned char    reclen;
  unsigned int     loadofs;
  unsigned char    rectyp;
  unsigned char    data[IHEX_MAXDATA];
  unsigned char    cksum;
};


static int b2ihex(unsigned char * inbuf, int bufsize, 
             int recsize, int startaddr,
             char * outfile, FILE * outf);

static int ihex2b(char * infile, FILE * inf,
             AVRMEM * mem, int bufsize);

static int b2srec(unsigned char * inbuf, int bufsize, 
           int recsize, int startaddr,
           char * outfile, FILE * outf);

static int srec2b(char * infile, FILE * inf,
             AVRMEM * mem, int bufsize);

static int ihex_readrec(struct ihexrec * ihex, char * rec);

static int srec_readrec(struct ihexrec * srec, char * rec);

static int fileio_rbin(struct fioparms * fio,
                  char * filename, FILE * f, AVRMEM * mem, int size);

static int fileio_ihex(struct fioparms * fio, 
                  char * filename, FILE * f, AVRMEM * mem, int size);

static int fileio_srec(struct fioparms * fio,
                  char * filename, FILE * f, AVRMEM * mem, int size);

static int fileio_num(struct fioparms * fio,
		char * filename, FILE * f, AVRMEM * mem, int size,
		FILEFMT fmt);

static int fmt_autodetect(char * fname);



char * fmtstr(FILEFMT format)
{
  switch (format) {
    case FMT_AUTO : return "auto-detect"; break;
    case FMT_SREC : return "Motorola S-Record"; break;
    case FMT_IHEX : return "Intel Hex"; break;
    case FMT_RBIN : return "raw binary"; break;
    default       : return "invalid format"; break;
  };
}



static int b2ihex(unsigned char * inbuf, int bufsize, 
           int recsize, int startaddr,
           char * outfile, FILE * outf)
{
  unsigned char * buf;
  unsigned int nextaddr;
  int n, nbytes, n_64k;
  int i;
  unsigned char cksum;

  if (recsize > 255) {
    fprintf(stderr, "%s: recsize=%d, must be < 256\n",
              progname, recsize);
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
      fprintf(outf, "%02X\n", cksum);
      
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
 * Given an open file 'inf' which contains Intel Hex formated data,
 * parse the file and lay it out within the memory buffer pointed to
 * by outbuf.  The size of outbuf, 'bufsize' is honored; if data would
 * fall outsize of the memory buffer outbuf, an error is generated.
 *
 * Return the maximum memory address within 'outbuf' that was written.
 * If an error occurs, return -1.
 *
 * */
static int ihex2b(char * infile, FILE * inf,
             AVRMEM * mem, int bufsize)
{
  char buffer [ MAX_LINE_LEN ];
  unsigned int nextaddr, baseaddr, maxaddr, offsetaddr;
  int i;
  int lineno;
  int len;
  struct ihexrec ihex;
  int rc;

  lineno      = 0;
  baseaddr    = 0;
  maxaddr     = 0;
  offsetaddr  = 0;
  nextaddr    = 0;

  while (fgets((char *)buffer,MAX_LINE_LEN,inf)!=NULL) {
    lineno++;
    len = strlen(buffer);
    if (buffer[len-1] == '\n') 
      buffer[--len] = 0;
    if (buffer[0] != ':')
      continue;
    rc = ihex_readrec(&ihex, buffer);
    if (rc < 0) {
      fprintf(stderr, "%s: invalid record at line %d of \"%s\"\n",
              progname, lineno, infile);
      return -1;
    }
    else if (rc != ihex.cksum) {
      fprintf(stderr, "%s: ERROR: checksum mismatch at line %d of \"%s\"\n",
              progname, lineno, infile);
      fprintf(stderr, "%s: checksum=0x%02x, computed checksum=0x%02x\n",
              progname, ihex.cksum, rc);
      return -1;
    }

    switch (ihex.rectyp) {
      case 0: /* data record */
        nextaddr = ihex.loadofs + baseaddr;
        if ((nextaddr + ihex.reclen) > (bufsize+offsetaddr)) {
          fprintf(stderr, 
                  "%s: ERROR: address 0x%04x out of range at line %d of %s\n",
                  progname, nextaddr+ihex.reclen, lineno, infile);
          return -1;
        }
        for (i=0; i<ihex.reclen; i++) {
          mem->buf[nextaddr+i-offsetaddr] = ihex.data[i];
          mem->tags[nextaddr+i-offsetaddr] = TAG_ALLOCATED;
        }
        if (nextaddr+ihex.reclen > maxaddr)
          maxaddr = nextaddr+ihex.reclen;
        break;

      case 1: /* end of file record */
        return maxaddr-offsetaddr;
        break;

      case 2: /* extended segment address record */
        baseaddr = (ihex.data[0] << 8 | ihex.data[1]) << 4;
        break;

      case 3: /* start segment address record */
        /* we don't do anything with the start address */
        break;

      case 4: /* extended linear address record */
        baseaddr = (ihex.data[0] << 8 | ihex.data[1]) << 16;
        if(nextaddr == 0) offsetaddr = baseaddr;	// if provided before any data, then remember it
        break;

      case 5: /* start linear address record */
        /* we don't do anything with the start address */
        break;

      default:
        fprintf(stderr, 
                "%s: don't know how to deal with rectype=%d " 
                "at line %d of %s\n",
                progname, ihex.rectyp, lineno, infile);
        return -1;
        break;
    }

  } /* while */

  fprintf(stderr, 
          "%s: WARNING: no end of file record found for Intel Hex "
          "file \"%s\"\n",
          progname, infile);

  return maxaddr-offsetaddr;
}

static int b2srec(unsigned char * inbuf, int bufsize, 
           int recsize, int startaddr,
           char * outfile, FILE * outf)
{
  unsigned char * buf;
  unsigned int nextaddr;
  int n, nbytes, addr_width;
  int i;
  unsigned char cksum;

  char * tmpl=0;

  if (recsize > 255) {
    fprintf(stderr, "%s: ERROR: recsize=%d, must be < 256\n",
            progname, recsize);
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
        fprintf(stderr, "%s: ERROR: address=%d, out of range\n",
                progname, nextaddr);
        return -1;
      }

      fprintf(outf, tmpl, n + addr_width + 1, nextaddr);		

      cksum += n + addr_width + 1;

      for (i=addr_width; i>0; i--) 
        cksum += (nextaddr >> (i-1) * 8) & 0xff;

      for (i=nextaddr; i<nextaddr + n; i++) {
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
  else if (startaddr <= 0xffffffff) {
    addr_width = 4;
    tmpl="S9%02X%08X";
  }

  fprintf(outf, tmpl, n + addr_width + 1, nextaddr);

  cksum += n + addr_width +1;
  for (i=addr_width; i>0; i--) 
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
    addr_width = 3;	/* S2,S8-record */
  else if (srec->rectyp == 0x33 || srec->rectyp == 0x37) 
    addr_width = 4;	/* S3,S7-record */

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


static int srec2b(char * infile, FILE * inf,
           AVRMEM * mem, int bufsize)
{
  char buffer [ MAX_LINE_LEN ];
  unsigned int nextaddr, baseaddr, maxaddr;
  int i;
  int lineno;
  int len;
  struct ihexrec srec;
  int rc;
  int reccount;
  unsigned char datarec;

  char * msg = 0;

  lineno   = 0;
  baseaddr = 0;
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
      fprintf(stderr, "%s: ERROR: invalid record at line %d of \"%s\"\n",
              progname, lineno, infile);
      return -1;
    }
    else if (rc != srec.cksum) {
      fprintf(stderr, "%s: ERROR: checksum mismatch at line %d of \"%s\"\n",
              progname, lineno, infile);
      fprintf(stderr, "%s: checksum=0x%02x, computed checksum=0x%02x\n",
              progname, srec.cksum, rc);
      return -1;
    }

    datarec=0;	
    switch (srec.rectyp) {
      case 0x30: /* S0 - header record*/
        /* skip */
        break;

      case 0x31: /* S1 - 16 bit address data record */
        datarec=1;
        msg="%s: ERROR: address 0x%04x out of range at line %d of %s\n";    
        break;

      case 0x32: /* S2 - 24 bit address data record */
        datarec=1;
        msg="%s: ERROR: address 0x%06x out of range at line %d of %s\n";
        break;

      case 0x33: /* S3 - 32 bit address data record */
        datarec=1;
        msg="%s: ERROR: address 0x%08x out of range at line %d of %s\n";
        break;

      case 0x34: /* S4 - symbol record (LSI extension) */
        fprintf(stderr, 
                "%s: ERROR: not supported record at line %d of %s\n",
                progname, lineno, infile);
        return -1;

      case 0x35: /* S5 - count of S1,S2 and S3 records previously tx'd */
        if (srec.loadofs != reccount){
          fprintf(stderr, 
                  "%s: ERROR: count of transmitted data records mismatch "
                  "at line %d of \"%s\"\n",
                  progname, lineno, infile);
          fprintf(stderr, "%s: transmitted data records= %d, expected "
                  "value= %d\n",
                  progname, reccount, srec.loadofs);
          return -1;
        }
        break;

      case 0x37: /* S7 Record - end record for 32 bit address data */
      case 0x38: /* S8 Record - end record for 24 bit address data */
      case 0x39: /* S9 Record - end record for 16 bit address data */
        return maxaddr;

      default:
        fprintf(stderr, 
                "%s: ERROR: don't know how to deal with rectype S%d " 
                "at line %d of %s\n",
                progname, srec.rectyp, lineno, infile);
        return -1;
    }

    if (datarec == 1) {
      nextaddr = srec.loadofs + baseaddr;
      if (nextaddr + srec.reclen > bufsize) {
        fprintf(stderr, msg, progname, nextaddr+srec.reclen, lineno, infile);
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

  fprintf(stderr, 
          "%s: WARNING: no end of file record found for Motorola S-Records "
          "file \"%s\"\n",
          progname, infile);

  return maxaddr;
}

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



static int fileio_rbin(struct fioparms * fio,
                  char * filename, FILE * f, AVRMEM * mem, int size)
{
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
      fprintf(stderr, "%s: fileio: invalid operation=%d\n",
              progname, fio->op);
      return -1;
  }

  if (rc < 0 || (fio->op == FIO_WRITE && rc < size)) {
    fprintf(stderr, 
            "%s: %s error %s %s: %s; %s %d of the expected %d bytes\n", 
            progname, fio->iodesc, fio->dir, filename, strerror(errno),
            fio->rw, rc, size);
    return -1;
  }

  return rc;
}


static int fileio_imm(struct fioparms * fio,
               char * filename, FILE * f, AVRMEM * mem, int size)
{
  int rc = 0;
  char * e, * p;
  unsigned long b;
  int loc;

  switch (fio->op) {
    case FIO_READ:
      loc = 0;
      p = strtok(filename, " ,");
      while (p != NULL && loc < size) {
        b = strtoul(p, &e, 0);
	/* check for binary formated (0b10101001) strings */
	b = (strncmp (p, "0b", 2))?
	    strtoul (p, &e, 0):
	    strtoul (p + 2, &e, 2);
        if (*e != 0) {
          fprintf(stderr,
                  "%s: invalid byte value (%s) specified for immediate mode\n",
                  progname, p);
          return -1;
        }
        mem->buf[loc] = b;
        mem->tags[loc++] = TAG_ALLOCATED;
        p = strtok(NULL, " ,");
        rc = loc;
      }
      break;
    default:
      fprintf(stderr, "%s: fileio: invalid operation=%d\n",
              progname, fio->op);
      return -1;
  }

  if (rc < 0 || (fio->op == FIO_WRITE && rc < size)) {
    fprintf(stderr, 
            "%s: %s error %s %s: %s; %s %d of the expected %d bytes\n", 
            progname, fio->iodesc, fio->dir, filename, strerror(errno),
            fio->rw, rc, size);
    return -1;
  }

  return rc;
}


static int fileio_ihex(struct fioparms * fio, 
                  char * filename, FILE * f, AVRMEM * mem, int size)
{
  int rc;

  switch (fio->op) {
    case FIO_WRITE:
      rc = b2ihex(mem->buf, size, 32, 0, filename, f);
      if (rc < 0) {
        return -1;
      }
      break;

    case FIO_READ:
      rc = ihex2b(filename, f, mem, size);
      if (rc < 0)
        return -1;
      break;

    default:
      fprintf(stderr, "%s: invalid Intex Hex file I/O operation=%d\n",
              progname, fio->op);
      return -1;
      break;
  }

  return rc;
}


static int fileio_srec(struct fioparms * fio,
                  char * filename, FILE * f, AVRMEM * mem, int size)
{
  int rc;

  switch (fio->op) {
    case FIO_WRITE:
      rc = b2srec(mem->buf, size, 32, 0, filename, f);
      if (rc < 0) {
        return -1;
      }
      break;

    case FIO_READ:
      rc = srec2b(filename, f, mem, size);
      if (rc < 0)
        return -1;
      break;

    default:
      fprintf(stderr, "%s: ERROR: invalid Motorola S-Records file I/O "
              "operation=%d\n",
              progname, fio->op);
      return -1;
      break;
  }

  return rc;
}


static int fileio_num(struct fioparms * fio,
	       char * filename, FILE * f, AVRMEM * mem, int size,
	       FILEFMT fmt)
{
  const char *prefix;
  char cbuf[20];
  int base, i, num;

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

  switch (fio->op) {
    case FIO_WRITE:
      break;
    default:
      fprintf(stderr, "%s: fileio: invalid operation=%d\n",
              progname, fio->op);
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
  fprintf(stderr, "%s: error writing to %s: %s\n",
	  progname, filename, strerror(errno));
  return -1;
}


int fileio_setparms(int op, struct fioparms * fp)
{
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
      fprintf(stderr, "%s: invalid I/O operation %d\n",
              progname, op);
      return -1;
      break;
  }

  return 0;
}



static int fmt_autodetect(char * fname)
{
  FILE * f;
  unsigned char buf[MAX_LINE_LEN];
  int i;
  int len;
  int found;

  f = fopen(fname, "r");
  if (f == NULL) {
    fprintf(stderr, "%s: error opening %s: %s\n",
            progname, fname, strerror(errno));
    return -1;
  }

  while (fgets((char *)buf, MAX_LINE_LEN, f)!=NULL) {
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
  }

  fclose(f);
  return -1;
}



int fileio(int op, char * filename, FILEFMT format, 
             struct avrpart * p, char * memtype, int size)
{
  int rc;
  FILE * f;
  char * fname;
  struct fioparms fio;
  AVRMEM * mem;
  int using_stdio;

  mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    fprintf(stderr, 
            "fileio(): memory type \"%s\" not configured for device \"%s\"\n",
            memtype, p->desc);
    return -1;
  }

  rc = fileio_setparms(op, &fio);
  if (rc < 0)
    return -1;

  if (fio.op == FIO_READ)
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
    if (using_stdio) {
      fprintf(stderr, 
              "%s: can't auto detect file format when using stdin/out.\n"
              "     Please specify a file format using the -f option and try again.\n", 
              progname);
      exit(1);
    }

    format = fmt_autodetect(fname);
    if (format < 0) {
      fprintf(stderr, 
              "%s: can't determine file format for %s, specify explicitly\n",
              progname, fname);
      return -1;
    }

    if (quell_progress < 2) {
      fprintf(stderr, "%s: %s file %s auto detected as %s\n", 
              progname, fio.iodesc, fname, fmtstr(format));
    }
  }

#if defined(WIN32NATIVE)
  /* Open Raw Binary format in binary mode on Windows.*/
  if(format == FMT_RBIN)
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
        fprintf(stderr, "%s: can't open %s file %s: %s\n",
                progname, fio.iodesc, fname, strerror(errno));
        return -1;
      }
    }
  }

  switch (format) {
    case FMT_IHEX:
      rc = fileio_ihex(&fio, fname, f, mem, size);
      break;

    case FMT_SREC:
      rc = fileio_srec(&fio, fname, f, mem, size);
      break;

    case FMT_RBIN:
      rc = fileio_rbin(&fio, fname, f, mem, size);
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
      fprintf(stderr, "%s: invalid %s file format: %d\n",
              progname, fio.iodesc, format);
      return -1;
  }

  if (rc > 0) {
    if ((op == FIO_READ) && (strcasecmp(mem->desc, "flash") == 0)) {
      /*
       * if we are reading flash, just mark the size as being the
       * highest non-0xff byte
       */
      rc = avr_mem_hiaddr(mem);
    }
  }
  if (format != FMT_IMM && !using_stdio) {
    fclose(f);
  }

  return rc;
}

