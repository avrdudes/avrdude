/*
 * avrdude - A Uploader/Downloader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "avr.h"
#include "fileio.h"


#define IHEX_MAXDATA 256

#define MAX_LINE_LEN 256  /* max line length for ASCII format input files */


struct ihexrec {
  unsigned char    reclen;
  unsigned short   loadofs;
  unsigned char    rectyp;
  unsigned char    data[IHEX_MAXDATA];
  unsigned char    cksum;
};


extern char * progname;
extern char   progbuf[];

int b2ihex(unsigned char * inbuf, int bufsize, 
             int recsize, int startaddr,
             char * outfile, FILE * outf);

int ihex2b(char * infile, FILE * inf,
             unsigned char * outbuf, int bufsize);

int fileio_rbin(struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size);

int fileio_ihex(struct fioparms * fio, 
                  char * filename, FILE * f, unsigned char * buf, int size);

int fileio_srec(struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size);

int fmt_autodetect(char * fname);



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



int b2ihex(unsigned char * inbuf, int bufsize, 
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


int ihex_readrec(struct ihexrec * ihex, char * rec)
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

int ihex2b(char * infile, FILE * inf,
             unsigned char * outbuf, int bufsize)
{
  char buffer [ MAX_LINE_LEN ];
  unsigned char * buf;
  unsigned int nextaddr, baseaddr, maxaddr;
  int i;
  int lineno;
  int len;
  struct ihexrec ihex;
  int rc;

  lineno   = 0;
  buf      = outbuf;
  baseaddr = 0;
  maxaddr  = 0;

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
        if (nextaddr + ihex.reclen > bufsize) {
          fprintf(stderr, 
                  "%s: ERROR: address 0x%04x out of range at line %d of %s\n",
                  progname, nextaddr+ihex.reclen, lineno, infile);
          return -1;
        }
        for (i=0; i<ihex.reclen; i++) {
          buf[nextaddr+i] = ihex.data[i];
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

  return maxaddr;
}



int fileio_rbin(struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size)
{
  int rc;

  switch (fio->op) {
    case FIO_READ:
      rc = fread(buf, 1, size, f);
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


int fileio_ihex(struct fioparms * fio, 
                  char * filename, FILE * f, unsigned char * buf, int size)
{
  int rc;

  switch (fio->op) {
    case FIO_WRITE:
      rc = b2ihex(buf, size, 32, 0, filename, f);
      if (rc < 0) {
        return -1;
      }
      break;

    case FIO_READ:
      rc = ihex2b(filename, f, buf, size);
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


int fileio_srec(struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size)
{
  fprintf(stderr, "%s: Motorola S-Record %s format not yet supported\n",
          progname, fio->iodesc);
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



int fmt_autodetect(char * fname)
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
    if (found)
      return FMT_RBIN;

    /* check for lines that look like intel hex */
    if ((buf[0] == ':') && (len >= 11)) {
      found = 1;
      for (i=1; i<len; i++) {
        if (!isxdigit(buf[1])) {
          found = 0;
          break;
        }
      }
      if (found)
        return FMT_IHEX;
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
      if (found)
        return FMT_SREC;
    }
  }

  return -1;
}



int fileio(int op, char * filename, FILEFMT format, 
             struct avrpart * p, char * memtype, int size)
{
  int rc;
  FILE * f;
  char * fname;
  unsigned char * buf;
  struct fioparms fio;
  int i;
  AVRMEM * mem;

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

  if (strcmp(filename, "-")==0) {
    if (fio.op == FIO_READ) {
      fname = "<stdin>";
      f = stdin;
    }
    else {
      fname = "<stdout>";
      f = stdout;
    }
  }
  else {
    fname = filename;
    f = fopen(fname, fio.mode);
    if (f == NULL) {
      fprintf(stderr, "%s: can't open %s file %s: %s\n",
              progname, fio.iodesc, fname, strerror(errno));
      return -1;
    }
  }

  /* point at the requested memory buffer */
  buf = mem->buf;
  if (fio.op == FIO_READ)
    size = mem->size;

  if (fio.op == FIO_READ) {
    /* 0xff fill unspecified memory */
    for (i=0; i<size; i++) {
      buf[i] = 0xff;
    }
  }

  if (format == FMT_AUTO) {
    format = fmt_autodetect(fname);
    if (format < 0) {
      fprintf(stderr, 
              "%s: can't determine file format for %s, specify explicitly\n",
              progname, fname);
      return -1;
    }

    fprintf(stderr, "%s: %s file %s auto detected as %s\n", 
            progname, fio.iodesc, fname, fmtstr(format));
  }

  switch (format) {
    case FMT_IHEX:
      rc = fileio_ihex(&fio, fname, f, buf, size);
      break;

    case FMT_SREC:
      rc = fileio_srec(&fio, fname, f, buf, size);
      break;

    case FMT_RBIN:
      rc = fileio_rbin(&fio, fname, f, buf, size);
      break;

    default:
      fprintf(stderr, "%s: invalid %s file format: %d\n",
              progname, fio.iodesc, format);
      return -1;
  }

  return rc;
}

