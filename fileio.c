/*
 * Copyright 2000  Brian S. Dean <bsd@bsdhome.com>
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BRIAN S. DEAN ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BRIAN S. DEAN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * 
 */

/* $Id$ */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "avr.h"
#include "fileio.h"


extern char * progname;
extern char   progbuf[];


int b2ihex ( unsigned char * inbuf, int bufsize, 
             int recsize, int startaddr,
             char * outfile, FILE * outf );

int ihex2b ( char * infile, FILE * inf,
             unsigned char * outbuf, int bufsize );

int fileio_rbin ( struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size );

int fileio_ihex ( struct fioparms * fio, 
                  char * filename, FILE * f, unsigned char * buf, int size );

int fileio_srec ( struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size );

int fmt_autodetect ( char * fname );



char * fmtstr ( FILEFMT format )
{
  switch (format) {
    case FMT_AUTO : return "auto-detect"; break;
    case FMT_SREC : return "Motorola S-Record"; break;
    case FMT_IHEX : return "Intel Hex"; break;
    case FMT_RBIN : return "raw binary"; break;
    default       : return "invalid format"; break;
  };
}



int b2ihex ( unsigned char * inbuf, int bufsize, 
             int recsize, int startaddr,
             char * outfile, FILE * outf )
{
  unsigned char * buf;
  unsigned int nextaddr;
  int n, nbytes;
  int i;
  unsigned char cksum;

  if (recsize > 255) {
    fprintf ( stderr, "%s: recsize=%d, must be < 256\n",
              progname, recsize );
    return -1;
  }

  nextaddr = startaddr;
  buf      = inbuf;
  nbytes   = 0;

  while (bufsize) {
    n = recsize;
    if (n > bufsize)
      n = bufsize;

    if (n) {
      cksum = 0;
      fprintf ( outf, ":%02X%04X00", n, nextaddr );
      cksum += n + ((nextaddr >> 8) & 0x0ff) + (nextaddr & 0x0ff);
      for (i=0; i<n; i++) {
        fprintf ( outf, "%02X", buf[i] );
        cksum += buf[i];
      }
      cksum = -cksum;
      fprintf ( outf, "%02X\n", cksum );
      
      nextaddr += n;
      nbytes   += n;
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
  fprintf ( outf, ":%02X%04X01", n, nextaddr );
  cksum += n + ((nextaddr >> 8) & 0x0ff) + (nextaddr & 0x0ff) + 1;
  cksum = -cksum;
  fprintf ( outf, "%02X\n", cksum );

  return nbytes;
}


int ihex2b ( char * infile, FILE * inf,
             unsigned char * outbuf, int bufsize )
{
  unsigned char buffer [ MAX_LINE_LEN ];
  unsigned char * buf;
  unsigned int nextaddr;
  unsigned int b;
  int n, nbytes;
  int i, j;
  unsigned int cksum, rectype;
  int lineno;

  lineno   = 0;
  buf      = outbuf;
  nbytes   = 0;

  while (fgets((char *)buffer,MAX_LINE_LEN,inf)!=NULL) {
    lineno++;
    if (buffer[0] != ':')
      continue;
    if (sscanf((char *)&buffer[1], 
               "%02x%04x%02x", &n, &nextaddr, &rectype) != 3) {
      fprintf(stderr, "%s: invalid record at line %d of \"%s\"\n",
              progname, lineno, infile);
      exit(1);
    }

    if ((rectype != 0) && (rectype != 1)) {
      fprintf(stderr, 
              "%s: don't know how to deal with rectype=%d " 
              "at line %d of %s\n",
              progname, rectype, lineno, infile);
      exit(1);
    }

    if (n && ((nextaddr + n) > bufsize)) {
      fprintf(stderr, "%s: address 0x%04x out of range at line %d of %s\n",
              progname, nextaddr+n, lineno, infile);
      return -1;
    }

    /* start computing a checksum */
    cksum = n + ((nextaddr >> 8 ) & 0x0ff) + (nextaddr & 0x0ff) + rectype;

    for (i=0; i<n; i++) {
      if (sscanf((char *)&buffer[i*2+9], "%02x", &b) != 1) {
        fprintf(stderr, "%s: can't scan byte number %d at line %d of %s\n",
                progname, i, lineno, infile);
        /* display the buffer and the position of the scan error */
        fprintf(stderr, "%s", buffer);
        for (j=0; j<9+2*i; j++) {
          fprintf(stderr, " ");
        }
        fprintf(stderr, "^\n");
        return -1;
      }

      buf[nextaddr + i] = b;
      cksum += b;
    }

    nbytes += n;

    /*-----------------------------------------------------------------
      read the cksum value from the record and compare it with our
      computed value
      -----------------------------------------------------------------*/
    if (sscanf((char *)&buffer[n*2+9], "%02x", &b) != 1) {
      fprintf(stderr, "%s: can't scan byte number %d at line %d of %s\n",
              progname, i, lineno, infile);
      /* display the buffer and the position of the scan error */
      fprintf(stderr, "%s", buffer);
      for (j=0; j<9+2*i; j++) {
        fprintf(stderr, " ");
      }
      fprintf(stderr, "^\n");
      return -1;
    }

    cksum = -cksum & 0xff;
    if (cksum != b) {
      fprintf(stderr, 
              "%s: WARNING: cksum error for line %d of \"%s\": computed=%02x "
              "found=%02x\n",
              progname, lineno, infile, cksum, b);
      /* return -1; */
    }

    if (rectype == 1) {
      /* end of record */
      return nbytes;
    }

  } /* while */

  return nbytes;
}



int fileio_rbin ( struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size )
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


int fileio_ihex ( struct fioparms * fio, 
                  char * filename, FILE * f, unsigned char * buf, int size )
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


int fileio_srec ( struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size )
{
  fprintf(stderr, "%s: Motorola S-Record %s format not yet supported\n",
          progname, fio->iodesc);
  return -1;
}


int fileio_setparms ( int op, struct fioparms * fp )
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



int fmt_autodetect ( char * fname )
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



int fileio ( int op, char * filename, FILEFMT format, 
             struct avrpart * p, AVRMEM memtype, int size )
{
  int rc;
  FILE * f;
  char * fname;
  unsigned char * buf;
  struct fioparms fio;
  int i;

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

  switch (memtype) {
    case AVR_EEPROM:
      buf = p->eeprom;
      if (fio.op == FIO_READ)
        size = p->eeprom_size;
      break;

    case AVR_FLASH:
      buf = p->flash;
      if (fio.op == FIO_READ)
        size = p->flash_size;
      break;
      
    default:
      fprintf(stderr, "%s: invalid memory type for %s: %d\n",
              progname, fio.iodesc, memtype);
      return -1;
  }

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

    fprintf(stderr, "%s: %s file %s auto detected as %s\n\n", 
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

