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

#ifndef __fileio_h__
#define __fileio_h__

typedef enum {
  FMT_AUTO,
  FMT_SREC,
  FMT_IHEX,
  FMT_RBIN
} FILEFMT;

struct fioparms {
  int    op;
  char * mode;
  char * iodesc;
  char * dir;
  char * rw;
};

enum {
  FIO_READ,
  FIO_WRITE
};

char * fmtstr(FILEFMT format);

int fileio_setparms(int op, struct fioparms * fp);

int fileio(int op, char * filename, FILEFMT format, 
           struct avrpart * p, char * memtype, int size);

#endif
