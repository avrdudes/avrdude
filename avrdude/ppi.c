/*
 * avrdude - A Downloader/Uploader for AVR device programmers
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

#if defined(__FreeBSD__) || defined(__linux__)

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#if defined(__FreeBSD__)
#include <dev/ppbus/ppi.h>
#elif defined(__linux__)
#include "linux_ppdev.h"
#endif

#include "avr.h"
#include "pindefs.h"
#include "pgm.h"
#include "ppi.h"

extern char * progname;

/*
 * set 'get' and 'set' appropriately for subsequent passage to ioctl()
 * to get/set the specified PPI registers.  
 */
static int ppi_getops(int reg, unsigned long * get, unsigned long * set)
{
  switch (reg) {
    case PPIDATA:
      *set = PPISDATA;
      *get = PPIGDATA;
      break;
    case PPICTRL:
      *set = PPISCTRL;
      *get = PPIGCTRL;
      break;
    case PPISTATUS:
      *set = PPISSTATUS;
      *get = PPIGSTATUS;
      break;
    default:
      fprintf(stderr, "%s: avr_set(): invalid register=%d\n",
              progname, reg);
      return -1;
      break;
  }

  return 0;
}


/*
 * set the indicated bit of the specified register.
 */
int ppi_set(int fd, int reg, int bit)
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops(reg, &get, &set);
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v |= bit;
  ioctl(fd, set, &v);

  return 0;
}


/*
 * clear the indicated bit of the specified register.
 */
int ppi_clr(int fd, int reg, int bit)
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops(reg, &get, &set);
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v &= ~bit;
  ioctl(fd, set, &v);

  return 0;
}


/*
 * get the indicated bit of the specified register.
 */
int ppi_get(int fd, int reg, int bit)
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops(reg, &get, &set);
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v &= bit;

  return v; /* v == bit */
}

/*
 * toggle the indicated bit of the specified register.
 */
int ppi_toggle(int fd, int reg, int bit)
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops(reg, &get, &set);
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v ^= bit;
  ioctl(fd, set, &v);

  return 0;
}


/*
 * get all bits of the specified register.
 */
int ppi_getall(int fd, int reg)
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops(reg, &get, &set);
  if (rc)
    return -1;

  ioctl(fd, get, &v);

  return (int)v;
}

/*
 * set all bits of the specified register to val.
 */
int ppi_setall(int fd, int reg, int val)
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops(reg, &get, &set);
  if (rc)
    return -1;

  v = val;
  ioctl(fd, set, &v);

  return 0;
}


int ppi_open(char * port)
{
  int fd;

  fd = open(port, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "%s: can't open device \"%s\": %s\n",
              progname, port, strerror(errno));
    return -1;
  }

  return fd;
}


void ppi_close(int fd)
{
  close(fd);
}


#endif
