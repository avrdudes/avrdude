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
#include <unistd.h>
#include </sys/dev/ppbus/ppi.h>

#include "ppi.h"

extern char * progname;

/*
 * set 'get' and 'set' appropriately for subsequent passage to ioctl()
 * to get/set the specified PPI registers.  
 */
int ppi_getops ( int reg, unsigned long * get, unsigned long * set )
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
              progname, reg );
      return -1;
      break;
  }

  return 0;
}


/*
 * set the indicated bit of the specified register.
 */
int ppi_set ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
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
int ppi_clr ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
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
int ppi_get ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v &= bit;

  return (v == bit);
}

/*
 * toggle the indicated bit of the specified register.
 */
int ppi_toggle ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
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
int ppi_getall ( int fd, int reg )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  ioctl(fd, get, &v);

  return (int)v;
}

/*
 * set all bits of the specified register to val.
 */
int ppi_setall ( int fd, int reg, int val )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  v = val;
  ioctl(fd, set, &v);

  return 0;
}

/*
 * pulse the indicated bit of the specified register.
 */
int ppi_pulse ( int fd, int reg, int bit )
{
  ppi_toggle(fd, reg, bit);
  ppi_toggle(fd, reg, bit);

  return 0;
}


/*
 * infinite loop, sensing on the pin that we use to read data out of
 * the device; this is a debugging aid, you can insert a call to this
 * function in 'main()' and can use it to determine whether your sense
 * pin is actually sensing.  
 */
int ppi_sense_test ( int fd, int reg, int bit )
{
  unsigned char v, pv;

  pv = 1;
  do {
    usleep(100000); /* check every 100 ms */
    v = ppi_get(fd, reg, bit);
    if (v != pv) {
      fprintf ( stderr, "sense bit = %d\n", v );
    }
    pv = v;
  } while(1);

  return 0;
}


