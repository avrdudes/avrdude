/*
 * avrdude - A Uploader/Downloader for AVR device programmers
 * Copyright (C) 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef __pgm_h__
#define __pgm_h__

#include <limits.h>

#include "avrpart.h"
#include "lists.h"
#include "pindefs.h"


#define ON  1
#define OFF 0

#define PGM_DESCLEN 80
#define PGM_PORTLEN PATH_MAX
#define PGM_TYPELEN 32
typedef struct programmer_t {
  LISTID id;
  char desc[PGM_DESCLEN];
  char type[PGM_TYPELEN];
  char port[PGM_PORTLEN];
  unsigned int pinno[N_PINS];
  int ppidata;
  int fd;
  int  page_size;  /* page size if the programmer supports paged write/load */
  int  (*rdy_led)        (struct programmer_t * pgm, int value);
  int  (*err_led)        (struct programmer_t * pgm, int value);
  int  (*pgm_led)        (struct programmer_t * pgm, int value);
  int  (*vfy_led)        (struct programmer_t * pgm, int value);
  int  (*initialize)     (struct programmer_t * pgm, AVRPART * p);
  void (*display)        (struct programmer_t * pgm, char * p);
  int  (*save)           (struct programmer_t * pgm);
  void (*restore)        (struct programmer_t * pgm);
  void (*enable)         (struct programmer_t * pgm);
  void (*disable)        (struct programmer_t * pgm);
  void (*powerup)        (struct programmer_t * pgm);
  void (*powerdown)      (struct programmer_t * pgm);
  int  (*program_enable) (struct programmer_t * pgm, AVRPART * p);
  int  (*chip_erase)     (struct programmer_t * pgm, AVRPART * p);
  int  (*cmd)            (struct programmer_t * pgm, unsigned char cmd[4], 
                          unsigned char res[4]);
  void (*open)           (struct programmer_t * pgm, char * port);
  void (*close)          (struct programmer_t * pgm);
  int  (*paged_write)    (struct programmer_t * pgm, AVRPART * p, AVRMEM * m, 
                          int page_size, int n_bytes);
  int  (*paged_load)     (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          int page_size, int n_bytes);
} PROGRAMMER;


PROGRAMMER * pgm_new(void);

#endif
