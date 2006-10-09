/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2004  Brian S. Dean <bsd@bsdhome.com>
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

extern LISTID       programmers;

typedef enum {
  EXIT_VCC_UNSPEC,
  EXIT_VCC_ENABLED,
  EXIT_VCC_DISABLED
} exit_vcc_t;

typedef enum {
  EXIT_RESET_UNSPEC,
  EXIT_RESET_ENABLED,
  EXIT_RESET_DISABLED
} exit_reset_t;

typedef struct programmer_t {
  LISTID id;
  char desc[PGM_DESCLEN];
  char type[PGM_TYPELEN];
  char port[PGM_PORTLEN];
  unsigned int pinno[N_PINS];
  exit_vcc_t exit_vcc;
  exit_reset_t exit_reset;
  int ppidata;
  int ppictrl;
  int baudrate;
  double bitclock;    /* JTAG ICE clock period in microseconds */
  int ispdelay;    /* ISP clock delay */
  int fd;
  int  page_size;  /* page size if the programmer supports paged write/load */
  int  (*rdy_led)        (struct programmer_t * pgm, int value);
  int  (*err_led)        (struct programmer_t * pgm, int value);
  int  (*pgm_led)        (struct programmer_t * pgm, int value);
  int  (*vfy_led)        (struct programmer_t * pgm, int value);
  int  (*initialize)     (struct programmer_t * pgm, AVRPART * p);
  void (*display)        (struct programmer_t * pgm, char * p);
  void (*enable)         (struct programmer_t * pgm);
  void (*disable)        (struct programmer_t * pgm);
  void (*powerup)        (struct programmer_t * pgm);
  void (*powerdown)      (struct programmer_t * pgm);
  int  (*program_enable) (struct programmer_t * pgm, AVRPART * p);
  int  (*chip_erase)     (struct programmer_t * pgm, AVRPART * p);
  int  (*cmd)            (struct programmer_t * pgm, unsigned char cmd[4], 
                          unsigned char res[4]);
  int  (*open)           (struct programmer_t * pgm, char * port);
  void (*close)          (struct programmer_t * pgm);
  int  (*paged_write)    (struct programmer_t * pgm, AVRPART * p, AVRMEM * m, 
                          int page_size, int n_bytes);
  int  (*paged_load)     (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          int page_size, int n_bytes);
  void (*write_setup)    (struct programmer_t * pgm, AVRPART * p, AVRMEM * m);
  int  (*write_byte)     (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          unsigned long addr, unsigned char value);
  int  (*read_byte)      (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          unsigned long addr, unsigned char * value);
  int  (*read_sig_bytes) (struct programmer_t * pgm, AVRPART * p, AVRMEM * m);
  void (*print_parms)    (struct programmer_t * pgm);
  int  (*set_vtarget)    (struct programmer_t * pgm, double v);
  int  (*set_varef)      (struct programmer_t * pgm, double v);
  int  (*set_fosc)       (struct programmer_t * pgm, double v);
  int  (*set_sck_period) (struct programmer_t * pgm, double v);
  int  (*setpin)         (struct programmer_t * pgm, int pin, int value);
  int  (*getpin)         (struct programmer_t * pgm, int pin);
  int  (*highpulsepin)   (struct programmer_t * pgm, int pin);
  int  (*parseexitspecs) (struct programmer_t * pgm, char *s);
  int  (*perform_osccal) (struct programmer_t * pgm);
  char config_file[PATH_MAX]; /* config file where defined */
  int  lineno;                /* config file line number */
  char flag;		      /* for private use of the programmer */
} PROGRAMMER;


PROGRAMMER * pgm_new(void);

#if defined(WIN32NATIVE)

#include "ac_cfg.h"
#include <windows.h>

/* usleep replacements */
/* sleep Windows in ms, Unix usleep in us
 #define usleep(us) Sleep((us)<20000?20:us/1000)
 #define usleep(us) Sleep(us/1000)
 #define ANTIWARP 3
 #define usleep(us) Sleep(us/1000*ANTIWARP)
*/
void usleep(unsigned long us);

#if !defined(HAVE_GETTIMEOFDAY)
int gettimeofday(struct timeval *tv, struct timezone *tz);
#endif /* HAVE_GETTIMEOFDAY */

#endif /* __win32native_h */


#endif
