/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2004  Brian S. Dean <bsd@bsdhome.com>
 * Copyright 2007 Joerg Wunsch <j@uriah.heep.sax.de>
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

#ifndef pgm_h
#define pgm_h

#include <limits.h>

#include "avrpart.h"
#include "lists.h"
#include "pindefs.h"
#include "serial.h"

#define ON  1
#define OFF 0

#define PGM_DESCLEN 80
#define PGM_PORTLEN PATH_MAX
#define PGM_TYPELEN 32
#define PGM_USBSTRINGLEN 256

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

typedef enum {
  EXIT_DATAHIGH_UNSPEC,
  EXIT_DATAHIGH_ENABLED,
  EXIT_DATAHIGH_DISABLED
} exit_datahigh_t;

typedef enum {
  CONNTYPE_PARALLEL,
  CONNTYPE_SERIAL,
  CONNTYPE_USB
} conntype_t;

typedef struct programmer_t {
  LISTID id;
  char desc[PGM_DESCLEN];
  char type[PGM_TYPELEN];
  char port[PGM_PORTLEN];
  void (*initpgm)(struct programmer_t * pgm);
  unsigned int pinno[N_PINS];
  struct pindef_t pin[N_PINS];
  exit_vcc_t exit_vcc;
  exit_reset_t exit_reset;
  exit_datahigh_t exit_datahigh;
  conntype_t conntype;
  int ppidata;
  int ppictrl;
  int baudrate;
  int usbvid, usbpid;
  char usbdev[PGM_USBSTRINGLEN], usbsn[PGM_USBSTRINGLEN];
  char usbvendor[PGM_USBSTRINGLEN], usbproduct[PGM_USBSTRINGLEN];
  double bitclock;    /* JTAG ICE clock period in microseconds */
  int ispdelay;    /* ISP clock delay */
  union filedescriptor fd;
  int  page_size;  /* page size if the programmer supports paged write/load */
  int  (*rdy_led)        (struct programmer_t * pgm, int value);
  int  (*err_led)        (struct programmer_t * pgm, int value);
  int  (*pgm_led)        (struct programmer_t * pgm, int value);
  int  (*vfy_led)        (struct programmer_t * pgm, int value);
  int  (*initialize)     (struct programmer_t * pgm, AVRPART * p);
  void (*display)        (struct programmer_t * pgm, const char * p);
  void (*enable)         (struct programmer_t * pgm);
  void (*disable)        (struct programmer_t * pgm);
  void (*powerup)        (struct programmer_t * pgm);
  void (*powerdown)      (struct programmer_t * pgm);
  int  (*program_enable) (struct programmer_t * pgm, AVRPART * p);
  int  (*chip_erase)     (struct programmer_t * pgm, AVRPART * p);
  int  (*cmd)            (struct programmer_t * pgm, const unsigned char *cmd,
                          unsigned char *res);
  int  (*cmd_tpi)        (struct programmer_t * pgm, const unsigned char *cmd,
                          int cmd_len, unsigned char res[], int res_len);
  int  (*spi)            (struct programmer_t * pgm, const unsigned char *cmd,
                          unsigned char *res, int count);
  int  (*open)           (struct programmer_t * pgm, char * port);
  void (*close)          (struct programmer_t * pgm);
  int  (*paged_write)    (struct programmer_t * pgm, AVRPART * p, AVRMEM * m, 
                          unsigned int page_size, unsigned int baseaddr,
                          unsigned int n_bytes);
  int  (*paged_load)     (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          unsigned int page_size, unsigned int baseaddr,
                          unsigned int n_bytes);
  int  (*page_erase)     (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          unsigned int baseaddr);
  void (*write_setup)    (struct programmer_t * pgm, AVRPART * p, AVRMEM * m);
  int  (*write_byte)     (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          unsigned long addr, unsigned char value);
  int  (*read_byte)      (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          unsigned long addr, unsigned char * value);
  int  (*read_sig_bytes) (struct programmer_t * pgm, AVRPART * p, AVRMEM * m);
  void (*print_parms)    (struct programmer_t * pgm);
  int  (*set_vtarget)    (struct programmer_t * pgm, double v);
  int  (*set_varef)      (struct programmer_t * pgm, unsigned int chan, double v);
  int  (*set_fosc)       (struct programmer_t * pgm, double v);
  int  (*set_sck_period) (struct programmer_t * pgm, double v);
  int  (*setpin)         (struct programmer_t * pgm, int pinfunc, int value);
  int  (*getpin)         (struct programmer_t * pgm, int pinfunc);
  int  (*highpulsepin)   (struct programmer_t * pgm, int pinfunc);
  int  (*parseexitspecs) (struct programmer_t * pgm, char *s);
  int  (*perform_osccal) (struct programmer_t * pgm);
  int  (*parseextparams) (struct programmer_t * pgm, LISTID xparams);
  void (*setup)          (struct programmer_t * pgm);
  void (*teardown)       (struct programmer_t * pgm);
  char config_file[PATH_MAX]; /* config file where defined */
  int  lineno;                /* config file line number */
  void *cookie;		      /* for private use by the programmer */
  char flag;		      /* for private use of the programmer */
} PROGRAMMER;

#ifdef __cplusplus
extern "C" {
#endif

PROGRAMMER * pgm_new(void);
PROGRAMMER * pgm_dup(const PROGRAMMER * const src);
void         pgm_free(PROGRAMMER * const p);

void programmer_display(PROGRAMMER * pgm, const char * p);

/* show is a mask like this (1<<PIN_AVR_SCK)|(1<<PIN_AVR_MOSI)| ... */
#define SHOW_ALL_PINS (~0u)
#define SHOW_PPI_PINS ((1<<PPI_AVR_VCC)|(1<<PPI_AVR_BUFF))
#define SHOW_AVR_PINS ((1<<PIN_AVR_RESET)|(1<<PIN_AVR_SCK)|(1<<PIN_AVR_MOSI)|(1<<PIN_AVR_MISO))
#define SHOW_LED_PINS ((1<<PIN_LED_ERR)|(1<<PIN_LED_RDY)|(1<<PIN_LED_PGM)|(1<<PIN_LED_VFY))
void pgm_display_generic_mask(PROGRAMMER * pgm, const char * p, unsigned int show);
void pgm_display_generic(PROGRAMMER * pgm, const char * p);

PROGRAMMER * locate_programmer(LISTID programmers, const char * configid);

typedef void (*walk_programmers_cb)(const char *name, const char *desc,
                                    const char *cfgname, int cfglineno,
                                    void *cookie);
void walk_programmers(LISTID programmers, walk_programmers_cb cb, void *cookie);

void sort_programmers(LISTID programmers);

#ifdef __cplusplus
}
#endif

#endif
