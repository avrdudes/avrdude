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
#include "bitbang.h"

#define SLOW_TOGGLE 0

struct ppipins_t {
  int pin;
  int reg;
  int bit;
  int inverted;
};

struct ppipins_t ppipins[] = {
  {  1, PPICTRL,   0x01, 1 },
  {  2, PPIDATA,   0x01, 0 },
  {  3, PPIDATA,   0x02, 0 },
  {  4, PPIDATA,   0x04, 0 },
  {  5, PPIDATA,   0x08, 0 },
  {  6, PPIDATA,   0x10, 0 },
  {  7, PPIDATA,   0x20, 0 },
  {  8, PPIDATA,   0x40, 0 },
  {  9, PPIDATA,   0x80, 0 },
  { 10, PPISTATUS, 0x40, 0 },
  { 11, PPISTATUS, 0x80, 1 },
  { 12, PPISTATUS, 0x20, 0 },
  { 13, PPISTATUS, 0x10, 0 },
  { 14, PPICTRL,   0x02, 1 }, 
  { 15, PPISTATUS, 0x08, 0 },
  { 16, PPICTRL,   0x04, 0 }, 
  { 17, PPICTRL,   0x08, 1 }
};

#define NPINS (sizeof(ppipins)/sizeof(struct ppipins_t))


extern char * progname;
extern int do_cycles;
extern int verbose;

int par_setpin(int fd, int pin, int value)
{

  pin &= PIN_MASK;

  if (pin < 1 || pin > 17)
    return -1;

  pin--;

  if (ppipins[pin].inverted)
    value = !value;

  if (value)
    ppi_set(fd, ppipins[pin].reg, ppipins[pin].bit);
  else
    ppi_clr(fd, ppipins[pin].reg, ppipins[pin].bit);

#if SLOW_TOGGLE
  usleep(1000);
#endif

  return 0;
}


int par_getpin(int fd, int pin)
{
  int value;

  pin &= PIN_MASK;

  if (pin < 1 || pin > 17)
    return -1;

  pin--;

  value = ppi_get(fd, ppipins[pin].reg, ppipins[pin].bit);

  if (value)
    value = 1;
    
  if (ppipins[pin].inverted)
    value = !value;

  return value;
}


int par_highpulsepin(int fd, int pin)
{

  if (pin < 1 || pin > 17)
    return -1;

  pin--;

  ppi_set(fd, ppipins[pin].reg, ppipins[pin].bit);
#if SLOW_TOGGLE
  usleep(1000);
#endif
  ppi_clr(fd, ppipins[pin].reg, ppipins[pin].bit);

#if SLOW_TOGGLE
  usleep(1000);
#endif

  return 0;
}

int par_getpinmask(int pin)
{
  pin &= PIN_MASK;

  if (pin < 1 || pin > 17)
    return -1;

  return ppipins[pin-1].bit;
}


char vccpins_buf[64];
char * vccpins_str(unsigned int pmask)
{
  unsigned int mask;
  int pin;
  char b2[8];
  char * b;

  b = vccpins_buf;

  b[0] = 0;
  for (pin = 2, mask = 1; mask < 0x80; mask = mask << 1, pin++) {
    if (pmask & mask) {
      sprintf(b2, "%d", pin);
      if (b[0] != 0)
        strcat(b, ",");
      strcat(b, b2);
    }
  }

  return b;
}

/*
 * apply power to the AVR processor
 */
void par_powerup(PROGRAMMER * pgm)
{
  ppi_set(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power up */
  usleep(100000);
}


/*
 * remove power from the AVR processor
 */
void par_powerdown(PROGRAMMER * pgm)
{
  ppi_clr(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power down */
}

void par_disable(PROGRAMMER * pgm)
{
  ppi_set(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_BUFF]);
}

void par_enable(PROGRAMMER * pgm)
{
  /*
   * Prepare to start talking to the connected device - pull reset low
   * first, delay a few milliseconds, then enable the buffer.  This
   * sequence allows the AVR to be reset before the buffer is enabled
   * to avoid a short period of time where the AVR may be driving the
   * programming lines at the same time the programmer tries to.  Of
   * course, if a buffer is being used, then the /RESET line from the
   * programmer needs to be directly connected to the AVR /RESET line
   * and not via the buffer chip.
   */

  par_setpin(pgm->fd, pgm->pinno[PIN_AVR_RESET], 0);
  usleep(1);

  /*
   * enable the 74367 buffer, if connected; this signal is active low
   */
  ppi_clr(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_BUFF]);
}

int par_open(PROGRAMMER * pgm, char * port)
{
  int rc;

  pgm->fd = ppi_open(port);
  if (pgm->fd < 0) {
    fprintf(stderr, "%s: failed to open parallel port \"%s\"\n\n",
            progname, port);
    exit(1);
  }

  ppi_claim(pgm);

  /*
   * save pin values, so they can be restored when device is closed
   */
  rc = ppi_getall(pgm->fd, PPIDATA);
  if (rc < 0) {
    fprintf(stderr, "%s: error reading status of ppi data port\n", progname);
    return -1;
  }
  pgm->ppidata = rc;

  rc = ppi_getall(pgm->fd, PPICTRL);
  if (rc < 0) {
    fprintf(stderr, "%s: error reading status of ppi ctrl port\n", progname);
    return -1;
  }
  pgm->ppictrl = rc;

  return 0;
}


void par_close(PROGRAMMER * pgm)
{
  /*
   * Restore pin values before closing,
   * but ensure that buffers are turned off.
   */
  pgm->ppidata |= pgm->pinno[PPI_AVR_BUFF];
  ppi_setall(pgm->fd, PPIDATA, pgm->ppidata);
  ppi_setall(pgm->fd, PPICTRL, pgm->ppictrl);

  ppi_release(pgm);

  ppi_close(pgm->fd);
  pgm->fd = -1;
}

void par_display(PROGRAMMER * pgm, char * p)
{
  char vccpins[64];
  char buffpins[64];

  if (pgm->pinno[PPI_AVR_VCC]) {
    snprintf(vccpins, sizeof(vccpins), " = pins %s",
             vccpins_str(pgm->pinno[PPI_AVR_VCC]));
  }
  else {
    strcpy(vccpins, " (not used)");
  }

  if (pgm->pinno[PPI_AVR_BUFF]) {
    snprintf(buffpins, sizeof(buffpins), " = pins %s", 
             vccpins_str(pgm->pinno[PPI_AVR_BUFF]));
  }
  else {
    strcpy(buffpins, " (not used)");
  }

  fprintf(stderr, 
          "%s  VCC     = 0x%02x%s\n"
          "%s  BUFF    = 0x%02x%s\n"
          "%s  RESET   = %d\n"
          "%s  SCK     = %d\n"
          "%s  MOSI    = %d\n"
          "%s  MISO    = %d\n"
          "%s  ERR LED = %d\n"
          "%s  RDY LED = %d\n"
          "%s  PGM LED = %d\n"
          "%s  VFY LED = %d\n",

          p, pgm->pinno[PPI_AVR_VCC], vccpins,
          p, pgm->pinno[PPI_AVR_BUFF], buffpins,
          p, pgm->pinno[PIN_AVR_RESET],
          p, pgm->pinno[PIN_AVR_SCK],
          p, pgm->pinno[PIN_AVR_MOSI],
          p, pgm->pinno[PIN_AVR_MISO],
          p, pgm->pinno[PIN_LED_ERR],
          p, pgm->pinno[PIN_LED_RDY],
          p, pgm->pinno[PIN_LED_PGM],
          p, pgm->pinno[PIN_LED_VFY]);
}


void par_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "PPI");

  pgm->rdy_led        = bitbang_rdy_led;
  pgm->err_led        = bitbang_err_led;
  pgm->pgm_led        = bitbang_pgm_led;
  pgm->vfy_led        = bitbang_vfy_led;
  pgm->initialize     = bitbang_initialize;
  pgm->display        = par_display;
  pgm->enable         = par_enable;
  pgm->disable        = par_disable;
  pgm->powerup        = par_powerup;
  pgm->powerdown      = par_powerdown;
  pgm->program_enable = bitbang_program_enable;
  pgm->chip_erase     = bitbang_chip_erase;
  pgm->cmd            = bitbang_cmd;
  pgm->open           = par_open;
  pgm->close          = par_close;
  
  /* this is a parallel port bitbang device */
  pgm->flag	      = 0;
}
