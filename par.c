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
#include "par.h"
#include "ppi.h"

#define SLOW_TOGGLE 0

struct ppipins_t {
  int pin;
  int reg;
  int bit;
  int inverted;
};

static struct ppipins_t pins[] = {
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

#define NPINS (sizeof(pins)/sizeof(struct ppipins_t))


extern char * progname;
extern int do_cycles;


static int par_setpin          (int fd, int pin, int value);

static int par_getpin          (int fd, int pin);

static int par_pulsepin        (int fd, int pin);


static int  par_rdy_led        (PROGRAMMER * pgm, int value);

static int  par_err_led        (PROGRAMMER * pgm, int value);

static int  par_pgm_led        (PROGRAMMER * pgm, int value);

static int  par_vfy_led        (PROGRAMMER * pgm, int value);

static int  par_cmd            (PROGRAMMER * pgm, unsigned char cmd[4], 
                                unsigned char res[4]);

static int  par_chip_erase     (PROGRAMMER * pgm, AVRPART * p);

static int  par_program_enable (PROGRAMMER * pgm, AVRPART * p);

static void par_powerup        (PROGRAMMER * pgm);

static void par_powerdown      (PROGRAMMER * pgm);

static int  par_initialize     (PROGRAMMER * pgm, AVRPART * p);

static int  par_save           (PROGRAMMER * pgm);

static void par_restore        (PROGRAMMER * pgm);

static void par_disable        (PROGRAMMER * pgm);

static void par_enable         (PROGRAMMER * pgm);

static void par_open           (PROGRAMMER * pgm, char * port);

static void par_close          (PROGRAMMER * pgm);


static int par_setpin(int fd, int pin, int value)
{

  if (pin < 1 || pin > 17)
    return -1;

  pin--;

  if (pins[pin].inverted)
    value = !value;

  if (value)
    ppi_set(fd, pins[pin].reg, pins[pin].bit);
  else
    ppi_clr(fd, pins[pin].reg, pins[pin].bit);

#if SLOW_TOGGLE
  usleep(1000);
#endif

  return 0;
}


static int par_getpin(int fd, int pin)
{
  int value;

  if (pin < 1 || pin > 17)
    return -1;

  pin--;

  value = ppi_get(fd, pins[pin].reg, pins[pin].bit);

  if (value)
    value = 1;
    
  if (pins[pin].inverted)
    value = !value;

  return value;
}


static int par_pulsepin(int fd, int pin)
{

  if (pin < 1 || pin > 17)
    return -1;

  pin--;

  ppi_toggle(fd, pins[pin].reg, pins[pin].bit);

#if SLOW_TOGGLE
  usleep(1000);
#endif

  ppi_toggle(fd, pins[pin].reg, pins[pin].bit);

#if SLOW_TOGGLE
  usleep(1000);
#endif

  return 0;
}

int par_getpinmask(int pin)
{
  if (pin < 1 || pin > 17)
    return -1;

  return pins[pin-1].bit;
}


static char vccpins_buf[64];
static char * vccpins_str(unsigned int pmask)
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
 * transmit and receive a byte of data to/from the AVR device
 */
static unsigned char par_txrx(PROGRAMMER * pgm, unsigned char byte)
{
  int i;
  unsigned char r, b, rbyte;

  rbyte = 0;
  for (i=0; i<8; i++) {
    b = (byte >> (7-i)) & 0x01;

    /* 
     * read the result bit (it is either valid from a previous clock
     * pulse or it is ignored in the current context)
     */
    r = par_getpin(pgm->fd, pgm->pinno[PIN_AVR_MISO]);
    
    /* set the data input line as desired */
    par_setpin(pgm->fd, pgm->pinno[PIN_AVR_MOSI], b);
    
    /* 
     * pulse the clock line, clocking in the MOSI data, and clocking out
     * the next result bit
     */
    par_pulsepin(pgm->fd, pgm->pinno[PIN_AVR_SCK]);

    rbyte = rbyte | (r << (7-i));
  }

  return rbyte;
}





static int par_rdy_led(PROGRAMMER * pgm, int value)
{
  par_setpin(pgm->fd, pgm->pinno[PIN_LED_RDY], !value);
  return 0;
}

static int par_err_led(PROGRAMMER * pgm, int value)
{
  par_setpin(pgm->fd, pgm->pinno[PIN_LED_ERR], !value);
  return 0;
}

static int par_pgm_led(PROGRAMMER * pgm, int value)
{
  par_setpin(pgm->fd, pgm->pinno[PIN_LED_PGM], !value);
  return 0;
}

static int par_vfy_led(PROGRAMMER * pgm, int value)
{
  par_setpin(pgm->fd, pgm->pinno[PIN_LED_VFY], !value);
  return 0;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
static int par_cmd(PROGRAMMER * pgm, unsigned char cmd[4], 
                   unsigned char res[4])
{
  int i;

  for (i=0; i<4; i++) {
    res[i] = par_txrx(pgm, cmd[i]);
  }

#if 0
  fprintf(stderr, "avr_cmd(): [ ");
  for (i=0; i<4; i++)
    fprintf(stderr, "%02x ", cmd[i]);
  fprintf(stderr, "] [ ");
  for (i=0; i<4; i++)
    fprintf(stderr, "%02x ", res[i]);
  fprintf(stderr, "]\n");
#endif

  return 0;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
static int par_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char cmd[4];
  unsigned char res[4];
  int cycles;
  int rc;

  if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
    fprintf(stderr, "chip erase instruction not defined for part \"%s\"\n",
            p->desc);
    return -1;
  }

  rc = avr_get_cycle_count(pgm, p, &cycles);

  /*
   * only print out the current cycle count if we aren't going to
   * display it below 
   */
  if (!do_cycles && ((rc >= 0) && (cycles != 0xffffffff))) {
    fprintf(stderr,
            "%s: current erase-rewrite cycle count is %d%s\n",
            progname, cycles, 
            do_cycles ? "" : " (if being tracked)");
  }

  pgm->pgm_led(pgm, ON);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  pgm->pgm_led(pgm, OFF);

  if (do_cycles && (cycles != -1)) {
    if (cycles == 0x00ffff) {
      cycles = 0;
    }
    cycles++;
    fprintf(stderr, "%s: erase-rewrite cycle count is now %d\n", 
            progname, cycles);
    avr_put_cycle_count(pgm, p, cycles);
  }

  return 0;
}

/*
 * issue the 'program enable' command to the AVR device
 */
static int par_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char cmd[4];
  unsigned char res[4];

  if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
    fprintf(stderr, "program enable instruction not defined for part \"%s\"\n",
            p->desc);
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
  pgm->cmd(pgm, cmd, res);

  if (res[2] != cmd[1])
    return -2;

  return 0;
}


/*
 * apply power to the AVR processor
 */
static void par_powerup(PROGRAMMER * pgm)
{
  ppi_set(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power up */
  usleep(100000);
}


/*
 * remove power from the AVR processor
 */
static void par_powerdown(PROGRAMMER * pgm)
{
  ppi_clr(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power down */
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int par_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  int rc;
  int tries;

  pgm->powerup(pgm);
  usleep(20000);

  par_setpin(pgm->fd, pgm->pinno[PIN_AVR_SCK], 0);
  par_setpin(pgm->fd, pgm->pinno[PIN_AVR_RESET], 0);
  usleep(20000);

  par_pulsepin(pgm->fd, pgm->pinno[PIN_AVR_RESET]);

  usleep(20000); /* 20 ms XXX should be a per-chip parameter */

  /*
   * Enable programming mode.  If we are programming an AT90S1200, we
   * can only issue the command and hope it worked.  If we are using
   * one of the other chips, the chip will echo 0x53 when issuing the
   * third byte of the command.  In this case, try up to 32 times in
   * order to possibly get back into sync with the chip if we are out
   * of sync.
   */
  if (strcmp(p->desc, "AT90S1200")==0) {
    pgm->program_enable(pgm, p);
  }
  else {
    tries = 0;
    do {
      rc = pgm->program_enable(pgm, p);
      if ((rc == 0)||(rc == -1))
        break;
      par_pulsepin(pgm->fd, pgm->pinno[PIN_AVR_SCK]);
      tries++;
    } while (tries < 65);

    /*
     * can't sync with the device, maybe it's not attached?
     */
    if (rc) {
      fprintf(stderr, "%s: AVR device not responding\n", progname);
      return -1;
    }
  }

  return 0;
}


static int par_save(PROGRAMMER * pgm)
{
  int rc;

  rc = ppi_getall(pgm->fd, PPIDATA);
  if (rc < 0) {
    fprintf(stderr, "%s: error reading status of ppi data port\n", progname);
    return -1;
  }

  pgm->ppidata = rc;

  return 0;
}

static void par_restore(PROGRAMMER * pgm)
{
  ppi_setall(pgm->fd, PPIDATA, pgm->ppidata);
}

static void par_disable(PROGRAMMER * pgm)
{
  ppi_set(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_BUFF]);
}

static void par_enable(PROGRAMMER * pgm)
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


static void par_open(PROGRAMMER * pgm, char * port)
{
  pgm->fd = ppi_open(port);
  if (pgm->fd < 0) {
    fprintf(stderr, "%s: failed to open parallel port \"%s\"\n\n",
            progname, port);
    exit(1);
  }

  ppi_claim(pgm);
}


static void par_close(PROGRAMMER * pgm)
{
  ppi_release(pgm);

  ppi_close(pgm->fd);
  pgm->fd = -1;
}


static void par_display(PROGRAMMER * pgm, char * p)
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


  fprintf(stderr, "%sProgrammer Pin Configuration: %s (%s)\n", p, 
          (char *)ldata(lfirst(pgm->id)), pgm->desc);

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

  pgm->rdy_led        = par_rdy_led;
  pgm->err_led        = par_err_led;
  pgm->pgm_led        = par_pgm_led;
  pgm->vfy_led        = par_vfy_led;
  pgm->initialize     = par_initialize;
  pgm->display        = par_display;
  pgm->save           = par_save;
  pgm->restore        = par_restore;
  pgm->enable         = par_enable;
  pgm->disable        = par_disable;
  pgm->powerup        = par_powerup;
  pgm->powerdown      = par_powerdown;
  pgm->program_enable = par_program_enable;
  pgm->chip_erase     = par_chip_erase;
  pgm->cmd            = par_cmd;
  pgm->open           = par_open;
  pgm->close          = par_close;
}


