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
#include "ppi.h"

#define SLOW_TOGGLE 0

extern char * progname;
extern int do_cycles;

/*
 * PPI registers
 */
enum {
  PPIDATA,
  PPICTRL,
  PPISTATUS
};

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


int ppi_getops    (int reg, unsigned long * get, unsigned long * set);

int ppi_set       (int fd, int reg, int bit);

int ppi_clr       (int fd, int reg, int bit);

int ppi_get       (int fd, int reg, int bit);

int ppi_toggle    (int fd, int reg, int bit);

int ppi_getall    (int fd, int reg);

int ppi_setall    (int fd, int reg, int val);

int ppi_pulse     (int fd, int reg, int bit);

int ppi_setpin    (int fd, int pin, int value);

int ppi_getpin    (int fd, int pin);

int ppi_pulsepin  (int fd, int pin);

int ppi_getpinreg (int pin);

int ppi_sense     (int fd);


int  ppi_rdy_led        (PROGRAMMER * pgm, int value);

int  ppi_err_led        (PROGRAMMER * pgm, int value);

int  ppi_pgm_led        (PROGRAMMER * pgm, int value);

int  ppi_vfy_led        (PROGRAMMER * pgm, int value);

int  ppi_cmd            (PROGRAMMER * pgm, unsigned char cmd[4], 
                         unsigned char res[4]);

int  ppi_chip_erase     (PROGRAMMER * pgm, AVRPART * p);

int  ppi_program_enable (PROGRAMMER * pgm, AVRPART * p);

void ppi_powerup        (PROGRAMMER * pgm);

void ppi_powerdown      (PROGRAMMER * pgm);

int  ppi_initialize     (PROGRAMMER * pgm, AVRPART * p);

int  ppi_save           (PROGRAMMER * pgm);

void ppi_restore        (PROGRAMMER * pgm);

void ppi_disable        (PROGRAMMER * pgm);

void ppi_enable         (PROGRAMMER * pgm);

void ppi_open           (PROGRAMMER * pgm, char * port);

void ppi_close          (PROGRAMMER * pgm);









static char vccpins_buf[64];
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
 * set 'get' and 'set' appropriately for subsequent passage to ioctl()
 * to get/set the specified PPI registers.  
 */
int ppi_getops(int reg, unsigned long * get, unsigned long * set)
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

/*
 * pulse the indicated bit of the specified register.
 */
int ppi_pulse(int fd, int reg, int bit)
{
  ppi_toggle(fd, reg, bit);

#if SLOW_TOGGLE
  usleep(1000);
#endif

  ppi_toggle(fd, reg, bit);

#if SLOW_TOGGLE
  usleep(1000);
#endif

  return 0;
}


int ppi_setpin(int fd, int pin, int value)
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


int ppi_getpin(int fd, int pin)
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


int ppi_pulsepin(int fd, int pin)
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


int ppi_getpinmask(int pin)
{
  if (pin < 1 || pin > 17)
    return -1;

  return pins[pin-1].bit;
}


int ppi_getpinreg(int pin)
{
  if (pin < 1 || pin > 17)
    return -1;

  return pins[pin-1].reg;
}


/*
 * infinite loop, sensing on the pin that we use to read data out of
 * the device; this is a debugging aid, you can insert a call to this
 * function in 'main()' and can use it to determine whether your sense
 * pin is actually sensing.  
 */
int ppi_sense(int fd)
{
  unsigned int pr;
  int count;
  char buf[128];
  int i;

  count = 0;

  fprintf(stderr, 
            "parallel port data:\n"
            "         111111111\n"
            "123456789012345678\n");

  buf[17] = 0;
  pr = 1;
  do {
    usleep(1); /* don't be too much of a cpu hog */
    for (i=1; i<=17; i++) {
      buf[i-1] = ppi_getpin(fd, i);
      if (buf[i-1])
        buf[i-1] = '|';
      else
        buf[i-1] = '.';
    }
    fprintf(stderr, "\r                   \r%s", buf);

  } while(1);

  return 0;
}




/*
 * transmit and receive a byte of data to/from the AVR device
 */
unsigned char ppi_txrx(PROGRAMMER * pgm, unsigned char byte)
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
    r = ppi_getpin(pgm->fd, pgm->pinno[PIN_AVR_MISO]);
    
    /* set the data input line as desired */
    ppi_setpin(pgm->fd, pgm->pinno[PIN_AVR_MOSI], b);
    
    /* 
     * pulse the clock line, clocking in the MOSI data, and clocking out
     * the next result bit
     */
    ppi_pulsepin(pgm->fd, pgm->pinno[PIN_AVR_SCK]);

    rbyte = rbyte | (r << (7-i));
  }

  return rbyte;
}





int ppi_rdy_led(PROGRAMMER * pgm, int value)
{
  ppi_setpin(pgm->fd, pgm->pinno[PIN_LED_RDY], !value);
  return 0;
}

int ppi_err_led(PROGRAMMER * pgm, int value)
{
  ppi_setpin(pgm->fd, pgm->pinno[PIN_LED_ERR], !value);
  return 0;
}

int ppi_pgm_led(PROGRAMMER * pgm, int value)
{
  ppi_setpin(pgm->fd, pgm->pinno[PIN_LED_PGM], !value);
  return 0;
}

int ppi_vfy_led(PROGRAMMER * pgm, int value)
{
  ppi_setpin(pgm->fd, pgm->pinno[PIN_LED_VFY], !value);
  return 0;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
int ppi_cmd(PROGRAMMER * pgm, unsigned char cmd[4], unsigned char res[4])
{
  int i;

  for (i=0; i<4; i++) {
    res[i] = ppi_txrx(pgm, cmd[i]);
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
int ppi_chip_erase(PROGRAMMER * pgm, AVRPART * p)
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
int ppi_program_enable(PROGRAMMER * pgm, AVRPART * p)
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
void ppi_powerup(PROGRAMMER * pgm)
{
  ppi_set(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power up */
  usleep(100000);
}


/*
 * remove power from the AVR processor
 */
void ppi_powerdown(PROGRAMMER * pgm)
{
  ppi_clr(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power down */
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
int ppi_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  int rc;
  int tries;

  pgm->powerup(pgm);
  usleep(20000);

  ppi_setpin(pgm->fd, pgm->pinno[PIN_AVR_SCK], 0);
  ppi_setpin(pgm->fd, pgm->pinno[PIN_AVR_RESET], 0);
  usleep(20000);

  ppi_pulsepin(pgm->fd, pgm->pinno[PIN_AVR_RESET]);

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
      ppi_pulsepin(pgm->fd, pgm->pinno[PIN_AVR_SCK]);
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


int ppi_save(PROGRAMMER * pgm)
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

void ppi_restore(PROGRAMMER * pgm)
{
  ppi_setall(pgm->fd, PPIDATA, pgm->ppidata);
}

void ppi_disable(PROGRAMMER * pgm)
{
  ppi_set(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_BUFF]);
}

void ppi_enable(PROGRAMMER * pgm)
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

  ppi_setpin(pgm->fd, pgm->pinno[PIN_AVR_RESET], 0);
  usleep(1);

  /*
   * enable the 74367 buffer, if connected; this signal is active low
   */
  ppi_clr(pgm->fd, PPIDATA, pgm->pinno[PPI_AVR_BUFF]);
}


void ppi_open(PROGRAMMER * pgm, char * port)
{
  pgm->fd = open(port, O_RDWR);
  if (pgm->fd < 0) {
    fprintf(stderr, "%s: can't open device \"%s\": %s\n\n",
              progname, port, strerror(errno));
    exit(1);
  }

  ppi_claim(pgm);
}


void ppi_close(PROGRAMMER * pgm)
{
  ppi_release(pgm);

  close(pgm->fd);
  pgm->fd = -1;
}


void ppi_display(PROGRAMMER * pgm, char * p)
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


void ppi_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "PPI");

  pgm->rdy_led        = ppi_rdy_led;
  pgm->err_led        = ppi_err_led;
  pgm->pgm_led        = ppi_pgm_led;
  pgm->vfy_led        = ppi_vfy_led;
  pgm->initialize     = ppi_initialize;
  pgm->display        = ppi_display;
  pgm->save           = ppi_save;
  pgm->restore        = ppi_restore;
  pgm->enable         = ppi_enable;
  pgm->disable        = ppi_disable;
  pgm->powerup        = ppi_powerup;
  pgm->powerdown      = ppi_powerdown;
  pgm->program_enable = ppi_program_enable;
  pgm->chip_erase     = ppi_chip_erase;
  pgm->cmd            = ppi_cmd;
  pgm->open           = ppi_open;
  pgm->close          = ppi_close;
}


