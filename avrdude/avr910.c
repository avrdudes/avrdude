/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003  Theodore A. Roth  <troth@openavr.org>
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

/*
 * avrdude interface for Atmel Low Cost Serial programmers which adher to the
 * protocol described in application note avr910.
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "avr.h"
#include "pgm.h"
#include "avr910.h"
#include "serial.h"

extern char * progname;
extern int do_cycles;

static char has_auto_incr_addr;


static int avr910_send(PROGRAMMER * pgm, char * buf, size_t len)
{
  return serial_send(pgm->fd, buf, len);
}


static int avr910_recv(PROGRAMMER * pgm, char * buf, size_t len)
{
  return serial_recv(pgm->fd, buf, len);
}


static int avr910_drain(PROGRAMMER * pgm, int display)
{
  return serial_drain(pgm->fd, display);
}


static void avr910_vfy_cmd_sent(PROGRAMMER * pgm, char * errmsg)
{
  char c;

  avr910_recv(pgm, &c, 1);
  if (c != '\r') {
    fprintf(stderr, "%s: error: programmer did not respond to command: %s\n",
            progname, errmsg);
    exit(1);
  }
}


static int avr910_rdy_led(PROGRAMMER * pgm, int value)
{
  /* Do nothing. */

  return 0;
}


static int avr910_err_led(PROGRAMMER * pgm, int value)
{
  /* Do nothing. */

  return 0;
}


static int avr910_pgm_led(PROGRAMMER * pgm, int value)
{
  /* Do nothing. */

  return 0;
}


static int avr910_vfy_led(PROGRAMMER * pgm, int value)
{
  /* Do nothing. */

  return 0;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
static int avr910_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  avr910_send(pgm, "e", 1);
  avr910_vfy_cmd_sent(pgm, "chip erase");

  return 0;
}


static void avr910_enter_prog_mode(PROGRAMMER * pgm)
{
  avr910_send(pgm, "P", 1);
  avr910_vfy_cmd_sent(pgm, "enter prog mode");
}


static void avr910_leave_prog_mode(PROGRAMMER * pgm)
{
  avr910_send(pgm, "L", 1);
  avr910_vfy_cmd_sent(pgm, "leave prog mode");
}


/*
 * issue the 'program enable' command to the AVR device
 */
static int avr910_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  return -1;
}


/*
 * apply power to the AVR processor
 */
static void avr910_powerup(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}


/*
 * remove power from the AVR processor
 */
static void avr910_powerdown(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int avr910_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  char id[8];
  char sw[2];
  char hw[2];
  char buf[10];
  char type;
  unsigned char c;
  int dev_supported = 0;

  /* Get the programmer identifier. Programmer returns exactly 7 chars
     _without_ the null.*/

  avr910_send(pgm, "S", 1);
  memset (id, 0, sizeof(id));
  avr910_recv(pgm, id, sizeof(id)-1);

  /* Get the HW and SW versions to see if the programmer is present. */

  avr910_send(pgm, "V", 1);
  avr910_recv(pgm, sw, sizeof(sw));

  avr910_send(pgm, "v", 1);
  avr910_recv(pgm, hw, sizeof(hw));

  /* Get the programmer type (serial or parallel). Expect serial. */

  avr910_send(pgm, "p", 1);
  avr910_recv(pgm, &type, 1);

  fprintf(stderr, "Found programmer: Id = \"%s\"; type = %c\n", id, type);
  fprintf(stderr, "    Software Version = %c.%c; ", sw[0], sw[1]);
  fprintf(stderr, "Hardware Version = %c.%c\n", hw[0], hw[1]);

  /* See if programmer supports autoincrement of address. */

  avr910_send(pgm, "a", 1);
  avr910_recv(pgm, &has_auto_incr_addr, 1);
  if (has_auto_incr_addr == 'Y')
      fprintf(stderr, "Programmer supports auto addr increment.\n");

  /* Get list of devices that the programmer supports. */

  avr910_send(pgm, "t", 1);
  fprintf(stderr, "\nProgrammer supports the following devices:\n");
  while (1) {
    avr910_recv(pgm, &c, 1);
    if (c == 0)
      break;
    fprintf(stderr, "    Device code: 0x%02x\n", c);

    /* FIXME: Need to lookup devcode and report the device. */

    if (p->avr910_devcode == c)
      dev_supported = 1;
  };
  fprintf(stderr,"\n");

  if (!dev_supported) {
    fprintf(stderr,
            "%s: error: selected device is not supported by programmer: %s\n",
            progname, p->id);
    exit(1);
  }

  /* Tell the programmer which part we selected. */

  buf[0] = 'T';
  buf[1] = p->avr910_devcode;

  avr910_send(pgm, buf, 2);
  avr910_vfy_cmd_sent(pgm, "select device");

  avr910_enter_prog_mode(pgm);

  return 0;
}


static int avr910_save(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return 0;
}


static void avr910_restore(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}


static void avr910_disable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}


static void avr910_enable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
static int avr910_cmd(PROGRAMMER * pgm, unsigned char cmd[4], 
                      unsigned char res[4])
{
  unsigned char buf[5];

  /* FIXME: Insert version check here */

  buf[0] = '.';                 /* New Universal Command */
  buf[1] = cmd[0];
  buf[2] = cmd[1];
  buf[3] = cmd[2];
  buf[4] = cmd[3];

  avr910_send (pgm, buf, 5);
  avr910_recv (pgm, buf, 2);

  res[0] = 0x00;                /* Dummy value */
  res[1] = cmd[0];
  res[2] = cmd[1];
  res[3] = buf[0];

  return 0;
}


static void avr910_open(PROGRAMMER * pgm, char * port)
{
  strcpy(pgm->port, port);
  pgm->fd = serial_open(port, 19200);

  /*
   * drain any extraneous input
   */
  avr910_drain (pgm, 0);
}

static void avr910_close(PROGRAMMER * pgm)
{
  avr910_leave_prog_mode(pgm);

  serial_close(pgm->fd);
  pgm->fd = -1;
}


static void avr910_display(PROGRAMMER * pgm, char * p)
{
  return;
}


static void avr910_set_addr(PROGRAMMER * pgm, unsigned long addr)
{
  unsigned char cmd[3];

  cmd[0] = 'A';
  cmd[1] = (addr >> 8) & 0xff;
  cmd[2] = addr & 0xff;
  
  avr910_send(pgm, cmd, sizeof(cmd));
  avr910_vfy_cmd_sent(pgm, "set addr");
}


/*
 * For some reason, if we don't do this when writing to flash, the first byte
 * of flash is not programmed. I susect that the board got out of sync after
 * the erase and sending another command gets us back in sync. -TRoth
 */
static void avr910_write_setup(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m)
{
  if (strcmp(m->desc, "flash") == 0) {
    avr910_send(pgm, "y", 1);
    avr910_vfy_cmd_sent(pgm, "clear LED");
  }
}


static int avr910_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                             unsigned long addr, unsigned char value)
{
  unsigned char cmd[2];

  if (strcmp(m->desc, "flash") == 0) {
    if (addr & 0x01) {
      cmd[0] = 'C';             /* Write Program Mem high byte */
    }
    else {
      cmd[0] = 'c';
    }

    addr >>= 1;
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    cmd[0] = 'D';
  }
  else {
    return -1;
  }

  cmd[1] = value;

  avr910_set_addr(pgm, addr);

  avr910_send(pgm, cmd, sizeof(cmd));
  avr910_vfy_cmd_sent(pgm, "write byte");

  return 0;
}


static int avr910_read_byte_flash(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                  unsigned long addr, unsigned char * value)
{
  static int cached = 0;
  static unsigned char cvalue;
  static unsigned long caddr;

  if (cached && ((caddr + 1) == addr)) {
    *value = cvalue;
    cached = 0;
  }
  else {
    unsigned char buf[2];

    avr910_set_addr(pgm, addr >> 1);

    avr910_send(pgm, "R", 1);

    /* Read back the program mem word (MSB first) */
    avr910_recv(pgm, buf, sizeof(buf));

    if ((addr & 0x01) == 0) {
      *value = buf[1];
      cached = 1;
      cvalue = buf[0];
      caddr = addr;
    }
    else {
      *value = buf[0];
    }
  }

  return 0;
}


static int avr910_read_byte_eeprom(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                   unsigned long addr, unsigned char * value)
{
  avr910_set_addr(pgm, addr);
  avr910_send(pgm, "d", 1);
  avr910_recv(pgm, value, 1);

  return 0;
}


static int avr910_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                            unsigned long addr, unsigned char * value)
{
  if (strcmp(m->desc, "flash") == 0) {
    return avr910_read_byte_flash(pgm, p, m, addr, value);
  }

  if (strcmp(m->desc, "eeprom") == 0) {
    return avr910_read_byte_eeprom(pgm, p, m, addr, value);
  }

  return -1;
}


static int avr910_paged_write_flash(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                                    int page_size, int n_bytes)
{
  unsigned char cmd[] = {'c', 'C'};
  unsigned char buf[2];
  unsigned int addr = 0;
  unsigned int max_addr = n_bytes;
  unsigned int page_addr;
  int page_bytes = page_size;
  int page_wr_cmd_pending = 0;

  avr910_write_setup(pgm, p, m);

  page_addr = addr;
  avr910_set_addr(pgm, addr>>1);

  while (addr < max_addr) {
    page_wr_cmd_pending = 1;
    buf[0] = cmd[addr & 0x01];
    buf[1] = m->buf[addr];
    avr910_send(pgm, buf, sizeof(buf));
    avr910_vfy_cmd_sent(pgm, "write byte");

    addr++;
    page_bytes--;

    if ((has_auto_incr_addr != 'Y') && ((addr & 0x01) == 0)) {
      avr910_set_addr(pgm, addr>>1);
    }
    else if (m->paged && (page_bytes == 0)) {
      /* Send the "Issue Page Write" if we have sent a whole page. */

      avr910_set_addr(pgm, page_addr>>1);
      avr910_send(pgm, "m", 1);
      avr910_vfy_cmd_sent(pgm, "flush page");

      /* Set page address for next page. */

      page_addr = addr;
      page_bytes = page_size;
    }

    report_progress (addr, max_addr, NULL);
  }

  /* If we didn't send the page wr cmd after the last byte written in the
     loop, send it now. */

  if (page_wr_cmd_pending) {
    avr910_set_addr(pgm, page_addr>>1);
    avr910_send(pgm, "m", 1);
    avr910_vfy_cmd_sent(pgm, "flush final page");
  }

  return addr;
}


static int avr910_paged_write_eeprom(PROGRAMMER * pgm, AVRPART * p,
                                     AVRMEM * m, int page_size, int n_bytes)
{
  unsigned char cmd[2];
  unsigned int addr = 0;
  unsigned int max_addr = n_bytes;

  avr910_set_addr(pgm, addr);

  cmd[0] = 'D';

  while (addr < max_addr) {
    cmd[1] = m->buf[addr];
    avr910_send(pgm, cmd, sizeof(cmd));
    avr910_vfy_cmd_sent(pgm, "write byte");

    addr++;

    if (has_auto_incr_addr != 'Y') {
      avr910_set_addr(pgm, addr);
    }

    report_progress (addr, max_addr, NULL);
  }

  return addr;
}


static int avr910_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  if (strcmp(m->desc, "flash") == 0) {
    return avr910_paged_write_flash(pgm, p, m, page_size, n_bytes);
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    return avr910_paged_write_eeprom(pgm, p, m, page_size, n_bytes);
  }
  else {
    return -2;
  }
}


static int avr910_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                             int page_size, int n_bytes)
{
  unsigned char cmd;
  int rd_size;
  unsigned int addr = 0;
  unsigned int max_addr;
  unsigned char buf[2];

  if (strcmp(m->desc, "flash") == 0) {
    cmd = 'R';
    rd_size = 2;                /* read two bytes per addr */
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    cmd = 'd';
    rd_size = 1;
  }
  else {
    return -2;
  }

  max_addr = n_bytes/rd_size;

  avr910_set_addr(pgm, addr);

  while (addr < max_addr) {
    avr910_send(pgm, &cmd, 1);
    if (cmd == 'R') {
      /* The 'R' command returns two bytes, MSB first, we need to put the data
         into the memory buffer LSB first. */
      avr910_recv(pgm, buf, 2);
      m->buf[addr*2]   = buf[1];  /* LSB */
      m->buf[addr*2+1] = buf[0];  /* MSB */
    }
    else {
      avr910_recv(pgm, &m->buf[addr], 1);
    }

    addr++;

    if (has_auto_incr_addr != 'Y') {
      avr910_set_addr(pgm, addr);
    }

    report_progress (addr, max_addr, NULL);
  }

  return addr * rd_size;
}

/* Signature byte reads are always 3 bytes. */

static int avr910_read_sig_bytes(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m)
{
  if (m->size < 3) {
    fprintf(stderr, "%s: memsize too small for sig byte read", progname);
    return -1;
  }

  avr910_send(pgm, "s", 1);
  avr910_recv(pgm, m->buf, 3);

  return 3;
}


void avr910_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "avr910");

  /*
   * mandatory functions
   */
  pgm->rdy_led        = avr910_rdy_led;
  pgm->err_led        = avr910_err_led;
  pgm->pgm_led        = avr910_pgm_led;
  pgm->vfy_led        = avr910_vfy_led;
  pgm->initialize     = avr910_initialize;
  pgm->display        = avr910_display;
  pgm->save           = avr910_save;
  pgm->restore        = avr910_restore;
  pgm->enable         = avr910_enable;
  pgm->disable        = avr910_disable;
  pgm->powerup        = avr910_powerup;
  pgm->powerdown      = avr910_powerdown;
  pgm->program_enable = avr910_program_enable;
  pgm->chip_erase     = avr910_chip_erase;
  pgm->cmd            = avr910_cmd;
  pgm->open           = avr910_open;
  pgm->close          = avr910_close;

  /*
   * optional functions
   */

  pgm->write_setup = avr910_write_setup;
  pgm->write_byte = avr910_write_byte;
  pgm->read_byte = avr910_read_byte;

  pgm->paged_write = avr910_paged_write;
  pgm->paged_load = avr910_paged_load;

  pgm->read_sig_bytes = avr910_read_sig_bytes;
}
