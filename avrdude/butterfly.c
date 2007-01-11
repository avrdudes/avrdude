/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * Copyright (C) 2005 Joerg Wunsch <j@uriah.heep.sax.de>
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
 * avrdude interface for the serial programming mode of the Atmel butterfly
 * evaluation board. This board features a bootloader which uses a protocol
 * very similar, but not identical, to the one described in application note
 * avr910.
 *
 * Actually, the butterfly uses a predecessor of the avr910 protocol
 * which is described in application notes avr109 (generic AVR
 * bootloader) and avr911 (opensource programmer).  This file now
 * fully handles the features present in avr109.  It should probably
 * be renamed to avr109, but we rather stick with the old name inside
 * the file.  We'll provide aliases for "avr109" and "avr911" in
 * avrdude.conf so users could call it by these name as well.
 */


#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "avr.h"
#include "pgm.h"
#include "butterfly.h"
#include "serial.h"

extern char * progname;
extern int do_cycles;

static char has_auto_incr_addr;
static unsigned buffersize = 0;

/* These two defines are only for debugging. Will remove them once it starts
   working. */

#define show_func_info() \
  fprintf(stderr, "%s: line %d: called %s()\n", \
          __FILE__, __LINE__, __FUNCTION__)

#define no_show_func_info()


static int butterfly_send(PROGRAMMER * pgm, char * buf, size_t len)
{
  no_show_func_info();

  return serial_send(&pgm->fd, (unsigned char *)buf, len);
}


static int butterfly_recv(PROGRAMMER * pgm, char * buf, size_t len)
{
  int rv;

  no_show_func_info();

  rv = serial_recv(&pgm->fd, (unsigned char *)buf, len);
  if (rv < 0) {
    fprintf(stderr,
	    "%s: butterfly_recv(): programmer is not responding\n",
	    progname);
    exit(1);
  }
  return 0;
}


static int butterfly_drain(PROGRAMMER * pgm, int display)
{
  no_show_func_info();

  return serial_drain(&pgm->fd, display);
}


static void butterfly_vfy_cmd_sent(PROGRAMMER * pgm, char * errmsg)
{
  char c;

  butterfly_recv(pgm, &c, 1);
  if (c != '\r') {
    fprintf(stderr, "%s: error: programmer did not respond to command: %s\n",
            progname, errmsg);
    exit(1);
  }
}


static int butterfly_rdy_led(PROGRAMMER * pgm, int value)
{
  no_show_func_info();

  /* Do nothing. */

  return 0;
}


static int butterfly_err_led(PROGRAMMER * pgm, int value)
{
  no_show_func_info();

  /* Do nothing. */

  return 0;
}


static int butterfly_pgm_led(PROGRAMMER * pgm, int value)
{
  no_show_func_info();

  /* Do nothing. */

  return 0;
}


static int butterfly_vfy_led(PROGRAMMER * pgm, int value)
{
  no_show_func_info();

  /* Do nothing. */

  return 0;
}


/*
 * issue the 'chip erase' command to the butterfly board
 */
static int butterfly_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  no_show_func_info();

  butterfly_send(pgm, "e", 1);
  butterfly_vfy_cmd_sent(pgm, "chip erase");

  return 0;
}


static void butterfly_enter_prog_mode(PROGRAMMER * pgm)
{
  butterfly_send(pgm, "P", 1);
  butterfly_vfy_cmd_sent(pgm, "enter prog mode");
}


static void butterfly_leave_prog_mode(PROGRAMMER * pgm)
{
  butterfly_send(pgm, "L", 1);
  butterfly_vfy_cmd_sent(pgm, "leave prog mode");
}


/*
 * issue the 'program enable' command to the AVR device
 */
static int butterfly_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  no_show_func_info();

  return -1;
}


/*
 * apply power to the AVR processor
 */
static void butterfly_powerup(PROGRAMMER * pgm)
{
  no_show_func_info();

  /* Do nothing. */

  return;
}


/*
 * remove power from the AVR processor
 */
static void butterfly_powerdown(PROGRAMMER * pgm)
{
  no_show_func_info();

  /* Do nothing. */

  return;
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int butterfly_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  char id[8];
  char sw[2];
  char hw[2];
  char buf[10];
  char type;
  char c, devtype_1st;

  no_show_func_info();

  /*
   * Send some ESC to activate butterfly bootloader.  This is not needed
   * for plain avr109 bootloaders but does not harm there either.
   */
  fprintf(stderr, "Connecting to programmer: ");
  do {
    putc('.', stderr);
    butterfly_send(pgm, "\033", 1);
    butterfly_drain(pgm, 0);
    butterfly_send(pgm, "S", 1);
    butterfly_recv(pgm, &c, 1);
    if (c != '?') {
        putc('\n', stderr);
        /*
         * Got a useful response, continue getting the programmer
         * identifier. Programmer returns exactly 7 chars _without_
         * the null.
         */
      id[0] = c;
      butterfly_recv(pgm, &id[1], sizeof(id)-2);
      id[sizeof(id)-1] = '\0';
    }
  } while (c == '?');

  /* Get the HW and SW versions to see if the programmer is present. */

  butterfly_send(pgm, "V", 1);
  butterfly_recv(pgm, sw, sizeof(sw));

  butterfly_send(pgm, "v", 1);
  butterfly_recv(pgm, hw, 1);	/* first, read only _one_ byte */
  if (hw[0]!='?') {
    butterfly_recv(pgm, &hw[1], 1);/* now, read second byte */
  };

  /* Get the programmer type (serial or parallel). Expect serial. */

  butterfly_send(pgm, "p", 1);
  butterfly_recv(pgm, &type, 1);

  fprintf(stderr, "Found programmer: Id = \"%s\"; type = %c\n", id, type);
  fprintf(stderr, "    Software Version = %c.%c; ", sw[0], sw[1]);
  if (hw[0]=='?') {
    fprintf(stderr, "No Hardware Version given.\n");
  } else {
    fprintf(stderr, "Hardware Version = %c.%c\n", hw[0], hw[1]);
  };

  /* See if programmer supports autoincrement of address. */

  butterfly_send(pgm, "a", 1);
  butterfly_recv(pgm, &has_auto_incr_addr, 1);
  if (has_auto_incr_addr == 'Y')
      fprintf(stderr, "Programmer supports auto addr increment.\n");

  /* Check support for buffered memory access, abort if not available */

  butterfly_send(pgm, "b", 1);
  butterfly_recv(pgm, &c, 1);
  if (c != 'Y') {
    fprintf(stderr,
            "%s: error: buffered memory access not supported. Maybe it isn't\n"\
            "a butterfly/AVR109 but a AVR910 device?\n", progname);
    exit(1);
  };
  butterfly_recv(pgm, &c, 1);
  buffersize = (unsigned int)(unsigned char)c<<8;
  butterfly_recv(pgm, &c, 1);
  buffersize += (unsigned int)(unsigned char)c;
  fprintf(stderr,
    "Programmer supports buffered memory access with buffersize=%i bytes.\n",
     buffersize);

  /* Get list of devices that the programmer supports. */

  butterfly_send(pgm, "t", 1);
  fprintf(stderr, "\nProgrammer supports the following devices:\n");
  devtype_1st = 0;
  while (1) {
    butterfly_recv(pgm, &c, 1);
    if (devtype_1st == 0)
      devtype_1st = c;

    if (c == 0)
      break;
    fprintf(stderr, "    Device code: 0x%02x\n", (unsigned int)(unsigned char)c);
  };
  fprintf(stderr,"\n");

  /* Tell the programmer which part we selected.
     According to the AVR109 code, this is ignored by the bootloader.  As
     some early versions might not properly ignore it, rather pick up the
     first device type as reported above than anything out of avrdude.conf,
     so to avoid a potential conflict.  There appears to be no general
     agreement on AVR910 device IDs beyond the ones from the original
     appnote 910. */

  buf[0] = 'T';
  buf[1] = devtype_1st;

  butterfly_send(pgm, buf, 2);
  butterfly_vfy_cmd_sent(pgm, "select device");

  butterfly_enter_prog_mode(pgm);

  return 0;
}



static void butterfly_disable(PROGRAMMER * pgm)
{
  no_show_func_info();

  butterfly_leave_prog_mode(pgm);

  return;
}


static void butterfly_enable(PROGRAMMER * pgm)
{
  no_show_func_info();

  return;
}


static int butterfly_open(PROGRAMMER * pgm, char * port)
{
  no_show_func_info();

  strcpy(pgm->port, port);
  /*
   *  If baudrate was not specified use 19200 Baud
   */
  if(pgm->baudrate == 0) {
    pgm->baudrate = 19200;
  }
  serial_open(port, pgm->baudrate, &pgm->fd);

  /*
   * drain any extraneous input
   */
  butterfly_drain (pgm, 0);

  return 0;
}


static void butterfly_close(PROGRAMMER * pgm)
{
  no_show_func_info();

  /* "exit programmer" */
  butterfly_send(pgm, "E", 1);
  butterfly_vfy_cmd_sent(pgm, "exit bootloader");

  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}


static void butterfly_display(PROGRAMMER * pgm, char * p)
{
  no_show_func_info();

  return;
}


static void butterfly_set_addr(PROGRAMMER * pgm, unsigned long addr)
{
  char cmd[3];

  cmd[0] = 'A';
  cmd[1] = (addr >> 8) & 0xff;
  cmd[2] = addr & 0xff;
  
  butterfly_send(pgm, cmd, sizeof(cmd));
  butterfly_vfy_cmd_sent(pgm, "set addr");
}



static int butterfly_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                             unsigned long addr, unsigned char value)
{
  char cmd[6];
  int size;

  no_show_func_info();

  if ((strcmp(m->desc, "flash") == 0) || (strcmp(m->desc, "eeprom") == 0))
  {
    cmd[0] = 'B';
    cmd[1] = 0;
    if ((cmd[3] = toupper(m->desc[0])) == 'E') {	/* write to eeprom */
      cmd[2] = 1;
      cmd[4] = value;
      size = 5;
    } else {						/* write to flash */
      /* @@@ not yet implemented */
      cmd[2] = 2;
      size = 6;
      return -1;
    }
    butterfly_set_addr(pgm, addr);
  }
  else if (strcmp(m->desc, "lock") == 0)
  {
    cmd[0] = 'l';
    cmd[1] = value;
    size = 2;
  }
  else
    return -1;

  butterfly_send(pgm, cmd, size);
  butterfly_vfy_cmd_sent(pgm, "write byte");

  return 0;
}


static int butterfly_read_byte_flash(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
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
    char buf[2];

    butterfly_set_addr(pgm, addr >> 1);

    butterfly_send(pgm, "g\000\002F", 4);

    /* Read back the program mem word (MSB first) */
    butterfly_recv(pgm, buf, sizeof(buf));

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


static int butterfly_read_byte_eeprom(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                   unsigned long addr, unsigned char * value)
{
  butterfly_set_addr(pgm, addr);
  butterfly_send(pgm, "g\000\001E", 4);
  butterfly_recv(pgm, (char *)value, 1);
  return 0;
}


static int butterfly_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                            unsigned long addr, unsigned char * value)
{
  char cmd;

  no_show_func_info();

  if (strcmp(m->desc, "flash") == 0) {
    return butterfly_read_byte_flash(pgm, p, m, addr, value);
  }

  if (strcmp(m->desc, "eeprom") == 0) {
    return butterfly_read_byte_eeprom(pgm, p, m, addr, value);
  }

  if (strcmp(m->desc, "lfuse") == 0) {
    cmd = 'F';
  }
  else if (strcmp(m->desc, "hfuse") == 0) {
    cmd = 'N';
  }
  else if (strcmp(m->desc, "efuse") == 0) {
    cmd = 'Q';
  }
  else if (strcmp(m->desc, "lock") == 0) {
    cmd = 'r';
  }
  else
    return -1;

  butterfly_send(pgm, &cmd, 1);
  butterfly_recv(pgm, (char *)value, 1);

  return *value == '?'? -1: 0;
}



static int butterfly_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  unsigned int addr = 0;
  unsigned int max_addr = n_bytes;
  char *cmd;
  unsigned int blocksize = buffersize;

  if (strcmp(m->desc, "flash") && strcmp(m->desc, "eeprom")) 
    return -2;

  if (m->desc[0] == 'e')
    blocksize = 1;		/* Write to eeprom single bytes only */

  butterfly_set_addr(pgm, addr);

#if 0
  usleep(1000000);
  butterfly_send(pgm, "y", 1);
  butterfly_vfy_cmd_sent(pgm, "clear LED");
#endif

  cmd = malloc(4+blocksize);
  if (!cmd) return -1;
  cmd[0] = 'B';
  cmd[3] = toupper(m->desc[0]);

  while (addr < max_addr) {
    if ((max_addr - addr) < blocksize) {
      blocksize = max_addr - addr;
    };
    memcpy(&cmd[4], &m->buf[addr], blocksize);
    cmd[1] = (blocksize >> 8) & 0xff;
    cmd[2] = blocksize & 0xff;

    butterfly_send(pgm, cmd, 4+blocksize);
    butterfly_vfy_cmd_sent(pgm, "write block");

    addr += blocksize;

    report_progress (addr, max_addr, NULL);
  } /* while */
  free(cmd);

  return addr;
}



static int butterfly_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                             int page_size, int n_bytes)
{
  unsigned int addr = 0;
  unsigned int max_addr = n_bytes;
  int rd_size = 1;

  /* check parameter syntax: only "flash" or "eeprom" is allowed */
  if (strcmp(m->desc, "flash") && strcmp(m->desc, "eeprom")) 
    return -2;

  {		/* use buffered mode */
    char cmd[4];
    int blocksize = buffersize;

    cmd[0] = 'g';
    cmd[3] = toupper(m->desc[0]);

    butterfly_set_addr(pgm, addr);
    while (addr < max_addr) {
      if ((max_addr - addr) < blocksize) {
        blocksize = max_addr - addr;
      };
      cmd[1] = (blocksize >> 8) & 0xff;
      cmd[2] = blocksize & 0xff;

      butterfly_send(pgm, cmd, 4);
      butterfly_recv(pgm, (char *)&m->buf[addr], blocksize);

      addr += blocksize;

      report_progress (addr, max_addr, NULL);
    } /* while */
  }

  return addr * rd_size;
}


/* Signature byte reads are always 3 bytes. */
static int butterfly_read_sig_bytes(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m)
{
  unsigned char tmp;

  no_show_func_info();

  if (m->size < 3) {
    fprintf(stderr, "%s: memsize too small for sig byte read", progname);
    return -1;
  }

  butterfly_send(pgm, "s", 1);
  butterfly_recv(pgm, (char *)m->buf, 3);
  /* Returned signature has wrong order. */
  tmp = m->buf[2];
  m->buf[2] = m->buf[0];
  m->buf[0] = tmp;

  return 3;
}


void butterfly_initpgm(PROGRAMMER * pgm)
{
  no_show_func_info();

  strcpy(pgm->type, "avr910");

  /*
   * mandatory functions
   */
  pgm->rdy_led        = butterfly_rdy_led;
  pgm->err_led        = butterfly_err_led;
  pgm->pgm_led        = butterfly_pgm_led;
  pgm->vfy_led        = butterfly_vfy_led;
  pgm->initialize     = butterfly_initialize;
  pgm->display        = butterfly_display;
  pgm->enable         = butterfly_enable;
  pgm->disable        = butterfly_disable;
  pgm->powerup        = butterfly_powerup;
  pgm->powerdown      = butterfly_powerdown;
  pgm->program_enable = butterfly_program_enable;
  pgm->chip_erase     = butterfly_chip_erase;
  pgm->open           = butterfly_open;
  pgm->close          = butterfly_close;
  pgm->read_byte      = butterfly_read_byte;
  pgm->write_byte     = butterfly_write_byte;

  /*
   * optional functions
   */

  pgm->paged_write = butterfly_paged_write;
  pgm->paged_load = butterfly_paged_load;

  pgm->read_sig_bytes = butterfly_read_sig_bytes;
}
