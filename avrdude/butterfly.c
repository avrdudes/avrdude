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
 * avrdude interface for the serial programming mode of the Atmel butterfly
 * evaluation board. This board features a bootloader which uses a protocol
 * very similar, but not identical, to the one described in application note
 * avr910.
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

  return serial_send(pgm->fd, buf, len);
}


static int butterfly_recv(PROGRAMMER * pgm, char * buf, size_t len)
{
  no_show_func_info();

  return serial_recv(pgm->fd, buf, len);
}


static int butterfly_drain(PROGRAMMER * pgm, int display)
{
  no_show_func_info();

  return serial_drain(pgm->fd, display);
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
 * initialize the AVR device and prepare it to accept commands
 */
static int butterfly_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  char id[8];
  char sw[2];
  char hw[2];
  char buf[10];
  char type;
  unsigned char c;
  int dev_supported = 0;

  no_show_func_info();

  /* send some ESC to activate butterfly bootloader */
  butterfly_send(pgm, "\033\033\033\033", 4);
  butterfly_drain(pgm, 0);

  /* Get the programmer identifier. Programmer returns exactly 7 chars
     _without_ the null.*/

  butterfly_send(pgm, "S", 1);
  memset (id, 0, sizeof(id));
  butterfly_recv(pgm, id, sizeof(id)-1);

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
            "a butterfly but a AVR910 device?\n", progname);
    exit(1);
  };
  butterfly_recv(pgm, &c, 1);
  buffersize = c<<8;
  butterfly_recv(pgm, &c, 1);
  buffersize += c;
  fprintf(stderr,
    "Programmer supports buffered memory access with buffersize=%i bytes.\n",
     buffersize);

  /* Get list of devices that the programmer supports. */

  butterfly_send(pgm, "t", 1);
  fprintf(stderr, "\nProgrammer supports the following devices:\n");
  while (1) {
    butterfly_recv(pgm, &c, 1);
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

  butterfly_send(pgm, buf, 2);
  butterfly_vfy_cmd_sent(pgm, "select device");

  butterfly_enter_prog_mode(pgm);

  return 0;
}


static void butterfly_disable(PROGRAMMER * pgm)
{
  no_show_func_info();

  /* Do nothing. */

  return;
}


static void butterfly_enable(PROGRAMMER * pgm)
{
  no_show_func_info();

  /* Do nothing. */

  return;
}


static int butterfly_open(PROGRAMMER * pgm, char * port)
{
  no_show_func_info();

  strcpy(pgm->port, port);
  pgm->fd = serial_open(port, 19200);

  /*
   * drain any extraneous input
   */
  butterfly_drain (pgm, 0);

  return 0;
}


static void butterfly_close(PROGRAMMER * pgm)
{
  no_show_func_info();

  butterfly_leave_prog_mode(pgm);

  serial_close(pgm->fd);
  pgm->fd = -1;
}


static void butterfly_display(PROGRAMMER * pgm, char * p)
{
  no_show_func_info();

  return;
}


static void butterfly_set_addr(PROGRAMMER * pgm, unsigned long addr)
{
  unsigned char cmd[3];

  cmd[0] = 'A';
  cmd[1] = (addr >> 8) & 0xff;
  cmd[2] = addr & 0xff;
  
  butterfly_send(pgm, cmd, sizeof(cmd));
  butterfly_vfy_cmd_sent(pgm, "set addr");
}



static int butterfly_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                             unsigned long addr, unsigned char value)
{
  unsigned char cmd[6];
  int size;

  no_show_func_info();

  if ((strcmp(m->desc, "flash") != 0) && (strcmp(m->desc, "eeprom") != 0))
    return -1;

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
  };

  butterfly_set_addr(pgm, addr);

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
    unsigned char buf[2];

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
  butterfly_recv(pgm, value, 1);
  return 0;
}


static int butterfly_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                            unsigned long addr, unsigned char * value)
{
  no_show_func_info();

  if (strcmp(m->desc, "flash") == 0) {
    return butterfly_read_byte_flash(pgm, p, m, addr, value);
  }

  if (strcmp(m->desc, "eeprom") == 0) {
    return butterfly_read_byte_eeprom(pgm, p, m, addr, value);
  }

  return -1;
}



static int butterfly_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  unsigned int addr = 0;
  unsigned int max_addr = n_bytes;
  unsigned char *cmd;
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
    unsigned char cmd[4];
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
      butterfly_recv(pgm, &m->buf[addr], blocksize);

      addr += blocksize;

      report_progress (addr, max_addr, NULL);
    } /* while */
  }

  return addr * rd_size;
}


/* Signature byte reads are always 3 bytes. */
static int butterfly_read_sig_bytes(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m)
{
  no_show_func_info();

  if (m->size < 3) {
    fprintf(stderr, "%s: memsize too small for sig byte read", progname);
    return -1;
  }

  butterfly_send(pgm, "s", 1);
  butterfly_recv(pgm, m->buf, 3);

  return 3;
}


void butterfly_initpgm(PROGRAMMER * pgm)
{
  no_show_func_info();

  strcpy(pgm->type, "avr910");

  /*
   * mandatory functions
   */
  pgm->initialize     = butterfly_initialize;
  pgm->display        = butterfly_display;
  pgm->enable         = butterfly_enable;
  pgm->disable        = butterfly_disable;
  pgm->program_enable = butterfly_program_enable;
  pgm->chip_erase     = butterfly_chip_erase;
/*  pgm->cmd		not supported, use default error message */
  pgm->open           = butterfly_open;
  pgm->close          = butterfly_close;

  /*
   * optional functions
   */

  pgm->write_byte = butterfly_write_byte;
  pgm->read_byte = butterfly_read_byte;

  pgm->paged_write = butterfly_paged_write;
  pgm->paged_load = butterfly_paged_load;

  pgm->read_sig_bytes = butterfly_read_sig_bytes;
}
