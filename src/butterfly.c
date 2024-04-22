/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * Copyright (C) 2005, 2007 Joerg Wunsch <j@uriah.heep.sax.de>
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


#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "butterfly.h"

/*
 * Private data for this programmer.
 */
struct pdata
{
  char has_auto_incr_addr;
  unsigned int buffersize;

  int ctype;                    // Cache one byte for flash
  unsigned char cvalue;
  unsigned long caddr;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

// Print error and return when command failed
#define EI(x) do {                 \
  int Eret = (x);                  \
  if(Eret < 0) {                   \
    pmsg_error("%s failed\n", #x); \
    return -1;                     \
  }                                \
} while(0)

#define EV(x) do {                 \
  int Eret = (x);                  \
  if(Eret < 0) {                   \
    pmsg_error("%s failed\n", #x); \
    return;                        \
  }                                \
} while(0)


static void butterfly_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof(struct pdata));
}

static void butterfly_teardown(PROGRAMMER *pgm) {
  mmt_free(pgm->cookie);
  pgm->cookie = NULL;
}

static int butterfly_send(const PROGRAMMER *pgm, char *buf, size_t len) {
  return serial_send(&pgm->fd, (unsigned char *)buf, len);
}


static int butterfly_recv(const PROGRAMMER *pgm, char *buf, size_t len) {
  return serial_recv(&pgm->fd, (unsigned char *) buf, len);
}


static int butterfly_drain(const PROGRAMMER *pgm, int display) {
  return serial_drain(&pgm->fd, display);
}


static int butterfly_vfy_cmd_sent(const PROGRAMMER *pgm, char *errmsg) {
  char c;

  EI(butterfly_recv(pgm, &c, 1));
  if (c != '\r') {
    pmsg_error("protocol error for command: %s\n", errmsg);
    return -1;
  }

  return 0;
}


static int butterfly_default_led(const PROGRAMMER *pgm, int value) {
  // No LED: do nothing
  return 0;
}


/*
 * issue the 'chip erase' command to the butterfly board
 */
static int butterfly_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  long bak_timeout = serial_recv_timeout;
  AVRMEM *fl = avr_locate_flash(p);
  int ret = 0;

  // Estimated time it takes to erase all pages in bootloader
  long new_timeout = p->chip_erase_delay * (fl? fl->num_pages: 999);
  if(serial_recv_timeout < new_timeout)
    serial_recv_timeout = new_timeout;

  EI(butterfly_send(pgm, "e", 1));
  if(butterfly_vfy_cmd_sent(pgm, "chip erase") < 0)
    ret = -1;

  serial_recv_timeout = bak_timeout;
  return ret;
}


static int butterfly_enter_prog_mode(const PROGRAMMER *pgm) {
  EI(butterfly_send(pgm, "P", 1));
  return butterfly_vfy_cmd_sent(pgm, "enter prog mode");
}


static void butterfly_leave_prog_mode(const PROGRAMMER *pgm) {
  EV(butterfly_send(pgm, "L", 1));
  butterfly_vfy_cmd_sent(pgm, "leave prog mode");
}


static int butterfly_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  return butterfly_enter_prog_mode(pgm);
}


/*
 * apply power to the AVR processor
 */
static void butterfly_powerup(const PROGRAMMER *pgm) {
  /* Do nothing. */

  return;
}


/*
 * remove power from the AVR processor
 */
static void butterfly_powerdown(const PROGRAMMER *pgm) {
  /* Do nothing. */

  return;
}

#define IS_BUTTERFLY_MK 0x0001

/*
 * initialize the AVR device and prepare it to accept commands
 */
static int butterfly_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  char id[8];
  char sw[2];
  char hw[2];
  char buf[10];
  char type;
  char c, devtype_1st;

  /*
   * Send some ESC to activate butterfly bootloader.  This is not needed
   * for plain avr109 bootloaders but does not harm there either.
   */
  msg_notice("connecting to programmer: ");
  if (pgm->flag & IS_BUTTERFLY_MK)
    {
      char mk_reset_cmd[6] = {"#aR@S\r"};
      unsigned char mk_timeout = 0;

      msg_notice(".");
      EI(butterfly_send(pgm, mk_reset_cmd, sizeof(mk_reset_cmd)));
      usleep(20000); 

      do
	{
	  c = 27; 
	  EI(butterfly_send(pgm, &c, 1));
	  usleep(20000);
	  c = 0xaa;
	  usleep(80000);
	  EI(butterfly_send(pgm, &c, 1));
	  if (mk_timeout % 10 == 0)
            msg_notice(".");
	} while (mk_timeout++ < 10);

      EI(butterfly_recv(pgm, &c, 1));
      if ( c != 'M' && c != '?')
        {
          msg_error("\n");
          pmsg_error("connection failed");
          return -1;
        }
      else
        {
	  id[0] = 'M'; id[1] = 'K'; id[2] = '2'; id[3] = 0;
	}
    }
  else
    {
      do {
	msg_notice(".");
	EI(butterfly_send(pgm, "\033", 1));
	(void) butterfly_drain(pgm, 0);
	EI(butterfly_send(pgm, "S", 1));
	EI(butterfly_recv(pgm, &c, 1));
	if (c != '?') {
	    msg_notice("\n");
	    /*
	     * Got a useful response, continue getting the programmer
	     * identifier. Programmer returns exactly 7 chars _without_
	     * the null.
	     */
	  id[0] = c;
	  EI(butterfly_recv(pgm, &id[1], sizeof(id)-2));
	  id[sizeof(id)-1] = '\0';
	}
      } while (c == '?');
    }

  /* Get the HW and SW versions to see if the programmer is present. */
  (void) butterfly_drain(pgm, 0);

  EI(butterfly_send(pgm, "V", 1));
  EI(butterfly_recv(pgm, sw, sizeof(sw)));

  EI(butterfly_send(pgm, "v", 1));
  EI(butterfly_recv(pgm, hw, 1)); // First, read only _one_ byte
  if (hw[0]!='?') {
    EI(butterfly_recv(pgm, &hw[1], 1)); // Now, read second byte
  };

  /* Get the programmer type (serial or parallel). Expect serial. */

  EI(butterfly_send(pgm, "p", 1));
  EI(butterfly_recv(pgm, &type, 1));

  msg_notice("Programmer id    = %s; type = %c\n", id, type);
  msg_notice("Software version = %c.%c; ", sw[0], sw[1]);
  if (hw[0]=='?') {
    msg_notice("no hardware version given\n");
  } else {
    msg_notice("Hardware version = %c.%c\n", hw[0], hw[1]);
  };

  /* See if programmer supports autoincrement of address. */

  EI(butterfly_send(pgm, "a", 1));
  EI(butterfly_recv(pgm, &PDATA(pgm)->has_auto_incr_addr, 1));
  if (PDATA(pgm)->has_auto_incr_addr == 'Y')
      msg_notice("programmer supports auto addr increment\n");

  /* Check support for buffered memory access, abort if not available */

  EI(butterfly_send(pgm, "b", 1));
  EI(butterfly_recv(pgm, &c, 1));
  if (c != 'Y') {
    pmsg_notice("buffered memory access not supported; maybe it isn't\n"\
      "a butterfly/AVR109 but a AVR910 device?\n");
    return -1;
  };
  EI(butterfly_recv(pgm, &c, 1));
  PDATA(pgm)->buffersize = (unsigned int)(unsigned char)c<<8;
  EI(butterfly_recv(pgm, &c, 1));
  PDATA(pgm)->buffersize += (unsigned int)(unsigned char)c;
  msg_notice("programmer supports buffered memory access with buffersize=%i bytes\n",
                  PDATA(pgm)->buffersize);

  /* Get list of devices that the programmer supports. */

  EI(butterfly_send(pgm, "t", 1));
  msg_notice2("\nProgrammer supports the following devices:\n");
  devtype_1st = 0;
  while (1) {
    EI(butterfly_recv(pgm, &c, 1));
    if (devtype_1st == 0)
      devtype_1st = c;

    if (c == 0)
      break;
    msg_notice2("    Device code: 0x%02x\n", (unsigned int) (unsigned char) c);
  };
  msg_notice2("\n");

  /* Tell the programmer which part we selected.
     According to the AVR109 code, this is ignored by the bootloader.  As
     some early versions might not properly ignore it, rather pick up the
     first device type as reported above than anything out of avrdude.conf,
     so to avoid a potential conflict.  There appears to be no general
     agreement on AVR910 device IDs beyond the ones from the original
     appnote 910. */

  buf[0] = 'T';
  buf[1] = devtype_1st;

  EI(butterfly_send(pgm, buf, 2));
  if (butterfly_vfy_cmd_sent(pgm, "select device") < 0)
      return -1;

  pmsg_notice("devcode selected: 0x%02x\n", (unsigned) buf[1]);

  if(pgm->program_enable(pgm, p) < 0)
    return -1;
  (void) butterfly_drain(pgm, 0);

  return 0;
}



static void butterfly_disable(const PROGRAMMER *pgm) {
  butterfly_leave_prog_mode(pgm);

  return;
}


static void butterfly_enable(PROGRAMMER *pgm, const AVRPART *p) {
  return;
}


static int butterfly_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;
  pgm->port = port;
  /*
   *  If baudrate was not specified use 19200 Baud
   */
  if(pgm->baudrate == 0) {
    pgm->baudrate = 19200;
  }
  pinfo.serialinfo.baud = pgm->baudrate;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  (void) butterfly_drain(pgm, 0);

  return 0;
}


static void butterfly_close(PROGRAMMER * pgm)
{
  /* "exit programmer" */
  EV(butterfly_send(pgm, "E", 1));
  butterfly_vfy_cmd_sent(pgm, "exit bootloader");

  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}


static void butterfly_display(const PROGRAMMER *pgm, const char *p) {
  return;
}


static void butterfly_set_addr(const PROGRAMMER *pgm, unsigned long addr) {
  if( addr < 0x10000 ) {
    char cmd[3];

    cmd[0] = 'A';
    cmd[1] = (addr >> 8) & 0xff;
    cmd[2] = addr & 0xff;
  
    EV(butterfly_send(pgm, cmd, sizeof(cmd)));
    butterfly_vfy_cmd_sent(pgm, "set addr");
  } else {
    char cmd[4];

    cmd[0] = 'H';
    cmd[1] = (addr >> 16) & 0xff;
    cmd[2] = (addr >> 8) & 0xff;
    cmd[3] = addr & 0xff;

    EV(butterfly_send(pgm, cmd, sizeof(cmd)));
    butterfly_vfy_cmd_sent(pgm, "set extaddr");
  }
}


static void butterfly_set_extaddr(const PROGRAMMER *pgm, unsigned long addr) {
  char cmd[4];

  cmd[0] = 'H';
  cmd[1] = (addr >> 16) & 0xff;
  cmd[2] = (addr >> 8) & 0xff;
  cmd[3] = addr & 0xff;

  EV(butterfly_send(pgm, cmd, sizeof(cmd)));
  butterfly_vfy_cmd_sent(pgm, "set extaddr");
}



static int butterfly_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                             unsigned long addr, unsigned char value)
{
  char cmd[6];
  int size;

  if(mem_is_flash(m)) {
    int ext_addr = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;

    PDATA(pgm)->ctype = 0;    // Invalidate read cache
    cmd[0] = 'B';
    cmd[1] = 0;
    cmd[2] = 2;
    cmd[3] = 'F';
    size = 6;
    (ext_addr? butterfly_set_extaddr: butterfly_set_addr)(pgm, addr >> 1);

    return -1;                  // @@@ not yet implemented (and what about usersig?)
  }

  if(mem_is_eeprom(m)) {
    cmd[0] = 'B';
    cmd[1] = 0;
    cmd[2] = 1;
    cmd[3] = 'E';
    cmd[4] = value;
    size = 5;
    butterfly_set_addr(pgm, addr);
  } else if(mem_is_lock(m)) {
    cmd[0] = 'l';
    cmd[1] = value;
    size = 2;
  } else if(mem_is_readonly(m)) {
    unsigned char is;
    if(pgm->read_byte(pgm, p, m, addr, &is) >= 0 && is == value)
      return 0;

    pmsg_error("cannot write to read-only memory %s of %s\n", m->desc, p->desc);
    return -1;
  }
  else
    return -1;

  EI(butterfly_send(pgm, cmd, size));
  if (butterfly_vfy_cmd_sent(pgm, "write byte") < 0)
      return -1;

  return 0;
}


static int butterfly_read_byte_flash(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                  unsigned long addr, unsigned char * value)
{
  int ext_addr = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;
  char mtype = mem_is_flash(m)? 'F': mem_is_sigrow(m)? 'P': mem_is_userrow(m)? 'U': '?';

  if(mtype == '?') {
    pmsg_error("cannot read memory %s\n", m->desc);
    return -1;
  }

  if(PDATA(pgm)->ctype == mtype && PDATA(pgm)->caddr == addr) {
    *value = PDATA(pgm)->cvalue;
    return 0;
  }

  char buf[2];                // Read word and cache the other byte
  char msg[4] = {'g', 0x00, 0x02, mtype};

  (ext_addr? butterfly_set_extaddr: butterfly_set_addr)(pgm, addr >> 1);
  EI(butterfly_send(pgm, msg, 4));
  EI(butterfly_recv(pgm, buf, sizeof(buf)));

  PDATA(pgm)->ctype = mtype;
  *value = buf[addr & 1];
  PDATA(pgm)->cvalue = buf[1 - (addr & 1)];
  PDATA(pgm)->caddr = addr ^ 1;

  return 0;
}


static int butterfly_read_byte_eeprom(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                   unsigned long addr, unsigned char * value)
{
  butterfly_set_addr(pgm, addr);
  EI(butterfly_send(pgm, "g\000\001E", 4));
  EI(butterfly_recv(pgm, (char *) value, 1));
  return 0;
}

static int butterfly_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                            unsigned long addr, unsigned char * value)
{
  char cmd;

  if (mem_is_flash(m) || mem_is_sigrow(m) || mem_is_userrow(m)) {
    return butterfly_read_byte_flash(pgm, p, m, addr, value);
  }

  if (mem_is_eeprom(m)) {
    return butterfly_read_byte_eeprom(pgm, p, m, addr, value);
  }

  if (mem_is_lfuse(m)) {
    cmd = 'F';
  }
  else if (mem_is_hfuse(m)) {
    cmd = 'N';
  }
  else if (mem_is_efuse(m)) {
    cmd = 'Q';
  }
  else if (mem_is_lock(m)) {
    cmd = 'r';
  }
  else
    return -1;

  EI(butterfly_send(pgm, &cmd, 1));
  EI(butterfly_recv(pgm, (char *) value, 1));

  return *value == '?'? -1: 0;
}



static int butterfly_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                 unsigned int page_size,
                                 unsigned int addr, unsigned int n_bytes)
{
  unsigned int max_addr = addr + n_bytes;
  char *cmd;
  unsigned int blocksize = PDATA(pgm)->buffersize;
  int ext_addr = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;
  int isee = mem_is_eeprom(m);

  if (!mem_is_flash(m) && !isee && !mem_is_userrow(m))
    return -2;

  if(isee)                      // Write single bytes to EEPROM
    blocksize = 1;
  else
    PDATA(pgm)->ctype = 0;      // Invalidate flash byte read cache

  (ext_addr? butterfly_set_extaddr: butterfly_set_addr)(pgm, isee? addr: addr>>1);

#if 0
  usleep(1000000);
  EI(butterfly_send(pgm, "y", 1));
  if (butterfly_vfy_cmd_sent(pgm, "clear LED") < 0)
    return -1;
#endif

  cmd = mmt_malloc(4+blocksize);

  cmd[0] = 'B';
  cmd[3] = isee? 'E': mem_is_flash(m)? 'F': 'U';

  while (addr < max_addr) {
    if ((max_addr - addr) < blocksize)
      blocksize = max_addr - addr;

    memcpy(&cmd[4], &m->buf[addr], blocksize);
    cmd[1] = (blocksize >> 8) & 0xff;
    cmd[2] = blocksize & 0xff;

    if(butterfly_send(pgm, cmd, 4+blocksize) < 0 ||
       butterfly_vfy_cmd_sent(pgm, "write block") < 0) {

      mmt_free(cmd);
      return -1;
    }

    addr += blocksize;
  }
  mmt_free(cmd);

  return n_bytes;
}



static int butterfly_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes)
{
  unsigned int max_addr = addr + n_bytes;
  int blocksize = PDATA(pgm)->buffersize;
  int ext_addr = m->op[AVR_OP_LOAD_EXT_ADDR] != NULL;
  int isee = mem_is_eeprom(m);

  // Only flash, EEPROM or usersig/userrow is allowed
  if (!mem_is_flash(m) && !isee && !mem_is_userrow(m))
    return -2;

  if(isee)                      // Read single bytes from EEPROM
    blocksize = 1;

  char cmd[4];

  cmd[0] = 'g';
  cmd[3] = isee? 'E': mem_is_flash(m)? 'F': 'U';

  (ext_addr? butterfly_set_extaddr: butterfly_set_addr)(pgm, isee? addr: addr>>1);

  while (addr < max_addr) {
    if ((max_addr - addr) < (unsigned int) blocksize)
      blocksize = max_addr - addr;

    cmd[1] = (blocksize >> 8) & 0xff;
    cmd[2] = blocksize & 0xff;

    EI(butterfly_send(pgm, cmd, 4));
    EI(butterfly_recv(pgm, (char *) &m->buf[addr], blocksize));

    addr += blocksize;
  }

  return n_bytes;
}


/* Signature byte reads are always 3 bytes. */
static int butterfly_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m) {
  unsigned char tmp;

  if (m->size < 3) {
    pmsg_error("memsize too small for sig byte read");
    return -1;
  }

  EI(butterfly_send(pgm, "s", 1));
  EI(butterfly_recv(pgm, (char *) m->buf, 3));
  /* Returned signature has wrong order. */
  tmp = m->buf[2];
  m->buf[2] = m->buf[0];
  m->buf[0] = tmp;

  return 3;
}

const char butterfly_desc[] = "Atmel Butterfly evaluation board; Atmel AppNotes AVR109, AVR911";

void butterfly_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "butterfly");

  /*
   * mandatory functions
   */
  pgm->rdy_led        = butterfly_default_led;
  pgm->err_led        = butterfly_default_led;
  pgm->pgm_led        = butterfly_default_led;
  pgm->vfy_led        = butterfly_default_led;
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

  pgm->setup          = butterfly_setup;
  pgm->teardown       = butterfly_teardown;
  pgm->flag = 0;
}

const char butterfly_mk_desc[] = "Mikrokopter.de Butterfly";

void butterfly_mk_initpgm(PROGRAMMER *pgm) {
  butterfly_initpgm(pgm);
  strcpy(pgm->type, "butterfly_mk");
  pgm->flag = IS_BUTTERFLY_MK;
}
