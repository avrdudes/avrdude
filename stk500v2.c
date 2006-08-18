/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005 Erik Walthinsen
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2006 David Moore
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
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
/* Based on Id: stk500.c,v 1.46 2004/12/22 01:52:45 bdean Exp */

/*
 * avrdude interface for Atmel STK500V2 programmer
 *
 * As the AVRISP mkII device is basically an STK500v2 one that can
 * only talk across USB, and that misses any kind of framing protocol,
 * this is handled here as well.
 *
 * Note: most commands use the "universal command" feature of the
 * programmer in a "pass through" mode, exceptions are "program
 * enable", "paged read", and "paged write".
 *
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "avr.h"
#include "pgm.h"
#include "stk500_private.h"	// temp until all code converted
#include "stk500v2_private.h"
#include "serial.h"
#include "usbdevs.h"

#define STK500V2_XTAL 7372800U

#if 0
#define DEBUG(format,args...) fprintf(stderr, format, ## args)
#else
#define DEBUG(format,args...)
#endif

#if 0
#define DEBUGRECV(format,args...) fprintf(stderr, format, ## args)
#else
#define DEBUGRECV(format,args...)
#endif

enum hvmode
{
  PPMODE, HVSPMODE
};


extern int    verbose;
extern char * progname;
extern int do_cycles;

/*
 * See stk500pp_read_byte() for an explanation of the flash and
 * EEPROM page caches.
 */
static unsigned char *flash_pagecache;
static unsigned long flash_pageaddr;
static unsigned int flash_pagesize;

static unsigned char *eeprom_pagecache;
static unsigned long eeprom_pageaddr;
static unsigned int eeprom_pagesize;

static unsigned char command_sequence = 1;
static int is_mk2;		/* Is the device an AVRISP mkII? */

static enum
{
  PGMTYPE_UNKNOWN,
  PGMTYPE_STK500,
  PGMTYPE_AVRISP,
  PGMTYPE_AVRISP_MKII,
}
pgmtype;

static const char *pgmname[] =
{
  "unknown",
  "STK500",
  "AVRISP",
  "AVRISP mkII",
};

static int stk500v2_getparm(PROGRAMMER * pgm, unsigned char parm, unsigned char * value);
static int stk500v2_setparm(PROGRAMMER * pgm, unsigned char parm, unsigned char value);
static void stk500v2_print_parms1(PROGRAMMER * pgm, char * p);
static int stk500v2_is_page_empty(unsigned int address, int page_size,
                                  const unsigned char *buf);

static unsigned int stk500v2_mode_for_pagesize(unsigned int pagesize);

#if defined(HAVE_LIBUSB)
static int stk500v2_set_sck_period_mk2(PROGRAMMER * pgm, double v);
#endif

static int stk500v2_send_mk2(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  if (serial_send(pgm->fd, data, len) != 0) {
    fprintf(stderr,"%s: stk500_send_mk2(): failed to send command to serial port\n",progname);
    exit(1);
  }

  return 0;
}

static int stk500v2_send(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  unsigned char buf[275 + 6];		// max MESSAGE_BODY of 275 bytes, 6 bytes overhead
  int i;

  if (is_mk2)
    return stk500v2_send_mk2(pgm, data, len);

  buf[0] = MESSAGE_START;
  buf[1] = command_sequence;
  buf[2] = len / 256;
  buf[3] = len % 256;
  buf[4] = TOKEN;
  memcpy(buf+5, data, len);

  // calculate the XOR checksum
  buf[5+len] = 0;
  for (i=0;i<5+len;i++)
    buf[5+len] ^= buf[i];

  DEBUG("STK500V2: stk500v2_send(");
  for (i=0;i<len+6;i++) DEBUG("0x%02x ",buf[i]);
  DEBUG(", %d)\n",len+6);

  if (serial_send(pgm->fd, buf, len+6) != 0) {
    fprintf(stderr,"%s: stk500_send(): failed to send command to serial port\n",progname);
    exit(1);
  }

  return 0;
}


static int stk500v2_drain(PROGRAMMER * pgm, int display)
{
  return serial_drain(pgm->fd, display);
}

static int stk500v2_recv_mk2(PROGRAMMER * pgm, unsigned char msg[],
			     size_t maxsize)
{
  int rv;

  rv = serial_recv(pgm->fd, msg, maxsize);
  if (rv < 0) {
    fprintf(stderr, "%s: stk500v2_recv_mk2: error in USB receive\n", progname);
    return -1;
  }

  return rv;
}

static int stk500v2_recv(PROGRAMMER * pgm, unsigned char msg[], size_t maxsize) {
  enum states { sINIT, sSTART, sSEQNUM, sSIZE1, sSIZE2, sTOKEN, sDATA, sCSUM, sDONE }  state = sSTART;
  int msglen = 0;
  int curlen = 0;
  int timeout = 0;
  unsigned char c, checksum = 0;

  long timeoutval = 5;		// seconds
  struct timeval tv;
  double tstart, tnow;

  if (is_mk2)
    return stk500v2_recv_mk2(pgm, msg, maxsize);

  DEBUG("STK500V2: stk500v2_recv(): ");

  gettimeofday(&tv, NULL);
  tstart = tv.tv_sec;

  while ( (state != sDONE ) && (!timeout) ) {
    if (serial_recv(pgm->fd, &c, 1) < 0)
      goto timedout;
    DEBUG("0x%02x ",c);
    checksum ^= c;

    switch (state) {
      case sSTART:
        DEBUGRECV("hoping for start token...");
        if (c == MESSAGE_START) {
          DEBUGRECV("got it\n");
          checksum = MESSAGE_START;
          state = sSEQNUM;
        } else
          DEBUGRECV("sorry\n");
        break;
      case sSEQNUM:
        DEBUGRECV("hoping for sequence...\n");
        if (c == command_sequence) {
          DEBUGRECV("got it, incrementing\n");
          state = sSIZE1;
          command_sequence++;
        } else {
          DEBUGRECV("sorry\n");
          state = sSTART;
        }
        break;
      case sSIZE1:
        DEBUGRECV("hoping for size LSB\n");
        msglen = c*256;
        state = sSIZE2;
        break;
      case sSIZE2:
        DEBUGRECV("hoping for size MSB...");
        msglen += c;
        DEBUG(" msg is %d bytes\n",msglen);
        state = sTOKEN;
        break;
      case sTOKEN:
        if (c == TOKEN) state = sDATA;
        else state = sSTART;
        break;
      case sDATA:
        if (curlen < maxsize) {
          msg[curlen] = c;
        } else {
          fprintf(stderr, "%s: stk500v2_recv(): buffer too small, received %d byte into %zd byte buffer\n",
                  progname,curlen,maxsize);
          return -2;
        }
        if ((curlen == 0) && (msg[0] == ANSWER_CKSUM_ERROR)) {
          fprintf(stderr, "%s: stk500v2_recv(): previous packet sent with wrong checksum\n",
                  progname);
          return -3;
        }
        curlen++;
        if (curlen == msglen) state = sCSUM;
        break;
      case sCSUM:
        if (checksum == 0) {
          state = sDONE;
        } else {
          state = sSTART;
          fprintf(stderr, "%s: stk500v2_recv(): checksum error\n",
                  progname);
          return -4;
        }
        break;
      default:
        fprintf(stderr, "%s: stk500v2_recv(): unknown state\n",
                progname);
        return -5;
     } /* switch */

     gettimeofday(&tv, NULL);
     tnow = tv.tv_sec;
     if (tnow-tstart > timeoutval) {			// wuff - signed/unsigned/overflow
      timedout:
       fprintf(stderr, "%s: stk500_2_ReceiveMessage(): timeout\n",
               progname);
       return -1;
     }

  } /* while */
  DEBUG("\n");

  return msglen+6;
}



static int stk500v2_getsync(PROGRAMMER * pgm) {
  int tries = 0;
  unsigned char buf[1], resp[32];
  int status;

  DEBUG("STK500V2: stk500v2_getsync()\n");

retry:
  tries++;

  // send the sync command and see if we can get there
  buf[0] = CMD_SIGN_ON;
  stk500v2_send(pgm, buf, 1);

  // try to get the response back and see where we got
  status = stk500v2_recv(pgm, resp, sizeof(resp));

  // if we got bytes returned, check to see what came back
  if (status > 0) {
    if ((resp[0] == CMD_SIGN_ON) && (resp[1] == STATUS_CMD_OK) &&
	(status > 3)) {
      // success!
      unsigned int siglen = resp[2];
      if (siglen >= strlen("STK500_2") &&
	  memcmp(resp + 3, "STK500_2", strlen("STK500_2")) == 0) {
	pgmtype = PGMTYPE_STK500;
      } else if (siglen >= strlen("AVRISP_2") &&
		 memcmp(resp + 3, "AVRISP_2", strlen("AVRISP_2")) == 0) {
	pgmtype = PGMTYPE_AVRISP;
      } else if (siglen >= strlen("AVRISP_MK2") &&
		 memcmp(resp + 3, "AVRISP_MK2", strlen("AVRISP_MK2")) == 0) {
	pgmtype = PGMTYPE_AVRISP_MKII;
      } else {
	resp[siglen + 3] = 0;
	if (verbose)
	  fprintf(stderr,
		  "%s: stk500v2_getsync(): got response from unknown "
		  "programmer %s, assuming STK500\n",
		  progname, resp + 3);
	pgmtype = PGMTYPE_STK500;
      }
      if (verbose >= 2)
	fprintf(stderr,
		"%s: stk500v2_getsync(): found %s programmer\n",
		progname, pgmname[pgmtype]);
      return 0;
    } else {
      if (tries > 33) {
        fprintf(stderr,
                "%s: stk500v2_getsync(): can't communicate with device: resp=0x%02x\n",
                progname, resp[0]);
        return -6;
      } else
        goto retry;
    }

  // or if we got a timeout
  } else if (status == -1) {
    if (tries > 33) {
      fprintf(stderr,"%s: stk500v2_getsync(): timeout communicating with programmer\n",
              progname);
      return -1;
    } else
      goto retry;

  // or any other error
  } else {
    if (tries > 33) {
      fprintf(stderr,"%s: stk500v2_getsync(): error communicating with programmer: (%d)\n",
              progname,status);
    } else
      goto retry;
  }

  return 0;
}

static int stk500v2_command
(PROGRAMMER * pgm, unsigned char * buf, size_t len, size_t maxlen) {
  int i;
  int tries = 0;
  int status;

  DEBUG("STK500V2: stk500v2_command(");
  for (i=0;i<len;i++) DEBUG("0x%02hhx ",buf[i]);
  DEBUG(", %d)\n",len);

retry:
  tries++;

  // send the command to the programmer
  stk500v2_send(pgm,buf,len);

  // attempt to read the status back
  status = stk500v2_recv(pgm,buf,maxlen);

  // if we got a successful readback, return
  if (status > 0) {
    DEBUG(" = %d\n",status);
    return status;
  }

  // otherwise try to sync up again
  status = stk500v2_getsync(pgm);
  if (status != 0) {
    if (tries > 33) {
      fprintf(stderr,"%s: stk500v2_command(): failed miserably to execute command 0x%02x\n",
              progname,buf[0]);
      return -1;
    } else
      goto retry;
  }

  DEBUG(" = 0\n");
  return 0;
}

static int stk500v2_cmd(PROGRAMMER * pgm, unsigned char cmd[4],
                        unsigned char res[4])
{
  unsigned char buf[8];
  int result;

  DEBUG("STK500V2: stk500v2_cmd(%02x,%02x,%02x,%02x)\n",cmd[0],cmd[1],cmd[2],cmd[3]);

  buf[0] = CMD_SPI_MULTI;
  buf[1] = 4;
  buf[2] = 4;
  buf[3] = 0;
  buf[4] = cmd[0];
  buf[5] = cmd[1];
  buf[6] = cmd[2];
  buf[7] = cmd[3];

  result = stk500v2_command(pgm, buf, 8, sizeof(buf));
  if (buf[1] != STATUS_CMD_OK) {
    fprintf(stderr, "%s: stk500v2_cmd(): failed to send command\n",
            progname);
    return -1;
  }

  res[0] = buf[2];
  res[1] = buf[3];
  res[2] = buf[4];
  res[3] = buf[5];

  return 0;
}


static int stk500hv_cmd(PROGRAMMER * pgm, unsigned char cmd[4],
                        unsigned char res[4])
{

  fprintf(stderr, "%s: stk500hv_command(): no direct SPI supported for PP mode\n",
	  progname);
  return -1;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
static int stk500v2_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  int result;
  unsigned char buf[16];

  if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
    fprintf(stderr, "%s: stk500v2_chip_erase: chip erase instruction not defined for part \"%s\"\n",
            progname, p->desc);
    return -1;
  }

  pgm->pgm_led(pgm, ON);

  buf[0] = CMD_CHIP_ERASE_ISP;
  buf[1] = p->chip_erase_delay / 1000;
  buf[2] = 0;	// use delay (?)
  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], buf+3);
  result = stk500v2_command(pgm, buf, 7, sizeof(buf));
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  pgm->pgm_led(pgm, OFF);

  return result;
}

/*
 * issue the 'chip erase' command to the AVR device, generic HV mode
 */
static int stk500hv_chip_erase(PROGRAMMER * pgm, AVRPART * p, enum hvmode mode)
{
  int result;
  unsigned char buf[3];

  pgm->pgm_led(pgm, ON);

  if (mode == PPMODE) {
    buf[0] = CMD_CHIP_ERASE_PP;
    buf[1] = p->chiperasepulsewidth;
    buf[2] = p->chiperasepolltimeout;
  } else {
    buf[0] = CMD_CHIP_ERASE_HVSP;
    buf[1] = p->chiperasepolltimeout;
    buf[2] = p->chiperasetime;
  }
  result = stk500v2_command(pgm, buf, 3, sizeof(buf));
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  pgm->pgm_led(pgm, OFF);

  return result;
}

/*
 * issue the 'chip erase' command to the AVR device, parallel mode
 */
static int stk500pp_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  return stk500hv_chip_erase(pgm, p, PPMODE);
}

/*
 * issue the 'chip erase' command to the AVR device, HVSP mode
 */
static int stk500hvsp_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  return stk500hv_chip_erase(pgm, p, HVSPMODE);
}

/*
 * issue the 'program enable' command to the AVR device
 */
static int stk500v2_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[16];

  if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
    fprintf(stderr, "%s: stk500v2_program_enable(): program enable instruction not defined for part \"%s\"\n",
            progname, p->desc);
    return -1;
  }

  buf[0] = CMD_ENTER_PROGMODE_ISP;
  buf[1] = p->timeout;
  buf[2] = p->stabdelay;
  buf[3] = p->cmdexedelay;
  buf[4] = p->synchloops;
  buf[5] = p->bytedelay;
  buf[6] = p->pollvalue;
  buf[7] = p->pollindex;
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], buf+8);

  return stk500v2_command(pgm, buf, 12, sizeof(buf));
}

/*
 * issue the 'program enable' command to the AVR device, parallel mode
 */
static int stk500pp_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[16];

  buf[0] = CMD_ENTER_PROGMODE_PP;
  buf[1] = p->hventerstabdelay;
  buf[2] = p->progmodedelay;
  buf[3] = p->latchcycles;
  buf[4] = p->togglevtg;
  buf[5] = p->poweroffdelay;
  buf[6] = p->resetdelayms;
  buf[7] = p->resetdelayus;

  return stk500v2_command(pgm, buf, 8, sizeof(buf));
}

/*
 * issue the 'program enable' command to the AVR device, HVSP mode
 */
static int stk500hvsp_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[16];

  buf[0] = CMD_ENTER_PROGMODE_HVSP;
  buf[1] = p->hventerstabdelay;
  buf[2] = p->hvspcmdexedelay;
  buf[3] = p->synchcycles;
  buf[4] = p->latchcycles;
  buf[5] = p->togglevtg;
  buf[6] = p->poweroffdelay;
  buf[7] = p->resetdelayms;
  buf[8] = p->resetdelayus;

  return stk500v2_command(pgm, buf, 9, sizeof(buf));
}



/*
 * initialize the AVR device and prepare it to accept commands
 */
static int stk500v2_initialize(PROGRAMMER * pgm, AVRPART * p)
{

  pgmtype = PGMTYPE_UNKNOWN;

  return pgm->program_enable(pgm, p);
}


/*
 * initialize the AVR device and prepare it to accept commands, generic HV mode
 */
static int stk500hv_initialize(PROGRAMMER * pgm, AVRPART * p, enum hvmode mode)
{
  unsigned char buf[CTL_STACK_SIZE + 1];
  int result;
  LNODEID ln;
  AVRMEM * m;

  if (p->ctl_stack_type != (mode == PPMODE? CTL_STACK_PP: CTL_STACK_HVSP)) {
    fprintf(stderr,
	    "%s: stk500hv_initialize(): "
	    "%s programming control stack not defined for part \"%s\"\n",
            progname,
	    (mode == PPMODE? "parallel": "high-voltage serial"),
	    p->desc);
    return -1;
  }

  buf[0] = CMD_SET_CONTROL_STACK;
  memcpy(buf + 1, p->controlstack, CTL_STACK_SIZE);

  result = stk500v2_command(pgm, buf, CTL_STACK_SIZE + 1, sizeof(buf));

  if (result < 0 || buf[1] != STATUS_CMD_OK) {
    fprintf(stderr,
	    "%s: stk500pp_initalize(): "
	    "failed to set control stack, got 0x%02x\n",
            progname, buf[1]);
    return -1;
  }

  /*
   * Examine the avrpart's memory definitions, and initialize the page
   * caches.  For devices/memory that are not page oriented, treat
   * them as page size 1 for EEPROM, and 2 for flash.
   */
  flash_pagesize = 2;
  eeprom_pagesize = 1;
  for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
    m = ldata(ln);
    if (strcmp(m->desc, "flash") == 0) {
      if (m->page_size > 0)
	flash_pagesize = m->page_size;
    } else if (strcmp(m->desc, "eeprom") == 0) {
      if (m->page_size > 0)
	eeprom_pagesize = m->page_size;
    }
  }
  free(flash_pagecache);
  free(eeprom_pagecache);
  if ((flash_pagecache = malloc(flash_pagesize)) == NULL) {
    fprintf(stderr, "%s: stk500pp_initialize(): Out of memory\n",
	    progname);
    return -1;
  }
  if ((eeprom_pagecache = malloc(eeprom_pagesize)) == NULL) {
    fprintf(stderr, "%s: stk500pp_initialize(): Out of memory\n",
	    progname);
    free(flash_pagecache);
    return -1;
  }
  flash_pageaddr = eeprom_pageaddr = (unsigned long)-1L;

  pgmtype = PGMTYPE_UNKNOWN;

  return pgm->program_enable(pgm, p);
}

/*
 * initialize the AVR device and prepare it to accept commands, PP mode
 */
static int stk500pp_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  return stk500hv_initialize(pgm, p, PPMODE);
}

/*
 * initialize the AVR device and prepare it to accept commands, HVSP mode
 */
static int stk500hvsp_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  return stk500hv_initialize(pgm, p, HVSPMODE);
}

static void stk500v2_disable(PROGRAMMER * pgm)
{
  unsigned char buf[16];
  int result;

  buf[0] = CMD_LEAVE_PROGMODE_ISP;
  buf[1] = 1; // preDelay;
  buf[2] = 1; // postDelay;

  result = stk500v2_command(pgm, buf, 3, sizeof(buf));

  if (buf[1] != STATUS_CMD_OK) {
    fprintf(stderr, "%s: stk500v2_disable(): failed to leave programming mode, got 0x%02x\n",
            progname,buf[1]);
    exit(1);
  }

  return;
}

/*
 * Leave programming mode, generic HV mode
 */
static void stk500hv_disable(PROGRAMMER * pgm, enum hvmode mode)
{
  unsigned char buf[16];
  int result;

  free(flash_pagecache);
  flash_pagecache = NULL;
  free(eeprom_pagecache);
  eeprom_pagecache = NULL;

  buf[0] = mode == PPMODE? CMD_LEAVE_PROGMODE_PP: CMD_LEAVE_PROGMODE_HVSP;
  buf[1] = 15;  // p->hvleavestabdelay;
  buf[2] = 15;  // p->resetdelay;

  result = stk500v2_command(pgm, buf, 3, sizeof(buf));

  if (result < 0 || buf[1] != STATUS_CMD_OK) {
    fprintf(stderr,
	    "%s: stk500hv_disable(): "
	    "failed to leave programming mode, got 0x%02x\n",
            progname,buf[1]);
    exit(1);
  }

  return;
}

/*
 * Leave programming mode, PP mode
 */
static void stk500pp_disable(PROGRAMMER * pgm)
{
  stk500hv_disable(pgm, PPMODE);
}

/*
 * Leave programming mode, HVSP mode
 */
static void stk500hvsp_disable(PROGRAMMER * pgm)
{
  stk500hv_disable(pgm, HVSPMODE);
}

static void stk500v2_enable(PROGRAMMER * pgm)
{
  return;
}


static int stk500v2_open(PROGRAMMER * pgm, char * port)
{
  long baud = 115200;

  DEBUG("STK500V2: stk500v2_open()\n");

  if (pgm->baudrate)
    baud = pgm->baudrate;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev_frame;
    baud = USB_DEVICE_AVRISPMKII;
    is_mk2 = 1;
    pgm->set_sck_period = stk500v2_set_sck_period_mk2;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  pgm->fd = serial_open(port, baud);

  /*
   * drain any extraneous input
   */
  stk500v2_drain(pgm, 0);

  stk500v2_getsync(pgm);

  stk500v2_drain(pgm, 0);

  if (pgm->bitclock != 0.0) {
    if (pgm->set_sck_period(pgm, pgm->bitclock) != 0)
      return -1;
  }

  return 0;
}


static void stk500v2_close(PROGRAMMER * pgm)
{
  DEBUG("STK500V2: stk500v2_close()\n");

  serial_close(pgm->fd);
  pgm->fd = -1;
}


static int stk500v2_loadaddr(PROGRAMMER * pgm, unsigned int addr)
{
  unsigned char buf[16];
  int result;

  DEBUG("STK500V2: stk500v2_loadaddr(%d)\n",addr);

  buf[0] = CMD_LOAD_ADDRESS;
  buf[1] = (addr >> 24) & 0xff;
  buf[2] = (addr >> 16) & 0xff;
  buf[3] = (addr >> 8) & 0xff;
  buf[4] = addr & 0xff;

  result = stk500v2_command(pgm, buf, 5, sizeof(buf));

  if (buf[1] != STATUS_CMD_OK) {
    fprintf(stderr, "%s: stk500v2_loadaddr(): failed to set load address, got 0x%02x\n",
            progname,buf[1]);
    return -1;
  }

  return 0;
}


/*
 * Read a single byte, generic HV mode
 */
static int stk500hv_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			      unsigned long addr, unsigned char * value,
			      enum hvmode mode)
{
  int result, cmdlen = 2;
  char buf[266];
  unsigned long paddr = 0UL, *paddr_ptr = NULL;
  unsigned int pagesize = 0, use_ext_addr = 0, addrshift = 0;
  unsigned char *cache_ptr = NULL;

  if (verbose >= 2)
    fprintf(stderr, "%s: stk500hv_read_byte(.., %s, 0x%lx, ...)\n",
	    progname, mem->desc, addr);

  if (strcmp(mem->desc, "flash") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_FLASH_PP: CMD_READ_FLASH_HVSP;
    cmdlen = 3;
    pagesize = mem->page_size;
    if (pagesize == 0)
      pagesize = 2;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &flash_pageaddr;
    cache_ptr = flash_pagecache;
    addrshift = 1;
    /*
     * If bit 31 is set, this indicates that the following read/write
     * operation will be performed on a memory that is larger than
     * 64KBytes. This is an indication to STK500 that a load extended
     * address must be executed.
     */
    if (mem->op[AVR_OP_LOAD_EXT_ADDR] != NULL) {
      use_ext_addr = (1U << 31);
    }
  } else if (strcmp(mem->desc, "eeprom") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_EEPROM_PP: CMD_READ_EEPROM_HVSP;
    cmdlen = 3;
    pagesize = mem->page_size;
    if (pagesize == 0)
      pagesize = 1;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &eeprom_pageaddr;
    cache_ptr = eeprom_pagecache;
  } else if (strcmp(mem->desc, "lfuse") == 0 ||
	     strcmp(mem->desc, "fuse") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_FUSE_PP: CMD_READ_FUSE_HVSP;
    addr = 0;
  } else if (strcmp(mem->desc, "hfuse") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_FUSE_PP: CMD_READ_FUSE_HVSP;
    addr = 1;
  } else if (strcmp(mem->desc, "efuse") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_FUSE_PP: CMD_READ_FUSE_HVSP;
    addr = 2;
  } else if (strcmp(mem->desc, "lock") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_LOCK_PP: CMD_READ_LOCK_HVSP;
  } else if (strcmp(mem->desc, "calibration") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_OSCCAL_PP: CMD_READ_OSCCAL_HVSP;
  } else if (strcmp(mem->desc, "signature") == 0) {
    buf[0] = mode == PPMODE? CMD_READ_SIGNATURE_PP: CMD_READ_SIGNATURE_HVSP;
  }

  /*
   * In HV mode, we have to use paged reads for flash and
   * EEPROM, and cache the results in a page cache.
   *
   * Page cache validation is based on "{flash,eeprom}_pageaddr"
   * (holding the base address of the most recent cache fill
   * operation).  This variable is set to (unsigned long)-1L when the
   * cache needs to be invalidated.
   */
  if (pagesize && paddr == *paddr_ptr) {
    *value = cache_ptr[addr & (pagesize - 1)];
    return 0;
  }

  if (cmdlen == 3) {
    /* long command, fill in # of bytes */
    buf[1] = (pagesize >> 8) & 0xff;
    buf[2] = pagesize & 0xff;

    /* flash and EEPROM reads require the load address command */
    stk500v2_loadaddr(pgm, use_ext_addr | (paddr >> addrshift));
  } else {
    buf[1] = addr;
  }

  if (verbose >= 2)
    fprintf(stderr, "%s: stk500hv_read_byte(): Sending read memory command: ",
	    progname);

  result = stk500v2_command(pgm, buf, cmdlen, sizeof(buf));

  if (result < 0 || buf[1] != STATUS_CMD_OK) {
    fprintf(stderr,
	    "%s: stk500hv_read_byte(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, result);
    return -1;
  }

  if (pagesize) {
    *paddr_ptr = paddr;
    memcpy(cache_ptr, buf + 2, pagesize);
    *value = cache_ptr[addr & (pagesize - 1)];
  } else {
    *value = buf[2];
  }

  return 0;
}

/*
 * Read a single byte, PP mode
 */
static int stk500pp_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			      unsigned long addr, unsigned char * value)
{
  return stk500hv_read_byte(pgm, p, mem, addr, value, PPMODE);
}

/*
 * Read a single byte, HVSP mode
 */
static int stk500hvsp_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
				unsigned long addr, unsigned char * value)
{
  return stk500hv_read_byte(pgm, p, mem, addr, value, HVSPMODE);
}

/*
 * Write one byte, generic HV mode
 */
static int stk500hv_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			       unsigned long addr, unsigned char data,
			       enum hvmode mode)
{
  int result, cmdlen, timeout = 0, pulsewidth = 0;
  char buf[266];
  unsigned long paddr = 0UL, *paddr_ptr = NULL;
  unsigned int pagesize = 0, use_ext_addr = 0, addrshift = 0;
  unsigned char *cache_ptr = NULL;

  if (verbose >= 2)
    fprintf(stderr, "%s: stk500hv_write_byte(.., %s, 0x%lx, ...)\n",
	    progname, mem->desc, addr);

  if (strcmp(mem->desc, "flash") == 0) {
    buf[0] = mode == PPMODE? CMD_PROGRAM_FLASH_PP: CMD_PROGRAM_FLASH_HVSP;
    pagesize = mem->page_size;
    if (pagesize == 0)
      pagesize = 2;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &flash_pageaddr;
    cache_ptr = flash_pagecache;
    addrshift = 1;
    /*
     * If bit 31 is set, this indicates that the following read/write
     * operation will be performed on a memory that is larger than
     * 64KBytes. This is an indication to STK500 that a load extended
     * address must be executed.
     */
    if (mem->op[AVR_OP_LOAD_EXT_ADDR] != NULL) {
      use_ext_addr = (1U << 31);
    }
  } else if (strcmp(mem->desc, "eeprom") == 0) {
    buf[0] = mode == PPMODE? CMD_PROGRAM_EEPROM_PP: CMD_PROGRAM_EEPROM_HVSP;
    pagesize = mem->page_size;
    if (pagesize == 0)
      pagesize = 1;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &eeprom_pageaddr;
    cache_ptr = eeprom_pagecache;
  } else if (strcmp(mem->desc, "lfuse") == 0 ||
	     strcmp(mem->desc, "fuse") == 0) {
    buf[0] = mode == PPMODE? CMD_PROGRAM_FUSE_PP: CMD_PROGRAM_FUSE_HVSP;
    addr = 0;
    pulsewidth = p->programfusepulsewidth;
    timeout = p->programfusepolltimeout;
  } else if (strcmp(mem->desc, "hfuse") == 0) {
    buf[0] = mode == PPMODE? CMD_PROGRAM_FUSE_PP: CMD_PROGRAM_FUSE_HVSP;
    addr = 1;
    pulsewidth = p->programfusepulsewidth;
    timeout = p->programfusepolltimeout;
  } else if (strcmp(mem->desc, "efuse") == 0) {
    buf[0] = mode == PPMODE? CMD_PROGRAM_FUSE_PP: CMD_PROGRAM_FUSE_HVSP;
    addr = 2;
    pulsewidth = p->programfusepulsewidth;
    timeout = p->programfusepolltimeout;
  } else if (strcmp(mem->desc, "lock") == 0) {
    buf[0] = mode == PPMODE? CMD_PROGRAM_LOCK_PP: CMD_PROGRAM_LOCK_HVSP;
    pulsewidth = p->programlockpulsewidth;
    timeout = p->programlockpolltimeout;
  } else {
    fprintf(stderr,
	    "%s: stk500hv_write_byte(): "
	    "unsupported memory type: %s\n",
	    progname, mem->desc);
    return -1;
  }

  cmdlen = 5 + pagesize;

  /*
   * In HV mode, we have to use paged writes for flash and
   * EEPROM.  As both, flash and EEPROM cells can only be programmed
   * from `1' to `0' bits (even EEPROM does not support auto-erase in
   * parallel mode), we just pre-fill the page cache with 0xff, so all
   * those cells that are outside our current address will remain
   * unaffected.
   */
  if (pagesize) {
    memset(cache_ptr, 0xff, pagesize);
    cache_ptr[addr & (pagesize - 1)] = data;

    /* long command, fill in # of bytes */
    buf[1] = (pagesize >> 8) & 0xff;
    buf[2] = pagesize & 0xff;

    /*
     * Synthesize the mode byte.  This is simpler than adding yet
     * another parameter to the avrdude.conf file.  We calculate the
     * bits corresponding to the page size, as explained in AVR068.
     * We set bit 7, to indicate this is to actually write the page to
     * the target device.  We set bit 6 to indicate this is the very
     * last page to be programmed, whatever this means -- we just
     * pretend we don't know any better. ;-)  Bit 0 is set if this is
     * a paged memory, which means it has a page size of more than 2.
     */
    buf[3] = 0x80 | 0x40;
    if (pagesize > 2) {
      buf[3] |= stk500v2_mode_for_pagesize(pagesize);
      buf[3] |= 0x01;
    }
    buf[4] = mem->delay;
    memcpy(buf + 5, cache_ptr, pagesize);

    /* flash and EEPROM reads require the load address command */
    stk500v2_loadaddr(pgm, use_ext_addr | (paddr >> addrshift));
  } else {
    buf[1] = addr;
    buf[2] = data;
    if (mode == PPMODE) {
      buf[3] = pulsewidth;
      buf[4] = timeout;
    } else {
      buf[3] = timeout;
      cmdlen--;
    }
  }

  if (verbose >= 2)
    fprintf(stderr, "%s: stk500hv_write_byte(): Sending write memory command: ",
	    progname);

  result = stk500v2_command(pgm, buf, cmdlen, sizeof(buf));

  if (result < 0 || buf[1] != STATUS_CMD_OK) {
    fprintf(stderr,
	    "%s: stk500hv_write_byte(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, result);
    return -1;
  }

  if (pagesize) {
    /* Invalidate the page cache. */
    *paddr_ptr = (unsigned long)-1L;
  }

  return 0;
}

/*
 * Write one byte, PP mode
 */
static int stk500pp_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			       unsigned long addr, unsigned char data)
{
  return stk500hv_write_byte(pgm, p, mem, addr, data, PPMODE);
}

/*
 * Write one byte, HVSP mode
 */
static int stk500hvsp_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			       unsigned long addr, unsigned char data)
{
  return stk500hv_write_byte(pgm, p, mem, addr, data, HVSPMODE);
}


static int stk500v2_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  unsigned int addr, block_size, last_addr, hiaddr, addrshift, use_ext_addr;
  unsigned char commandbuf[10];
  unsigned char buf[266];
  unsigned char cmds[4];
  int result;
  OPCODE * rop, * wop;

  DEBUG("STK500V2: stk500v2_paged_write(..,%s,%d,%d)\n",m->desc,page_size,n_bytes);

  if (page_size == 0) page_size = 256;
  hiaddr = UINT_MAX;
  addrshift = 0;
  use_ext_addr = 0;

  // determine which command is to be used
  if (strcmp(m->desc, "flash") == 0) {
    addrshift = 1;
    commandbuf[0] = CMD_PROGRAM_FLASH_ISP;
    /*
     * If bit 31 is set, this indicates that the following read/write
     * operation will be performed on a memory that is larger than
     * 64KBytes. This is an indication to STK500 that a load extended
     * address must be executed.
     */
    if (m->op[AVR_OP_LOAD_EXT_ADDR] != NULL) {
      use_ext_addr = (1U << 31);
    }
  } else if (strcmp(m->desc, "eeprom") == 0) {
    commandbuf[0] = CMD_PROGRAM_EEPROM_ISP;
  }
  commandbuf[4] = m->delay;

  if (addrshift == 0) {
    wop = m->op[AVR_OP_WRITE];
    rop = m->op[AVR_OP_READ];
  }
  else {
    wop = m->op[AVR_OP_WRITE_LO];
    rop = m->op[AVR_OP_READ_LO];
  }

  // if the memory is paged, load the appropriate commands into the buffer
  if (m->mode & 0x01) {
    commandbuf[3] = m->mode | 0x80;		// yes, write the page to flash

    if (m->op[AVR_OP_LOADPAGE_LO] == NULL) {
      fprintf(stderr, "%s: stk500v2_paged_write: loadpage instruction not defined for part \"%s\"\n",
              progname, p->desc);
      return -1;
    }
    avr_set_bits(m->op[AVR_OP_LOADPAGE_LO], cmds);
    commandbuf[5] = cmds[0];

    if (m->op[AVR_OP_WRITEPAGE] == NULL) {
      fprintf(stderr, "%s: stk500v2_paged_write: write page instruction not defined for part \"%s\"\n",
              progname, p->desc);
      return -1;
    }
    avr_set_bits(m->op[AVR_OP_WRITEPAGE], cmds);
    commandbuf[6] = cmds[0];

  // otherwise, we need to load different commands in
  } 
  else {
    commandbuf[3] = m->mode | 0x80;		// yes, write the words to flash

    if (wop == NULL) {
      fprintf(stderr, "%s: stk500v2_paged_write: write instruction not defined for part \"%s\"\n",
              progname, p->desc);
      return -1;
    }
    avr_set_bits(wop, cmds);
    commandbuf[5] = cmds[0];
    commandbuf[6] = 0;
  }

  // the read command is common to both methods
  if (rop == NULL) {
    fprintf(stderr, "%s: stk500v2_paged_write: read instruction not defined for part \"%s\"\n",
            progname, p->desc);
    return -1;
  }
  avr_set_bits(rop, cmds);
  commandbuf[7] = cmds[0];

  commandbuf[8] = m->readback[0];
  commandbuf[9] = m->readback[1];

  last_addr=UINT_MAX;		/* impossible address */

  for (addr=0; addr < n_bytes; addr += page_size) {
    report_progress(addr,n_bytes,NULL);

    if ((n_bytes-addr) < page_size)
      block_size = n_bytes - addr;
    else
      block_size = page_size;

    DEBUG("block_size at addr %d is %d\n",addr,block_size);

    if(commandbuf[0] == CMD_PROGRAM_FLASH_ISP){
      if (stk500v2_is_page_empty(addr, block_size, m->buf)) {
          continue;
      }
    }

    memcpy(buf,commandbuf,sizeof(commandbuf));

    buf[1] = block_size >> 8;
    buf[2] = block_size & 0xff;

    if((last_addr==UINT_MAX)||(last_addr+block_size != addr)){
      stk500v2_loadaddr(pgm, use_ext_addr | (addr >> addrshift));
    }
    last_addr=addr;

    memcpy(buf+10,m->buf+addr, block_size);

    result = stk500v2_command(pgm,buf,block_size+10, sizeof(buf));
    if (buf[1] != STATUS_CMD_OK) {
      fprintf(stderr,"%s: stk500v2_paged_write: write command failed with %d\n",
              progname,buf[1]);
      return -1;
    }
  }

  return n_bytes;
}

/*
 * Write pages of flash/EEPROM, generic HV mode
 */
static int stk500hv_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
				  int page_size, int n_bytes,
				  enum hvmode mode)
{
  unsigned int addr, block_size, last_addr, hiaddr, addrshift, use_ext_addr;
  unsigned char commandbuf[5], buf[266];
  int result;

  DEBUG("STK500V2: stk500hv_paged_write(..,%s,%d,%d)\n",m->desc,page_size,n_bytes);

  hiaddr = UINT_MAX;
  addrshift = 0;
  use_ext_addr = 0;

  // determine which command is to be used
  if (strcmp(m->desc, "flash") == 0) {
    addrshift = 1;
    flash_pageaddr = (unsigned long)-1L;
    commandbuf[0] = mode == PPMODE? CMD_PROGRAM_FLASH_PP: CMD_PROGRAM_FLASH_HVSP;
    /*
     * If bit 31 is set, this indicates that the following read/write
     * operation will be performed on a memory that is larger than
     * 64KBytes. This is an indication to STK500 that a load extended
     * address must be executed.
     */
    if (m->op[AVR_OP_LOAD_EXT_ADDR] != NULL) {
      use_ext_addr = (1U << 31);
    }
  } else if (strcmp(m->desc, "eeprom") == 0) {
    eeprom_pageaddr = (unsigned long)-1L;
    commandbuf[0] = mode == PPMODE? CMD_PROGRAM_EEPROM_PP: CMD_PROGRAM_EEPROM_HVSP;
  }
  /*
   * Synthesize the mode byte.  This is simpler than adding yet
   * another parameter to the avrdude.conf file.  We calculate the
   * bits corresponding to the page size, as explained in AVR068.  We
   * set bit 7, to indicate this is to actually write the page to the
   * target device.  We set bit 6 to indicate this is the very last
   * page to be programmed, whatever this means -- we just pretend we
   * don't know any better. ;-)  Finally, we set bit 0 to say this is
   * a paged memory, after all, that's why we got here at all.
   */
  commandbuf[3] = 0x80 | 0x40;
  if (page_size > 2) {
    commandbuf[3] |= stk500v2_mode_for_pagesize(page_size);
    commandbuf[3] |= 0x01;
  }
  commandbuf[4] = m->delay;

  if (page_size == 0) page_size = 256;

  last_addr = UINT_MAX;		/* impossible address */

  for (addr = 0; addr < n_bytes; addr += page_size) {
    report_progress(addr,n_bytes,NULL);

    if ((n_bytes-addr) < page_size)
      block_size = n_bytes - addr;
    else
      block_size = page_size;

    DEBUG("block_size at addr %d is %d\n",addr,block_size);

    if (addrshift == 1) {
      if (stk500v2_is_page_empty(addr, block_size, m->buf)) {
          continue;
      }
    }

    memcpy(buf, commandbuf, sizeof(commandbuf));

    buf[1] = page_size >> 8;
    buf[2] = page_size & 0xff;

    if ((last_addr == UINT_MAX) || (last_addr + block_size != addr)) {
      stk500v2_loadaddr(pgm, use_ext_addr | (addr >> addrshift));
    }
    last_addr=addr;

    memcpy(buf + 5, m->buf + addr, block_size);
    if (block_size != page_size)
      memset(buf + 5 + block_size, 0xff, page_size - block_size);

    result = stk500v2_command(pgm, buf, page_size + 5, sizeof(buf));
    if (buf[1] != STATUS_CMD_OK) {
      fprintf(stderr, "%s: stk500hv_paged_write: write command failed with %d\n",
              progname, buf[1]);
      return -1;
    }
  }

  return n_bytes;
}

/*
 * Write pages of flash/EEPROM, PP mode
 */
static int stk500pp_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
				int page_size, int n_bytes)
{
  return stk500hv_paged_write(pgm, p, m, page_size, n_bytes, PPMODE);
}

/*
 * Write pages of flash/EEPROM, HVSP mode
 */
static int stk500hvsp_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
				  int page_size, int n_bytes)
{
  return stk500hv_paged_write(pgm, p, m, page_size, n_bytes, HVSPMODE);
}

static int stk500v2_is_page_empty(unsigned int address, int page_size,
                                const unsigned char *buf)
{
    int i;
    for(i = 0; i < page_size; i++) {
        if(buf[address + i] != 0xFF) {
            /* Page is not empty. */
            return(0);
        }
    }

    /* Page is empty. */
    return(1);
}

static int stk500v2_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                             int page_size, int n_bytes)
{
  unsigned int addr, block_size, hiaddr, addrshift, use_ext_addr;
  unsigned char commandbuf[4];
  unsigned char buf[275];	// max buffer size for stk500v2 at this point
  unsigned char cmds[4];
  int result;
  OPCODE * rop;

  DEBUG("STK500V2: stk500v2_paged_load(..,%s,%d,%d)\n",m->desc,page_size,n_bytes);

  page_size = m->readsize;

  rop = m->op[AVR_OP_READ];

  hiaddr = UINT_MAX;
  addrshift = 0;
  use_ext_addr = 0;

  // determine which command is to be used
  if (strcmp(m->desc, "flash") == 0) {
    commandbuf[0] = CMD_READ_FLASH_ISP;
    rop = m->op[AVR_OP_READ_LO];
    addrshift = 1;
    /*
     * If bit 31 is set, this indicates that the following read/write
     * operation will be performed on a memory that is larger than
     * 64KBytes. This is an indication to STK500 that a load extended
     * address must be executed.
     */
    if (m->op[AVR_OP_LOAD_EXT_ADDR] != NULL) {
      use_ext_addr = (1U << 31);
    }
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    commandbuf[0] = CMD_READ_EEPROM_ISP;
  }

  // the read command is common to both methods
  if (rop == NULL) {
    fprintf(stderr, "%s: stk500v2_paged_load: read instruction not defined for part \"%s\"\n",
            progname, p->desc);
    return -1;
  }
  avr_set_bits(rop, cmds);
  commandbuf[3] = cmds[0];

  for (addr=0; addr < n_bytes; addr += page_size) {
    report_progress(addr, n_bytes,NULL);

    if ((n_bytes-addr) < page_size)
      block_size = n_bytes - addr;
    else
      block_size = page_size;
    DEBUG("block_size at addr %d is %d\n",addr,block_size);

    memcpy(buf,commandbuf,sizeof(commandbuf));

    buf[1] = block_size >> 8;
    buf[2] = block_size & 0xff;

    // Ensure a new "load extended address" will be issued
    // when crossing a 64 KB boundary in flash.
    if (hiaddr != (addr & ~0xFFFF)) {
      hiaddr = addr & ~0xFFFF;
      stk500v2_loadaddr(pgm, use_ext_addr | (addr >> addrshift));
    }

    result = stk500v2_command(pgm,buf,4,sizeof(buf));
    if (buf[1] != STATUS_CMD_OK) {
      fprintf(stderr,"%s: stk500v2_paged_load: read command failed with %d\n",
              progname,buf[1]);
      return -1;
    }
#if 0
    for (i=0;i<page_size;i++) {
      fprintf(stderr,"%02X",buf[2+i]);
      if (i%16 == 15) fprintf(stderr,"\n");
    }
#endif

    memcpy(&m->buf[addr], &buf[2], block_size);
  }

  return n_bytes;
}


/*
 * Read pages of flash/EEPROM, generic HV mode
 */
static int stk500hv_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
			       int page_size, int n_bytes,
			       enum hvmode mode)
{
  unsigned int addr, block_size, hiaddr, addrshift, use_ext_addr;
  unsigned char commandbuf[3], buf[266];
  int result;

  DEBUG("STK500V2: stk500hv_paged_load(..,%s,%d,%d)\n",m->desc,page_size,n_bytes);

  page_size = m->readsize;

  hiaddr = UINT_MAX;
  addrshift = 0;
  use_ext_addr = 0;

  // determine which command is to be used
  if (strcmp(m->desc, "flash") == 0) {
    commandbuf[0] = mode == PPMODE? CMD_READ_FLASH_PP: CMD_READ_FLASH_HVSP;
    addrshift = 1;
    /*
     * If bit 31 is set, this indicates that the following read/write
     * operation will be performed on a memory that is larger than
     * 64KBytes. This is an indication to STK500 that a load extended
     * address must be executed.
     */
    if (m->op[AVR_OP_LOAD_EXT_ADDR] != NULL) {
      use_ext_addr = (1U << 31);
    }
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    commandbuf[0] = mode == PPMODE? CMD_READ_EEPROM_PP: CMD_READ_EEPROM_HVSP;
  }

  for (addr = 0; addr < n_bytes; addr += page_size) {
    report_progress(addr, n_bytes, NULL);

    if ((n_bytes-addr) < page_size)
      block_size = n_bytes - addr;
    else
      block_size = page_size;
    DEBUG("block_size at addr %d is %d\n",addr,block_size);

    memcpy(buf, commandbuf, sizeof(commandbuf));

    buf[1] = block_size >> 8;
    buf[2] = block_size & 0xff;

    // Ensure a new "load extended address" will be issued
    // when crossing a 64 KB boundary in flash.
    if (hiaddr != (addr & ~0xFFFF)) {
      hiaddr = addr & ~0xFFFF;
      stk500v2_loadaddr(pgm, use_ext_addr | (addr >> addrshift));
    }

    result = stk500v2_command(pgm, buf, 3, sizeof(buf));
    if (buf[1] != STATUS_CMD_OK) {
      fprintf(stderr, "%s: stk500hv_paged_load: read command failed with %d\n",
              progname, buf[1]);
      return -1;
    }
#if 0
    for (i = 0; i < page_size; i++) {
      fprintf(stderr, "%02X", buf[2 + i]);
      if (i % 16 == 15) fprintf(stderr, "\n");
    }
#endif

    memcpy(&m->buf[addr], &buf[2], block_size);
  }

  return n_bytes;
}

/*
 * Read pages of flash/EEPROM, PP mode
 */
static int stk500pp_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
			       int page_size, int n_bytes)
{
  return stk500hv_paged_load(pgm, p, m, page_size, n_bytes, PPMODE);
}

/*
 * Read pages of flash/EEPROM, HVSP mode
 */
static int stk500hvsp_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
				 int page_size, int n_bytes)
{
  return stk500hv_paged_load(pgm, p, m, page_size, n_bytes, HVSPMODE);
}


static int stk500v2_set_vtarget(PROGRAMMER * pgm, double v)
{
  unsigned char uaref, utarg;

  utarg = (unsigned)((v + 0.049) * 10);

  if (stk500v2_getparm(pgm, PARAM_VADJUST, &uaref) != 0) {
    fprintf(stderr,
	    "%s: stk500v2_set_vtarget(): cannot obtain V[aref]\n",
	    progname);
    return -1;
  }

  if (uaref > utarg) {
    fprintf(stderr,
	    "%s: stk500v2_set_vtarget(): reducing V[aref] from %.1f to %.1f\n",
	    progname, uaref / 10.0, v);
    if (stk500v2_setparm(pgm, PARAM_VADJUST, utarg)
	!= 0)
      return -1;
  }
  return stk500v2_setparm(pgm, PARAM_VTARGET, utarg);
}


static int stk500v2_set_varef(PROGRAMMER * pgm, double v)
{
  unsigned char uaref, utarg;

  uaref = (unsigned)((v + 0.049) * 10);

  if (stk500v2_getparm(pgm, PARAM_VTARGET, &utarg) != 0) {
    fprintf(stderr,
	    "%s: stk500v2_set_varef(): cannot obtain V[target]\n",
	    progname);
    return -1;
  }

  if (uaref > utarg) {
    fprintf(stderr,
	    "%s: stk500v2_set_varef(): V[aref] must not be greater than "
	    "V[target] = %.1f\n",
	    progname, utarg / 10.0);
    return -1;
  }
  return stk500v2_setparm(pgm, PARAM_VADJUST, uaref);
}


static int stk500v2_set_fosc(PROGRAMMER * pgm, double v)
{
  int fosc;
  unsigned char prescale, cmatch;
  static unsigned ps[] = {
    1, 8, 32, 64, 128, 256, 1024
  };
  int idx, rc;

  prescale = cmatch = 0;
  if (v > 0.0) {
    if (v > STK500V2_XTAL / 2) {
      const char *unit;
      if (v > 1e6) {
        v /= 1e6;
        unit = "MHz";
      } else if (v > 1e3) {
        v /= 1e3;
        unit = "kHz";
      } else
        unit = "Hz";
      fprintf(stderr,
          "%s: stk500v2_set_fosc(): f = %.3f %s too high, using %.3f MHz\n",
          progname, v, unit, STK500V2_XTAL / 2e6);
      fosc = STK500V2_XTAL / 2;
    } else
      fosc = (unsigned)v;

    for (idx = 0; idx < sizeof(ps) / sizeof(ps[0]); idx++) {
      if (fosc >= STK500V2_XTAL / (256 * ps[idx] * 2)) {
        /* this prescaler value can handle our frequency */
        prescale = idx + 1;
        cmatch = (unsigned)(STK500V2_XTAL / (2 * fosc * ps[idx])) - 1;
        break;
      }
    }
    if (idx == sizeof(ps) / sizeof(ps[0])) {
      fprintf(stderr, "%s: stk500v2_set_fosc(): f = %u Hz too low, %u Hz min\n",
          progname, fosc, STK500V2_XTAL / (256 * 1024 * 2));
      return -1;
    }
  }

  if ((rc = stk500v2_setparm(pgm, PARAM_OSC_PSCALE, prescale)) != 0
      || (rc = stk500v2_setparm(pgm, PARAM_OSC_CMATCH, cmatch)) != 0)
    return rc;

  return 0;
}

/* The list of SCK frequencies supported by the AVRISP mkII, as listed
 * in AVR069 */
double avrispmkIIfreqs[] = {
	8000000, 4000000, 2000000, 1000000, 500000, 250000, 125000,
	96386, 89888, 84211, 79208, 74767, 70797, 67227, 64000,
	61069, 58395, 55945, 51613, 49690, 47905, 46243, 43244,
	41885, 39409, 38278, 36200, 34335, 32654, 31129, 29740,
	28470, 27304, 25724, 24768, 23461, 22285, 21221, 20254,
	19371, 18562, 17583, 16914, 16097, 15356, 14520, 13914,
	13224, 12599, 12031, 11511, 10944, 10431, 9963, 9468,
	9081, 8612, 8239, 7851, 7498, 7137, 6809, 6478, 6178,
	5879, 5607, 5359, 5093, 4870, 4633, 4418, 4209, 4019,
	3823, 3645, 3474, 3310, 3161, 3011, 2869, 2734, 2611,
	2484, 2369, 2257, 2152, 2052, 1956, 1866, 1779, 1695,
	1615, 1539, 1468, 1398, 1333, 1271, 1212, 1155, 1101,
	1049, 1000, 953, 909, 866, 826, 787, 750, 715, 682,
	650, 619, 590, 563, 536, 511, 487, 465, 443, 422,
	402, 384, 366, 349, 332, 317, 302, 288, 274, 261,
	249, 238, 226, 216, 206, 196, 187, 178, 170, 162,
	154, 147, 140, 134, 128, 122, 116, 111, 105, 100,
	95.4, 90.9, 86.6, 82.6, 78.7, 75.0, 71.5, 68.2,
	65.0, 61.9, 59.0, 56.3, 53.6, 51.1
};

#if defined(HAVE_LIBUSB)
static int stk500v2_set_sck_period_mk2(PROGRAMMER * pgm, double v)
{
  int i;

  for (i = 0; i < sizeof(avrispmkIIfreqs); i++) {
    if (1 / avrispmkIIfreqs[i] >= v)
      break;
  }

  fprintf(stderr, "Using p = %.2f us for SCK (param = %d)\n",
		  1000000 / avrispmkIIfreqs[i], i);

  return stk500v2_setparm(pgm, PARAM_SCK_DURATION, i);
}
#endif /* HAVE_LIBUSB */

/*
 * Return the "mode" value for the parallel and HVSP modes that
 * corresponds to the pagesize.
 */
static unsigned int stk500v2_mode_for_pagesize(unsigned int pagesize)
{
  switch (pagesize)
    {
    case 256:  return 0u << 1;
    case 2:    return 1u << 1;
    case 4:    return 2u << 1;
    case 8:    return 3u << 1;
    case 16:   return 4u << 1;
    case 32:   return 5u << 1;
    case 64:   return 6u << 1;
    case 128:  return 7u << 1;
    }
  fprintf(stderr,
	  "%s: stk500v2_mode_for_pagesize(): invalid pagesize: %u\n",
	  progname, pagesize);
  exit(1);
}

/* This code assumes that each count of the SCK duration parameter
   represents 8/f, where f is the clock frequency of the STK500V2 master
   processors (not the target).  This number comes from Atmel
   application note AVR061.  It appears that the STK500V2 bit bangs SCK.
   For small duration values, the actual SCK width is larger than
   expected.  As the duration value increases, the SCK width error
   diminishes. */
static int stk500v2_set_sck_period(PROGRAMMER * pgm, double v)
{
  unsigned char dur;
  double min, max;

  min = 8.0 / STK500V2_XTAL;
  max = 255 * min;
  dur = v / min + 0.5;

  if (v < min) {
      dur = 1;
      fprintf(stderr,
	      "%s: stk500v2_set_sck_period(): p = %.1f us too small, using %.1f us\n",
	      progname, v / 1e-6, dur * min / 1e-6);
  } else if (v > max) {
      dur = 255;
      fprintf(stderr,
	      "%s: stk500v2_set_sck_period(): p = %.1f us too large, using %.1f us\n",
	      progname, v / 1e-6, dur * min / 1e-6);
  }

  return stk500v2_setparm(pgm, PARAM_SCK_DURATION, dur);
}


static int stk500v2_getparm(PROGRAMMER * pgm, unsigned char parm, unsigned char * value)
{
  unsigned char buf[32];

  buf[0] = CMD_GET_PARAMETER;
  buf[1] = parm;

  if (stk500v2_command(pgm, buf, 2, sizeof(buf)) < 0) {
    fprintf(stderr,"%s: stk500v2_getparm(): failed to get parameter 0x%02x\n",
            progname, parm);
    return -1;
  }

  *value = buf[2];

  return 0;
}

static int stk500v2_setparm_real(PROGRAMMER * pgm, unsigned char parm, unsigned char value)
{
  unsigned char buf[32];

  buf[0] = CMD_SET_PARAMETER;
  buf[1] = parm;
  buf[2] = value;

  if (stk500v2_command(pgm, buf, 3, sizeof(buf)) < 0) {
    fprintf(stderr, "\n%s: stk500v2_setparm(): failed to set parameter 0x%02x\n",
            progname, parm);
    return -1;
  }

  return 0;
}

static int stk500v2_setparm(PROGRAMMER * pgm, unsigned char parm, unsigned char value)
{
  unsigned char current_value;
  int res;

  res = stk500v2_getparm(pgm, parm, &current_value);
  if (res < 0)
    fprintf(stderr, "%s: Unable to get parameter 0x%02x\n", progname, parm);

  // don't issue a write if the correct value is already set.
  if (value == current_value) {
    fprintf(stderr, "%s: Skipping paramter write; parameter value already set.\n", progname);
    return 0;
  }

  return stk500v2_setparm_real(pgm, parm, value);
}

static void stk500v2_display(PROGRAMMER * pgm, char * p)
{
  unsigned char maj, min, hdw, topcard;
  const char *topcard_name;

  stk500v2_getparm(pgm, PARAM_HW_VER, &hdw);
  stk500v2_getparm(pgm, PARAM_SW_MAJOR, &maj);
  stk500v2_getparm(pgm, PARAM_SW_MINOR, &min);

  fprintf(stderr, "%sHardware Version: %d\n", p, hdw);
  fprintf(stderr, "%sFirmware Version: %d.%02d\n", p, maj, min);

  if (pgmtype == PGMTYPE_STK500) {
    stk500v2_getparm(pgm, PARAM_TOPCARD_DETECT, &topcard);
    switch (topcard) {
      case 0xAA: topcard_name = "STK501"; break;
      case 0x55: topcard_name = "STK502"; break;
      case 0xFA: topcard_name = "STK503"; break;
      case 0xEE: topcard_name = "STK504"; break;
      case 0xE4: topcard_name = "STK505"; break;
      case 0xDD: topcard_name = "STK520"; break;
      default: topcard_name = "Unknown"; break;
    }
    fprintf(stderr, "%sTopcard         : %s\n", p, topcard_name);
  }
  stk500v2_print_parms1(pgm, p);

  return;
}


static void stk500v2_print_parms1(PROGRAMMER * pgm, char * p)
{
  unsigned char vtarget, vadjust, osc_pscale, osc_cmatch, sck_duration;

  stk500v2_getparm(pgm, PARAM_VTARGET, &vtarget);
  stk500v2_getparm(pgm, PARAM_VADJUST, &vadjust);
  stk500v2_getparm(pgm, PARAM_OSC_PSCALE, &osc_pscale);
  stk500v2_getparm(pgm, PARAM_OSC_CMATCH, &osc_cmatch);
  stk500v2_getparm(pgm, PARAM_SCK_DURATION, &sck_duration);

  fprintf(stderr, "%sVtarget         : %.1f V\n", p, vtarget / 10.0);
  fprintf(stderr, "%sVaref           : %.1f V\n", p, vadjust / 10.0);
  fprintf(stderr, "%sOscillator      : ", p);
  if (osc_pscale == 0)
    fprintf(stderr, "Off\n");
  else {
    int prescale = 1;
    double f = STK500V2_XTAL / 2;
    const char *unit;

    switch (osc_pscale) {
      case 2: prescale = 8; break;
      case 3: prescale = 32; break;
      case 4: prescale = 64; break;
      case 5: prescale = 128; break;
      case 6: prescale = 256; break;
      case 7: prescale = 1024; break;
    }
    f /= prescale;
    f /= (osc_cmatch + 1);
    if (f > 1e6) {
      f /= 1e6;
      unit = "MHz";
    } else if (f > 1e3) {
      f /= 1000;
      unit = "kHz";
    } else
      unit = "Hz";
    fprintf(stderr, "%.3f %s\n", f, unit);
  }
  if (is_mk2)
    fprintf(stderr, "%sSCK period      : %.2f us\n", p,
	  (float) 1000000 / avrispmkIIfreqs[sck_duration]);
  else
    fprintf(stderr, "%sSCK period      : %.1f us\n", p,
	  sck_duration * 8.0e6 / STK500V2_XTAL + 0.05);

  return;
}


static void stk500v2_print_parms(PROGRAMMER * pgm)
{
  stk500v2_print_parms1(pgm, "");
}


void stk500v2_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK500V2");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500v2_initialize;
  pgm->display        = stk500v2_display;
  pgm->enable         = stk500v2_enable;
  pgm->disable        = stk500v2_disable;
  pgm->program_enable = stk500v2_program_enable;
  pgm->chip_erase     = stk500v2_chip_erase;
  pgm->cmd            = stk500v2_cmd;
  pgm->open           = stk500v2_open;
  pgm->close          = stk500v2_close;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500v2_paged_write;
  pgm->paged_load     = stk500v2_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk500v2_set_vtarget;
  pgm->set_varef      = stk500v2_set_varef;
  pgm->set_fosc       = stk500v2_set_fosc;
  pgm->set_sck_period = stk500v2_set_sck_period;
  pgm->page_size      = 256;
}

void stk500pp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK500PP");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500pp_initialize;
  pgm->display        = stk500v2_display;
  pgm->enable         = stk500v2_enable;
  pgm->disable        = stk500pp_disable;
  pgm->program_enable = stk500pp_program_enable;
  pgm->chip_erase     = stk500pp_chip_erase;
  pgm->cmd            = stk500hv_cmd;
  pgm->open           = stk500v2_open;
  pgm->close          = stk500v2_close;

  /*
   * optional functions
   */
  pgm->read_byte      = stk500pp_read_byte;
  pgm->write_byte     = stk500pp_write_byte;
  pgm->paged_write    = stk500pp_paged_write;
  pgm->paged_load     = stk500pp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk500v2_set_vtarget;
  pgm->set_varef      = stk500v2_set_varef;
  pgm->set_fosc       = stk500v2_set_fosc;
  pgm->set_sck_period = stk500v2_set_sck_period;
  pgm->page_size      = 256;
}

void stk500hvsp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK500HVSP");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500hvsp_initialize;
  pgm->display        = stk500v2_display;
  pgm->enable         = stk500v2_enable;
  pgm->disable        = stk500hvsp_disable;
  pgm->program_enable = stk500hvsp_program_enable;
  pgm->chip_erase     = stk500hvsp_chip_erase;
  pgm->cmd            = stk500hv_cmd;
  pgm->open           = stk500v2_open;
  pgm->close          = stk500v2_close;

  /*
   * optional functions
   */
  pgm->read_byte      = stk500hvsp_read_byte;
  pgm->write_byte     = stk500hvsp_write_byte;
  pgm->paged_write    = stk500hvsp_paged_write;
  pgm->paged_load     = stk500hvsp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk500v2_set_vtarget;
  pgm->set_varef      = stk500v2_set_varef;
  pgm->set_fosc       = stk500v2_set_fosc;
  pgm->set_sck_period = stk500v2_set_sck_period;
  pgm->page_size      = 256;
}
