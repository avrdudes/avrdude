/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005 Erik Walthinsen
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2006 David Moore
 * Copyright (C) 2006,2007 Joerg Wunsch <j@uriah.heep.sax.de>
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"
#include "stk500_private.h"	// temp until all code converted
#include "stk500v2_private.h"
#include "serial.h"
#include "usbdevs.h"

/*
 * We need to import enough from the JTAG ICE mkII definitions to be
 * able to talk to the ICE, query some parameters etc.  The macro
 * JTAGMKII_PRIVATE_EXPORTED limits the amount of definitions that
 * jtagmkII_private.h will export, so to avoid conflicts with those
 * names that are identical to the STK500v2 ones.
 */
#include "jtagmkII.h"           // public interfaces from jtagmkII.c
#define JTAGMKII_PRIVATE_EXPORTED
#include "jtagmkII_private.h"

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


/*
 * Private data for this programmer.
 */
struct pdata
{
  /*
   * See stk500pp_read_byte() for an explanation of the flash and
   * EEPROM page caches.
   */
  unsigned char *flash_pagecache;
  unsigned long flash_pageaddr;
  unsigned int flash_pagesize;

  unsigned char *eeprom_pagecache;
  unsigned long eeprom_pageaddr;
  unsigned int eeprom_pagesize;

  unsigned char command_sequence;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))


/*
 * Data structure for displaying STK600 routing and socket cards.
 */
struct carddata
{
  int id;
  const char *name;
};

/* XXX That should become part of the private pgm-> data. */
static enum
{
  PGMTYPE_UNKNOWN,
  PGMTYPE_STK500,
  PGMTYPE_AVRISP,
  PGMTYPE_AVRISP_MKII,
  PGMTYPE_JTAGICE_MKII,
  PGMTYPE_STK600,
}
pgmtype;

static const char *pgmname[] =
{
  "unknown",
  "STK500",
  "AVRISP",
  "AVRISP mkII",
  "JTAG ICE mkII",
  "STK600",
};

/* XXX That should become part of the private pgm-> data. */
static AVRPART *lastpart;

struct jtagispentry
{
  unsigned char cmd;
  unsigned short size;
#define SZ_READ_FLASH_EE USHRT_MAX
#define SZ_SPI_MULTI     (USHRT_MAX - 1)
};

static struct jtagispentry jtagispcmds[] = {
  /* generic */
  { CMD_SET_PARAMETER, 2 },
  { CMD_GET_PARAMETER, 3 },
  { CMD_OSCCAL, 2 },
  { CMD_LOAD_ADDRESS, 2 },
  /* ISP mode */
  { CMD_ENTER_PROGMODE_ISP, 2 },
  { CMD_LEAVE_PROGMODE_ISP, 2 },
  { CMD_CHIP_ERASE_ISP, 2 },
  { CMD_PROGRAM_FLASH_ISP, 2 },
  { CMD_READ_FLASH_ISP, SZ_READ_FLASH_EE },
  { CMD_PROGRAM_EEPROM_ISP, 2 },
  { CMD_READ_EEPROM_ISP, SZ_READ_FLASH_EE },
  { CMD_PROGRAM_FUSE_ISP, 3 },
  { CMD_READ_FUSE_ISP, 4 },
  { CMD_PROGRAM_LOCK_ISP, 3 },
  { CMD_READ_LOCK_ISP, 4 },
  { CMD_READ_SIGNATURE_ISP, 4 },
  { CMD_READ_OSCCAL_ISP, 4 },
  { CMD_SPI_MULTI, SZ_SPI_MULTI },
  /* all HV modes */
  { CMD_SET_CONTROL_STACK, 2 },
  /* HVSP mode */
  { CMD_ENTER_PROGMODE_HVSP, 2 },
  { CMD_LEAVE_PROGMODE_HVSP, 2 },
  { CMD_CHIP_ERASE_HVSP, 2 },
  { CMD_PROGRAM_FLASH_HVSP, 2 },
  { CMD_READ_FLASH_HVSP, SZ_READ_FLASH_EE },
  { CMD_PROGRAM_EEPROM_HVSP, 2 },
  { CMD_READ_EEPROM_HVSP, SZ_READ_FLASH_EE },
  { CMD_PROGRAM_FUSE_HVSP, 2 },
  { CMD_READ_FUSE_HVSP, 3 },
  { CMD_PROGRAM_LOCK_HVSP, 2 },
  { CMD_READ_LOCK_HVSP, 3 },
  { CMD_READ_SIGNATURE_HVSP, 3 },
  { CMD_READ_OSCCAL_HVSP, 3 },
  /* PP mode */
  { CMD_ENTER_PROGMODE_PP, 2 },
  { CMD_LEAVE_PROGMODE_PP, 2 },
  { CMD_CHIP_ERASE_PP, 2 },
  { CMD_PROGRAM_FLASH_PP, 2 },
  { CMD_READ_FLASH_PP, SZ_READ_FLASH_EE },
  { CMD_PROGRAM_EEPROM_PP, 2 },
  { CMD_READ_EEPROM_PP, SZ_READ_FLASH_EE },
  { CMD_PROGRAM_FUSE_PP, 2 },
  { CMD_READ_FUSE_PP, 3 },
  { CMD_PROGRAM_LOCK_PP, 2 },
  { CMD_READ_LOCK_PP, 3 },
  { CMD_READ_SIGNATURE_PP, 3 },
  { CMD_READ_OSCCAL_PP, 3 },
};

static int stk500v2_getparm(PROGRAMMER * pgm, unsigned char parm, unsigned char * value);
static int stk500v2_setparm(PROGRAMMER * pgm, unsigned char parm, unsigned char value);
static int stk500v2_getparm2(PROGRAMMER * pgm, unsigned char parm, unsigned int * value);
static int stk500v2_setparm2(PROGRAMMER * pgm, unsigned char parm, unsigned int value);
static int stk500v2_setparm_real(PROGRAMMER * pgm, unsigned char parm, unsigned char value);
static void stk500v2_print_parms1(PROGRAMMER * pgm, const char * p);
static int stk500v2_is_page_empty(unsigned int address, int page_size,
                                  const unsigned char *buf);

static unsigned int stk500v2_mode_for_pagesize(unsigned int pagesize);

static int stk500v2_set_sck_period_mk2(PROGRAMMER * pgm, double v);

static int stk600_set_sck_period(PROGRAMMER * pgm, double v);

static void stk600_setup_xprog(PROGRAMMER * pgm);
static int stk600_xprog_program_enable(PROGRAMMER * pgm, AVRPART * p);

static void stk500v2_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
    fprintf(stderr,
	    "%s: stk500v2_setup(): Out of memory allocating private data\n",
	    progname);
    exit(1);
  }
  memset(pgm->cookie, 0, sizeof(struct pdata));
  PDATA(pgm)->command_sequence = 1;
}

static void stk500v2_teardown(PROGRAMMER * pgm)
{
  free(pgm->cookie);
}


static unsigned short
b2_to_u16(unsigned char *b)
{
  unsigned short l;
  l = b[0];
  l += (unsigned)b[1] << 8;

  return l;
}

static int stk500v2_send_mk2(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  if (serial_send(&pgm->fd, data, len) != 0) {
    fprintf(stderr,"%s: stk500_send_mk2(): failed to send command to serial port\n",progname);
    exit(1);
  }

  return 0;
}

static unsigned short get_jtagisp_return_size(unsigned char cmd)
{
  int i;

  for (i = 0; i < sizeof jtagispcmds / sizeof jtagispcmds[0]; i++)
    if (jtagispcmds[i].cmd == cmd)
      return jtagispcmds[i].size;

  return 0;
}

/*
 * Send the data as a JTAG ICE mkII encapsulated ISP packet.
 * Unlike what AVR067 says, the packet gets a length of our
 * response buffer prepended, and replies with RSP_SPI_DATA
 * if successful.
 */
static int stk500v2_jtagmkII_send(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  unsigned char *cmdbuf;
  int rv;
  unsigned short sz;

  sz = get_jtagisp_return_size(data[0]);
  if (sz == 0) {
    fprintf(stderr, "%s: unsupported encapsulated ISP command: %#x\n",
	    progname, data[0]);
    return -1;
  }
  if (sz == SZ_READ_FLASH_EE) {
    /*
     * For CMND_READ_FLASH_ISP and CMND_READ_EEPROM_ISP, extract the
     * size of the return data from the request.  Note that the
     * request itself has the size in big endian format, while we are
     * supposed to deliver it in little endian.
     */
    sz = 3 + (data[1] << 8) + data[2];
  } else if (sz == SZ_SPI_MULTI) {
    /*
     * CMND_SPI_MULTI has the Rx size encoded in its 3rd byte.
     */
    sz = 3 + data[2];
  }

  if ((cmdbuf = malloc(len + 3)) == NULL) {
    fprintf(stderr, "%s: out of memory for command packet\n",
            progname);
    exit(1);
  }
  cmdbuf[0] = CMND_ISP_PACKET;
  cmdbuf[1] = sz & 0xff;
  cmdbuf[2] = (sz >> 8) & 0xff;
  memcpy(cmdbuf + 3, data, len);
  rv = jtagmkII_send(pgm, cmdbuf, len + 3);
  free(cmdbuf);

  return rv;
}

static int stk500v2_send(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  unsigned char buf[275 + 6];		// max MESSAGE_BODY of 275 bytes, 6 bytes overhead
  int i;

  if (pgmtype == PGMTYPE_AVRISP_MKII || pgmtype == PGMTYPE_STK600)
    return stk500v2_send_mk2(pgm, data, len);
  else if (pgmtype == PGMTYPE_JTAGICE_MKII)
    return stk500v2_jtagmkII_send(pgm, data, len);

  buf[0] = MESSAGE_START;
  buf[1] = PDATA(pgm)->command_sequence;
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

  if (serial_send(&pgm->fd, buf, len+6) != 0) {
    fprintf(stderr,"%s: stk500_send(): failed to send command to serial port\n",progname);
    exit(1);
  }

  return 0;
}


static int stk500v2_drain(PROGRAMMER * pgm, int display)
{
  return serial_drain(&pgm->fd, display);
}

static int stk500v2_recv_mk2(PROGRAMMER * pgm, unsigned char msg[],
			     size_t maxsize)
{
  int rv;

  rv = serial_recv(&pgm->fd, msg, maxsize);
  if (rv < 0) {
    fprintf(stderr, "%s: stk500v2_recv_mk2: error in USB receive\n", progname);
    return -1;
  }

  return rv;
}

static int stk500v2_jtagmkII_recv(PROGRAMMER * pgm, unsigned char msg[],
                                  size_t maxsize)
{
  int rv;
  unsigned char *jtagmsg;

  rv = jtagmkII_recv(pgm, &jtagmsg);
  if (rv <= 0) {
    fprintf(stderr, "%s: stk500v2_jtagmkII_recv(): error in jtagmkII_recv()\n",
            progname);
    return -1;
  }
  if (rv - 1 > maxsize) {
    fprintf(stderr,
            "%s: stk500v2_jtagmkII_recv(): got %u bytes, have only room for %u bytes\n",
            progname, (unsigned)rv - 1, (unsigned)maxsize);
    rv = maxsize;
  }
  switch (jtagmsg[0]) {
  case RSP_SPI_DATA:
    break;
  case RSP_FAILED:
    fprintf(stderr, "%s: stk500v2_jtagmkII_recv(): failed\n",
	    progname);
    return -1;
  case RSP_ILLEGAL_MCU_STATE:
    fprintf(stderr, "%s: stk500v2_jtagmkII_recv(): illegal MCU state\n",
	    progname);
    return -1;
  default:
    fprintf(stderr, "%s: stk500v2_jtagmkII_recv(): unknown status %d\n",
	    progname, jtagmsg[0]);
    return -1;
  }
  memcpy(msg, jtagmsg + 1, rv - 1);
  return rv;
}

static int stk500v2_recv(PROGRAMMER * pgm, unsigned char msg[], size_t maxsize) {
  enum states { sINIT, sSTART, sSEQNUM, sSIZE1, sSIZE2, sTOKEN, sDATA, sCSUM, sDONE }  state = sSTART;
  unsigned int msglen = 0;
  unsigned int curlen = 0;
  int timeout = 0;
  unsigned char c, checksum = 0;

  long timeoutval = 5;		// seconds
  struct timeval tv;
  double tstart, tnow;

  if (pgmtype == PGMTYPE_AVRISP_MKII || pgmtype == PGMTYPE_STK600)
    return stk500v2_recv_mk2(pgm, msg, maxsize);
  else if (pgmtype == PGMTYPE_JTAGICE_MKII)
    return stk500v2_jtagmkII_recv(pgm, msg, maxsize);

  DEBUG("STK500V2: stk500v2_recv(): ");

  gettimeofday(&tv, NULL);
  tstart = tv.tv_sec;

  while ( (state != sDONE ) && (!timeout) ) {
    if (serial_recv(&pgm->fd, &c, 1) < 0)
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
        if (c == PDATA(pgm)->command_sequence) {
          DEBUGRECV("got it, incrementing\n");
          state = sSIZE1;
          PDATA(pgm)->command_sequence++;
        } else {
          DEBUGRECV("sorry\n");
          state = sSTART;
        }
        break;
      case sSIZE1:
        DEBUGRECV("hoping for size LSB\n");
        msglen = (unsigned)c * 256;
        state = sSIZE2;
        break;
      case sSIZE2:
        DEBUGRECV("hoping for size MSB...");
        msglen += (unsigned)c;
        DEBUG(" msg is %u bytes\n",msglen);
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

  return (int)(msglen+6);
}



static int stk500v2_getsync(PROGRAMMER * pgm) {
  int tries = 0;
  unsigned char buf[1], resp[32];
  int status;

  DEBUG("STK500V2: stk500v2_getsync()\n");

  if (pgmtype == PGMTYPE_JTAGICE_MKII)
    return 0;

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
      } else if (siglen >= strlen("STK600") &&
	  memcmp(resp + 3, "STK600", strlen("STK600")) == 0) {
	pgmtype = PGMTYPE_STK600;
      } else {
	resp[siglen + 3] = 0;
	if (verbose)
	  fprintf(stderr,
		  "%s: stk500v2_getsync(): got response from unknown "
		  "programmer %s, assuming STK500\n",
		  progname, resp + 3);
	pgmtype = PGMTYPE_STK500;
      }
      if (verbose >= 3)
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

static int stk500v2_command(PROGRAMMER * pgm, unsigned char * buf,
                            size_t len, size_t maxlen) {
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
    if (status < 2) {
      fprintf(stderr, "%s: stk500v2_command(): short reply\n", progname);
      return -1;
    }
    if (buf[0] == CMD_XPROG_SETMODE || buf[0] == CMD_XPROG) {
        /*
         * Decode XPROG wrapper errors.
         */
        const char *msg;
        int i;

        /*
         * For CMD_XPROG_SETMODE, the status is returned in buf[1].
         * For CMD_XPROG, buf[1] contains the XPRG_CMD_* command, and
         * buf[2] contains the status.
         */
        i = buf[0] == CMD_XPROG_SETMODE? 1: 2;

        if (buf[i] != XPRG_ERR_OK) {
            switch (buf[i]) {
            case XPRG_ERR_FAILED:   msg = "Failed"; break;
            case XPRG_ERR_COLLISION: msg = "Collision"; break;
            case XPRG_ERR_TIMEOUT:  msg = "Timeout"; break;
            default:                msg = "Unknown"; break;
            }
            fprintf(stderr, "%s: stk500v2_command(): error in %s: %s\n",
                    progname,
                    (buf[0] == CMD_XPROG_SETMODE? "CMD_XPROG_SETMODE": "CMD_XPROG"),
                    msg);
            return -1;
        }
        return 0;
    } else {
        /*
         * Decode STK500v2 errors.
         */
        if (buf[1] == STATUS_CMD_OK)
            return status;
        if (buf[1] == STATUS_CMD_FAILED)
            fprintf(stderr, "%s: stk500v2_command(): command failed\n", progname);
        else
            fprintf(stderr, "%s: stk500v2_command(): unknown status 0x%02x\n",
                    progname, buf[1]);
        return -1;
    }
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
  if (result < 0) {
    fprintf(stderr, "%s: stk500v2_cmd(): failed to send command\n",
            progname);
    return -1;
  } else if (result < 6) {
    fprintf(stderr, "%s: stk500v2_cmd(): short reply, len = %d\n",
            progname, result);
    return -1;
  }

  res[0] = buf[2];
  res[1] = buf[3];
  res[2] = buf[4];
  res[3] = buf[5];

  return 0;
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

static struct
{
  unsigned int state;
  const char *description;
} connection_status[] =
{
  { STATUS_CONN_FAIL_MOSI, "MOSI fail" },
  { STATUS_CONN_FAIL_RST, "RST fail" },
  { STATUS_CONN_FAIL_SCK, "SCK fail" },
  { STATUS_TGT_NOT_DETECTED, "Target not detected" },
  { STATUS_TGT_REVERSE_INSERTED, "Target reverse inserted" },
  { STATUS_CONN_FAIL_MOSI | STATUS_CONN_FAIL_RST,
    "MOSI and RST fail" },
  { STATUS_CONN_FAIL_MOSI | STATUS_CONN_FAIL_RST | STATUS_CONN_FAIL_SCK,
    "MOSI, RST, and SCK fail" },
  { STATUS_CONN_FAIL_RST | STATUS_CONN_FAIL_SCK,
    "RST and SCK fail" },
  { STATUS_CONN_FAIL_MOSI | STATUS_CONN_FAIL_SCK,
    "MOSI and SCK fail" },
};

/*
 * issue the 'program enable' command to the AVR device
 */
static int stk500v2_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[16];
  size_t i;
  const char *msg;
  int rv;

  lastpart = p;

  if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
    fprintf(stderr, "%s: stk500v2_program_enable(): program enable instruction not defined for part \"%s\"\n",
	    progname, p->desc);
    return -1;
  }

  if (pgmtype == PGMTYPE_STK500 || pgmtype == PGMTYPE_STK600)
      /* Activate AVR-style (low active) RESET */
      stk500v2_setparm_real(pgm, PARAM_RESET_POLARITY, 0x01);

  if (pgmtype == PGMTYPE_STK600) {
    buf[0] = CMD_CHECK_TARGET_CONNECTION;
    if (stk500v2_command(pgm, buf, 1, sizeof(buf)) < 0) {
      fprintf(stderr, "%s: stk500v2_program_enable(): cannot get connection status\n",
            progname);
      return -1;
    }
    if (buf[2] != STATUS_ISP_READY) {
      msg = "Unknown";

      for (i = 0;
           i < sizeof connection_status / sizeof connection_status[0];
           i++)
        if (connection_status[i].state == (unsigned)buf[2]) {
          msg = connection_status[i].description;
          break;
        }

      fprintf(stderr, "%s: stk500v2_program_enable():"
              " bad STK600 connection status: %s (0x%02x)\n",
            progname, msg, buf[2]);
      return -1;
    }
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
  buf[10] = buf[11] = 0;

  rv = stk500v2_command(pgm, buf, 12, sizeof(buf));

  if (rv < 0) {
    buf[0] = CMD_CHECK_TARGET_CONNECTION;
    if (stk500v2_command(pgm, buf, 1, sizeof(buf)) < 0) {
      fprintf(stderr, "%s: stk500v2_program_enable(): cannot get connection status\n",
            progname);
      return -1;
    }

    msg = "Unknown";

    for (i = 0;
	 i < sizeof connection_status / sizeof connection_status[0];
	 i++)
      if (connection_status[i].state == (unsigned)buf[2]) {
	msg = connection_status[i].description;
	break;
      }

    fprintf(stderr, "%s: stk500v2_program_enable():"
	    " bad STK600 connection status: %s (0x%02x)\n",
            progname, msg, buf[2]);
  }

  return rv;
}

/*
 * issue the 'program enable' command to the AVR device, parallel mode
 */
static int stk500pp_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[16];

  lastpart = p;

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

  lastpart = p;

  buf[0] = pgmtype == PGMTYPE_STK600?
  CMD_ENTER_PROGMODE_HVSP_STK600:
  CMD_ENTER_PROGMODE_HVSP;
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

  if (pgmtype == PGMTYPE_STK600 &&
      (p->flags & AVRPART_HAS_PDI) != 0) {
    /*
     * This is an ATxmega device, must use XPROG protocol for the
     * remaining actions.
     */
    stk600_setup_xprog(pgm);
  }

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

  if (result < 0) {
    fprintf(stderr,
	    "%s: stk500pp_initalize(): "
	    "failed to set control stack\n",
            progname);
    return -1;
  }

  /*
   * Examine the avrpart's memory definitions, and initialize the page
   * caches.  For devices/memory that are not page oriented, treat
   * them as page size 1 for EEPROM, and 2 for flash.
   */
  PDATA(pgm)->flash_pagesize = 2;
  PDATA(pgm)->eeprom_pagesize = 1;
  for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
    m = ldata(ln);
    if (strcmp(m->desc, "flash") == 0) {
      if (m->page_size > 0)
	PDATA(pgm)->flash_pagesize = m->page_size;
    } else if (strcmp(m->desc, "eeprom") == 0) {
      if (m->page_size > 0)
	PDATA(pgm)->eeprom_pagesize = m->page_size;
    }
  }
  free(PDATA(pgm)->flash_pagecache);
  free(PDATA(pgm)->eeprom_pagecache);
  if ((PDATA(pgm)->flash_pagecache = malloc(PDATA(pgm)->flash_pagesize)) == NULL) {
    fprintf(stderr, "%s: stk500pp_initialize(): Out of memory\n",
	    progname);
    return -1;
  }
  if ((PDATA(pgm)->eeprom_pagecache = malloc(PDATA(pgm)->eeprom_pagesize)) == NULL) {
    fprintf(stderr, "%s: stk500pp_initialize(): Out of memory\n",
	    progname);
    free(PDATA(pgm)->flash_pagecache);
    return -1;
  }
  PDATA(pgm)->flash_pageaddr = PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;

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

  if (result < 0) {
    fprintf(stderr,
            "%s: stk500v2_disable(): failed to leave programming mode\n",
            progname);
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

  free(PDATA(pgm)->flash_pagecache);
  PDATA(pgm)->flash_pagecache = NULL;
  free(PDATA(pgm)->eeprom_pagecache);
  PDATA(pgm)->eeprom_pagecache = NULL;

  buf[0] = mode == PPMODE? CMD_LEAVE_PROGMODE_PP:
  (pgmtype == PGMTYPE_STK600?
   CMD_LEAVE_PROGMODE_HVSP_STK600:
   CMD_LEAVE_PROGMODE_HVSP);
  buf[1] = 15;  // p->hvleavestabdelay;
  buf[2] = 15;  // p->resetdelay;

  result = stk500v2_command(pgm, buf, 3, sizeof(buf));

  if (result < 0) {
    fprintf(stderr,
	    "%s: stk500hv_disable(): "
	    "failed to leave programming mode\n",
            progname);
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

  pgmtype = PGMTYPE_UNKNOWN;

  if(strcasecmp(port, "avrdoper") == 0){
#if defined(HAVE_LIBUSB) || defined(WIN32NATIVE)
    serdev = &avrdoper_serdev;
    pgmtype = PGMTYPE_STK500;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

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
    pgmtype = PGMTYPE_AVRISP_MKII;
    pgm->set_sck_period = stk500v2_set_sck_period_mk2;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  serial_open(port, baud, &pgm->fd);

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


static int stk600_open(PROGRAMMER * pgm, char * port)
{
  long baud = 115200;

  DEBUG("STK500V2: stk600_open()\n");

  if (pgm->baudrate)
    baud = pgm->baudrate;

  pgmtype = PGMTYPE_UNKNOWN;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev_frame;
    baud = USB_DEVICE_STK600;
    pgmtype = PGMTYPE_STK600;
    pgm->set_sck_period = stk600_set_sck_period;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  serial_open(port, baud, &pgm->fd);

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

  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
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

  if (result < 0) {
    fprintf(stderr,
            "%s: stk500v2_loadaddr(): failed to set load address\n",
            progname);
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
  unsigned char buf[266];
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
    paddr_ptr = &PDATA(pgm)->flash_pageaddr;
    cache_ptr = PDATA(pgm)->flash_pagecache;
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
    paddr_ptr = &PDATA(pgm)->eeprom_pageaddr;
    cache_ptr = PDATA(pgm)->eeprom_pagecache;
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

  if (result < 0) {
    fprintf(stderr,
	    "%s: stk500hv_read_byte(): "
	    "timeout/error communicating with programmer\n",
	    progname);
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
  unsigned char buf[266];
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
    paddr_ptr = &PDATA(pgm)->flash_pageaddr;
    cache_ptr = PDATA(pgm)->flash_pagecache;
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
    paddr_ptr = &PDATA(pgm)->eeprom_pageaddr;
    cache_ptr = PDATA(pgm)->eeprom_pagecache;
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

  if (result < 0) {
    fprintf(stderr,
	    "%s: stk500hv_write_byte(): "
	    "timeout/error communicating with programmer\n",
	    progname);
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
    if (result < 0) {
      fprintf(stderr,
              "%s: stk500v2_paged_write: write command failed\n",
              progname);
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
    PDATA(pgm)->flash_pageaddr = (unsigned long)-1L;
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
    PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;
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
    if (result < 0) {
      fprintf(stderr,
              "%s: stk500hv_paged_write: write command failed\n",
              progname);
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
    if (result < 0) {
      fprintf(stderr,
              "%s: stk500v2_paged_load: read command failed\n",
              progname);
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
    if (result < 0) {
      fprintf(stderr,
              "%s: stk500hv_paged_load: read command failed\n",
              progname);
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


static int stk500v2_set_varef(PROGRAMMER * pgm, unsigned int chan /* unused */,
                              double v)
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
static double avrispmkIIfreqs[] = {
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

static int stk500v2_set_sck_period_mk2(PROGRAMMER * pgm, double v)
{
  int i;

  for (i = 0; i < sizeof(avrispmkIIfreqs); i++) {
    if (1 / avrispmkIIfreqs[i] >= v)
      break;
  }

  if (verbose > 2)
    fprintf(stderr, "Using p = %.2f us for SCK (param = %d)\n",
	    1000000 / avrispmkIIfreqs[i], i);

  return stk500v2_setparm(pgm, PARAM_SCK_DURATION, i);
}

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


static int stk600_set_vtarget(PROGRAMMER * pgm, double v)
{
  unsigned char utarg;
  unsigned int uaref;
  int rv;

  utarg = (unsigned)((v + 0.049) * 10);

  if (stk500v2_getparm2(pgm, PARAM2_AREF0, &uaref) != 0) {
    fprintf(stderr,
	    "%s: stk500v2_set_vtarget(): cannot obtain V[aref][0]\n",
	    progname);
    return -1;
  }

  if (uaref > (unsigned)utarg * 10) {
    fprintf(stderr,
	    "%s: stk500v2_set_vtarget(): reducing V[aref][0] from %.2f to %.1f\n",
	    progname, uaref / 100.0, v);
    uaref = 10 * (unsigned)utarg;
    if (stk500v2_setparm2(pgm, PARAM2_AREF0, uaref)
	!= 0)
      return -1;
  }

  if (stk500v2_getparm2(pgm, PARAM2_AREF1, &uaref) != 0) {
    fprintf(stderr,
	    "%s: stk500v2_set_vtarget(): cannot obtain V[aref][1]\n",
	    progname);
    return -1;
  }

  if (uaref > (unsigned)utarg * 10) {
    fprintf(stderr,
	    "%s: stk500v2_set_vtarget(): reducing V[aref][1] from %.2f to %.1f\n",
	    progname, uaref / 100.0, v);
    uaref = 10 * (unsigned)utarg;
    if (stk500v2_setparm2(pgm, PARAM2_AREF1, uaref)
	!= 0)
      return -1;
  }

  /*
   * Vtarget on the STK600 can only be adjusted while not being in
   * programming mode.
   */
  pgm->disable(pgm);
  rv = stk500v2_setparm(pgm, PARAM_VTARGET, utarg);
  pgm->program_enable(pgm, lastpart);

  return rv;
}


static int stk600_set_varef(PROGRAMMER * pgm, unsigned int chan, double v)
{
  unsigned char utarg;
  unsigned int uaref;

  uaref = (unsigned)((v + 0.0049) * 100);

  if (stk500v2_getparm(pgm, PARAM_VTARGET, &utarg) != 0) {
    fprintf(stderr,
	    "%s: stk500v2_set_varef(): cannot obtain V[target]\n",
	    progname);
    return -1;
  }

  if (uaref > (unsigned)utarg * 10) {
    fprintf(stderr,
	    "%s: stk500v2_set_varef(): V[aref] must not be greater than "
	    "V[target] = %.1f\n",
	    progname, utarg / 10.0);
    return -1;
  }

  switch (chan)
  {
  case 0:
    return stk500v2_setparm2(pgm, PARAM2_AREF0, uaref);

  case 1:
    return stk500v2_setparm2(pgm, PARAM2_AREF1, uaref);

  default:
    fprintf(stderr,
	    "%s: stk500v2_set_varef(): invalid channel %d\n",
	    progname, chan);
    return -1;
  }
}


static int stk600_set_fosc(PROGRAMMER * pgm, double v)
{
  unsigned int oct, dac;

  oct = 1.443 * log(v / 1039.0);
  dac = 2048 - (2078.0 * pow(2, (double)(10 + oct))) / v;

  return stk500v2_setparm2(pgm, PARAM2_CLOCK_CONF, (oct << 12) | (dac << 2));
}

static int stk600_set_sck_period(PROGRAMMER * pgm, double v)
{
  unsigned int sck;

  sck = ceil((16e6 / (2 * 1.0 / v)) - 1);

  if (sck >= 4096)
    sck = 4095;

  return stk500v2_setparm2(pgm, PARAM2_SCK_DURATION, sck);
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
  if (value == current_value && verbose > 2) {
    fprintf(stderr, "%s: Skipping paramter write; parameter value already set.\n", progname);
    return 0;
  }

  return stk500v2_setparm_real(pgm, parm, value);
}

static int stk500v2_getparm2(PROGRAMMER * pgm, unsigned char parm, unsigned int * value)
{
  unsigned char buf[32];

  buf[0] = CMD_GET_PARAMETER;
  buf[1] = parm;

  if (stk500v2_command(pgm, buf, 2, sizeof(buf)) < 0) {
    fprintf(stderr,"%s: stk500v2_getparm2(): failed to get parameter 0x%02x\n",
            progname, parm);
    return -1;
  }

  *value = ((unsigned)buf[2] << 8) | buf[3];

  return 0;
}

static int stk500v2_setparm2(PROGRAMMER * pgm, unsigned char parm, unsigned int value)
{
  unsigned char buf[32];

  buf[0] = CMD_SET_PARAMETER;
  buf[1] = parm;
  buf[2] = value >> 8;
  buf[3] = value;

  if (stk500v2_command(pgm, buf, 4, sizeof(buf)) < 0) {
    fprintf(stderr, "\n%s: stk500v2_setparm2(): failed to set parameter 0x%02x\n",
            progname, parm);
    return -1;
  }

  return 0;
}

/*
 * These two tables can be semi-automatically updated from
 * targetboards.xml using tools/get-stk600-cards.xsl.
 */
static struct carddata routing_cards[] =
{
  { 0x01, "STK600-RC020T-1" },
  { 0x03, "STK600-RC028T-3" },
  { 0x05, "STK600-RC040M-5" },
  { 0x08, "STK600-RC020T-8" },
  { 0x0A, "STK600-RC040M-4" },
  { 0x0C, "STK600-RC008T-2" },
  { 0x0D, "STK600-RC028M-6" },
  { 0x10, "STK600-RC064M-10" },
  { 0x11, "STK600-RC100M-11" },
  { 0x13, "STK600-RC100X-13" },
  { 0x18, "STK600-RC100M-18" },
  { 0x19, "STK600-RCPWM-19" },
  { 0x1B, "STK600-RC32U-20" },
  { 0x1C, "STK600-RC014T-12" },
  { 0x1E, "STK600-RC064U-17" },
  { 0x1F, "STK600-RCuC3B0-21" },
  { 0x20, "STK600-RCPWM-22" },
  { 0x21, "STK600-RC020T-23" },
  { 0x22, "STK600-RC044M-24" },
  { 0x23, "STK600-RC044U-25" },
  { 0x55, "STK600-RC064M-9" },
  { 0xA0, "STK600-RC008T-7" },
};

static struct carddata socket_cards[] =
{
  { 0x02, "STK600-TQFP32" },
  { 0x03, "STK600-TQFP100" },
  { 0x04, "STK600-SOIC" },
  { 0x09, "STK600-TinyX3U" },
  { 0x0C, "STK600-TSSOP44" },
  { 0x0D, "STK600-TQFP44" },
  { 0x0E, "STK600-TQFP64-2" },
  { 0x0F, "STK600-ATMEGA2560" },
  { 0x55, "STK600-TQFP64" },
  { 0x69, "STK600-uC3-144" },
  { 0xF1, "STK600-DIP" },
};

static const char *stk600_get_cardname(const struct carddata *table,
				       size_t nele, int id)
{
  const struct carddata *cdp;

  if (id == 0xFF)
    /* 0xFF means this card is not present at all. */
    return "Not present";

  for (cdp = table; nele > 0; cdp++, nele--)
    if (cdp->id == id)
      return cdp->name;

  return "Unknown";
}


static void stk500v2_display(PROGRAMMER * pgm, const char * p)
{
  unsigned char maj, min, hdw, topcard, maj_s1, min_s1, maj_s2, min_s2;
  unsigned int rev;
  const char *topcard_name, *pgmname;

  switch (pgmtype) {
    case PGMTYPE_UNKNOWN:     pgmname = "Unknown"; break;
    case PGMTYPE_STK500:      pgmname = "STK500"; break;
    case PGMTYPE_AVRISP:      pgmname = "AVRISP"; break;
    case PGMTYPE_AVRISP_MKII: pgmname = "AVRISP mkII"; break;
    case PGMTYPE_STK600:      pgmname = "STK600"; break;
    default:                  pgmname = "None";
  }
  if (pgmtype != PGMTYPE_JTAGICE_MKII) {
    fprintf(stderr, "%sProgrammer Model: %s\n", p, pgmname);
    stk500v2_getparm(pgm, PARAM_HW_VER, &hdw);
    stk500v2_getparm(pgm, PARAM_SW_MAJOR, &maj);
    stk500v2_getparm(pgm, PARAM_SW_MINOR, &min);
    fprintf(stderr, "%sHardware Version: %d\n", p, hdw);
    fprintf(stderr, "%sFirmware Version Master : %d.%02d\n", p, maj, min);
    if (pgmtype == PGMTYPE_STK600) {
      stk500v2_getparm(pgm, PARAM_SW_MAJOR_SLAVE1, &maj_s1);
      stk500v2_getparm(pgm, PARAM_SW_MINOR_SLAVE1, &min_s1);
      stk500v2_getparm(pgm, PARAM_SW_MAJOR_SLAVE2, &maj_s2);
      stk500v2_getparm(pgm, PARAM_SW_MINOR_SLAVE2, &min_s2);
      fprintf(stderr, "%sFirmware Version Slave 1: %d.%02d\n", p, maj_s1, min_s1);
      fprintf(stderr, "%sFirmware Version Slave 2: %d.%02d\n", p, maj_s2, min_s2);
    }
  }

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
  } else if (pgmtype == PGMTYPE_STK600) {
    stk500v2_getparm(pgm, PARAM_ROUTINGCARD_ID, &topcard);
    fprintf(stderr, "%sRouting card    : %s\n", p,
	    stk600_get_cardname(routing_cards,
				sizeof routing_cards / sizeof routing_cards[0],
				topcard));
    stk500v2_getparm(pgm, PARAM_SOCKETCARD_ID, &topcard);
    fprintf(stderr, "%sSocket card     : %s\n", p,
	    stk600_get_cardname(socket_cards,
				sizeof socket_cards / sizeof socket_cards[0],
				topcard));
    stk500v2_getparm2(pgm, PARAM2_RC_ID_TABLE_REV, &rev);
    fprintf(stderr, "%sRC_ID table rev : %d\n", p, rev);
    stk500v2_getparm2(pgm, PARAM2_EC_ID_TABLE_REV, &rev);
    fprintf(stderr, "%sEC_ID table rev : %d\n", p, rev);
  }
  stk500v2_print_parms1(pgm, p);

  return;
}

static double
f_to_kHz_MHz(double f, const char **unit)
{
  if (f > 1e6) {
    f /= 1e6;
    *unit = "MHz";
  } else if (f > 1e3) {
    f /= 1000;
    *unit = "kHz";
  } else
    *unit = "Hz";

  return f;
}

static void stk500v2_print_parms1(PROGRAMMER * pgm, const char * p)
{
  unsigned char vtarget, vadjust, osc_pscale, osc_cmatch, sck_duration;
  unsigned int sck_stk600, clock_conf, dac, oct, varef;
  unsigned char vtarget_jtag[4];
  int prescale;
  double f;
  const char *unit;

  if (pgmtype == PGMTYPE_JTAGICE_MKII) {
    jtagmkII_getparm(pgm, PAR_OCD_VTARGET, vtarget_jtag);
    fprintf(stderr, "%sVtarget         : %.1f V\n", p,
	    b2_to_u16(vtarget_jtag) / 1000.0);
  } else {
    stk500v2_getparm(pgm, PARAM_VTARGET, &vtarget);
    fprintf(stderr, "%sVtarget         : %.1f V\n", p, vtarget / 10.0);
  }

  switch (pgmtype) {
  case PGMTYPE_STK500:
    stk500v2_getparm(pgm, PARAM_SCK_DURATION, &sck_duration);
    stk500v2_getparm(pgm, PARAM_VADJUST, &vadjust);
    stk500v2_getparm(pgm, PARAM_OSC_PSCALE, &osc_pscale);
    stk500v2_getparm(pgm, PARAM_OSC_CMATCH, &osc_cmatch);
    fprintf(stderr, "%sSCK period      : %.1f us\n", p,
	  sck_duration * 8.0e6 / STK500V2_XTAL + 0.05);
    fprintf(stderr, "%sVaref           : %.1f V\n", p, vadjust / 10.0);
    fprintf(stderr, "%sOscillator      : ", p);
    if (osc_pscale == 0)
      fprintf(stderr, "Off\n");
    else {
      prescale = 1;
      f = STK500V2_XTAL / 2;

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
      f = f_to_kHz_MHz(f, &unit);
      fprintf(stderr, "%.3f %s\n", f, unit);
    }
    break;

  case PGMTYPE_AVRISP_MKII:
  case PGMTYPE_JTAGICE_MKII:
    stk500v2_getparm(pgm, PARAM_SCK_DURATION, &sck_duration);
    fprintf(stderr, "%sSCK period      : %.2f us\n", p,
	    (float) 1000000 / avrispmkIIfreqs[sck_duration]);
    break;

  case PGMTYPE_STK600:
    stk500v2_getparm2(pgm, PARAM2_AREF0, &varef);
    fprintf(stderr, "%sVaref 0         : %.2f V\n", p, varef / 100.0);
    stk500v2_getparm2(pgm, PARAM2_AREF1, &varef);
    fprintf(stderr, "%sVaref 1         : %.2f V\n", p, varef / 100.0);
    stk500v2_getparm2(pgm, PARAM2_SCK_DURATION, &sck_stk600);
    fprintf(stderr, "%sSCK period      : %.2f us\n", p,
	    (float) (sck_stk600 + 1) / 8.0);
    stk500v2_getparm2(pgm, PARAM2_CLOCK_CONF, &clock_conf);
    oct = (clock_conf & 0xf000) >> 12u;
    dac = (clock_conf & 0x0ffc) >> 2u;
    f = pow(2, (double)oct) * 2078.0 / (2 - (double)dac / 1024.0);
    f = f_to_kHz_MHz(f, &unit);
    fprintf(stderr, "%sOscillator      : %.3f %s\n",
            p, f, unit);
    break;

  default:
    fprintf(stderr, "%sSCK period      : %.1f us\n", p,
	  sck_duration * 8.0e6 / STK500V2_XTAL + 0.05);
    break;
  }

  return;
}


static void stk500v2_print_parms(PROGRAMMER * pgm)
{
  stk500v2_print_parms1(pgm, "");
}

static int stk500v2_perform_osccal(PROGRAMMER * pgm)
{
  unsigned char buf[32];
  int rv;

  buf[0] = CMD_OSCCAL;

  rv = stk500v2_command(pgm, buf, 1, sizeof(buf));
  if (rv < 0) {
    fprintf(stderr, "%s: stk500v2_perform_osccal(): failed\n",
            progname);
    return -1;
  }

  return 0;
}

/*
 * Wrapper functions for the JTAG ICE mkII in ISP mode.  This mode
 * uses the normal JTAG ICE mkII packet stream to communicate with the
 * ICE, but then encapsulates AVRISP mkII commands using
 * CMND_ISP_PACKET.
 */

/*
 * Open a JTAG ICE mkII in ISP mode.
 */
static int stk500v2_jtagmkII_open(PROGRAMMER * pgm, char * port)
{
  long baud;

  if (verbose >= 2)
    fprintf(stderr, "%s: stk500v2_jtagmkII_open()\n", progname);

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  baud = 19200;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    baud = USB_DEVICE_JTAGICEMKII;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  serial_open(port, baud, &pgm->fd);

  /*
   * drain any extraneous input
   */
  stk500v2_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_SPI) != 0) {
    fprintf(stderr, "%s: failed to sync with the JTAG ICE mkII in ISP mode\n",
            progname);
    pgm->close(pgm);		/* sign off correctly */
    exit(1);
  }

  pgmtype = PGMTYPE_JTAGICE_MKII;

  if (pgm->bitclock != 0.0) {
    if (pgm->set_sck_period(pgm, pgm->bitclock) != 0)
      return -1;
  }

  return 0;
}


/*
 * Wrapper functions for the AVR Dragon in ISP mode.  This mode
 * uses the normal JTAG ICE mkII packet stream to communicate with the
 * ICE, but then encapsulates AVRISP mkII commands using
 * CMND_ISP_PACKET.
 */

/*
 * Open an AVR Dragon in ISP mode.
 */
static int stk500v2_dragon_isp_open(PROGRAMMER * pgm, char * port)
{
  long baud;

  if (verbose >= 2)
    fprintf(stderr, "%s: stk500v2_dragon_isp_open()\n", progname);

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  baud = 19200;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    baud = USB_DEVICE_AVRDRAGON;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  serial_open(port, baud, &pgm->fd);

  /*
   * drain any extraneous input
   */
  stk500v2_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_SPI) != 0) {
    fprintf(stderr, "%s: failed to sync with the JTAG ICE mkII in ISP mode\n",
            progname);
    pgm->close(pgm);		/* sign off correctly */
    exit(1);
  }

  pgmtype = PGMTYPE_JTAGICE_MKII;

  if (pgm->bitclock != 0.0) {
    if (pgm->set_sck_period(pgm, pgm->bitclock) != 0)
      return -1;
  }

  return 0;
}


/*
 * Wrapper functions for the AVR Dragon in HV mode.  This mode
 * uses the normal JTAG ICE mkII packet stream to communicate with the
 * ICE, but then encapsulates AVRISP mkII commands using
 * CMND_ISP_PACKET.
 */

/*
 * Open an AVR Dragon in HV mode (HVSP or parallel).
 */
static int stk500v2_dragon_hv_open(PROGRAMMER * pgm, char * port)
{
  long baud;

  if (verbose >= 2)
    fprintf(stderr, "%s: stk500v2_dragon_hv_open()\n", progname);

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  baud = 19200;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    baud = USB_DEVICE_AVRDRAGON;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  serial_open(port, baud, &pgm->fd);

  /*
   * drain any extraneous input
   */
  stk500v2_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_HV) != 0) {
    fprintf(stderr, "%s: failed to sync with the JTAG ICE mkII in HV mode\n",
            progname);
    pgm->close(pgm);		/* sign off correctly */
    exit(1);
  }

  pgmtype = PGMTYPE_JTAGICE_MKII;

  if (pgm->bitclock != 0.0) {
    if (pgm->set_sck_period(pgm, pgm->bitclock) != 0)
      return -1;
  }

  return 0;
}

/*
 * XPROG wrapper
 */
static int stk600_xprog_command(PROGRAMMER * pgm, unsigned char *b,
                                unsigned int cmdsize, unsigned int responsesize)
{
    unsigned char *newb;
    unsigned int s;
    int rv;

    if (cmdsize < responsesize)
        s = responsesize;
    else
        s = cmdsize;

    if ((newb = malloc(s + 1)) == 0) {
        fprintf(stderr, "%s: stk600_xprog_cmd(): out of memory\n",
                progname);
        return -1;
    }

    newb[0] = CMD_XPROG;
    memcpy(newb + 1, b, cmdsize);
    rv = stk500v2_command(pgm, newb, cmdsize + 1, responsesize + 1);
    if (rv == 0) {
        memcpy(b, newb + 1, responsesize);
    }

    free(newb);

    return rv;
}


/*
 * issue the 'program enable' command to the AVR device, XPROG version
 */
static int stk600_xprog_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
    unsigned char buf[16];
    unsigned int eepagesize = 42;
    unsigned int nvm_base;
    AVRMEM *mem;

    if (p->nvm_base == 0) {
        fprintf(stderr,
                "%s: stk600_xprog_program_enable(): no nvm_base parameter for XPROG device\n",
                progname);
        return -1;
    }
    if ((mem = avr_locate_mem(p, "eeprom")) != NULL) {
        if (mem->page_size == 0) {
            fprintf(stderr,
                    "%s: stk600_xprog_program_enable(): no EEPROM page_size parameter for XPROG device\n",
                    progname);
            return -1;
        }
        eepagesize = mem->page_size;
    }

    buf[0] = CMD_XPROG_SETMODE;
    buf[1] = 0;                 /* PDI mode */
    if (stk500v2_command(pgm, buf, 2, sizeof(buf)) < 0) {
        fprintf(stderr,
                "%s: stk600_xprog_program_enable(): CMD_XPROG_SETMODE failed\n",
                progname);
        return -1;
    }

    buf[0] = XPRG_CMD_ENTER_PROGMODE;
    if (stk600_xprog_command(pgm, buf, 1, 2) < 0) {
        fprintf(stderr,
                "%s: stk600_xprog_program_enable(): XPRG_CMD_ENTER_PROGMODE failed\n",
                progname);
        return -1;
    }

    buf[0] = XPRG_CMD_SET_PARAM;
    buf[1] = XPRG_PARAM_NVMBASE;
    nvm_base = p->nvm_base;
    /*
     * The 0x01000000 appears to be an indication to the programmer
     * that the respective address is located in IO (i.e., SRAM)
     * memory address space rather than flash.  This is not documented
     * anywhere in AVR079 but matches what AVR Studio does.
     */
    nvm_base |= 0x01000000;
    buf[2] = nvm_base >> 24;
    buf[3] = nvm_base >> 16;
    buf[4] = nvm_base >> 8;
    buf[5] = nvm_base;
    if (stk600_xprog_command(pgm, buf, 6, 2) < 0) {
        fprintf(stderr,
                "%s: stk600_xprog_program_enable(): XPRG_CMD_SET_PARAM(XPRG_PARAM_NVMBASE) failed\n",
                progname);
        return -1;
    }

    if (mem != NULL) {
        buf[0] = XPRG_CMD_SET_PARAM;
        buf[1] = XPRG_PARAM_EEPPAGESIZE;
        buf[2] = eepagesize >> 8;
        buf[3] = eepagesize;
        if (stk600_xprog_command(pgm, buf, 4, 2) < 0) {
            fprintf(stderr,
                    "%s: stk600_xprog_program_enable(): XPRG_CMD_SET_PARAM(XPRG_PARAM_EEPPAGESIZE) failed\n",
                    progname);
            return -1;
        }
    }

    return 0;
}

static void stk600_xprog_disable(PROGRAMMER * pgm)
{
    unsigned char buf[2];

    buf[0] = XPRG_CMD_LEAVE_PROGMODE;
    if (stk600_xprog_command(pgm, buf, 1, 2) < 0) {
        fprintf(stderr,
                "%s: stk600_xprog_program_enable(): XPRG_CMD_LEAVE_PROGMODE failed\n",
                progname);
    }
}

static int stk600_xprog_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
				   unsigned long addr, unsigned char data)
{
    unsigned char b[10];

    /*
     * Fancy offsets everywhere.
     * This is probably what AVR079 means when writing about the
     * "TIF address space".
     */
    if (strcmp(mem->desc, "flash") == 0) {
        b[1] = XPRG_MEM_TYPE_APPL;
        addr += 0x00800000;
    } else if (strcmp(mem->desc, "boot") == 0) {
        b[1] = XPRG_MEM_TYPE_BOOT;
        addr += 0x00800000;
    } else if (strcmp(mem->desc, "eeprom") == 0) {
        b[1] = XPRG_MEM_TYPE_EEPROM;
        addr += 0x008c0000;
    } else if (strcmp(mem->desc, "lockbits") == 0) {
        b[1] = XPRG_MEM_TYPE_LOCKBITS;
        addr += 0x008f0000;
    } else if (strcmp(mem->desc, "usersig") == 0) {
        b[1] = XPRG_MEM_TYPE_USERSIG;
        addr += 0x008e0000;
    } else {
        fprintf(stderr,
                "%s: stk600_xprog_write_byte(): unknown memory \"%s\"\n",
                progname, mem->desc);
        return -1;
    }
    addr += mem->offset;

    b[0] = XPRG_CMD_WRITE_MEM;
    b[2] = 0;			/* pagemode: non-paged write */
    b[3] = addr >> 24;
    b[4] = addr >> 16;
    b[5] = addr >> 8;
    b[6] = addr;
    b[7] = 0;
    b[8] = 1;
    b[9] = data;
    if (stk600_xprog_command(pgm, b, 10, 2) < 0) {
        fprintf(stderr,
                "%s: stk600_xprog_write_byte(): XPRG_CMD_WRITE_MEM failed\n",
                progname);
        return -1;
    }
    return 0;
}


static int stk600_xprog_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                                  unsigned long addr, unsigned char * value)
{
    unsigned char b[8];

    /*
     * Fancy offsets everywhere.
     * This is probably what AVR079 means when writing about the
     * "TIF address space".
     */
    if (strcmp(mem->desc, "flash") == 0) {
        b[1] = XPRG_MEM_TYPE_APPL;
        addr += 0x00800000;
    } else if (strcmp(mem->desc, "boot") == 0) {
        b[1] = XPRG_MEM_TYPE_BOOT;
        addr += 0x00800000;
    } else if (strcmp(mem->desc, "eeprom") == 0) {
        b[1] = XPRG_MEM_TYPE_EEPROM;
        addr += 0x008c0000;
    } else if (strcmp(mem->desc, "signature") == 0) {
        b[1] = XPRG_MEM_TYPE_APPL;
        addr += 0x01000000;
    } else if (strncmp(mem->desc, "fuse", strlen("fuse")) == 0) {
        b[1] = XPRG_MEM_TYPE_FUSE;
        addr += 0x008f0000;
    } else if (strcmp(mem->desc, "lockbits") == 0) {
        b[1] = XPRG_MEM_TYPE_LOCKBITS;
        addr += 0x008f0000;
    } else if (strcmp(mem->desc, "calibration") == 0) {
        b[1] = XPRG_MEM_TYPE_FACTORY_CALIBRATION;
        addr += 0x008e0000;
    } else if (strcmp(mem->desc, "usersig") == 0) {
        b[1] = XPRG_MEM_TYPE_USERSIG;
        addr += 0x008e0000;
    } else {
        fprintf(stderr,
                "%s: stk600_xprog_read_byte(): unknown memory \"%s\"\n",
                progname, mem->desc);
        return -1;
    }
    addr += mem->offset;

    b[0] = XPRG_CMD_READ_MEM;
    b[2] = addr >> 24;
    b[3] = addr >> 16;
    b[4] = addr >> 8;
    b[5] = addr;
    b[6] = 0;
    b[7] = 1;
    if (stk600_xprog_command(pgm, b, 8, 3) < 0) {
        fprintf(stderr,
                "%s: stk600_xprog_read_byte(): XPRG_CMD_READ_MEM failed\n",
                progname);
        return -1;
    }
    *value = b[2];
    return 0;
}


static int stk600_xprog_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
				   int page_size, int n_bytes)
{
    unsigned char *b;
    unsigned int addr;
    unsigned int offset;
    unsigned char memtype;
    int n_bytes_orig = n_bytes;

    /*
     * The XPROG read command supports at most 256 bytes in one
     * transfer.
     */
    if (page_size > 256)
	page_size = 256;	/* not really a page size anymore */

    /*
     * Fancy offsets everywhere.
     * This is probably what AVR079 means when writing about the
     * "TIF address space".
     */
    if (strcmp(mem->desc, "flash") == 0) {
        memtype = XPRG_MEM_TYPE_APPL;
        addr = 0x00800000;
    } else if (strcmp(mem->desc, "boot") == 0) {
        memtype = XPRG_MEM_TYPE_BOOT;
        addr = 0x00800000;
    } else if (strcmp(mem->desc, "eeprom") == 0) {
        memtype = XPRG_MEM_TYPE_EEPROM;
        addr = 0x008c0000;
    } else {
        fprintf(stderr,
                "%s: stk600_xprog_paged_load(): unknown paged memory \"%s\"\n",
                progname, mem->desc);
        return -1;
    }

    if ((b = malloc(page_size + 2)) == NULL) {
	fprintf(stderr,
                "%s: stk600_xprog_paged_load(): out of memory\n",
                progname);
        return -1;
    }

    offset = 0;
    while (n_bytes != 0) {
	report_progress(offset, n_bytes_orig, NULL);
	b[0] = XPRG_CMD_READ_MEM;
	b[1] = memtype;
	b[2] = addr >> 24;
	b[3] = addr >> 16;
	b[4] = addr >> 8;
	b[5] = addr;
	b[6] = page_size >> 8;
	b[7] = page_size;
	if (stk600_xprog_command(pgm, b, 8, page_size + 2) < 0) {
	    fprintf(stderr,
		    "%s: stk600_xprog_paged_load(): XPRG_CMD_READ_MEM failed\n",
		    progname);
	    return -1;
	}
	memcpy(mem->buf + offset, b + 2, page_size);
	if (n_bytes < page_size) {
	    n_bytes = page_size;
	}
	offset += page_size;
	addr += page_size;
	n_bytes -= page_size;
    }
    free(b);

    return n_bytes_orig;
}

static int stk600_xprog_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
				    int page_size, int n_bytes)
{
    unsigned char *b;
    unsigned int addr;
    unsigned int offset;
    unsigned char memtype;
    int n_bytes_orig = n_bytes;
    size_t writesize;

    /*
     * The XPROG read command supports at most 256 bytes in one
     * transfer.
     */
    if (page_size > 512) {
	fprintf(stderr,
		"%s: stk600_xprog_paged_write(): cannot handle page size > 512\n",
		progname);
	return -1;
    }

    /*
     * Fancy offsets everywhere.
     * This is probably what AVR079 means when writing about the
     * "TIF address space".
     */
    if (strcmp(mem->desc, "flash") == 0) {
        memtype = XPRG_MEM_TYPE_APPL;
        addr = 0x00800000;
    } else if (strcmp(mem->desc, "boot") == 0) {
        memtype = XPRG_MEM_TYPE_BOOT;
        addr = 0x00800000;
    } else if (strcmp(mem->desc, "eeprom") == 0) {
        memtype = XPRG_MEM_TYPE_EEPROM;
        addr = 0x008c0000;
    } else {
        fprintf(stderr,
                "%s: stk600_xprog_paged_write(): unknown paged memory \"%s\"\n",
                progname, mem->desc);
        return -1;
    }

    if ((b = malloc(page_size + 9)) == NULL) {
	fprintf(stderr,
                "%s: stk600_xprog_paged_write(): out of memory\n",
                progname);
        return -1;
    }

    offset = 0;
    while (n_bytes != 0) {
	report_progress(offset, n_bytes_orig, NULL);

	if (page_size > 256) {
	    /*
	     * AVR079 is not quite clear.  While it suggests that
	     * downloading up to 512 bytes (256 words) were OK, it
	     * obviously isn't -- 512-byte pages on the ATxmega128A1
	     * are getting corrupted when written as a single piece.
	     * It writes random junk somewhere beyond byte 256.
	     * Splitting it into 256 byte chunks, and only setting the
	     * erase page / write page bits in the final chunk helps.
	     */
	    if (page_size % 256 != 0) {
		fprintf(stderr,
			"%s: stk600_xprog_paged_write(): page size not multiple of 256\n",
			progname);
		return -1;
	    }
	    unsigned int chunk;
	    for (chunk = 0; chunk < page_size; chunk += 256) {
                if (n_bytes < 256) {
                    memset(b + 9 + n_bytes, 0xff, 256 - n_bytes);
                    writesize = n_bytes;
                } else {
                    writesize = 256;
                }
		b[0] = XPRG_CMD_WRITE_MEM;
		b[1] = memtype;
		if (chunk + 256 == page_size) {
		    b[2] = 3;	/* last chunk: erase page | write page */
		} else {
		    b[2] = 0;	/* initial/intermediate chunk: just download */
		}
		b[3] = addr >> 24;
		b[4] = addr >> 16;
		b[5] = addr >> 8;
		b[6] = addr;
		b[7] = 1;
		b[8] = 0;
		memcpy(b + 9, mem->buf + offset, writesize);
		if (stk600_xprog_command(pgm, b, 256 + 9, 2) < 0) {
		    fprintf(stderr,
			    "%s: stk600_xprog_paged_write(): XPRG_CMD_WRITE_MEM failed\n",
			    progname);
		    return -1;
		}
		if (n_bytes < 256)
		    n_bytes = 256;

		offset += 256;
		addr += 256;
		n_bytes -= 256;
	    }
	} else {
	    if (n_bytes < page_size) {
		/*
		 * This can easily happen if the input file was not a
		 * multiple of the page size.
		 */
		memset(b + 9 + n_bytes, 0xff, page_size - n_bytes);
                writesize = n_bytes;
            } else {
                writesize = page_size;
            }
	    b[0] = XPRG_CMD_WRITE_MEM;
	    b[1] = memtype;
	    b[2] = 3;		/* erase page | write page */
	    b[3] = addr >> 24;
	    b[4] = addr >> 16;
	    b[5] = addr >> 8;
	    b[6] = addr;
	    b[7] = page_size >> 8;
	    b[8] = page_size;
	    memcpy(b + 9, mem->buf + offset, writesize);
	    if (stk600_xprog_command(pgm, b, page_size + 9, 2) < 0) {
		fprintf(stderr,
			"%s: stk600_xprog_paged_write(): XPRG_CMD_WRITE_MEM failed\n",
			progname);
		return -1;
	    }
	    if (n_bytes < page_size)
		n_bytes = page_size;

	    offset += page_size;
	    addr += page_size;
	    n_bytes -= page_size;
	}
    }
    free(b);

    return n_bytes_orig;
}

static int stk600_xprog_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
    unsigned char b[6];

    b[0] = XPRG_CMD_ERASE;
    b[1] = XPRG_ERASE_CHIP;
    b[2] = b[3] = b[4] = b[5] = 0;
    if (stk600_xprog_command(pgm, b, 6, 2) < 0) {
	    fprintf(stderr,
		    "%s: stk600_xprog_chip_erase(): XPRG_CMD_ERASE(XPRG_ERASE_CHIP) failed\n",
		    progname);
	    return -1;
	}
    return 0;
}

/*
 * Modify pgm's methods for XPROG operation.
 */
static void stk600_setup_xprog(PROGRAMMER * pgm)
{
    pgm->program_enable = stk600_xprog_program_enable;
    pgm->disable = stk600_xprog_disable;
    pgm->read_byte = stk600_xprog_read_byte;
    pgm->write_byte = stk600_xprog_write_byte;
    pgm->paged_load = stk600_xprog_paged_load;
    pgm->paged_write = stk600_xprog_paged_write;
    pgm->chip_erase = stk600_xprog_chip_erase;
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
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;

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
  pgm->perform_osccal = stk500v2_perform_osccal;
  pgm->setup          = stk500v2_setup;
  pgm->teardown       = stk500v2_teardown;
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
  pgm->open           = stk500v2_open;
  pgm->close          = stk500v2_close;
  pgm->read_byte      = stk500pp_read_byte;
  pgm->write_byte     = stk500pp_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500pp_paged_write;
  pgm->paged_load     = stk500pp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk500v2_set_vtarget;
  pgm->set_varef      = stk500v2_set_varef;
  pgm->set_fosc       = stk500v2_set_fosc;
  pgm->set_sck_period = stk500v2_set_sck_period;
  pgm->setup          = stk500v2_setup;
  pgm->teardown       = stk500v2_teardown;
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
  pgm->open           = stk500v2_open;
  pgm->close          = stk500v2_close;
  pgm->read_byte      = stk500hvsp_read_byte;
  pgm->write_byte     = stk500hvsp_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500hvsp_paged_write;
  pgm->paged_load     = stk500hvsp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk500v2_set_vtarget;
  pgm->set_varef      = stk500v2_set_varef;
  pgm->set_fosc       = stk500v2_set_fosc;
  pgm->set_sck_period = stk500v2_set_sck_period;
  pgm->setup          = stk500v2_setup;
  pgm->teardown       = stk500v2_teardown;
  pgm->page_size      = 256;
}

void stk500v2_jtagmkII_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "JTAGMKII_ISP");

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
  pgm->open           = stk500v2_jtagmkII_open;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500v2_paged_write;
  pgm->paged_load     = stk500v2_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_sck_period = stk500v2_set_sck_period_mk2;
  pgm->perform_osccal = stk500v2_perform_osccal;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->page_size      = 256;
}

void stk500v2_dragon_isp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "DRAGON_ISP");

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
  pgm->open           = stk500v2_dragon_isp_open;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500v2_paged_write;
  pgm->paged_load     = stk500v2_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_sck_period = stk500v2_set_sck_period_mk2;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->page_size      = 256;
}

void stk500v2_dragon_pp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "DRAGON_PP");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500pp_initialize;
  pgm->display        = stk500v2_display;
  pgm->enable         = stk500v2_enable;
  pgm->disable        = stk500pp_disable;
  pgm->program_enable = stk500pp_program_enable;
  pgm->chip_erase     = stk500pp_chip_erase;
  pgm->open           = stk500v2_dragon_hv_open;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = stk500pp_read_byte;
  pgm->write_byte     = stk500pp_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500pp_paged_write;
  pgm->paged_load     = stk500pp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk500v2_set_vtarget;
  pgm->set_varef      = stk500v2_set_varef;
  pgm->set_fosc       = stk500v2_set_fosc;
  pgm->set_sck_period = stk500v2_set_sck_period_mk2;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->page_size      = 256;
}

void stk500v2_dragon_hvsp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "DRAGON_HVSP");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500hvsp_initialize;
  pgm->display        = stk500v2_display;
  pgm->enable         = stk500v2_enable;
  pgm->disable        = stk500hvsp_disable;
  pgm->program_enable = stk500hvsp_program_enable;
  pgm->chip_erase     = stk500hvsp_chip_erase;
  pgm->open           = stk500v2_dragon_hv_open;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = stk500hvsp_read_byte;
  pgm->write_byte     = stk500hvsp_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500hvsp_paged_write;
  pgm->paged_load     = stk500hvsp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk500v2_set_vtarget;
  pgm->set_varef      = stk500v2_set_varef;
  pgm->set_fosc       = stk500v2_set_fosc;
  pgm->set_sck_period = stk500v2_set_sck_period_mk2;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->page_size      = 256;
}

void stk600_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK600");

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
  pgm->open           = stk600_open;
  pgm->close          = stk500v2_close;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500v2_paged_write;
  pgm->paged_load     = stk500v2_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk600_set_vtarget;
  pgm->set_varef      = stk600_set_varef;
  pgm->set_fosc       = stk600_set_fosc;
  pgm->set_sck_period = stk600_set_sck_period;
  pgm->perform_osccal = stk500v2_perform_osccal;
  pgm->setup          = stk500v2_setup;
  pgm->teardown       = stk500v2_teardown;
  pgm->page_size      = 256;
}

void stk600pp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK600PP");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500pp_initialize;
  pgm->display        = stk500v2_display;
  pgm->enable         = stk500v2_enable;
  pgm->disable        = stk500pp_disable;
  pgm->program_enable = stk500pp_program_enable;
  pgm->chip_erase     = stk500pp_chip_erase;
  pgm->open           = stk600_open;
  pgm->close          = stk500v2_close;
  pgm->read_byte      = stk500pp_read_byte;
  pgm->write_byte     = stk500pp_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500pp_paged_write;
  pgm->paged_load     = stk500pp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk600_set_vtarget;
  pgm->set_varef      = stk600_set_varef;
  pgm->set_fosc       = stk600_set_fosc;
  pgm->set_sck_period = stk600_set_sck_period;
  pgm->setup          = stk500v2_setup;
  pgm->teardown       = stk500v2_teardown;
  pgm->page_size      = 256;
}

void stk600hvsp_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK600HVSP");

  /*
   * mandatory functions
   */
  pgm->initialize     = stk500hvsp_initialize;
  pgm->display        = stk500v2_display;
  pgm->enable         = stk500v2_enable;
  pgm->disable        = stk500hvsp_disable;
  pgm->program_enable = stk500hvsp_program_enable;
  pgm->chip_erase     = stk500hvsp_chip_erase;
  pgm->open           = stk600_open;
  pgm->close          = stk500v2_close;
  pgm->read_byte      = stk500hvsp_read_byte;
  pgm->write_byte     = stk500hvsp_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500hvsp_paged_write;
  pgm->paged_load     = stk500hvsp_paged_load;
  pgm->print_parms    = stk500v2_print_parms;
  pgm->set_vtarget    = stk600_set_vtarget;
  pgm->set_varef      = stk600_set_varef;
  pgm->set_fosc       = stk600_set_fosc;
  pgm->set_sck_period = stk600_set_sck_period;
  pgm->setup          = stk500v2_setup;
  pgm->teardown       = stk500v2_teardown;
  pgm->page_size      = 256;
}

