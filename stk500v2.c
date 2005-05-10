/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005 Erik Walthinsen
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bsdhome.com>
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

/* $Id$
/* Based on Id: stk500.c,v 1.46 2004/12/22 01:52:45 bdean Exp */

/*
 * avrdude interface for Atmel STK500V2 programmer
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
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "avr.h"
#include "pgm.h"
#include "stk500_private.h"	// temp until all code converted
#include "stk500v2_private.h"
#include "serial.h"

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


extern int    verbose;
extern char * progname;
extern int do_cycles;

static unsigned char command_sequence = 1;


static int stk500v2_getparm(PROGRAMMER * pgm, unsigned char parm, unsigned char * value);
static int stk500v2_setparm(PROGRAMMER * pgm, unsigned char parm, unsigned char value);
static void stk500v2_print_parms1(PROGRAMMER * pgm, char * p);
static int stk500v2_is_page_empty(unsigned int address, int page_size,
                                  const unsigned char *buf);


static int stk500v2_send(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  unsigned char buf[275 + 6];		// max MESSAGE_BODY of 275 bytes, 6 bytes overhead
  int i;

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


static int stk500v2_recv(PROGRAMMER * pgm, unsigned char msg[], size_t maxsize) {
  enum states { sINIT, sSTART, sSEQNUM, sSIZE1, sSIZE2, sTOKEN, sDATA, sCSUM, sDONE }  state = sSTART;
  int msglen = 0;
  int curlen = 0;
  int timeout = 0;
  unsigned char c, checksum = 0;

  long timeoutval = 5;		// seconds
  struct timeval tv;
  double tstart, tnow;

  DEBUG("STK500V2: stk500v2_recv(): ");

  gettimeofday(&tv, NULL);
  tstart = tv.tv_sec;

  while ( (state != sDONE ) && (!timeout) ) {
    serial_recv(pgm->fd, &c, 1);
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
          fprintf(stderr, "%s: stk500v2_recv(): buffer too small, received %d byte into %d byte buffer\n",
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
    if (resp[0] == STATUS_CMD_OK) {
      // success!
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

static int stk500v2_command(PROGRAMMER * pgm, char * buf, size_t len, size_t maxlen) {
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
 * initialize the AVR device and prepare it to accept commands
 */
static int stk500v2_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  return pgm->program_enable(pgm, p);
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

static void stk500v2_enable(PROGRAMMER * pgm)
{
  return;
}


static int stk500v2_open(PROGRAMMER * pgm, char * port)
{
  DEBUG("STK500V2: stk500v2_open()\n");

  strcpy(pgm->port, port);
  if (pgm->baudrate)
    pgm->fd = serial_open(port, pgm->baudrate);
  else
    pgm->fd = serial_open(port, 115200);

  /*
   * drain any extraneous input
   */
  stk500v2_drain(pgm, 0);

  stk500v2_getsync(pgm);

  stk500v2_drain(pgm, 0);

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


static int stk500v2_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  int addr, block_size;
  unsigned char commandbuf[10];
  unsigned char buf[266];
  unsigned char cmds[4];
  int result;

  DEBUG("STK500V2: stk500v2_paged_write(..,%s,%d,%d)\n",m->desc,page_size,n_bytes);

  if (page_size == 0) page_size = 256;

  // determine which command is to be used
  if (strcmp(m->desc, "flash") == 0) {
    commandbuf[0] = CMD_PROGRAM_FLASH_ISP;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    commandbuf[0] = CMD_PROGRAM_EEPROM_ISP;
  }
  commandbuf[4] = m->delay;

  // if the memory is paged, load the appropriate commands into the buffer
  if (m->mode & 0x01) {
    commandbuf[3] = m->mode | 0x80;		// yes, write the stupid page to flash

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
  } else {
    commandbuf[3] = m->mode | 0x80;		// yes, write the stupid words to flash

    if (m->op[AVR_OP_WRITE_LO] == NULL) {
      fprintf(stderr, "%s: stk500v2_paged_write: write instruction not defined for part \"%s\"\n",
              progname, p->desc);
      return -1;
    }
    avr_set_bits(m->op[AVR_OP_WRITE_LO], cmds);
    commandbuf[6] = cmds[0];
  }

  // the read command is common to both methods
  if (m->op[AVR_OP_READ_LO] == NULL) {
    fprintf(stderr, "%s: stk500v2_paged_write: read instruction not defined for part \"%s\"\n",
            progname, p->desc);
    return -1;
  }
  avr_set_bits(m->op[AVR_OP_READ_LO], cmds);
  commandbuf[7] = cmds[0];

  commandbuf[8] = m->readback[0];
  commandbuf[9] = m->readback[1];

  stk500v2_loadaddr(pgm, 0);

  for (addr=0; addr < n_bytes; addr += page_size) {
    report_progress(addr,n_bytes,NULL);

    if ((n_bytes-addr) < page_size)
      block_size = n_bytes - addr;
    else
      block_size = page_size;
    DEBUG("block_size at addr %d is %d\n",addr,block_size);

    memcpy(buf,commandbuf,sizeof(commandbuf));

    buf[1] = block_size >> 8;
    buf[2] = block_size & 0xff;

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
  int addr, block_size;
  unsigned char commandbuf[4];
  unsigned char buf[275];	// max buffer size for stk500v2 at this point
  unsigned char cmds[4];
  int result, i;

  DEBUG("STK500V2: stk500v2_paged_load(..,%s,%d,%d)\n",m->desc,page_size,n_bytes);

  page_size = m->readsize;

  // determine which command is to be used
  if (strcmp(m->desc, "flash") == 0) {
    commandbuf[0] = CMD_READ_FLASH_ISP;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    commandbuf[0] = CMD_READ_EEPROM_ISP;
  }

  // the read command is common to both methods
  if (m->op[AVR_OP_READ_LO] == NULL) {
    fprintf(stderr, "%s: stk500v2_paged_write: read instruction not defined for part \"%s\"\n",
            progname, p->desc);
    return -1;
  }
  avr_set_bits(m->op[AVR_OP_READ_LO], cmds);
  commandbuf[3] = cmds[0];

  stk500v2_loadaddr(pgm, 0);

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

    result = stk500v2_command(pgm,buf,4,sizeof(buf));
    if (buf[1] != STATUS_CMD_OK) {
      fprintf(stderr,"%s: stk500v2_paged_write: read command failed with %d\n",
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

  return 0;
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


static int stk500v2_setparm(PROGRAMMER * pgm, unsigned char parm, unsigned char value)
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


static void stk500v2_display(PROGRAMMER * pgm, char * p)
{
  unsigned char maj, min, hdw, topcard;
  const char *topcard_name;

  stk500v2_getparm(pgm, PARAM_HW_VER, &hdw);
  stk500v2_getparm(pgm, PARAM_SW_MAJOR, &maj);
  stk500v2_getparm(pgm, PARAM_SW_MINOR, &min);
  stk500v2_getparm(pgm, PARAM_TOPCARD_DETECT, &topcard);

  fprintf(stderr, "%sHardware Version: %d\n", p, hdw);
  fprintf(stderr, "%sFirmware Version: %d.%d\n", p, maj, min);

  if (1) {			// should check to see if it's a stk500 first
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
