/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005 Joerg Wunsch <j@uriah.heep.sax.de>
 *
 * Derived from stk500 code which is:
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2005 Erik Walthinsen
 *
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
 * avrdude interface for Atmel JTAG ICE mkII programmer
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
#include "crc16.h"
#include "pgm.h"
#include "jtagmkII_private.h"
#include "serial.h"


extern int    verbose;
extern char * progname;
extern int do_cycles;

/*
 * XXX There should really be a programmer-specific private data
 * pointer in struct PROGRAMMER.
 */
static unsigned short command_sequence; /* Next cmd seqno to issue. */

/*
 * See jtagmkII_read_byte() for an explanation of the flash and
 * EEPROM page caches.
 */
static unsigned char *flash_pagecache;
static unsigned long flash_pageaddr;
static unsigned int flash_pagesize;

static unsigned char *eeprom_pagecache;
static unsigned long eeprom_pageaddr;
static unsigned int eeprom_pagesize;

static int prog_enabled;	/* Cached value of PROGRAMMING status. */
static unsigned char serno[6];	/* JTAG ICE serial number. */
/*
 * The OCDEN fuse is bit 7 of the high fuse (hfuse).  In order to
 * perform memory operations on MTYPE_SPM and MTYPE_EEPROM, OCDEN
 * needs to be programmed.
 *
 * OCDEN should probably rather be defined via the configuration, but
 * if this ever changes to a different fuse byte for one MCU, quite
 * some code here needs to be generalized anyway.
 */
#define OCDEN (1 << 7)

/* The length of the device descriptor is firmware-dependent. */
static size_t device_descriptor_length;

static int jtagmkII_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			      unsigned long addr, unsigned char * value);
static int jtagmkII_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			       unsigned long addr, unsigned char data);
static int jtagmkII_set_sck_period(PROGRAMMER * pgm, double v);
static int jtagmkII_getparm(PROGRAMMER * pgm, unsigned char parm,
			    unsigned char * value);
static int jtagmkII_setparm(PROGRAMMER * pgm, unsigned char parm,
			    unsigned char * value);
static void jtagmkII_print_parms1(PROGRAMMER * pgm, char * p);

static unsigned long
b4_to_u32(unsigned char *b)
{
  unsigned long l;
  l = b[0];
  l += (unsigned)b[1] << 8;
  l += (unsigned)b[2] << 16;
  l += (unsigned)b[3] << 24;

  return l;
}

static void
u32_to_b4(unsigned char *b, unsigned long l)
{
  b[0] = l & 0xff;
  b[1] = (l >> 8) & 0xff;
  b[2] = (l >> 16) & 0xff;
  b[3] = (l >> 24) & 0xff;
}

static unsigned short
b2_to_u16(unsigned char *b)
{
  unsigned short l;
  l = b[0];
  l += (unsigned)b[1] << 8;

  return l;
}

static void
u16_to_b2(unsigned char *b, unsigned short l)
{
  b[0] = l & 0xff;
  b[1] = (l >> 8) & 0xff;
}

static void jtagmkII_print_memory(unsigned char *b, size_t s)
{
  int i;

  if (s < 2)
    return;

  for (i = 0; i < s - 1; i++) {
    fprintf(stderr, "0x%02x ", b[i + 1]);
    if (i % 16 == 15)
      putc('\n', stderr);
    else
      putc(' ', stderr);
  }
  if (i % 16 != 0)
    putc('\n', stderr);
}

static void jtagmkII_prmsg(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  int i;

  if (verbose >= 4) {
    fprintf(stderr, "Raw message:\n");

    for (i = 0; i < len; i++) {
      fprintf(stderr, "0x%02x", data[i]);
      if (i % 16 == 15)
	putc('\n', stderr);
      else
	putchar(' ');
    }
    if (i % 16 != 0)
      putc('\n', stderr);
  }

  switch (data[0]) {
  case RSP_OK:
    fprintf(stderr, "OK\n");
    break;

  case RSP_FAILED:
    fprintf(stderr, "FAILED\n");
    break;

  case RSP_ILLEGAL_BREAKPOINT:
    fprintf(stderr, "Illegal breakpoint\n");
    break;

  case RSP_ILLEGAL_COMMAND:
    fprintf(stderr, "Illegal command\n");
    break;

  case RSP_ILLEGAL_EMULATOR_MODE:
    fprintf(stderr, "Illegal emulator mode");
    if (len > 1)
      switch (data[1]) {
      case EMULATOR_MODE_DEBUGWIRE: fprintf(stderr, ": DebugWire"); break;
      case EMULATOR_MODE_JTAG:      fprintf(stderr, ": JTAG"); break;
      case EMULATOR_MODE_UNKNOWN:   fprintf(stderr, ": Unknown"); break;
      case EMULATOR_MODE_SPI:       fprintf(stderr, ": SPI"); break;
      }
    putc('\n', stderr);
    break;

  case RSP_ILLEGAL_JTAG_ID:
    fprintf(stderr, "Illegal JTAG ID\n");
    break;

  case RSP_ILLEGAL_MCU_STATE:
    fprintf(stderr, "Illegal MCU state");
    if (len > 1)
      switch (data[1]) {
      case STOPPED:     fprintf(stderr, ": Stopped"); break;
      case RUNNING:     fprintf(stderr, ": Running"); break;
      case PROGRAMMING: fprintf(stderr, ": Programming"); break;
      }
    putc('\n', stderr);
    break;

  case RSP_ILLEGAL_MEMORY_TYPE:
    fprintf(stderr, "Illegal memory type\n");
    break;

  case RSP_ILLEGAL_MEMORY_RANGE:
    fprintf(stderr, "Illegal memory range\n");
    break;

  case RSP_ILLEGAL_PARAMETER:
    fprintf(stderr, "Illegal parameter\n");
    break;

  case RSP_ILLEGAL_POWER_STATE:
    fprintf(stderr, "Illegal power state\n");
    break;

  case RSP_ILLEGAL_VALUE:
    fprintf(stderr, "Illegal value\n");
    break;

  case RSP_NO_TARGET_POWER:
    fprintf(stderr, "No target power\n");
    break;

  case RSP_SIGN_ON:
    fprintf(stderr, "Sign-on succeeded\n");
    /* Sign-on data will be printed below anyway. */
    break;

  case RSP_MEMORY:
    fprintf(stderr, "memory contents:\n");
    jtagmkII_print_memory(data, len);
    break;

  case RSP_PARAMETER:
    fprintf(stderr, "parameter values:\n");
    jtagmkII_print_memory(data, len);
    break;

  case RSP_SPI_DATA:
    fprintf(stderr, "SPI data returned:\n");
    for (i = 1; i < len; i++)
      fprintf(stderr, "0x%02x ", data[i]);
    putc('\n', stderr);
    break;

  case EVT_BREAK:
    fprintf(stderr, "BREAK event");
    if (len >= 6) {
      fprintf(stderr, ", PC = 0x%lx, reason ", b4_to_u32(data + 1));
      switch (data[5]) {
      case 0x00:
	fprintf(stderr, "unspecified");
	break;
      case 0x01:
	fprintf(stderr, "program break");
	break;
      case 0x02:
	fprintf(stderr, "data break PDSB");
	break;
      case 0x03:
	fprintf(stderr, "data break PDMSB");
	break;
      default:
	fprintf(stderr, "unknown: 0x%02x", data[5]);
      }
    }
    putc('\n', stderr);
    break;

  default:
    fprintf(stderr, "unknown message 0x%02x\n", data[0]);
  }

  putc('\n', stderr);
}


static int jtagmkII_send(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  unsigned char *buf;

  if (verbose >= 3)
    fprintf(stderr, "\n%s: jtagmkII_send(): sending %zd bytes\n",
	    progname, len);

  if ((buf = malloc(len + 10)) == NULL)
    {
      fprintf(stderr, "%s: jtagmkII_send(): out of memory",
	      progname);
      return -1;
    }

  buf[0] = MESSAGE_START;
  u16_to_b2(buf + 1, command_sequence);
  u32_to_b4(buf + 3, len);
  buf[7] = TOKEN;
  memcpy(buf + 8, data, len);

  crcappend(buf, len + 8);

  if (serial_send(pgm->fd, buf, len + 10) != 0) {
    fprintf(stderr,
	    "%s: jtagmkII_send(): failed to send command to serial port\n",
	    progname);
    exit(1);
  }

  free(buf);

  return 0;
}


static int jtagmkII_drain(PROGRAMMER * pgm, int display)
{
  return serial_drain(pgm->fd, display);
}


/*
 * Receive one frame, return it in *msg.  Received sequence number is
 * returned in seqno.  Any valid frame will be returned, regardless
 * whether it matches the expected sequence number, including event
 * notification frames (seqno == 0xffff).
 *
 * Caller must eventually free the buffer.
 */
static int jtagmkII_recv_frame(PROGRAMMER * pgm, unsigned char **msg,
			       unsigned short * seqno) {
  enum states { sSTART,
		/* NB: do NOT change the sequence of the following: */
		sSEQNUM1, sSEQNUM2,
		sSIZE1, sSIZE2, sSIZE3, sSIZE4,
		sTOKEN,
		sDATA,
		sCSUM1, sCSUM2,
		/* end NB */
		sDONE
  }  state = sSTART;
  unsigned long msglen = 0, l = 0;
  int headeridx = 0;
  int timeout = 0;
  int ignorpkt = 0;
  int rv;
  unsigned char c, *buf = NULL, header[8];
  unsigned short r_seqno = 0;
  unsigned short checksum = 0;

  struct timeval tv;
  double timeoutval = 5;	/* seconds */
  double tstart, tnow;

  if (verbose >= 3)
    fprintf(stderr, "%s: jtagmkII_recv():\n", progname);

  gettimeofday(&tv, NULL);
  tstart = tv.tv_sec;

  while ( (state != sDONE ) && (!timeout) ) {
    if (state == sDATA) {
      rv = 0;
      if (ignorpkt) {
	/* skip packet's contents */
	for(l = 0; l < msglen; l++)
	  rv += serial_recv(pgm->fd, &c, 1);
      } else {
	rv += serial_recv(pgm->fd, buf + 8, msglen);
      }
      if (rv != 0) {
	timedout:
	/* timeout in receive */
	if (verbose > 1)
	  fprintf(stderr,
		  "%s: jtagmkII_recv(): Timeout receiving packet\n",
		  progname);
	free(buf);
	return -1;
      }
    } else {
      if (serial_recv(pgm->fd, &c, 1) != 0)
	goto timedout;
    }
    checksum ^= c;

    if (state < sDATA)
      header[headeridx++] = c;

    switch (state) {
      case sSTART:
        if (c == MESSAGE_START) {
          state = sSEQNUM1;
        } else {
	  headeridx = 0;
	}
        break;
      case sSEQNUM1:
      case sSEQNUM2:
	r_seqno >>= 8;
	r_seqno |= ((unsigned)c << 8);
	state++;
	break;
      case sSIZE1:
      case sSIZE2:
      case sSIZE3:
      case sSIZE4:
	msglen >>= 8;
	msglen |= ((unsigned)c << 24);
        state++;
        break;
      case sTOKEN:
        if (c == TOKEN) {
	  state = sDATA;
	  if (msglen > MAX_MESSAGE) {
	    fprintf(stderr,
		    "%s: jtagmkII_recv(): msglen %lu exceeds max message "
		    "size %u, ignoring message\n",
		    progname, msglen, MAX_MESSAGE);
	    state = sSTART;
	    headeridx = 0;
	  } else if ((buf = malloc(msglen + 10)) == NULL) {
	    fprintf(stderr, "%s: jtagmkII_recv(): out of memory\n",
		    progname);
	    ignorpkt++;
	  } else {
	    memcpy(buf, header, 8);
	  }
	} else {
	  state = sSTART;
	  headeridx = 0;
	}
        break;
      case sDATA:
	/* The entire payload has been read above. */
	l = msglen + 8;
        state = sCSUM1;
        break;
      case sCSUM1:
      case sCSUM2:
	buf[l++] = c;
	if (state == sCSUM2) {
	  if (crcverify(buf, msglen + 10)) {
	    if (verbose >= 3)
	      fprintf(stderr, "%s: jtagmkII_recv(): CRC OK",
		      progname);
	    state = sDONE;
	  } else {
	    fprintf(stderr, "%s: jtagmkII_recv(): checksum error\n",
		    progname);
	    free(buf);
	    return -4;
	  }
	} else
	  state++;
        break;
      default:
        fprintf(stderr, "%s: jtagmkII_recv(): unknown state\n",
                progname);
	free(buf);
        return -5;
     }

     gettimeofday(&tv, NULL);
     tnow = tv.tv_sec;
     if (tnow - tstart > timeoutval) {
       fprintf(stderr, "%s: jtagmkII_recv_frame(): timeout\n",
               progname);
       return -1;
     }

  }
  if (verbose >= 3)
fprintf(stderr, "\n");

  *seqno = r_seqno;
  *msg = buf;

  return msglen;
}

static int jtagmkII_recv(PROGRAMMER * pgm, unsigned char **msg) {
  unsigned short r_seqno;
  int rv;

  for (;;) {
    if ((rv = jtagmkII_recv_frame(pgm, msg, &r_seqno)) <= 0)
      return rv;
    if (verbose >= 3)
      fprintf(stderr, "%s: jtagmkII_recv(): "
	      "Got message seqno %d (command_sequence == %d)\n",
	      progname, r_seqno, command_sequence);
    if (r_seqno == command_sequence) {
      if (++command_sequence == 0xffff)
	command_sequence = 0;
      /*
       * We move the payload to the beginning of the buffer, to make
       * the job easier for the caller.  We have to return the
       * original pointer though, as the caller must free() it.
       */
      memmove(*msg, *msg + 8, rv);
      return rv;
    }
    if (r_seqno == 0xffff) {
      if (verbose >= 3)
	fprintf(stderr, "%s: jtagmkII_recv(): got asynchronous event\n",
		progname);
    } else {
      if (verbose >= 2)
	fprintf(stderr, "%s: jtagmkII_recv(): "
		"got wrong sequence number, %u != %u\n",
		progname, r_seqno, command_sequence);
    }
    free(*msg);
  }
  return 0;
}


static int jtagmkII_getsync(PROGRAMMER * pgm) {
  int tries;
#define MAXTRIES 33
  unsigned char buf[3], *resp, c = 0xff;
  int status;
  unsigned int fwver;

  if (verbose >= 3)
    fprintf(stderr, "%s: jtagmkII_getsync()\n", progname);

  for (tries = 0; tries < MAXTRIES; tries++) {

    /* Get the sign-on information. */
    buf[0] = CMND_GET_SIGN_ON;
    if (verbose >= 2)
      fprintf(stderr, "%s: jtagmkII_getsync(): Sending sign-on command: ",
	      progname);
    jtagmkII_send(pgm, buf, 1);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
	fprintf(stderr, "%s: jtagmkII_getsync(): sign-on command: "
		"status %d\n",
		progname, status);
    } else if (verbose >= 3) {
      putc('\n', stderr);
      jtagmkII_prmsg(pgm, resp, status);
    } else if (verbose == 2)
      fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);

    if (status > 0) {
      if ((c = resp[0]) == RSP_SIGN_ON) {
	fwver = ((unsigned)resp[8] << 8) | (unsigned)resp[7];
	memcpy(serno, resp + 10, 6);
	if (verbose >= 1 && status > 17) {
	  fprintf(stderr, "JTAG ICE mkII sign-on message:\n");
	  fprintf(stderr, "Communications protocol version: %u\n",
		  (unsigned)resp[1]);
	  fprintf(stderr, "M_MCU:\n");
	  fprintf(stderr, "  boot-loader FW version:        %u\n",
		  (unsigned)resp[2]);
	  fprintf(stderr, "  firmware version:              %u.%02u\n",
		  (unsigned)resp[4], (unsigned)resp[3]);
	  fprintf(stderr, "  hardware version:              %u\n",
		  (unsigned)resp[5]);
	  fprintf(stderr, "S_MCU:\n");
	  fprintf(stderr, "  boot-loader FW version:        %u\n",
		  (unsigned)resp[6]);
	  fprintf(stderr, "  firmware version:              %u.%02u\n",
		  (unsigned)resp[8], (unsigned)resp[7]);
	  fprintf(stderr, "  hardware version:              %u\n",
		  (unsigned)resp[9]);
	  fprintf(stderr, "Serial number:                   "
		  "%02x:%02x:%02x:%02x:%02x:%02x\n",
		  serno[0], serno[1], serno[2], serno[3], serno[4], serno[5]);
	  resp[status - 1] = '\0';
	  fprintf(stderr, "Device ID:                       %s\n",
		  resp + 16);
	}
	break;
      }
      free(resp);
    }
  }
  if (tries >= MAXTRIES) {
    if (status <= 0)
      fprintf(stderr,
	      "%s: jtagmkII_getsync(): "
	      "timeout/error communicating with programmer (status %d)\n",
	      progname, status);
    else
      fprintf(stderr,
	      "%s: jtagmkII_getsync(): "
	      "bad response to sign-on command: 0x%02x\n",
	      progname, c);
    return -1;
  }

  device_descriptor_length = sizeof(struct device_descriptor);
  /*
   * There's no official documentation from Atmel about what firmware
   * revision matches what device descriptor length.  The algorithm
   * below has been found empirically.
   */
#define FWVER(maj, min) ((maj << 8) | (min))
  if (fwver < FWVER(3, 16)) {
    device_descriptor_length -= 2;
    fprintf(stderr,
	    "%s: jtagmkII_getsync(): "
	    "S_MCU firmware version might be too old to work correctly\n",
	    progname);
  } else if (fwver < FWVER(4, 0)) {
    device_descriptor_length -= 2;
  }
#undef FWVER
  if (verbose >= 2)
    fprintf(stderr,
	    "%s: jtagmkII_getsync(): Using a %zu-byte device descriptor\n",
	    progname, device_descriptor_length);

  /* Turn the ICE into JTAG mode */
  buf[0] = EMULATOR_MODE_JTAG;
  if (jtagmkII_setparm(pgm, PAR_EMULATOR_MODE, buf) < 0)
    return -1;

  /* GET SYNC forces the target into STOPPED mode */
  buf[0] = CMND_GET_SYNC;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_getsync(): Sending get sync command: ",
	    progname);
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_getsync(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_getsync(): "
	    "bad response to set parameter command: 0x%02x\n",
	    progname, c);
    return -1;
  }

  return 0;
}

static int jtagmkII_cmd(PROGRAMMER * pgm, unsigned char cmd[4],
                        unsigned char res[4])
{

  fprintf(stderr, "%s: jtagmkII_command(): no direct SPI supported for JTAG\n",
	  progname);
  return -1;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
static int jtagmkII_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  int status;
  unsigned char buf[1], *resp, c;

  buf[0] = CMND_CHIP_ERASE;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_chip_erase(): Sending chip erase command: ",
	    progname);
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_chip_erase(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_chip_erase(): "
	    "bad response to chip erase command: 0x%02x\n",
	    progname, c);
    return -1;
  }

  pgm->initialize(pgm, p);

  return 0;
}

static void jtagmkII_set_devdescr(PROGRAMMER * pgm, AVRPART * p)
{
  int status;
  unsigned char *resp, c;
  LNODEID ln;
  AVRMEM * m;
  struct {
    unsigned char cmd;
    struct device_descriptor dd;
  } sendbuf;

  memset(&sendbuf, 0, sizeof sendbuf);
  sendbuf.cmd = CMND_SET_DEVICE_DESCRIPTOR;
  sendbuf.dd.ucSPMCRAddress = p->spmcr;
  sendbuf.dd.ucRAMPZAddress = p->rampz;
  sendbuf.dd.ucIDRAddress = p->idr;
  u16_to_b2(sendbuf.dd.EECRAddress, p->eecr);
  sendbuf.dd.ucAllowFullPageBitstream =
    (p->flags & AVRPART_ALLOWFULLPAGEBITSTREAM) != 0;
  sendbuf.dd.EnablePageProgramming =
    (p->flags & AVRPART_ENABLEPAGEPROGRAMMING) != 0;
  for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
    m = ldata(ln);
    if (strcmp(m->desc, "flash") == 0) {
      flash_pagesize = m->page_size;
      u32_to_b4(sendbuf.dd.ulFlashSize, m->size);
      u16_to_b2(sendbuf.dd.uiFlashPageSize, flash_pagesize);
      u16_to_b2(sendbuf.dd.uiFlashpages, m->size / flash_pagesize);
    } else if (strcmp(m->desc, "eeprom") == 0) {
      sendbuf.dd.ucEepromPageSize = eeprom_pagesize = m->page_size;
    }
  }

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_set_devdescr(): "
	    "Sending set device descriptor command: ",
	    progname);
  jtagmkII_send(pgm, (unsigned char *)&sendbuf,
		device_descriptor_length + sizeof(unsigned char));

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_set_devdescr(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_set_devdescr(): "
	    "bad response to set device descriptor command: 0x%02x\n",
	    progname, c);
  }
}

/*
 * Reset the target.
 */
static int jtagmkII_reset(PROGRAMMER * pgm)
{
  int status;
  unsigned char buf[1], *resp, c;

  buf[0] = CMND_RESET;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_reset(): Sending reset command: ",
	    progname);
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_reset(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_reset(): "
	    "bad response to reset command: 0x%02x\n",
	    progname, c);
    return -1;
  }

  return 0;
}

static int jtagmkII_program_enable_dummy(PROGRAMMER * pgm, AVRPART * p)
{

  return 0;
}

static int jtagmkII_program_enable(PROGRAMMER * pgm)
{
  int status;
  unsigned char buf[1], *resp, c;

  if (prog_enabled)
    return 0;

  buf[0] = CMND_ENTER_PROGMODE;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_program_enable(): "
	    "Sending enter progmode command: ",
	    progname);
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_program_enable(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_program_enable(): "
	    "bad response to enter progmode command: 0x%02x\n",
	    progname, c);
    if (c == RSP_ILLEGAL_JTAG_ID)
      fprintf(stderr, "%s: JTAGEN fuse disabled?\n", progname);
    return -1;
  }

  prog_enabled = 1;
  return 0;
}

static int jtagmkII_program_disable(PROGRAMMER * pgm)
{
  int status;
  unsigned char buf[1], *resp, c;

  if (!prog_enabled)
    return 0;

  buf[0] = CMND_LEAVE_PROGMODE;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_program_disable(): "
	    "Sending leave progmode command: ",
	    progname);
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_program_disable(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_program_disable(): "
	    "bad response to leave progmode command: 0x%02x\n",
	    progname, c);
    return -1;
  }

  prog_enabled = 0;
  (void)jtagmkII_reset(pgm);

  return 0;
}

static unsigned char jtagmkII_get_baud(long baud)
{
  static struct {
    long baud;
    unsigned char val;
  } baudtab[] = {
    { 2400L, PAR_BAUD_2400 },
    { 4800L, PAR_BAUD_4800 },
    { 9600L, PAR_BAUD_9600 },
    { 19200L, PAR_BAUD_19200 },
    { 38400L, PAR_BAUD_38400 },
    { 57600L, PAR_BAUD_57600 },
    { 115200L, PAR_BAUD_115200 },
    { 14400L, PAR_BAUD_14400 },
  };
  int i;

  for (i = 0; i < sizeof baudtab / sizeof baudtab[0]; i++)
    if (baud == baudtab[i].baud)
      return baudtab[i].val;

  return 0;
}

/*
 * initialize the AVR device and prepare it to accept commands
 */
static int jtagmkII_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  AVRMEM hfuse;
  unsigned char b;

  if (!(p->flags & AVRPART_HAS_JTAG)) {
    fprintf(stderr, "%s: jtagmkII_initialize(): part %s has no JTAG interface\n",
	    progname, p->desc);
    return -1;
  }

  if (pgm->baudrate && pgm->baudrate != 19200) {
    if ((b = jtagmkII_get_baud(pgm->baudrate)) == 0) {
      fprintf(stderr, "%s: jtagmkII_initialize(): unsupported baudrate %d\n",
	      progname, pgm->baudrate);
    } else {
      if (verbose >= 2)
	fprintf(stderr, "%s: jtagmkII_initialize(): "
		"trying to set baudrate to %d\n",
		progname, pgm->baudrate);
      if (jtagmkII_setparm(pgm, PAR_BAUD_RATE, &b) == 0)
	serial_setspeed(pgm->fd, pgm->baudrate);
    }
  }
  if (pgm->bitclock != 0.0) {
    if (verbose >= 2)
      fprintf(stderr, "%s: jtagmkII_initialize(): "
	      "trying to set JTAG clock period to %.1f us\n",
	      progname, pgm->bitclock);
    if (jtagmkII_set_sck_period(pgm, pgm->bitclock) != 0)
      return -1;
  }

  /*
   * Must set the device descriptor before entering programming mode.
   */
  jtagmkII_set_devdescr(pgm, p);

  free(flash_pagecache);
  free(eeprom_pagecache);
  if ((flash_pagecache = malloc(flash_pagesize)) == NULL) {
    fprintf(stderr, "%s: jtagmkII_initialize(): Out of memory\n",
	    progname);
    return -1;
  }
  if ((eeprom_pagecache = malloc(eeprom_pagesize)) == NULL) {
    fprintf(stderr, "%s: jtagmkII_initialize(): Out of memory\n",
	    progname);
    free(flash_pagecache);
    return -1;
  }
  flash_pageaddr = eeprom_pageaddr = (unsigned long)-1L;

  if (jtagmkII_reset(pgm) < 0)
    return -1;

  strcpy(hfuse.desc, "hfuse");
  if (jtagmkII_read_byte(pgm, p, &hfuse, 1, &b) < 0)
    return -1;
  if ((b & OCDEN) != 0)
    fprintf(stderr,
	    "%s: jtagmkII_initialize(): warning: OCDEN fuse not programmed, "
	    "single-byte EEPROM updates not possible\n",
	    progname);

  return 0;
}


static void jtagmkII_disable(PROGRAMMER * pgm)
{

  free(flash_pagecache);
  flash_pagecache = NULL;
  free(eeprom_pagecache);
  eeprom_pagecache = NULL;

  (void)jtagmkII_program_disable(pgm);
}

static void jtagmkII_enable(PROGRAMMER * pgm)
{
  return;
}


static int jtagmkII_open(PROGRAMMER * pgm, char * port)
{
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_open()\n", progname);

#if defined(HAVE_LIBUSB)
  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.
   */
  if (strncmp(port, "usb", 3) == 0)
    serdev = &usb_serdev;
#endif

  strcpy(pgm->port, port);
  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pgm->fd = serial_open(port, 19200);

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  jtagmkII_getsync(pgm);

  return 0;
}


static void jtagmkII_close(PROGRAMMER * pgm)
{
  int status;
  unsigned char buf[1], *resp, c;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_close()\n", progname);

  buf[0] = CMND_GO;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_close(): Sending GO command: ",
	    progname);
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_close(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
  } else {
    if (verbose >= 3) {
      putc('\n', stderr);
      jtagmkII_prmsg(pgm, resp, status);
    } else if (verbose == 2)
      fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
    c = resp[0];
    free(resp);
    if (c != RSP_OK) {
      fprintf(stderr,
	      "%s: jtagmkII_close(): "
	      "bad response to GO command: 0x%02x\n",
	      progname, c);
    }
  }

  buf[0] = CMND_SIGN_OFF;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_close(): Sending sign-off command: ",
	    progname);
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_close(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_close(): "
	    "bad response to sign-off command: 0x%02x\n",
	    progname, c);
  }

  serial_close(pgm->fd);
  pgm->fd = -1;
}


static int jtagmkII_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
				int page_size, int n_bytes)
{
  int addr, block_size;
  unsigned char *cmd;
  unsigned char *resp;
  int status, tries;
  long otimeout = serial_recv_timeout;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_paged_write(.., %s, %d, %d)\n",
	    progname, m->desc, page_size, n_bytes);

  if (jtagmkII_program_enable(pgm) < 0)
    return -1;

  if ((cmd = malloc(page_size + 10)) == NULL) {
    fprintf(stderr, "%s: jtagmkII_paged_write(): Out of memory\n",
	    progname);
    return -1;
  }

  cmd[0] = CMND_WRITE_MEMORY;
  if (strcmp(m->desc, "flash") == 0) {
    cmd[1] = MTYPE_FLASH_PAGE;
    flash_pageaddr = (unsigned long)-1L;
    page_size = flash_pagesize;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    cmd[1] = MTYPE_EEPROM_PAGE;
    eeprom_pageaddr = (unsigned long)-1L;
    page_size = eeprom_pagesize;
  }

  if (page_size == 0) page_size = 256;

  serial_recv_timeout = 100;
  for (addr = 0; addr < n_bytes; addr += page_size) {
    report_progress(addr, n_bytes,NULL);

    if ((n_bytes-addr) < page_size)
      block_size = n_bytes - addr;
    else
      block_size = page_size;
    if (verbose >= 3)
      fprintf(stderr, "%s: jtagmkII_paged_write(): "
	      "block_size at addr %d is %d\n",
	      progname, addr, block_size);

    u32_to_b4(cmd + 2, page_size);
    u32_to_b4(cmd + 6, addr);

    /*
     * The JTAG ICE will refuse to write anything but a full page, at
     * least for the flash ROM.  If a partial page has been requested,
     * set the remainder to 0xff.  (Maybe we should rather read back
     * the existing contents instead before?  Doesn't matter much, as
     * bits cannot be written to 1 anyway.)
     */
    memset(cmd + 10, 0xff, page_size);
    memcpy(cmd + 10, m->buf + addr, block_size);

    tries = 0;

    retry:
    if (verbose >= 2)
      fprintf(stderr, "%s: jtagmkII_paged_write(): "
	      "Sending write memory command: ",
	      progname);
    jtagmkII_send(pgm, cmd, page_size + 10);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
      if (verbose >= 2)
	putc('\n', stderr);
      if (verbose >= 1)
	fprintf(stderr,
		"%s: jtagmkII_paged_write(): "
		"timeout/error communicating with programmer (status %d)\n",
		progname, status);
      if (tries++ < 4) {
	serial_recv_timeout *= 2;
	goto retry;
      }
      fprintf(stderr,
	      "%s: jtagmkII_paged_write(): fatal timeout/"
	      "error communicating with programmer (status %d)\n",
	      progname, status);
      free(cmd);
      serial_recv_timeout = otimeout;
      return -1;
    }
    if (verbose >= 3) {
      putc('\n', stderr);
      jtagmkII_prmsg(pgm, resp, status);
    } else if (verbose == 2)
      fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
    if (resp[0] != RSP_OK) {
      fprintf(stderr,
	      "%s: jtagmkII_paged_write(): "
	      "bad response to write memory command: 0x%02x\n",
	      progname, resp[0]);
      free(resp);
      free(cmd);
      serial_recv_timeout = otimeout;
      return -1;
    }
    free(resp);
  }

  free(cmd);
  serial_recv_timeout = otimeout;

  return n_bytes;
}

static int jtagmkII_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
			       int page_size, int n_bytes)
{
  int addr, block_size;
  unsigned char cmd[10];
  unsigned char *resp;
  int status, tries;
  long otimeout = serial_recv_timeout;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_paged_load(.., %s, %d, %d)\n",
	    progname, m->desc, page_size, n_bytes);

  if (jtagmkII_program_enable(pgm) < 0)
    return -1;

  page_size = m->readsize;

  cmd[0] = CMND_READ_MEMORY;
  if (strcmp(m->desc, "flash") == 0) {
    cmd[1] = MTYPE_FLASH_PAGE;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    cmd[1] = MTYPE_EEPROM_PAGE;
  }

  serial_recv_timeout = 100;
  for (addr = 0; addr < n_bytes; addr += page_size) {
    report_progress(addr, n_bytes,NULL);

    if ((n_bytes-addr) < page_size)
      block_size = n_bytes - addr;
    else
      block_size = page_size;
    if (verbose >= 3)
      fprintf(stderr, "%s: jtagmkII_paged_load(): "
	      "block_size at addr %d is %d\n",
	      progname, addr, block_size);

    u32_to_b4(cmd + 2, block_size);
    u32_to_b4(cmd + 6, addr);

    tries = 0;

    retry:
    if (verbose >= 2)
      fprintf(stderr, "%s: jtagmkII_paged_load(): Sending read memory command: ",
	      progname);
    jtagmkII_send(pgm, cmd, 10);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
      if (verbose >= 2)
	putc('\n', stderr);
      if (verbose >= 1)
	fprintf(stderr,
		"%s: jtagmkII_paged_load(): "
		"timeout/error communicating with programmer (status %d)\n",
		progname, status);
      if (tries++ < 4) {
	serial_recv_timeout *= 2;
	goto retry;
      }
      fprintf(stderr,
	      "%s: jtagmkII_paged_load(): fatal timeout/"
	      "error communicating with programmer (status %d)\n",
	      progname, status);
      serial_recv_timeout = otimeout;
      return -1;
    }
    if (verbose >= 3) {
      putc('\n', stderr);
      jtagmkII_prmsg(pgm, resp, status);
    } else if (verbose == 2)
      fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
    if (resp[0] != RSP_MEMORY) {
      fprintf(stderr,
	      "%s: jtagmkII_paged_load(): "
	      "bad response to read memory command: 0x%02x\n",
	      progname, resp[0]);
      free(resp);
      serial_recv_timeout = otimeout;
      return -1;
    }
    memcpy(m->buf + addr, resp + 1, status);
    free(resp);
  }
  serial_recv_timeout = otimeout;

  return n_bytes;
}

static int jtagmkII_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			      unsigned long addr, unsigned char * value)
{
  unsigned char cmd[10];
  unsigned char *resp = NULL, *cache_ptr = NULL;
  int status, tries;
  unsigned long paddr = 0UL, *paddr_ptr = NULL;
  unsigned int pagesize = 0;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_read_byte(.., %s, 0x%lx, ...)\n",
	    progname, mem->desc, addr);

  if (jtagmkII_program_enable(pgm) < 0)
    return -1;

  cmd[0] = CMND_READ_MEMORY;

  if (strcmp(mem->desc, "flash") == 0) {
    cmd[1] = MTYPE_FLASH_PAGE;
    pagesize = mem->page_size;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &flash_pageaddr;
    cache_ptr = flash_pagecache;
  } else if (strcmp(mem->desc, "eeprom") == 0) {
    cmd[1] = MTYPE_EEPROM_PAGE;
    pagesize = mem->page_size;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &eeprom_pageaddr;
    cache_ptr = eeprom_pagecache;
  } else if (strcmp(mem->desc, "lfuse") == 0) {
    cmd[1] = MTYPE_FUSE_BITS;
    addr = 0;
  } else if (strcmp(mem->desc, "hfuse") == 0) {
    cmd[1] = MTYPE_FUSE_BITS;
    addr = 1;
  } else if (strcmp(mem->desc, "efuse") == 0) {
    cmd[1] = MTYPE_FUSE_BITS;
    addr = 2;
  } else if (strcmp(mem->desc, "lock") == 0) {
    cmd[1] = MTYPE_LOCK_BITS;
  } else if (strcmp(mem->desc, "calibration") == 0) {
    cmd[1] = MTYPE_OSCCAL_BYTE;
  } else if (strcmp(mem->desc, "signature") == 0) {
    cmd[1] = MTYPE_SIGN_JTAG;
  }

  /*
   * To improve the read speed, we used paged reads for flash and
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

  if (pagesize) {
    u32_to_b4(cmd + 2, pagesize);
    u32_to_b4(cmd + 6, paddr);
  } else {
    u32_to_b4(cmd + 2, 1);
    u32_to_b4(cmd + 6, addr);
  }

  tries = 0;
  retry:
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_read_byte(): Sending read memory command: ",
	    progname);
  jtagmkII_send(pgm, cmd, 10);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    if (verbose >= 1)
      fprintf(stderr,
	      "%s: jtagmkII_read_byte(): "
	      "timeout/error communicating with programmer (status %d)\n",
	      progname, status);
    if (tries++ < 3)
      goto retry;
    fprintf(stderr,
	    "%s: jtagmkII_read_byte(): "
	    "fatal timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    goto fail;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  if (resp[0] != RSP_MEMORY) {
    fprintf(stderr,
	    "%s: jtagmkII_read_byte(): "
	    "bad response to read memory command: 0x%02x\n",
	    progname, resp[0]);
    goto fail;
  }

  if (pagesize) {
    *paddr_ptr = paddr;
    memcpy(cache_ptr, resp + 1, pagesize);
    *value = cache_ptr[addr & (pagesize - 1)];
  } else
    *value = resp[1];

  free(resp);
  return 0;

fail:
  /*
   * XXX should return an error status here, but that would cause
   * the generic methods to retry the request using the SPI method,
   * which is complete nonsense for JTAG.
   */
  *value = 42;
  free(resp);
  return 0;
}

static int jtagmkII_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			       unsigned long addr, unsigned char data)
{
  unsigned char cmd[11];
  unsigned char *resp = NULL, writedata;
  int status, tries, need_progmode = 1;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_write_byte(.., %s, 0x%lx, ...)\n",
	    progname, mem->desc, addr);

  writedata = data;
  cmd[0] = CMND_WRITE_MEMORY;
  if (strcmp(mem->desc, "flash") == 0) {
    cmd[1] = MTYPE_SPM;
    need_progmode = 0;
    flash_pageaddr = (unsigned long)-1L;
  } else if (strcmp(mem->desc, "eeprom") == 0) {
    cmd[1] = MTYPE_EEPROM;
    need_progmode = 0;
    eeprom_pageaddr = (unsigned long)-1L;
  } else if (strcmp(mem->desc, "lfuse") == 0) {
    cmd[1] = MTYPE_FUSE_BITS;
    addr = 0;
  } else if (strcmp(mem->desc, "hfuse") == 0) {
    cmd[1] = MTYPE_FUSE_BITS;
    addr = 1;
  } else if (strcmp(mem->desc, "efuse") == 0) {
    cmd[1] = MTYPE_FUSE_BITS;
    addr = 2;
  } else if (strcmp(mem->desc, "lock") == 0) {
    cmd[1] = MTYPE_LOCK_BITS;
  } else if (strcmp(mem->desc, "calibration") == 0) {
    cmd[1] = MTYPE_OSCCAL_BYTE;
  } else if (strcmp(mem->desc, "signature") == 0) {
    cmd[1] = MTYPE_SIGN_JTAG;
  }

  if (need_progmode) {
    if (jtagmkII_program_enable(pgm) < 0)
      return -1;
  } else {
    if (jtagmkII_program_disable(pgm) < 0)
      return -1;
  }

  u32_to_b4(cmd + 2, 1);
  u32_to_b4(cmd + 6, addr);
  cmd[10] = writedata;

  tries = 0;
  retry:
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_write_byte(): Sending write memory command: ",
	    progname);
  jtagmkII_send(pgm, cmd, 11);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    if (verbose > 1)
      fprintf(stderr,
	      "%s: jtagmkII_write_byte(): "
	      "timeout/error communicating with programmer (status %d)\n",
	      progname, status);
    if (tries++ < 3)
      goto retry;
    fprintf(stderr,
	    "%s: jtagmkII_write_byte(): "
	    "fatal timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    goto fail;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  if (resp[0] != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_write_byte(): "
	    "bad response to write memory command: 0x%02x\n",
	    progname, resp[0]);
    goto fail;
  }

  free(resp);
  return 0;

fail:
  /*
   * XXX should return an error status here, but that would cause
   * the generic methods to retry the request using the SPI method,
   * which is complete nonsense for JTAG.
   */
  free(resp);
  return 0;
}


/*
 * Set the JTAG clock.  The actual frequency is quite a bit of
 * guesswork, based on the values claimed by AVR Studio.  Inside the
 * JTAG ICE, the value is the delay count of a delay loop between the
 * JTAG clock edges.  A count of 0 bypasses the delay loop.
 *
 * As the STK500 expresses it as a period length (and we actualy do
 * program a period length as well), we rather call it by that name.
 */
static int jtagmkII_set_sck_period(PROGRAMMER * pgm, double v)
{
  unsigned char dur;

  v = 1 / v;			/* convert to frequency */
  if (v >= 6.4e6)
    dur = 0;
  else if (v >= 2.8e6)
    dur = 1;
  else if (v >= 20.9e3)
    dur = (unsigned char)(5.35e6 / v);
  else
    dur = 255;

  return jtagmkII_setparm(pgm, PAR_OCD_JTAG_CLK, &dur);
}


/*
 * Read an emulator parameter.  As the maximal parameter length is 4
 * bytes by now, we always copy out 4 bytes to *value, so the caller
 * must have allocated sufficient space.
 */
static int jtagmkII_getparm(PROGRAMMER * pgm, unsigned char parm,
			    unsigned char * value)
{
  int status;
  unsigned char buf[2], *resp, c;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_getparm()\n", progname);

  buf[0] = CMND_GET_PARAMETER;
  buf[1] = parm;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_getparm(): "
	    "Sending get parameter command (parm 0x%02x): ",
	    progname, parm);
  jtagmkII_send(pgm, buf, 2);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_getparm(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  if (c != RSP_PARAMETER) {
    fprintf(stderr,
	    "%s: jtagmkII_getparm(): "
	    "bad response to get parameter command: 0x%02x\n",
	    progname, c);
    free(resp);
    return -1;
  }

  memcpy(value, resp + 1, 4);
  free(resp);

  return 0;
}

/*
 * Write an emulator parameter.
 */
static int jtagmkII_setparm(PROGRAMMER * pgm, unsigned char parm,
			    unsigned char * value)
{
  int status;
  /*
   * As the maximal parameter length is 4 bytes, we use a fixed-length
   * buffer, as opposed to malloc()ing it.
   */
  unsigned char buf[2 + 4], *resp, c;
  size_t size;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_setparm()\n", progname);

  switch (parm) {
  case PAR_HW_VERSION: size = 2; break;
  case PAR_FW_VERSION: size = 4; break;
  case PAR_EMULATOR_MODE: size = 1; break;
  case PAR_BAUD_RATE: size = 1; break;
  case PAR_OCD_VTARGET: size = 2; break;
  case PAR_OCD_JTAG_CLK: size = 1; break;
  default:
    fprintf(stderr, "%s: jtagmkII_setparm(): unknown parameter 0x%02x\n",
	    progname, parm);
    return -1;
  }

  buf[0] = CMND_SET_PARAMETER;
  buf[1] = parm;
  memcpy(buf + 2, value, size);
  if (verbose >= 2)
    fprintf(stderr, "%s: jtagmkII_setparm(): "
	    "Sending set parameter command (parm 0x%02x, %zu bytes): ",
	    progname, parm, size);
  jtagmkII_send(pgm, buf, size + 2);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtagmkII_setparm(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtagmkII_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtagmkII_setparm(): "
	    "bad response to set parameter command: 0x%02x\n",
	    progname, c);
    return -1;
  }

  return 0;
}


static void jtagmkII_display(PROGRAMMER * pgm, char * p)
{
  unsigned char hw[4], fw[4];

  if (jtagmkII_getparm(pgm, PAR_HW_VERSION, hw) < 0 ||
      jtagmkII_getparm(pgm, PAR_FW_VERSION, fw) < 0)
    return;

  fprintf(stderr, "%sM_MCU hardware version: %d\n", p, hw[0]);
  fprintf(stderr, "%sM_MCU firmware version: %d.%02d\n", p, fw[1], fw[0]);
  fprintf(stderr, "%sS_MCU hardware version: %d\n", p, hw[1]);
  fprintf(stderr, "%sS_MCU firmware version: %d.%02d\n", p, fw[3], fw[2]);
  fprintf(stderr, "%sSerial number:          %02x:%02x:%02x:%02x:%02x:%02x\n",
	  p, serno[0], serno[1], serno[2], serno[3], serno[4], serno[5]);

  jtagmkII_print_parms1(pgm, p);

  return;
}


static void jtagmkII_print_parms1(PROGRAMMER * pgm, char * p)
{
  unsigned char vtarget[4], jtag_clock[4];
  char clkbuf[20];
  double clk;

  if (jtagmkII_getparm(pgm, PAR_OCD_VTARGET, vtarget) < 0 ||
      jtagmkII_getparm(pgm, PAR_OCD_JTAG_CLK, jtag_clock) < 0)
    return;

  if (jtag_clock[0] == 0) {
    strcpy(clkbuf, "6.4 MHz");
    clk = 6.4e6;
  } else if (jtag_clock[0] == 1) {
    strcpy(clkbuf, "2.8 MHz");
    clk = 2.8e6;
  } else if (jtag_clock[0] <= 5) {
    sprintf(clkbuf, "%.1f MHz", 5.35 / (double)jtag_clock[0]);
    clk = 5.35e6 / (double)jtag_clock[0];
  } else {
    sprintf(clkbuf, "%.1f kHz", 5.35e3 / (double)jtag_clock[0]);
    clk = 5.35e6 / (double)jtag_clock[0];
  }

  fprintf(stderr, "%sVtarget         : %.1f V\n", p,
	  b2_to_u16(vtarget) / 1000.0);
  fprintf(stderr, "%sJTAG clock      : %s (%.1f us)\n", p, clkbuf,
	  1.0e6 / clk);

  return;
}


static void jtagmkII_print_parms(PROGRAMMER * pgm)
{
  jtagmkII_print_parms1(pgm, "");
}


void jtagmkII_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "JTAGMKII");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_dummy;
  pgm->chip_erase     = jtagmkII_chip_erase;
  pgm->cmd            = jtagmkII_cmd;
  pgm->open           = jtagmkII_open;
  pgm->close          = jtagmkII_close;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->set_sck_period = jtagmkII_set_sck_period;
  pgm->page_size      = 256;
}
