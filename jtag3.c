/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2012 Joerg Wunsch <j@uriah.heep.sax.de>
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* $Id$ */

/*
 * avrdude interface for Atmel JTAGICE3 programmer
 */

#include "ac_cfg.h"

#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "avrdude.h"
#include "avr.h"
#include "crc16.h"
#include "pgm.h"
#include "jtag3.h"
#include "jtag3_private.h"
#include "serial.h"
#include "usbdevs.h"

/*
 * Private data for this programmer.
 */
struct pdata
{
  unsigned short command_sequence; /* Next cmd seqno to issue. */

  /*
   * See jtag3_read_byte() for an explanation of the flash and
   * EEPROM page caches.
   */
  unsigned char *flash_pagecache;
  unsigned long flash_pageaddr;
  unsigned int flash_pagesize;

  unsigned char *eeprom_pagecache;
  unsigned long eeprom_pageaddr;
  unsigned int eeprom_pagesize;

  int prog_enabled;	     /* Cached value of PROGRAMMING status. */

  /* JTAG chain stuff */
  unsigned char jtagchain[4];

  /* Start address of Xmega boot area */
  unsigned long boot_start;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

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

/*
 * pgm->flag is marked as "for private use of the programmer".
 * The following defines this programmer's use of that field.
 */
#define PGM_FL_IS_DW		(0x0001)
#define PGM_FL_IS_PDI           (0x0002)
#define PGM_FL_IS_JTAG          (0x0004)

static int jtag3_open(PROGRAMMER * pgm, char * port);

static int jtag3_command(PROGRAMMER *pgm, unsigned char *cmd, unsigned int cmdlen,
			 unsigned char **resp, const char *descr);

static int jtag3_initialize(PROGRAMMER * pgm, AVRPART * p);
static int jtag3_chip_erase(PROGRAMMER * pgm, AVRPART * p);
static int jtag3_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                                unsigned long addr, unsigned char * value);
static int jtag3_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                                unsigned long addr, unsigned char data);
static int jtag3_reset(PROGRAMMER * pgm, unsigned char flags);
static int jtag3_set_sck_period(PROGRAMMER * pgm, double v);
static int jtag3_setparm(PROGRAMMER * pgm, unsigned char scope,
			 unsigned char section, unsigned char parm,
			 unsigned char *value, unsigned char length);
static void jtag3_print_parms1(PROGRAMMER * pgm, const char * p);
static int jtag3_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes);
static unsigned char jtag3_memtype(PROGRAMMER * pgm, AVRPART * p, unsigned long addr);
static unsigned int jtag3_memaddr(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, unsigned long addr);


void jtag3_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
    fprintf(stderr,
	    "%s: jtag3_setup(): Out of memory allocating private data\n",
	    progname);
    exit(1);
  }
  memset(pgm->cookie, 0, sizeof(struct pdata));
}

void jtag3_teardown(PROGRAMMER * pgm)
{
  free(pgm->cookie);
}


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

static void jtag3_print_data(unsigned char *b, size_t s)
{
  int i;

  if (s < 2)
    return;

  for (i = 0; i < s; i++) {
    fprintf(stderr, "0x%02x", b[i]);
    if (i % 16 == 15)
      putc('\n', stderr);
    else
      putc(' ', stderr);
  }
  if (i % 16 != 0)
    putc('\n', stderr);
}

static void jtag3_prmsg(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  int i;

  if (verbose >= 4) {
    fprintf(stderr, "Raw message:\n");

    for (i = 0; i < len; i++) {
      fprintf(stderr, "%02x ", data[i]);
      if (i % 16 == 15)
	putc('\n', stderr);
      else
	putchar(' ');
    }
    if (i % 16 != 0)
      putc('\n', stderr);
  }

  switch (data[0]) {
    case SCOPE_INFO:
      fprintf(stderr, "[info] ");
      break;

    case SCOPE_GENERAL:
      fprintf(stderr, "[general] ");
      break;

    case SCOPE_AVR_ISP:
      fprintf(stderr, "[AVRISP] ");
      jtag3_print_data(data + 1, len - 1);
      return;

    case SCOPE_AVR:
      fprintf(stderr, "[AVR] ");
      break;

    default:
      fprintf(stderr, "[scope 0x%02x] ", data[0]);
      break;
  }

  switch (data[1]) {
    case RSP3_OK:
      fprintf(stderr, "OK\n");
      break;

    case RSP3_FAILED:
      fprintf(stderr, "FAILED");
      if (len > 3)
      {
	char reason[50];
	sprintf(reason, "0x%02x", data[3]);
	switch (data[3])
	{
	  case RSP3_FAIL_NO_ANSWER:
	    strcpy(reason, "target does not answer");
	    break;

	  case RSP3_FAIL_NO_TARGET_POWER:
	    strcpy(reason, "no target power");
	    break;

	  case RSP3_FAIL_NOT_UNDERSTOOD:
	    strcpy(reason, "command not understood");
	    break;

	  case RSP3_FAIL_WRONG_MODE:
	    strcpy(reason, "wrong (programming) mode");
	    break;

	  case RSP3_FAIL_PDI:
	    strcpy(reason, "PDI failure");
	    break;

	  case RSP3_FAIL_UNSUPP_MEMORY:
	    strcpy(reason, "unsupported memory type");
	    break;

	  case RSP3_FAIL_WRONG_LENGTH:
	    strcpy(reason, "wrong length in memory access");
	    break;

	  case RSP3_FAIL_DEBUGWIRE:
	    strcpy(reason, "debugWIRE communication failed");
	    break;
	}
	fprintf(stderr, ", reason: %s\n", reason);
      }
      else
      {
	fprintf(stderr, ", unspecified reason\n");
      }
      break;

    case RSP3_DATA:
      fprintf(stderr, "Data returned:\n");
      jtag3_print_data(data + 2, len - 2);
      break;

    case RSP3_INFO:
      fprintf(stderr, "Info returned:\n");
      for (i = 2; i < len; i++) {
	if (isprint(data[i]))
	  putc(data[i], stderr);
	else
	  fprintf(stderr, "\\%03o", data[i]);
      }
      putc('\n', stderr);
      break;

    case RSP3_PC:
      if (len < 7)
      {
	fprintf(stderr, "PC reply too short\n");
      }
      else
      {
	unsigned long pc = (data[6] << 24) | (data[5] << 16)
	  | (data[4] << 8) | data[3];
	fprintf(stderr, "PC 0x%0lx\n", pc);
      }
      break;

  default:
    fprintf(stderr, "unknown message 0x%02x\n", data[1]);
  }
}

static void jtag3_prevent(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  int i;

  if (verbose >= 4) {
    fprintf(stderr, "Raw event:\n");

    for (i = 0; i < len; i++) {
      fprintf(stderr, "%02x ", data[i]);
      if (i % 16 == 15)
	putc('\n', stderr);
      else
	putchar(' ');
    }
    if (i % 16 != 0)
      putc('\n', stderr);
  }

  fprintf(stderr, "Event serial 0x%04x, ",
	  (data[3] << 8) | data[2]);

  switch (data[4]) {
    case SCOPE_INFO:
      fprintf(stderr, "[info] ");
      break;

    case SCOPE_GENERAL:
      fprintf(stderr, "[general] ");
      break;

    case SCOPE_AVR:
      fprintf(stderr, "[AVR] ");
      break;

    default:
      fprintf(stderr, "[scope 0x%02x] ", data[0]);
      break;
  }

  switch (data[5]) {
  case EVT3_BREAK:
    fprintf(stderr, "BREAK");
    if (len >= 11) {
      fprintf(stderr, ", PC = 0x%lx, reason ", b4_to_u32(data + 6));
      switch (data[10]) {
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
	fprintf(stderr, "unknown: 0x%02x", data[10]);
      }
      /* There are two more bytes of data which always appear to be
       * 0x01, 0x00.  Purpose unknown. */
    }
    break;

  case EVT3_SLEEP:
    if (len >= 8 && data[7] == 0)
      fprintf(stderr, "sleeping");
    else if (len >= 8 && data[7] == 1)
      fprintf(stderr, "wakeup");
    else
      fprintf(stderr, "unknown SLEEP event");
    break;

  case EVT3_POWER:
    if (len >= 8 && data[7] == 0)
      fprintf(stderr, "power-down");
    else if (len >= 8 && data[7] == 1)
      fprintf(stderr, "power-up");
    else
      fprintf(stderr, "unknown POWER event");
    break;

  default:
    fprintf(stderr, "UNKNOWN 0x%02x", data[5]);
    break;
  }
  putc('\n', stderr);
}



int jtag3_send(PROGRAMMER * pgm, unsigned char * data, size_t len)
{
  unsigned char *buf;

  if (verbose >= 3)
    fprintf(stderr, "\n%s: jtag3_send(): sending %lu bytes\n",
	    progname, (unsigned long)len);

  if ((buf = malloc(len + 4)) == NULL)
    {
      fprintf(stderr, "%s: jtag3_send(): out of memory",
	      progname);
      return -1;
    }

  buf[0] = TOKEN;
  buf[1] = 0;                   /* dummy */
  u16_to_b2(buf + 2, PDATA(pgm)->command_sequence);
  memcpy(buf + 4, data, len);

  if (serial_send(&pgm->fd, buf, len + 4) != 0) {
    fprintf(stderr,
	    "%s: jtag3_send(): failed to send command to serial port\n",
	    progname);
    exit(1);
  }

  free(buf);

  return 0;
}


static int jtag3_drain(PROGRAMMER * pgm, int display)
{
  return serial_drain(&pgm->fd, display);
}


/*
 * Receive one frame, return it in *msg.  Received sequence number is
 * returned in seqno.  Any valid frame will be returned, regardless
 * whether it matches the expected sequence number, including event
 * notification frames (seqno == 0xffff).
 *
 * Caller must eventually free the buffer.
 */
static int jtag3_recv_frame(PROGRAMMER * pgm, unsigned char **msg) {
  int rv;
  unsigned char *buf = NULL;

  if (verbose >= 4)
    fprintf(stderr, "%s: jtag3_recv():\n", progname);

  if ((buf = malloc(pgm->fd.usb.max_xfer)) == NULL) {
    fprintf(stderr, "%s: jtag3_recv(): out of memory\n",
	    progname);
    return -1;
  }

  rv = serial_recv(&pgm->fd, buf, pgm->fd.usb.max_xfer);

  if (rv < 0) {
    /* timeout in receive */
    if (verbose > 1)
      fprintf(stderr,
	      "%s: jtag3_recv(): Timeout receiving packet\n",
	      progname);
    free(buf);
    return -1;
  }

  *msg = buf;

  return rv;
}

int jtag3_recv(PROGRAMMER * pgm, unsigned char **msg) {
  unsigned short r_seqno;
  int rv;

  for (;;) {
    if ((rv = jtag3_recv_frame(pgm, msg)) <= 0)
      return rv;

    if ((rv & USB_RECV_FLAG_EVENT) != 0) {
      if (verbose >= 3)
	jtag3_prevent(pgm, *msg, rv & USB_RECV_LENGTH_MASK);

      free(*msg);
      continue;
    }

    rv &= USB_RECV_LENGTH_MASK;
    r_seqno = ((*msg)[2] << 8) | (*msg)[1];
    if (verbose >= 3)
      fprintf(stderr, "%s: jtag3_recv(): "
	      "Got message seqno %d (command_sequence == %d)\n",
	      progname, r_seqno, PDATA(pgm)->command_sequence);
    if (r_seqno == PDATA(pgm)->command_sequence) {
      if (++(PDATA(pgm)->command_sequence) == 0xffff)
	PDATA(pgm)->command_sequence = 0;
      /*
       * We move the payload to the beginning of the buffer, to make
       * the job easier for the caller.  We have to return the
       * original pointer though, as the caller must free() it.
       */
      memmove(*msg, *msg + 3, rv);
      rv -= 3;

      return rv;
    }
    if (verbose >= 2)
      fprintf(stderr, "%s: jtag3_recv(): "
	      "got wrong sequence number, %u != %u\n",
	      progname, r_seqno, PDATA(pgm)->command_sequence);

    free(*msg);
  }
}

static int jtag3_command(PROGRAMMER *pgm, unsigned char *cmd, unsigned int cmdlen,
			 unsigned char **resp, const char *descr)
{
  int status;
  unsigned char c;

  if (verbose >= 2)
    fprintf(stderr, "%s: Sending %s command: ",
	    progname, descr);
  jtag3_send(pgm, cmd, cmdlen);

  status = jtag3_recv(pgm, resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: %s command: timeout/error communicating with programmer (status %d)\n",
	    progname, descr, status);
    return -1;
  } else if (verbose >= 3) {
    putc('\n', stderr);
    jtag3_prmsg(pgm, *resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", (*resp)[1], status);

  c = (*resp)[1];
  if ((c & RSP3_STATUS_MASK) != RSP3_OK) {
    fprintf(stderr,
	    "%s: bad response to %s command: 0x%02x\n",
	    progname, descr, c);
    free(*resp);
    resp = 0;
    return -1;
  }

  return status;
}


int jtag3_getsync(PROGRAMMER * pgm, int mode) {

  unsigned char buf[3], *resp;

  if (verbose >= 3)
    fprintf(stderr, "%s: jtag3_getsync()\n", progname);

  /* Get the sign-on information. */
  buf[0] = SCOPE_GENERAL;
  buf[1] = CMD3_SIGN_ON;
  buf[2] = 0;

  if (jtag3_command(pgm, buf, 3, &resp, "sign-on") < 0)
    return -1;

  free(resp);

  return 0;
}

/*
 * issue the 'chip erase' command to the AVR device
 */
static int jtag3_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[8], *resp;

  buf[0] = SCOPE_AVR;
  buf[1] = CMD3_ERASE_MEMORY;
  buf[2] = 0;
  buf[3] = XMEGA_ERASE_CHIP;
  buf[4] = buf[5] = buf[6] = buf[7] = 0; /* page address */

  if (jtag3_command(pgm, buf, 8, &resp, "chip erase") < 0)
    return -1;

  free(resp);
  return 0;
}

/*
 * There is no chip erase functionality in debugWire mode.
 */
static int jtag3_chip_erase_dw(PROGRAMMER * pgm, AVRPART * p)
{

  fprintf(stderr, "%s: Chip erase not supported in debugWire mode\n",
	  progname);

  return 0;
}

/*
 * Reset the target.
 */
static int jtag3_reset(PROGRAMMER * pgm, unsigned char flags)
{
#if 0
  int status;
  unsigned char buf[2], *resp, c;

  /*
   * In debugWire mode, don't reset.  Do a forced stop, and tell the
   * ICE to stop any timers, too.
   */
  if (pgm->flag & PGM_FL_IS_DW) {
    unsigned char parm[] = { 0 };

    (void)jtag3_setparm(pgm, PAR_TIMERS_RUNNING, parm);
  }

  buf[0] = (pgm->flag & PGM_FL_IS_DW)? CMND_FORCED_STOP: CMND_RESET;
  buf[1] = (pgm->flag & PGM_FL_IS_DW)? 1: flags;
  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_reset(): Sending %s command: ",
	    progname, (pgm->flag & PGM_FL_IS_DW)? "stop": "reset");
  jtag3_send(pgm, buf, 2);

  status = jtag3_recv(pgm, &resp);
  if (status <= 0) {
    if (verbose >= 2)
      putc('\n', stderr);
    fprintf(stderr,
	    "%s: jtag3_reset(): "
	    "timeout/error communicating with programmer (status %d)\n",
	    progname, status);
    return -1;
  }
  if (verbose >= 3) {
    putc('\n', stderr);
    jtag3_prmsg(pgm, resp, status);
  } else if (verbose == 2)
    fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[1], status);
  c = resp[1];
  free(resp);
  if (c != RSP_OK) {
    fprintf(stderr,
	    "%s: jtag3_reset(): "
	    "bad response to reset command: %s\n",
	    progname, jtag3_get_rc(c));
    return -1;
  }

#endif
  return 0;
}

static int jtag3_program_enable_dummy(PROGRAMMER * pgm, AVRPART * p)
{
  return 0;
}

static int jtag3_program_enable(PROGRAMMER * pgm)
{
  unsigned char buf[3], *resp;
  int use_ext_reset;

  if (PDATA(pgm)->prog_enabled)
    return 0;

  for (use_ext_reset = 0; use_ext_reset <= 1; use_ext_reset++) {
    buf[0] = SCOPE_AVR;
    buf[1] = CMD3_ENTER_PROGMODE;
    buf[2] = 0;

    if (jtag3_command(pgm, buf, 3, &resp, "enter progmode") >= 0) {
      free(resp);
      break;
    }

    /* XXX activate external reset here */
    if (verbose > 0)
      fprintf(stderr,
	      "%s: retrying with external reset applied\n",
	      progname);
  }

  PDATA(pgm)->prog_enabled = 1;

  return 0;
}

static int jtag3_program_disable(PROGRAMMER * pgm)
{
  unsigned char buf[3], *resp;

  if (!PDATA(pgm)->prog_enabled)
    return 0;

  buf[0] = SCOPE_AVR;
  buf[1] = CMD3_LEAVE_PROGMODE;
  buf[2] = 0;

  if (jtag3_command(pgm, buf, 3, &resp, "leave progmode") < 0)
    return -1;

  free(resp);

  PDATA(pgm)->prog_enabled = 0;
  //(void)jtag3_reset(pgm, 0x01);

  return 0;
}

/*
 * initialize the AVR device and prepare it to accept commands
 */
static int jtag3_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  AVRMEM hfuse;
  unsigned char conn = 0, parm[4];
  const char *ifname;
  unsigned char cmd[4], *resp, b;
  int status;

  if (pgm->flag & PGM_FL_IS_DW) {
    ifname = "debugWire";
    if (p->flags & AVRPART_HAS_DW)
      conn = PARM3_CONN_DW;
  } else if (pgm->flag & PGM_FL_IS_PDI) {
    ifname = "PDI";
    if (p->flags & AVRPART_HAS_PDI)
      conn = PARM3_CONN_PDI;
  } else {
    ifname = "JTAG";
    if (p->flags & AVRPART_HAS_JTAG)
      conn = PARM3_CONN_JTAG;
  }

  if (conn == 0) {
    fprintf(stderr, "%s: jtag3_initialize(): part %s has no %s interface\n",
	    progname, p->desc, ifname);
    return -1;
  }

  if (p->flags & AVRPART_HAS_PDI)
    parm[0] = PARM3_ARCH_XMEGA;
  else if (p->flags & AVRPART_HAS_DW)
    parm[0] = PARM3_ARCH_TINY;
  else
    parm[0] = PARM3_ARCH_MEGA;
  if (jtag3_setparm(pgm, SCOPE_AVR, 0, PARM3_ARCH, parm, 1) < 0)
    return -1;

  parm[0] = PARM3_SESS_PROGRAMMING;
  if (jtag3_setparm(pgm, SCOPE_AVR, 0, PARM3_SESS_PURPOSE, parm, 1) < 0)
    return -1;

  parm[0] = conn;
  if (jtag3_setparm(pgm, SCOPE_AVR, 1, PARM3_CONNECTION, parm, 1) < 0)
    return -1;

  if (conn == PARM3_CONN_JTAG && pgm->bitclock != 0.0)
  {
    unsigned int clock = 1E-3 / pgm->bitclock; /* kHz */
    if (verbose >= 2)
      fprintf(stderr, "%s: jtag3_initialize(): "
	      "trying to set JTAG clock to %u kHz\n",
	      progname, clock);
    parm[0] = clock & 0xff;
    parm[1] = (clock >> 8) & 0xff;
    if (jtag3_setparm(pgm, SCOPE_AVR, 1,
		      ((p->flags & AVRPART_HAS_PDI)? PARM3_CLK_XMEGA_JTAG: PARM3_CLK_MEGA_PROG),
		      parm, 2) < 0)
      return -1;
  }
  if (conn == PARM3_CONN_PDI && pgm->bitclock != 0.0)
  {
    unsigned int clock = 1E-3 / pgm->bitclock; /* kHz */
    if (verbose >= 2)
      fprintf(stderr, "%s: jtag3_initialize(): "
	      "trying to set PDI clock to %u kHz\n",
	      progname, clock);
    parm[0] = clock & 0xff;
    parm[1] = (clock >> 8) & 0xff;
    if (jtag3_setparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_PDI, parm, 2) < 0)
      return -1;
  }
  if (conn == PARM3_CONN_JTAG)
  {
    if (verbose >= 2)
      fprintf(stderr, "%s: jtag3_initialize(): "
	      "trying to set JTAG daisy-chain info to %d,%d,%d,%d\n",
	      progname,
	      PDATA(pgm)->jtagchain[0], PDATA(pgm)->jtagchain[1],
	      PDATA(pgm)->jtagchain[2], PDATA(pgm)->jtagchain[3]);
    if (jtag3_setparm(pgm, SCOPE_AVR, 1, PARM3_JTAGCHAIN, PDATA(pgm)->jtagchain, 4) < 0)
      return -1;
  }

  /* set device descriptor data */
  if ((p->flags & AVRPART_HAS_PDI))
  {
    struct xmega_device_desc xd;
    LNODEID ln;
    AVRMEM * m;

    u16_to_b2(xd.nvm_base_addr, p->nvm_base);
    u16_to_b2(xd.mcu_base_addr, p->mcu_base);

    for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
      m = ldata(ln);
      if (strcmp(m->desc, "flash") == 0) {
        PDATA(pgm)->flash_pagesize = m->page_size;
	u16_to_b2(xd.flash_page_size, m->page_size);
      } else if (strcmp(m->desc, "eeprom") == 0) {
	xd.eeprom_page_size = m->page_size;
	u16_to_b2(xd.eeprom_size, m->size);
	u32_to_b4(xd.nvm_eeprom_offset, m->offset);
      } else if (strcmp(m->desc, "application") == 0) {
	u32_to_b4(xd.app_size, m->size);
	u32_to_b4(xd.nvm_app_offset, m->offset);
      } else if (strcmp(m->desc, "boot") == 0) {
	u16_to_b2(xd.boot_size, m->size);
	u32_to_b4(xd.nvm_boot_offset, m->offset);
      } else if (strcmp(m->desc, "fuse1") == 0) {
	u32_to_b4(xd.nvm_fuse_offset, m->offset & ~7);
      } else if (strcmp(m->desc, "lock") == 0) {
	u32_to_b4(xd.nvm_lock_offset, m->offset);
      } else if (strcmp(m->desc, "usersig") == 0) {
	u32_to_b4(xd.nvm_user_sig_offset, m->offset);
      } else if (strcmp(m->desc, "prodsig") == 0) {
	u32_to_b4(xd.nvm_prod_sig_offset, m->offset);
      } else if (strcmp(m->desc, "data") == 0) {
	u32_to_b4(xd.nvm_data_offset, m->offset);
      }
    }

    if (jtag3_setparm(pgm, SCOPE_AVR, 2, PARM3_DEVICEDESC, (unsigned char *)&xd, sizeof xd) < 0)
      return -1;
  }
  else
  {
    struct mega_device_desc md;
    LNODEID ln;
    AVRMEM * m;

    memset(&md, 0, sizeof md);

    for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
      m = ldata(ln);
      if (strcmp(m->desc, "flash") == 0) {
	PDATA(pgm)->flash_pagesize = m->page_size;
	u16_to_b2(md.flash_page_size, m->page_size);
	u32_to_b4(md.flash_size, m->size);
	// do we need it?  just a wild guess
	u32_to_b4(md.boot_address, (m->size - m->page_size * 4) / 2);
      } else if (strcmp(m->desc, "eeprom") == 0) {
	PDATA(pgm)->eeprom_pagesize = m->page_size;
	md.eeprom_page_size = m->page_size;
	u16_to_b2(md.eeprom_size, m->size);
      }
    }

    //md.sram_offset[2] = p->sram;  // do we need it?
    md.ocd_revision = 3;        /* XXX! */
    md.always_one = 1;
    md.allow_full_page_bitstream = (p->flags & AVRPART_ALLOWFULLPAGEBITSTREAM) != 0;
    md.idr_address = p->idr;

    if (p->eecr == 0)
      p->eecr = 0x3f;		/* matches most "modern" mega/tiny AVRs */
    md.eearh_address = p->eecr - 0x20 + 3;
    md.eearl_address = p->eecr - 0x20 + 2;
    md.eecr_address = p->eecr - 0x20;
    md.eedr_address = p->eecr - 0x20 + 1;
    md.spmcr_address = p->spmcr;
    //md.osccal_address = p->osccal;  // do we need it at all?

    if (jtag3_setparm(pgm, SCOPE_AVR, 2, PARM3_DEVICEDESC, (unsigned char *)&md, sizeof md) < 0)
      return -1;
  }

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_SIGN_ON;
  cmd[2] = 0;
  cmd[3] = 0;			/* external reset */

  if ((status = jtag3_command(pgm, cmd, 4, &resp, "AVR sign-on")) < 0)
    return -1;

  /*
   * Depending on the target connection, there are two different
   * possible replies of the ICE.  For a JTAG connection, the reply
   * format is RSP3_DATA, followed by 4 bytes of the JTAG ID read from
   * the device (followed by a trailing 0).  For all other connections
   * (except ISP which is handled completely differently, but that
   * doesn't apply here anyway), the response is just RSP_OK.
   */
  if (resp[1] == RSP3_DATA && status >= 7 && verbose >= 1)
    /* JTAG ID has been returned */
    fprintf(stderr, "%s: JTAG ID returned: 0x%02x 0x%02x 0x%02x 0x%02x\n",
	    progname, resp[3], resp[4], resp[5], resp[6]);

  free(resp);

  PDATA(pgm)->boot_start = ULONG_MAX;
  if ((p->flags & AVRPART_HAS_PDI)) {
    /*
     * Find out where the border between application and boot area
     * is.
     */
    AVRMEM *bootmem = avr_locate_mem(p, "boot");
    AVRMEM *flashmem = avr_locate_mem(p, "flash");
    if (bootmem == NULL || flashmem == NULL) {
      fprintf(stderr,
	      "%s: jtagmk3_initialize(): Cannot locate \"flash\" and \"boot\" memories in description\n",
	      progname);
    } else {
      PDATA(pgm)->boot_start = bootmem->offset - flashmem->offset;
    }
  }

  free(PDATA(pgm)->flash_pagecache);
  free(PDATA(pgm)->eeprom_pagecache);
  if ((PDATA(pgm)->flash_pagecache = malloc(PDATA(pgm)->flash_pagesize)) == NULL) {
    fprintf(stderr, "%s: jtag3_initialize(): Out of memory\n",
	    progname);
    return -1;
  }
  if ((PDATA(pgm)->eeprom_pagecache = malloc(PDATA(pgm)->eeprom_pagesize)) == NULL) {
    fprintf(stderr, "%s: jtag3_initialize(): Out of memory\n",
	    progname);
    free(PDATA(pgm)->flash_pagecache);
    return -1;
  }
  PDATA(pgm)->flash_pageaddr = PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;

  if (jtag3_reset(pgm, 0x01) < 0)
    return -1;

  if ((pgm->flag & PGM_FL_IS_JTAG) && !(p->flags & AVRPART_HAS_PDI)) {
    strcpy(hfuse.desc, "hfuse");
    if (jtag3_read_byte(pgm, p, &hfuse, 1, &b) < 0)
      return -1;
    if ((b & OCDEN) != 0)
      fprintf(stderr,
	      "%s: jtag3_initialize(): warning: OCDEN fuse not programmed, "
	      "single-byte EEPROM updates not possible\n",
	      progname);
  }

  return 0;
}

static void jtag3_disable(PROGRAMMER * pgm)
{

  free(PDATA(pgm)->flash_pagecache);
  PDATA(pgm)->flash_pagecache = NULL;
  free(PDATA(pgm)->eeprom_pagecache);
  PDATA(pgm)->eeprom_pagecache = NULL;

  /*
   * jtag3_program_disable() doesn't do anything if the
   * device is currently not in programming mode, so just
   * call it unconditionally here.
   */
  (void)jtag3_program_disable(pgm);
}

static void jtag3_enable(PROGRAMMER * pgm)
{
  return;
}

static int jtag3_parseextparms(PROGRAMMER * pgm, LISTID extparms)
{
  LNODEID ln;
  const char *extended_param;
  int rv = 0;

  for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if (strncmp(extended_param, "jtagchain=", strlen("jtagchain=")) == 0) {
      unsigned int ub, ua, bb, ba;
      if (sscanf(extended_param, "jtagchain=%u,%u,%u,%u", &ub, &ua, &bb, &ba)
          != 4) {
        fprintf(stderr,
                "%s: jtag3_parseextparms(): invalid JTAG chain '%s'\n",
                progname, extended_param);
        rv = -1;
        continue;
      }
      if (verbose >= 2) {
        fprintf(stderr,
                "%s: jtag3_parseextparms(): JTAG chain parsed as:\n"
                "%s %u units before, %u units after, %u bits before, %u bits after\n",
                progname,
                progbuf, ub, ua, bb, ba);
      }
      PDATA(pgm)->jtagchain[0] = ub;
      PDATA(pgm)->jtagchain[1] = ua;
      PDATA(pgm)->jtagchain[2] = bb;
      PDATA(pgm)->jtagchain[3] = ba;

      continue;
    }

    fprintf(stderr,
            "%s: jtag3_parseextparms(): invalid extended parameter '%s'\n",
            progname, extended_param);
    rv = -1;
  }

  return rv;
}


static int jtag3_open(PROGRAMMER * pgm, char * port)
{
  long baud;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_open()\n", progname);

  /*
   * The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev_frame;
    baud = USB_DEVICE_JTAGICE3;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_3;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_3;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_3;
    pgm->fd.usb.eep = USBDEV_EVT_EP_READ_3;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, baud, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtag3_drain(pgm, 0);

  if (jtag3_getsync(pgm, PARM3_CONN_JTAG) < 0)
    return -1;

  return 0;
}

static int jtag3_open_dw(PROGRAMMER * pgm, char * port)
{
  long baud;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_open_dw()\n", progname);

  /*
   * The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev_frame;
    baud = USB_DEVICE_JTAGICE3;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_3;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_3;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_3;
    pgm->fd.usb.eep = USBDEV_EVT_EP_READ_3;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, baud, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtag3_drain(pgm, 0);

  if (jtag3_getsync(pgm, PARM3_CONN_DW) < 0)
    return -1;

  return 0;
}

static int jtag3_open_pdi(PROGRAMMER * pgm, char * port)
{
  long baud;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_open_pdi()\n", progname);

  /*
   * The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (strncmp(port, "usb", 3) == 0) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev_frame;
    baud = USB_DEVICE_JTAGICE3;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_3;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_3;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_3;
    pgm->fd.usb.eep = USBDEV_EVT_EP_READ_3;
#else
    fprintf(stderr, "avrdude was compiled without usb support.\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, baud, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtag3_drain(pgm, 0);

  if (jtag3_getsync(pgm, PARM3_CONN_PDI) < 0)
    return -1;

  return 0;
}


void jtag3_close(PROGRAMMER * pgm)
{
  unsigned char buf[4], *resp;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_close()\n", progname);

#if 0
  if (pgm->flag & PGM_FL_IS_PDI) {
    /* When in PDI mode, restart target. */
    buf[0] = CMND_GO;
    if (verbose >= 2)
      fprintf(stderr, "%s: jtag3_close(): Sending GO command: ",
	      progname);
    jtag3_send(pgm, buf, 1);

    status = jtag3_recv(pgm, &resp);
    if (status <= 0) {
      if (verbose >= 2)
	putc('\n', stderr);
      fprintf(stderr,
	      "%s: jtag3_close(): "
	      "timeout/error communicating with programmer (status %d)\n",
	      progname, status);
    } else {
      if (verbose >= 3) {
	putc('\n', stderr);
	jtag3_prmsg(pgm, resp, status);
      } else if (verbose == 2)
	fprintf(stderr, "0x%02x (%d bytes msg)\n", resp[1], status);
      c = resp[1];
      free(resp);
      if (c != RSP_OK) {
	fprintf(stderr,
		"%s: jtag3_close(): "
		"bad response to GO command: %s\n",
		progname, jtag3_get_rc(c));
      }
    }
  }
#endif

  buf[0] = SCOPE_AVR;
  buf[1] = CMD3_SIGN_OFF;
  buf[2] = buf[3] = 0;

  if (jtag3_command(pgm, buf, 3, &resp, "AVR sign-off") >= 0)
    free(resp);

  buf[0] = SCOPE_GENERAL;
  buf[1] = CMD3_SIGN_OFF;

  if (jtag3_command(pgm, buf, 4, &resp, "sign-off") >= 0)
    free(resp);

  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}

static int jtag3_page_erase(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                               unsigned int addr)
{
  unsigned char cmd[8], *resp;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_page_erase(.., %s, 0x%x)\n",
	    progname, m->desc, addr);

  if (!(p->flags & AVRPART_HAS_PDI)) {
    fprintf(stderr, "%s: jtag3_page_erase: not an Xmega device\n",
	    progname);
    return -1;
  }

  if (jtag3_program_enable(pgm) < 0)
    return -1;

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_ERASE_MEMORY;
  cmd[2] = 0;

  if (strcmp(m->desc, "flash") == 0) {
    if (jtag3_memtype(pgm, p, addr) == MTYPE_FLASH)
      cmd[3] = XMEGA_ERASE_APP_PAGE;
    else
      cmd[3] = XMEGA_ERASE_BOOT_PAGE;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    cmd[3] = XMEGA_ERASE_EEPROM_PAGE;
  } else if ( ( strcmp(m->desc, "usersig") == 0 ) ) {
    cmd[3] = XMEGA_ERASE_USERSIG;
  } else if ( ( strcmp(m->desc, "boot") == 0 ) ) {
    cmd[3] = XMEGA_ERASE_BOOT_PAGE;
  } else {
    cmd[3] = XMEGA_ERASE_APP_PAGE;
  }

  u32_to_b4(cmd + 4, addr + m->offset);

  if (jtag3_command(pgm, cmd, 8, &resp, "page erase") < 0)
    return -1;

  free(resp);
  return 0;
}

static int jtag3_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes)
{
  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char *cmd;
  unsigned char *resp;
  int status, dynamic_memtype = 0;
  long otimeout = serial_recv_timeout;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_paged_write(.., %s, %d, %d)\n",
	    progname, m->desc, page_size, n_bytes);

  if (!(pgm->flag & PGM_FL_IS_DW) && jtag3_program_enable(pgm) < 0)
    return -1;

  if (page_size == 0) page_size = 256;

  if ((cmd = malloc(page_size + 13)) == NULL) {
    fprintf(stderr, "%s: jtag3_paged_write(): Out of memory\n",
	    progname);
    return -1;
  }

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_WRITE_MEMORY;
  cmd[2] = 0;
  if (strcmp(m->desc, "flash") == 0) {
    PDATA(pgm)->flash_pageaddr = (unsigned long)-1L;
    cmd[3] = jtag3_memtype(pgm, p, addr);
    if (p->flags & AVRPART_HAS_PDI)
      /* dynamically decide between flash/boot memtype */
      dynamic_memtype = 1;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    if (pgm->flag & PGM_FL_IS_DW) {
      /*
       * jtag3_paged_write() to EEPROM attempted while in
       * DW mode.  Use jtag3_write_byte() instead.
       */
      for (; addr < maxaddr; addr++) {
	status = jtag3_write_byte(pgm, p, m, addr, m->buf[addr]);
	if (status < 0) {
	  free(cmd);
	  return -1;
	}
      }
      free(cmd);
      return n_bytes;
    }
    cmd[3] = ( p->flags & AVRPART_HAS_PDI ) ? MTYPE_EEPROM : MTYPE_EEPROM_PAGE;
    PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;
  } else if ( ( strcmp(m->desc, "usersig") == 0 ) ) {
    cmd[3] = MTYPE_USERSIG;
  } else if ( ( strcmp(m->desc, "boot") == 0 ) ) {
    cmd[3] = MTYPE_BOOT_FLASH;
  } else if ( p->flags & AVRPART_HAS_PDI ) {
    cmd[3] = MTYPE_FLASH;
  } else {
    cmd[3] = MTYPE_SPM;
  }
  serial_recv_timeout = 100;
  for (; addr < maxaddr; addr += page_size) {
    if ((maxaddr - addr) < page_size)
      block_size = maxaddr - addr;
    else
      block_size = page_size;
    if (verbose >= 3)
      fprintf(stderr, "%s: jtag3_paged_write(): "
	      "block_size at addr %d is %d\n",
	      progname, addr, block_size);

    if (dynamic_memtype)
      cmd[3] = jtag3_memtype(pgm, p, addr);

    u32_to_b4(cmd + 8, page_size);
    u32_to_b4(cmd + 4, jtag3_memaddr(pgm, p, m, addr));
    cmd[12] = 0;

    /*
     * The JTAG ICE will refuse to write anything but a full page, at
     * least for the flash ROM.  If a partial page has been requested,
     * set the remainder to 0xff.  (Maybe we should rather read back
     * the existing contents instead before?  Doesn't matter much, as
     * bits cannot be written to 1 anyway.)
     */
    memset(cmd + 13, 0xff, page_size);
    memcpy(cmd + 13, m->buf + addr, block_size);

    if ((status = jtag3_command(pgm, cmd, page_size + 13,
				&resp, "write memory")) < 0) {
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

static int jtag3_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                               unsigned int page_size,
                               unsigned int addr, unsigned int n_bytes)
{
  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char cmd[12];
  unsigned char *resp;
  int status, dynamic_memtype = 0;
  long otimeout = serial_recv_timeout;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_paged_load(.., %s, %d, %d)\n",
	    progname, m->desc, page_size, n_bytes);

  if (!(pgm->flag & PGM_FL_IS_DW) && jtag3_program_enable(pgm) < 0)
    return -1;

  page_size = m->readsize;

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_READ_MEMORY;
  cmd[2] = 0;

  if (strcmp(m->desc, "flash") == 0) {
    cmd[3] = jtag3_memtype(pgm, p, addr);
    if (p->flags & AVRPART_HAS_PDI)
      /* dynamically decide between flash/boot memtype */
      dynamic_memtype = 1;
  } else if (strcmp(m->desc, "eeprom") == 0) {
    cmd[3] = ( p->flags & AVRPART_HAS_PDI ) ? MTYPE_EEPROM : MTYPE_EEPROM_PAGE;
    if (pgm->flag & PGM_FL_IS_DW)
      return -1;
  } else if ( ( strcmp(m->desc, "prodsig") == 0 ) ) {
    cmd[3] = MTYPE_PRODSIG;
  } else if ( ( strcmp(m->desc, "usersig") == 0 ) ) {
    cmd[3] = MTYPE_USERSIG;
  } else if ( ( strcmp(m->desc, "boot") == 0 ) ) {
    cmd[3] = MTYPE_BOOT_FLASH;
  } else if ( p->flags & AVRPART_HAS_PDI ) {
    cmd[3] = MTYPE_FLASH;
  } else {
    cmd[3] = MTYPE_SPM;
  }
  serial_recv_timeout = 100;
  for (; addr < maxaddr; addr += page_size) {
    if ((maxaddr - addr) < page_size)
      block_size = maxaddr - addr;
    else
      block_size = page_size;
    if (verbose >= 3)
      fprintf(stderr, "%s: jtag3_paged_load(): "
	      "block_size at addr %d is %d\n",
	      progname, addr, block_size);

    if (dynamic_memtype)
      cmd[3] = jtag3_memtype(pgm, p, addr);

    u32_to_b4(cmd + 8, block_size);
    u32_to_b4(cmd + 4, jtag3_memaddr(pgm, p, m, addr));

    if ((status = jtag3_command(pgm, cmd, 12, &resp, "read memory")) < 0)
      return -1;

    if (resp[1] != RSP3_DATA ||
	status < block_size + 4) {
      fprintf(stderr, "%s: wrong/short reply to read memory command\n",
	      progname);
      serial_recv_timeout = otimeout;
      free(resp);
      return -1;
    }
    memcpy(m->buf + addr, resp + 3, status - 4);
    free(resp);
  }
  serial_recv_timeout = otimeout;

  return n_bytes;
}

static int jtag3_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			      unsigned long addr, unsigned char * value)
{
  unsigned char cmd[12];
  unsigned char *resp, *cache_ptr = NULL;
  int status, unsupp = 0;
  unsigned long paddr = 0UL, *paddr_ptr = NULL;
  unsigned int pagesize = 0;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_read_byte(.., %s, 0x%lx, ...)\n",
	    progname, mem->desc, addr);

  if (!(pgm->flag & PGM_FL_IS_DW) && jtag3_program_enable(pgm) < 0)
    return -1;

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_READ_MEMORY;
  cmd[2] = 0;

  cmd[3] = ( p->flags & AVRPART_HAS_PDI ) ? MTYPE_FLASH : MTYPE_FLASH_PAGE;
  if (strcmp(mem->desc, "flash") == 0 ||
      strcmp(mem->desc, "application") == 0 ||
      strcmp(mem->desc, "apptable") == 0 ||
      strcmp(mem->desc, "boot") == 0) {
    pagesize = PDATA(pgm)->flash_pagesize;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &PDATA(pgm)->flash_pageaddr;
    cache_ptr = PDATA(pgm)->flash_pagecache;
  } else if (strcmp(mem->desc, "eeprom") == 0) {
    if ( (pgm->flag & PGM_FL_IS_DW) || ( p->flags & AVRPART_HAS_PDI ) ) {
      cmd[3] = MTYPE_EEPROM;
    } else {
      cmd[3] = MTYPE_EEPROM_PAGE;
    }
    pagesize = mem->page_size;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &PDATA(pgm)->eeprom_pageaddr;
    cache_ptr = PDATA(pgm)->eeprom_pagecache;
  } else if (strcmp(mem->desc, "lfuse") == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
    addr = 0;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "hfuse") == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
    addr = 1;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "efuse") == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
    addr = 2;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "lock") == 0) {
    cmd[3] = MTYPE_LOCK_BITS;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strncmp(mem->desc, "fuse", strlen("fuse")) == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
  } else if (strcmp(mem->desc, "usersig") == 0) {
    cmd[3] = MTYPE_USERSIG;
  } else if (strcmp(mem->desc, "prodsig") == 0) {
    cmd[3] = MTYPE_PRODSIG;
  } else if (strcmp(mem->desc, "calibration") == 0) {
    cmd[3] = MTYPE_OSCCAL_BYTE;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "signature") == 0) {
    static unsigned char signature_cache[2];

    cmd[3] = MTYPE_SIGN_JTAG;

    /*
     * dW can read out the signature on JTAGICE3, but only allows
     * for a full three-byte read.  We cache them in a local
     * variable to avoid multiple reads.  This optimization does not
     * harm for other connection types either.
     */
    u32_to_b4(cmd + 8, 3);
    u32_to_b4(cmd + 4, 0);

    if (addr == 0) {
      if ((status = jtag3_command(pgm, cmd, 12, &resp, "read memory")) < 0)
	return -1;

      signature_cache[0] = resp[4];
      signature_cache[1] = resp[5];
      *value = resp[3];
      free(resp);
      return 0;
    } else if (addr <= 2) {
      *value = signature_cache[addr - 1];
      return 0;
    } else {
      /* should not happen */
      fprintf(stderr, "address out of range for signature memory: %u\n", addr);
      return -1;
    }
  }

  /*
   * If the respective memory area is not supported under debugWire,
   * leave here.
   */
  if (unsupp) {
    *value = 42;
    return -1;
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
    u32_to_b4(cmd + 8, pagesize);
    u32_to_b4(cmd + 4, paddr);
  } else {
    u32_to_b4(cmd + 8, 1);
    u32_to_b4(cmd + 4, addr);
  }

  if ((status = jtag3_command(pgm, cmd, 12, &resp, "read memory")) < 0)
    return -1;

  if (resp[1] != RSP3_DATA ||
      status < (pagesize? pagesize: 1) + 4) {
    fprintf(stderr, "%s: wrong/short reply to read memory command\n",
	    progname);
    free(resp);
    return -1;
  }

  if (pagesize) {
    *paddr_ptr = paddr;
    memcpy(cache_ptr, resp + 3, pagesize);
    *value = cache_ptr[addr & (pagesize - 1)];
  } else
    *value = resp[3];

  free(resp);
  return 0;
}

static int jtag3_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			       unsigned long addr, unsigned char data)
{
  unsigned char cmd[14];
  unsigned char *resp;
  int status, need_progmode = 1, unsupp = 0;

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_write_byte(.., %s, 0x%lx, ...)\n",
	    progname, mem->desc, addr);

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_WRITE_MEMORY;
  cmd[2] = 0;
  cmd[3] = ( p->flags & AVRPART_HAS_PDI ) ? MTYPE_FLASH : MTYPE_SPM;
  if (strcmp(mem->desc, "flash") == 0) {
     need_progmode = 0;
     PDATA(pgm)->flash_pageaddr = (unsigned long)-1L;
     if (pgm->flag & PGM_FL_IS_DW)
       unsupp = 1;
  } else if (strcmp(mem->desc, "eeprom") == 0) {
    cmd[3] = MTYPE_EEPROM;
    need_progmode = 0;
    PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;
  } else if (strcmp(mem->desc, "lfuse") == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
    addr = 0;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "hfuse") == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
    addr = 1;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "efuse") == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
    addr = 2;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strncmp(mem->desc, "fuse", strlen("fuse")) == 0) {
    cmd[3] = MTYPE_FUSE_BITS;
  } else if (strcmp(mem->desc, "usersig") == 0) {
    cmd[3] = MTYPE_USERSIG;
  } else if (strcmp(mem->desc, "prodsig") == 0) {
    cmd[3] = MTYPE_PRODSIG;
  } else if (strcmp(mem->desc, "lock") == 0) {
    cmd[3] = MTYPE_LOCK_BITS;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "calibration") == 0) {
    cmd[3] = MTYPE_OSCCAL_BYTE;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (strcmp(mem->desc, "signature") == 0) {
    cmd[3] = MTYPE_SIGN_JTAG;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  }

  if (unsupp)
    return -1;

  if (need_progmode) {
    if (jtag3_program_enable(pgm) < 0)
      return -1;
  } else {
    if (jtag3_program_disable(pgm) < 0)
      return -1;
  }

  u32_to_b4(cmd + 8, 1);
  u32_to_b4(cmd + 4, addr);
  cmd[12] = 0;
  cmd[13] = data;

  if ((status = jtag3_command(pgm, cmd, 14, &resp, "write memory")) < 0)
    return -1;

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
static int jtag3_set_sck_period(PROGRAMMER * pgm, double v)
{
#if 0
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

  return jtag3_setparm(pgm, PAR_OCD_JTAG_CLK, &dur);
#endif
  return 0;
}


/*
 * Read (an) emulator parameter(s).
 */
int jtag3_getparm(PROGRAMMER * pgm, unsigned char scope,
		  unsigned char section, unsigned char parm,
		  unsigned char *value, unsigned char length)
{
  int status;
  unsigned char buf[6], *resp, c;
  char descr[60];

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_getparm()\n", progname);

  buf[0] = scope;
  buf[1] = CMD3_GET_PARAMETER;
  buf[2] = 0;
  buf[3] = section;
  buf[4] = parm;
  buf[5] = length;

  sprintf(descr, "get parameter (scope 0x%02x, section %d, parm %d)",
	  scope, section, parm);

  if ((status = jtag3_command(pgm, buf, 6, &resp, descr)) < 0)
    return -1;

  c = resp[1];
  if (c != RSP3_DATA || status < 3) {
    fprintf(stderr,
	    "%s: jtag3_getparm(): "
	    "bad response to %s\n",
	    progname, descr);
    free(resp);
    return -1;
  }

  status -= 3;
  memcpy(value, resp + 3, (length < status? length: status));
  free(resp);

  return 0;
}

/*
 * Write an emulator parameter.
 */
static int jtag3_setparm(PROGRAMMER * pgm, unsigned char scope,
			 unsigned char section, unsigned char parm,
			 unsigned char *value, unsigned char length)
{
  int status;
  unsigned char *buf, *resp;
  char descr[60];

  if (verbose >= 2)
    fprintf(stderr, "%s: jtag3_setparm()\n", progname);

  sprintf(descr, "set parameter (scope 0x%02x, section %d, parm %d)",
	  scope, section, parm);

  if ((buf = malloc(6 + length)) == NULL)
  {
    fprintf(stderr, "%s: jtag3_setparm(): Out of memory\n",
	    progname);
    return -1;
  }

  buf[0] = scope;
  buf[1] = CMD3_SET_PARAMETER;
  buf[2] = 0;
  buf[3] = section;
  buf[4] = parm;
  buf[5] = length;
  memcpy(buf + 6, value, length);

  status = jtag3_command(pgm, buf, length + 6, &resp, descr);

  free(buf);
  free(resp);

  return status;
}


static void jtag3_display(PROGRAMMER * pgm, const char * p)
{
  unsigned char parms[5];
  unsigned char cmd[4], *resp, c;
  int status;

  /*
   * Ask for:
   *  PARM3_HW_VER (1 byte)
   *  PARM3_FW_MAJOR (1 byte)
   *  PARM3_FW_MINOR (1 byte)
   *  PARM3_FW_RELEASE (2 bytes)
   */
  if (jtag3_getparm(pgm, SCOPE_GENERAL, 0, PARM3_HW_VER, parms, 5) < 0)
    return;

  cmd[0] = SCOPE_INFO;
  cmd[1] = CMD3_GET_INFO;
  cmd[2] = 0;
  cmd[3] = CMD3_INFO_SERIAL;

  if ((status = jtag3_command(pgm, cmd, 4, &resp, "get info (serial number)")) < 0)
    return;

  c = resp[1];
  if (c != RSP3_INFO) {
    fprintf(stderr,
	    "%s: jtag3_display(): response is not RSP3_INFO\n",
	    progname);
    free(resp);
    return;
  }
  memmove(resp, resp + 3, status - 3);
  resp[status - 3] = 0;

  fprintf(stderr, "%sICE hardware version: %d\n", p, parms[0]);
  fprintf(stderr, "%sICE firmware version: %d.%02d (rel. %d)\n", p,
	  parms[1], parms[2],
	  (parms[3] | (parms[4] << 8)));
  fprintf(stderr, "%sSerial number   : %s\n", p, resp);
  free(resp);

  jtag3_print_parms1(pgm, p);
}


static void jtag3_print_parms1(PROGRAMMER * pgm, const char * p)
{
  unsigned char buf[2];

  if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_VTARGET, buf, 2) < 0)
    return;

  fprintf(stderr, "%sVtarget         : %.2f V\n", p,
	  b2_to_u16(buf) / 1000.0);

  if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_MEGA_PROG, buf, 2) < 0)
    return;
  fprintf(stderr, "%sJTAG clock megaAVR/program: %u kHz\n", p,
	  b2_to_u16(buf));

  if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_MEGA_DEBUG, buf, 2) < 0)
    return;
  fprintf(stderr, "%sJTAG clock megaAVR/debug:   %u kHz\n", p,
	  b2_to_u16(buf));

  if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_JTAG, buf, 2) < 0)
    return;
  fprintf(stderr, "%sJTAG clock Xmega: %u kHz\n", p,
	  b2_to_u16(buf));

  if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_PDI, buf, 2) < 0)
    return;
  fprintf(stderr, "%sPDI clock Xmega : %u kHz\n", p,
	  b2_to_u16(buf));
}

static void jtag3_print_parms(PROGRAMMER * pgm)
{
  jtag3_print_parms1(pgm, "");
}

static unsigned char jtag3_memtype(PROGRAMMER * pgm, AVRPART * p, unsigned long addr)
{
  if ( p->flags & AVRPART_HAS_PDI ) {
    if (addr >= PDATA(pgm)->boot_start)
      return MTYPE_BOOT_FLASH;
    else
      return MTYPE_FLASH;
  } else {
    return MTYPE_FLASH_PAGE;
  }
}

static unsigned int jtag3_memaddr(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, unsigned long addr)
{
  if ((p->flags & AVRPART_HAS_PDI) != 0) {
    if (addr >= PDATA(pgm)->boot_start)
      /*
       * all memories but "flash" are smaller than boot_start anyway, so
       * no need for an extra check we are operating on "flash"
       */
      return addr - PDATA(pgm)->boot_start;
    else
      /* normal flash, or anything else */
      return addr;
  }
  /*
   * Non-Xmega device.
   */
  return addr;
}


const char jtag3_desc[] = "Atmel JTAGICE3";

void jtag3_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "JTAGICE3");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtag3_initialize;
  pgm->display        = jtag3_display;
  pgm->enable         = jtag3_enable;
  pgm->disable        = jtag3_disable;
  pgm->program_enable = jtag3_program_enable_dummy;
  pgm->chip_erase     = jtag3_chip_erase;
  pgm->open           = jtag3_open;
  pgm->close          = jtag3_close;
  pgm->read_byte      = jtag3_read_byte;
  pgm->write_byte     = jtag3_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtag3_paged_write;
  pgm->paged_load     = jtag3_paged_load;
  pgm->page_erase     = jtag3_page_erase;
  pgm->print_parms    = jtag3_print_parms;
  pgm->set_sck_period = jtag3_set_sck_period;
  pgm->parseextparams = jtag3_parseextparms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_JTAG;
}

const char jtag3_dw_desc[] = "Atmel JTAGICE3 in debugWire mode";

void jtag3_dw_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "JTAGICE3_DW");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtag3_initialize;
  pgm->display        = jtag3_display;
  pgm->enable         = jtag3_enable;
  pgm->disable        = jtag3_disable;
  pgm->program_enable = jtag3_program_enable_dummy;
  pgm->chip_erase     = jtag3_chip_erase_dw;
  pgm->open           = jtag3_open_dw;
  pgm->close          = jtag3_close;
  pgm->read_byte      = jtag3_read_byte;
  pgm->write_byte     = jtag3_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtag3_paged_write;
  pgm->paged_load     = jtag3_paged_load;
  pgm->print_parms    = jtag3_print_parms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_DW;
}

const char jtag3_pdi_desc[] = "Atmel JTAGICE3 in PDI mode";

void jtag3_pdi_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "JTAGICE3_PDI");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtag3_initialize;
  pgm->display        = jtag3_display;
  pgm->enable         = jtag3_enable;
  pgm->disable        = jtag3_disable;
  pgm->program_enable = jtag3_program_enable_dummy;
  pgm->chip_erase     = jtag3_chip_erase;
  pgm->open           = jtag3_open_pdi;
  pgm->close          = jtag3_close;
  pgm->read_byte      = jtag3_read_byte;
  pgm->write_byte     = jtag3_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtag3_paged_write;
  pgm->paged_load     = jtag3_paged_load;
  pgm->page_erase     = jtag3_page_erase;
  pgm->print_parms    = jtag3_print_parms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_PDI;
}

