/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005-2007 Joerg Wunsch <j@uriah.heep.sax.de>
 *
 * Derived from stk500 code which is:
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bdmicro.com>
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* $Id$ */

/*
 * avrdude interface for Atmel JTAG ICE mkII programmer
 *
 * The AVR Dragon also uses the same protocol, so it is handled here
 * as well.
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
#include "libavrdude.h"

#include "crc16.h"
#include "jtagmkII.h"
#include "jtagmkII_private.h"
#include "usbdevs.h"

/*
 * Private data for this programmer.
 */
struct pdata
{
  unsigned short command_sequence; /* Next cmd seqno to issue. */

  /*
   * See jtagmkII_read_byte() for an explanation of the flash and
   * EEPROM page caches.
   */
  unsigned char *flash_pagecache;
  unsigned long flash_pageaddr;
  unsigned int flash_pagesize;

  unsigned char *eeprom_pagecache;
  unsigned long eeprom_pageaddr;
  unsigned int eeprom_pagesize;

  int prog_enabled;	     /* Cached value of PROGRAMMING status. */
  unsigned char serno[6];	/* JTAG ICE serial number. */

  /* JTAG chain stuff */
  unsigned char jtagchain[4];

  /* Serial RTS/DTR setting */
  int rts_mode;

  /* The length of the device descriptor is firmware-dependent. */
  size_t device_descriptor_length;

  /* Start address of Xmega boot area */
  unsigned long boot_start;

  /* Major firmware version (needed for Xmega programming) */
  unsigned int fwver;

  /* Most recent operation was a memory write or erase */
  int recently_written;

#define FLAGS32_INIT_SMC      1 // Part will undergo chip erase
#define FLAGS32_WRITE         2 // At least one write operation specified
  // Couple of flag bits for AVR32 programming
  int flags32;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

#define RC(x) { x, #x },
static struct {
  unsigned int code;
  const char *descr;
} jtagresults[] = {
  RC(RSP_DEBUGWIRE_SYNC_FAILED)
  RC(RSP_FAILED)
  RC(RSP_ILLEGAL_BREAKPOINT)
  RC(RSP_ILLEGAL_COMMAND)
  RC(RSP_ILLEGAL_EMULATOR_MODE)
  RC(RSP_ILLEGAL_JTAG_ID)
  RC(RSP_ILLEGAL_MCU_STATE)
  RC(RSP_ILLEGAL_MEMORY_TYPE)
  RC(RSP_ILLEGAL_MEMORY_RANGE)
  RC(RSP_ILLEGAL_PARAMETER)
  RC(RSP_ILLEGAL_POWER_STATE)
  RC(RSP_ILLEGAL_VALUE)
  RC(RSP_NO_TARGET_POWER)
  RC(RSP_SET_N_PARAMETERS)
};

/*
 * pgm->flag is marked as "for private use of the programmer".
 * The following defines this programmer's use of that field.
 */
#define PGM_FL_IS_DW		(0x0001)
#define PGM_FL_IS_PDI           (0x0002)
#define PGM_FL_IS_JTAG          (0x0004)

static int jtagmkII_open(PROGRAMMER *pgm, const char *port);

static int jtagmkII_initialize(const PROGRAMMER *pgm, const AVRPART *p);
static int jtagmkII_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
static int jtagmkII_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                                unsigned long addr, unsigned char * value);
static int jtagmkII_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                                unsigned long addr, unsigned char data);
static int jtagmkII_reset(const PROGRAMMER *pgm, unsigned char flags);
static int jtagmkII_set_sck_period(const PROGRAMMER *pgm, double v);
static int jtagmkII_setparm(const PROGRAMMER *pgm, unsigned char parm,
                            unsigned char * value);
static void jtagmkII_print_parms1(const PROGRAMMER *pgm, const char *p, FILE *fp);
static int jtagmkII_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes);
static unsigned char jtagmkII_mtype(const PROGRAMMER *pgm, const AVRPART *p, unsigned long addr);
static unsigned int jtagmkII_memaddr(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr);

// AVR32
#define ERROR_SAB 0xFFFFFFFF

static int jtagmkII_open32(PROGRAMMER *pgm, const char *port);
static void jtagmkII_close32(PROGRAMMER * pgm);
static int jtagmkII_reset32(const PROGRAMMER *pgm, unsigned short flags);
static int jtagmkII_initialize32(const PROGRAMMER *pgm, const AVRPART *p);
static int jtagmkII_chip_erase32(const PROGRAMMER *pgm, const AVRPART *p);
static unsigned long jtagmkII_read_SABaddr(const PROGRAMMER *pgm, unsigned long addr,
                      unsigned int prefix); // ERROR_SAB illegal
static int jtagmkII_write_SABaddr(const PROGRAMMER *pgm, unsigned long addr,
                                  unsigned int prefix, unsigned long val);
static int jtagmkII_avr32_reset(const PROGRAMMER *pgm, unsigned char val,
                                  unsigned char ret1, unsigned char ret2);
static int jtagmkII_smc_init32(const PROGRAMMER *pgm);
static int jtagmkII_paged_write32(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                  unsigned int page_size,
                                  unsigned int addr, unsigned int n_bytes);
static int jtagmkII_flash_lock32(const PROGRAMMER *pgm, unsigned char lock,
                                  unsigned int page);
static int jtagmkII_flash_erase32(const PROGRAMMER *pgm, unsigned int page);
static int jtagmkII_flash_write_page32(const PROGRAMMER *pgm, unsigned int page);
static int jtagmkII_flash_clear_pagebuffer32(const PROGRAMMER *pgm);
static int jtagmkII_paged_load32(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                 unsigned int page_size,
                                 unsigned int addr, unsigned int n_bytes);

void jtagmkII_setup(PROGRAMMER *pgm) {
  pgm->cookie = cfg_malloc("jtagmkII_setup()", sizeof(struct pdata));
  PDATA(pgm)->rts_mode = RTS_MODE_DEFAULT;
}

void jtagmkII_teardown(PROGRAMMER *pgm) {
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
static unsigned long
b4_to_u32r(unsigned char *b)
{
  unsigned long l;
  l = b[3];
  l += (unsigned)b[2] << 8;
  l += (unsigned)b[1] << 16;
  l += (unsigned)b[0] << 24;

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
static void
u32_to_b4r(unsigned char *b, unsigned long l)
{
  b[3] = l & 0xff;
  b[2] = (l >> 8) & 0xff;
  b[1] = (l >> 16) & 0xff;
  b[0] = (l >> 24) & 0xff;
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

static const char *
jtagmkII_get_rc(unsigned int rc)
{
  static char msg[64];

  for (size_t i = 0; i < sizeof jtagresults/sizeof*jtagresults; i++)
    if (jtagresults[i].code == rc)
      return jtagresults[i].descr;

  sprintf(msg, "Unknown JTAG ICE mkII result code 0x%02x", rc);
  return msg;
}


static void jtagmkII_print_memory(unsigned char *b, size_t s)
{
  size_t i;

  if (s < 2)
    return;

  for (i = 0; i < s - 1; i++) {
    msg_info("0x%02x ", b[i + 1]);
    if (i % 16 == 15)
      msg_info("\n");
    else
      msg_info(" ");
  }
  if (i % 16 != 0)
    msg_info("\n");
}

static void jtagmkII_prmsg(const PROGRAMMER *pgm_unused, unsigned char *data, size_t len) {
  size_t i;

  if (verbose >= 4) {
    msg_trace("Raw message:\n");

    for (i = 0; i < len; i++) {
      msg_trace("0x%02x", data[i]);
      if (i % 16 == 15)
	msg_trace("\n");
      else
	msg_trace(" ");
    }
    if (i % 16 != 0)
      msg_trace("\n");
  }

  switch (data[0]) {
  case RSP_OK:
    msg_info("OK\n");
    break;

  case RSP_FAILED:
    msg_info("FAILED\n");
    break;

  case RSP_ILLEGAL_BREAKPOINT:
    msg_info("Illegal breakpoint\n");
    break;

  case RSP_ILLEGAL_COMMAND:
    msg_info("Illegal command\n");
    break;

  case RSP_ILLEGAL_EMULATOR_MODE:
    msg_info("Illegal emulator mode");
    if (len > 1)
      switch (data[1]) {
      case EMULATOR_MODE_DEBUGWIRE: msg_info(": DebugWire"); break;
      case EMULATOR_MODE_JTAG:      msg_info(": JTAG"); break;
      case EMULATOR_MODE_HV:        msg_info(": HVSP/PP"); break;
      case EMULATOR_MODE_SPI:       msg_info(": SPI"); break;
      case EMULATOR_MODE_JTAG_XMEGA: msg_info(": JTAG/Xmega"); break;
      }
    msg_info("\n");
    break;

  case RSP_ILLEGAL_JTAG_ID:
    msg_info("Illegal JTAG ID\n");
    break;

  case RSP_ILLEGAL_MCU_STATE:
    msg_info("Illegal MCU state");
    if (len > 1)
      switch (data[1]) {
      case STOPPED:     msg_info(": Stopped"); break;
      case RUNNING:     msg_info(": Running"); break;
      case PROGRAMMING: msg_info(": Programming"); break;
      }
    msg_info("\n");
    break;

  case RSP_ILLEGAL_MEMORY_TYPE:
    msg_info("Illegal memory type\n");
    break;

  case RSP_ILLEGAL_MEMORY_RANGE:
    msg_info("Illegal memory range\n");
    break;

  case RSP_ILLEGAL_PARAMETER:
    msg_info("Illegal parameter\n");
    break;

  case RSP_ILLEGAL_POWER_STATE:
    msg_info("Illegal power state\n");
    break;

  case RSP_ILLEGAL_VALUE:
    msg_info("Illegal value\n");
    break;

  case RSP_NO_TARGET_POWER:
    msg_info("No target power\n");
    break;

  case RSP_SIGN_ON:
    msg_info("Sign-on succeeded\n");
    /* Sign-on data will be printed below anyway. */
    break;

  case RSP_MEMORY:
    msg_info("memory contents:\n");
    jtagmkII_print_memory(data, len);
    break;

  case RSP_PARAMETER:
    msg_info("parameter values:\n");
    jtagmkII_print_memory(data, len);
    break;

  case RSP_SPI_DATA:
    msg_info("SPI data returned:\n");
    for (i = 1; i < len; i++)
      msg_info("0x%02x ", data[i]);
    msg_info("\n");
    break;

  case EVT_BREAK:
    msg_info("BREAK event");
    if (len >= 6) {
      msg_info(", PC = 0x%lx, reason ", b4_to_u32(data + 1));
      switch (data[5]) {
      case 0x00:
	msg_info("unspecified");
	break;
      case 0x01:
	msg_info("program break");
	break;
      case 0x02:
	msg_info("data break PDSB");
	break;
      case 0x03:
	msg_info("data break PDMSB");
	break;
      default:
	msg_info("unknown: 0x%02x", data[5]);
      }
    }
    msg_info("\n");
    break;

  default:
    msg_info("unknown message 0x%02x\n", data[0]);
  }

  msg_info("\n");
}


int jtagmkII_send(const PROGRAMMER *pgm, unsigned char *data, size_t len) {
  unsigned char *buf;

  msg_debug("\n");
  pmsg_debug("jtagmkII_send(): sending %lu bytes\n", (unsigned long) len);

  if ((buf = malloc(len + 10)) == NULL)
    {
      pmsg_error("out of memory");
      return -1;
    }

  buf[0] = MESSAGE_START;
  u16_to_b2(buf + 1, PDATA(pgm)->command_sequence);
  u32_to_b4(buf + 3, len);
  buf[7] = TOKEN;
  memcpy(buf + 8, data, len);

  crcappend(buf, len + 8);

  if (serial_send(&pgm->fd, buf, len + 10) != 0) {
    pmsg_error("unable to send command to serial port\n");
    free(buf);
    return -1;
  }

  free(buf);

  return 0;
}


static int jtagmkII_drain(const PROGRAMMER *pgm, int display) {
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
static int jtagmkII_recv_frame(const PROGRAMMER *pgm, unsigned char **msg,
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

  double timeoutval = 100;	/* seconds */
  double tstart, tnow;

  pmsg_trace("jtagmkII_recv():\n");

  tstart = avr_timestamp();

  while ( (state != sDONE ) && (!timeout) ) {
    if (state == sDATA) {
      rv = 0;
      if (ignorpkt) {
	/* skip packet's contents */
	for(l = 0; l < msglen; l++)
	  rv += serial_recv(&pgm->fd, &c, 1);
      } else {
	rv += serial_recv(&pgm->fd, buf + 8, msglen);
      }
      if (rv != 0) {
	timedout:
	/* timeout in receive */
        pmsg_notice2("jtagmkII_recv(): timeout receiving packet\n");
	free(buf);
	return -1;
      }
    } else {
      if (serial_recv(&pgm->fd, &c, 1) != 0)
	goto timedout;
    }

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
	    pmsg_warning("msglen %lu exceeds max message size %u, ignoring message\n",
              msglen, MAX_MESSAGE);
	    state = sSTART;
	    headeridx = 0;
	  } else if ((buf = malloc(msglen + 10)) == NULL) {
	    pmsg_error("out of memory\n");
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
	    if (verbose >= 9)
	      pmsg_trace2("jtagmkII_recv(): CRC OK");
	    state = sDONE;
	  } else {
	    pmsg_error("wrong checksum\n");
	    free(buf);
	    return -4;
	  }
	} else
	  state++;
        break;
      default:
        pmsg_error("unknown state\n");
	free(buf);
        return -5;
     }

     tnow = avr_timestamp();
     if (tnow - tstart > timeoutval) {
       pmsg_error("timeout\n");
       free(buf);
       return -1;
     }

  }
  msg_debug("\n");

  *seqno = r_seqno;
  *msg = buf;

  return msglen;
}

int jtagmkII_recv(const PROGRAMMER *pgm, unsigned char **msg) {
  unsigned short r_seqno;
  int rv;

  for (;;) {
    if ((rv = jtagmkII_recv_frame(pgm, msg, &r_seqno)) <= 0)
      return rv;
    pmsg_debug("jtagmkII_recv(): got message seqno %d (command_sequence == %d)\n",
      r_seqno, PDATA(pgm)->command_sequence);
    if (r_seqno == PDATA(pgm)->command_sequence) {
      if (++(PDATA(pgm)->command_sequence) == 0xffff)
	PDATA(pgm)->command_sequence = 0;
      /*
       * We move the payload to the beginning of the buffer, to make
       * the job easier for the caller.  We have to return the
       * original pointer though, as the caller must free() it.
       */
      memmove(*msg, *msg + 8, rv);

      if(verbose > 3)
        trace_buffer(__func__, *msg, rv);

      return rv;
    }
    if (r_seqno == 0xffff) {
      pmsg_debug("jtagmkII_recv(): got asynchronous event\n");
    } else {
      pmsg_notice2("jtagmkII_recv(): got wrong sequence number, %u != %u\n",
        r_seqno, PDATA(pgm)->command_sequence);
    }
    free(*msg);
  }
}


int jtagmkII_getsync(const PROGRAMMER *pgm, int mode) {
  int tries;
#define MAXTRIES 10
  unsigned char buf[3], *resp, c = 0xff;
  int status;
  unsigned int fwver, hwver;
  int is_dragon;

  pmsg_debug("jtagmkII_getsync()\n");

  if (str_starts(pgm->type, "JTAG")) {
    is_dragon = 0;
  } else if (str_starts(pgm->type, "DRAGON")) {
    is_dragon = 1;
  } else {
    pmsg_error("programmer is neither JTAG ICE mkII nor AVR Dragon\n");
    return -1;
  }
  for (tries = 0; tries < MAXTRIES; tries++) {

    /* Get the sign-on information. */
    buf[0] = CMND_GET_SIGN_ON;
    pmsg_notice2("jtagmkII_getsync() attempt %d of %d: sending sign-on command: ",
      tries + 1, MAXTRIES);
    jtagmkII_send(pgm, buf, 1);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
	    pmsg_warning("attempt %d of %d: sign-on command: status %d\n",
              tries + 1, MAXTRIES, status);
    } else if (verbose >= 3) {
      msg_debug("\n");
      jtagmkII_prmsg(pgm, resp, status);
    } else
      msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);

    if (status > 0) {
      if ((c = resp[0]) == RSP_SIGN_ON) {
	fwver = ((unsigned)resp[8] << 8) | (unsigned)resp[7];
        PDATA(pgm)->fwver = fwver;
	hwver = (unsigned)resp[9];
	memcpy(PDATA(pgm)->serno, resp + 10, 6);
	if (status > 17) {
	  imsg_notice2("JTAG ICE mkII sign-on message:\n");
	  imsg_notice2("Communications protocol version: %u\n",
		  (unsigned)resp[1]);
	  imsg_notice2("M_MCU:\n");
	  imsg_notice2("  boot-loader FW version:        %u\n",
		  (unsigned)resp[2]);
	  imsg_notice2("  firmware version:              %u.%02u\n",
		  (unsigned)resp[4], (unsigned)resp[3]);
	  imsg_notice2("  hardware version:              %u\n",
		  (unsigned)resp[5]);
	  imsg_notice2("S_MCU:\n");
	  imsg_notice2("  boot-loader FW version:        %u\n",
		  (unsigned)resp[6]);
	  imsg_notice2("  firmware version:              %u.%02u\n",
		  (unsigned)resp[8], (unsigned)resp[7]);
	  imsg_notice2("  hardware version:              %u\n",
		  (unsigned)resp[9]);
	  imsg_notice2("Serial number:                   "
		  "%02x:%02x:%02x:%02x:%02x:%02x\n",
		  PDATA(pgm)->serno[0], PDATA(pgm)->serno[1], PDATA(pgm)->serno[2], PDATA(pgm)->serno[3], PDATA(pgm)->serno[4], PDATA(pgm)->serno[5]);
	  resp[status - 1] = '\0';
	  imsg_notice2("Device ID:                       %s\n",
		  resp + 16);
	}
	free(resp);
	break;
      }
      free(resp);
    }
  }
  if (tries >= MAXTRIES) {
    if (status <= 0)
      pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    else
      pmsg_error("bad response to sign-on command: %s\n", jtagmkII_get_rc(c));
    return -1;
  }

  PDATA(pgm)->device_descriptor_length = sizeof(struct device_descriptor);
  /*
   * There's no official documentation from Atmel about what firmware
   * revision matches what device descriptor length.  The algorithm
   * below has been found empirically.
   */
#define FWVER(maj, min) ((maj << 8) | (min))
  if (!is_dragon && fwver < FWVER(3, 16)) {
    PDATA(pgm)->device_descriptor_length -= 2;
    pmsg_warning("S_MCU firmware version might be too old to work correctly\n");
  } else if (!is_dragon && fwver < FWVER(4, 0)) {
    PDATA(pgm)->device_descriptor_length -= 2;
  }
  if (mode != EMULATOR_MODE_SPI)
    pmsg_notice2("jtagmkII_getsync(): using a %u-byte device descriptor\n",
      (unsigned) PDATA(pgm)->device_descriptor_length);
  if (mode == EMULATOR_MODE_SPI) {
    PDATA(pgm)->device_descriptor_length = 0;
    if (!is_dragon && fwver < FWVER(4, 14)) {
      pmsg_error("ISP functionality requires firmware version >= 4.14\n");
      return -1;
    }
  }
  if (mode == EMULATOR_MODE_PDI || mode == EMULATOR_MODE_JTAG_XMEGA) {
    if (!is_dragon && mode == EMULATOR_MODE_PDI && hwver < 1) {
      pmsg_error("Xmega PDI support requires hardware revision >= 1\n");
      return -1;
    }
    if (!is_dragon && fwver < FWVER(5, 37)) {
      pmsg_error("Xmega support requires firmware version >= 5.37\n");
      return -1;
    }
    if (is_dragon && fwver < FWVER(6, 11)) {
      pmsg_error("Xmega support requires firmware version >= 6.11\n");
      return -1;
    }
  }
#undef FWVER

  if(mode < 0) return 0;  // for AVR32

  tries = 0;
retry:
  /* Turn the ICE into JTAG or ISP mode as requested. */
  buf[0] = mode;
  if (jtagmkII_setparm(pgm, PAR_EMULATOR_MODE, buf) < 0) {
    if (mode == EMULATOR_MODE_SPI) {
      pmsg_warning("ISP activation failed, trying debugWire\n");
      buf[0] = EMULATOR_MODE_DEBUGWIRE;
      if (jtagmkII_setparm(pgm, PAR_EMULATOR_MODE, buf) < 0)
	return -1;
      else {
	/*
	 * We are supposed to send a CMND_RESET with the
	 * MONCOM_DISABLE flag set right now, and then
	 * restart from scratch.
	 *
	 * As this will make the ICE sign off from USB, so
	 * we risk losing our USB connection, it's easier
	 * to instruct the user to restart AVRDUDE rather
	 * than trying to cope with all this inside the
	 * program.
	 */
	(void)jtagmkII_reset(pgm, 0x04);
	if (tries++ > 3) {
	    pmsg_error("unable to return from debugWIRE to ISP\n");
	    return -1;
	}
	pmsg_warning("target prepared for ISP, signed off\n");
        imsg_warning("now retrying without power-cycling the target\n");
        goto retry;
      }
    } else {
      return -1;
    }
  }

  /* GET SYNC forces the target into STOPPED mode */
  buf[0] = CMND_GET_SYNC;
  pmsg_notice2("jtagmkII_getsync(): sending get sync command: ");
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to set parameter command: %s\n", jtagmkII_get_rc(c));
    return -1;
  }

  return 0;
}

/*
 * issue the 'chip erase' command to the AVR device
 */
static int jtagmkII_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  int status, len;
  unsigned char buf[6], *resp, c;

  if (p->prog_modes & (PM_PDI | PM_UPDI)) {
    buf[0] = CMND_XMEGA_ERASE;
    buf[1] = XMEGA_ERASE_CHIP;
    memset(buf + 2, 0, 4);      /* address of area to be erased */
    len = 6;
  } else {
    buf[0] = CMND_CHIP_ERASE;
    len = 1;
  }
  pmsg_notice2("jtagmkII_chip_erase(): sending %schip erase command: ",
    p->prog_modes & (PM_PDI | PM_UPDI)? "Xmega ": "");
  jtagmkII_send(pgm, buf, len);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to chip erase command: %s\n", jtagmkII_get_rc(c));
    return -1;
  }

  if (!(p->prog_modes & (PM_PDI | PM_UPDI)))
      pgm->initialize(pgm, p);

  PDATA(pgm)->recently_written = 1;
  return 0;
}

/*
 * There is no chip erase functionality in debugWire mode.
 */
static int jtagmkII_chip_erase_dw(const PROGRAMMER *pgm_unused, const AVRPART *p_unused) {

  pmsg_info("chip erase not supported in debugWire mode\n");

  return 0;
}

static void jtagmkII_set_devdescr(const PROGRAMMER *pgm, const AVRPART *p) {
  int status;
  unsigned char *resp, c;
  LNODEID ln;
  AVRMEM *flm;
  struct {
    unsigned char cmd;
    struct device_descriptor dd;
  } sendbuf;

  memset(&sendbuf, 0, sizeof sendbuf);
  sendbuf.cmd = CMND_SET_DEVICE_DESCRIPTOR;
  sendbuf.dd.ucSPMCRAddress = p->spmcr;
  sendbuf.dd.ucRAMPZAddress = p->rampz;
  sendbuf.dd.ucIDRAddress = p->idr;
  if((flm = avr_locate_flash(p)) && p->boot_section_size > 0) {
    unsigned int sbls = (flm->size - p->boot_section_size)/2; // Words
    sendbuf.dd.uiStartSmallestBootLoaderSection[0] = sbls;
    sendbuf.dd.uiStartSmallestBootLoaderSection[1] = sbls>>8;
  }
  u16_to_b2(sendbuf.dd.EECRAddress, p->eecr? p->eecr: 0x3f); // Unset eecr means 0x3f
  sendbuf.dd.ucAllowFullPageBitstream =
    (p->flags & AVRPART_ALLOWFULLPAGEBITSTREAM) != 0;
  sendbuf.dd.EnablePageProgramming =
    (p->flags & AVRPART_ENABLEPAGEPROGRAMMING) != 0;
  for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
    AVRMEM *m = ldata(ln);
    if (mem_is_flash(m)) {
      if (m->page_size > 256)
        PDATA(pgm)->flash_pagesize = 256;
      else
        PDATA(pgm)->flash_pagesize = m->page_size;
      u32_to_b4(sendbuf.dd.ulFlashSize, m->size);
      u16_to_b2(sendbuf.dd.uiFlashPageSize, m->page_size);
      u16_to_b2(sendbuf.dd.uiFlashpages, m->size / m->page_size);
      if (p->prog_modes & PM_debugWIRE) {
	memcpy(sendbuf.dd.ucFlashInst, p->flash_instr, FLASH_INSTR_SIZE);
	memcpy(sendbuf.dd.ucEepromInst, p->eeprom_instr, EEPROM_INSTR_SIZE);
      }
    } else if (mem_is_eeprom(m)) {
      sendbuf.dd.ucEepromPageSize = PDATA(pgm)->eeprom_pagesize = m->page_size;
    }
  }
  sendbuf.dd.ucCacheType =
    p->prog_modes & (PM_PDI | PM_UPDI)? 0x02 /* ATxmega */: 0x00;

  pmsg_notice2("jtagmkII_set_devdescr(): "
    "Sending set device descriptor command: ");
  jtagmkII_send(pgm, (unsigned char *)&sendbuf,
		PDATA(pgm)->device_descriptor_length + sizeof(unsigned char));

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to set device descriptor command: %s\n", jtagmkII_get_rc(c));
  }
}

static void jtagmkII_set_xmega_params(const PROGRAMMER *pgm, const AVRPART *p) {
  int status, fuseinit = 0;
  unsigned char *resp, c;
  LNODEID ln;
  AVRMEM * m;
  struct {
    unsigned char cmd;
    struct xmega_device_desc dd;
  } sendbuf;

  memset(&sendbuf, 0, sizeof sendbuf);
  sendbuf.cmd = CMND_SET_XMEGA_PARAMS;
  u16_to_b2(sendbuf.dd.whatever, 0x0002);
  sendbuf.dd.datalen = 47;
  u16_to_b2(sendbuf.dd.nvm_base_addr, p->nvm_base);
  u16_to_b2(sendbuf.dd.mcu_base_addr, p->mcu_base);

  for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
    m = ldata(ln);
    if (mem_is_flash(m)) {
      if (m->page_size > 256)
        PDATA(pgm)->flash_pagesize = 256;
      else
        PDATA(pgm)->flash_pagesize = m->page_size;
      u16_to_b2(sendbuf.dd.flash_page_size, m->page_size);
    } else if (mem_is_eeprom(m)) {
      sendbuf.dd.eeprom_page_size = m->page_size;
      u16_to_b2(sendbuf.dd.eeprom_size, m->size);
      u32_to_b4(sendbuf.dd.nvm_eeprom_offset, m->offset);
    } else if (mem_is_application(m)) {
      u32_to_b4(sendbuf.dd.app_size, m->size);
      u32_to_b4(sendbuf.dd.nvm_app_offset, m->offset);
    } else if (mem_is_boot(m)) {
      u16_to_b2(sendbuf.dd.boot_size, m->size);
      u32_to_b4(sendbuf.dd.nvm_boot_offset, m->offset);
    } else if(mem_is_a_fuse(m) && !fuseinit++) { // Any fuse is OK
      u32_to_b4(sendbuf.dd.nvm_fuse_offset, m->offset & ~15);
    } else if (mem_is_lock(m)) {
      u32_to_b4(sendbuf.dd.nvm_lock_offset, m->offset);
    } else if (mem_is_userrow(m)) {
      u32_to_b4(sendbuf.dd.nvm_user_sig_offset, m->offset);
    } else if (mem_is_sigrow(m)) {
      u32_to_b4(sendbuf.dd.nvm_prod_sig_offset, m->offset);
      pmsg_notice2("prod_sig_offset addr 0x%05x\n", m->offset);
    }
  }
  if(p->prog_modes & (PM_PDI | PM_UPDI))
    u32_to_b4(sendbuf.dd.nvm_data_offset, DATA_OFFSET);

  pmsg_notice2("%s() sending set Xmega params command: ", __func__);
  jtagmkII_send(pgm, (unsigned char *)&sendbuf, sizeof sendbuf);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to set device descriptor command: %s\n", jtagmkII_get_rc(c));
  }
}

/*
 * Reset the target.
 */
static int jtagmkII_reset(const PROGRAMMER *pgm, unsigned char flags) {
  int status;
  unsigned char buf[2], *resp, c;

  /*
   * In debugWire mode, don't reset.  Do a forced stop, and tell the
   * ICE to stop any timers, too.
   */
  if (pgm->flag & PGM_FL_IS_DW) {
    unsigned char parm[] = { 0 };

    (void)jtagmkII_setparm(pgm, PAR_TIMERS_RUNNING, parm);
  }

  buf[0] = (pgm->flag & PGM_FL_IS_DW)? CMND_FORCED_STOP: CMND_RESET;
  buf[1] = (pgm->flag & PGM_FL_IS_DW)? 1: flags;
  pmsg_notice2("jtagmkII_reset(): sending %s command: ",
    (pgm->flag & PGM_FL_IS_DW)? "stop": "reset");
  jtagmkII_send(pgm, buf, 2);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to reset command: %s\n", jtagmkII_get_rc(c));
    return -1;
  }

  return 0;
}

static int jtagmkII_program_enable_INFO(const PROGRAMMER *pgm_unused, const AVRPART *p_unused) {
  return 0;
}

static int jtagmkII_program_enable(const PROGRAMMER *pgm) {
  int status;
  unsigned char buf[1], *resp, c;
  int use_ext_reset;

  if (PDATA(pgm)->prog_enabled)
    return 0;

  for (use_ext_reset = 0; use_ext_reset <= 1; use_ext_reset++) {
    buf[0] = CMND_ENTER_PROGMODE;
    pmsg_notice2("jtagmkII_program_enable(): "
      "Sending enter progmode command: ");
    jtagmkII_send(pgm, buf, 1);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
      msg_notice2("\n");
      pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
      return -1;
    }
    if (verbose >= 3) {
      msg_debug("\n");
      jtagmkII_prmsg(pgm, resp, status);
    } else
      msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
    c = resp[0];
    free(resp);
    if (c != RSP_OK) {
      pmsg_warning("bad response to enter progmode command: %s\n", jtagmkII_get_rc(c));
      if (c == RSP_ILLEGAL_JTAG_ID) {
	if (use_ext_reset == 0) {
	  unsigned char parm[] = { 1};
          pmsg_warning("retrying with external reset applied\n");

	  (void)jtagmkII_setparm(pgm, PAR_EXTERNAL_RESET, parm);
	  continue;
	}

	pmsg_error("JTAGEN fuse disabled?\n");
	return -1;
      }
    }
  }

  PDATA(pgm)->prog_enabled = 1;
  return 0;
}

static int jtagmkII_program_disable(const PROGRAMMER *pgm) {
  int status;
  unsigned char buf[1], *resp, c;

  if (!PDATA(pgm)->prog_enabled)
    return 0;

  buf[0] = CMND_LEAVE_PROGMODE;
  pmsg_notice2("jtagmkII_program_disable(): "
    "Sending leave progmode command: ");
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to leave progmode command: %s\n", jtagmkII_get_rc(c));
    return -1;
  }

  PDATA(pgm)->recently_written  = 0;
  PDATA(pgm)->prog_enabled = 0;
  (void)jtagmkII_reset(pgm, 0x01);

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
  /* Extension to jtagmkII protocol: extra baud rates, standard series. */
  { 153600L, PAR_BAUD_153600 },
  { 230400L, PAR_BAUD_230400 },
  { 460800L, PAR_BAUD_460800 },
  { 921600L, PAR_BAUD_921600 },
  /* Extension to jtagmkII protocol: extra baud rates, binary series. */
  { 128000L, PAR_BAUD_128000 },
  { 256000L, PAR_BAUD_256000 },
  { 512000L, PAR_BAUD_512000 },
  { 1024000L, PAR_BAUD_1024000 },
  /* Extension to jtagmkII protocol: extra baud rates, decimal series. */
  { 150000L, PAR_BAUD_150000 },
  { 200000L, PAR_BAUD_200000 },
  { 250000L, PAR_BAUD_250000 },
  { 300000L, PAR_BAUD_300000 },
  { 400000L, PAR_BAUD_400000 },
  { 500000L, PAR_BAUD_500000 },
  { 600000L, PAR_BAUD_600000 },
  { 666666L, PAR_BAUD_666666 },
  { 1000000L, PAR_BAUD_1000000 },
  { 1500000L, PAR_BAUD_1500000 },
  { 2000000L, PAR_BAUD_2000000 },
  { 3000000L, PAR_BAUD_3000000 },
};

  for (size_t i = 0; i < sizeof baudtab/sizeof*baudtab; i++)
    if (baud == baudtab[i].baud)
      return baudtab[i].val;

  return 0;
}

/*
 * initialize the AVR device and prepare it to accept commands
 */
static int jtagmkII_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char b;
  int ok;
  const char *ifname;

  if (PDATA(pgm)->rts_mode != RTS_MODE_DEFAULT) {
    pmsg_info("forcing serial DTR/RTS handshake lines %s\n",
      PDATA(pgm)->rts_mode == RTS_MODE_LOW ? "LOW" : "HIGH");
  }

  /* Abort and print error if programmer does not support the target microcontroller */
  if((str_starts(pgm->type, "JTAGMKII_UPDI") && !(p->prog_modes & PM_UPDI)) ||
      (str_starts(pgmid, "jtagmkII") && (p->prog_modes & PM_UPDI))) {
    msg_error("programmer %s does not support target %s\n\n", pgmid, p->desc);
    return -1;
  }

  ok = 0;
  if (pgm->flag & PGM_FL_IS_DW) {
    ifname = "debugWire";
    if (p->prog_modes & PM_debugWIRE)
      ok = 1;
  } else if (pgm->flag & PGM_FL_IS_PDI) {
    ifname = "PDI";
    if (p->prog_modes & (PM_PDI | PM_UPDI))
      ok = 1;
  } else {
    ifname = "JTAG";
    if (p->prog_modes & (PM_JTAG | PM_JTAGmkI | PM_XMEGAJTAG | PM_AVR32JTAG))
      ok = 1;
  }

  if (!ok) {
    pmsg_error("part %s has no %s interface\n", p->desc, ifname);
    return -1;
  }

  if ((serdev->flags & SERDEV_FL_CANSETSPEED) && pgm->baudrate && pgm->baudrate != 19200) {
    if ((b = jtagmkII_get_baud(pgm->baudrate)) == 0) {
      pmsg_error("unsupported baudrate %d\n", pgm->baudrate);
    } else {
      pmsg_notice2("jtagmkII_initialize(): "
	"trying to set baudrate to %d\n", pgm->baudrate);
      if (jtagmkII_setparm(pgm, PAR_BAUD_RATE, &b) == 0)
	serial_setparams(&pgm->fd, pgm->baudrate, SERIAL_8N1);
    }
  }
  if ((pgm->flag & PGM_FL_IS_JTAG) && pgm->bitclock != 0.0) {
    pmsg_notice2("jtagmkII_initialize(): "
      "trying to set JTAG clock period to %.1f us\n", pgm->bitclock);
    if (jtagmkII_set_sck_period(pgm, pgm->bitclock) != 0)
      return -1;
  }

  if ((pgm->flag & PGM_FL_IS_JTAG) &&
      jtagmkII_setparm(pgm, PAR_DAISY_CHAIN_INFO, PDATA(pgm)->jtagchain) < 0) {
    pmsg_error("unable to setup JTAG chain\n");
    return -1;
  }

  /*
   * If this is an ATxmega device in JTAG mode, change the emulator
   * mode from JTAG to JTAG_XMEGA.
   */
  if ((pgm->flag & PGM_FL_IS_JTAG) &&
      (p->prog_modes & (PM_PDI | PM_UPDI))) {
    if (jtagmkII_getsync(pgm, EMULATOR_MODE_JTAG_XMEGA) < 0)
      return -1;
  }
  /*
   * Must set the device descriptor before entering programming mode.
   */
  if (PDATA(pgm)->fwver >= 0x700 && (p->prog_modes & (PM_PDI | PM_UPDI)) != 0)
    jtagmkII_set_xmega_params(pgm, p);
  else
    jtagmkII_set_devdescr(pgm, p);

  PDATA(pgm)->boot_start = ULONG_MAX;
  if ((p->prog_modes & (PM_PDI | PM_UPDI))) {
    // Find the border between application and boot area
    AVRMEM *bootmem = avr_locate_boot(p);
    AVRMEM *flashmem = avr_locate_flash(p);
    if (bootmem == NULL || flashmem == NULL) {
      if(str_starts(pgmid, "jtagmkII"))
        pmsg_error("cannot locate flash or boot memories in description\n");
    } else {
      if (PDATA(pgm)->fwver < 0x700) {
        /* V7+ firmware does not need this anymore */
        unsigned char par[4];

        u32_to_b4(par, flashmem->offset);
        (void) jtagmkII_setparm(pgm, PAR_PDI_OFFSET_START, par);
        u32_to_b4(par, bootmem->offset);
        (void) jtagmkII_setparm(pgm, PAR_PDI_OFFSET_END, par);
      }

      PDATA(pgm)->boot_start = bootmem->offset - flashmem->offset;
    }
  }

  free(PDATA(pgm)->flash_pagecache);
  free(PDATA(pgm)->eeprom_pagecache);
  if ((PDATA(pgm)->flash_pagecache = malloc(PDATA(pgm)->flash_pagesize)) == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }
  if ((PDATA(pgm)->eeprom_pagecache = malloc(PDATA(pgm)->eeprom_pagesize)) == NULL) {
    pmsg_error("out of memory\n");
    free(PDATA(pgm)->flash_pagecache);
    return -1;
  }
  PDATA(pgm)->flash_pageaddr = PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;

  if (PDATA(pgm)->fwver >= 0x700 && (p->prog_modes & (PM_PDI | PM_UPDI))) {
    /*
     * Work around for
     * https://savannah.nongnu.org/bugs/index.php?37942
     *
     * Firmware version 7.24 (at least) on the Dragon behaves very
     * strange when it gets a RESET request here.  All subsequent
     * responses are completely off, so the emulator becomes unusable.
     * This appears to be a firmware bug (earlier versions, at least
     * 7.14, didn't experience this), but by omitting the RESET for
     * Xmega devices, we can work around it.
     */
  } else {
    if (jtagmkII_reset(pgm, 0x01) < 0)
      return -1;
  }

  if ((pgm->flag & PGM_FL_IS_JTAG) && !(p->prog_modes & (PM_PDI | PM_UPDI))) {
    int ocden = 0;
    if(avr_get_config_value(pgm, p, "ocden", &ocden) == 0 && ocden) // ocden == 1 means disabled
      pmsg_warning("OCDEN fuse not programmed, single-byte EEPROM updates not possible\n");
  }

  if (pgm->read_chip_rev && p->prog_modes & (PM_PDI | PM_UPDI)) {
    unsigned char chip_rev[AVR_CHIP_REVLEN];
    pgm->read_chip_rev(pgm, p, chip_rev);
    pmsg_notice("silicon revision: %x.%x\n", chip_rev[0] >> 4, chip_rev[0] & 0x0f);
  }

  return 0;
}

static void jtagmkII_disable(const PROGRAMMER *pgm) {

  free(PDATA(pgm)->flash_pagecache);
  PDATA(pgm)->flash_pagecache = NULL;
  free(PDATA(pgm)->eeprom_pagecache);
  PDATA(pgm)->eeprom_pagecache = NULL;

  /*
   * jtagmkII_program_disable() doesn't do anything if the
   * device is currently not in programming mode, so just
   * call it unconditionally here.
   */
  (void)jtagmkII_program_disable(pgm);
}

static void jtagmkII_enable(PROGRAMMER *pgm, const AVRPART *p) {
  // Page erase only useful for classic parts with usersig mem or AVR8X/XMEGAs
  if(!(p->prog_modes & (PM_PDI | PM_UPDI)))
    if(!avr_locate_usersig(p))
      pgm->page_erase = NULL;

  if(pgm->flag & PGM_FL_IS_DW)
    pgm->page_erase = NULL;

  return;
}

static int jtagmkII_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int rv = 0;

  for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if (pgm->flag & PGM_FL_IS_JTAG) {
      if (str_eq(extended_param, "jtagchain=")) {
        unsigned int ub, ua, bb, ba;
        if (sscanf(extended_param, "jtagchain=%u,%u,%u,%u", &ub, &ua, &bb, &ba) != 4) {
          pmsg_error("invalid JTAG chain '%s'\n", extended_param);
          rv = -1;
          continue;
        }
        pmsg_notice2("jtagmkII_parseextparms(): JTAG chain parsed as:\n");
        imsg_notice2("%u units before, %u units after, %u bits before, %u bits after\n",
          ub, ua, bb, ba);
        PDATA(pgm)->jtagchain[0] = ub;
        PDATA(pgm)->jtagchain[1] = ua;
        PDATA(pgm)->jtagchain[2] = bb;
        PDATA(pgm)->jtagchain[3] = ba;

        continue;
      }
    }

    if (pgm->flag & PGM_FL_IS_PDI) {
      char rts_mode[5];
      if (sscanf(extended_param, "rtsdtr=%4s", rts_mode) == 1) {
        if (str_caseeq(rts_mode, "low")) {
          PDATA(pgm)->rts_mode = RTS_MODE_LOW;
        } else if (str_caseeq(rts_mode, "high")) {
          PDATA(pgm)->rts_mode = RTS_MODE_HIGH;
        } else {
          pmsg_error("RTS/DTR mode must be LOW or HIGH\n");
          return -1;
        }
        continue;
      }
    }

    if (str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      if (pgm->flag & PGM_FL_IS_JTAG)
        msg_error("  -xjtagchain=UB,UA,BB,BA Setup the JTAG scan chain order\n");
      if (pgm->flag & PGM_FL_IS_PDI)
        msg_error("  -xrtsdtr=low,high       Force RTS/DTR lines low or high state during programming\n");
      msg_error(  "  -xhelp                  Show this help menu and exit\n");
      exit(0);
    }

    pmsg_error("invalid extended parameter '%s'\n", extended_param);
    rv = -1;
  }

  return rv;
}


static int jtagmkII_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pmsg_notice2("jtagmkII_open()\n");

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pinfo.serialinfo.baud = 19200;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (str_starts(port, "usb")) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.flags = 0;
    pinfo.usbinfo.pid = USB_DEVICE_JTAGICEMKII;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_MKII;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_MKII;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_MKII;
    pgm->fd.usb.eep = 0;           /* no seperate EP for events */
#else
    msg_error("avrdude was compiled without usb support\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_JTAG) < 0)
    return -1;

  return 0;
}

static int jtagmkII_open_dw(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pmsg_notice2("jtagmkII_open_dw()\n");

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pinfo.serialinfo.baud = 19200;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (str_starts(port, "usb")) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.flags = 0;
    pinfo.usbinfo.pid = USB_DEVICE_JTAGICEMKII;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_MKII;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_MKII;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_MKII;
    pgm->fd.usb.eep = 0;           /* no seperate EP for events */
#else
    msg_error("avrdude was compiled without usb support\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_DEBUGWIRE) < 0)
    return -1;

  return 0;
}

static int jtagmkII_open_pdi(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pmsg_notice2("jtagmkII_open_pdi()\n");

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pinfo.serialinfo.baud = 19200;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (str_starts(port, "usb")) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.flags = 0;
    pinfo.usbinfo.pid = USB_DEVICE_JTAGICEMKII;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_MKII;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_MKII;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_MKII;
    pgm->fd.usb.eep = 0;           /* no seperate EP for events */
#else
    msg_error("avrdude was compiled without usb support\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  /* Set RTS/DTR high or low based on the user specified rts_mode */
  if (PDATA(pgm)->rts_mode != RTS_MODE_DEFAULT) {
    serial_set_dtr_rts(&pgm->fd, 0);
    serial_set_dtr_rts(&pgm->fd, PDATA(pgm)->rts_mode == RTS_MODE_LOW ? 1 : 0);
  }

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_PDI) < 0)
    return -1;

  return 0;
}


static int jtagmkII_dragon_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pmsg_notice2("jtagmkII_dragon_open()\n");

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pinfo.serialinfo.baud = 19200;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (str_starts(port, "usb")) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.flags = 0;
    pinfo.usbinfo.pid = USB_DEVICE_AVRDRAGON;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_MKII;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_MKII;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_MKII;
    pgm->fd.usb.eep = 0;           /* no seperate EP for events */
#else
    msg_error("avrdude was compiled without usb support\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_JTAG) < 0)
    return -1;

  return 0;
}


static int jtagmkII_dragon_open_dw(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pmsg_notice2("jtagmkII_dragon_open_dw()\n");

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pinfo.serialinfo.baud = 19200;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (str_starts(port, "usb")) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.flags = 0;
    pinfo.usbinfo.pid = USB_DEVICE_AVRDRAGON;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_MKII;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_MKII;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_MKII;
    pgm->fd.usb.eep = 0;           /* no seperate EP for events */
#else
    msg_error("avrdude was compiled without usb support\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_DEBUGWIRE) < 0)
    return -1;

  return 0;
}


static int jtagmkII_dragon_open_pdi(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  pmsg_notice2("jtagmkII_dragon_open_pdi()\n");

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pinfo.serialinfo.baud = 19200;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (str_starts(port, "usb")) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.flags = 0;
    pinfo.usbinfo.pid = USB_DEVICE_AVRDRAGON;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_MKII;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_MKII;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_MKII;
    pgm->fd.usb.eep = 0;           /* no seperate EP for events */
#else
    msg_error("avrdude was compiled without usb support\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  if (jtagmkII_getsync(pgm, EMULATOR_MODE_PDI) < 0)
    return -1;

  return 0;
}


void jtagmkII_close(PROGRAMMER * pgm)
{
  int status;
  unsigned char buf[1], *resp, c;

  pmsg_notice2("jtagmkII_close()\n");

  if (pgm->flag & (PGM_FL_IS_PDI | PGM_FL_IS_JTAG)) {
    /* When in PDI or JTAG mode, restart target. */
    buf[0] = CMND_GO;
    pmsg_notice2("jtagmkII_close(): sending GO command: ");
    jtagmkII_send(pgm, buf, 1);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
      msg_notice2("\n");
      pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    } else {
      if (verbose >= 3) {
	msg_debug("\n");
	jtagmkII_prmsg(pgm, resp, status);
      } else
	msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
      c = resp[0];
      free(resp);
      if (c != RSP_OK) {
	pmsg_error("bad response to GO command: %s\n", jtagmkII_get_rc(c));
      }
    }
  }

  buf[0] = CMND_SIGN_OFF;
  pmsg_notice2("jtagmkII_close(): sending sign-off command: ");
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to sign-off command: %s\n", jtagmkII_get_rc(c));
  }

  if (PDATA(pgm)->rts_mode != RTS_MODE_DEFAULT) {
    pmsg_info("releasing DTR/RTS handshake lines\n");
    serial_set_dtr_rts(&pgm->fd, 0);
  }

  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;

  /* The AVR Dragon and the Arduino Nano Every needs a delay
   * after a programming session has ended before Avrdude can
   * communicate with the programmer again.
   */
  if (str_casestarts(pgmid, "dragon"))
    usleep(1000*1000*1.5);
  else if (str_caseeq(pgmid, "nanoevery"))
    usleep(1000*1000*0.5);
}

static int jtagmkII_page_erase(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                               unsigned int addr)
{
  unsigned char cmd[6];
  unsigned char *resp;
  int status, tries;
  long otimeout = serial_recv_timeout;

  pmsg_notice2("jtagmkII_page_erase(.., %s, 0x%x)\n", m->desc, addr);

  if (!(p->prog_modes & (PM_PDI | PM_UPDI)) && !mem_is_userrow(m)) {
    pmsg_error("page erase only available for AVR8X/XMEGAs or classic-part usersig mem\n");
    return -1;
  }
  if ((pgm->flag & PGM_FL_IS_DW)) {
    pmsg_error("not applicable to debugWIRE\n");
    return -1;
  }

  if (jtagmkII_program_enable(pgm) < 0)
    return -1;

  cmd[0] = CMND_XMEGA_ERASE;
  if (mem_is_flash(m)) {
    if (jtagmkII_mtype(pgm, p, addr) == MTYPE_FLASH)
      cmd[1] = XMEGA_ERASE_APP_PAGE;
    else
      cmd[1] = XMEGA_ERASE_BOOT_PAGE;
  } else if (mem_is_eeprom(m)) {
    cmd[1] = XMEGA_ERASE_EEPROM_PAGE;
  } else if (mem_is_userrow(m)) {
    cmd[1] = XMEGA_ERASE_USERSIG;
  } else if (mem_is_boot(m)) {
    cmd[1] = XMEGA_ERASE_BOOT_PAGE;
  } else {
    cmd[1] = XMEGA_ERASE_APP_PAGE;
  }
  serial_recv_timeout = 100;

  /*
   * Don't use jtagmkII_memaddr() here.  While with all other
   * commands, firmware 7+ doesn't require the NVM offsets being
   * applied, the erase page commands make an exception, and do
   * require the NVM offsets as part of the (page) address.
   */
  u32_to_b4(cmd + 2, addr + m->offset);

  tries = 0;

  retry:
    pmsg_notice2("jtagmkII_page_erase(): "
      "Sending Xmega erase command: ");
  jtagmkII_send(pgm, cmd, sizeof cmd);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_warning("timeout/error communicating with programmer (status %d)\n", status);
    if (tries++ < 4) {
      serial_recv_timeout *= 2;
      goto retry;
    }
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    serial_recv_timeout = otimeout;
    return -1;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  if (resp[0] != RSP_OK) {
    pmsg_error("bad response to xmega erase command: %s\n", jtagmkII_get_rc(resp[0]));
    free(resp);
    serial_recv_timeout = otimeout;
    return -1;
  }
  free(resp);

  serial_recv_timeout = otimeout;

  return 0;
}

static int jtagmkII_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes)
{
  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char *cmd;
  unsigned char *resp;
  int status, tries, dynamic_mtype = 0;
  long otimeout = serial_recv_timeout;

  pmsg_notice2("jtagmkII_paged_write(.., %s, %d, %d)\n", m->desc, page_size, n_bytes);

  if (!(pgm->flag & PGM_FL_IS_DW) && jtagmkII_program_enable(pgm) < 0)
    return -1;

  if (page_size == 0) page_size = 256;
  else if (page_size > 256) page_size = 256;

  if ((cmd = malloc(page_size + 10)) == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }

  cmd[0] = CMND_WRITE_MEMORY;
  if (mem_is_flash(m)) {
    PDATA(pgm)->flash_pageaddr = (unsigned long)-1L;
    cmd[1] = jtagmkII_mtype(pgm, p, addr);
    if (p->prog_modes & (PM_PDI | PM_UPDI)) // Dynamically decide between flash/boot mtype
      dynamic_mtype = 1;
  } else if (mem_is_eeprom(m)) {
    if (pgm->flag & PGM_FL_IS_DW) {
      /*
       * jtagmkII_paged_write() to EEPROM attempted while in
       * DW mode.  Use jtagmkII_write_byte() instead.
       */
      for (; addr < maxaddr; addr++) {
	status = jtagmkII_write_byte(pgm, p, m, addr, m->buf[addr]);
	if (status < 0) {
	  free(cmd);
	  return -1;
	}
      }
      free(cmd);
      return n_bytes;
    }
    cmd[1] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_EEPROM_XMEGA: MTYPE_EEPROM_PAGE;
    PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;
  } else if (mem_is_userrow(m)) {
    cmd[1] = MTYPE_USERSIG;
  } else if (mem_is_boot(m)) {
    cmd[1] = MTYPE_BOOT_FLASH;
  } else if (p->prog_modes & (PM_PDI | PM_UPDI)) {
    cmd[1] = MTYPE_FLASH;
  } else {
    cmd[1] = MTYPE_SPM;
  }
  serial_recv_timeout = 200;
  for (; addr < maxaddr; addr += page_size) {
    if ((maxaddr - addr) < page_size)
      block_size = maxaddr - addr;
    else
      block_size = page_size;
    pmsg_debug("jtagmkII_paged_write(): "
      "block_size at addr %d is %d\n", addr, block_size);

    if (dynamic_mtype)
      cmd[1] = jtagmkII_mtype(pgm, p, addr);

    u32_to_b4(cmd + 2, page_size);
    u32_to_b4(cmd + 6, jtagmkII_memaddr(pgm, p, m, addr));

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
      pmsg_notice2("jtagmkII_paged_write(): "
        "Sending write memory command: ");
    jtagmkII_send(pgm, cmd, page_size + 10);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
      msg_notice2("\n");
      pmsg_warning("timeout/error communicating with programmer (status %d)\n", status);
      if (tries++ < 4) {
	serial_recv_timeout *= 2;
	goto retry;
      }
      pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
      free(cmd);
      serial_recv_timeout = otimeout;
      return -1;
    }
    if (verbose >= 3) {
      msg_debug("\n");
      jtagmkII_prmsg(pgm, resp, status);
    } else
      msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
    if (resp[0] != RSP_OK) {
      pmsg_error("bad response to write memory command: %s\n", jtagmkII_get_rc(resp[0]));
      free(resp);
      free(cmd);
      serial_recv_timeout = otimeout;
      return -1;
    }
    free(resp);
  }

  free(cmd);
  serial_recv_timeout = otimeout;

  PDATA(pgm)->recently_written = 1;
  return n_bytes;
}

static int jtagmkII_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                               unsigned int page_size,
                               unsigned int addr, unsigned int n_bytes)
{
  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char cmd[10];
  unsigned char *resp;
  int status, tries, dynamic_mtype = 0;
  long otimeout = serial_recv_timeout;

  pmsg_notice2("jtagmkII_paged_load(.., %s, %d, %d)\n", m->desc, page_size, n_bytes);

  if (!(pgm->flag & PGM_FL_IS_DW) && jtagmkII_program_enable(pgm) < 0)
    return -1;

  page_size = m->readsize;

  cmd[0] = CMND_READ_MEMORY;
  if (mem_is_flash(m)) {
    cmd[1] = jtagmkII_mtype(pgm, p, addr);
    if (p->prog_modes & (PM_PDI | PM_UPDI)) // Dynamically decide between flash/boot mtype
      dynamic_mtype = 1;
  } else if (mem_is_eeprom(m)) {
    cmd[1] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_EEPROM: MTYPE_EEPROM_PAGE;
    if (pgm->flag & PGM_FL_IS_DW)
      return -1;
  } else if (mem_is_sigrow(m)) {
    cmd[1] = MTYPE_PRODSIG;
  } else if (mem_is_userrow(m)) {
    cmd[1] = MTYPE_USERSIG;
  } else if (mem_is_boot(m)) {
    cmd[1] = MTYPE_BOOT_FLASH;
  } else if (p->prog_modes & (PM_PDI | PM_UPDI)) {
    cmd[1] = MTYPE_FLASH;
  } else {
    cmd[1] = MTYPE_SPM;
  }
  serial_recv_timeout = 100;
  for (; addr < maxaddr; addr += page_size) {
    if ((maxaddr - addr) < page_size)
      block_size = maxaddr - addr;
    else
      block_size = page_size;
    pmsg_debug("jtagmkII_paged_load(): "
      "block_size at addr %d is %d\n", addr, block_size);

    if (dynamic_mtype)
      cmd[1] = jtagmkII_mtype(pgm, p, addr);

    u32_to_b4(cmd + 2, block_size);
    u32_to_b4(cmd + 6, jtagmkII_memaddr(pgm, p, m, addr));

    tries = 0;

  retry:
    pmsg_notice2("jtagmkII_paged_load(): sending read memory command: ");
    jtagmkII_send(pgm, cmd, 10);

    status = jtagmkII_recv(pgm, &resp);
    if (status <= 0) {
      msg_notice2("\n");
      pmsg_warning("timeout/error communicating with programmer (status %d)\n", status);
      if (tries++ < 4) {
	serial_recv_timeout *= 2;
	goto retry;
      }
      pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
      serial_recv_timeout = otimeout;
      return -1;
    }
    if (verbose >= 3) {
      msg_debug("\n");
      jtagmkII_prmsg(pgm, resp, status);
    } else
      msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
    if (resp[0] != RSP_MEMORY) {
      pmsg_error("bad response to read memory command: %s\n", jtagmkII_get_rc(resp[0]));
      free(resp);
      serial_recv_timeout = otimeout;
      return -1;
    }
    memcpy(m->buf + addr, resp + 1, status-1);
    free(resp);
  }
  serial_recv_timeout = otimeout;

  PDATA(pgm)->recently_written = 0;
  return n_bytes;
}

static int jtagmkII_read_chip_rev(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev) {
  // XMEGA using JTAG or PDI, tinyAVR0/1/2, megaAVR0, AVR-Dx, AVR-Ex using UPDI
  if(p->prog_modes & (PM_PDI | PM_UPDI)) {
    AVRMEM *m = avr_locate_io(p);
    if(!m) {
      pmsg_error("cannot locate io memory; is avrdude.conf up to date?\n");
      return -1;
    }
    int status = pgm->read_byte(pgm, p, m,
        p->prog_modes & PM_PDI? p->mcu_base+3 :p->syscfg_base+1, chip_rev);
    if (status < 0)
      return status;
  } else {
    pmsg_error("target does not have a chip revision that can be read\n");
    return -1;
  }

  pmsg_debug("jtagmkII_read_chip_rev(): received chip silicon revision: 0x%02x\n", *chip_rev);
  return 0;
}

static int jtagmkII_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
			      unsigned long addr, unsigned char * value)
{
  unsigned char cmd[10];
  unsigned char *resp = NULL, *cache_ptr = NULL;
  int status, tries, unsupp;
  unsigned long paddr = 0UL, *paddr_ptr = NULL;
  unsigned int pagesize = 0;

  pmsg_notice2("jtagmkII_read_byte(.., %s, 0x%lx, ...)\n", mem->desc, addr);

  if (!(pgm->flag & PGM_FL_IS_DW) && jtagmkII_program_enable(pgm) < 0)
    return -1;

  cmd[0] = CMND_READ_MEMORY;
  unsupp = 0;

  addr += mem->offset;
  cmd[1] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_FLASH: MTYPE_FLASH_PAGE;
  if (mem_is_in_flash(mem)) {
    pagesize = PDATA(pgm)->flash_pagesize;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &PDATA(pgm)->flash_pageaddr;
    cache_ptr = PDATA(pgm)->flash_pagecache;
  } else if (mem_is_eeprom(mem)) {
    if ( (pgm->flag & PGM_FL_IS_DW) || (p->prog_modes & (PM_PDI | PM_UPDI)) ) {
      /* debugWire cannot use page access for EEPROM */
      cmd[1] = MTYPE_EEPROM;
    } else {
      cmd[1] = MTYPE_EEPROM_PAGE;
      pagesize = mem->page_size;
      paddr = addr & ~(pagesize - 1);
      paddr_ptr = &PDATA(pgm)->eeprom_pageaddr;
      cache_ptr = PDATA(pgm)->eeprom_pagecache;
    }
  } else if(mem_is_a_fuse(mem) || mem_is_fuses(mem)) {
    cmd[1] = MTYPE_FUSE_BITS;
    if(!(p->prog_modes & (PM_PDI | PM_UPDI)) && mem_is_a_fuse(mem))
      addr = mem_fuse_offset(mem);
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_lock(mem)) {
    cmd[1] = MTYPE_LOCK_BITS;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_userrow(mem)) {
    cmd[1] = MTYPE_USERSIG;
  } else if (mem_is_sigrow(mem)) {
    if (p->prog_modes & (PM_PDI | PM_UPDI)) {
      cmd[1] = MTYPE_PRODSIG;
      pmsg_notice2("is_sigrow addr 0x%05lx\n", addr);
    } else {
      cmd[1] = addr&1? MTYPE_OSCCAL_BYTE: MTYPE_SIGN_JTAG;
      addr /= 2;
    }
  } else if (mem_is_calibration(mem)) {
    cmd[1] = MTYPE_OSCCAL_BYTE;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_signature(mem)) {
    cmd[1] = MTYPE_SIGN_JTAG;

    if (pgm->flag & PGM_FL_IS_DW) {
      /*
       * In debugWire mode, there is no accessible memory area to read
       * the signature from, but the essential two bytes can be read
       * as a parameter from the ICE.
       */
      unsigned char parm[4];

      switch (addr) {
        case 0:
          *value = 0x1E;		/* Atmel vendor ID */
          break;

        case 1:
        case 2:
          if (jtagmkII_getparm(pgm, PAR_TARGET_SIGNATURE, parm) < 0)
            return -1;
          *value = parm[2 - addr];
          break;

        default:
          pmsg_error("illegal address %lu for signature memory\n", addr);
          return -1;
        }
      return 0;
    }
  } else if ((p->prog_modes & (PM_PDI | PM_UPDI)) && mem_is_in_sigrow(mem)) {
    cmd[1] = MTYPE_PRODSIG;
    pmsg_notice2("in_sigrow addr 0x%05lx\n", addr);
  } else if (mem_is_io(mem) || mem_is_sram(mem)) {
    cmd[1] = MTYPE_FLASH;
    addr += avr_data_offset(p);
  } else {
    pmsg_error("unknown memory %s\n", mem->desc);
    return -1;
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
    u32_to_b4(cmd + 2, pagesize);
    u32_to_b4(cmd + 6, paddr);
  } else {
    u32_to_b4(cmd + 2, 1);
    u32_to_b4(cmd + 6, addr);
  }

  tries = 0;
retry:
  pmsg_notice2("jtagmkII_read_byte(): sending read memory command: ");
  jtagmkII_send(pgm, cmd, 10);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_warning("timeout/error communicating with programmer (status %d)\n", status);
    if (tries++ < 3)
      goto retry;
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    if (status < 0)
      resp = 0;
    goto fail;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  if (resp[0] != RSP_MEMORY) {
    pmsg_error("bad response to read memory command: %s\n", jtagmkII_get_rc(resp[0]));
    goto fail;
  }

  if (pagesize) {
    *paddr_ptr = paddr;
    memcpy(cache_ptr, resp + 1, pagesize);
    *value = cache_ptr[addr & (pagesize - 1)];
  } else
    *value = resp[1];

  free(resp);
  PDATA(pgm)->recently_written = 0;
  return 0;

fail:
  free(resp);
  return -1;
}

static int jtagmkII_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
			       unsigned long addr, unsigned char data)
{
  unsigned char cmd[12];
  unsigned char *resp = NULL, writedata, writedata2 = 0xFF;
  int status, tries, need_progmode = 1, unsupp = 0, writesize = 1;

  pmsg_notice2("jtagmkII_write_byte(.., %s, 0x%lx, ...)\n", mem->desc, addr);

  addr += mem->offset;

  writedata = data;
  cmd[0] = CMND_WRITE_MEMORY;
  cmd[1] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_FLASH: MTYPE_SPM;
  if (mem_is_flash(mem)) {
     if ((addr & 1) == 1) {
       /* odd address = high byte */
       writedata = 0xFF;	/* don't modify the low byte */
       writedata2 = data;
       addr &= ~1L;
     }
     writesize = 2;
     if(str_eq(p->family_id, "megaAVR") || str_eq(p->family_id, "tinyAVR")) // AVRs with UPDI except AVR-Dx/Ex
       need_progmode = 0;
     PDATA(pgm)->flash_pageaddr = (unsigned long)-1L;
     if (pgm->flag & PGM_FL_IS_DW)
       unsupp = 1;
  } else if (mem_is_eeprom(mem)) {
    cmd[1] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_EEPROM_XMEGA: MTYPE_EEPROM;
    if(str_eq(p->family_id, "megaAVR") || str_eq(p->family_id, "tinyAVR")) // AVRs with UPDI except AVR-Dx/Ex
      need_progmode = 0;
    PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;
  } else if (mem_is_a_fuse(mem) || mem_is_fuses(mem)) {
    cmd[1] = MTYPE_FUSE_BITS;
    if(!(p->prog_modes & (PM_PDI | PM_UPDI)) && mem_is_a_fuse(mem))
      addr = mem_fuse_offset(mem);
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_userrow(mem)) {
    cmd[1] = MTYPE_USERSIG;
  } else if (mem_is_lock(mem)) {
    cmd[1] = MTYPE_LOCK_BITS;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_io(mem) || mem_is_sram(mem)) {
    cmd[1] = MTYPE_FLASH; // Works with jtag2updi, does not work with any xmega
    addr += avr_data_offset(p);
  } else if(mem_is_readonly(mem)) {
    unsigned char is;
    if(pgm->read_byte(pgm, p, mem, addr, &is) >= 0 && is == data)
      return 0;

    pmsg_error("cannot write to read-only memory %s of %s\n", mem->desc, p->desc);
    return -1;
  } else {
    pmsg_error("unknown memory %s\n", mem->desc);
    return -1;
  }

  if (unsupp) {
    pmsg_error("unsupported memory %s in debugWIRE mode\n", mem->desc);
    return -1;
  }

  if (need_progmode) {
    if (jtagmkII_program_enable(pgm) < 0)
      return -1;
  } else {
    if (jtagmkII_program_disable(pgm) < 0)
      return -1;
  }

  u32_to_b4(cmd + 2, writesize);
  u32_to_b4(cmd + 6, addr);
  cmd[10] = writedata;
  cmd[11] = writedata2;

  tries = 0;
retry:
  pmsg_notice2("jtagmkII_write_byte(): sending write memory command: ");
  jtagmkII_send(pgm, cmd, 10 + writesize);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_notice2("jtagmkII_write_byte(): "
      "timeout/error communicating with programmer (status %d)\n", status);
    if (tries++ < 3)
      goto retry;
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    goto fail;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  if (resp[0] != RSP_OK) {
    pmsg_error("bad response to write memory command: %s\n", jtagmkII_get_rc(resp[0]));
    goto fail;
  }

  free(resp);
  PDATA(pgm)->recently_written = 1;
  return 0;

fail:
  free(resp);
  return -1;
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
static int jtagmkII_set_sck_period(const PROGRAMMER *pgm, double v) {
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

static int jtagmkII_get_sck_period(const PROGRAMMER *pgm, double *v) {
  unsigned char buf[4];
  double clk;
  if (jtagmkII_getparm(pgm, PAR_OCD_JTAG_CLK, buf) < 0) {
    pmsg_error("cannot read JTAG clock speed\n");
    return -1;
  }

  if (buf[0] == 0)
    clk = 6.4e6;
  else if (buf[0] == 1)
    clk = 2.8e6;
  else
    clk = 5.35e6 / buf[0];

  *v = 1 / clk;
  return 0;
}

/*
 * Read an emulator parameter.  As the maximal parameter length is 4
 * bytes by now, we always copy out 4 bytes to *value, so the caller
 * must have allocated sufficient space.
 */
int jtagmkII_getparm(const PROGRAMMER *pgm, unsigned char parm,
		     unsigned char * value)
{
  int status;
  unsigned char buf[2], *resp, c;

  pmsg_notice2("jtagmkII_getparm()\n");

  buf[0] = CMND_GET_PARAMETER;
  buf[1] = parm;
  pmsg_notice2("jtagmkII_getparm(): "
    "Sending get parameter command (parm 0x%02x): ", parm);
  jtagmkII_send(pgm, buf, 2);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  if (c != RSP_PARAMETER) {
    pmsg_error("bad response to get parameter command: %s\n", jtagmkII_get_rc(c));
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
static int jtagmkII_setparm(const PROGRAMMER *pgm, unsigned char parm,
			    unsigned char * value)
{
  int status;
  /*
   * As the maximal parameter length is 4 bytes, we use a fixed-length
   * buffer, as opposed to malloc()ing it.
   */
  unsigned char buf[2 + 4], *resp, c;
  size_t size;
  const char *parstr = "???";

  pmsg_notice2("jtagmkII_setparm()\n");

  switch (parm) {
  case PAR_HW_VERSION: size = 2; parstr ="hw_version"; break;
  case PAR_FW_VERSION: size = 4; parstr ="fw_version"; break;
  case PAR_EMULATOR_MODE: size = 1; parstr ="emulator_mode"; break;
  case PAR_BAUD_RATE: size = 1; parstr ="baud_rate"; break;
  case PAR_OCD_VTARGET: size = 2; parstr ="ocd_vtarget"; break;
  case PAR_OCD_JTAG_CLK: size = 1; parstr ="ocd_jtag_clk"; break;
  case PAR_TIMERS_RUNNING: size = 1; parstr ="timers_running"; break;
  case PAR_EXTERNAL_RESET: size = 1; parstr ="external_reset"; break;
  case PAR_DAISY_CHAIN_INFO: size = 4; parstr ="daisy_chain_info"; break;
  case PAR_PDI_OFFSET_START: size = 4; parstr ="pdi_offset_start"; break;
  case PAR_PDI_OFFSET_END: size = 4; parstr ="pdi_offset_end"; break;
  default:
    pmsg_error("unknown parameter 0x%02x\n", parm);
    return -1;
  }

  buf[0] = CMND_SET_PARAMETER;
  buf[1] = parm;
  memcpy(buf + 2, value, size);
  pmsg_notice2("%s() sending set parameter command "
    "(parm %s 0x%02x, %u bytes): ", __func__, parstr, parm, (unsigned) size);
  jtagmkII_send(pgm, buf, size + 2);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to set parameter %s: %s\n", parstr, jtagmkII_get_rc(c));
    return -1;
  }

  return 0;
}


static void jtagmkII_display(const PROGRAMMER *pgm, const char *p) {
  unsigned char hw[4], fw[4];

  if (jtagmkII_getparm(pgm, PAR_HW_VERSION, hw) < 0 ||
      jtagmkII_getparm(pgm, PAR_FW_VERSION, fw) < 0)
    return;
  msg_info("%sMain MCU HW version   : %d\n", p, hw[0]);
  msg_info("%sMain MCU FW version   : %d.%02d\n", p, fw[1], fw[0]);
  msg_info("%sSec. MCU HW version   : %d\n", p, hw[1]);
  msg_info("%sSec. MCU FW version   : %d.%02d\n", p, fw[3], fw[2]);
  msg_info("%sSerial number         : %02x:%02x:%02x:%02x:%02x:%02x\n", p,
    PDATA(pgm)->serno[0], PDATA(pgm)->serno[1], PDATA(pgm)->serno[2],
    PDATA(pgm)->serno[3], PDATA(pgm)->serno[4], PDATA(pgm)->serno[5]);

  jtagmkII_print_parms1(pgm, p, stderr);

  return;
}


static void jtagmkII_print_parms1(const PROGRAMMER *pgm, const char *p, FILE *fp) {
  unsigned char vtarget[4], jtag_clock[4];
  char clkbuf[20];
  double clk;

  if (pgm->extra_features & HAS_VTARG_READ) {
    if (jtagmkII_getparm(pgm, PAR_OCD_VTARGET, vtarget) < 0)
      return;
    fmsg_out(fp, "%sVtarget               : %.1f V\n", p, b2_to_u16(vtarget) / 1000.0);
  }

  if ((pgm->flag & PGM_FL_IS_JTAG)) {
    if (jtagmkII_getparm(pgm, PAR_OCD_JTAG_CLK, jtag_clock) < 0)
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
      fmsg_out(fp, "%sJTAG clock            : %s (%.1f us)\n", p, clkbuf, 1.0e6 / clk);
    }
  }

  return;
}

static void jtagmkII_print_parms(const PROGRAMMER *pgm, FILE *fp) {
  jtagmkII_print_parms1(pgm, "", fp);
}

static unsigned char jtagmkII_mtype(const PROGRAMMER *pgm, const AVRPART *p, unsigned long addr) {
  if (p->prog_modes & (PM_PDI | PM_UPDI)) {
    if (addr >= PDATA(pgm)->boot_start)
      return MTYPE_BOOT_FLASH;
    else
      return MTYPE_FLASH;
  } else {
    return MTYPE_FLASH_PAGE;
  }
}

static unsigned int jtagmkII_memaddr(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr) {
  /*
   * Xmega devices handled by V7+ firmware don't want to be told their
   * m->offset within the write memory command.
   */
  if (PDATA(pgm)->fwver >= 0x700 && (p->prog_modes & (PM_PDI | PM_UPDI))) {
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
   * Old firmware, or non-Xmega device.  Non-Xmega (and non-AVR32)
   * devices always have an m->offset of 0, so we don't have to
   * distinguish them here.
   */
  return addr + m->offset;
}


#ifdef __OBJC__
#pragma mark -
#endif

static int jtagmkII_avr32_reset(const PROGRAMMER *pgm, unsigned char val,
                                unsigned char ret1, unsigned char ret2)
{
  int status;
  unsigned char buf[3], *resp;

  pmsg_notice("jtagmkII_avr32_reset(%2.2x)\n", val);

  buf[0] = CMND_GET_IR;
  buf[1] = 0x0C;
  status = jtagmkII_send(pgm, buf, 2);
  if(status < 0) return -1;

  status = jtagmkII_recv(pgm, &resp);
  if (status != 2 || resp[0] != 0x87 || resp[1] != ret1) {
    pmsg_notice("jtagmkII_avr32_reset(): "
      "Get_IR, expecting %2.2x but got %2.2x\n", ret1, resp[1]);

    //return -1;
  }

  buf[0] = CMND_GET_xxx;
  buf[1] = 5;
  buf[2] = val;
  status = jtagmkII_send(pgm, buf, 3);
  if(status < 0) return -1;

  status = jtagmkII_recv(pgm, &resp);
  if (status != 2 || resp[0] != 0x87 || resp[1] != ret2) {
    pmsg_notice("jtagmkII_avr32_reset(): "
      "Get_XXX, expecting %2.2x but got %2.2x\n", ret2, resp[1]);
    //return -1;
  }

  return 0;
}

// At init: AVR32_RESET_READ_IR | AVR32_RESET_READ_READ_CHIPINFO
static int jtagmkII_reset32(const PROGRAMMER *pgm, unsigned short flags) {
  int status, j, lineno;
  unsigned char *resp, buf[3];
  unsigned long val=0;

  pmsg_notice("jtagmkII_reset32(%2.2x)\n", flags);

  status = -1;

  // Happens at the start of a programming operation
  if(flags & AVR32_RESET_READ) {
    buf[0] = CMND_GET_IR;
    buf[1] = 0x11;
    status = jtagmkII_send(pgm, buf, 2);
    if(status < 0) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_recv(pgm, &resp);
    if (status != 2 || resp[0] != 0x87 || resp[1] != 01)
      {lineno = __LINE__; goto eRR;};
  }

  if(flags & (AVR32_RESET_WRITE | AVR32_SET4RUNNING)) {
    // AVR_RESET(0x1F)
    status = jtagmkII_avr32_reset(pgm, 0x1F, 0x01, 0x00);
    if(status < 0) {lineno = __LINE__; goto eRR;}
    // AVR_RESET(0x07)
    status = jtagmkII_avr32_reset(pgm, 0x07, 0x11, 0x1F);
    if(status < 0) {lineno = __LINE__; goto eRR;}
  }

  //if(flags & AVR32_RESET_COMMON)
  {
    val = jtagmkII_read_SABaddr(pgm, AVR32_DS, 0x01);
    if(val != 0) {lineno = __LINE__; goto eRR;}
    val = jtagmkII_read_SABaddr(pgm, AVR32_DC, 0x01);
    if(val != 0) {lineno = __LINE__; goto eRR;}
  }

  if(flags & (AVR32_RESET_READ | AVR32_RESET_CHIP_ERASE)) {
    status = jtagmkII_write_SABaddr(pgm, AVR32_DC, 0x01,
                                    AVR32_DC_DBE | AVR32_DC_DBR);
    if(status < 0) return -1;
  }

  if(flags & (AVR32_RESET_WRITE | AVR32_SET4RUNNING)) {
    status = jtagmkII_write_SABaddr(pgm, AVR32_DC, 0x01,
             AVR32_DC_ABORT | AVR32_DC_RESET | AVR32_DC_DBE | AVR32_DC_DBR);
    if(status < 0) return -1;
    for(j=0; j<21; ++j) {
      val = jtagmkII_read_SABaddr(pgm, AVR32_DS, 0x01);
    }
    if(val != 0x04000000) {lineno = __LINE__; goto eRR;}

    // AVR_RESET(0x00)
    status = jtagmkII_avr32_reset(pgm, 0x00, 0x01, 0x07);
    if(status < 0) {lineno = __LINE__; goto eRR;}
  }
//  if(flags & (AVR32_RESET_READ | AVR32_RESET_WRITE))
  {
    for(j=0; j<2; ++j) {
      val = jtagmkII_read_SABaddr(pgm, AVR32_DS, 0x01);
      if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
      if((val&0x05000020) != 0x05000020) {lineno = __LINE__; goto eRR;}
    }
  }

  //if(flags & (AVR32_RESET_READ | AVR32_RESET_WRITE | AVR32_RESET_CHIP_ERASE))
  {
    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe7b00044);  // mtdr 272, R0
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCSR, 0x01);
    if(val != 0x00000001) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCCPU, 0x01);
    if(val != 0x00000000) {lineno = __LINE__; goto eRR;}
  }

  // Read chip configuration - common for all
  if(flags & (AVR32_RESET_READ | AVR32_RESET_WRITE | AVR32_RESET_CHIP_ERASE)) {
    for(j=0; j<2; ++j) {
      val = jtagmkII_read_SABaddr(pgm, AVR32_DS, 0x01);
      if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
      if((val&0x05000020) != 0x05000020) {lineno = __LINE__; goto eRR;}
    }

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe7b00044);  // mtdr 272, R0
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCSR, 0x01);
    if(val != 0x00000001) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCCPU, 0x01);
    if(val != 0x00000000) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe1b00040);  // mfsr R0, 256
    if(status < 0) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe7b00044);  // mtdr 272, R0
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCSR, 0x01);
    if(val != 0x00000001) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCCPU, 0x01);
    if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DCEMU, 0x01, 0x00000000);
    if(status < 0) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe5b00045);  // mtdr R0, 276
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DS, 0x01);
    if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
    if((val&0x05000020) != 0x05000020) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe7b00044);  // mtdr 272, R0
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCSR, 0x01);
    if(val != 0x00000001) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCCPU, 0x01);
    if(val != 0x00000000) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe1b00041);  // mfsr R0, 260
    if(status < 0) {lineno = __LINE__; goto eRR;}
    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe7b00044);  // mtdr 272, R0
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCSR, 0x01);
    if(val != 0x00000001) {lineno = __LINE__; goto eRR;}
    val = jtagmkII_read_SABaddr(pgm, AVR32_DCCPU, 0x01);
    if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DCEMU, 0x01, 0x00000000);
    if(status < 0) {lineno = __LINE__; goto eRR;}
    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe5b00045);  // mtdr R0, 276
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, 0x00000010, 0x06); // need to recheck who does this ...
    if(val != 0x00000000) {lineno = __LINE__; goto eRR;}
  }

  if(flags & AVR32_RESET_CHIP_ERASE) {
    status = jtagmkII_avr32_reset(pgm, 0x1f, 0x01, 0x00);
    if(status < 0) {lineno = __LINE__; goto eRR;}
    status = jtagmkII_avr32_reset(pgm, 0x01, 0x11, 0x1f);
    if(status < 0) {lineno = __LINE__; goto eRR;}
  }

  if(flags & AVR32_SET4RUNNING) {
    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe1b00014);  // mfsr R0, 80
    if(status < 0) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe7b00044);  // mtdr 272, R0
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCSR, 0x01);
    if(val != 0x00000001) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DCCPU, 0x01);
    if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DCEMU, 0x01, 0x00000000);
    if(status < 0) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xe5b00045);  // mfdr R0, 276
    if(status < 0) {lineno = __LINE__; goto eRR;}

    val = jtagmkII_read_SABaddr(pgm, AVR32_DS, 0x01);
    if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
    if((val&0x05000020) != 0x05000020) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_write_SABaddr(pgm, AVR32_DINST, 0x01, 0xd623d703);  // retd
    if(status < 0) {lineno = __LINE__; goto eRR;}
  }

  return 0;

  eRR:
    pmsg_error("reset failed at line %d (status=%x val=%lx)\n", lineno, status, val);
    return -1;
}

static int jtagmkII_smc_init32(const PROGRAMMER *pgm) {
  int status, lineno;
  unsigned long val;

  // HMATRIX 0xFFFF1000
  status = jtagmkII_write_SABaddr(pgm, 0xffff1018, 0x05, 0x04000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1024, 0x05, 0x04000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1008, 0x05, 0x04000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1078, 0x05, 0x04000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1088, 0x05, 0x04000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  status = jtagmkII_write_SABaddr(pgm, 0xffff1018, 0x05, 0x08000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1024, 0x05, 0x08000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1008, 0x05, 0x08000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1078, 0x05, 0x08000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1088, 0x05, 0x08000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  status = jtagmkII_write_SABaddr(pgm, 0xffff1018, 0x05, 0x10000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1024, 0x05, 0x10000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1008, 0x05, 0x10000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1078, 0x05, 0x10000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1088, 0x05, 0x10000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  status = jtagmkII_write_SABaddr(pgm, 0xffff1018, 0x05, 0x00020000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1024, 0x05, 0x00020000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1008, 0x05, 0x00020000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1078, 0x05, 0x00020000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1088, 0x05, 0x00020000);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  status = jtagmkII_write_SABaddr(pgm, 0xffff1018, 0x05, 0x02000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1024, 0x05, 0x02000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1008, 0x05, 0x02000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1078, 0x05, 0x02000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xffff1088, 0x05, 0x02000000);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  status = jtagmkII_write_SABaddr(pgm, 0xfffe1c00, 0x05, 0x00010001);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xfffe1c04, 0x05, 0x05070a0b);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xfffe1c08, 0x05, 0x000b000c);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  status = jtagmkII_write_SABaddr(pgm, 0xfffe1c0c, 0x05, 0x00031103);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  // switchToClockSource
  val = jtagmkII_read_SABaddr(pgm, 0xffff0c28, 0x05);
  if (val != 0x00000000) {lineno = __LINE__; goto eRR;} // OSC 0
  status = jtagmkII_write_SABaddr(pgm, 0xffff0c28, 0x05, 0x0000607);
  if (status < 0) {lineno = __LINE__; goto eRR;}
  val = jtagmkII_read_SABaddr(pgm, 0xffff0c00, 0x05);
  if (val != 0x00000000) {lineno = __LINE__; goto eRR;} // PLL 0
  status = jtagmkII_write_SABaddr(pgm, 0xffff0c00, 0x05, 0x0000004);
  if (status < 0) {lineno = __LINE__; goto eRR;} // Power Manager
  status = jtagmkII_write_SABaddr(pgm, 0xffff0c00, 0x05, 0x0000005);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  usleep(1000000);

  val = jtagmkII_read_SABaddr(pgm, 0xfffe1408, 0x05);
  if (val != 0x0000a001) {lineno = __LINE__; goto eRR;} // PLL 0

  // need a small delay to let clock stabliize
  usleep(50*1000);

  return 0;

  eRR:
    pmsg_error("init failed at line %d\n", lineno);
    return -1;
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int jtagmkII_initialize32(const PROGRAMMER *pgm, const AVRPART *p) {
  int status, j;
  unsigned char buf[6], *resp;

  if (jtagmkII_setparm(pgm, PAR_DAISY_CHAIN_INFO, PDATA(pgm)->jtagchain) < 0) {
    pmsg_error("unable to setup JTAG chain\n");
    return -1;
  }

  free(PDATA(pgm)->flash_pagecache);
  free(PDATA(pgm)->eeprom_pagecache);
  if ((PDATA(pgm)->flash_pagecache = malloc(PDATA(pgm)->flash_pagesize)) == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }
  if ((PDATA(pgm)->eeprom_pagecache = malloc(PDATA(pgm)->eeprom_pagesize)) == NULL) {
    pmsg_error("out of memory\n");
    free(PDATA(pgm)->flash_pagecache);
    return -1;
  }
  PDATA(pgm)->flash_pageaddr = PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;

  for(j=0; j<2; ++j) {
    buf[0] = CMND_GET_IR;
    buf[1] = 0x1;
    if(jtagmkII_send(pgm, buf, 2) < 0)
      return -1;
    status = jtagmkII_recv(pgm, &resp);
    if(status <= 0 || resp[0] != 0x87) {
      msg_notice2("\n");
      pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
      return -1;
    }
    free(resp);

    memset(buf, 0, sizeof(buf));
    buf[0] = CMND_GET_xxx;
    buf[1] = 0x20;
    if(jtagmkII_send(pgm, buf, 6) < 0)
      return -1;
    status = jtagmkII_recv(pgm, &resp);
    if(status <= 0 || resp[0] != 0x87) {
      msg_notice2("\n");
      pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
      return -1;
    }

    if (status != 5 ||
    resp[2] != p->signature[0] ||
    resp[3] != p->signature[1] ||
    resp[4] != p->signature[2]) {
      if (ovsigck) {
        pmsg_warning("expected signature for %s is %02X %02X %02X\n", p->desc,
          p->signature[0], p->signature[1], p->signature[2]);
      } else {
        pmsg_error("expected signature for %s is %02X %02X %02X\n", p->desc,
          p->signature[0], p->signature[1], p->signature[2]);
        imsg_error("double check chip or use -F to override this check\n");
        return -1;
      }
    }
    free(resp);
  }

  return 0;
}

static int jtagmkII_chip_erase32(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  int status=0, loops;
  unsigned char *resp, buf[3], x, ret[4], *retP;
  unsigned long val=0;
  unsigned int lineno;

  pmsg_notice("jtagmkII_chip_erase32()\n");

  status = jtagmkII_reset32(pgm, AVR32_RESET_CHIP_ERASE);
  if(status != 0) {lineno = __LINE__; goto eRR;}

  // sequence of IR transitions
  ret[0] = 0x01;
  ret[1] = 0x05;
  ret[2] = 0x01;
  ret[3] = 0x00;

  retP = ret;
  for(loops=0; loops<1000; ++loops) {
    buf[0] = CMND_GET_IR;
    buf[1] = 0x0F;
    status = jtagmkII_send(pgm, buf, 2);
    if(status < 0) {lineno = __LINE__; goto eRR;}

    status = jtagmkII_recv(pgm, &resp);
    if (status != 2 || resp[0] != 0x87) {
      {lineno = __LINE__; goto eRR;}
    }
    x = resp[1];
    free(resp);
    if(x == *retP) ++retP;
    if(*retP == 0x00) break;
  }
  if(loops == 1000) {lineno = __LINE__; goto eRR;}

  status = jtagmkII_avr32_reset(pgm, 0x00, 0x01, 0x01);
  if(status < 0) {lineno = __LINE__; goto eRR;}

  val = jtagmkII_read_SABaddr(pgm, 0x00000010, 0x06);
  if(val != 0x00000000) {lineno = __LINE__; goto eRR;}

  // AVR32 "special"
  buf[0] = CMND_SET_PARAMETER;
  buf[1] = 0x03;
  buf[2] = 0x02;
  jtagmkII_send(pgm, buf, 3);
  status = jtagmkII_recv(pgm, &resp);
  if(status < 0 || resp[0] != RSP_OK) {lineno = __LINE__; goto eRR;}
  free(resp);

  return 0;

  eRR:
    pmsg_error("chip erase failed at line %d (status=%x val=%lx)\n", lineno, status, val);
    return -1;
}

static unsigned long jtagmkII_read_SABaddr(const PROGRAMMER *pgm, unsigned long addr,
                                           unsigned int prefix)
{
  unsigned char buf[6], *resp;
  int status;
  unsigned long val;
  unsigned long otimeout = serial_recv_timeout;

  serial_recv_timeout = 256;

  buf[0] = CMND_READ_SAB;
  buf[1] = prefix;
  u32_to_b4r(&buf[2], addr);

  if(jtagmkII_send(pgm, buf, 6) < 0)
    return ERROR_SAB;

  status = jtagmkII_recv(pgm, &resp);
  if(status <= 0 || resp[0] != 0x87) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d) resp=%x\n", status, resp[0]);
    serial_recv_timeout = otimeout;

    if(status > 0) {
      int i;
      msg_error("cmd: ");
      for(i=0; i<6; ++i)
        msg_error("%2.2x ", buf[i]);
      msg_error("\n");
      msg_error("Data: ");
      for(i=0; i<status; ++i)
        msg_error("%2.2x ", resp[i]);
      msg_error("\n");
    }
    return ERROR_SAB;
  }

  if(status != 5) {
    msg_notice2("\n");
    pmsg_error("wrong number of bytes (status %d)\n", status);
    serial_recv_timeout = otimeout;
    return ERROR_SAB;
  }

  val = b4_to_u32r(&resp[1]);
  free(resp);

  msg_notice2("\n");
  pmsg_notice("jtagmkII_read_SABaddr(): OCD Register %lx -> %4.4lx\n", addr, val);
  serial_recv_timeout = otimeout;
  return val;
}

static int jtagmkII_write_SABaddr(const PROGRAMMER *pgm, unsigned long addr,
                                  unsigned int prefix, unsigned long val)
{
  unsigned char buf[10], *resp;
  int status;

  buf[0] = CMND_WRITE_SAB;
  buf[1] = prefix;
  u32_to_b4r(&buf[2], addr);
  u32_to_b4r(&buf[6], val);

  if(jtagmkII_send(pgm, buf, 10) < 0)
    return -1;

  status = jtagmkII_recv(pgm, &resp);
  if(status <= 0 || resp[0] != RSP_OK) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }

  msg_notice2("\n");
  pmsg_notice("jtagmkII_write_SABaddr(): OCD Register %lx -> %4.4lx\n", addr, val);

  return 0;
}

static int jtagmkII_open32(PROGRAMMER *pgm, const char *port) {
  int status;
  unsigned char buf[6], *resp;
  union pinfo pinfo;

  pmsg_notice2("jtagmkII_open32()\n");

  /*
   * The JTAG ICE mkII always starts with a baud rate of 19200 Bd upon
   * attaching.  If the config file or command-line parameters specify
   * a higher baud rate, we switch to it later on, after establishing
   * the connection with the ICE.
   */
  pinfo.serialinfo.baud = 19200;
  pinfo.serialinfo.cflags = SERIAL_8N1;

  /*
   * If the port name starts with "usb", divert the serial routines
   * to the USB ones.  The serial_open() function for USB overrides
   * the meaning of the "baud" parameter to be the USB device ID to
   * search for.
   */
  if (str_starts(port, "usb")) {
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev;
    pinfo.usbinfo.vid = USB_VENDOR_ATMEL;
    pinfo.usbinfo.flags = 0;
    pinfo.usbinfo.pid = USB_DEVICE_JTAGICEMKII;
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_MKII;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_MKII;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_MKII;
    pgm->fd.usb.eep = 0;           /* no seperate EP for events */
#else
    msg_error("avrdude was compiled without usb support\n");
    return -1;
#endif
  }

  strcpy(pgm->port, port);
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  /*
   * drain any extraneous input
   */
  jtagmkII_drain(pgm, 0);

  status = jtagmkII_getsync(pgm, -1);
  if(status < 0) return -1;

  // AVR32 "special"
  buf[0] = CMND_SET_PARAMETER;
  buf[1] = 0x2D;
  buf[2] = 0x03;
  jtagmkII_send(pgm, buf, 3);
  status = jtagmkII_recv(pgm, &resp);
  if(status < 0 || resp[0] != RSP_OK)
    return -1;
  free(resp);

  buf[1] = 0x03;
  buf[2] = 0x02;
  jtagmkII_send(pgm, buf, 3);
  status = jtagmkII_recv(pgm, &resp);
  if(status < 0 || resp[0] != RSP_OK)
    return -1;
  free(resp);

  buf[1] = 0x03;
  buf[2] = 0x04;
  jtagmkII_send(pgm, buf, 3);
  status = jtagmkII_recv(pgm, &resp);
  if(status < 0 || resp[0] != RSP_OK)
    return -1;
  free(resp);

  return 0;
}

static void jtagmkII_close32(PROGRAMMER * pgm)
{
  int status, lineno;
  unsigned char *resp, buf[3], c;
  unsigned long val=0;

  pmsg_notice2("jtagmkII_close32()\n");

  // AVR32 "special"
  buf[0] = CMND_SET_PARAMETER;
  buf[1] = 0x03;
  buf[2] = 0x02;
  jtagmkII_send(pgm, buf, 3);
  status = jtagmkII_recv(pgm, &resp);
  if(status < 0 || resp[0] != RSP_OK) {lineno = __LINE__; goto eRR;}
  free(resp);

  buf[0] = CMND_SIGN_OFF;
  pmsg_notice2("jtagmkII_close(): sending sign-off command: ");
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return;
  }
  if (verbose >= 3) {
    msg_debug("\n");
    jtagmkII_prmsg(pgm, resp, status);
  } else
    msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to sign-off command: %s\n", jtagmkII_get_rc(c));
  }

  ret:
    serial_close(&pgm->fd);
    pgm->fd.ifd = -1;
    return;

  eRR:
    pmsg_error("close failed at line %d (status=%x val=%lx)\n", lineno, status, val);
    goto ret;
}

static int jtagmkII_paged_load32(const PROGRAMMER *pgm, const AVRPART *p_unused, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char cmd[7];
  unsigned char *resp;
  int lineno, status;
  unsigned long val=0;
  long otimeout = serial_recv_timeout;

  pmsg_notice2("jtagmkII_paged_load32(.., %s, %d, %d)\n", m->desc, page_size, n_bytes);

  serial_recv_timeout = 256;

  if(!(PDATA(pgm)->flags32 & FLAGS32_WRITE)) {
    status = jtagmkII_reset32(pgm, AVR32_RESET_READ);
    if(status != 0) {lineno = __LINE__; goto eRR;}
  }

  // Init SMC and set clocks
  if(!(PDATA(pgm)->flags32 & FLAGS32_INIT_SMC)) {
    status = jtagmkII_smc_init32(pgm);
    if(status != 0) {lineno = __LINE__; goto eRR;} // PLL 0
    PDATA(pgm)->flags32 |= FLAGS32_INIT_SMC;
  }

  //msg_error("\n pageSize=%d bytes=%d pages=%d m->offset=0x%x pgm->page_size %d\n",
  //        page_size, n_bytes, pages, m->offset, pgm->page_size);

  cmd[0] = CMND_READ_MEMORY32;
  cmd[1] = 0x40;
  cmd[2] = 0x05;

  for (; addr < maxaddr; addr += block_size) {
    block_size = maxaddr - addr < (unsigned int) pgm->page_size?
      maxaddr - addr: (unsigned int) pgm->page_size;
    pmsg_debug("jtagmkII_paged_load32(): "
      "block_size at addr %d is %d\n", addr, block_size);

    u32_to_b4r(cmd + 3, m->offset + addr);

    status = jtagmkII_send(pgm, cmd, 7);
    if(status<0) {lineno = __LINE__; goto eRR;}
    status = jtagmkII_recv(pgm, &resp);
    if(status<0) {lineno = __LINE__; goto eRR;}

    if (verbose >= 3) {
      msg_debug("\n");
      jtagmkII_prmsg(pgm, resp, status);
    } else
      msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
    if (resp[0] != 0x87) {
      pmsg_error("bad response to write memory command: %s\n", jtagmkII_get_rc(resp[0]));
      free(resp);
      return -1;
    }
    memcpy(m->buf + addr, resp + 1, block_size);
    free(resp);

  }

  serial_recv_timeout = otimeout;

  status = jtagmkII_reset32(pgm, AVR32_SET4RUNNING);
  if(status < 0) {lineno = __LINE__; goto eRR;}

  return addr;

  eRR:
    serial_recv_timeout = otimeout;
    pmsg_error("paged load failed at line %d (status=%x val=%lx)\n", lineno, status, val);
    return -1;
}

static int jtagmkII_paged_write32(const PROGRAMMER *pgm, const AVRPART *p_unused , const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  unsigned int block_size;
  unsigned char *cmd=NULL;
  unsigned char *resp;
  int lineno, status, pages, sPageNum, pageNum, blocks;
  unsigned long val=0;
  unsigned long otimeout = serial_recv_timeout;
  unsigned int maxaddr = addr + n_bytes;

  serial_recv_timeout = 256;

  if(n_bytes == 0) return -1;

  status = jtagmkII_reset32(pgm, AVR32_RESET_WRITE);
  if(status != 0) {lineno = __LINE__; goto eRR;}
  PDATA(pgm)->flags32 |= FLAGS32_WRITE;

  pages = (n_bytes - addr - 1)/page_size + 1;
  sPageNum = addr/page_size;
  //msg_error("\n pageSize=%d bytes=%d pages=%d m->offset=0x%x pgm->page_size %d\n",
  //        page_size, n_bytes, pages, m->offset, pgm->page_size);

  // Before any errors can happen
  if ((cmd = malloc(pgm->page_size + 10)) == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }

  // Init SMC and set clocks
  if(!(PDATA(pgm)->flags32 & FLAGS32_INIT_SMC)) {
    status = jtagmkII_smc_init32(pgm);
    if(status != 0) {lineno = __LINE__; goto eRR;} // PLL 0
    PDATA(pgm)->flags32 |= FLAGS32_INIT_SMC;
  }

  // First unlock the pages
  for(pageNum=sPageNum; pageNum < pages; ++pageNum) {
    status =jtagmkII_flash_lock32(pgm, 0, pageNum);
    if(status < 0) {lineno = __LINE__; goto eRR;}
  }

  // Then erase them (guess could do this in the same loop above?)
  for(pageNum=sPageNum; pageNum < pages; ++pageNum) {
    status =jtagmkII_flash_erase32(pgm, pageNum);
    if(status < 0) {lineno = __LINE__; goto eRR;}
  }

  cmd[0] = CMND_WRITE_MEMORY32;
  u32_to_b4r(&cmd[1], 0x40000000);  // who knows
  cmd[5] = 0x5;

  for(pageNum=sPageNum; pageNum < pages; ++pageNum) {

    status = jtagmkII_flash_clear_pagebuffer32(pgm);
    if(status != 0) {lineno = __LINE__; goto eRR;}

    for(blocks=0; blocks<2; ++blocks) {
      block_size = maxaddr - addr < (unsigned int) pgm->page_size?
        maxaddr - addr: (unsigned int) pgm->page_size;
      pmsg_debug("jtagmkII_paged_write32(): "
        "block_size at addr %d is %d\n", addr, block_size);

      u32_to_b4r(cmd + 6, m->offset + addr);
      memset(cmd + 10, 0xff, pgm->page_size);
      memcpy(cmd + 10, m->buf + addr, block_size);

      status = jtagmkII_send(pgm, cmd, pgm->page_size + 10);
      if(status<0) {lineno = __LINE__; goto eRR;}
      status = jtagmkII_recv(pgm, &resp);
      if (status<0) {lineno = __LINE__; goto eRR;}

      if (verbose >= 3) {
        msg_debug("\n");
        jtagmkII_prmsg(pgm, resp, status);
      } else
        msg_notice2("0x%02x (%d bytes msg)\n", resp[0], status);
      if (resp[0] != RSP_OK) {
        pmsg_error("bad response to write memory command: %s\n", jtagmkII_get_rc(resp[0]));
        free(resp);
        free(cmd);
        return -1;
      }
      free(resp);

      addr += block_size;


    }
    status = jtagmkII_flash_write_page32(pgm, pageNum);
    if(status < 0) {lineno = __LINE__; goto eRR;}
  }
  serial_recv_timeout = otimeout;

  status = jtagmkII_reset32(pgm, AVR32_SET4RUNNING);  // AVR32_SET4RUNNING | AVR32_RELEASE_JTAG
  if(status < 0) {lineno = __LINE__; goto eRR;}

  free(cmd);
  return addr;

  eRR:
    serial_recv_timeout = otimeout;
    free(cmd);
    pmsg_error("paged write failed at line %d (status=%x val=%lx)\n", lineno, status, val);
    return -1;
}


static int jtagmkII_flash_lock32(const PROGRAMMER *pgm, unsigned char lock, unsigned int page) {
  int status, lineno, i;
  unsigned long val, cmd=0;

  for(i=0; i<256; ++i) {
    val = jtagmkII_read_SABaddr(pgm, AVR32_FLASHC_FSR, 0x05);
    if(val == ERROR_SAB) continue;
    if(val & AVR32_FLASHC_FSR_RDY) break;
  }
  if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
  if(!(val&AVR32_FLASHC_FSR_RDY)) {lineno = __LINE__; goto eRR;} // Flash better be ready

  page <<= 8;
  cmd = AVR32_FLASHC_FCMD_KEY | page | (lock ? AVR32_FLASHC_FCMD_LOCK : AVR32_FLASHC_FCMD_UNLOCK);
  status = jtagmkII_write_SABaddr(pgm, AVR32_FLASHC_FCMD, 0x05, cmd);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  return 0;

  eRR:
    pmsg_error("flash lock failed at line %d page %d cmd %8.8lx\n", lineno, page, cmd);
  return -1;
}

static int jtagmkII_flash_erase32(const PROGRAMMER *pgm, unsigned int page) {
  int status, lineno, i;
  unsigned long val=0, cmd=0, err=0;

  for(i=0; i<256; ++i) {
    val = jtagmkII_read_SABaddr(pgm, AVR32_FLASHC_FSR, 0x05);
    if(val == ERROR_SAB) continue;
    if(val & AVR32_FLASHC_FSR_RDY) break;
  }
  if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
  if(!(val&AVR32_FLASHC_FSR_RDY)) {lineno = __LINE__; goto eRR;} // Flash better be ready

  page <<= 8;
  cmd = AVR32_FLASHC_FCMD_KEY | page | AVR32_FLASHC_FCMD_ERASE_PAGE;
  status = jtagmkII_write_SABaddr(pgm, AVR32_FLASHC_FCMD, 0x05, cmd);
  if (status < 0) {lineno = __LINE__; goto eRR;}

//msg_error("ERASE %x -> %x\n", cmd, AVR32_FLASHC_FCMD);

  err = 0;
  for(i=0; i<256; ++i) {
    val = jtagmkII_read_SABaddr(pgm, AVR32_FLASHC_FSR, 0x05);
    if(val == ERROR_SAB) continue;
    err |= val;
    if(val & AVR32_FLASHC_FSR_RDY) break;
  }
  if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
  if(!(val & AVR32_FLASHC_FSR_RDY)) {lineno = __LINE__; goto eRR;}
  if(err & AVR32_FLASHC_FSR_ERR) {lineno = __LINE__; goto eRR;}

  return 0;

  eRR:
    pmsg_error("flash erase failed at line %d page %d cmd %8.8lx val %lx\n", lineno, page, cmd, val);
    return -1;
}

static int jtagmkII_flash_write_page32(const PROGRAMMER *pgm, unsigned int page) {
  int status, lineno, i;
  unsigned long val=0, cmd, err;

  page <<= 8;
  cmd = AVR32_FLASHC_FCMD_KEY | page | AVR32_FLASHC_FCMD_WRITE_PAGE;
  status = jtagmkII_write_SABaddr(pgm, AVR32_FLASHC_FCMD, 0x05, cmd);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  err = 0;
  for(i=0; i<256; ++i) {
    val = jtagmkII_read_SABaddr(pgm, AVR32_FLASHC_FSR, 0x05);
    if(val == ERROR_SAB) continue;
    err |= val;
    if(val & AVR32_FLASHC_FSR_RDY) break;
  }
  if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
  if(!(val & AVR32_FLASHC_FSR_RDY)) {lineno = __LINE__; goto eRR;}
  if(err & AVR32_FLASHC_FSR_ERR) {lineno = __LINE__; goto eRR;}

  return 0;

  eRR:
    pmsg_error("flash write failed at line %d page %d cmd %8.8lx val %lx\n", lineno, page, cmd, val);
    return -1;
}

static int jtagmkII_flash_clear_pagebuffer32(const PROGRAMMER *pgm) {
  int status, lineno, i;
  unsigned long val=0, cmd, err;

  cmd = AVR32_FLASHC_FCMD_KEY | AVR32_FLASHC_FCMD_CLEAR_PAGE_BUFFER;
  status = jtagmkII_write_SABaddr(pgm, AVR32_FLASHC_FCMD, 0x05, cmd);
  if (status < 0) {lineno = __LINE__; goto eRR;}

  err = 0;
  for(i=0; i<256; ++i) {
    val = jtagmkII_read_SABaddr(pgm, AVR32_FLASHC_FSR, 0x05);
    if(val == ERROR_SAB) continue;
    err |= val;
    if(val & AVR32_FLASHC_FSR_RDY) break;
  }
  if(val == ERROR_SAB) {lineno = __LINE__; goto eRR;}
  if(!(val & AVR32_FLASHC_FSR_RDY)) {lineno = __LINE__; goto eRR;}
  if(err & AVR32_FLASHC_FSR_ERR) {lineno = __LINE__; goto eRR;}

  return 0;

  eRR:
    pmsg_error("clear page buffer failed at line %d cmd %8.8lx val %lx\n", lineno, cmd, val);
    return -1;
}

// Periodic calls in terminal mode to keep the programmer jtagmkii_updi enabled
static int jtagmkII_updi_term_keep_alive(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  unsigned char buf[2], *resp, c = 0xff;
  int status;

  buf[0] = CMND_GET_SYNC;
  jtagmkII_send(pgm, buf, 1);

  status = jtagmkII_recv(pgm, &resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_error("timeout/error communicating with programmer (status %d)\n", status);
    return -1;
  }

  c = resp[0];
  free(resp);
  if (c != RSP_OK) {
    pmsg_error("bad response to `get_sync` command: %s\n", jtagmkII_get_rc(c));
    return -1;
  }

  return 0;
}

// Read the signature if most recent operation was an erase or a write to ensure it finished
static int jtagmkII_updi_end_programming(const PROGRAMMER *pgm, const AVRPART *p) {
  return PDATA(pgm)->recently_written? avr_read(pgm, p, "signature", NULL): 0;
}

#ifdef __OBJC__
#pragma mark -
#endif

const char jtagmkII_desc[] = "Atmel JTAG ICE mkII";

void jtagmkII_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "JTAGMKII");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase;
  pgm->open           = jtagmkII_open;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->page_erase     = jtagmkII_page_erase;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->set_sck_period = jtagmkII_set_sck_period;
  pgm->get_sck_period = jtagmkII_get_sck_period;
  pgm->parseextparams = jtagmkII_parseextparms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->read_chip_rev  = jtagmkII_read_chip_rev;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_JTAG;
}

const char jtagmkII_dw_desc[] = "Atmel JTAG ICE mkII in debugWire mode";

void jtagmkII_dw_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "JTAGMKII_DW");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase_dw;
  pgm->open           = jtagmkII_open_dw;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_DW;
}

const char jtagmkII_pdi_desc[] = "Atmel JTAG ICE mkII in PDI mode";

void jtagmkII_pdi_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "JTAGMKII_PDI");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase;
  pgm->open           = jtagmkII_open_pdi;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->page_erase     = jtagmkII_page_erase;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->read_chip_rev  = jtagmkII_read_chip_rev;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_PDI;
}

const char jtagmkII_updi_desc[] = "Atmel JTAG ICE mkII in UPDI mode";

void jtagmkII_updi_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "JTAGMKII_UPDI");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase;
  pgm->open           = jtagmkII_open_pdi;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->page_erase     = jtagmkII_page_erase;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->parseextparams = jtagmkII_parseextparms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->read_chip_rev  = jtagmkII_read_chip_rev;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_PDI;
  pgm->term_keep_alive= jtagmkII_updi_term_keep_alive;
  pgm->end_programming= jtagmkII_updi_end_programming;
}

const char jtagmkII_dragon_desc[] = "Atmel AVR Dragon in JTAG mode";

void jtagmkII_dragon_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "DRAGON_JTAG");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase;
  pgm->open           = jtagmkII_dragon_open;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->page_erase     = jtagmkII_page_erase;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->set_sck_period = jtagmkII_set_sck_period;
  pgm->get_sck_period = jtagmkII_get_sck_period;
  pgm->parseextparams = jtagmkII_parseextparms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->read_chip_rev  = jtagmkII_read_chip_rev;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_JTAG;
}

const char jtagmkII_dragon_dw_desc[] = "Atmel AVR Dragon in debugWire mode";

void jtagmkII_dragon_dw_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "DRAGON_DW");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase_dw;
  pgm->open           = jtagmkII_dragon_open_dw;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_DW;
}

const char jtagmkII_avr32_desc[] = "Atmel JTAG ICE mkII in AVR32 mode";

void jtagmkII_avr32_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "JTAGMKII_AVR32");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize32;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase32;
  pgm->open           = jtagmkII_open32;
  pgm->close          = jtagmkII_close32;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write32;
  pgm->paged_load     = jtagmkII_paged_load32;
  pgm->print_parms    = jtagmkII_print_parms;
  //pgm->set_sck_period = jtagmkII_set_sck_period;
  //pgm->parseextparams = jtagmkII_parseextparms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_JTAG;
}

const char jtagmkII_dragon_pdi_desc[] = "Atmel AVR Dragon in PDI mode";

void jtagmkII_dragon_pdi_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "DRAGON_PDI");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtagmkII_initialize;
  pgm->display        = jtagmkII_display;
  pgm->enable         = jtagmkII_enable;
  pgm->disable        = jtagmkII_disable;
  pgm->program_enable = jtagmkII_program_enable_INFO;
  pgm->chip_erase     = jtagmkII_chip_erase;
  pgm->open           = jtagmkII_dragon_open_pdi;
  pgm->close          = jtagmkII_close;
  pgm->read_byte      = jtagmkII_read_byte;
  pgm->write_byte     = jtagmkII_write_byte;

  /*
   * optional functions
   */
  pgm->paged_write    = jtagmkII_paged_write;
  pgm->paged_load     = jtagmkII_paged_load;
  pgm->page_erase     = jtagmkII_page_erase;
  pgm->print_parms    = jtagmkII_print_parms;
  pgm->setup          = jtagmkII_setup;
  pgm->teardown       = jtagmkII_teardown;
  pgm->read_chip_rev  = jtagmkII_read_chip_rev;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_PDI;
}
