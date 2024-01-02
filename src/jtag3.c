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
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "crc16.h"
#include "jtag3.h"
#include "jtag3_private.h"
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

  int prog_enabled;         /* Cached value of PROGRAMMING status. */

  /* JTAG chain stuff */
  unsigned char jtagchain[4];

  /* Start address of Xmega boot area */
  unsigned long boot_start;

  /* Flag for triggering HV UPDI */
  bool use_hvupdi;

  /* Get/set flags for SUFFER register */
  bool suffer_get;
  bool suffer_set;
  unsigned char suffer_data[2];

  /* Get/set flags for target power switch */
  bool vtarg_switch_get;
  bool vtarg_switch_set;
  unsigned char vtarg_switch_data[2];

  /* Get/set flags for adjustable target voltage */
  bool vtarg_get;
  bool vtarg_set;
  double vtarg_data;

  /* Flag for PICkit4/SNAP mode switching */
  int pk4_snap_mode;

  /* SIB string cache */
  char sib_string[AVR_SIBLEN];

  /* Function to set the appropriate clock parameter */
  int (*set_sck)(const PROGRAMMER *, unsigned char *);
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

/*
 * pgm->flag is marked as "for private use of the programmer".
 * The following defines this programmer's use of that field.
 */
#define PGM_FL_IS_DW            (0x0001)
#define PGM_FL_IS_PDI           (0x0002)
#define PGM_FL_IS_JTAG          (0x0004)
#define PGM_FL_IS_EDBG          (0x0008)
#define PGM_FL_IS_UPDI          (0x0010)
#define PGM_FL_IS_TPI           (0x0020)

static int jtag3_open(PROGRAMMER *pgm, const char *port);
static int jtag3_edbg_prepare(const PROGRAMMER *pgm);
static int jtag3_edbg_signoff(const PROGRAMMER *pgm);
static int jtag3_edbg_send(const PROGRAMMER *pgm, unsigned char *data, size_t len);
static int jtag3_edbg_recv_frame(const PROGRAMMER *pgm, unsigned char **msg);

static int jtag3_initialize(const PROGRAMMER *pgm, const AVRPART *p);
static int jtag3_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
static int jtag3_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                                unsigned long addr, unsigned char * value);
static int jtag3_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                                unsigned long addr, unsigned char data);
static int jtag3_set_sck_period(const PROGRAMMER *pgm, double v);
void jtag3_display(const PROGRAMMER *pgm, const char *p);
void jtag3_print_parms1(const PROGRAMMER *pgm, const char *p, FILE *fp);
static int jtag3_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes);
static unsigned char jtag3_mtype(const PROGRAMMER *pgm, const AVRPART *p, unsigned long addr);
static unsigned int jtag3_memaddr(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr);


void jtag3_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
    pmsg_error("out of memory allocating private data\n");
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

static void
u32_to_b4_big_endian(unsigned char *b, unsigned long l)
{
  b[0] = (l >> 24) & 0xff;
  b[1] = (l >> 16) & 0xff;
  b[2] = (l >> 8) & 0xff;
  b[3] = l & 0xff;
}

static void
u16_to_b2_big_endian(unsigned char *b, unsigned short l)
{
  b[0] = (l >> 8) & 0xff;
  b[1] = l & 0xff;
}

static void jtag3_print_data(unsigned char *b, size_t s)
{
  size_t i;

  if (s < 2)
    return;

  for (i = 0; i < s; i++) {
    msg_info("0x%02x", b[i]);
    if (i % 16 == 15)
      msg_info("\n");
    else
      msg_info(" ");
  }
  if (i % 16 != 0)
    msg_info("\n");
}

static void jtag3_prmsg(const PROGRAMMER *pgm, unsigned char *data, size_t len) {
  if (verbose >= 4) {
    size_t i;

    msg_trace("Raw message:\n");

    for (i = 0; i < len; i++) {
      msg_trace("%02x ", data[i]);
      if (i % 16 == 15)
        msg_trace("\n");
      else
        msg_trace(" ");
    }
    if (i % 16 != 0)
      msg_trace("\n");
  }

  switch (data[0]) {
    case SCOPE_INFO:
      msg_info("[info] ");
      break;

    case SCOPE_GENERAL:
      msg_info("[general] ");
      break;

    case SCOPE_AVR_ISP:
      msg_info("[AVRISP] ");
      jtag3_print_data(data + 1, len - 1);
      return;

    case SCOPE_AVR:
      msg_info("[AVR] ");
      break;

    default:
      msg_info("[scope 0x%02x] ", data[0]);
      break;
  }

  switch (data[1]) {
    case RSP3_OK:
      msg_info("OK\n");
      break;

    case RSP3_FAILED:
      msg_info("FAILED");
      if (len > 3)
      {
        char reason[50];
        sprintf(reason, "0x%02x", data[3]);
        switch (data[3]) {
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
        msg_info(", reason: %s\n", reason);
      }
      else {
        msg_info(", unspecified reason\n");
      }
      break;

    case RSP3_DATA:
      msg_info("Data returned:\n");
      jtag3_print_data(data + 2, len - 2);
      break;

    case RSP3_INFO:
      msg_info("Info returned:\n");
      for (size_t i = 2; i < len; i++) {
        if (isprint(data[i]))
          msg_info("%c", data[i]);
        else
          msg_info("\\%03o", data[i]);
      }
      msg_info("\n");
      break;

    case RSP3_PC:
      if (len < 7) {
        msg_info("PC reply too short\n");
      }
      else {
        unsigned long pc = (data[6] << 24) | (data[5] << 16)
          | (data[4] << 8) | data[3];
        msg_info("PC 0x%0lx\n", pc);
      }
      break;

    default:
      msg_info("unknown message 0x%02x\n", data[1]);
  }
}

static int jtag3_errcode(int reason)
{
  if (reason == RSP3_FAIL_OCD_LOCKED ||
      reason == RSP3_FAIL_CRC_FAILURE)
    return LIBAVRDUDE_SOFTFAIL;
  return LIBAVRDUDE_GENERAL_FAILURE;
}

static void jtag3_prevent(const PROGRAMMER *pgm, unsigned char *data, size_t len) {
  if (verbose >= 4) {
    size_t i;

    msg_trace("Raw event:\n");

    for (i = 0; i < len; i++) {
      msg_trace("%02x ", data[i]);
      if (i % 16 == 15)
        msg_trace("\n");
      else
        msg_trace(" ");
    }
    if (i % 16 != 0)
      msg_trace("\n");
  }

  msg_info("Event serial 0x%04x, ", (data[3] << 8) | data[2]);

  switch (data[4]) {
    case SCOPE_INFO:
      msg_info("[info] ");
      break;

    case SCOPE_GENERAL:
      msg_info("[general] ");
      break;

    case SCOPE_AVR:
      msg_info("[AVR] ");
      break;

    default:
      msg_info("[scope 0x%02x] ", data[0]);
      break;
  }

  switch (data[5]) {
    case EVT3_BREAK:
      msg_info("BREAK");
      if (len >= 11) {
        msg_info(", PC = 0x%lx, reason ", b4_to_u32(data + 6));
        switch (data[10]) {
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
          msg_info("unknown: 0x%02x", data[10]);
        }
        /* There are two more bytes of data which always appear to be
        * 0x01, 0x00.  Purpose unknown. */
      }
      break;

    case EVT3_SLEEP:
      if (len >= 8 && data[7] == 0)
        msg_info("sleeping");
      else if (len >= 8 && data[7] == 1)
        msg_info("wakeup");
      else
        msg_info("unknown SLEEP event");
      break;

    case EVT3_POWER:
      if (len >= 8 && data[7] == 0)
        msg_info("power-down");
      else if (len >= 8 && data[7] == 1)
        msg_info("power-up");
      else
        msg_info("unknown POWER event");
      break;

    default:
      msg_info("UNKNOWN 0x%02x", data[5]);
      break;
  }
  msg_info("\n");
}

int jtag3_send(const PROGRAMMER *pgm, unsigned char *data, size_t len) {
  unsigned char *buf;

  if (pgm->flag & PGM_FL_IS_EDBG)
    return jtag3_edbg_send(pgm, data, len);

  msg_debug("\n");
  pmsg_debug("jtag3_send(): sending %lu bytes\n", (unsigned long) len);

  if ((buf = malloc(len + 4)) == NULL) {
    pmsg_error("out of memory");
    return -1;
  }

  buf[0] = TOKEN;
  buf[1] = 0;                   /* dummy */
  u16_to_b2(buf + 2, PDATA(pgm)->command_sequence);
  memcpy(buf + 4, data, len);

  if (serial_send(&pgm->fd, buf, len + 4) != 0) {
    pmsg_error("unable to send command to serial port\n");
    free(buf);
    return -1;
  }

  free(buf);

  return 0;
}

static int jtag3_edbg_send(const PROGRAMMER *pgm, unsigned char *data, size_t len) {
  unsigned char buf[USBDEV_MAX_XFER_3];
  unsigned char status[USBDEV_MAX_XFER_3];
  int rv;

  if (verbose >= 4) {
    memset(buf, 0, USBDEV_MAX_XFER_3);
    memset(status, 0, USBDEV_MAX_XFER_3);
  }

  msg_debug("\n");
  pmsg_debug("jtag3_edbg_send(): sending %lu bytes\n", (unsigned long) len);

  /* 4 bytes overhead for CMD, fragment #, and length info */
  int max_xfer = pgm->fd.usb.max_xfer;

  int nfragments = (len + max_xfer - 1) / max_xfer;
  if (nfragments > 1) {
    pmsg_debug("jtag3_edbg_send(): fragmenting into %d packets\n", nfragments);
  }
  int frag;
  for (frag = 0; frag < nfragments; frag++) {
    int this_len;

    /* All fragments have the (CMSIS-DAP layer) CMD, the fragment
      * identifier, and the length field. */
    buf[0] = EDBG_VENDOR_AVR_CMD;
    buf[1] = ((frag + 1) << 4) | nfragments;

    if (frag == 0) {
      /* Only first fragment has TOKEN and seq#, thus four bytes
        * less payload than subsequent fragments. */
      this_len = (int) len < max_xfer - 8? (int) len: max_xfer - 8;
      buf[2] = (this_len + 4) >> 8;
      buf[3] = (this_len + 4) & 0xff;
      buf[4] = TOKEN;
      buf[5] = 0;                   /* dummy */
      u16_to_b2(buf + 6, PDATA(pgm)->command_sequence);
      if(this_len < 0) {
        pmsg_error("unexpected this_len = %d\n", this_len);
        return -1;
      }
      memcpy(buf + 8, data, this_len);
    }
    else {
      this_len = (int) len < max_xfer - 4? (int) len: max_xfer - 4;
      buf[2] = (this_len) >> 8;
      buf[3] = (this_len) & 0xff;
      if(this_len < 0) {
        pmsg_error("unexpected this_len = %d\n", this_len);
        return -1;
      }
      memcpy(buf + 4, data, this_len);
    }

    if (serial_send(&pgm->fd, buf, max_xfer) != 0) {
      pmsg_notice("jtag3_edbg_send(): unable to send command to serial port\n");
      return -1;
    }
    rv = serial_recv(&pgm->fd, status, max_xfer);

    if (rv < 0) {
      /* timeout in receive */
      pmsg_notice2("jtag3_edbg_send(): timeout receiving packet\n");
      return -1;
    }
    if (status[0] != EDBG_VENDOR_AVR_CMD ||
        (frag == nfragments - 1 && status[1] != 0x01)) {
      /* what to do in this case? */
      pmsg_notice("jtag3_edbg_send(): unexpected response 0x%02x, 0x%02x\n", status[0], status[1]);
    }
    data += this_len;
    len -= this_len;
  }

  return 0;
}

/*
 * Send out all the CMSIS-DAP stuff needed to prepare the ICE.
 */
static int jtag3_edbg_prepare(const PROGRAMMER *pgm) {
  unsigned char buf[USBDEV_MAX_XFER_3];
  unsigned char status[USBDEV_MAX_XFER_3];
  int rv;

  msg_debug("\n");
  pmsg_debug("jtag3_edbg_prepare()\n");

  if (verbose >= 4)
    memset(buf, 0, USBDEV_MAX_XFER_3);

  buf[0] = CMSISDAP_CMD_CONNECT;
  buf[1] = CMSISDAP_CONN_SWD;
  if (serial_send(&pgm->fd, buf, pgm->fd.usb.max_xfer) != 0) {
    pmsg_error("unable to send command to serial port\n");
    return -1;
  }
  rv = serial_recv(&pgm->fd, status, pgm->fd.usb.max_xfer);
  if (rv != pgm->fd.usb.max_xfer) {
    pmsg_error("unable to read from serial port (%d)\n", rv);
    return -1;
  }
  if (status[0] != CMSISDAP_CMD_CONNECT ||
      status[1] == 0)
    pmsg_error("unexpected response 0x%02x, 0x%02x\n", status[0], status[1]);
  pmsg_notice2("jtag3_edbg_prepare(): connection status 0x%02x\n", status[1]);

  buf[0] = CMSISDAP_CMD_LED;
  buf[1] = CMSISDAP_LED_CONNECT;
  buf[2] = 1;
  if (serial_send(&pgm->fd, buf, pgm->fd.usb.max_xfer) != 0) {
    pmsg_error("unable to send command to serial port\n");
    return -1;
  }
  rv = serial_recv(&pgm->fd, status, pgm->fd.usb.max_xfer);
  if (rv != pgm->fd.usb.max_xfer) {
    pmsg_error("unable to read from serial port (%d)\n", rv);
    return -1;
  }
  if (status[0] != CMSISDAP_CMD_LED ||
      status[1] != 0)
    pmsg_error("unexpected response 0x%02x, 0x%02x\n", status[0], status[1]);

  return 0;
}


/*
 * Send out all the CMSIS-DAP stuff when signing off.
 */
static int jtag3_edbg_signoff(const PROGRAMMER *pgm) {
  unsigned char buf[USBDEV_MAX_XFER_3];
  unsigned char status[USBDEV_MAX_XFER_3];
  int rv;

  msg_debug("\n");
  pmsg_debug("jtag3_edbg_signoff()\n");

  if (verbose >= 4)
    memset(buf, 0, USBDEV_MAX_XFER_3);

  buf[0] = CMSISDAP_CMD_LED;
  buf[1] = CMSISDAP_LED_CONNECT;
  buf[2] = 0;
  if (serial_send(&pgm->fd, buf, pgm->fd.usb.max_xfer) != 0) {
    pmsg_notice("jtag3_edbg_signoff(): unable to send command to serial port\n");
    return -1;
  }
  rv = serial_recv(&pgm->fd, status, pgm->fd.usb.max_xfer);
  if (rv != pgm->fd.usb.max_xfer) {
    pmsg_notice("jtag3_edbg_signoff(): unable to read from serial port (%d)\n", rv);
    return -1;
  }
  if (status[0] != CMSISDAP_CMD_LED ||
      status[1] != 0)
    pmsg_notice("jtag3_edbg_signoff(): unexpected response 0x%02x, 0x%02x\n", status[0], status[1]);

  buf[0] = CMSISDAP_CMD_DISCONNECT;
  if (serial_send(&pgm->fd, buf, pgm->fd.usb.max_xfer) != 0) {
    pmsg_notice("jtag3_edbg_signoff(): unable to send command to serial port\n");
    return -1;
  }
  rv = serial_recv(&pgm->fd, status, pgm->fd.usb.max_xfer);
  if (rv != pgm->fd.usb.max_xfer) {
    pmsg_notice("jtag3_edbg_signoff(): unable to read from serial port (%d)\n", rv);
    return -1;
  }
  if (status[0] != CMSISDAP_CMD_DISCONNECT ||
      status[1] != 0)
    pmsg_notice("jtag3_edbg_signoff(): unexpected response 0x%02x, 0x%02x\n", status[0], status[1]);

  return 0;
}


static int jtag3_drain(const PROGRAMMER *pgm, int display) {
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
static int jtag3_recv_frame(const PROGRAMMER *pgm, unsigned char **msg) {
  int rv;
  unsigned char *buf = NULL;

  if (pgm->flag & PGM_FL_IS_EDBG)
    return jtag3_edbg_recv_frame(pgm, msg);

  pmsg_trace("jtag3_recv_frame():\n");

  if ((buf = malloc(pgm->fd.usb.max_xfer)) == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }
  if (verbose >= 4)
    memset(buf, 0, pgm->fd.usb.max_xfer);

  rv = serial_recv(&pgm->fd, buf, pgm->fd.usb.max_xfer);

  if (rv < 0) {
    /* timeout in receive */
    pmsg_notice2("jtag3_recv_frame(): timeout receiving packet\n");
    free(buf);
    return -1;
  }

  *msg = buf;

  return rv;
}

static int jtag3_edbg_recv_frame(const PROGRAMMER *pgm, unsigned char **msg) {
  int rv, len = 0;
  unsigned char *buf = NULL;
  unsigned char *request;

  pmsg_trace("jtag3_edbg_recv():\n");

  if ((buf = malloc(USBDEV_MAX_XFER_3)) == NULL) {
    pmsg_notice("jtag3_edbg_recv(): out of memory\n");
    return -1;
  }
  if ((request = malloc(pgm->fd.usb.max_xfer)) == NULL) {
    pmsg_notice("jtag3_edbg_recv(): out of memory\n");
    free(buf);
    return -1;
  }

  *msg = buf;

  int nfrags = 0;
  int thisfrag = 0;

  do {
    request[0] = EDBG_VENDOR_AVR_RSP;

    if (serial_send(&pgm->fd, request, pgm->fd.usb.max_xfer) != 0) {
      pmsg_notice("jtag3_edbg_recv(): unable to send CMSIS-DAP vendor command\n");
      free(request);
      free(*msg);
      return -1;
    }

    rv = serial_recv(&pgm->fd, buf, pgm->fd.usb.max_xfer);

    if (rv < 0) {
      /* timeout in receive */
      pmsg_notice2("jtag3_edbg_recv(): timeout receiving packet\n");
      free(*msg);
      free(request);
      return -1;
    }

    if (buf[0] != EDBG_VENDOR_AVR_RSP) {
      pmsg_notice("jtag3_edbg_recv(): unexpected response 0x%02x\n", buf[0]);
      free(*msg);
      free(request);
      return -1;
    }

    if (buf[1] == 0) {
      // Documentation says:
      // "FragmentInfo 0x00 indicates that no response data is
      // available, and the rest of the packet is ignored."
      pmsg_notice("jtag3_edbg_recv(): no response available\n");
      free(*msg);
      free(request);
      return -1;
    }

    /* calculate fragment information */
    if (thisfrag == 0) {
      /* first fragment */
      nfrags = buf[1] & 0x0F;
      thisfrag = 1;
    } else {
      if (nfrags != (buf[1] & 0x0F)) {
        pmsg_notice("jtag3_edbg_recv(): "
          "Inconsistent # of fragments; had %d, now %d\n",
          nfrags, (buf[1] & 0x0F));
        free(*msg);
        free(request);
        return -1;
      }
    }
    if (thisfrag != ((buf[1] >> 4) & 0x0F)) {
      pmsg_notice("jtag3_edbg_recv(): "
        "inconsistent fragment number; expect %d, got %d\n",
        thisfrag, ((buf[1] >> 4) & 0x0F));
      free(*msg);
      free(request);
      return -1;
    }

    int thislen = (buf[2] << 8) | buf[3];
    if (thislen > rv + 4) {
      pmsg_notice("jtag3_edbg_recv(): unexpected length value (%d > %d)\n", thislen, rv + 4);
      thislen = rv + 4;
    }
    if (len + thislen > USBDEV_MAX_XFER_3) {
      pmsg_notice("jtag3_edbg_recv(): length exceeds max size (%d > %d)\n", len + thislen, USBDEV_MAX_XFER_3);
      thislen = USBDEV_MAX_XFER_3 - len;
    }
    memmove(buf, buf + 4, thislen);
    thisfrag++;
    len += thislen;
    buf += thislen;
  } while (thisfrag <= nfrags);

  free(request);
  return len;
}

int jtag3_recv(const PROGRAMMER *pgm, unsigned char **msg) {
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
    pmsg_debug("jtag3_recv(): "
      "Got message seqno %d (command_sequence == %d)\n", r_seqno, PDATA(pgm)->command_sequence);
    if (r_seqno == PDATA(pgm)->command_sequence) {
      if (++(PDATA(pgm)->command_sequence) == 0xffff)
        PDATA(pgm)->command_sequence = 0;
      /*
       * We move the payload to the beginning of the buffer, to make
       * the job easier for the caller.  We have to return the
       * original pointer though, as the caller must free() it.
       */
      rv -= 3;
      if (rv < 0) {
        pmsg_error("unexpected return value %d from jtag3_recv_frame()\n", rv);
        free(*msg);
        return -1;
      }
      memmove(*msg, *msg + 3, rv);

      return rv;
    }
    pmsg_notice2("jtag3_recv(): "
      "got wrong sequence number, %u != %u\n", r_seqno, PDATA(pgm)->command_sequence);

    free(*msg);
  }
}

int jtag3_command(const PROGRAMMER *pgm, unsigned char *cmd, unsigned int cmdlen,
                  unsigned char **resp, const char *descr) {
  int status;
  unsigned char c;

  pmsg_notice2("sending %s command: ", descr);
  jtag3_send(pgm, cmd, cmdlen);

  status = jtag3_recv(pgm, resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_notice2("%s command: timeout/error communicating with programmer (status %d)\n", descr, status);
    if (status == 0)
      free(*resp);
    return LIBAVRDUDE_GENERAL_FAILURE;
  } else if (verbose >= 3) {
    msg_info("\n");
    jtag3_prmsg(pgm, *resp, status);
  } else {
    msg_notice2("0x%02x (%d bytes msg)\n", (*resp)[1], status);
  }

  c = (*resp)[1] & RSP3_STATUS_MASK;
  if (c != RSP3_OK) {
    if ((c == RSP3_FAILED) &&
        ((*resp)[3] == RSP3_FAIL_OCD_LOCKED || (*resp)[3] == RSP3_FAIL_CRC_FAILURE)) {
      pmsg_error("device is locked; chip erase required to unlock\n");
    } else {
      pmsg_notice("bad response to %s command: 0x%02x\n", descr, c);
    }
    status = (*resp)[3];
    free(*resp);
    resp = 0;
    return jtag3_errcode(status);
  }

  return status;
}


int jtag3_getsync(const PROGRAMMER *pgm, int mode) {

  unsigned char buf[3], *resp;

  pmsg_debug("jtag3_getsync()\n");

  /* XplainedMini boards do not need this, and early revisions had a
   * firmware bug where they complained about it. */
  if ((pgm->flag & PGM_FL_IS_EDBG) &&
      !str_starts(pgmid, "xplainedmini")) {
    if (jtag3_edbg_prepare(pgm) < 0) {
      return -1;
    }
  }

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
static int jtag3_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
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
 * UPDI 'unlock' -> 'enter progmode' with chip erase key
 */
static int jtag3_unlock_erase_key(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char buf[8], *resp;

  buf[0] = 1; /* Enable */
  if (jtag3_setparm(pgm, SCOPE_AVR, SET_GET_CTXT_OPTIONS, PARM3_OPT_CHIP_ERASE_TO_ENTER, buf, 1) < 0)
    return -1;

  buf[0] = SCOPE_AVR;
  buf[1] = CMD3_ENTER_PROGMODE;
  buf[2] = 0;

  if (jtag3_command(pgm, buf, 3, &resp, "enter progmode") < 0)
    return -1;
  PDATA(pgm)->prog_enabled = 1;

  buf[0] = 0; /* Disable */
  if (jtag3_setparm(pgm, SCOPE_AVR, SET_GET_CTXT_OPTIONS, PARM3_OPT_CHIP_ERASE_TO_ENTER, buf, 1) < 0)
    return -1;

  free(resp);
  return 0;
}


/*
 * There is no chip erase functionality in debugWire mode.
 */
static int jtag3_chip_erase_dw(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_error("chip erase not supported in debugWire mode\n");
  return 0;
}

static int jtag3_program_enable_dummy(const PROGRAMMER *pgm, const AVRPART *p) {
  return 0;
}

static int jtag3_program_enable(const PROGRAMMER *pgm) {
  unsigned char buf[3], *resp;
  int status;

  if (PDATA(pgm)->prog_enabled)
    return 0;

  buf[0] = SCOPE_AVR;
  buf[1] = CMD3_ENTER_PROGMODE;
  buf[2] = 0;

  if ((status = jtag3_command(pgm, buf, 3, &resp, "enter progmode")) >= 0) {
    free(resp);
    PDATA(pgm)->prog_enabled = 1;

    return LIBAVRDUDE_SUCCESS;
  }

  return status;
}

static int jtag3_program_disable(const PROGRAMMER *pgm) {
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

  return 0;
}

static int jtag3_set_sck_xmega_pdi(const PROGRAMMER *pgm, unsigned char *clk) {
  return jtag3_setparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_PDI, clk, 2);
}

static int jtag3_set_sck_xmega_jtag(const PROGRAMMER *pgm, unsigned char *clk) {
  return jtag3_setparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_JTAG, clk, 2);
}

static int jtag3_set_sck_mega_jtag(const PROGRAMMER *pgm, unsigned char *clk) {
  return jtag3_setparm(pgm, SCOPE_AVR, 1, PARM3_CLK_MEGA_PROG, clk, 2);
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int jtag3_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char conn = 0, parm[4];
  const char *ifname;
  unsigned char cmd[4], *resp;
  int status;

  /*
   * At least, as of firmware 2.12, the JTAGICE3 doesn't handle
   * splitting packets correctly.  On a large transfer, the first
   * split packets are correct, but remaining packets contain just
   * garbage.
   *
   * We move the check here so in case future firmware versions fix
   * this, the check below can be made dependended on the actual
   * firmware level.  Retrieving the firmware version can always be
   * accomplished with USB 1.1 (64 byte max) packets.
   *
   * Allow to override the check by -F (so users could try on newer
   * firmware), but warn loudly.
   */
  if (jtag3_getparm(pgm, SCOPE_GENERAL, 0, PARM3_FW_MAJOR, parm, 2) < 0)
    return -1;
  if (pgm->fd.usb.max_xfer < USBDEV_MAX_XFER_3 && (pgm->flag & PGM_FL_IS_EDBG) == 0) {
    if (ovsigck) {
      pmsg_warning("JTAGICE3's firmware %d.%d is broken on USB 1.1 connections\n", parm[0], parm[1]);
      imsg_warning("forced to continue by option -F; THIS PUTS THE DEVICE'S DATA INTEGRITY AT RISK!\n");
    } else {
      pmsg_error("JTAGICE3's firmware %d.%d is broken on USB 1.1 connections\n", parm[0], parm[1]);
      return -1;
    }
  }

  if (pgm->flag & PGM_FL_IS_DW) {
    ifname = "debugWire";
    if (p->prog_modes & PM_debugWIRE)
      conn = PARM3_CONN_DW;
  } else if (pgm->flag & PGM_FL_IS_PDI) {
    ifname = "PDI";
    if (p->prog_modes & PM_PDI)
      conn = PARM3_CONN_PDI;
  } else if (pgm->flag & PGM_FL_IS_UPDI) {
    ifname = "UPDI";
    if (p->prog_modes & PM_UPDI)
      conn = PARM3_CONN_UPDI;
  } else {
    ifname = "JTAG";
    if (p->prog_modes & (PM_JTAG | PM_JTAGmkI | PM_XMEGAJTAG | PM_AVR32JTAG))
      conn = PARM3_CONN_JTAG;
  }

  if (conn == 0) {
    pmsg_error("part %s has no %s interface\n", p->desc, ifname);
    return -1;
  }

  if (p->prog_modes & PM_PDI)
    parm[0] = PARM3_ARCH_XMEGA;
  else if (p->prog_modes & PM_UPDI)
    parm[0] = PARM3_ARCH_UPDI;
  else if (p->prog_modes & PM_debugWIRE)
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

  if (conn == PARM3_CONN_PDI || conn == PARM3_CONN_UPDI)
    PDATA(pgm)->set_sck = jtag3_set_sck_xmega_pdi;
  else if (conn == PARM3_CONN_JTAG) {
    if (p->prog_modes & PM_PDI)
      PDATA(pgm)->set_sck = jtag3_set_sck_xmega_jtag;
    else
      PDATA(pgm)->set_sck = jtag3_set_sck_mega_jtag;
  }
  if (pgm->bitclock != 0.0 && PDATA(pgm)->set_sck != NULL) {
    unsigned int clock = 1E-3 / pgm->bitclock; /* kHz */
    pmsg_notice2("jtag3_initialize(): "
      "trying to set JTAG clock to %u kHz\n", clock);
    parm[0] = clock & 0xff;
    parm[1] = (clock >> 8) & 0xff;
    if (PDATA(pgm)->set_sck(pgm, parm) < 0)
      return -1;
  }

  if (conn == PARM3_CONN_JTAG) {
    pmsg_notice2("jtag3_initialize(): "
      "trying to set JTAG daisy-chain info to %d,%d,%d,%d\n",
      PDATA(pgm)->jtagchain[0], PDATA(pgm)->jtagchain[1],
      PDATA(pgm)->jtagchain[2], PDATA(pgm)->jtagchain[3]);
    if (jtag3_setparm(pgm, SCOPE_AVR, 1, PARM3_JTAGCHAIN, PDATA(pgm)->jtagchain, 4) < 0)
      return -1;
  }

  if (verbose > 0 && quell_progress < 2)
    jtag3_print_parms1(pgm, progbuf, stderr);

  // Read or write SUFFER register
  if (PDATA(pgm)->suffer_get || PDATA(pgm)->suffer_set) {
    // Read existing SUFFER value
    if (jtag3_getparm(pgm, SCOPE_EDBG, MEDBG_REG_SUFFER_BANK + 0x10, MEDBG_REG_SUFFER_OFFSET, PDATA(pgm)->suffer_data, 1) < 0)
      return -1;
    if (!PDATA(pgm)->suffer_set)
      msg_info("SUFFER register value read as 0x%02x\n", PDATA(pgm)->suffer_data[0]);
    // Write new SUFFER value
    else {
      if (jtag3_setparm(pgm, SCOPE_EDBG, MEDBG_REG_SUFFER_BANK + 0x10, MEDBG_REG_SUFFER_OFFSET, PDATA(pgm)->suffer_data+1, 1) < 0)
        return -1;
      msg_info("SUFFER register value changed from 0x%02x to 0x%02x\n", PDATA(pgm)->suffer_data[0], PDATA(pgm)->suffer_data[1]);
    }
  }

  // Read or write Vtarg switch
  if (PDATA(pgm)->vtarg_switch_get || PDATA(pgm)->vtarg_switch_set) {
    // Read existing Vtarg switch value
    if (jtag3_getparm(pgm, SCOPE_EDBG, EDBG_CTXT_CONTROL, EDBG_CONTROL_TARGET_POWER, PDATA(pgm)->vtarg_switch_data, 1) < 0)
      return -1;
    if (!PDATA(pgm)->vtarg_switch_set)
      msg_info("Vtarg switch setting read as %u: target power is switched %s\n", PDATA(pgm)->vtarg_switch_data[0], PDATA(pgm)->vtarg_switch_data[0] ? "on" : "off");
    // Write Vtarg switch value
    else {
      if (jtag3_setparm(pgm, SCOPE_EDBG, EDBG_CTXT_CONTROL, EDBG_CONTROL_TARGET_POWER, PDATA(pgm)->vtarg_switch_data+1, 1) < 0)
        return -1;
      imsg_info("Vtarg switch setting changed from %u to %u\n", PDATA(pgm)->vtarg_switch_data[0], PDATA(pgm)->vtarg_switch_data[1]);
      // Exit early is the target power switch is off and print sensible info message
      if (PDATA(pgm)->vtarg_switch_data[1] == 0) {
        imsg_info("Turn on the Vtarg switch to establish connection with the target\n\n");
        return -1;
      }
    }
  }

  // Read or write target voltage
  if (PDATA(pgm)->vtarg_get || PDATA(pgm)->vtarg_set) {
    // Read current target voltage set value
    unsigned char buf[2];
    if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_VADJUST, buf, 2) < 0)
      return -1;
    double vtarg_read = b2_to_u16(buf) / 1000.0;
    if (PDATA(pgm)->vtarg_get)
      msg_info("Target voltage value read as %.2fV\n", vtarg_read);
    // Write target voltage value
    else {
      u16_to_b2(buf, (unsigned)(PDATA(pgm)->vtarg_data * 1000));
      msg_info("Changing target voltage from %.2f to %.2fV\n", vtarg_read, PDATA(pgm)->vtarg_data);
      if (jtag3_setparm(pgm, SCOPE_GENERAL, 1, PARM3_VADJUST, buf, sizeof(buf)) < 0) {
        msg_warning("Cannot set target voltage %.2fV\n", PDATA(pgm)->vtarg_data);
        return -1;
      }
    }
  }

  /* set device descriptor data */
  if ((p->prog_modes & PM_PDI)) {
    struct xmega_device_desc xd;
    LNODEID ln;
    AVRMEM * m;
    int fuseinit = 0;

    u16_to_b2(xd.nvm_base_addr, p->nvm_base);
    u16_to_b2(xd.mcu_base_addr, p->mcu_base);

    for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
      m = ldata(ln);
      if (mem_is_flash(m)) {
        if (m->readsize != 0 && m->readsize < m->page_size)
          PDATA(pgm)->flash_pagesize = m->readsize;
        else
          PDATA(pgm)->flash_pagesize = m->page_size;
        u16_to_b2(xd.flash_page_size, m->page_size);
      } else if (mem_is_eeprom(m)) {
        PDATA(pgm)->eeprom_pagesize = m->page_size;
        xd.eeprom_page_size = m->page_size;
        u16_to_b2(xd.eeprom_size, m->size);
        u32_to_b4(xd.nvm_eeprom_offset, m->offset);
      } else if (mem_is_application(m)) {
        u32_to_b4(xd.app_size, m->size);
        u32_to_b4(xd.nvm_app_offset, m->offset);
      } else if (mem_is_boot(m)) {
        u16_to_b2(xd.boot_size, m->size);
        u32_to_b4(xd.nvm_boot_offset, m->offset);
      } else if (mem_is_a_fuse(m) && !fuseinit++) { // Any fuse is OK
        u32_to_b4(xd.nvm_fuse_offset, m->offset & ~15);
      } else if (mem_is_lock(m)) {
        u32_to_b4(xd.nvm_lock_offset, m->offset);
      } else if (mem_is_userrow(m)) {
        u32_to_b4(xd.nvm_user_sig_offset, m->offset);
      } else if (mem_is_sigrow(m)) {
        u32_to_b4(xd.nvm_prod_sig_offset, m->offset);
      }
    }
    if(p->prog_modes & (PM_PDI | PM_UPDI))
      u32_to_b4(xd.nvm_data_offset, DATA_OFFSET);

    if (jtag3_setparm(pgm, SCOPE_AVR, 2, PARM3_DEVICEDESC, (unsigned char *)&xd, sizeof xd) < 0)
      return -1;
  }
  else if ((p->prog_modes & PM_UPDI)) {
    struct updi_device_desc xd;
    LNODEID ln;
    AVRMEM *m;

    u16_to_b2(xd.nvm_base_addr, p->nvm_base);
    u16_to_b2(xd.ocd_base_addr, p->ocd_base);
    xd.hvupdi_variant = p->hvupdi_variant;

    for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
      m = ldata(ln);
      if (mem_is_flash(m)) {
        u16_to_b2(xd.prog_base, m->offset&0xFFFF);
        xd.prog_base_msb = m->offset>>16;

        if (m->readsize != 0 && m->readsize < m->page_size)
          PDATA(pgm)->flash_pagesize = m->readsize;
        else
          PDATA(pgm)->flash_pagesize = m->page_size;
        xd.flash_page_size = m->page_size & 0xFF;
        xd.flash_page_size_msb = (m->page_size)>>8;

        u32_to_b4(xd.flash_bytes, m->size);

        if (m->offset > 0xFFFF)
          xd.address_mode = UPDI_ADDRESS_MODE_24BIT;
        else
          xd.address_mode = UPDI_ADDRESS_MODE_16BIT;
      }
      else if (mem_is_eeprom(m)) {
        PDATA(pgm)->eeprom_pagesize = m->page_size;
        xd.eeprom_page_size = m->page_size;

        u16_to_b2(xd.eeprom_bytes, m->size);
        u16_to_b2(xd.eeprom_base, m->offset);
      }
      else if (mem_is_userrow(m)) {
        u16_to_b2(xd.user_sig_bytes, m->size);
        u16_to_b2(xd.user_sig_base, m->offset);
      }
      else if (mem_is_signature(m)) {
        u16_to_b2(xd.signature_base, m->offset);
        xd.device_id[0] = p->signature[1];
        xd.device_id[1] = p->signature[2];
      }
      else if (mem_is_fuses(m)) {
        xd.fuses_bytes = m->size;
        u16_to_b2(xd.fuses_base, m->offset);
      }
      else if (mem_is_lock(m)) {
        u16_to_b2(xd.lockbits_base, m->offset);
      }
    }

    // Generate UPDI high-voltage pulse if user asks for it and hardware supports it
    if (p->prog_modes & PM_UPDI &&
        PDATA(pgm)->use_hvupdi == true &&
        p->hvupdi_variant != HV_UPDI_VARIANT_1) {
      parm[0] = PARM3_UPDI_HV_NONE;
      for (LNODEID ln = lfirst(pgm->hvupdi_support); ln; ln = lnext(ln)) {
        if(*(int *) ldata(ln) == p->hvupdi_variant) {
          pmsg_notice("sending HV pulse to targets %s pin\n",
                      p->hvupdi_variant == HV_UPDI_VARIANT_0? "UPDI": "RESET");
          parm[0] = PARM3_UPDI_HV_SIMPLE_PULSE;
          break;
        }
      }
      if (parm[0] == PARM3_UPDI_HV_NONE) {
        pmsg_error("%s does not support sending HV pulse to target %s\n", pgm->desc, p->desc);
        return -1;
      }
      if (jtag3_setparm(pgm, SCOPE_AVR, 3, PARM3_OPT_12V_UPDI_ENABLE, parm, 1) < 0)
        return -1;
    }

    u16_to_b2(xd.default_min_div1_voltage, DEFAULT_MINIMUM_CHARACTERISED_DIV1_VOLTAGE_MV);
    u16_to_b2(xd.default_min_div2_voltage, DEFAULT_MINIMUM_CHARACTERISED_DIV2_VOLTAGE_MV);
    u16_to_b2(xd.default_min_div4_voltage, DEFAULT_MINIMUM_CHARACTERISED_DIV4_VOLTAGE_MV);
    u16_to_b2(xd.default_min_div8_voltage, DEFAULT_MINIMUM_CHARACTERISED_DIV8_VOLTAGE_MV);
    u16_to_b2(xd.pdi_pad_fmax, MAX_FREQUENCY_SHARED_UPDI_PIN);
    xd.syscfg_offset = FUSES_SYSCFG0_OFFSET;
    xd.syscfg_write_mask_and = 0xFF;
    xd.syscfg_write_mask_or = 0x00;
    xd.syscfg_erase_mask_and = 0xFF;
    xd.syscfg_erase_mask_or = 0x00;

    msg_notice2("UPDI SET: \n\t"
      "xd->prog_base_msb=%x\n\t"
      "xd->prog_base=%x %x\n\t"
      "xd->flash_page_size_msb=%x\n\t"
      "xd->flash_page_size=%x\n\t"
      "xd->eeprom_page_size=%x\n\t"
      "xd->nvmctrl=%x %x\n\t"
      "xd->ocd=%x %x\n\t"
      "xd->address_mode=%x\n",
      xd.prog_base_msb,
      xd.prog_base[0], xd.prog_base[1],
      xd.flash_page_size_msb,
      xd.flash_page_size,
      xd.eeprom_page_size,
      xd.nvm_base_addr[0], xd.nvm_base_addr[1],
      xd.ocd_base_addr[0], xd.ocd_base_addr[1],
      xd.address_mode);

    if (jtag3_setparm(pgm, SCOPE_AVR, 2, PARM3_DEVICEDESC, (unsigned char *)&xd, sizeof xd) < 0)
      return -1;
  }
  else {
    struct mega_device_desc md;
    LNODEID ln;
    AVRMEM * m;
    unsigned int flashsize = 0;

    memset(&md, 0, sizeof md);

    for (ln = lfirst(p->mem); ln; ln = lnext(ln)) {
      m = ldata(ln);
      if (mem_is_flash(m)) {
        if (m->readsize != 0 && m->readsize < m->page_size)
          PDATA(pgm)->flash_pagesize = m->readsize;
        else
          PDATA(pgm)->flash_pagesize = m->page_size;
        u16_to_b2(md.flash_page_size, m->page_size);
        u32_to_b4(md.flash_size, (flashsize = m->size));
        // do we need it?  just a wild guess
        u32_to_b4(md.boot_address, (m->size - m->page_size * 4) / 2);
      } else if (mem_is_eeprom(m)) {
        PDATA(pgm)->eeprom_pagesize = m->page_size;
        md.eeprom_page_size = m->page_size;
        u16_to_b2(md.eeprom_size, m->size);
      }
    }

    u16_to_b2(md.sram_offset, 0x100);  // do we need it? YES, but it won't be used

    if (p->ocdrev == -1) {
      int ocdrev;

      /* lacking a proper definition, guess the OCD revision */
      if (p->prog_modes & PM_debugWIRE)
        ocdrev = 1;        /* exception: ATtiny13, 2313, 4313 */
      else if (flashsize > 128 * 1024)
        ocdrev = 4;
      else
        ocdrev = 3;        /* many exceptions from that, actually */
      pmsg_warning("part definition for %s lacks ocdrev; guessing %d\n", p->desc, ocdrev);
      md.ocd_revision = ocdrev;
    } else {
      md.ocd_revision = p->ocdrev;
    }
    md.always_one = 1;
    md.allow_full_page_bitstream = (p->flags & AVRPART_ALLOWFULLPAGEBITSTREAM) != 0;
    md.idr_address = p->idr;

    unsigned char eecr = p->eecr? p->eecr: 0x3f; // Use default 0x3f if not set
    md.eearh_address = eecr - 0x20 + 3;
    md.eearl_address = eecr - 0x20 + 2;
    md.eecr_address = eecr - 0x20;
    md.eedr_address = eecr - 0x20 + 1;
    md.spmcr_address = p->spmcr;
    //md.osccal_address = p->osccal;  // do we need it at all?

    if (jtag3_setparm(pgm, SCOPE_AVR, 2, PARM3_DEVICEDESC, (unsigned char *)&md, sizeof md) < 0)
      return -1;
  }

  int use_ext_reset;

  for (use_ext_reset = 0; use_ext_reset <= 1; use_ext_reset++) {
    cmd[0] = SCOPE_AVR;
    cmd[1] = CMD3_SIGN_ON;
    cmd[2] = 0;
    cmd[3] = use_ext_reset;            /* external reset */

    if ((status = jtag3_command(pgm, cmd, 4, &resp, "AVR sign-on")) >= 0)
      break;

    pmsg_notice("retrying with external reset applied\n");
  }

  if (use_ext_reset > 1) {
    if(str_eq(pgm->type, "JTAGICE3") && (p->prog_modes & (PM_JTAG | PM_JTAGmkI | PM_XMEGAJTAG | PM_AVR32JTAG)))
      pmsg_error("JTAGEN fuse disabled?\n");
    return -1;
  }

  /*
   * Depending on the target connection, there are three different
   * possible replies of the ICE.  For a JTAG connection, the reply
   * format is RSP3_DATA, followed by 4 bytes of the JTAG ID read from
   * the device (followed by a trailing 0).
   * For a UPDI connection the reply format is RSP3_DATA, followed by
   * 4 bytes of the SIB Family_ID read from the device (followed by a
   * trailing 0).
   * For all other connections
   * (except ISP which is handled completely differently, but that
   * doesn't apply here anyway), the response is just RSP_OK.
   */
  if (resp[1] == RSP3_DATA && status >= 7) {
    if (p->prog_modes & PM_UPDI) {
      /* Partial Family_ID has been returned */
      pmsg_notice("partial Family_ID returned: \"%c%c%c%c\"\n",
                  resp[3], resp[4], resp[5], resp[6]);
    }
    else
      /* JTAG ID has been returned */
      pmsg_notice("JTAG ID returned: 0x%02x 0x%02x 0x%02x 0x%02x\n",
                  resp[3], resp[4], resp[5], resp[6]);
  }

  free(resp);

  if (pgm->read_sib) {
    if (pgm->read_sib(pgm, p, PDATA(pgm)->sib_string) < 0) {
      pmsg_warning("cannot read SIB string from target %s\n", p->desc);
    }
  }

  // Read chip silicon revision
  if(pgm->read_chip_rev && p->prog_modes & (PM_PDI | PM_UPDI)) {
    unsigned char chip_rev[AVR_CHIP_REVLEN];
    pgm->read_chip_rev(pgm, p, chip_rev);
    pmsg_notice("silicon revision: %x.%x\n", chip_rev[0] >> 4, chip_rev[0] & 0x0f);
  }

  PDATA(pgm)->boot_start = ULONG_MAX;
  if (p->prog_modes & PM_PDI) {
    // Find the border between application and boot area
    AVRMEM *bootmem = avr_locate_boot(p);
    AVRMEM *flashmem = avr_locate_flash(p);
    if (bootmem == NULL || flashmem == NULL) {
      pmsg_error("cannot locate flash or boot memories in description\n");
    } else {
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

  return 0;
}

static void jtag3_disable(const PROGRAMMER *pgm) {

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

static void jtag3_enable(PROGRAMMER *pgm, const AVRPART *p) {
  // Page erase only useful for classic parts with usersig mem or AVR8X/XMEGAs
  if(!(p->prog_modes & (PM_PDI | PM_UPDI)))
    if(!avr_locate_usersig(p))
      pgm->page_erase = NULL;
}

static int jtag3_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int rv = 0;

  for(ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if(str_starts(extended_param, "jtagchain=") && (pgm->prog_modes & (PM_JTAG | PM_XMEGAJTAG | PM_AVR32JTAG))) {
      unsigned int ub, ua, bb, ba;
      if(sscanf(extended_param, "jtagchain=%u,%u,%u,%u", &ub, &ua, &bb, &ba) != 4) {
        pmsg_error("invalid JTAG chain %s\n", extended_param);
        rv = -1;
        break;
      }
      pmsg_notice2("jtag3_parseextparms(): JTAG chain parsed as:\n");
      imsg_notice2("%u units before, %u units after, %u bits before, %u bits after\n",
                   ub, ua, bb, ba);
      PDATA(pgm)->jtagchain[0] = ub;
      PDATA(pgm)->jtagchain[1] = ua;
      PDATA(pgm)->jtagchain[2] = bb;
      PDATA(pgm)->jtagchain[3] = ba;
      continue;
    }

    // HVUPDI
    // All programmers that supports UPDI programming should have hvupdi_support=1 in avrdude.conf
    // Type 0: 12V pulse on UPDI pin
    // Type 1: No HV UPDI
    // Type 2: 12V pulse on RESET pin
    if(str_starts(extended_param, "hvupdi")) {
      if(lsize(pgm->hvupdi_support) < 1) {
        pmsg_error("programmer does not support high voltage UPDI programming\n");
        rv = -1;
        break;
      }
      if(!str_eq(extended_param, "hvupdi")) {
        pmsg_error("invalid -xhvupdi value %s. Use -xhvupdi\n", extended_param);
        rv = -1;
        break;
      }
      PDATA(pgm)->use_hvupdi = true;
      continue;
    }

    // SUFFER bits
    // Bit 7 ARDUINO: Adds control of extra LEDs when set to 0
    // Bit 6..3: Reserved (must be set to 1)
    // Bit 2 EOF: Agressive power-down, sleep after 5 seconds if no USB enumeration when set to 0
    // Bit 1 LOWP: forces running at 1 MHz when bit set to 0
    // Bit 0 FUSE: Fuses are safe-masked when bit sent to 1 Fuses are unprotected when set to 0
    if(str_starts(extended_param, "suffer")) {
      if(pgm->extra_features & HAS_SUFFER) {
        // Set SUFFER value
        if(str_starts(extended_param, "suffer=")) {
          if(sscanf(extended_param, "suffer=%hhi", PDATA(pgm)->suffer_data+1) < 1) {
            pmsg_error("invalid -xsuffer=<value> %s\n", extended_param);
            rv = -1;
            break;
          }
          if((PDATA(pgm)->suffer_data[1] & 0x78) != 0x78) {
            PDATA(pgm)->suffer_data[1] |= 0x78;
            pmsg_info("setting -xsuffer=0x%02x so that reserved bits 3..6 are set\n",
              PDATA(pgm)->suffer_data[1]);
          }
          PDATA(pgm)->suffer_set = true;
          continue;
        }
        // Get SUFFER value
        if(str_eq(extended_param, "suffer")) {
          PDATA(pgm)->suffer_get = true;
          continue;
        }
        pmsg_error("invalid suffer setting %s. Use -xsuffer or -xsuffer=<arg>\n", extended_param);
        rv = -1;
        break;
      }
    }

    if(str_starts(extended_param, "vtarg_switch")) {
      if(pgm->extra_features & HAS_VTARG_SWITCH) {
        // Set Vtarget switch value
        if(str_starts(extended_param, "vtarg_switch=")) {
          int sscanf_success = sscanf(extended_param, "vtarg_switch=%hhi", PDATA(pgm)->vtarg_switch_data+1);
          if(sscanf_success < 1 || PDATA(pgm)->vtarg_switch_data[1] > 1) {
            pmsg_error("invalid vtarg_switch value %s\n", extended_param);
            rv = -1;
            break;
          }
          PDATA(pgm)->vtarg_switch_set = true;
          continue;
        }
        // Get Vtarget switch value
        if(str_eq(extended_param, "vtarg_switch")) {
          PDATA(pgm)->vtarg_switch_get = true;
          continue;
        }
        pmsg_error("invalid vtarg_switch setting %s. Use -xvtarg_switch or -xvtarg_switch=<0..1>\n", extended_param);
        rv = -1;
        break;
      }
    }

    if(str_starts(extended_param, "vtarg")) {
      if(pgm->extra_features & HAS_VTARG_ADJ) {
        // Set target voltage
        if(str_starts(extended_param, "vtarg=") ) {
          double vtarg_set_val = 0;
          int sscanf_success = sscanf(extended_param, "vtarg=%lf", &vtarg_set_val);
          PDATA(pgm)->vtarg_data = (double)((int)(vtarg_set_val * 100 + .5)) / 100;
          if(sscanf_success < 1 || vtarg_set_val < 0) {
            pmsg_error("invalid vtarg value %s\n", extended_param);
            rv = -1;
            break;
          }
          PDATA(pgm)->vtarg_set = true;
          continue;
        }
        // Get target voltage
        else if(str_eq(extended_param, "vtarg")) {
          PDATA(pgm)->vtarg_get = true;
          continue;
        }
        pmsg_error("invalid vtarg setting %s. Use -xvtarg or -xvtarg=<arg>\n", extended_param);
        rv = -1;
        break;
      }
    }

    if(str_starts(extended_param, "mode") &&
      (str_starts(pgmid, "pickit4") || str_starts(pgmid, "snap"))) {
      // Flag a switch to AVR mode
      if(str_caseeq(extended_param, "mode=avr")) {
        PDATA(pgm)->pk4_snap_mode = PK4_SNAP_MODE_AVR;
        continue;
      }
      // Flag a switch to PIC mode
      if(str_caseeq(extended_param, "mode=pic")) {
        PDATA(pgm)->pk4_snap_mode = PK4_SNAP_MODE_PIC;
        continue;
      }
      pmsg_error("invalid mode setting %s. Use -xmode=avr or -xmode=pic\n", extended_param);
      rv = -1;
      break;
    }

    if(str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      if(str_eq(pgm->type, "JTAGICE3"))
        msg_error("  -xjtagchain=UB,UA,BB,BA Setup the JTAG scan chain order\n");
      if(lsize(pgm->hvupdi_support) > 1)
        msg_error("  -xhvupdi                Enable high-voltage UPDI initialization\n");
      if(pgm->extra_features & HAS_SUFFER) {
        msg_error("  -xsuffer                Read SUFFER register value\n");
        msg_error("  -xsuffer=<arg>          Set SUFFER register value\n");
      }
      if(pgm->extra_features & HAS_VTARG_SWITCH) {
        msg_error("  -xvtarg_switch          Read on-board target voltage switch state\n");
        msg_error("  -xvtarg_switch=<0..1>   Set on-board target voltage switch state\n");
      }
      if(pgm->extra_features & HAS_VTARG_ADJ) {
        msg_error("  -xvtarg                 Read on-board target supply voltage\n");
        msg_error("  -xvtarg=<arg>           Set on-board target supply voltage\n");
      }
      if(str_starts(pgmid, "pickit4") || str_starts(pgmid, "snap"))
        msg_error("  -xmode=avr|pic          Set programmer to AVR or PIC mode, then exit\n");
      msg_error  ("  -xhelp                  Show this help menu and exit\n");
      exit(0);
    }

    pmsg_error("invalid extended parameter %s\n", extended_param);
    rv = -1;
  }

  return rv;
}

int jtag3_open_common(PROGRAMMER *pgm, const char *port, int mode_switch) {
  union pinfo pinfo;
  LNODEID usbpid;
  int rv = -1;

#if !defined(HAVE_LIBUSB) && !defined(HAVE_LIBHIDAPI)
  pmsg_error("was compiled without USB or HIDAPI support\n");
  return -1;
#endif

  if (!str_starts(port, "usb")) {
    pmsg_error("JTAGICE3/EDBG port names must start with usb\n");
    return -1;
  }

  // If the config entry did not specify a USB PID, insert the default one.
  if (lfirst(pgm->usbpid) == NULL)
    ladd(pgm->usbpid, (void *)USB_DEVICE_JTAGICE3);

  pinfo.usbinfo.vid = pgm->usbvid? pgm->usbvid: USB_VENDOR_ATMEL;

#if defined(HAVE_LIBHIDAPI)
  // Try HIDAPI first. LibUSB is more generic, but might
  // cause trouble for HID-class devices in some OSes
  serdev = &usbhid_serdev;
  for (usbpid = lfirst(pgm->usbpid); rv < 0 && usbpid != NULL; usbpid = lnext(usbpid)) {
    pinfo.usbinfo.flags = PINFO_FL_SILENT;
    pinfo.usbinfo.pid = *(int *)(ldata(usbpid));
    pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_3;
    pgm->fd.usb.rep = USBDEV_BULK_EP_READ_3;
    pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_3;
    pgm->fd.usb.eep = 0;

    strcpy(pgm->port, port);
    rv = serial_open(port, pinfo, &pgm->fd);
  }
  if (rv < 0) {
#endif    /* HAVE_LIBHIDAPI */
#if defined(HAVE_LIBUSB)
    serdev = &usb_serdev_frame;
    for (usbpid = lfirst(pgm->usbpid); rv < 0 && usbpid != NULL; usbpid = lnext(usbpid)) {
      pinfo.usbinfo.flags = PINFO_FL_SILENT;
      pinfo.usbinfo.pid = *(int *)(ldata(usbpid));
      pgm->fd.usb.max_xfer = USBDEV_MAX_XFER_3;
      pgm->fd.usb.rep = USBDEV_BULK_EP_READ_3;
      pgm->fd.usb.wep = USBDEV_BULK_EP_WRITE_3;
      pgm->fd.usb.eep = USBDEV_EVT_EP_READ_3;

      strcpy(pgm->port, port);
      rv = serial_open(port, pinfo, &pgm->fd);
    }
#endif    /* HAVE_LIBUSB */
#if defined(HAVE_LIBHIDAPI)
  }
#endif
  if (rv < 0) {
    // Check if SNAP or PICkit4 are in PIC mode
    for(LNODEID ln=lfirst(pgm->id); ln; ln=lnext(ln)) {
      if (str_starts(ldata(ln), "snap") || str_starts(ldata(ln), "pickit4")) {
        bool is_snap_pgm = str_starts(ldata(ln), "snap");
        pinfo.usbinfo.vid = USB_VENDOR_MICROCHIP;
        pinfo.usbinfo.pid = is_snap_pgm? USB_DEVICE_SNAP_PIC_MODE: USB_DEVICE_PICKIT4_PIC_MODE;
        const int bl_pid = is_snap_pgm? USB_DEVICE_SNAP_PIC_MODE_BL: USB_DEVICE_PICKIT4_PIC_MODE_BL;
        const char *pgmstr = is_snap_pgm? "MPLAB SNAP": "PICkit 4";
        const unsigned char exit_bl_cmd[] = {0xe6};
        const unsigned char enter_avr_mode_cmd[] = {0xf0, 0x01};
        const unsigned char reset_cmd[] = {0xed};

        int pic_mode = serial_open(port, pinfo, &pgm->fd);
        if(pic_mode < 0) {
          // Retry with bootloader USB PID
          pinfo.usbinfo.pid = bl_pid;
          pic_mode = serial_open(port, pinfo, &pgm->fd);
        }
        if(pic_mode >= 0) {
          msg_error("\n");
          pmsg_error("%s in %s mode detected\n",
            pgmstr, pinfo.usbinfo.pid == bl_pid? "bootloader": "PIC");
          if(mode_switch == PK4_SNAP_MODE_AVR) {
            imsg_error("switching to AVR mode\n");
            if(pinfo.usbinfo.pid == bl_pid)
              serial_send(&pgm->fd, exit_bl_cmd, sizeof(exit_bl_cmd));
            else {
              serial_send(&pgm->fd, enter_avr_mode_cmd, sizeof(enter_avr_mode_cmd));
              usleep(250*1000);
              serial_send(&pgm->fd, reset_cmd, sizeof(reset_cmd));
            }
            imsg_error("please run Avrdude again to continue the session\n\n");
          } else {
            imsg_error("to switch into AVR mode try\n");
            imsg_error("avrdude -c%s -p%s -P%s -xmode=avr\n", pgmid, partdesc, port);
          }
          serial_close(&pgm->fd);
          exit(0);
        }
      }
    }
    pmsg_error("no device found matching VID 0x%04x and PID list: ",
               (unsigned) pinfo.usbinfo.vid);
    int notfirst = 0;
    for (usbpid = lfirst(pgm->usbpid); usbpid; usbpid = lnext(usbpid)) {
      if (notfirst)
        msg_error(", ");
      msg_error("0x%04x", (unsigned int)(*(int *)(ldata(usbpid))));
      notfirst = 1;
    }

    char *serno;
    if ((serno = strchr(port, ':')))
      msg_error(" with SN %s", ++serno);
    msg_error("\n");

    return -1;
  }

  if (mode_switch == PK4_SNAP_MODE_AVR)
    pmsg_warning("programmer is already in AVR mode. Ignoring -xmode");

  // The event EP has been deleted by usb_open(), so we are
  // running on a CMSIS-DAP device, using EDBG protocol
  if (pgm->fd.usb.eep == 0) {
    pgm->flag |= PGM_FL_IS_EDBG;
    pmsg_notice2("found CMSIS-DAP compliant device, using EDBG protocol\n");
  }

  // Make USB serial number available to programmer
  if (serdev && serdev->usbsn)
    pgm->usbsn = serdev->usbsn;

  // Drain any extraneous input
  jtag3_drain(pgm, 0);

  // Switch from AVR to PIC mode
  if (mode_switch == PK4_SNAP_MODE_PIC) {
    imsg_error("switching to PIC mode\n");
    unsigned char *resp, buf[] = {SCOPE_GENERAL, CMD3_FW_UPGRADE, 0x00, 0x00, 0x70, 0x6d, 0x6a};
    if (jtag3_command(pgm, buf, sizeof(buf), &resp, "enter PIC mode") < 0) {
      imsg_error("entering PIC mode failed\n");
      return -1;
    }
    imsg_error("PIC mode switch successful\n");
    serial_close(&pgm->fd);
    exit(0);
  }

  return 0;
}



static int jtag3_open(PROGRAMMER *pgm, const char *port) {
  pmsg_notice2("jtag3_open()\n");

  int rc = jtag3_open_common(pgm, port, PDATA(pgm)->pk4_snap_mode);
  if (rc < 0)
    return rc;

  if (jtag3_getsync(pgm, PARM3_CONN_JTAG) < 0)
    return -1;

  return 0;
}

static int jtag3_open_dw(PROGRAMMER *pgm, const char *port) {
  pmsg_notice2("jtag3_open_dw()\n");

  if (jtag3_open_common(pgm, port, PDATA(pgm)->pk4_snap_mode) < 0)
    return -1;

  if (jtag3_getsync(pgm, PARM3_CONN_DW) < 0)
    return -1;

  return 0;
}

static int jtag3_open_pdi(PROGRAMMER *pgm, const char *port) {
  pmsg_notice2("jtag3_open_pdi()\n");

  if (jtag3_open_common(pgm, port, PDATA(pgm)->pk4_snap_mode) < 0)
    return -1;

  if (jtag3_getsync(pgm, PARM3_CONN_PDI) < 0)
    return -1;

  return 0;
}

static int jtag3_open_updi(PROGRAMMER *pgm, const char *port) {
  pmsg_notice2("jtag3_open_updi()\n");

  LNODEID ln;
  pmsg_notice2("HV UPDI support:");
  for (ln = lfirst(pgm->hvupdi_support); ln; ln = lnext(ln))
    msg_notice2(" %d", *(int *) ldata(ln));
  msg_notice2("\n");

  if (jtag3_open_common(pgm, port, PDATA(pgm)->pk4_snap_mode) < 0)
    return -1;

  if (jtag3_getsync(pgm, PARM3_CONN_UPDI) < 0)
    return -1;

  return 0;
}

void jtag3_close(PROGRAMMER * pgm) {
  unsigned char buf[4], *resp;

  pmsg_notice2("jtag3_close()\n");

  buf[0] = SCOPE_AVR;
  buf[1] = CMD3_SIGN_OFF;
  buf[2] = buf[3] = 0;

  if (jtag3_command(pgm, buf, 3, &resp, "AVR sign-off") >= 0)
    free(resp);

  buf[0] = SCOPE_GENERAL;
  buf[1] = CMD3_SIGN_OFF;

  if (jtag3_command(pgm, buf, 4, &resp, "sign-off") >= 0)
    free(resp);

  /* XplainedMini boards do not need this, and early revisions had a
   * firmware bug where they complained about it. */
  if ((pgm->flag & PGM_FL_IS_EDBG) &&
      !str_starts(pgmid, "xplainedmini")) {
    jtag3_edbg_signoff(pgm);
  }

  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}

static int jtag3_page_erase(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                            unsigned int addr) {
  unsigned char cmd[8], *resp;

  pmsg_notice2("jtag3_page_erase(.., %s, 0x%x)\n", m->desc, addr);

  if(!(p->prog_modes & (PM_PDI | PM_UPDI)) && !mem_is_userrow(m)) {
    pmsg_error("page erase only available for AVR8X/XMEGAs or classic-part usersig mem\n");
    return -1;
  }

  if (jtag3_program_enable(pgm) < 0)
    return -1;

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_ERASE_MEMORY;
  cmd[2] = 0;

  if (mem_is_in_flash(m)) {
    if (p->prog_modes & PM_UPDI || jtag3_mtype(pgm, p, addr) == MTYPE_FLASH)
      cmd[3] = XMEGA_ERASE_APP_PAGE;
    else
      cmd[3] = XMEGA_ERASE_BOOT_PAGE;
  } else if (mem_is_eeprom(m)) {
    cmd[3] = XMEGA_ERASE_EEPROM_PAGE;
  } else if (mem_is_userrow(m)) {
    cmd[3] = XMEGA_ERASE_USERSIG;
  } else {
    cmd[3] = XMEGA_ERASE_APP_PAGE;
  }

  unsigned int addr_adj = addr;
  if(p->prog_modes & PM_PDI)
    addr_adj += m->offset;
  else // PM_UPDI
    addr_adj = jtag3_memaddr(pgm, p, m, addr);

  u32_to_b4(cmd + 4, addr_adj);
  if (jtag3_command(pgm, cmd, 8, &resp, "page erase") < 0)
    return -1;

  free(resp);
  return 0;
}

static int jtag3_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes) {
  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char *cmd;
  unsigned char *resp;
  int status, dynamic_mtype = 0;
  long otimeout = serial_recv_timeout;

  pmsg_notice2("jtag3_paged_write(.., %s, %d, 0x%04x, %d)\n", m->desc, page_size, addr, n_bytes);

  block_size = jtag3_memaddr(pgm, p, m, addr);
  if(block_size != addr)
    imsg_notice2("mapped to address: 0x%04x\n", block_size);
  block_size = 0;

  if (!(pgm->flag & PGM_FL_IS_DW) && jtag3_program_enable(pgm) < 0)
    return -1;

  if (page_size == 0)
    page_size = 256;

  if ((cmd = malloc(page_size + 13)) == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_WRITE_MEMORY;
  cmd[2] = 0;
  if (mem_is_flash(m)) {
    PDATA(pgm)->flash_pageaddr = (unsigned long)-1L;
    cmd[3] = jtag3_mtype(pgm, p, addr);
    if (p->prog_modes & PM_PDI)
      /* dynamically decide between flash/boot mtype */
      dynamic_mtype = 1;
  } else if (mem_is_eeprom(m)) {
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
    cmd[3] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_EEPROM_XMEGA: MTYPE_EEPROM_PAGE;
    PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;
  } else if (mem_is_userrow(m)) {
    cmd[3] = MTYPE_USERSIG;
  } else if (mem_is_boot(m)) {
    cmd[3] = MTYPE_BOOT_FLASH;
  } else if (p->prog_modes & (PM_PDI | PM_UPDI)) {
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
    pmsg_debug("jtag3_paged_write(): "
      "block_size at addr %d is %d\n", addr, block_size);

    if (dynamic_mtype)
      cmd[3] = jtag3_mtype(pgm, p, addr);

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

static int jtag3_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                            unsigned int page_size,
                            unsigned int addr, unsigned int n_bytes) {
  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char cmd[12];
  unsigned char *resp;
  int status, dynamic_mtype = 0;
  long otimeout = serial_recv_timeout;

  pmsg_notice2("jtag3_paged_load(.., %s, %d, 0x%04x, %d)\n",
               m->desc, page_size, addr, n_bytes);

  block_size = jtag3_memaddr(pgm, p, m, addr);
  if(block_size != addr)
    imsg_notice2("mapped to address: 0x%04x\n", block_size);
  block_size = 0;

  if (!(pgm->flag & PGM_FL_IS_DW) && jtag3_program_enable(pgm) < 0)
    return -1;

  page_size = m->readsize;

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_READ_MEMORY;
  cmd[2] = 0;

  if (mem_is_flash(m)) {
    cmd[3] = jtag3_mtype(pgm, p, addr);
    if (p->prog_modes & PM_PDI)
      /* dynamically decide between flash/boot mtype */
      dynamic_mtype = 1;
  } else if (mem_is_eeprom(m)) {
    cmd[3] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_EEPROM: MTYPE_EEPROM_PAGE;
    if (pgm->flag & PGM_FL_IS_DW)
      return -1;
  } else if (mem_is_sigrow(m)) {
    cmd[3] = MTYPE_PRODSIG;
  } else if (mem_is_userrow(m)) {
    cmd[3] = MTYPE_USERSIG;
  } else if (mem_is_boot(m)) {
    cmd[3] = MTYPE_BOOT_FLASH;
  } else if (p->prog_modes & PM_PDI) {
    cmd[3] = MTYPE_FLASH;
  } else if (p->prog_modes & PM_UPDI) {
    cmd[3] = MTYPE_SRAM;
  } else {
    cmd[3] = MTYPE_SPM;
  }
  serial_recv_timeout = 100;
  for (; addr < maxaddr; addr += page_size) {
    if ((maxaddr - addr) < page_size)
      block_size = maxaddr - addr;
    else
      block_size = page_size;
    pmsg_debug("jtag3_paged_load(): "
               "block_size at addr %d is %d\n", addr, block_size);

    if (dynamic_mtype)
      cmd[3] = jtag3_mtype(pgm, p, addr);

    u32_to_b4(cmd + 8, block_size);
    u32_to_b4(cmd + 4, jtag3_memaddr(pgm, p, m, addr));

    if ((status = jtag3_command(pgm, cmd, 12, &resp, "read memory")) < 0)
      return -1;

    if (resp[1] != RSP3_DATA || status < (int) block_size + 4) {
      pmsg_error("wrong/short reply to read memory command\n");
      serial_recv_timeout = otimeout;
      free(resp);
      return -1;
    }

    if(status < 4) {
      pmsg_error("unexpected response from read memory jtag3_command()\n");
      free(resp);
      return -1;
    }

    memcpy(m->buf + addr, resp + 3, status - 4);
    free(resp);
  }
  serial_recv_timeout = otimeout;

  return n_bytes;
}

static int jtag3_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                           unsigned long addr, unsigned char * value) {
  unsigned char cmd[12];
  unsigned char *resp, *cache_ptr = NULL;
  int status, unsupp = 0;
  unsigned long paddr = 0UL, *paddr_ptr = NULL;
  unsigned int pagesize = 0;

  pmsg_notice2("jtag3_read_byte(.., %s, 0x%lx, ...)\n", mem->desc, addr);

  paddr = jtag3_memaddr(pgm, p, mem, addr);
  if (paddr != addr)
    imsg_notice2("mapped to address: 0x%lx\n", paddr);
  paddr = 0;

  if (mem->size < 1) {
    pmsg_error("cannot read byte from %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if (addr >= (unsigned long) mem->size) {
    pmsg_error("cannot read byte from %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size-1);
    return -1;
  }

  if (!(pgm->flag & PGM_FL_IS_DW))
    if ((status = jtag3_program_enable(pgm)) < 0)
      return status;

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_READ_MEMORY;
  cmd[2] = 0;

  cmd[3] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_FLASH: MTYPE_FLASH_PAGE;
  if (mem_is_in_flash(mem)) {
    addr += mem->offset & (512 * 1024 - 1); /* max 512 KiB flash */
    pagesize = PDATA(pgm)->flash_pagesize;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &PDATA(pgm)->flash_pageaddr;
    cache_ptr = PDATA(pgm)->flash_pagecache;
  } else if (mem_is_eeprom(mem)) {
    if ( (pgm->flag & PGM_FL_IS_DW) || (p->prog_modes & PM_PDI) || (p->prog_modes & PM_UPDI) ) {
      cmd[3] = MTYPE_EEPROM;
    } else {
      cmd[3] = MTYPE_EEPROM_PAGE;
    }
    pagesize = mem->page_size;
    paddr = addr & ~(pagesize - 1);
    paddr_ptr = &PDATA(pgm)->eeprom_pageaddr;
    cache_ptr = PDATA(pgm)->eeprom_pagecache;
  } else if (mem_is_a_fuse(mem) || mem_is_fuses(mem)) {
    cmd[3] = MTYPE_FUSE_BITS;
    if(!(p->prog_modes & PM_UPDI) && mem_is_a_fuse(mem))
      addr = mem_fuse_offset(mem);
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_lock(mem)) {
    cmd[3] = MTYPE_LOCK_BITS;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_userrow(mem)) {
    cmd[3] = MTYPE_USERSIG;
  } else if (mem_is_sigrow(mem)) {
    if (p->prog_modes & (PM_PDI | PM_UPDI)) {
      cmd[3] = MTYPE_PRODSIG;
    } else {
      cmd[3] = addr&1? MTYPE_OSCCAL_BYTE: MTYPE_SIGN_JTAG;
      addr /= 2;
      if (pgm->flag & PGM_FL_IS_DW)
        unsupp = 1;
    }
  } else if (mem_is_sernum(mem)) {
    cmd[3] = MTYPE_SIGN_JTAG;
  } else if (mem_is_osccal16(mem)) {
    cmd[3] = MTYPE_SIGN_JTAG;
  } else if (mem_is_osccal20(mem)) {
    cmd[3] = MTYPE_SIGN_JTAG;
  } else if (mem_is_tempsense(mem)) {
    cmd[3] = MTYPE_SIGN_JTAG;
  } else if (mem_is_osc16err(mem)) {
    cmd[3] = MTYPE_SIGN_JTAG;
  } else if (mem_is_osc20err(mem)) {
    cmd[3] = MTYPE_SIGN_JTAG;
  } else if (mem_is_calibration(mem)) {
    cmd[3] = MTYPE_OSCCAL_BYTE;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_io(mem) || mem_is_sram(mem)) {
    cmd[3] = MTYPE_SRAM;
  } else if (mem_is_sib(mem)) {
    if(addr >= AVR_SIBLEN) {
      pmsg_error("cannot read byte from %s sib as address 0x%04lx outside range [0, 0x%04x]\n",
        p->desc, addr, AVR_SIBLEN-1);
      return -1;
    }
    if(!*PDATA(pgm)->sib_string) {
      pmsg_error("cannot read byte from %s sib as memory not initialised\n", p->desc);
      return -1;
    }
    *value = PDATA(pgm)->sib_string[addr];
    return 0;
  } else if (mem_is_signature(mem)) {
    static unsigned char signature_cache[2];

    cmd[3] = MTYPE_SIGN_JTAG;

    /*
     * dW can read out the signature on JTAGICE3, but only allows
     * for a full three-byte read.  We cache them in a local
     * variable to avoid multiple reads.  This optimization does not
     * harm for other connection types either.
     */
    u32_to_b4(cmd + 8, 3);
    u32_to_b4(cmd + 4, jtag3_memaddr(pgm, p, mem, addr));

    if (addr == 0) {
      if ((status = jtag3_command(pgm, cmd, 12, &resp, "read memory")) < 0)
        return status;

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
      msg_error("address out of range for signature memory: %lu\n", addr);
      return -1;
    }
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
    u32_to_b4(cmd + 8, pagesize);
    u32_to_b4(cmd + 4, jtag3_memaddr(pgm, p, mem, paddr));

  } else {
    u32_to_b4(cmd + 8, 1);
    u32_to_b4(cmd + 4, jtag3_memaddr(pgm, p, mem, addr));
  }

  if ((status = jtag3_command(pgm, cmd, 12, &resp, "read memory")) < 0)
    return status;

  if (resp[1] != RSP3_DATA ||
      status < (int) (pagesize? pagesize: 1) + 4) {
    pmsg_error("wrong/short reply to read memory command\n");
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

static int jtag3_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                            unsigned long addr, unsigned char data) {
  unsigned char cmd[14];
  unsigned char *resp;
  unsigned char *cache_ptr = 0;
  int status, unsupp = 0;
  unsigned int pagesize = 0;
  unsigned long mapped_addr;

  pmsg_notice2("jtag3_write_byte(.., %s, 0x%lx, ...)\n", mem->desc, addr);

  mapped_addr = jtag3_memaddr(pgm, p, mem, addr);
  if(mapped_addr != addr)
    imsg_notice2("mapped to address: 0x%lx\n", mapped_addr);

  if(mem->size < 1) {
    pmsg_error("cannot write byte to %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  } else if(addr >= (unsigned long) mem->size) {
    pmsg_error("cannot write byte to %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size-1);
    return -1;
  }

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_WRITE_MEMORY;
  cmd[2] = 0;
  cmd[3] = p->prog_modes & (PM_PDI | PM_UPDI)? MTYPE_FLASH: MTYPE_SPM;
  if (mem_is_flash(mem)) {
     cache_ptr = PDATA(pgm)->flash_pagecache;
     pagesize = PDATA(pgm)->flash_pagesize;
     PDATA(pgm)->flash_pageaddr = (unsigned long)-1L;
     if (pgm->flag & PGM_FL_IS_DW)
       unsupp = 1;
  } else if (mem_is_eeprom(mem)) {
    if (pgm->flag & PGM_FL_IS_DW) {
      cmd[3] = MTYPE_EEPROM;
    } else {
      cache_ptr = PDATA(pgm)->eeprom_pagecache;
      pagesize = PDATA(pgm)->eeprom_pagesize;
    }
    PDATA(pgm)->eeprom_pageaddr = (unsigned long)-1L;
  } else if (mem_is_a_fuse(mem) || mem_is_fuses(mem)) {
    cmd[3] = MTYPE_FUSE_BITS;
    if(!(p->prog_modes & PM_UPDI) && mem_is_a_fuse(mem))
      addr = mem_fuse_offset(mem);
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_lock(mem)) {
    cmd[3] = MTYPE_LOCK_BITS;
    if (pgm->flag & PGM_FL_IS_DW)
      unsupp = 1;
  } else if (mem_is_userrow(mem)) {
    cmd[3] = MTYPE_USERSIG;
  } else if (mem_is_io(mem) || mem_is_sram(mem))
    cmd[3] = MTYPE_SRAM;

  // Read-only memories or unsupported by debugWire
  if(mem_is_readonly(mem) || unsupp) {
      unsigned char is;
      if(jtag3_read_byte(pgm, p, mem, addr, &is) >= 0 && is == data)
        return 0;
      if (unsupp && pgm->flag & PGM_FL_IS_DW)
        pmsg_error("debugWire interface does not support writing to memory %s\n", mem->desc);
      else
        pmsg_error("cannot write to read-only memory %s of %s\n", mem->desc, p->desc);
      return -1;
   }

  if (pagesize != 0) {
    /* flash or EEPROM write: use paged algorithm */
    unsigned char dummy;
    int i;

    /* step #1: ensure the page cache is up to date */
    if (jtag3_read_byte(pgm, p, mem, addr, &dummy) < 0)
      return -1;
    /* step #2: update our value in page cache, and copy
     * cache to mem->buf */
    cache_ptr[addr & (pagesize - 1)] = data;
    addr &= ~(pagesize - 1);    /* page base address */
    memcpy(mem->buf + addr, cache_ptr, pagesize);
    /* step #3: write back */
    i = jtag3_paged_write(pgm, p, mem, pagesize, addr, pagesize);

    return i < 0? -1: 0;
  }

  /* non-paged writes go here */
  if (!(pgm->flag & PGM_FL_IS_DW) && jtag3_program_enable(pgm) < 0)
    return -1;

  u32_to_b4(cmd + 8, 1);
  u32_to_b4(cmd + 4, jtag3_memaddr(pgm, p, mem, addr));
  cmd[12] = 0;
  cmd[13] = data;

  if ((status = jtag3_command(pgm, cmd, 14, &resp, "write memory")) < 0)
    return status;

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
static int jtag3_set_sck_period(const PROGRAMMER *pgm, double v) {
  unsigned char parm[2];
  unsigned int clock = 1E-3 / v; /* kHz */

  parm[0] = clock & 0xff;
  parm[1] = (clock >> 8) & 0xff;

  if (PDATA(pgm)->set_sck == NULL) {
    pmsg_error("no backend to set the SCK period for\n");
    return -1;
  }

  return (PDATA(pgm)->set_sck(pgm, parm) < 0)? -1: 0;
}


static int jtag3_get_sck_period(const PROGRAMMER *pgm, double *v) {
  unsigned char conn, arch;
  unsigned char buf[2];
  *v = 0;

  if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CONNECTION, &conn, 1) < 0) {
    pmsg_error("cannot obtain connection type\n");
    return -1;
  }
  if (jtag3_getparm(pgm, SCOPE_AVR, 0, PARM3_ARCH, &arch, 1) < 0) {
    pmsg_error("cannot obtain target architecture\n");
    return -1;
  }

  if (conn == PARM3_CONN_JTAG) {
    if (arch == PARM3_ARCH_XMEGA) {
      if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_JTAG, buf, 2) < 0) {
        pmsg_error("cannot read Xmega JTAG clock speed\n");
        return -1;
      }
    } else {
      if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_MEGA_PROG, buf, 2) < 0) {
        pmsg_error("cannot read JTAG clock speed\n");
        return -1;
      }
    }
  } else if (conn & (PARM3_CONN_PDI | PARM3_CONN_UPDI)) {
    if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_PDI, buf, 2) < 0) {
      pmsg_error("cannot read PDI/UPDI clock speed\n");
      return -1;
    }
  }

  if (b2_to_u16(buf) <= 0) {
    pmsg_error("cannot calculate programmer clock speed\n");
    return -1;
  }
  *v = 1.0/(1000*b2_to_u16(buf));

  return 0;
}


/*
 * Read (an) emulator parameter(s).
 */
int jtag3_getparm(const PROGRAMMER *pgm, unsigned char scope,
                  unsigned char section, unsigned char parm,
                  unsigned char *value, unsigned char length) {
  int status;
  unsigned char buf[6], *resp, c;
  char descr[60];

  pmsg_notice2("jtag3_getparm()\n");

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
    pmsg_notice("jtag3_getparm(): bad response to %s\n", descr);
    free(resp);
    return -1;
  }

  status -= 3;
  if (status < 0) {
    pmsg_error("unexpected return value %d from jtag3_command()\n", status);
    free(resp);
    return -1;
  }
  memcpy(value, resp + 3, (length < status? length: status));
  free(resp);

  return 0;
}

/*
 * Write an emulator parameter.
 */
int jtag3_setparm(const PROGRAMMER *pgm, unsigned char scope,
                  unsigned char section, unsigned char parm,
                  unsigned char *value, unsigned char length) {
  int status;
  unsigned char *buf, *resp;
  char descr[60];

  pmsg_notice2("jtag3_setparm()\n");

  sprintf(descr, "set parameter (scope 0x%02x, section %d, parm %d)",
          scope, section, parm);

  if ((buf = malloc(6 + length)) == NULL) {
    pmsg_error("out of memory\n");
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
  if (status >= 0)
    free(resp);

  return status;
}

int jtag3_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib) {
  int status;
  unsigned char cmd[12];
  unsigned char *resp = NULL;

  cmd[0] = SCOPE_AVR;
  cmd[1] = CMD3_READ_MEMORY;
  cmd[2] = 0;
  cmd[3] = MTYPE_SIB;
  u32_to_b4(cmd + 4, 0);
  u32_to_b4(cmd + 8, AVR_SIBLEN);

  if ((status = jtag3_command(pgm, cmd, 12, &resp, "read SIB")) < 0)
    return status;

  memcpy(sib, resp+3, AVR_SIBLEN);
  sib[AVR_SIBLEN-1] = 0; // Zero terminate string
  pmsg_debug("jtag3_read_sib(): received SIB: %s\n", sib);
  free(resp);
  return 0;
}

int jtag3_read_chip_rev(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev) {
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

  pmsg_debug("jtag3_read_chip_rev(): received chip silicon revision: 0x%02x\n", *chip_rev);
  return 0;
}

int jtag3_set_vtarget(const PROGRAMMER *pgm, double v) {
  unsigned uaref, utarg;
  unsigned char buf[2];

  utarg = (unsigned)(v * 1000);

  if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_VTARGET, buf, 2) < 0) {
    pmsg_warning("cannot obtain V[target]\n");
  }

  uaref = b2_to_u16(buf);
  u16_to_b2(buf, utarg);

  pmsg_notice2("jtag3_set_vtarget(): changing V[target] from %.1f to %.1f\n", uaref / 1000.0, v);

  if (jtag3_setparm(pgm, SCOPE_GENERAL, 1, PARM3_VADJUST, buf, sizeof(buf)) < 0) {
    pmsg_error("cannot confirm new V[target] value\n");
    return -1;
  }

  return 0;
}

int jtag3_get_vtarget(const PROGRAMMER *pgm, double *v) {
  unsigned char buf[2];

  if(jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_VTARGET, buf, 2) < 0) {
    pmsg_error("cannot read target voltage\n");
    return -1;
  }

  *v = b2_to_u16(buf)/1000.0;
  return 0;
}

void jtag3_display(const PROGRAMMER *pgm, const char *p) {
  unsigned char parms[5];
  unsigned char *resp = NULL;
  const char *sn;

  /*
   * Ask for:
   *  PARM3_HW_VER (1 byte)
   *  PARM3_FW_MAJOR (1 byte)
   *  PARM3_FW_MINOR (1 byte)
   *  PARM3_FW_RELEASE (2 bytes)
   */
  if (jtag3_getparm(pgm, SCOPE_GENERAL, 0, PARM3_HW_VER, parms, 5) < 0)
    return;

  // Use serial number pulled from the USB driver. If not present, query the programmer
  if (pgm->usbsn && *pgm->usbsn)
    sn = pgm->usbsn;
  else {
    unsigned char cmd[4], c;
    int status;
    cmd[0] = SCOPE_INFO;
    cmd[1] = CMD3_GET_INFO;
    cmd[2] = 0;
    cmd[3] = CMD3_INFO_SERIAL;

    if ((status = jtag3_command(pgm, cmd, 4, &resp, "get info (serial number)")) < 0) {
      free(resp);
      return;
    }

    c = resp[1];
    if (c != RSP3_INFO) {
      pmsg_error("response is not RSP3_INFO\n");
      free(resp);
      return;
    }
    if (status < 3) {
      msg_error("unexpected response from CMD3_GET_INFO command\n");
      free(resp);
      return;
    }
    memmove(resp, resp + 3, status - 3);
    resp[status - 3] = 0;
    sn = (const char*)resp;
  }
  msg_info("%sICE HW version        : %d\n", p, parms[0]);
  msg_info("%sICE FW version        : %d.%02d (rel. %d)\n",
    p, parms[1], parms[2], (parms[3] | (parms[4] << 8)));
  msg_info("%sSerial number         : %s\n", p, sn);
  free(resp);
}


void jtag3_print_parms1(const PROGRAMMER *pgm, const char *p, FILE *fp) {
  unsigned char prog_mode[2];
  unsigned char buf[3];

  if (pgm->extra_features & HAS_VTARG_READ) {
    if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_VTARGET, buf, 2) < 0)
      return;
    msg_info("%sVtarget               : %.2f V\n", p, b2_to_u16(buf)/1000.0);
  }

  // Print clocks if programmer type is not TPI
  if (!str_eq(pgm->type, "JTAGICE3_TPI")) {
    // Get current programming mode and target type from to determine what data to print
    if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CONNECTION, prog_mode, 1) < 0)
      return;
    if (jtag3_getparm(pgm, SCOPE_AVR, 0, PARM3_ARCH, &prog_mode[1], 1) < 0)
      return;
    if (prog_mode[0] == PARM3_CONN_JTAG) {
      if (prog_mode[1] == PARM3_ARCH_XMEGA) {
        if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_JTAG, buf, 2) < 0)
          return;
        if (b2_to_u16(buf) > 0)
          fmsg_out(fp, "%sJTAG clk Xmega        : %u kHz\n", p, b2_to_u16(buf));
      } else {
        if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_MEGA_PROG, buf, 2) < 0)
          return;
        if (b2_to_u16(buf) > 0)
          fmsg_out(fp, "%sJTAG clk prog.        : %u kHz\n", p, b2_to_u16(buf));

        if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_MEGA_DEBUG, buf, 2) < 0)
          return;
        if (b2_to_u16(buf) > 0)
          fmsg_out(fp, "%sJTAG clk debug        : %u kHz\n", p, b2_to_u16(buf));
      }
    }
    else if (prog_mode[0] == PARM3_CONN_PDI || prog_mode[0] == PARM3_CONN_UPDI) {
      if (jtag3_getparm(pgm, SCOPE_AVR, 1, PARM3_CLK_XMEGA_PDI, buf, 2) < 0)
        return;
      if (b2_to_u16(buf) > 0)
        fmsg_out(fp, "%sPDI/UPDI clk          : %u kHz\n", p, b2_to_u16(buf));
    }
  }

  // Print features unique to the Power Debugger
  for(LNODEID ln=lfirst(pgm->id); ln; ln=lnext(ln)) {
    if(str_starts(ldata(ln), "powerdebugger")) {
      short analog_raw_data;

      // Read generator set voltage value (VOUT)
      if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_VADJUST, buf, 2) < 0)
        return;
      analog_raw_data = b2_to_u16(buf);
      fmsg_out(fp, "%sVout set              : %.2f V\n", p, analog_raw_data / 1000.0);

      // Read measured generator voltage value (VOUT)
      if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_TSUP_VOLTAGE_MEAS, buf, 2) < 0)
        return;
      analog_raw_data = ((buf[0] & 0x0F) << 8) + buf[1];
      if ((buf[0] & 0xF0) != 0x30)
        pmsg_error("invalid PARM3_TSUP_VOLTAGE_MEAS data packet format\n");
      else {
        if (analog_raw_data & 0x0800)
          analog_raw_data |= 0xF000;
        fmsg_out(fp, "%sVout measured         : %.02f V\n", p, analog_raw_data / -200.0);
      }

      // Read channel A voltage
      if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_ANALOG_A_VOLTAGE, buf, 2) < 0)
        return;
      analog_raw_data = ((buf[0] & 0x0F) << 8) + buf[1];
      if ((buf[0] & 0xF0) != 0x20)
        pmsg_error("invalid PARM3_ANALOG_A_VOLTAGE data packet format\n");
      else {
        if (analog_raw_data & 0x0800)
          analog_raw_data |= 0xF000;
        fmsg_out(fp, "%sCh A voltage          : %.03f V\n", p, analog_raw_data / -200.0);
      }

      // Read channel A current
      if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_ANALOG_A_CURRENT, buf, 3) < 0)
        return;
      analog_raw_data = (buf[1] << 8) + buf[2];
      if (buf[0] != 0x90)
        pmsg_error("invalid PARM3_ANALOG_A_CURRENT data packet format\n");
      else
        fmsg_out(fp, "%sCh A current          : %.3f mA\n", p, analog_raw_data * 0.003472);

      // Read channel B voltage
      if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_ANALOG_B_VOLTAGE, buf, 2) < 0)
        return;
      analog_raw_data = ((buf[0] & 0x0F) << 8) + buf[1];
      if ((buf[0] & 0xF0) != 0x10)
        pmsg_error("invalid PARM3_ANALOG_B_VOLTAGE data packet format\n");
      else {
        if (analog_raw_data & 0x0800)
          analog_raw_data |= 0xF000;
        fmsg_out(fp, "%sCh B voltage          : %.03f V\n", p, analog_raw_data / -200.0);
      }

      // Read channel B current
      if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_ANALOG_B_CURRENT, buf, 3) < 0)
        return;
      analog_raw_data = ((buf[0] & 0x0F) << 8) + buf[1];
      if ((buf[0] & 0xF0) != 0x00)
        pmsg_error("invalid PARM3_ANALOG_B_CURRENT data packet format\n");
      else {
        if (analog_raw_data & 0x0800)
          analog_raw_data |= 0xF000;
        fmsg_out(fp, "%sCh B current          : %.3f mA\n", p, analog_raw_data * 0.555556);
      }
      break;
    }
  }
  fmsg_out(fp, "\n");
}

static void jtag3_print_parms(const PROGRAMMER *pgm, FILE *fp) {
  jtag3_print_parms1(pgm, "", fp);
}

static unsigned char jtag3_mtype(const PROGRAMMER *pgm, const AVRPART *p, unsigned long addr) {
  if (p->prog_modes & PM_PDI) {
    if (addr >= PDATA(pgm)->boot_start)
      return MTYPE_BOOT_FLASH;
    else
      return MTYPE_FLASH;
  } else {
    return MTYPE_FLASH_PAGE;
  }
}

static unsigned int jtag3_memaddr(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned long addr) {
  if (p->prog_modes & PM_PDI) {
    /*
     * All memories but "flash" are smaller than boot_start anyway, so
     * no need for an extra check we are operating on "flash"
     */
    if(addr >= PDATA(pgm)->boot_start)
      addr -= PDATA(pgm)->boot_start;
  } else if(p->prog_modes & PM_UPDI) { // Modern AVR8X part
    if(!mem_is_flash(m))
      if(m->size >= 1)
        addr += m->offset;
  } else {                      // Classic part
    if(mem_is_userrow(m))
      addr += m->offset;
  }

  return addr;
}

static unsigned char tpi_get_mtype(const AVRMEM *m) {
  return
    mem_is_a_fuse(m)? XPRG_MEM_TYPE_FUSE:
    mem_is_lock(m)? XPRG_MEM_TYPE_LOCKBITS:
    mem_is_calibration(m)? XPRG_MEM_TYPE_LOCKBITS: // Sic, uses offset to distingish memories
    mem_is_signature(m)? XPRG_MEM_TYPE_LOCKBITS:
    mem_is_sigrow(m)? XPRG_MEM_TYPE_LOCKBITS:
    XPRG_MEM_TYPE_APPL;         // Sic, TPI parts do not have eeprom
}

/*
 * Send the data as a JTAGICE3 encapsulated TPI packet.
 */
static int jtag3_send_tpi(const PROGRAMMER *pgm, unsigned char *data, size_t len) {
  unsigned char *cmdbuf;
  int rv;

  if ((cmdbuf = malloc(len + 1)) == NULL) {
    pmsg_error("jtag3_send_tpi(): out of memory for command packet\n");
    exit(1);
  }

  cmdbuf[0] = SCOPE_AVR_TPI;
  if (len > INT_MAX) {
    pmsg_error("invalid jtag3_send_tpi() packet length %lu\n", (unsigned long) len);
    free(cmdbuf);
    return -1;
  }
  memcpy(cmdbuf + 1, data, len);

  msg_trace("[TPI send] ");
  for (size_t i=1; i<=len; i++)
    msg_trace("0x%02x ", cmdbuf[i]);
  msg_trace("\n");

  rv = jtag3_send(pgm, cmdbuf, len + 1);
  free(cmdbuf);

  return rv;
}

int jtag3_recv_tpi(const PROGRAMMER *pgm, unsigned char **msg) {
  int rv;

  rv = jtag3_recv(pgm, msg);

  if (rv <= 0) {
    pmsg_error("jtag3_recv_tpi(): unable to receive\n");
    return -1;
  }
  rv = rv - 1;
  memcpy(*msg, *msg + 1, rv);

  msg_trace("[TPI recv] ");
  for (int i=0; i<rv; i++)
    msg_trace("0x%02x ", (*msg)[i]);
  msg_trace("\n");

  return rv;
}

int jtag3_command_tpi(const PROGRAMMER *pgm, unsigned char *cmd, unsigned int cmdlen,
                  unsigned char **resp, const char *descr) {
  int status;
  unsigned char c;

  jtag3_send_tpi(pgm, cmd, cmdlen);

  status = jtag3_recv_tpi(pgm, resp);
  if (status <= 0) {
    msg_notice2("\n");
    pmsg_notice2("TPI %s command: timeout/error communicating with programmer (status %d)\n", descr, status);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  c = (*resp)[1];
  if (c != XPRG_ERR_OK) {
    pmsg_error("[TPI] command %s FAILED! Status: 0x%02x\n", descr, c);
    status = (*resp)[3];
    free(*resp);
    resp = 0;
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return status;
}

/*
 * initialize the AVR device and prepare it to accept commands
 */
static int jtag3_initialize_tpi(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[3];
  unsigned char* resp;
  int status;

  // Read or write target voltage
  if (PDATA(pgm)->vtarg_get || PDATA(pgm)->vtarg_set) {
    // Read current target voltage set value
    unsigned char buf[2];
    if (jtag3_getparm(pgm, SCOPE_GENERAL, 1, PARM3_VADJUST, buf, 2) < 0)
      return -1;
    double vtarg_read = b2_to_u16(buf) / 1000.0;
    if (PDATA(pgm)->vtarg_get)
      msg_info("Target voltage value read as %.2fV\n", vtarg_read);
    // Write target voltage value
    else {
      u16_to_b2(buf, (unsigned)(PDATA(pgm)->vtarg_data * 1000));
      msg_info("Changing target voltage from %.2f to %.2fV\n", vtarg_read, PDATA(pgm)->vtarg_data);
      if (jtag3_setparm(pgm, SCOPE_GENERAL, 1, PARM3_VADJUST, buf, sizeof(buf)) < 0) {
        msg_warning("Cannot set target voltage %.2fV\n", PDATA(pgm)->vtarg_data);
        return -1;
      }
    }
  }

  if (verbose > 0 && quell_progress < 2)
    jtag3_print_parms1(pgm, progbuf, stderr);

  pmsg_notice2("jtag3_initialize_tpi() start\n");

  cmd[0] = XPRG_CMD_ENTER_PROGMODE;

  if ((status = jtag3_command_tpi(pgm, cmd, 1, &resp, "Enter Progmode")) < 0)
    return -1;
  free(resp);

  cmd[0] = XPRG_CMD_SET_PARAM;
  cmd[1] = XPRG_PARAM_NVMCMD_ADDR;
  cmd[2] = TPI_NVMCMD_ADDRESS;

  if ((status = jtag3_command_tpi(pgm, cmd, 3, &resp, "Set NVMCMD")) < 0)
    return -1;
  free(resp);

  cmd[0] = XPRG_CMD_SET_PARAM;
  cmd[1] = XPRG_PARAM_NVMCSR_ADDR;
  cmd[2] = TPI_NVMCSR_ADDRESS;

  if ((status = jtag3_command_tpi(pgm, cmd, 3, &resp, "Set NVMCSR")) < 0)
    return -1;
  free(resp);

  return 0;
}

static void jtag3_enable_tpi(PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_notice2("jtag3_enable_tpi() is empty. No action necessary.\n");
}

static void jtag3_disable_tpi(const PROGRAMMER *pgm) {
  unsigned char cmd[1];
  unsigned char* resp;
  int status;

  cmd[0] = XPRG_CMD_LEAVE_PROGMODE;

  if ((status = jtag3_command_tpi(pgm, cmd, 1, &resp, "Leave Progmode")) < 0)
    return;
  free(resp);
}

static int jtag3_read_byte_tpi(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                               unsigned long addr, unsigned char * value) {
  int status;
  const size_t len = 8;
  unsigned char cmd[8];  // Using "len" as array length causes msvc build jobs to fail with error C2057: expected constant expression
  unsigned char* resp;
  unsigned long paddr = 0UL;

  msg_notice2("\n");
  pmsg_notice2("jtag3_read_byte_tpi(.., %s, 0x%lx, ...)\n", mem->desc, addr);

  paddr = mem->offset + addr;

  cmd[0] = XPRG_CMD_READ_MEM;
  cmd[1] = tpi_get_mtype(mem);
  u32_to_b4_big_endian((cmd+2), paddr);  // Address
  u16_to_b2_big_endian((cmd+6), 1);      // Size

  if ((status = jtag3_command_tpi(pgm, cmd, len, &resp, "Read Byte")) < 0)
    return -1;
  *value = resp[2];
  free(resp);
  return 0;
}

static int jtag3_erase_tpi(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                           unsigned long addr) {
  const size_t len = 6;
  unsigned char cmd[6];  // Using "len" as array length causes msvc build jobs to fail with error C2057: expected constant expression
  unsigned char* resp;
  int status;
  unsigned long paddr = 0UL;

  cmd[0] = XPRG_CMD_ERASE;
  if (mem_is_a_fuse(mem)) {
    cmd[1] = XPRG_ERASE_CONFIG;
  } else if (mem_is_flash(mem)) {
    cmd[1] = XPRG_ERASE_APP;
  } else {
    pmsg_error("jtag3_erase_tpi() unsupported memory: %s\n", mem->desc);
    return -1;
  }
  paddr = (mem->offset + addr) | 0x01;  // An erase is triggered by an access to the hi-byte
  u32_to_b4_big_endian((cmd+2), paddr);

  if ((status = jtag3_command_tpi(pgm, cmd, len, &resp, "Erase")) < 0)
    return -1;
  free(resp);
  return 0;
}

static int jtag3_write_byte_tpi(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                                unsigned long addr, unsigned char data) {
  size_t len = 11;
  size_t data_size = 2;
  unsigned char cmd[17];
  unsigned char* resp;
  int status;
  unsigned long paddr = 0UL;

  if(mem_is_readonly(mem)) {
    unsigned char is;
    if(pgm->read_byte(pgm, p, mem, addr, &is) >= 0 && is == data)
      return 0;

    pmsg_error("cannot write to read-only memory %s of %s\n", mem->desc, p->desc);
    return -1;
  }

  status = jtag3_erase_tpi(pgm, p, mem, addr);
  if (status < 0) {
    pmsg_error("error in communication, received status 0x%02x\n", status);
    return -1;
  }

  paddr = mem->offset + addr;

  if (mem->n_word_writes != 0) {
    if (mem->n_word_writes == 2) {
      len = 13;
      data_size = 4;
    }
    else if (mem->n_word_writes == 4) {
      len = 17;
      data_size = 8;
    }
  }

  cmd[0] = XPRG_CMD_WRITE_MEM;
  cmd[1] = tpi_get_mtype(mem);
  cmd[2] = 0;  // Page Mode - Not used
  u32_to_b4_big_endian((cmd+3), paddr);      // Address
  u16_to_b2_big_endian((cmd+7), data_size);  // Size
  cmd[9] = data;
  cmd[10] = 0xFF;  // len = 11 if no n_word_writes
  cmd[11] = 0xFF;
  cmd[12] = 0xFF;  // len = 13 if n_word_writes == 2
  cmd[13] = 0xFF;
  cmd[14] = 0xFF;
  cmd[15] = 0xFF;
  cmd[16] = 0xFF;  // len = 17 if n_word_writes == 4

  if ((status = jtag3_command_tpi(pgm, cmd, len, &resp, "Write Byte")) < 0)
    return -1;
  free(resp);
  return 0;
}

static int jtag3_chip_erase_tpi(const PROGRAMMER *pgm, const AVRPART *p) {
  const size_t len = 6;
  unsigned char cmd[6];  // Using "len" as array length causes msvc build jobs to fail with error C2057: expected constant expression
  unsigned char* resp;
  int status;
  unsigned long paddr = 0UL;

  AVRMEM *m = avr_locate_flash(p);
  if (m == NULL) {
    pmsg_error("no flash memory for part %s\n", p->desc);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  // An erase is triggered by an access to the hi-byte
  paddr = m->offset | 0x01;

  cmd[0] = XPRG_CMD_ERASE;
  cmd[1] = XPRG_ERASE_CHIP;
  u32_to_b4_big_endian((cmd+2), paddr);

  if ((status = jtag3_command_tpi(pgm, cmd, len, &resp, "Chip Erase")) < 0)
    return -1;
  free(resp);
  return 0;
}

static int jtag3_open_tpi(PROGRAMMER *pgm, const char *port) {
  pmsg_notice2("jtag3_open_tpi()\n");

  if (jtag3_open_common(pgm, port, PDATA(pgm)->pk4_snap_mode) < 0)
    return -1;
  return 0;
}

void jtag3_close_tpi(PROGRAMMER *pgm) {
  pmsg_notice2("jtag3_close_tpi() is empty. No action necessary.\n");
}

static int jtag3_paged_load_tpi(const PROGRAMMER *pgm, const AVRPART *p,
                                const AVRMEM *m, unsigned int page_size,
                                unsigned int addr, unsigned int n_bytes) {
  unsigned int block_size = 0;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char cmd[8];
  unsigned char *resp;
  int status;
  long otimeout = serial_recv_timeout;

  msg_notice2("\n");
  pmsg_notice2("jtag3_paged_load_tpi(.., %s, %d, 0x%04x, %d)\n",
               m->desc, page_size, addr, n_bytes);

  if(m->offset)
    imsg_notice2("mapped to address: 0x%04x\n", (addr+m->offset));

  cmd[0] = XPRG_CMD_READ_MEM;
  cmd[1] = tpi_get_mtype(m);

  if(m->blocksize > (int) page_size)
    page_size = m->blocksize;

  serial_recv_timeout = 100;
  for (; addr < maxaddr; addr += page_size) {
    if ((maxaddr - addr) < page_size)
      block_size = maxaddr - addr;
    else
      block_size = page_size;
    pmsg_debug("jtag3_paged_load_tpi(): "
               "block_size at addr 0x%x is %d\n", addr, block_size);

    u32_to_b4_big_endian((cmd+2), addr + m->offset);  // Address
    u16_to_b2_big_endian((cmd+6), block_size);        // Size

    if ((status = jtag3_command_tpi(pgm, cmd, 8, &resp, "Read Memory")) < 0)
      return -1;

    if (resp[1] != XPRG_ERR_OK || status < (int) block_size + 2) {
      pmsg_error("wrong/short reply to read memory command\n");
      serial_recv_timeout = otimeout;
      free(resp);
      return -1;
    }

    if (status < 2) {
      pmsg_error("unexpected return value %d from jtag3_paged_load_tpi()\n", status);
      free(resp);
      return -1;
    }

    memcpy(m->buf + addr, resp + 2, status - 2);
    free(resp);
  }
  serial_recv_timeout = otimeout;

  return n_bytes;
}

static int jtag3_paged_write_tpi(const PROGRAMMER *pgm, const AVRPART *p,
                                 const AVRMEM *m, unsigned int page_size,
                                 unsigned int addr, unsigned int n_bytes) {
  unsigned int block_size;
  unsigned int maxaddr = addr + n_bytes;
  unsigned char *cmd;
  unsigned char *resp;
  int status;
  long otimeout = serial_recv_timeout;

  msg_notice2("\n");
  pmsg_notice2("jtag3_paged_write_tpi(.., %s, %d, 0x%04x, %d)\n", m->desc, page_size, addr, n_bytes);

  if(m->offset)
    imsg_notice2("mapped to address: 0x%04x\n", (addr+m->offset));

  if (page_size == 0)
    page_size = m->page_size;

  if ((cmd = malloc(page_size + 9)) == NULL) {
    pmsg_error("out of memory\n");
    return -1;
  }

  cmd[0] = XPRG_CMD_WRITE_MEM;
  cmd[1] = tpi_get_mtype(m);
  cmd[2] = 0;  // Page Mode; Not used - ignored

  serial_recv_timeout = 100;
  for (; addr < maxaddr; addr += page_size) {
    if ((maxaddr - addr) < page_size)
      block_size = maxaddr - addr;
    else
      block_size = page_size;
    pmsg_debug("jtag3_paged_write(): "
      "block_size at addr 0x%x is %d\n", addr, block_size);

    u32_to_b4_big_endian((cmd+3), addr + m->offset);  // Address
    u16_to_b2_big_endian((cmd+7), page_size);        // Size

    /*
     * If a partial page has been requested, set the remainder to 0xff.
     * (Maybe we should rather read back the existing contents instead
     * before?  Doesn't matter much, as bits cannot be written to 1 anyway.)
     */
    memset(cmd + 9, 0xff, page_size);
    memcpy(cmd + 9, m->buf + addr, block_size);

    if ((status = jtag3_command_tpi(pgm, cmd, page_size + 9,
        &resp, "Write Memory")) < 0) {
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


const char jtag3_desc[] = "Atmel JTAGICE3";

void jtag3_initpgm(PROGRAMMER *pgm) {
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
  pgm->get_sck_period = jtag3_get_sck_period;
  pgm->parseextparams = jtag3_parseextparms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_JTAG;
  pgm->read_chip_rev  = jtag3_read_chip_rev;

  /*
   * hardware dependent functions
   */
  if (pgm->extra_features & HAS_VTARG_READ)
    pgm->get_vtarget  = jtag3_get_vtarget;
  if (pgm->extra_features & HAS_VTARG_ADJ)
    pgm->set_vtarget  = jtag3_set_vtarget;
}

const char jtag3_dw_desc[] = "Atmel JTAGICE3 in debugWire mode";

void jtag3_dw_initpgm(PROGRAMMER *pgm) {
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
  pgm->page_erase     = NULL;
  pgm->print_parms    = jtag3_print_parms;
  pgm->parseextparams = jtag3_parseextparms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_DW;

  /*
   * hardware dependent functions
   */
  if (pgm->extra_features & HAS_VTARG_READ)
    pgm->get_vtarget  = jtag3_get_vtarget;
  if (pgm->extra_features & HAS_VTARG_ADJ)
    pgm->set_vtarget  = jtag3_set_vtarget;
}

const char jtag3_pdi_desc[] = "Atmel JTAGICE3 in PDI mode";

void jtag3_pdi_initpgm(PROGRAMMER *pgm) {
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
  pgm->set_sck_period = jtag3_set_sck_period;
  pgm->get_sck_period = jtag3_get_sck_period;
  pgm->parseextparams = jtag3_parseextparms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_PDI;
  pgm->read_chip_rev  = jtag3_read_chip_rev;

  /*
   * hardware dependent functions
   */
  if (pgm->extra_features & HAS_VTARG_READ)
    pgm->get_vtarget  = jtag3_get_vtarget;
  if (pgm->extra_features & HAS_VTARG_ADJ)
    pgm->set_vtarget  = jtag3_set_vtarget;
}

const char jtag3_updi_desc[] = "Atmel JTAGICE3 in UPDI mode";

void jtag3_updi_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "JTAGICE3_UPDI");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtag3_initialize;
  pgm->display        = jtag3_display;
  pgm->enable         = jtag3_enable;
  pgm->disable        = jtag3_disable;
  pgm->program_enable = jtag3_program_enable_dummy;
  pgm->chip_erase     = jtag3_chip_erase;
  pgm->open           = jtag3_open_updi;
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
  pgm->get_sck_period = jtag3_get_sck_period;
  pgm->parseextparams = jtag3_parseextparms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_UPDI;
  pgm->unlock         = jtag3_unlock_erase_key;
  pgm->read_sib       = jtag3_read_sib;
  pgm->read_chip_rev  = jtag3_read_chip_rev;

  /*
   * hardware dependent functions
   */
  if (pgm->extra_features & HAS_VTARG_READ)
    pgm->get_vtarget  = jtag3_get_vtarget;
  if (pgm->extra_features & HAS_VTARG_ADJ)
    pgm->set_vtarget  = jtag3_set_vtarget;
}

const char jtag3_tpi_desc[] = "Atmel JTAGICE3 in TPI mode";

void jtag3_tpi_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "JTAGICE3_TPI");

  /*
   * mandatory functions
   */
  pgm->initialize     = jtag3_initialize_tpi;
  pgm->display        = jtag3_display;
  pgm->enable         = jtag3_enable_tpi;
  pgm->disable        = jtag3_disable_tpi;
  pgm->program_enable = jtag3_program_enable_dummy;
  pgm->chip_erase     = jtag3_chip_erase_tpi;
  pgm->open           = jtag3_open_tpi;
  pgm->close          = jtag3_close_tpi;
  pgm->read_byte      = jtag3_read_byte_tpi;
  pgm->write_byte     = jtag3_write_byte_tpi;

  /*
   * optional functions
   */
  pgm->paged_write    = jtag3_paged_write_tpi;
  pgm->paged_load     = jtag3_paged_load_tpi;
  pgm->page_erase     = NULL;
  pgm->print_parms    = jtag3_print_parms;
  pgm->parseextparams = jtag3_parseextparms;
  pgm->setup          = jtag3_setup;
  pgm->teardown       = jtag3_teardown;
  pgm->page_size      = 256;
  pgm->flag           = PGM_FL_IS_TPI;

  /*
   * hardware dependent functions
   */
  if (pgm->extra_features & HAS_VTARG_READ)
    pgm->get_vtarget  = jtag3_get_vtarget;
}
