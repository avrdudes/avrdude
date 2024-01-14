/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2009 Lars Immisch
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
 * avrdude interface for Arduino programmer
 *
 * The Arduino programmer is mostly a STK500v1, just the signature bytes
 * are read differently.
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "stk500_private.h"
#include "stk500.h"
#include "arduino.h"

#define USERROW_V0_ADDR 0x1300  /* Magic number that characterizes NVMCTRL V0: USERROW=0x1300 */

/* Read single byte in interactive mode - arduino version */
static int arduino_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                             unsigned long addr, unsigned char * value)
{
  // Limited to Microchip AVR parts with UPDI pin.
  // Only authorized bootloaders can run it.
  if (!PDATA(pgm)->using_enhanced_memory)
    return -1;

  unsigned char buf[16];

  // Convert address for Cmnd_STK_READ_PAGE memory type 'E'.
  addr += m->offset - PDATA(pgm)->boot_eeprom_offset;

  // The valid "data" address width is 16 bits. Overflow is rounded.
  buf[0] = Cmnd_STK_LOAD_ADDRESS;
  buf[1] = addr & 0xff;
  buf[2] = (addr >> 8) & 0xff;
  buf[3] = Sync_CRC_EOP;

  serial_send(&pgm->fd, buf, 4);

  if (serial_recv(&pgm->fd, buf, 2) < 0)
    return -1;
  else if (buf[0] != Resp_STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -1;
  }
  if (buf[1] != Resp_STK_OK) {
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[0]);
    return -1;
  }

  buf[0] = Cmnd_STK_READ_PAGE;
  buf[1] = 0;
  buf[2] = 1;
  buf[3] = 'E';
  buf[4] = Sync_CRC_EOP;

  serial_send(&pgm->fd, buf, 5);

  if (serial_recv(&pgm->fd, buf, 3) < 0)
    return -1;
  else if (buf[0] != Resp_STK_INSYNC) {
    msg_error("\n");
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -2;
  }
  if (buf[2] != Resp_STK_OK) {
    msg_error("\n");
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[4]);
    return -3;
  }

  *value = buf[1];
  return 0;
}

/* read signature bytes - arduino version */
static int arduino_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m) {
  unsigned char buf[32];

  /* Signature byte reads are always 3 bytes. */

  if (m->size < 3) {
    pmsg_error("memsize too small for sig byte read");
    return -1;
  }

  buf[0] = Cmnd_STK_READ_SIGN;
  buf[1] = Sync_CRC_EOP;

  serial_send(&pgm->fd, buf, 2);

  if (serial_recv(&pgm->fd, buf, 5) < 0)
    return -1;
  if (buf[0] == Resp_STK_NOSYNC) {
    pmsg_error("programmer is out of sync\n");
	return -1;
  } else if (buf[0] != Resp_STK_INSYNC) {
    msg_error("\n");
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -2;
  }
  if (buf[4] != Resp_STK_OK) {
    msg_error("\n");
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[4]);
    return -3;
  }

  m->buf[0] = buf[1];
  m->buf[1] = buf[2];
  m->buf[2] = buf[3];

  // Enhanced bootloader support
  if (PDATA(pgm)->using_enhanced_memory) {
    unsigned char chip_rev;
    AVRMEM *m;
    m = avr_locate_eeprom(p);
    PDATA(pgm)->boot_eeprom_offset = m->offset;
    // The EEPROM offset of the affected chip is typically 0x1400 from oldest to newest.
    // That may not be the case in the future.

    // Get silicon revision
    if (arduino_read_byte(pgm, p, m, (p->syscfg_base + 1) - m->offset, &chip_rev) == 0) {
      pmsg_debug("arduino_read_byte(): received chip silicon revision: 0x%02x\n", chip_rev);
      pmsg_notice("silicon revision: '%c%x'\n", (chip_rev >> 4) + 'A', chip_rev & 0x0f);
    }
  }

  return 3;
}

static int arduino_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;
  strcpy(pgm->port, port);
  pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: 115200;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  if (serial_open(port, pinfo, &pgm->fd)==-1) {
    return -1;
  }

  // This code assumes a negative-logic USB to TTL serial adapter
  // Set RTS/DTR high to discharge the series-capacitor, if present
  serial_set_dtr_rts(&pgm->fd, 0);
  /*
   * Long wait needed for optiboot: otherwise the second of two bootloader
   * calls in quick succession fails:
   *
   * avrdude -c arduino -qqp m328p -U x.hex; avrdude -c arduino -qqp m328p -U x.hex
   */
  usleep(250 * 1000);
  // Pull the RTS/DTR line low to reset AVR
  serial_set_dtr_rts(&pgm->fd, 1);
  // Max 100 us: charging a cap longer creates a high reset spike above Vcc
  usleep(100);
  // Set the RTS/DTR line back to high, so direct connection to reset works
  serial_set_dtr_rts(&pgm->fd, 0);

  usleep(100 * 1000);

  /*
   * drain any extraneous input
   */
  stk500_drain(pgm, 0);

  if (stk500_getsync(pgm) < 0)
    return -1;

  return 0;
}

static void arduino_close(PROGRAMMER * pgm) {
  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}

static int arduino_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_info("ignored because it doesn't work for programmer %s\n", pgmid);
  return 0;
}

static void arduino_setup(PROGRAMMER * pgm)
{
  // Replaces stk500_parseextparms.
  // Now the "-xtal" usage is no longer displayed.
  if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
    pmsg_error("out of memory allocating private data\n");
    return;
  }
  memset(pgm->cookie, 0, sizeof(struct pdata));
  PDATA(pgm)->retry_attempts          = 10;
  PDATA(pgm)->using_enhanced_memory   = false;  // True when using "-c arduino -xem"
  PDATA(pgm)->boot_userrow_v0_offset  = USERROW_V0_ADDR;
  PDATA(pgm)->boot_nvmctrl_version    = 0;
  // If HWV can be obtained, it is detected as '0' or more.
  // The boot_eeprom_offset member is initialized with arduino_read_sig_bytes().
}

static void arduino_teardown(PROGRAMMER * pgm) {
  free(pgm->cookie);
  pgm->cookie = NULL;
}

static int arduino_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int attempts;
  int rv = 0;

  for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if (sscanf(extended_param, "attempts=%2d", &attempts) == 1) {
      // Negative numbers are also detected, so they are excluded.
      // That is, it accepts numbers from 1 to 99.
      if (attempts <= 0) {
        pmsg_info("illegal value specification\n");
      }
      else {
        PDATA(pgm)->retry_attempts = attempts;
        pmsg_info("setting number of retry attempts to %d\n", attempts);
        continue;
      }
    }

    else if (str_eq(extended_param, "em")) {
      // *** ENHANCED FEATURE : -c arduino -xem option ***
      PDATA(pgm)->using_enhanced_memory = true;
      // Check the validity of this feature with stk500_initialize()
      continue;
    }

    else if (str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -xattempts=<arg>      Specify no. connection retry attempts\n");
      msg_error("  -xem                  Enables enhanced memory instructions (optiboot_x etc.)\n");
      msg_error("  -xhelp                Show this help menu and exit\n");
      exit(0);
    }

    pmsg_error("invalid extended parameter %s\n", extended_param);
    rv = -1;
  }

  return rv;
}

/*** Periodic call in terminal mode to keep bootloader alive ***/
static int arduino_term_keep_alive(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  unsigned char buf[16];

  buf[0] = Cmnd_STK_GET_SYNC;
  buf[1] = Sync_CRC_EOP;

  serial_send(&pgm->fd, buf, 2);

  if (serial_recv(&pgm->fd, buf, 2) < 0)
    return -1;

  if (buf[0] != Resp_STK_INSYNC) {
    msg_error("\n");
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -2;
  }

  if (buf[1] != Resp_STK_OK) {
    msg_error("\n");
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[1]);
    return -3;
  }

  return 0;
}

const char arduino_desc[] = "Arduino programmer";

void arduino_initpgm(PROGRAMMER *pgm) {
  /* This is mostly a STK500; just the signature is read
     differently than on real STK500v1 
     and the DTR signal is set when opening the serial port
     for the Auto-Reset feature */
  stk500_initpgm(pgm);

  strcpy(pgm->type, "Arduino");
  pgm->read_sig_bytes  = arduino_read_sig_bytes;
  pgm->open            = arduino_open;
  pgm->close           = arduino_close;
  pgm->chip_erase      = arduino_chip_erase;
  pgm->setup           = arduino_setup;
  pgm->teardown        = arduino_teardown;
  pgm->parseextparams  = arduino_parseextparms;
  pgm->read_byte       = arduino_read_byte;   // write_byte() is not replaced (not used)
  pgm->term_keep_alive = arduino_term_keep_alive;

  disable_trailing_ff_removal(); /* so that arduino bootloader can ignore chip erase */
}
