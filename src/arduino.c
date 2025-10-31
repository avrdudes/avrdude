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

/*
 * avrdude interface for Arduino programmer
 *
 * The Arduino programmer is mostly a STK500v1, just the signature bytes
 * are read differently.
 */

#include <ac_cfg.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "stk500_private.h"
#include "stk500.h"
#include "arduino.h"

static int arduino_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  int attempts;
  int rv = 0;
  bool help = 0;

  for(LNODEID ln = lfirst(extparms); ln; ln = lnext(ln)) {
    const char *extended_param = ldata(ln);

    if(sscanf(extended_param, "attempts=%i", &attempts) == 1) {
      my.retry_attempts = attempts;
      pmsg_info("setting number of retry attempts to %d\n", attempts);
      continue;
    }

    if(str_eq(extended_param, "noautoreset")) {
      my.autoreset = false;
      continue;
    }

    if(str_eq(extended_param, "help")) {
      help = true;
      rv = LIBAVRDUDE_EXIT_OK;
    }

    if(!help) {
      pmsg_error("invalid extended parameter -x %s\n", extended_param);
      rv = -1;
    }
    msg_error("%s -c %s extended options:\n", progname, pgmid);
    msg_error("  -x attempts=<n> Specify the number <n> of connection retry attempts\n");
    msg_error("  -x noautoreset  Don't toggle RTS/DTR lines on port open to prevent a hardware reset\n");
    msg_error("  -x help         Show this help menu and exit\n");
    return rv;
  }
  return rv;
}

// Read signature bytes - arduino version
static int arduino_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m) {
  unsigned char buf[32];

  // Signature byte reads are always 3 bytes

  if(m->size < 3) {
    pmsg_error("memsize too small for sig byte read");
    return -1;
  }

  buf[0] = Cmnd_STK_READ_SIGN;
  buf[1] = Sync_CRC_EOP;

  serial_send(&pgm->fd, buf, 2);

  if(serial_recv(&pgm->fd, buf, 5) < 0)
    return -1;
  if(buf[0] == Resp_STK_NOSYNC) {
    pmsg_error("programmer is out of sync\n");
    return -1;
  } else if(buf[0] != Resp_STK_INSYNC) {
    msg_error("\n");
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", Resp_STK_INSYNC, buf[0]);
    return -2;
  }
  if(buf[4] != Resp_STK_OK) {
    msg_error("\n");
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", Resp_STK_OK, buf[4]);
    return -3;
  }

  m->buf[0] = buf[1];
  m->buf[1] = buf[2];
  m->buf[2] = buf[3];

  return 3;
}

static int arduino_open(PROGRAMMER *pgm, const char *port) {
  if(pgm->bitclock)
    pmsg_warning("-c %s does not support adjustable bitclock speed; ignoring -B\n", pgmid);

  pgm->port = port;
  union pinfo pinfo;
  pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: 115200;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  if(serial_open(port, pinfo, &pgm->fd) == -1) {
    return -1;
  }

  if(my.autoreset) {
    // This code assumes a negative-logic USB to TTL serial adapter
    // Set RTS/DTR high to discharge the series-capacitor, if present
    serial_set_dtr_rts(&pgm->fd, 0);
    /*
     * Long wait needed for optiboot: otherwise the second of two bootloader
     * calls in quick succession fails:
     *
     * avrdude -c arduino -qqp m328p -U x.hex; avrdude -c arduino -qqp m328p -U x.hex
     */
    usleep(250*1000);
    // Pull the RTS/DTR line low to reset AVR
    serial_set_dtr_rts(&pgm->fd, 1);
    // Max 100 us: charging a cap longer creates a high reset spike above Vcc
    usleep(100);
    // Set the RTS/DTR line back to high, so direct connection to reset works
    serial_set_dtr_rts(&pgm->fd, 0);

    usleep(100*1000);
  }
  // Drain any extraneous input
  stk500_drain(pgm, 0);

  if(stk500_getsync(pgm) < 0)
    return -1;

  return 0;
}

static void arduino_close(PROGRAMMER *pgm) {
  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}

const char arduino_desc[] = "Arduino programmer for bootloading";

void arduino_initpgm(PROGRAMMER *pgm) {
  /* This is mostly a STK500; just the signature is read
     differently than on real STK500v1
     and the DTR signal is set when opening the serial port
     for the Auto-Reset feature */
  stk500_initpgm(pgm);
  strcpy(pgm->type, "Arduino");
  pgm->read_sig_bytes = arduino_read_sig_bytes;
  pgm->open = arduino_open;
  pgm->close = arduino_close;
  pgm->parseextparams = arduino_parseextparms;

  cx->avr_disableffopt = 1;     // Disable trailing 0xff removal
}
