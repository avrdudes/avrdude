/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2011 Brett Hagman
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
 * avrdude interface for Wiring bootloaders
 *
 * http://wiring.org.co/
 *
 * The Wiring bootloader uses a near-complete STK500v2 protocol.
 * (Only ISP specific programming commands are not implemented
 * e.g. chip erase).
 * DTR and RTS signals are diddled to set the board into programming mode.
 *
 * Also includes an extended parameter to introduce a delay after opening to
 * accommodate multi-layered programmers/bootloaders. If the extended
 * parameter 'snooze' > 0, then no DTR/RTS toggle takes place, and AVRDUDE
 * will wait that amount of time in milliseconds before syncing. If the
 * extended parameter 'delay' is set then this number of milliseconds is
 * added to the usual delay of 80 ms after toggling DTR/RTS.
 *
 */

#include <ac_cfg.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "stk500v2_private.h"
#include "stk500v2.h"
#include "wiring.h"

struct wiringpdata {
  int snoozetime, delay;
  bool noautoreset;
};

// wiringpdata is our private data

// pdata is stk500v2's private data (inherited)

#define mywiring (*(struct wiringpdata *) (((struct pdata *) (pgm->cookie)) -> chained_pdata))

static void wiring_setup(PROGRAMMER *pgm) {
  // First, have STK500v2 backend allocate its own private data
  stk500v2_setup(pgm);

  // Then prepare our data and store in a safe place for the time being
  ((struct pdata *) (pgm->cookie))->chained_pdata = mmt_malloc(sizeof(struct wiringpdata));
}

static void wiring_teardown(PROGRAMMER *pgm) {
  if(pgm->cookie)
    mmt_free(((struct pdata *) (pgm->cookie))->chained_pdata);
  stk500v2_teardown(pgm);
  pgm->cookie = NULL;
}

static int wiring_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  const char *errstr;
  int rv = 0;
  bool help = 0;

  for(LNODEID ln = lfirst(extparms); ln; ln = lnext(ln)) {
    const char *extended_param = ldata(ln);

    if(str_starts(extended_param, "snooze=")) {
      int val = str_int(extended_param + 7, STR_INT32, &errstr);

      if(errstr || val < 0) {
        pmsg_error("-x %s: %s\n", extended_param, errstr? errstr: "snooze time cannot be negative");
        rv = -1;
        break;
      }
      pmsg_notice2("%s(): snooze time set to %d ms\n", __func__, val);
      mywiring.snoozetime = val;
      continue;
    }

    if(str_starts(extended_param, "delay=")) {
      int val = str_int(extended_param + 6, STR_INT32, &errstr);

      if(errstr) {
        pmsg_error("-x %s: %s\n", extended_param, errstr);
        rv = -1;
        break;
      }
      pmsg_notice2("%s(): delay set to %d ms\n", __func__, val);
      mywiring.delay = val;
      continue;
    }

    if(str_eq(extended_param, "noautoreset")) {
      mywiring.noautoreset = true;
      continue;
    }

    if(str_eq(extended_param, "help")) {
      help = true;
      rv = LIBAVRDUDE_EXIT;
    }

    if(!help) {
      pmsg_error("invalid extended parameter -x %s\n", extended_param);
      rv = -1;
    }
    msg_error("%s -c %s extended options:\n", progname, pgmid);
    msg_error("  -x snooze=<n>   Wait snooze <n> ms before protocol sync after port open\n");
    msg_error("  -x delay=<n>    Add delay [n] ms after reset, can be negative\n");
    msg_error("  -x noautoreset  Don't toggle RTS/DTR lines on port open to prevent a hardware reset\n");
    msg_error("  -x help         Show this help menu and exit\n");
    return rv;
  }

  return rv;
}

static int wiring_open(PROGRAMMER *pgm, const char *port) {
  int timetosnooze;
  union pinfo pinfo;

  pgm->port = port;
  pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: 115200;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  serial_open(port, pinfo, &pgm->fd);

  // If we have a snoozetime, then we wait and do NOT toggle DTR/RTS
  if(mywiring.snoozetime > 0) {
    timetosnooze = mywiring.snoozetime;

    pmsg_notice2("%s(): snoozing for %d ms\n", __func__, timetosnooze);
    while(timetosnooze--)
      usleep(1000);
    pmsg_notice2("%s(): done snoozing\n", __func__);
  } else if(mywiring.noautoreset == false) {
    // This code assumes a negative-logic USB to TTL serial adapter
    // Set RTS/DTR high to discharge the series-capacitor, if present
    pmsg_notice2("%s(): releasing DTR/RTS\n", __func__);
    serial_set_dtr_rts(&pgm->fd, 0);
    usleep(50*1000);

    // Pull the RTS/DTR line low to reset AVR
    pmsg_notice2("%s(): asserting DTR/RTS\n", __func__);
    serial_set_dtr_rts(&pgm->fd, 1);

    // Max 100 us: charging a cap longer creates a high reset spike above Vcc
    usleep(100);
    // Set the RTS/DTR line back to high, so direct connection to reset works
    serial_set_dtr_rts(&pgm->fd, 0);

    int delay = mywiring.delay;

    if((100 + delay) > 0)
      usleep((100 + delay)*1000);     // Wait until board comes out of reset
  }
  // Drain any extraneous input
  stk500v2_drain(pgm, 0);

  if(stk500v2_getsync(pgm) < 0) {
    pmsg_error("stk500v2_getsync() failed; try -x delay=n with some n in [-80, 100]\n");
    return -1;
  }

  return 0;
}

static void wiring_close(PROGRAMMER *pgm) {
  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}

const char wiring_desc[] = "Bootloader using the STK500v2 protocol (AVR068)";

void wiring_initpgm(PROGRAMMER *pgm) {
  // The Wiring bootloader uses a near-complete STK500v2 protocol
  stk500v2_initpgm(pgm);

  strcpy(pgm->type, "Wiring");

  pgm->open = wiring_open;
  pgm->close = wiring_close;
  pgm->setup = wiring_setup;
  pgm->teardown = wiring_teardown;
  pgm->parseextparams = wiring_parseextparms;
}
