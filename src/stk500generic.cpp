/*
 * avrdude - A Downloader/Uploader for AVR device programmers
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* $Id$ */

/*
 * avrdude interface for Atmel STK500 programmer
 *
 * This is a wrapper around the STK500[v1] and STK500v2 programmers.
 * Try to select the programmer type that actually responds, and
 * divert to the actual programmer implementation if successful.
 * Using -c stk500 instead of the appropriate -c stk500v1 or -c stk500v2
 * misses out on the extended -x options of the successful programmer.
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "stk500generic.h"
#include "stk500.h"
#include "stk500v2.h"

static PROGRAMMER *stk500vn;
static void stk500vn_exithook(void) {
  if(stk500vn && stk500vn->teardown)
    stk500vn->teardown(stk500vn);
}

static int stk500generic_open(PROGRAMMER *pgm, const char *port) {
  stk500v2_initpgm(pgm);
  if(pgm->setup)
    pgm->setup(pgm);
  if(pgm->open(pgm, port) >= 0) {
    stk500vn = pgm;
    atexit(stk500vn_exithook);
    pmsg_info("successfully opened stk500v2 device; please use -c stk500v2\n");
    return 0;
  }
  if(pgm->teardown)
    pgm->teardown(pgm);

  stk500_initpgm(pgm);
  if(pgm->setup)
    pgm->setup(pgm);
  if(pgm->open(pgm, port) >= 0) {
    stk500vn = pgm;
    atexit(stk500vn_exithook);
    pmsg_info("successfully opened stk500v1 device; please use -c stk500v1\n");
    return 0;
  }
  if(pgm->teardown)
    pgm->teardown(pgm);

  pmsg_error("probing stk500v2 failed, as did stk500v1; perhaps try -c stk500v1\n");
  return -1;
}

const char stk500generic_desc[] = "Atmel STK500, autodetect firmware version";

void stk500generic_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "STK500GENERIC");

  pgm->open           = stk500generic_open;
}
