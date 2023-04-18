/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2004  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef stk500_h
#define stk500_h

#ifdef __cplusplus
extern "C" {
#endif

extern const char stk500_desc[];
void stk500_initpgm(PROGRAMMER *pgm);

/* used by arduino.c to avoid duplicate code */
int stk500_getsync(const PROGRAMMER *pgm);
int stk500_drain(const PROGRAMMER *pgm, int display);

#ifdef __cplusplus
}
#endif

#include "xbee.h"

struct pdata {
  unsigned char ext_addr_byte;  // Record ext-addr byte set in the target device (if used)
  int retry_attempts;           // Number of connection attempts provided by the user
  int xbeeResetPin;             // Piggy back variable used by xbee programmmer

  // Get/set flags for adjustable target voltage
  bool vtarg_get;
  bool vtarg_set;
  double vtarg_data;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

#endif


