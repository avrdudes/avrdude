/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef __pindefs_h__
#define __pindefs_h__

#include <limits.h>

enum {
  PPI_AVR_VCC=1,
  PPI_AVR_BUFF,
  PIN_AVR_RESET,
  PIN_AVR_SCK,
  PIN_AVR_MOSI,
  PIN_AVR_MISO,
  PIN_LED_ERR,
  PIN_LED_RDY,
  PIN_LED_PGM,
  PIN_LED_VFY,
  N_PINS
};
#define PIN_MASK    (UINT_MAX>>1)
#define PIN_INVERSE (~(PIN_MASK))	/* flag for inverted pin in serbb */

#define LED_ON(fd,pin)  ppi_setpin(fd,pin,0)
#define LED_OFF(fd,pin) ppi_setpin(fd,pin,1)

#endif
