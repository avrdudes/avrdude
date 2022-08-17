/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2015-2020 David Sainty
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

#ifndef xbee_h__
#define xbee_h__

extern const char xbee_desc[];
void xbee_initpgm(PROGRAMMER *pgm);

/*
 * For non-direct mode (Over-The-Air) we need to issue XBee commands
 * to the remote XBee in order to reset the AVR CPU and initiate the
 * XBeeBoot bootloader.
 *
 * XBee IO port 3 is a somewhat-arbitrarily chosen pin that can be
 * connected directly to the AVR reset pin.
 *
 * Note that port 7 was not used because it is the only pin that can
 * be used as a CTS flow control output.  Port 6 is the only pin that
 * can be used as an RTS flow control input.
 *
 * Some off-the-shelf Arduino shields select a different pin.  For
 * example this one uses XBee IO port 7.
 *
 * https://wiki.dfrobot.com/Xbee_Shield_For_Arduino__no_Xbee___SKU_DFR0015_
 */
#ifndef XBEE_DEFAULT_RESET_PIN
#define XBEE_DEFAULT_RESET_PIN 3
#endif

#endif
