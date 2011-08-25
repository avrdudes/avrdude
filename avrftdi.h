/*
 * avrftdi - extension for avrdude, Wolfgang Moser, Ville Voipio
 * Copyright (C) 2011 Hannes Weisbach, Doug Springer
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

#ifndef avrftdi_h
#define avrfdti_h

#ifdef __cplusplus
extern "C" {
#endif

#define SCK 0x01
#define SDO 0x02
#define SDI 0x04

#define RX 0x20
#define TX 0x11

#define TRX (RX | TX)

#define TYPE_C_D 0x500
#define TYPE_H   0x700
#define TYPE_4H  0x800

#define E(x) if ((x)) { fprintf(stdout, "%s:%d %s() %s: %s (%d)\n\t%s\n", __FILE__, __LINE__, __FUNCTION__, \
	#x, strerror(errno), errno, ftdi_get_error_string(&ftdic)); return -1; }

#define E_VOID(x) if ((x)) { fprintf(stdout, "%s:%d %s() %s: %s (%d)\n\t%s\n", __FILE__, __LINE__, __FUNCTION__, \
	#x, strerror(errno), errno, ftdi_get_error_string(&ftdic)); }

void avrftdi_initpgm        (PROGRAMMER * pgm);

#ifdef __cplusplus
}
#endif

#endif


