/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2006  Thomas Fischl
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

#ifndef usbasp_h
#define usbasp_h

#include "avrpart.h"


#define	USBDEV_VENDOR	0x03eb	/* ATMEL */
#define	USBDEV_PRODUCT	0xc7B4 	/* USBasp */

#define USBASP_FUNC_CONNECT    1
#define USBASP_FUNC_DISCONNECT 2
#define USBASP_FUNC_TRANSMIT   3
#define USBASP_FUNC_READFLASH  4
#define USBASP_FUNC_ENABLEPROG 5
#define USBASP_FUNC_WRITEFLASH 6
#define USBASP_FUNC_READEEPROM 7
#define USBASP_FUNC_WRITEEEPROM 8

#define USBASP_BLOCKFLAG_FIRST    1
#define USBASP_BLOCKFLAG_LAST     2

#define USBASP_READBLOCKSIZE   200
#define USBASP_WRITEBLOCKSIZE  200


void usbasp_initpgm (PROGRAMMER * pgm);

#endif /* usbasp_h */
