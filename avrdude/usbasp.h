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


#define	USBASP_SHARED_VID   0x16C0  /* VOTI */
#define	USBASP_SHARED_PID   0x05DC  /* Obdev's free shared PID */

#define	USBASP_OLD_VID      0x03EB  /* ATMEL */
#define	USBASP_OLD_PID	    0xC7B4  /* (unoffical) USBasp */

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

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

#ifdef __cplusplus
extern "C" {
#endif

void usbasp_initpgm (PROGRAMMER * pgm);

#ifdef __cplusplus
}
#endif

#endif /* usbasp_h */
