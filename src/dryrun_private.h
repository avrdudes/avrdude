/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2023 Stefan Rueger <stefan.rueger@urclocks.com>
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

#ifndef dryrun_private_h__
#define dryrun_private_h__

// STK500v1 protocol constants

#define Resp_STK_OK             0x10
#define Resp_STK_INSYNC         0x14

#define Sync_CRC_EOP            0x20

#define Cmnd_STK_GET_SYNC       0x30
#define Cmnd_STK_ENTER_PROGMODE 0x50
#define Cmnd_STK_LEAVE_PROGMODE 0x51
#define Cmnd_STK_CHIP_ERASE     0x52
#define Cmnd_STK_LOAD_ADDRESS   0x55
#define Cmnd_STK_UNIVERSAL      0x56

#define Cmnd_STK_PROG_PAGE      0x64
#define Cmnd_STK_READ_PAGE      0x74
#define Cmnd_STK_READ_SIGN      0x75

// STK_UNIVERSAL commands
#define Subc_STK_UNIVERSAL_LEXT 0x4d000000u // Load extended address
#define Subc_STK_UNIVERSAL_CE   0xac800000u // Chip erase

#endif
