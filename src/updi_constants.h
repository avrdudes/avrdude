/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2021  Dawid Buchwald
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

/*
 * Based on pymcuprog
 * See https://github.com/microchip-pic-avr-tools/pymcuprog
 */

#ifndef updi_constants_h
#define updi_constants_h

#define UPDI_BREAK       0x00

#define UPDI_LDS         0x00
#define UPDI_STS         0x40
#define UPDI_LD          0x20
#define UPDI_ST          0x60
#define UPDI_LDCS        0x80
#define UPDI_STCS        0xC0
#define UPDI_REPEAT      0xA0
#define UPDI_KEY         0xE0

#define UPDI_PTR         0x00
#define UPDI_PTR_INC     0x04
#define UPDI_PTR_ADDRESS 0x08

#define UPDI_ADDRESS_8   0x00
#define UPDI_ADDRESS_16  0x04
#define UPDI_ADDRESS_24  0x08

#define UPDI_DATA_8      0x00
#define UPDI_DATA_16     0x01
#define UPDI_DATA_24     0x02

#define UPDI_KEY_SIB     0x04
#define UPDI_KEY_KEY     0x00

#define UPDI_KEY_64      0x00
#define UPDI_KEY_128     0x01
#define UPDI_KEY_256     0x02

#define UPDI_SIB_8BYTES  UPDI_KEY_64
#define UPDI_SIB_16BYTES UPDI_KEY_128
#define UPDI_SIB_32BYTES UPDI_KEY_256

#define UPDI_REPEAT_BYTE 0x00
#define UPDI_REPEAT_WORD 0x01

#define UPDI_PHY_SYNC    0x55
#define UPDI_PHY_ACK     0x40

#define UPDI_MAX_REPEAT_SIZE (0xFF+1) // Repeat counter of 1-byte, with off-by-one counting

//# CS and ASI Register Address map
#define UPDI_CS_STATUSA     0x00
#define UPDI_CS_STATUSB     0x01
#define UPDI_CS_CTRLA       0x02
#define UPDI_CS_CTRLB       0x03
#define UPDI_ASI_KEY_STATUS 0x07
#define UPDI_ASI_RESET_REQ  0x08
#define UPDI_ASI_CTRLA      0x09
#define UPDI_ASI_SYS_CTRLA  0x0A
#define UPDI_ASI_SYS_STATUS 0x0B
#define UPDI_ASI_CRC_STATUS 0x0C

#define UPDI_CTRLA_IBDLY_BIT    7
#define UPDI_CTRLB_CCDETDIS_BIT 3
#define UPDI_CTRLB_UPDIDIS_BIT  2

#define UPDI_KEY_NVM       "NVMProg "
#define UPDI_KEY_CHIPERASE "NVMErase"
#define UPDI_KEY_UROW      "NVMUs&te"

#define UPDI_ASI_STATUSA_REVID 4
#define UPDI_ASI_STATUSB_PESIG 0

#define UPDI_ASI_KEY_STATUS_CHIPERASE  3
#define UPDI_ASI_KEY_STATUS_NVMPROG    4
#define UPDI_ASI_KEY_STATUS_UROWWRITE  5

#define UPDI_ASI_SYS_STATUS_RSTSYS     5
#define UPDI_ASI_SYS_STATUS_INSLEEP    4
#define UPDI_ASI_SYS_STATUS_NVMPROG    3
#define UPDI_ASI_SYS_STATUS_UROWPROG   2
#define UPDI_ASI_SYS_STATUS_LOCKSTATUS 0

#define UPDI_ASI_SYS_CTRLA_UROW_FINAL  1

#define UPDI_RESET_REQ_VALUE  0x59

#endif /* updi_constants_h */
