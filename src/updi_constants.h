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

// FLASH CONTROLLER
#define UPDI_NVMCTRL_CTRLA    0x00
#define UPDI_NVMCTRL_CTRLB    0x01
#define UPDI_NVMCTRL_STATUS   0x02
#define UPDI_NVMCTRL_INTCTRL  0x03
#define UPDI_NVMCTRL_INTFLAGS 0x04
#define UPDI_NVMCTRL_DATAL    0x06
#define UPDI_NVMCTRL_DATAH    0x07
#define UPDI_NVMCTRL_ADDRL    0x08
#define UPDI_NVMCTRL_ADDRH    0x09

// NVMCTRL v0 CTRLA
#define UPDI_V0_NVMCTRL_CTRLA_NOP              0x00
#define UPDI_V0_NVMCTRL_CTRLA_WRITE_PAGE       0x01
#define UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE       0x02
#define UPDI_V0_NVMCTRL_CTRLA_ERASE_WRITE_PAGE 0x03
#define UPDI_V0_NVMCTRL_CTRLA_PAGE_BUFFER_CLR  0x04
#define UPDI_V0_NVMCTRL_CTRLA_CHIP_ERASE       0x05
#define UPDI_V0_NVMCTRL_CTRLA_ERASE_EEPROM     0x06
#define UPDI_V0_NVMCTRL_CTRLA_WRITE_FUSE       0x07

// NVMCTRL v2 CTRLA
#define UPDI_V2_NVMCTRL_CTRLA_NOCMD              0x00
#define UPDI_V2_NVMCTRL_CTRLA_FLASH_WRITE        0x02
#define UPDI_V2_NVMCTRL_CTRLA_FLASH_PAGE_ERASE   0x08
#define UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE_WRITE 0x13
#define UPDI_V2_NVMCTRL_CTRLA_CHIP_ERASE         0x20
#define UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE       0x30

// NVMCTRL v3 CTRLA
#define UPDI_V3_NVMCTRL_CTRLA_NOCMD                    0x00
#define UPDI_V3_NVMCTRL_CTRLA_NOP                      0x01
#define UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_WRITE         0x04
#define UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_ERASE_WRITE   0x05
#define UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_ERASE         0x08
#define UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_BUFFER_CLEAR  0x0F
#define UPDI_V3_NVMCTRL_CTRLA_EEPROM_PAGE_WRITE        0x14
#define UPDI_V3_NVMCTRL_CTRLA_EEPROM_PAGE_ERASE_WRITE  0x15
#define UPDI_V3_NVMCTRL_CTRLA_EEPROM_PAGE_ERASE        0x17
#define UPDI_V3_NVMCTRL_CTRLA_EEPROM_PAGE_BUFFER_CLEAR 0x1F
#define UPDI_V3_NVMCTRL_CTRLA_CHIP_ERASE               0x20
#define UPDI_V3_NVMCTRL_CTRLA_EEPROM_ERASE             0x30

#define UPDI_NVM_STATUS_WRITE_ERROR 2
#define UPDI_NVM_STATUS_EEPROM_BUSY 1
#define UPDI_NVM_STATUS_FLASH_BUSY  0

#endif /* updi_constants_h */
