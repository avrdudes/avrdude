/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2024  Dawid Buchwald
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

#ifndef updi_nvm_v5_h
#define updi_nvm_v5_h

#include "libavrdude.h"

#ifdef __cplusplus
extern "C" {
#endif

int updi_nvm_chip_erase_V5(const PROGRAMMER *pgm, const AVRPART *p);
int updi_nvm_erase_flash_page_V5(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address);
int updi_nvm_erase_eeprom_V5(const PROGRAMMER *pgm, const AVRPART *p);
int updi_nvm_erase_user_row_V5(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size);
int updi_nvm_write_flash_V5(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size);
int updi_nvm_write_user_row_V5(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size);
int updi_nvm_write_eeprom_V5(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size);
int updi_nvm_write_fuse_V5(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value);
int updi_nvm_wait_ready_V5(const PROGRAMMER *pgm, const AVRPART *p);
int updi_nvm_command_V5(const PROGRAMMER *pgm, const AVRPART *p, uint8_t command);

#ifdef __cplusplus
}
#endif

#endif /* updi_nvm_v5_h */
