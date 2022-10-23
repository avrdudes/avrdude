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

#ifndef updi_readwrite_h
#define updi_readwrite_h

#include "libavrdude.h"

#ifdef __cplusplus
extern "C" {
#endif

int updi_read_cs(const PROGRAMMER *pgm, uint8_t address, uint8_t *value);
int updi_write_cs(const PROGRAMMER *pgm, uint8_t address, uint8_t value);
int updi_write_key(const PROGRAMMER *pgm, unsigned char *buffer, uint8_t size_type, uint16_t size);
int updi_read_sib(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t size);
int updi_read_byte(const PROGRAMMER *pgm, uint32_t address, uint8_t *value);
int updi_write_byte(const PROGRAMMER *pgm, uint32_t address, uint8_t value);
int updi_read_data(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size);
int updi_write_data(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size);
int updi_read_data_words(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size);
int updi_write_data_words(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* updi_readwrite_h */
