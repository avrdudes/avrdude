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

#ifndef updi_link_h
#define updi_link_h

#include "libavrdude.h"

#ifdef __cplusplus
extern "C" {
#endif

int updi_link_open(PROGRAMMER * pgm);
void updi_link_close(PROGRAMMER * pgm);
int updi_link_init(const PROGRAMMER *pgm);
int updi_link_ldcs(const PROGRAMMER *pgm, uint8_t address, uint8_t *value);
int updi_link_stcs(const PROGRAMMER *pgm, uint8_t address, uint8_t value);
int updi_link_ld_ptr_inc(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t size);
int updi_link_ld_ptr_inc16(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t words);
int updi_link_st_ptr_inc(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t size);
int updi_link_st_ptr_inc16(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t words);
int updi_link_st_ptr_inc16_RSD(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t words, int blocksize);
int updi_link_repeat(const PROGRAMMER *pgm, uint16_t repeats);
int updi_link_read_sib(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t size);
int updi_link_key(const PROGRAMMER *pgm, unsigned char *buffer, uint8_t size_type, uint16_t size);
int updi_link_ld(const PROGRAMMER *pgm, uint32_t address, uint8_t *value);
int updi_link_ld16(const PROGRAMMER *pgm, uint32_t address, uint16_t *value);
int updi_link_st(const PROGRAMMER *pgm, uint32_t address, uint8_t value);
int updi_link_st16(const PROGRAMMER *pgm, uint32_t address, uint16_t value);
int updi_link_st_ptr(const PROGRAMMER *pgm, uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* updi_link_h */
