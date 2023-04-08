/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2005 Juliane Holzt <avrdude@juliane.holzt.de>
 * Copyright (C) 2011 Darell Tan <darell.tan@gmail.com>
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

#ifndef bitbang_h
#define bitbang_h

#ifdef __cplusplus
extern "C" {
#endif

int bitbang_setpin(int fd, int pin, int value);
int bitbang_getpin(int fd, int pin);
int bitbang_highpulsepin(int fd, int pin);
void bitbang_delay(unsigned int us);

int bitbang_check_prerequisites(const PROGRAMMER *pgm);

int  bitbang_rdy_led        (const PROGRAMMER *pgm, int value);
int  bitbang_err_led        (const PROGRAMMER *pgm, int value);
int  bitbang_pgm_led        (const PROGRAMMER *pgm, int value);
int  bitbang_vfy_led        (const PROGRAMMER *pgm, int value);
int  bitbang_cmd            (const PROGRAMMER *pgm, const unsigned char *cmd,
                                unsigned char *res);
int  bitbang_cmd_tpi        (const PROGRAMMER *pgm, const unsigned char *cmd,
                                int cmd_len, unsigned char *res, int res_len);
int  bitbang_cmd_hvsp       (const PROGRAMMER *pgm, const unsigned char *cmd,
				const unsigned char *data, unsigned char *res);
int  bitbang_spi            (const PROGRAMMER *pgm, const unsigned char *cmd,
                                unsigned char *res, int count);
int  bitbang_chip_erase     (const PROGRAMMER *pgm, const AVRPART *p);
int  bitbang_program_enable (const PROGRAMMER *pgm, const AVRPART *p);
void bitbang_powerup        (const PROGRAMMER *pgm);
void bitbang_powerdown      (const PROGRAMMER *pgm);
int  bitbang_initialize     (const PROGRAMMER *pgm, const AVRPART *p);
void bitbang_disable        (const PROGRAMMER *pgm);
void bitbang_enable         (PROGRAMMER *pgm, const AVRPART *p);

#ifdef __cplusplus
}
#endif

#endif
