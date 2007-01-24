/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef __avr_h__
#define __avr_h__

#include <stdio.h>

#include "avrpart.h"
#include "pgm.h"

typedef void (*FP_UpdateProgress)(int percent, double etime, char *hdr);

extern struct avrpart parts[];

extern FP_UpdateProgress update_progress;

#ifdef __cplusplus
extern "C" {
#endif

int avr_read_byte_default(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			  unsigned long addr, unsigned char * value);

int avr_read(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size,
             int verbose);

int avr_write_page(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                   unsigned long addr);

int avr_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                   unsigned long addr, unsigned char data);

int avr_write_byte_default(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
			   unsigned long addr, unsigned char data);

int avr_write(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size,
              int verbose);

int avr_signature(PROGRAMMER * pgm, AVRPART * p);

int avr_verify(AVRPART * p, AVRPART * v, char * memtype, int size);

int avr_get_cycle_count(PROGRAMMER * pgm, AVRPART * p, int * cycles);

int avr_put_cycle_count(PROGRAMMER * pgm, AVRPART * p, int cycles);;

int avr_mem_hiaddr(AVRMEM * mem);

int avr_chip_erase(PROGRAMMER * pgm, AVRPART * p);

void report_progress (int completed, int total, char *hdr);

#ifdef __cplusplus
}
#endif

#endif
