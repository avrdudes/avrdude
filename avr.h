/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
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
#include "lists.h"
#include "pgm.h"



extern struct avrpart parts[];



AVRPART * avr_find_part(char * p);

AVRPART * avr_new_part(void);

OPCODE * avr_new_opcode(void);

AVRMEM * avr_new_memtype(void);

AVRPART * avr_dup_part(AVRPART * d);

AVRMEM * avr_locate_mem(AVRPART * p, char * desc);

int avr_txrx_bit(int fd, int bit);

unsigned char avr_txrx(int fd, unsigned char byte);

int avr_set_bits(OPCODE * op, unsigned char * cmd);

int avr_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem, 
                  unsigned long addr, unsigned char * value);

int avr_read(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size, 
             int verbose);

int avr_write_page(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                   unsigned long addr);

int avr_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                   unsigned long addr, unsigned char data);

int avr_write(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size, 
              int verbose);

int avr_signature(PROGRAMMER * pgm, AVRPART * p);

char * avr_memtstr(int memtype);

int avr_initmem(AVRPART * p);

int avr_verify(AVRPART * p, AVRPART * v, char * memtype, int size);

void avr_mem_display(char * prefix, FILE * f, AVRMEM * m, int type, 
                     int verbose);

void avr_display(FILE * f, AVRPART * p, char * prefix, int verbose);

int avr_get_cycle_count(PROGRAMMER * pgm, AVRPART * p, int * cycles);

int avr_put_cycle_count(PROGRAMMER * pgm, AVRPART * p, int cycles);;

int avr_mem_hiaddr(AVRMEM * mem);

#endif
