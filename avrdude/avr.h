/*
 * Copyright (c) 2000, 2001, 2002  Brian S. Dean <bsd@bsdhome.com>
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BRIAN S. DEAN ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BRIAN S. DEAN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * 
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

#endif
