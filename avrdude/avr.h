/*
 * Copyright 2001  Brian S. Dean <bsd@bsdhome.com>
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

/*
 * AVR memory designations; the order of these is important, these are
 * used as indexes into statically initialized data, don't change them
 * around.  Specifically, avr_read_byte() and avr_write_byte() rely on
 * the order.
 */
#define AVR_M_EEPROM 0
#define AVR_M_FLASH  1

#define AVR_MAXMEMTYPES 2     /* just flash and eeprom */

#if 0
struct avrmem {
  int             startaddr;
  int             size;
  unsigned char * buf;
  struct avrmem * next;
};
#endif

typedef struct avrmem {
  int banked;                   /* bank addressed (e.g. ATmega flash) */
  int size;                     /* total memory size in bytes */
  int bank_size;                /* size of memory bank (if bank addressed) */
  int num_banks;                /* number of banks (if bank addressed) */
  int min_write_delay;          /* microseconds */
  int max_write_delay;          /* microseconds */
  unsigned char readback[2];    /* polled read-back values */
  unsigned char * buf;          /* pointer to memory buffer */
} AVRMEM;


struct avrpart {
  char          * partdesc;         /* long part name */
  char          * optiontag;        /* short part name */

  int             chip_erase_delay; /* microseconds */

  AVRMEM          mem[AVR_MAXMEMTYPES];

#if 0
  int             memsize[AVR_MAXMEMTYPES]; /* sizes for eeprom,
                                               flash, etc, indexed by
                                               AVR_EEPROM or AVR_FLASH */
  unsigned char   f_readback;       /* flash write polled readback value */
  unsigned char   e_readback[2];    /* eeprom write polled readback values */
  int             min_write_delay;  /* microseconds */
  int             max_write_delay;  /* microseconds */
#if 1
  unsigned char * mem[AVR_MAXMEMTYPES]; /* pointers to avr memory
                                           buffers, indexed by
                                           AVR_EEPROM or AVR_FLASH */
#else
  struct avrmem * mem[AVR_MAXMEMTYPES]; /* pointers to avr memory
                                           buffers, indexed by
                                           AVR_EEPROM or AVR_FLASH */
#endif
#endif
};


extern struct avrpart parts[];



int avr_list_parts(FILE * f, char * prefix);

struct avrpart * avr_find_part(char * p);

int avr_txrx_bit(int fd, int bit);

unsigned char avr_txrx(int fd, unsigned char byte);

int avr_cmd(int fd, unsigned char cmd[4], unsigned char res[4]);

unsigned char avr_read_byte(int fd, struct avrpart * p,
                              int memtype, unsigned long addr);

int avr_read(int fd, struct avrpart * p, int memtype);

int avr_write_bank(int fd, struct avrpart * p, int memtype, 
                   unsigned short bank);

int avr_write_byte(int fd, struct avrpart * p, int memtype, 
                     unsigned long addr, unsigned char data);

int avr_write(int fd, struct avrpart * p, int memtype, int size);

int avr_program_enable(int fd);

int avr_chip_erase(int fd, struct avrpart * p);

int avr_signature(int fd, unsigned char sig[4]);

void avr_powerup(int fd);

void avr_powerdown(int fd);

int avr_initialize(int fd, struct avrpart * p);

char * avr_memtstr(int memtype);

int avr_initmem(struct avrpart * p);

int avr_verify(struct avrpart * p, struct avrpart * v, int memtype, 
               int size);

void avr_display(FILE * f, struct avrpart * p, char * prefix);


#endif
