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

#include "lists.h"


/*
 * AVR serial programming instructions
 */
enum {
  AVR_OP_READ,
  AVR_OP_WRITE,
  AVR_OP_READ_LO,
  AVR_OP_READ_HI,
  AVR_OP_WRITE_LO,
  AVR_OP_WRITE_HI,
  AVR_OP_LOADPAGE_LO,
  AVR_OP_LOADPAGE_HI,
  AVR_OP_WRITEPAGE,
  AVR_OP_CHIP_ERASE,
  AVR_OP_PGM_ENABLE,
  AVR_OP_MAX
};


enum {
  AVR_CMDBIT_IGNORE,  /* bit is ignored on input and output */
  AVR_CMDBIT_VALUE,   /* bit is set to 0 or 1 for input or output */
  AVR_CMDBIT_ADDRESS, /* this bit represents an input address bit */
  AVR_CMDBIT_INPUT,   /* this bit is an input bit */
  AVR_CMDBIT_OUTPUT   /* this bit is an output bit */
};

/*
 * serial programming instruction bit specifications
 */
typedef struct cmdbit {
  int          type;  /* AVR_CMDBIT_* */
  int          bitno; /* which input bit to use for this command bit */
  int          value; /* bit value if type == AVR_CMDBIT_VALUD */
} CMDBIT;

typedef struct opcode {
  CMDBIT        bit[32]; /* opcode bit specs */
} OPCODE;


#define AVR_MEMDESCLEN 64
typedef struct avrmem {
  char desc[AVR_MEMDESCLEN];  /* memory description ("flash", "eeprom", etc) */
  int paged;                  /* page addressed (e.g. ATmega flash) */
  int size;                   /* total memory size in bytes */
  int page_size;              /* size of memory page (if page addressed) */
  int num_pages;              /* number of pages (if page addressed) */
  int min_write_delay;        /* microseconds */
  int max_write_delay;        /* microseconds */
  unsigned char readback[2];  /* polled read-back values */
  unsigned char * buf;        /* pointer to memory buffer */
  OPCODE * op[AVR_OP_MAX];    /* opcodes */
} AVRMEM;


#define AVR_DESCLEN 64
#define AVR_IDLEN   32
typedef struct avrpart {
  char          desc[AVR_DESCLEN];  /* long part name */
  char          id[AVR_IDLEN];      /* short part name */
  int           chip_erase_delay;   /* microseconds */
  OPCODE      * op[AVR_OP_MAX];     /* opcodes */

  LISTID        mem;                /* avr memory definitions */
} AVRPART;


extern struct avrpart parts[];



AVRPART * avr_find_part(char * p);

AVRPART * avr_new_part(void);

OPCODE * avr_new_opcode(void);

AVRMEM * avr_new_memtype(void);

AVRPART * avr_dup_part(AVRPART * d);

AVRMEM * avr_locate_mem(AVRPART * p, char * desc);

int avr_txrx_bit(int fd, int bit);

unsigned char avr_txrx(int fd, unsigned char byte);

int avr_cmd(int fd, unsigned char cmd[4], unsigned char res[4]);

int avr_read_byte(int fd, AVRPART * p, AVRMEM * mem, unsigned long addr, 
                  unsigned char * value);

int avr_read(int fd, AVRPART * p, char * memtype, int size, int verbose);

int avr_write_page(int fd, AVRPART * p, AVRMEM * mem,
                   unsigned long addr);

int avr_write_byte(int fd, AVRPART * p, AVRMEM * mem,
                   unsigned long addr, unsigned char data);

int avr_write(int fd, AVRPART * p, char * memtype, int size, int verbose);

int avr_program_enable(int fd, AVRPART * p);

int avr_chip_erase(int fd, AVRPART * p);

int avr_signature(int fd, AVRPART * p);

void avr_powerup(int fd);

void avr_powerdown(int fd);

int avr_initialize(int fd, AVRPART * p);

char * avr_memtstr(int memtype);

int avr_initmem(AVRPART * p);

int avr_verify(AVRPART * p, AVRPART * v, char * memtype, int size);

void avr_display(FILE * f, AVRPART * p, char * prefix);


#endif
