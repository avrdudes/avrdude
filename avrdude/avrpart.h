/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef __avrpart_h__
#define __avrpart_h__

#include <limits.h>

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

enum { /* these are assigned to reset_disposition of AVRPART */
  RESET_DEDICATED,    /* reset pin is dedicated */
  RESET_IO            /* reset pin might be configured as an I/O pin */
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


#define AVRPART_SERIALOK       0x0001  /* part supports serial programming */
#define AVRPART_PARALLELOK     0x0002  /* part supports parallel programming */
#define AVRPART_PSEUDOPARALLEL 0x0004  /* part has pseudo parallel support */

#define AVR_DESCLEN 64
#define AVR_IDLEN   32
typedef struct avrpart {
  char          desc[AVR_DESCLEN];  /* long part name */
  char          id[AVR_IDLEN];      /* short part name */
  int           stk500_devcode;     /* stk500 device code */
  int           avr910_devcode;     /* avr910 device code */
  int           chip_erase_delay;   /* microseconds */
  unsigned char pagel;              /* for parallel programming */
  unsigned char bs2;                /* for parallel programming */
  int           reset_disposition;  /* see RESET_ enums */
  int           retry_pulse;        /* retry program enable by pulsing
                                       this pin (PIN_AVR_*) */
  unsigned      flags;              /* see AVRPART_ masks */

  OPCODE      * op[AVR_OP_MAX];     /* opcodes */

  LISTID        mem;                /* avr memory definitions */
  char          config_file[PATH_MAX]; /* config file where defined */
  int           lineno;                /* config file line number */
} AVRPART;

#define AVR_MEMDESCLEN 64
typedef struct avrmem {
  char desc[AVR_MEMDESCLEN];  /* memory description ("flash", "eeprom", etc) */
  int paged;                  /* page addressed (e.g. ATmega flash) */
  int size;                   /* total memory size in bytes */
  int page_size;              /* size of memory page (if page addressed) */
  int num_pages;              /* number of pages (if page addressed) */
  int min_write_delay;        /* microseconds */
  int max_write_delay;        /* microseconds */
  int pwroff_after_write;     /* after this memory type is written to,
                                 the device must be powered off and
                                 back on, see errata
                                 http://www.atmel.com/atmel/acrobat/doc1280.pdf */
  unsigned char readback[2];  /* polled read-back values */
  unsigned char * buf;        /* pointer to memory buffer */
  OPCODE * op[AVR_OP_MAX];    /* opcodes */
} AVRMEM;

/* Functions for OPCODE structures */
OPCODE * avr_new_opcode(void);
int avr_set_bits(OPCODE * op, unsigned char * cmd);
int avr_set_addr(OPCODE * op, unsigned char * cmd, unsigned long addr);
int avr_set_input(OPCODE * op, unsigned char * cmd, unsigned char data);
int avr_get_output(OPCODE * op, unsigned char * res, unsigned char * data);

/* Functions for AVRMEM structures */
AVRMEM * avr_new_memtype(void);
int avr_initmem(AVRPART * p);
AVRMEM * avr_dup_mem(AVRMEM * m);
AVRMEM * avr_locate_mem(AVRPART * p, char * desc);
void avr_mem_display(char * prefix, FILE * f, AVRMEM * m, int type,
                     int verbose);

/* Functions for AVRPART structures */
AVRPART * avr_new_part(void);
AVRPART * avr_dup_part(AVRPART * d);
AVRPART * locate_part(LISTID parts, char * partdesc);
AVRPART * locate_part_by_avr910_devcode(LISTID parts, int devcode);
void list_parts(FILE * f, char * prefix, LISTID parts);
void avr_display(FILE * f, AVRPART * p, char * prefix, int verbose);

#endif
