#ifndef __avrpart_h__
#define __avrpart_h__

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


#define AVR_DESCLEN 64
#define AVR_IDLEN   32
typedef struct avrpart {
  char          desc[AVR_DESCLEN];  /* long part name */
  char          id[AVR_IDLEN];      /* short part name */
  int           devicecode;         /* Atmel STK500 device code */
  int           chip_erase_delay;   /* microseconds */
  OPCODE      * op[AVR_OP_MAX];     /* opcodes */

  LISTID        mem;                /* avr memory definitions */
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

#endif
