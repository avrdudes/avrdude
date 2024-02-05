/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) Joerg Wunsch
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

#ifndef libavrdude_h
#define libavrdude_h

#include <stdio.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include "avrintel.h"

typedef uint32_t pinmask_t;
/*
 * Values returned by library functions.
 * Some library functions also return a count, i.e. a positive
 * number greater than 0.
 */
#define LIBAVRDUDE_SUCCESS 0
#define LIBAVRDUDE_GENERAL_FAILURE (-1)
#define LIBAVRDUDE_NOTSUPPORTED (-2) // operation not supported
#define LIBAVRDUDE_SOFTFAIL (-3) // returned, eg, by avr_signature() if caller
                                 // might proceed with chip erase

/* formerly lists.h */

/*----------------------------------------------------------------------
  General purpose linked list routines - header file declarations.

  Author : Brian Dean
  Date   : 10 January, 1990
  ----------------------------------------------------------------------*/

typedef void * LISTID;
typedef void * LNODEID;


/*----------------------------------------------------------------------
  several defines to access the LIST structure as as stack or a queue
  --- use for program readability
  ----------------------------------------------------------------------*/
#define STACKID LISTID
#define SNODEID LNODEID
#define QUEUEID LISTID
#define QNODEID LNODEID


#define PUSH(s,d)    lins_n(s,d,1)   /* push 'd' onto the stack */
#define POP(s)       lrmv_n(s,1)     /* pop the stack */
#define LOOKSTACK(s) lget_n(s,1)     /* look at the top of the stack, 
					but don't pop */


#define ENQUEUE(q,d) lins_n(q,d,1)   /* put 'd' on the end of the queue */
#define DEQUEUE(q)   lrmv(q)         /* remove next item from the front of 
					the queue */
#define REQUEUE(q,d) ladd(q,d)       /* re-insert (push) item back on the
					front of the queue */
#define LOOKQUEUE(q) lget(q)         /* return next item on the queue, 
					but don't dequeue */
#define QUEUELEN(q)  lsize(q)       /* length of the queue */


#define LISTADD(l,d) ladd(l,d)       /* add to end of the list */
#define LISTRMV(l,d) lrmv_d(l,d)     /* remove from end of the list */


#ifdef __cplusplus
extern "C" {
#endif

/* .................... Function Prototypes .................... */

LISTID     lcreat      ( void * liststruct, int poolsize );
void       ldestroy    ( LISTID lid );
void       ldestroy_cb ( LISTID lid, void (*ucleanup)(void * data_ptr) );

LNODEID    lfirst ( LISTID  ); /* head of the list */
LNODEID    llast  ( LISTID  ); /* tail of the list */
LNODEID    lnext  ( LNODEID ); /* next item in the list */
LNODEID    lprev  ( LNODEID ); /* previous item in the list */
void     * ldata  ( LNODEID ); /* data at the current position */
int        lsize  ( LISTID  ); /* number of elements in the list */

int        ladd     ( LISTID lid, void * p );
int        laddo    ( LISTID lid, void *p, 
		      int (*compare)(const void *p1,const void *p2),
		      LNODEID * firstdup );
int        laddu    ( LISTID lid, void * p, 
		      int (*compare)(const void *p1,const void *p2));
int        lins_n   ( LISTID lid, void * d, unsigned int n );
int        lins_ln  ( LISTID lid, LNODEID lnid, void * data_ptr );

void     * lget    ( LISTID lid );
void     * lget_n  ( LISTID lid, unsigned int n );
LNODEID    lget_ln ( LISTID lid, unsigned int n );

void     * lrmv    ( LISTID lid );
void     * lrmv_n  ( LISTID lid, unsigned int n );
void     * lrmv_ln ( LISTID lid, LNODEID lnid );
void     * lrmv_d  ( LISTID lid, void * data_ptr );

LISTID     lcat    ( LISTID lid1, LISTID lid2 );

void       lsort   ( LISTID lid, int (*compare)(void * p1, void * p2));

void     * lsrch   ( LISTID lid, void * p, int (*compare)(void *p1,void *p2));

int        lprint  ( FILE * f, LISTID lid );

#ifdef __cplusplus
}
#endif

/* formerly avrpart.h */

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
  AVR_OP_LOAD_EXT_ADDR,
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

enum ctl_stack_t {
  CTL_STACK_NONE,     /* no control stack defined */
  CTL_STACK_PP,	      /* parallel programming control stack */
  CTL_STACK_HVSP      /* high voltage serial programming control stack */
};


/*
 * serial programming instruction bit specifications
 */
typedef struct cmdbit {
  int          type;  /* AVR_CMDBIT_* */
  int          bitno; /* which input bit to use for this command bit */
  int          value; /* bit value if type == AVR_CMDBIT_VALUE */
} CMDBIT;

typedef struct opcode {
  CMDBIT        bit[32]; /* opcode bit specs */
} OPCODE;


// Any changes here, please also reflect in dev_part_strct() of developer_opts.c
#define AVRPART_SERIALOK       1 // Part supports serial programming
#define AVRPART_PARALLELOK     2 // Part supports parallel programming
#define AVRPART_PSEUDOPARALLEL 4 // Part has pseudo parallel support
#define AVRPART_ALLOWFULLPAGEBITSTREAM 8 // JTAG ICE mkII param
#define AVRPART_ENABLEPAGEPROGRAMMING 16 // JTAG ICE mkII param
#define AVRPART_IS_AT90S1200  32 // Part is an AT90S1200, needs special treatment

// Programming modes for parts and programmers: reflect changes in lexer.l, developer_opts.c and config.c
#define PM_SPM                1 // Bootloaders, self-programming with SPM opcodes or NVM Controllers
#define PM_TPI                2 // Tiny Programming Interface (t4, t5, t9, t10, t20, t40, t102, t104)
#define PM_ISP                4 // SPI programming for In-System Programming (almost all classic parts)
#define PM_PDI                8 // Program and Debug Interface (xmega parts)
#define PM_UPDI              16 // Unified Program and Debug Interface
#define PM_HVSP              32 // High Voltage Serial Programming (some classic parts)
#define PM_HVPP              64 // High Voltage Parallel Programming (most non-HVSP classic parts)
#define PM_debugWIRE        128 // Simpler alternative to JTAG (a subset of HVPP/HVSP parts)
#define PM_JTAG             256 // Joint Test Action Group standard (some classic parts)
#define PM_JTAGmkI          512 // Subset of PM_JTAG, older parts, Atmel ICE mkI
#define PM_XMEGAJTAG       1024 // JTAG, some XMEGA parts
#define PM_AVR32JTAG       2048 // JTAG for 32-bit AVRs
#define PM_aWire           4096 // For 32-bit AVRs
#define PM_ALL           0x1fff // All programming interfaces

#define HV_UPDI_VARIANT_0      0 /* Shared UPDI/GPIO/RESET pin, HV on UPDI pin (tinyAVR0/1/2)*/
#define HV_UPDI_VARIANT_1      1 /* Dedicated UPDI pin, no HV (megaAVR0/AVR-Dx) */
#define HV_UPDI_VARIANT_2      2 /* Shared UPDI pin, HV on _RESET (AVR-DD/AVR-Ex) */

#define HAS_SUFFER             1
#define HAS_VTARG_SWITCH       2
#define HAS_VTARG_ADJ          4
#define HAS_VTARG_READ         8
#define HAS_FOSC_ADJ          16
#define HAS_VAREF_ADJ         32

#define AVR_FAMILYIDLEN 7
#define AVR_SIBLEN 32
#define AVR_CHIP_REVLEN 1
#define CTL_STACK_SIZE 32
#define FLASH_INSTR_SIZE 3
#define EEPROM_INSTR_SIZE 20

#define TAG_ALLOCATED          1    /* memory byte is allocated */

/*
 * Any changes in AVRPART or AVRMEM, please also ensure changes are made in
 *  - lexer.l
 *  - Either Component_t avr_comp[] of config.c or in config_gram.y
 *  - dev_part_strct() in developer_opts.c
 *  - avr_new_part() and/or avr_new_mem() in avrpart.c for
 *    initialisation; note that all const char * must be initialised with ""
 */
typedef struct avrpart {
  const char  * desc;               /* long part name */
  const char  * id;                 /* short part name */
  LISTID        comments;           // Used by developer options -p*/[ASsr...]
  LISTID        variants;           /* String with variant name and chip properties */
  const char  * parent_id;          /* Used by developer options */
  const char  * family_id;          /* family id in the SIB (avr8x) */
  int           prog_modes;         /* Programming interfaces, see #define PM_... */
  int           mcuid;              /* Unique id in 0..2039 for urclock programmer */
  int           n_interrupts;       /* Number of interrupts, used for vector bootloaders */
  int           n_page_erase;       /* If set, number of pages erased during NVM erase */
  int           n_boot_sections;    /* Number of boot sections */
  int           boot_section_size;  /* Size of (smallest) boot section, if any */
  int           hvupdi_variant;     /* HV pulse on UPDI pin, no pin or RESET pin */
  int           stk500_devcode;     /* stk500 device code */
  int           avr910_devcode;     /* avr910 device code */
  int           chip_erase_delay;   /* microseconds */
  unsigned char pagel;              /* for parallel programming */
  unsigned char bs2;                /* for parallel programming */
  unsigned char signature[3];       /* expected value of signature bytes */
  unsigned short usbpid;            /* USB DFU product ID (0 = none) */
  int           reset_disposition;  /* see RESET_ enums */
  int           retry_pulse;        /* retry program enable by pulsing
                                       this pin (PIN_AVR_*) */
  unsigned      flags;              /* see AVRPART_ masks */

  /* STK500 v2 parameters from ATDF files */
  int           timeout;
  int           stabdelay;
  int           cmdexedelay;
  int           synchloops;
  int           bytedelay;
  int           pollindex;
  unsigned char pollvalue;
  int           predelay;
  int           postdelay;
  int           pollmethod;

  enum ctl_stack_t ctl_stack_type;  /* what to use the ctl stack for */
  unsigned char controlstack[CTL_STACK_SIZE]; /* stk500v2 PP/HVSP ctl stack */
  unsigned char flash_instr[FLASH_INSTR_SIZE]; /* flash instructions (debugWire, JTAG) */
  unsigned char eeprom_instr[EEPROM_INSTR_SIZE]; /* EEPROM instructions (debugWire, JTAG) */

  /* STK500 v2 hv mode parameters */
  int           hventerstabdelay;
  int           progmodedelay;
  int           latchcycles;
  int           togglevtg;
  int           poweroffdelay;
  int           resetdelayms;
  int           resetdelayus;
  int           hvleavestabdelay;
  int           resetdelay;
  int           chiperasepulsewidth;
  int           chiperasepolltimeout;
  int           chiperasetime;
  int           programfusepulsewidth;
  int           programfusepolltimeout;
  int           programlockpulsewidth;
  int           programlockpolltimeout;
  int           synchcycles;
  int           hvspcmdexedelay;


  /* debugWIRE and/or JTAG ICE mkII XML file parameters */
  unsigned char idr;            /* I/O address of IDR (OCD) reg */
  unsigned char rampz;          /* I/O address of RAMPZ reg */
  unsigned char spmcr;          /* memory address of SPMCR reg */
  unsigned char eecr;           /* memory address of EECR reg */
  unsigned char eind;           /* memory address of EIND reg */
  unsigned int mcu_base;        /* Base address of MCU control block in ATxmega devices */
  unsigned int nvm_base;        /* Base address of NVM controller in ATxmega devices */
  unsigned int ocd_base;        /* Base address of OCD module in AVR8X/UPDI devices */
  unsigned int syscfg_base;     /* Base address of revision ID in AVR8X/UPDI devices */
  int           ocdrev;         /* OCD revision (JTAGICE3 parameter, from AS6 XML files) */

  /* Bootloader paramater */
  unsigned char autobaud_sync;  /* Sync byte for bootloader autobaud,  must be <= 0x30 */
  int factory_fcpu;             /* Initial F_CPU after reset assuming factory settings */

  OPCODE      * op[AVR_OP_MAX]; /* opcodes */

  LISTID        mem;            /* avr memory definitions */
  LISTID        mem_alias;      /* memory alias definitions */
  const char  * config_file;    /* config file where defined */
  int           lineno;         /* config file line number */
} AVRPART;


typedef unsigned int memtype_t;
typedef struct {
  const char *str;
  memtype_t type;
} memtable_t;

// The least significant 4 bits of type are the offset of a fuse in fuses mem
#define MEM_FUSEOFF_MASK     15 // Mask for offset
#define MEM_FUSE0             0 // fuse lfuse fuse0 wdtcfg
#define MEM_FUSE1             1 // hfuse fuse1 bodcfg
#define MEM_FUSE2             2 // efuse fuse2 osccfg
#define MEM_FUSE4             4 // fuse4 tcd0cfg
#define MEM_FUSE5             5 // fuse5 syscfg0
#define MEM_FUSE6             6 // fuse6 syscfg1
#define MEM_FUSE7             7 // fuse7 append codesize
#define MEM_FUSE8             8 // fuse8 bootend bootsize
#define MEM_FUSEA            10 // fusea pdicfg

// Individual memories that may have different names in different parts
#define MEM_EEPROM      (1<< 4) // eeprom
#define MEM_FLASH       (1<< 5) // flash
#define MEM_APPLICATION (1<< 6) // application
#define MEM_APPTABLE    (1<< 7) // apptable
#define MEM_BOOT        (1<< 8) // boot
#define MEM_FUSES       (1<< 9) // fuses
#define MEM_LOCK        (1<<10) // lock lockbits
#define MEM_SIGROW      (1<<11) // prodsig sigrow
#define MEM_SIGNATURE   (1<<12) // signature
#define MEM_CALIBRATION (1<<13) // calibration
#define MEM_TEMPSENSE   (1<<14) // tempsense
#define MEM_SERNUM      (1<<15) // sernum
#define MEM_OSCCAL16    (1<<16) // osccal16
#define MEM_OSCCAL20    (1<<17) // osccal20
#define MEM_OSC16ERR    (1<<18) // osc16err
#define MEM_OSC20ERR    (1<<19) // osc20err
#define MEM_BOOTROW     (1<<20) // bootrow
#define MEM_USERROW     (1<<21) // userrow usersig
#define MEM_IO          (1<<22) // io
#define MEM_SRAM        (1<<23) // sram
#define MEM_SIB         (1<<24) // sib

// Attributes
#define MEM_IN_FLASH    (1<<27) // flash application apptable boot
#define MEM_IS_A_FUSE   (1<<28) // fuse [elh]fuse fuseN wdtcfg bodcfg osccfg tcd0cfg syscfg0 syscfg1 append codesize bootend bootsize pdicfg
#define MEM_USER_TYPE   (1<<29) // userrow usersig bootrow
#define MEM_IN_SIGROW   (1<<30) // prodsig sigrow signature calibration sernum tempsense osccal16 osccal20 osc16err osc20err
#define MEM_READONLY   (1U<<31) // sib prodsig sigrow signature sernum tempsense calibration osccal16 osccal20 osc16err osc20err

// Marcos to locate a memory type or a fuse
#define avr_locate_eeprom(p) avr_locate_mem_by_type((p), MEM_EEPROM)
#define avr_locate_flash(p) avr_locate_mem_by_type((p), MEM_FLASH)
#define avr_locate_application(p) avr_locate_mem_by_type((p), MEM_APPLICATION)
#define avr_locate_apptable(p) avr_locate_mem_by_type((p), MEM_APPTABLE)
#define avr_locate_boot(p) avr_locate_mem_by_type((p), MEM_BOOT)
#define avr_locate_fuses(p) avr_locate_mem_by_type((p), MEM_FUSES)
#define avr_locate_lock(p) avr_locate_mem_by_type((p), MEM_LOCK)
#define avr_locate_lockbits(p) avr_locate_mem_by_type((p), MEM_LOCK)
#define avr_locate_prodsig(p) avr_locate_mem_by_type((p), MEM_SIGROW)
#define avr_locate_sigrow(p) avr_locate_mem_by_type((p), MEM_SIGROW)
#define avr_locate_signature(p) avr_locate_mem_by_type((p), MEM_SIGNATURE)
#define avr_locate_calibration(p) avr_locate_mem_by_type((p), MEM_CALIBRATION)
#define avr_locate_tempsense(p) avr_locate_mem_by_type((p), MEM_TEMPSENSE)
#define avr_locate_sernum(p) avr_locate_mem_by_type((p), MEM_SERNUM)
#define avr_locate_osccal16(p) avr_locate_mem_by_type((p), MEM_OSCCAL16)
#define avr_locate_osccal20(p) avr_locate_mem_by_type((p), MEM_OSCCAL20)
#define avr_locate_osc16err(p) avr_locate_mem_by_type((p), MEM_OSC16ERR)
#define avr_locate_osc20err(p) avr_locate_mem_by_type((p), MEM_OSC20ERR)
#define avr_locate_bootrow(p) avr_locate_mem_by_type((p), MEM_BOOTROW)
#define avr_locate_usersig(p) avr_locate_mem_by_type((p), MEM_USERROW)
#define avr_locate_userrow(p) avr_locate_mem_by_type((p), MEM_USERROW)
#define avr_locate_io(p) avr_locate_mem_by_type((p), MEM_IO)
#define avr_locate_sram(p) avr_locate_mem_by_type((p), MEM_SRAM)
#define avr_locate_sib(p) avr_locate_mem_by_type((p), MEM_SIB)

#define avr_locate_fuse(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE0)
#define avr_locate_lfuse(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE0)
#define avr_locate_hfuse(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE1)
#define avr_locate_efuse(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE2)
#define avr_locate_fuse0(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE0)
#define avr_locate_wdtcfg(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE0)
#define avr_locate_fuse1(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE1)
#define avr_locate_bodcfg(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE1)
#define avr_locate_fuse2(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE2)
#define avr_locate_osccfg(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE2)
#define avr_locate_fuse4(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE4)
#define avr_locate_tcd0cfg(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE4)
#define avr_locate_fuse5(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE5)
#define avr_locate_syscfg0(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE5)
#define avr_locate_fuse6(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE6)
#define avr_locate_syscfg1(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE6)
#define avr_locate_fuse7(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE7)
#define avr_locate_append(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE7)
#define avr_locate_codesize(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE7)
#define avr_locate_fuse8(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE8)
#define avr_locate_bootend(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE8)
#define avr_locate_bootsize(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSE8)
#define avr_locate_fusea(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSEA)
#define avr_locate_pdicfg(p) avr_locate_mem_by_type((p), MEM_IS_A_FUSE | MEM_FUSEA)

// Fuse offset and memory type/attribute macros
#define mem_is_eeprom(mem) (!!((mem)->type & MEM_EEPROM))
#define mem_is_flash(mem) (!!((mem)->type & MEM_FLASH))
#define mem_is_application(mem) (!!((mem)->type & MEM_APPLICATION))
#define mem_is_apptable(mem) (!!((mem)->type & MEM_APPTABLE))
#define mem_is_boot(mem) (!!((mem)->type & MEM_BOOT))
#define mem_is_fuses(mem) (!!((mem)->type & MEM_FUSES))
#define mem_is_lock(mem) (!!((mem)->type & MEM_LOCK))
#define mem_is_sigrow(mem) (!!((mem)->type & MEM_SIGROW))
#define mem_is_signature(mem) (!!((mem)->type & MEM_SIGNATURE))
#define mem_is_calibration(mem) (!!((mem)->type & MEM_CALIBRATION))
#define mem_is_tempsense(mem) (!!((mem)->type & MEM_TEMPSENSE))
#define mem_is_sernum(mem) (!!((mem)->type & MEM_SERNUM))
#define mem_is_osccal16(mem) (!!((mem)->type & MEM_OSCCAL16))
#define mem_is_osccal20(mem) (!!((mem)->type & MEM_OSCCAL20))
#define mem_is_osc16err(mem) (!!((mem)->type & MEM_OSC16ERR))
#define mem_is_osc20err(mem) (!!((mem)->type & MEM_OSC20ERR))
#define mem_is_bootrow(mem) (!!((mem)->type & MEM_BOOTROW))
#define mem_is_userrow(mem) (!!((mem)->type & MEM_USERROW))
#define mem_is_io(mem) (!!((mem)->type & MEM_IO))
#define mem_is_sram(mem) (!!((mem)->type & MEM_SRAM))
#define mem_is_sib(mem) (!!((mem)->type & MEM_SIB))

#define mem_is_in_flash(mem) (!!((mem)->type & MEM_IN_FLASH))
#define mem_is_a_fuse(mem) (!!((mem)->type & MEM_IS_A_FUSE))
#define mem_is_in_fuses(mem) (!!((mem)->type & (MEM_FUSES | MEM_IS_A_FUSE))) // If fuses exists, that is
#define mem_is_user_type(mem) (!!((mem)->type & MEM_USER_TYPE))
#define mem_is_in_sigrow(mem) (!!((mem)->type & MEM_IN_SIGROW)) // If sigrow exists, that is
#define mem_is_readonly(mem) (!!((mem)->type & MEM_READONLY))
#define mem_is_paged_type(mem) (!!((mem)->type & (MEM_IN_FLASH | MEM_EEPROM | MEM_USER_TYPE)))
#define mem_is_lfuse(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE0))
#define mem_is_hfuse(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE1))
#define mem_is_efuse(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE2))
#define mem_is_fuse0(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE0))
#define mem_is_fuse1(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE1))
#define mem_is_fuse2(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE2))
#define mem_is_fuse4(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE4))
#define mem_is_fuse5(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE5))
#define mem_is_fuse6(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE6))
#define mem_is_fuse7(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE7))
#define mem_is_fuse8(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSE8))
#define mem_is_fusea(m) (((m)->type&(MEM_IS_A_FUSE|MEM_FUSEOFF_MASK)) == (MEM_IS_A_FUSE|MEM_FUSEA))

#define mem_fuse_offset(mem) ((mem)->type & MEM_FUSEOFF_MASK) // Valid if mem_is_a_fuse(mem)

typedef struct avrmem {
  const char *desc;           /* memory description ("flash", "eeprom", etc) */
  memtype_t type;             /* internally used type, cannot be set in conf files */
  LISTID comments;            // Used by developer options -p*/[ASsr...]
  int paged;                  /* 16-bit page addressed, e.g., ATmega flash but not EEPROM */
  int size;                   /* total memory size in bytes */
  int page_size;              /* size of memory page (if page addressed) */
  int num_pages;              /* number of pages (if page addressed) */
  int initval;                /* factory setting of fuses and lock bits */
  int bitmask;                /* bits used in fuses and lock bits */
  int n_word_writes;          /* TPI only: number words to write at a time */
  unsigned int offset;        /* offset in IO memory (ATxmega) */
  int min_write_delay;        /* microseconds */
  int max_write_delay;        /* microseconds */
  int pwroff_after_write;     /* after this memory is written to,
                                 the device must be powered off and
                                 back on, see errata
                                 https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/doc1042.pdf */
  unsigned char readback[2];  /* polled read-back values */

  int mode;                   /* stk500 v2 xml file parameter */
  int delay;                  /* stk500 v2 xml file parameter */
  int blocksize;              /* stk500 v2 xml file parameter */
  int readsize;               /* stk500 v2 xml file parameter */
  int pollindex;              /* stk500 v2 xml file parameter */

  unsigned char * buf;        /* pointer to memory buffer */
  unsigned char * tags;       /* allocation tags */
  OPCODE * op[AVR_OP_MAX];    /* opcodes */
} AVRMEM;

typedef struct avrmem_alias {
  const char *desc;           /* alias name ("syscfg0" etc.) */
  AVRMEM *aliased_mem;
} AVRMEM_ALIAS;

#ifdef __cplusplus
extern "C" {
#endif


int intlog2(unsigned int n);

/* Functions for OPCODE structures */
OPCODE * avr_new_opcode(void);
void     avr_free_opcode(OPCODE * op);
int avr_set_bits(const OPCODE *op, unsigned char *cmd);
int avr_set_addr(const OPCODE *op, unsigned char *cmd, unsigned long addr);
int avr_set_addr_mem(const AVRMEM *mem, int opnum, unsigned char *cmd, unsigned long addr);
int avr_set_input(const OPCODE *op, unsigned char *cmd, unsigned char data);
int avr_get_output(const OPCODE *op, const unsigned char *res, unsigned char *data);
int avr_get_output_index(const OPCODE *op);
char cmdbitchar(CMDBIT cb);
char *cmdbitstr(CMDBIT cb);
const char *opcodename(int opnum);
char *opcode2str(const OPCODE *op, int opnum, int detailed);

/* Functions for AVRMEM structures */
AVRMEM * avr_new_mem(void);
AVRMEM_ALIAS * avr_new_memalias(void);
int avr_initmem(const AVRPART *p);
AVRMEM * avr_dup_mem(const AVRMEM *m);
void     avr_free_mem(AVRMEM * m);
void     avr_free_memalias(AVRMEM_ALIAS * m);
AVRMEM * avr_locate_mem(const AVRPART *p, const char *desc);
AVRMEM * avr_locate_mem_noalias(const AVRPART *p, const char *desc);
AVRMEM * avr_locate_fuse_by_offset(const AVRPART *p, unsigned int off);
AVRMEM * avr_locate_mem_by_type(const AVRPART *p, memtype_t type);
unsigned int avr_data_offset(const AVRPART *p);
AVRMEM_ALIAS * avr_locate_memalias(const AVRPART *p, const char *desc);
AVRMEM_ALIAS * avr_find_memalias(const AVRPART *p, const AVRMEM *m_orig);
void avr_mem_display(FILE *f, const AVRPART *p, const char *prefix);

/* Functions for AVRPART structures */
AVRPART * avr_new_part(void);
AVRPART * avr_dup_part(const AVRPART *d);
void      avr_free_part(AVRPART * d);
AVRPART * locate_part(const LISTID parts, const char *partdesc);
AVRPART * locate_part_by_avr910_devcode(const LISTID parts, int devcode);
AVRPART * locate_part_by_signature(const LISTID parts, unsigned char *sig, int sigsize);
AVRPART * locate_part_by_signature_pm(const LISTID parts, unsigned char *sig, int sigsize, int prog_modes);
const char *avr_prog_modes_str(int pm);
void avr_display(FILE *f, const AVRPART *p, const char *prefix, int verbose);
int avr_variants_display(FILE *f, const AVRPART *p, const char *prefix);

typedef void (*walk_avrparts_cb)(const char *name, const char *desc,
                                 const char *cfgname, int cfglineno,
                                 void *cookie);
void walk_avrparts(LISTID avrparts, walk_avrparts_cb cb, void *cookie);
void sort_avrparts(LISTID avrparts);

// cmp can be, eg, str_caseeq or str_casematch
int part_eq(AVRPART *p, const char *string, int (*cmp)(const char *, const char *));

int compare_memory_masked(AVRMEM * m, uint8_t buf1, uint8_t buf2);

#ifdef __cplusplus
}
#endif

/* formerly pindefs.h */

enum {
  PPI_AVR_VCC = 1,
  PPI_AVR_BUFF,
  PIN_AVR_RESET,
  PIN_AVR_SCK,
  PIN_AVR_SDO,
  PIN_AVR_SDI,
  PIN_JTAG_TCK,
  PIN_JTAG_TDI,
  PIN_JTAG_TDO,
  PIN_JTAG_TMS,
  PIN_LED_ERR,
  PIN_LED_RDY,
  PIN_LED_PGM,
  PIN_LED_VFY,
  N_PINS
};

#define PIN_MASK    (UINT_MAX>>1)
#define PIN_INVERSE (~(PIN_MASK))	/* flag for inverted pin in serbb */
#define PIN_MIN     0   /* smallest allowed pin number */
#define PIN_MAX     31  /* largest allowed pin number */

#ifdef HAVE_LINUXGPIO
/* Embedded systems might have a lot more gpio than only 0-31 */
#undef PIN_MAX
#define PIN_MAX     1000 /* largest allowed pin number */
#endif

/** Number of pins in each element of the bitfield */
#define PIN_FIELD_ELEMENT_SIZE (sizeof(pinmask_t) * 8)
/** Numer of elements to store the complete bitfield of all pins */
#define PIN_FIELD_SIZE ((PIN_MAX+1 + PIN_FIELD_ELEMENT_SIZE-1)/PIN_FIELD_ELEMENT_SIZE)

/**
 * This sets the corresponding bits to 1 or 0, the inverse mask is used to invert the value in necessary.
 * It uses only the lowest element (index=0) of the bitfield, which should be enough for most
 * programmers.
 *
 * @param[in] x       input value
 * @param[in] pgm     the programmer whose pin definitions to use
 * @param[in] pinname the logical name of the pin (PIN_AVR_*, ...)
 * @param[in] level   the logical level (level != 0 => 1, level == 0 => 0),
 *                      if the pin is defined as inverted the resulting bit is also inverted
 * @returns           the input value with the relevant bits modified
 */
#define SET_BITS_0(x,pgm,pinname,level) (((x) & ~(pgm)->pin[pinname].mask[0]) \
    | (\
        (pgm)->pin[pinname].mask[0] & ( \
             (level) \
             ?~((pgm)->pin[pinname].inverse[0]) \
             : ((pgm)->pin[pinname].inverse[0]) \
        ) \
    ) \
)

/**
 * Check if the corresponding bit is set (returns != 0) or cleared.
 * The inverse mask is used, to invert the relevant bits.
 * If the pin definition contains multiple pins, then a single set pin leads to return value != 0.
 * Then you have to check the relevant bits of the returned value, if you need more information.
 * It uses only the lowest element (index=0) of the bitfield, which should be enough for most
 * programmers.
 *
 * @param[in] x       input value
 * @param[in] pgm     the programmer whose pin definitions to use
 * @param[in] pinname the logical name of the pin (PIN_AVR_*, ...)
 * @returns           the input value with only the relevant bits (which are already inverted,
 *                      so you get always the logical level)
 */
#define GET_BITS_0(x,pgm,pinname)       (((x) ^ (pgm)->pin[pinname].inverse[0]) & (pgm)->pin[pinname].mask[0])

/**
 * Data structure to hold used pins by logical function (PIN_AVR_*, ...)
 */
struct pindef_t {
  pinmask_t mask[PIN_FIELD_SIZE]; ///< bitfield of used pins
  pinmask_t inverse[PIN_FIELD_SIZE]; ///< bitfield of inverse/normal usage of used pins
};

/**
 * Data structure to define a checklist of valid pins for each function.
 */
struct pin_checklist_t {
  int pinname; ///< logical pinname eg. PIN_AVR_SCK
  int mandatory; ///< is this a mandatory pin
  const struct pindef_t* valid_pins; ///< mask defines allowed pins, inverse define is they might be used inverted
};

/**
 * Adds a pin in the pin definition as normal or inverse pin.
 *
 * @param[out] pindef pin definition to update
 * @param[in] pin number of pin [0..PIN_MAX]
 * @param[in] inverse inverse (true) or normal (false) pin
 */
void pin_set_value(struct pindef_t * const pindef, const int pin, const bool inverse);

/**
 * Clear all defined pins in pindef.
 *
 * @param[out] pindef pin definition to clear
 */
void pin_clear_all(struct pindef_t * const pindef);

struct programmer_t; /* forward declaration */

/**
 * Convert for given programmer new pin definitions to old pin definitions.
 *
 * @param[inout] pgm programmer whose pins shall be converted.
 */
int pgm_fill_old_pins(struct programmer_t * const pgm);

/**
 * This function checks all pin of pgm against the constraints given in the checklist.
 * It checks if 
 * @li any invalid pins are used
 * @li valid pins are used inverted when not allowed
 * @li any pins are used by more than one function
 * @li any mandatory pin is not set all.
 *
 * In case of any error it report the wrong function and the pin numbers.
 * For verbose >= 2 it also reports the possible correct values.
 * For verbose >=3 it shows also which pins were ok.
 *
 * @param[in] pgm the programmer to check
 * @param[in] checklist the constraint for the pins
 * @param[in] size the number of entries in checklist
 * @param[in] output false suppresses error messages to the user
 * @returns 0 if all pin definitions are valid, -1 otherwise
 */
int pins_check(const struct programmer_t * const pgm, const struct pin_checklist_t * const checklist, const int size, const bool output);

/**
 * Returns the name of the pin as string.
 * 
 * @param pinname the pinname which we want as string.
 * @returns a string with the pinname, or <unknown> if pinname is invalid.
 */
const char * avr_pin_name(int pinname);

/**
 * Returns the name of the pin as lowercase string.
 *
 * @param pinname the pinname which we want as string.
 * @returns a lowercase string with the pinname, or <unknown> if pinname is invalid.
 */
const char * avr_pin_lcname(int pinname);

/**
 * This function returns a string of defined pins, eg, ~1,2,~4,~5,7 or " (not used)"
 * Another execution of this function will overwrite the previous result in the static buffer.
 *
 * @param[in] pindef the pin definition for which we want the string representation
 * @returns pointer to a static string.
 */
const char * pins_to_str(const struct pindef_t * const pindef);

/**
 * This function returns a string of defined pins, eg, ~1, 2, ~4, ~5, 7 or ""
 *
 * @param[in] pindef the pin definition for which we want the string representation
 * @returns a pointer to a string, which was created by strdup
 */
char *pins_to_strdup(const struct pindef_t * const pindef);

/**
 * This function returns a string representation of pins in the mask, eg, 1,3,5-7,9,12
 * Another execution of this function will overwrite the previous result in the static buffer.
 * Consecutive pin number are represented as start-end.
 *
 * @param[in] pinmask the pin mask for which we want the string representation
 * @returns pointer to a static string.
 */
const char * pinmask_to_str(const pinmask_t * const pinmask);

/* formerly serial.h */

/* This is the API for the generic serial interface. The implementations are
   actually provided by the target dependant files:

   ser_posix.c : posix serial interface.
   ser_win32.c : native win32 serial interface.

   The target file will be selected at configure time. */

extern long serial_recv_timeout;  /* ms */
extern long serial_drain_timeout; /* ms */

union filedescriptor
{
  int ifd;
  void *pfd;
  struct
  {
    void *handle;
    int rep;                    /* bulk read endpoint */
    int wep;                    /* bulk write endpoint */
    int eep;                    /* event read endpoint */
    int max_xfer;               /* max transfer size */
    int use_interrupt_xfer;     /* device uses interrupt transfers */
  } usb;
};

#define SERIAL_CS5          0x0000
#define SERIAL_CS6          0x0001
#define SERIAL_CS7          0x0002
#define SERIAL_CS8          0x0004

#define SERIAL_NO_CSTOPB    0x0000
#define SERIAL_CSTOPB       0x0008

#define SERIAL_NO_CREAD     0x0000
#define SERIAL_CREAD        0x0010

#define SERIAL_NO_PARITY    0x0000
#define SERIAL_PARENB       0x0020
#define SERIAL_PARODD       0x0040

#define SERIAL_NO_CLOCAL    0x0000
#define SERIAL_CLOCAL       0x0080

#define SERIAL_8N1 (SERIAL_CS8 | SERIAL_NO_CSTOPB | SERIAL_CREAD | SERIAL_NO_PARITY | SERIAL_CLOCAL)
#define SERIAL_8E1 (SERIAL_CS8 | SERIAL_NO_CSTOPB | SERIAL_CREAD | SERIAL_PARENB    | SERIAL_CLOCAL)
#define SERIAL_8E2 (SERIAL_CS8 | SERIAL_CSTOPB    | SERIAL_CREAD | SERIAL_PARENB    | SERIAL_CLOCAL)

union pinfo
{
  struct {
    long baud;
    unsigned long cflags;
  } serialinfo;
  struct
  {
    unsigned short vid;
    unsigned short pid;
    unsigned short flags;
#define PINFO_FL_USEHID         0x0001
#define PINFO_FL_SILENT         0x0002  /* don't complain if not found */
  } usbinfo;
};


struct serial_device {
  // open should return -1 on error, other values on success
  int (*open)(const char *port, union pinfo pinfo, union filedescriptor *fd);
  int (*setparams)(const union filedescriptor *fd, long baud, unsigned long cflags);
  void (*close)(union filedescriptor *fd);
  void (*rawclose)(union filedescriptor *fd); // Don't restore terminal attributes (Linux)

  int (*send)(const union filedescriptor *fd, const unsigned char * buf, size_t buflen);
  int (*recv)(const union filedescriptor *fd, unsigned char * buf, size_t buflen);
  int (*drain)(const union filedescriptor *fd, int display);

  int (*set_dtr_rts)(const union filedescriptor *fd, int is_on);

  const char *usbsn;
  int flags;
#define SERDEV_FL_NONE         0x0000 /* no flags */
#define SERDEV_FL_CANSETSPEED  0x0001 /* device can change speed */
};

extern struct serial_device *serdev;
extern struct serial_device serial_serdev;
extern struct serial_device usb_serdev;
extern struct serial_device usb_serdev_frame;
extern struct serial_device avrdoper_serdev;
extern struct serial_device usbhid_serdev;

#define serial_open (serdev->open)
#define serial_setparams (serdev->setparams)
#define serial_close (serdev->close)
#define serial_rawclose (serdev->rawclose)
#define serial_send (serdev->send)
#define serial_recv (serdev->recv)
#define serial_drain (serdev->drain)
#define serial_set_dtr_rts (serdev->set_dtr_rts)

// See avrcache.c
typedef struct {                // Memory cache for a subset of cached pages
  int size, page_size;          // Size of cache (flash or eeprom size) and page size
  unsigned int offset;          // Offset of flash/eeprom memory
  unsigned char *cont, *copy;   // current memory contens and device copy of it
  unsigned char *iscached;      // iscached[i] set when page i has been loaded
} AVR_Cache;

/* formerly pgm.h */

#define OFF                  0 // Many contexts: reset, power, LEDs, ...
#define ON                   1 // Many contexts

#define PGM_PORTLEN PATH_MAX
#define PGM_TYPELEN 32

typedef enum {
  EXIT_VCC_UNSPEC,
  EXIT_VCC_ENABLED,
  EXIT_VCC_DISABLED
} exit_vcc_t;

typedef enum {
  EXIT_RESET_UNSPEC,
  EXIT_RESET_ENABLED,
  EXIT_RESET_DISABLED
} exit_reset_t;

typedef enum {
  EXIT_DATAHIGH_UNSPEC,
  EXIT_DATAHIGH_ENABLED,
  EXIT_DATAHIGH_DISABLED
} exit_datahigh_t;

typedef enum {
  CONNTYPE_PARALLEL,
  CONNTYPE_SERIAL,
  CONNTYPE_USB,
  CONNTYPE_SPI,
  CONNTYPE_LINUXGPIO
} conntype_t;


#define LED_N                 4 // Max number of LEDs driven by programmers
#define LED_RDY               0 // led_set(pgm, LED_RDY) or led_clr(pgm, LED_RDY)
#define LED_ERR               1 // led_set(pgm, LED_ERR) or led_clr(pgm, LED_ERR)
#define LED_PGM               2 // led_set(pgm, LED_PGM) or led_clr(pgm, LED_PGM)
#define LED_VFY               3 // led_set(pgm, LED_VFY) or led_clr(pgm, LED_VFY)
#define LED_BEG            (-1) // led_set(pgm, LED_BEG) initally clear all LEDs
#define LED_END            (-2) // led_set(pgm, LED_END) set error codes at extit
#define LED_NOP            (-3) // led_set(pgm, LED_NOP) periodic nop for blinking

#ifndef LED_FMAX
#define LED_FMAX           2.51 // Hz (max frequency at which LEDs change)
#endif

typedef struct {
  int now, chg, phy, end, set;  // LED states (current, change needed next period, physical, at end, ever set)
  unsigned long ms[LED_N];      // Time in ms after last physical change
} leds_t;

/*
 * Any changes in PROGRAMMER, please also ensure changes are made in
 *  - lexer.l
 *  - Either Component_t avr_comp[] of config.c or config_gram.y
 *  - dev_pgm_strct() in developer_opts.c
 *  - pgm_new() in pgm.c for initialisation; note that all const char * must
 *    be initialised with ""
 */
typedef struct programmer_t {
  LISTID id;
  const char *desc;
  void (*initpgm)(struct programmer_t *pgm); // Sets up the AVRDUDE programmer
  LISTID comments;              // Used by developer options -c*/[ASsr...]
  const char *parent_id;        // Used by developer options
  int prog_modes;               // Programming interfaces, see #define PM_...
  int is_serialadapter;         // Programmer is also a serialadapter
  int extra_features;
  struct pindef_t pin[N_PINS];
  conntype_t conntype;
  int baudrate;
  int usbvid;
  LISTID usbpid;
  const char *usbdev;
  const char *usbsn;
  const char *usbvendor;
  const char *usbproduct;
  LISTID hvupdi_support;        // List of UPDI HV variants the tool supports, see HV_UPDI_VARIANT_x

  // Values below are not set by config_gram.y; ensure fd is first for dev_pgm_raw()
  union filedescriptor fd;
  char type[PGM_TYPELEN];
  char port[PGM_PORTLEN];
  unsigned int pinno[N_PINS];   // TODO to be removed if old pin data no longer needed
  exit_vcc_t exit_vcc;          // Should these be set in avrdude.conf?
  exit_reset_t exit_reset;
  exit_datahigh_t exit_datahigh;
  int ppidata;
  int ppictrl;
  int ispdelay;                 // ISP clock delay
  int page_size;                // Page size if the programmer supports paged write/load
  double bitclock;              // JTAG ICE clock period in microseconds
  leds_t *leds;                 // State of LEDs as tracked by led_...()  functions in leds.c

  int  (*rdy_led)        (const struct programmer_t *pgm, int value);
  int  (*err_led)        (const struct programmer_t *pgm, int value);
  int  (*pgm_led)        (const struct programmer_t *pgm, int value);
  int  (*vfy_led)        (const struct programmer_t *pgm, int value);
  int  (*initialize)     (const struct programmer_t *pgm, const AVRPART *p); // Sets up the physical programmer
  void (*display)        (const struct programmer_t *pgm, const char *p);
  void (*enable)         (struct programmer_t *pgm, const AVRPART *p);
  void (*disable)        (const struct programmer_t *pgm);
  void (*powerup)        (const struct programmer_t *pgm);
  void (*powerdown)      (const struct programmer_t *pgm);
  int  (*program_enable) (const struct programmer_t *pgm, const AVRPART *p);
  int  (*chip_erase)     (const struct programmer_t *pgm, const AVRPART *p);
  int  (*unlock)         (const struct programmer_t *pgm, const AVRPART *p);
  int  (*cmd)            (const struct programmer_t *pgm, const unsigned char *cmd,
                          unsigned char *res);
  int  (*cmd_tpi)        (const struct programmer_t *pgm, const unsigned char *cmd,
                          int cmd_len, unsigned char res[], int res_len);
  int  (*spi)            (const struct programmer_t *pgm, const unsigned char *cmd,
                          unsigned char *res, int count);
  int  (*open)           (struct programmer_t *pgm, const char *port);
  void (*close)          (struct programmer_t *pgm);
  int  (*paged_write)    (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned int page_size, unsigned int baseaddr,
                          unsigned int n_bytes);
  int  (*paged_load)     (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned int page_size, unsigned int baseaddr,
                          unsigned int n_bytes);
  int  (*page_erase)     (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned int baseaddr);
  void (*write_setup)    (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m);
  int  (*write_byte)     (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned long addr, unsigned char value);
  int  (*read_byte)      (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned long addr, unsigned char *value);
  int  (*read_sig_bytes) (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m);
  int  (*read_sib)       (const struct programmer_t *pgm, const AVRPART *p, char *sib);
  int  (*read_chip_rev)  (const struct programmer_t *pgm, const AVRPART *p, unsigned char *chip_rev);
  int  (*term_keep_alive)(const struct programmer_t *pgm, const AVRPART *p);
  int  (*end_programming)(const struct programmer_t *pgm, const AVRPART *p);

  void (*print_parms)    (const struct programmer_t *pgm, FILE *fp);
  int  (*set_vtarget)    (const struct programmer_t *pgm, double v);
  int  (*get_vtarget)    (const struct programmer_t *pgm, double *v);
  int  (*set_varef)      (const struct programmer_t *pgm, unsigned int chan, double v);
  int  (*get_varef)      (const struct programmer_t *pgm, unsigned int chan, double *v);
  int  (*set_fosc)       (const struct programmer_t *pgm, double v);
  int  (*get_fosc)       (const struct programmer_t *pgm, double *v);
  int  (*set_sck_period) (const struct programmer_t *pgm, double v);
  int  (*get_sck_period) (const struct programmer_t *pgm, double *v);
  int  (*setpin)         (const struct programmer_t *pgm, int pinfunc, int value);
  int  (*getpin)         (const struct programmer_t *pgm, int pinfunc);
  int  (*highpulsepin)   (const struct programmer_t *pgm, int pinfunc);
  int  (*parseexitspecs) (struct programmer_t *pgm, const char *s);
  int  (*perform_osccal) (const struct programmer_t *pgm);
  int  (*parseextparams) (const struct programmer_t *pgm, const LISTID xparams);
  void (*setup)          (struct programmer_t *pgm);
  void (*teardown)       (struct programmer_t *pgm);
  int  (*flash_readhook) (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *flm, const char *fname, int size);
  // Cached r/w API for terminal reads/writes
  int (*write_byte_cached)(const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned long addr, unsigned char value);
  int (*read_byte_cached)(const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned long addr, unsigned char *value);
  int (*chip_erase_cached)(const struct programmer_t *pgm, const AVRPART *p);
  int (*page_erase_cached)(const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned int baseaddr);
  int (*readonly)        (const struct programmer_t *pgm, const AVRPART *p, const AVRMEM *m,
                          unsigned int addr);
  int (*flush_cache)     (const struct programmer_t *pgm, const AVRPART *p);
  int (*reset_cache)     (const struct programmer_t *pgm, const AVRPART *p);
  AVR_Cache *cp_flash, *cp_eeprom, *cp_bootrow, *cp_usersig;

  const char *config_file;      // Config file where defined
  int  lineno;                  // Config file line number
  void *cookie;                 // For private use by the programmer
  char flag;                    // For use by pgm->initpgm()
} PROGRAMMER;

typedef PROGRAMMER SERIALADAPTER; // Only a subset is needed for serial adapters
int is_programmer(const PROGRAMMER *p);
int is_serialadapter(const SERIALADAPTER *p);
void list_serialadapters(FILE *fp, const char *prefix, LISTID programmers);
void serialadapter_not_found(const char *sea_id);

#define NO_PIN   (PIN_MAX + 1U) // Magic pinno[] value for unused pins

#ifdef __cplusplus
extern "C" {
#endif

PROGRAMMER * pgm_new(void);
PROGRAMMER * pgm_dup(const PROGRAMMER *src);
void         pgm_free(PROGRAMMER *p);

void programmer_display(PROGRAMMER * pgm, const char * p);

/* show is a mask like this (1<<PIN_AVR_SCK)|(1<<PIN_AVR_SDO)| ... */
#define SHOW_ALL_PINS (~0u)
#define SHOW_PPI_PINS ((1<<PPI_AVR_VCC)|(1<<PPI_AVR_BUFF))
#define SHOW_AVR_PINS ((1<<PIN_AVR_RESET)|(1<<PIN_AVR_SCK)|(1<<PIN_AVR_SDO)|(1<<PIN_AVR_SDI))
#define SHOW_JTAG_PINS ((1<<PIN_JTAG_TCK)|(1<<PIN_JTAG_TDI)|(1<<PIN_JTAG_TDO)|(1<<PIN_JTAG_TMS))
#define SHOW_LED_PINS ((1<<PIN_LED_ERR)|(1<<PIN_LED_RDY)|(1<<PIN_LED_PGM)|(1<<PIN_LED_VFY))
void pgm_display_generic_mask(const PROGRAMMER *pgm, const char *p, unsigned int show);
void pgm_display_generic(const PROGRAMMER *pgm, const char *p);

PROGRAMMER *locate_programmer_set(const LISTID programmers, const char *id, const char **setid);
PROGRAMMER *locate_programmer_starts_set(const LISTID programmers, const char *id, const char **setid, AVRPART *prt);
PROGRAMMER *locate_programmer(const LISTID programmers, const char *configid);

typedef void (*walk_programmers_cb)(const char *name, const char *desc,
                                    const char *cfgname, int cfglineno,
                                    void *cookie);
void walk_programmers(LISTID programmers, walk_programmers_cb cb, void *cookie);

void sort_programmers(LISTID programmers);

#ifdef __cplusplus
}
#endif

/* formerly avr.h */

typedef void (*FP_UpdateProgress)(int percent, double etime, const char *hdr, int finish);

extern struct avrpart parts[];
extern memtable_t avr_mem_order[100];

extern FP_UpdateProgress update_progress;

#ifdef __cplusplus
extern "C" {
#endif

int avr_tpi_poll_nvmbsy(const PROGRAMMER *pgm);
int avr_tpi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
int avr_tpi_program_enable(const PROGRAMMER *pgm, const AVRPART *p, unsigned char guard_time);
int avr_read_byte_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
			  unsigned long addr, unsigned char * value);

int avr_read_mem(const PROGRAMMER * pgm, const AVRPART *p, const AVRMEM *mem, const AVRPART *v);

int avr_read(const PROGRAMMER * pgm, const AVRPART *p, const char *memstr, const AVRPART *v);

int avr_write_page(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                   unsigned long addr);

unsigned long avr_ustimestamp();

unsigned long avr_mstimestamp();

double avr_timestamp();

int avr_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                   unsigned long addr, unsigned char data);

int avr_read_byte_silent(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char *datap);

int avr_bitmask_data(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char data);

int avr_write_byte_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
			   unsigned long addr, unsigned char data);

int avr_write_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int size, int auto_erase);

int avr_write(const PROGRAMMER *pgm, const AVRPART *p, const char *memstr, int size, int auto_erase);

int avr_signature(const PROGRAMMER *pgm, const AVRPART *p);

int avr_mem_bitmask(const AVRPART *p, const AVRMEM *mem, int addr);

int avr_verify(const PROGRAMMER *pgm, const AVRPART *p, const AVRPART *v, const char *m, int size);

int avr_get_cycle_count(const PROGRAMMER *pgm, const AVRPART *p, int *cycles);

int avr_put_cycle_count(const PROGRAMMER *pgm, const AVRPART *p, int cycles);

char *avr_prog_modes(int pm);

int avr_get_mem_type(const char *str);

int avr_mem_is_flash_type(const AVRMEM *mem);

int avr_mem_is_eeprom_type(const AVRMEM *mem);

int avr_mem_is_usersig_type(const AVRMEM *mem);

int avr_mem_cmp(void *mem1, void *mem2);

int avr_mem_is_known(const char *str);

int avr_mem_might_be_known(const char *str);

#define disable_trailing_ff_removal() avr_mem_hiaddr(NULL)
int avr_mem_hiaddr(const AVRMEM * mem);

int avr_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);

int avr_unlock(const PROGRAMMER *pgm, const AVRPART *p);

void report_progress(int completed, int total, const char *hdr);

void trace_buffer(const char *funstr, const unsigned char *buf, size_t buflen);

int avr_has_paged_access(const PROGRAMMER *pgm, const AVRMEM *m);

int avr_read_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *buf);

int avr_write_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *data);

int avr_is_and(const unsigned char *s1, const unsigned char *s2, const unsigned char *s3, size_t n);

// Bytewise cached read/write API
int avr_read_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned long addr, unsigned char *value);
int avr_write_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned long addr, unsigned char data);
int avr_chip_erase_cached(const PROGRAMMER *pgm, const AVRPART *p);
int avr_page_erase_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned int baseaddr);
int avr_flush_cache(const PROGRAMMER *pgm, const AVRPART *p);
int avr_reset_cache(const PROGRAMMER *pgm, const AVRPART *p);

#ifdef __cplusplus
}
#endif

/* formerly fileio.h */

typedef enum {
  FMT_ERROR = -1,
  FMT_AUTO,
  FMT_SREC,
  FMT_IHEX,
  FMT_RBIN,
  FMT_IMM,
  FMT_EEGG,
  FMT_HEX,
  FMT_DEC,
  FMT_OCT,
  FMT_BIN,
  FMT_ELF,
  FMT_IHXC,
} FILEFMT;

struct fioparms {
  int    op;
  char * mode;
  char * iodesc;
  char * dir;
  char * rw;
  unsigned int fileoffset;
};

typedef struct {
  int addr, len;
} Segment_t;

enum {
  FIO_READ,
  FIO_WRITE,
  FIO_READ_FOR_VERIFY,
};

#ifdef __cplusplus
extern "C" {
#endif

FILEFMT fileio_format(char c);

char *fileio_fmtstr(FILEFMT format);

int fileio_fmtchr(FILEFMT format);

FILE *fileio_fopenr(const char *fname);

int fileio_fmt_autodetect_fp(FILE *f);

int fileio_fmt_autodetect(const char *fname);

int fileio(int oprwv, const char *filename, FILEFMT format,
  const AVRPART *p, const char *memstr, int size);

int segment_normalise(const AVRMEM *mem, Segment_t *segp);

int fileio_segments(int oprwv, const char *filename, FILEFMT format,
  const AVRPART *p, const AVRMEM *mem, int n, const Segment_t *seglist);

#ifdef __cplusplus
}
#endif


/* formerly update.h */

enum {
  DEVICE_READ,
  DEVICE_WRITE,
  DEVICE_VERIFY
};

enum updateflags {
  UF_NONE = 0,
  UF_NOWRITE = 1,
  UF_AUTO_ERASE = 2,
  UF_VERIFY = 4,
};


typedef struct update_t {
  const char *cmdline;          // -T line is stored here and takes precedence if it exists
  char *memstr;                 // Memory name for -U
  int   op;                     // Symbolic memory operation DEVICE_... for -U
  char *filename;               // Filename for -U, can be -
  int   format;                 // File format FMT_...
} UPDATE;

typedef struct {                // File reads for flash can exclude trailing 0xff, which are cut off
  int nbytes,                   // Number of bytes set including 0xff but excluding cut off, trailing 0xff
      nsections,                // Number of consecutive sections in source excluding cut off, trailing 0xff
      npages,                   // Number of memory pages needed excluding pages solely with trailing 0xff
      nfill,                    // Number of fill bytes to make up full pages that are needed
      ntrailing,                // Number of trailing 0xff in source
      firstaddr,                // First address set in [0, mem->size-1]
      lastaddr;                 // Highest address set by input file
} Filestats;


#ifdef __cplusplus
extern "C" {
#endif

UPDATE *parse_op(const char *s);
UPDATE *dup_update(const UPDATE *upd);
UPDATE *new_update(int op, const char *memstr, int filefmt, const char *fname);
UPDATE *cmd_update(const char *cmd);
extern void free_update(UPDATE *upd);
char *update_str(const UPDATE *upd);
int do_op(const PROGRAMMER *pgm, const AVRPART *p, const UPDATE *upd,
  enum updateflags flags);
int memstats(const AVRPART *p, const char *memstr, int size, Filestats *fsp);

// Helper functions for dry run to determine file access
int update_is_okfile(const char *fn);
int update_is_writeable(const char *fn);
int update_is_readable(const char *fn);

int update_dryrun(const AVRPART *p, UPDATE *upd);


#ifdef __cplusplus
}
#endif


/* formerly pgm_type.h */

/*LISTID programmer_types;*/

typedef struct programmer_type_t {
  const char * const id;
  void (*initpgm)(struct programmer_t *pgm);
  const char * const desc;
} PROGRAMMER_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

const PROGRAMMER_TYPE *locate_programmer_type(const char *id);

const char *locate_programmer_type_id(void (*initpgm)(struct programmer_t *pgm));

typedef void (*walk_programmer_types_cb)(const char *id, const char *desc,
                                    void *cookie);
void walk_programmer_types(/*LISTID programmer_types,*/ walk_programmer_types_cb cb, void *cookie);

#ifdef __cplusplus
}
#endif

/* formerly config.h */

extern LISTID      part_list;
extern LISTID      programmers;
extern const char *avrdude_conf_version;
extern const char *default_programmer;
extern const char *default_parallel;
extern const char *default_serial;
extern const char *default_spi;
extern int         default_baudrate;
extern double      default_bitclock;
extern char const *default_linuxgpio;
extern int         allow_subshells;

/* This name is fixed, it's only here for symmetry with
 * default_parallel and default_serial. */
#define DEFAULT_USB "usb"

#ifdef __cplusplus
extern "C" {
#endif

void *cfg_malloc(const char *funcname, size_t n);

void *cfg_realloc(const char *funcname, void *p, size_t n);

char *cfg_strdup(const char *funcname, const char *s);

int init_config(void);

void cleanup_config(void);

int read_config(const char * file);

const char *cache_string(const char *file);

unsigned char *cfg_unescapeu(unsigned char *d, const unsigned char *s);

char *cfg_unescape(char *d, const char *s);

char *cfg_escape(const char *s);

#ifdef __cplusplus
}
#endif

// Helper functions for more readable string checking

// Structure for string to data conversions
typedef struct {
  int size, sigsz, type;
  char *errstr, *warnstr, *str_ptr;
  AVRMEM *mem;
  union {
    float f;
    double d;
    int64_t ll;
    uint64_t ull;
    uint8_t a[8];
  };
} Str2data;

// Str2data type bit patterns
#define STR_1                 1 // 1-byte integer
#define STR_2                 2 // 2-byte integer
#define STR_4                 4 // 4-byte integer
#define STR_8                 8 // 8-byte integer
#define STR_UNSIGNED         16 // Unsigned integer, eg, 42U, -42U, 0x2a or 0b101010
#define STR_SIGNED           32 // Signed integer, eg, +42, -42, 42, -0x2a, +0x2a but not 0x2a
#define STR_INTEGER          63 // Any of above
#define STR_DOUBLE           64 // Double precision
#define STR_FLOAT           128 // Floating point
#define STR_REAL            192 // Float or double
#define STR_NUMBER          255 // Any of above
#define STR_STRING          256 // C-type string or character
#define STR_FILE            512 // File name containing data
#define STR_ANY            1023 // Any of above

// Abbreviations for specific types
#define STR_INT8   (STR_1 | STR_SIGNED) // Signed domain [-2^(n-1), 2^(n-1)-1]
#define STR_INT16  (STR_2 | STR_SIGNED)
#define STR_INT32  (STR_4 | STR_SIGNED)
#define STR_INT64  (STR_8 | STR_SIGNED)
#define STR_UINT8  (STR_1 | STR_UNSIGNED) // Unsigned domain [0, 2^n-1]; -u is same as 2^n-u
#define STR_UINT16 (STR_2 | STR_UNSIGNED) // Unsigned number u is deemed out of range
#define STR_UINT32 (STR_4 | STR_UNSIGNED) // if both u and -u are outside unsigned domain
#define STR_UINT64 (STR_8 | STR_UNSIGNED)
#define STR_XINT8   STR_1       // Unspecified signedness: numbers can occupy the asymmetric union
#define STR_XINT16  STR_2       // of signed domain and unsigned domain: [-2^(n-1), 2^n-1]
#define STR_XINT32  STR_4
#define STR_XINT64  STR_8

#ifdef __cplusplus
extern "C" {
#endif

int avr_locate_upidx(const AVRPART *p);
const Configitem_t *avr_locate_configitems(const AVRPART *p, int *ncp);
const char * const *avr_locate_isrtable(const AVRPART *p, int *nip);
const Register_file_t *avr_locate_register_file(const AVRPART *p, int *nrp);
const Register_file_t *avr_locate_register(const Register_file_t *rgf, int nr, const char *reg,
 int (*match)(const char *, const char*));
const Register_file_t **avr_locate_registerlist(const Register_file_t *rgf, int nr, const char *reg,
 int (*match)(const char *, const char*));
const Configitem_t *avr_locate_config(const Configitem_t *cfg, int nc, const char *name,
  int (*match)(const char *, const char*));
const Configitem_t **avr_locate_configlist(const Configitem_t *cfg, int nc, const char *name,
  int (*match)(const char *, const char*));
int avr_get_config_value(const PROGRAMMER *pgm, const AVRPART *p, const char *cname, int *valuep);
int avr_set_config_value(const PROGRAMMER *pgm, const AVRPART *p, const char *cname, int value);

int setport_from_serialadapter(char **portp, const SERIALADAPTER *ser, const char *sernum);
int setport_from_vid_pid(char **portp, int vid, int pid, const char *sernum);
int list_available_serialports(LISTID programmers);
int touch_serialport(char **portp, int baudrate, int nwaits);

int str_starts(const char *str, const char *starts);
int str_eq(const char *str1, const char *str2);
int str_contains(const char *str, const char *substr);
int str_ends(const char *str, const char *ends);
int str_casestarts(const char *str, const char *starts);
int str_caseends(const char *str, const char *ends);
int str_caseeq(const char *str1, const char *str2);
int str_match(const char *pattern, const char *string);
int str_casematch(const char *pattern, const char *string);
int str_matched_by(const char *string, const char *pattern);
int str_casematched_by(const char *string, const char *pattern);
int str_is_pattern(const char *str);
char *str_sprintf(const char *fmt, ...);
char *str_fgets(FILE *fp, const char **errpp);
char *str_lc(char *s);
char *str_uc(char *s);
char *str_lcfirst(char *s);
char *str_ucfirst(char *s);
char *str_utoa(unsigned n, char *buf, int base);
char *str_endnumber(const char *str);
const char *str_plural(int x);
const char *str_inname(const char *fn);
const char *str_outname(const char *fn);
const char *str_interval(int a, int b);
bool is_bigendian();
void change_endian(void *p, int size);
int memall(const void *p, char c, size_t n);
unsigned long long int str_ull(const char *str, char **endptr, int base);
Str2data *str_todata(const char *str, int type, const AVRPART *part, const char *memstr);
void str_freedata(Str2data *sd);
unsigned long long int str_int(const char *str, int type, const char **errpp);
int str_membuf(const char *str, int type, unsigned char *buf, int size, const char **errpp);
char *str_nexttok(char *buf, const char *delim, char **next);
int str_levenshtein(const char *str1, const char *str2, int swap, int subst, int add, int del);
size_t str_weighted_damerau_levenshtein(const char *str1, const char *str2);

int led_set(const PROGRAMMER *pgm, int led);
int led_clr(const PROGRAMMER *pgm, int led);
int led_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
int led_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char value);
int led_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char *value);
int led_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int baseaddr, unsigned int n_bytes);
int led_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int baseaddr, unsigned int n_bytes);
int led_page_erase(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int baseaddr);

int terminal_mode(const PROGRAMMER *pgm, const AVRPART *p);
int terminal_mode_noninteractive(const PROGRAMMER *pgm, const AVRPART *p);
int terminal_line(const PROGRAMMER *pgm, const AVRPART *p, const char *line);
char *terminal_get_input(const char *prompt);
void terminal_setup_update_progress();

#ifdef __cplusplus
}
#endif

/* formerly confwin.h */

#if defined(WIN32)

#ifdef __cplusplus
extern "C" {
#endif

void win_sys_config_set(char sys_config[PATH_MAX]);
void win_usr_config_set(char usr_config[PATH_MAX]);

#ifdef __cplusplus
}
#endif

#endif  /* WIN32 */


#endif  /* libavrdude_h */
