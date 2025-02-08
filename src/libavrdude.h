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

#ifndef libavrdude_h
#define libavrdude_h

#include <stdio.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if !defined(WIN32)
#include <termios.h>
#endif

#ifdef LIBAVRDUDE_INCLUDE_INTERNAL_HEADERS
#error LIBAVRDUDE_INCLUDE_INTERNAL_HEADERS is defined. Do not do that.
#endif

#define LIBAVRDUDE_INCLUDE_INTERNAL_HEADERS
#include "libavrdude-avrintel.h"
#undef  LIBAVRDUDE_INCLUDE_INTERNAL_HEADERS

/*
 * The libavrdude library contains useful functions for programming Microchip's
 * 8-bit AVR microprocessors. The command line program avrdude was written
 * using this library; its source code is a good example of how to use the
 * library. Out of necessity libavrdude routinely changes PROGRAMMER, AVRPART
 * and other structures to keep up with new programmers and with new parts and
 * programming interfaces from Microchip. Any application that uses this
 * library should ensure that it links to a libavrdude binary that is
 * compatible with this header file, ideally the version that was shipped
 * together with this header file or one that was compiled from source together
 * with the application.
 */

typedef uint32_t Pinmask;

/*
 * Values returned by library functions. Some library functions also return a
 * count, i.e. a positive number greater than 0.
 */
#define LIBAVRDUDE_SUCCESS 0
#define LIBAVRDUDE_GENERAL_FAILURE (-1)
#define LIBAVRDUDE_NOTSUPPORTED (-2) // Operation not supported
#define LIBAVRDUDE_SOFTFAIL (-3)     // Returned, eg, if caller might proceed with a plan B
#define LIBAVRDUDE_EXIT (-4)         // End all operations in this session

// Message system

// This functions is supposed to be supplied by the application
int avrdude_message2(FILE *fp, int lno, const char *file, const char *func,
  int msgmode, int msglvl, const char *format, ...);

enum msglvl {
  MSG_EXT_ERROR = (-3),         // OS-type error, no -v option, can be suppressed with -qqqqq
  MSG_ERROR = (-2),             // Avrdude error, no -v option, can be suppressed with -qqqq
  MSG_WARNING = (-1),           // Warning, no -v option, can be suppressed with -qqq
  MSG_INFO = 0,                 // Commentary, no -v option, can be suppressed with -qq
  MSG_NOTICE = 1,               // Displayed with -v
  MSG_NOTICE2 = 2,              // Displayed with -vv
  MSG_DEBUG = 3,                // Displayed with -vvv
  MSG_TRACE = 4,                // Displayed with -vvvv, show trace communication
  MSG_TRACE2 = 5,               // Displayed with -vvvvv
};

enum msgmode {
  MSG2_PROGNAME = 1,            // Start by printing progname
  MSG2_FUNCTION = 2,            // Print calling function (1st arg) after progname if >= notice
  MSG2_FILELINE = 4,            // Print source file and line number after function if >= debug
  MSG2_TYPE = 8,                // Print message type after function or progname
  MSG2_INDENT1 = 16,            // Start by printing indentation of progname+1 blanks
  MSG2_INDENT2 = 32,            // Start by printing indentation of progname+2 blanks
  MSG2_FLUSH = 64,              // Flush before and after printing
  MSG2_LEFT_MARGIN = 128,       // Print \n unless last character printed was \n
  MSG2_UCFIRST = 256            // Uppercase first character of output
};

// Formerly lists.h

/*----------------------------------------------------------------------
  General purpose linked list routines - header file declarations.

  Author : Brian Dean
  Date   : 10 January, 1990
  ----------------------------------------------------------------------*/

typedef void *LISTID;
typedef void *LNODEID;

/*----------------------------------------------------------------------
  several defines to access the LIST structure as as stack or a queue
  --- use for program readability
  ----------------------------------------------------------------------*/
#define STACKID LISTID
#define SNODEID LNODEID
#define QUEUEID LISTID
#define QNODEID LNODEID

#define PUSH(s,d)    lins_n(s,d,1) // Push 'd' onto the stack
#define POP(s)       lrmv_n(s,1)   // Pop the stack
#define LOOKSTACK(s) lget_n(s,1)   // Look at the top of the stack but don't pop

#define ENQUEUE(q,d) lins_n(q,d,1) // Put 'd' on the end of the queue
#define DEQUEUE(q)   lrmv(q)       // Remove next item from the front of the queue
#define REQUEUE(q,d) ladd(q,d)     // Re-insert (push) item back on the front of the queue
#define LOOKQUEUE(q) lget(q)       // Return next item on the queue, but don't dequeue
#define QUEUELEN(q)  lsize(q)      // Length of the queue

#define LISTADD(l,d) ladd(l,d)     // Add to end of the list
#define LISTRMV(l,d) lrmv_d(l,d)   // Remove from end of the list

#ifdef __cplusplus
extern "C" {
#endif

// .................... Function Prototypes ....................

  LISTID lcreat(void *liststruct, int poolsize);
  void ldestroy(LISTID lid);
  void ldestroy_cb(LISTID lid, void (*ucleanup)(void *data_ptr));

  LNODEID lfirst(LISTID);       // Head of the list
  LNODEID llast(LISTID);        // Tail of the list
  LNODEID lnext(LNODEID);       // Next item in the list
  LNODEID lprev(LNODEID);       // Previous item in the list
  void *ldata(LNODEID);         // Data at the current position
  int lsize(LISTID);            // Number of elements in the list

  int ladd(LISTID lid, void *p);
  int laddo(LISTID lid, void *p, int (*compare)(const void *p1, const void *p2), LNODEID *firstdup);
  int laddu(LISTID lid, void *p, int (*compare)(const void *p1, const void *p2));
  int lins_n(LISTID lid, void *d, unsigned int n);
  int lins_ln(LISTID lid, LNODEID lnid, void *data_ptr);

  void *lget(LISTID lid);
  void *lget_n(LISTID lid, unsigned int n);
  LNODEID lget_ln(LISTID lid, unsigned int n);

  void *lrmv(LISTID lid);
  void *lrmv_n(LISTID lid, unsigned int n);
  void *lrmv_ln(LISTID lid, LNODEID lnid);
  void *lrmv_d(LISTID lid, void *data_ptr);

  LISTID lcat(LISTID lid1, LISTID lid2);

  void lsort(LISTID lid, int (*compare)(void *p1, void *p2));

  void *lsrch(LISTID lid, void *p, int (*compare)(void *p1, void *p2));

  int lprint(FILE *f, LISTID lid);

#ifdef __cplusplus
}
#endif

// Formerly avrpart.h

// AVR serial programming instructions
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
  AVR_CMDBIT_IGNORE,            // Bit is ignored on input and output
  AVR_CMDBIT_VALUE,             // Bit is set to 0 or 1 for input or output
  AVR_CMDBIT_ADDRESS,           // This bit represents an input address bit
  AVR_CMDBIT_INPUT,             // This bit is an input bit
  AVR_CMDBIT_OUTPUT             // This bit is an output bit
};

enum {                          // These are assigned to reset_disposition of AVRPART
  RESET_DEDICATED,              // Reset pin is dedicated
  RESET_IO                      // Reset pin might be configured as an I/O pin
};

enum ctl_stack {
  CTL_STACK_NONE,               // No control stack defined
  CTL_STACK_PP,                 // Parallel programming control stack
  CTL_STACK_HVSP                // High voltage serial programming control stack
};

// Serial programming instruction bit specifications
typedef struct cmdbit {
  int type;                     // AVR_CMDBIT_*
  int bitno;                    // Which input bit to use for this command bit
  int value;                    // Bit value if type == AVR_CMDBIT_VALUE
} CMDBIT;

typedef struct opcode {
  CMDBIT bit[32];               // Opcode bit specs
} OPCODE;

// Any changes here, please also reflect in dev_part_strct() of developer_opts.c
#define AVRPART_SERIALOK       1        // Part supports serial programming
#define AVRPART_PARALLELOK     2        // Part supports parallel programming
#define AVRPART_PSEUDOPARALLEL 4        // Part has pseudo parallel support
#define AVRPART_ALLOWFULLPAGEBITSTREAM 8   // JTAG ICE mkII param
#define AVRPART_ENABLEPAGEPROGRAMMING 16   // JTAG ICE mkII param
#define AVRPART_IS_AT90S1200  32        // Part is an AT90S1200, needs special treatment

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
#define PM_Classic (PM_TPI | PM_ISP | PM_HVSP | PM_HVPP | PM_debugWIRE | PM_JTAG | PM_JTAGmkI)
#define PM_ALL           0x1fff // All programming interfaces

// Shortcut test for programmers and parts
#define is_spm(x)       (!!((x)->prog_modes & PM_SPM))
#define is_tpi(x)       (!!((x)->prog_modes & PM_TPI))
#define is_isp(x)       (!!((x)->prog_modes & PM_ISP))
#define is_pdi(x)       (!!((x)->prog_modes & PM_PDI))
#define is_updi(x)      (!!((x)->prog_modes & PM_UPDI))
#define is_hvsp(x)      (!!((x)->prog_modes & PM_HVSP))
#define is_hvpp(x)      (!!((x)->prog_modes & PM_HVPP))
#define is_debugwire(x) (!!((x)->prog_modes & PM_debugWIRE))
#define is_jtag(x)      (!!((x)->prog_modes & PM_JTAG))
#define is_jtagmki(x)   (!!((x)->prog_modes & PM_JTAGmkI))
#define is_xmegajtag(x) (!!((x)->prog_modes & PM_XMEGAJTAG))
#define is_avr32jtag(x) (!!((x)->prog_modes & PM_AVR32JTAG))
#define is_awire(x)     (!!((x)->prog_modes & PM_aWire))
#define is_classic(x)   (!!((x)->prog_modes & PM_Classic))
#define is_avr32(x)     (!!((x)->prog_modes & (PM_AVR32JTAG | PM_aWire)))

// Set of overlapping programming modes of programmer and part
#define joint_pm(pgm, p) ((pgm)->prog_modes & (p)->prog_modes)

// Shortcut test whether both programmer and part have that programming mode
#define both_spm(pgm, p)       (!!(joint_pm(pgm, p) & PM_SPM))
#define both_tpi(pgm, p)       (!!(joint_pm(pgm, p) & PM_TPI))
#define both_isp(pgm, p)       (!!(joint_pm(pgm, p) & PM_ISP))
#define both_pdi(pgm, p)       (!!(joint_pm(pgm, p) & PM_PDI))
#define both_updi(pgm, p)      (!!(joint_pm(pgm, p) & PM_UPDI))
#define both_hvsp(pgm, p)      (!!(joint_pm(pgm, p) & PM_HVSP))
#define both_hvpp(pgm, p)      (!!(joint_pm(pgm, p) & PM_HVPP))
#define both_debugwire(pgm, p) (!!(joint_pm(pgm, p) & PM_debugWIRE))
#define both_jtag(pgm, p)      (!!(joint_pm(pgm, p) & PM_JTAG))
#define both_jtagmki(pgm, p)   (!!(joint_pm(pgm, p) & PM_JTAGmkI))
#define both_xmegajtag(pgm, p) (!!(joint_pm(pgm, p) & PM_XMEGAJTAG))
#define both_avr32jtag(pgm, p) (!!(joint_pm(pgm, p) & PM_AVR32JTAG))
#define both_awire(pgm, p)     (!!(joint_pm(pgm, p) & PM_aWire))
#define both_classic(pgm, p)   (!!(joint_pm(pgm, p) & PM_Classic))

#define HV_UPDI_VARIANT_0     0 // Shared UPDI/GPIO/RESET pin, HV on UPDI pin (tinyAVR0/1/2)
#define HV_UPDI_VARIANT_1     1 // Dedicated UPDI pin, no HV (megaAVR0/AVR-Dx)
#define HV_UPDI_VARIANT_2     2 // Shared UPDI pin, HV on _RESET (AVR-DD/AVR-Ex)

#define HAS_SUFFER            1
#define HAS_VTARG_SWITCH      2
#define HAS_VTARG_ADJ         4
#define HAS_VTARG_READ        8
#define HAS_FOSC_ADJ         16
#define HAS_VAREF_ADJ        32

#define AVR_FAMILYIDLEN       7
#define AVR_SIBLEN           32
#define AVR_CHIP_REVLEN       1
#define CTL_STACK_SIZE       32
#define FLASH_INSTR_SIZE      3
#define EEPROM_INSTR_SIZE    20

#define TAG_ALLOCATED         1 // Memory byte is allocated

/*
 * Any changes in AVRPART or AVRMEM, please also ensure changes are made in
 *  - lexer.l
 *  - Either Component avr_comp[] of config.c or in config_gram.y
 *  - dev_part_strct() in developer_opts.c
 *  - avr_new_part() and/or avr_new_mem() in avrpart.c for
 *    initialisation; note that all const char * must be initialised with ""
 */
typedef struct avrpart {
  const char *desc;             // Long part name
  const char *id;               // Short part name
  LISTID comments;              // Used by developer options -p*/[ASsr...]
  LISTID variants;              // String with variant name and chip properties
  const char *parent_id;        // Used by developer options
  const char *family_id;        // Family id in the SIB (avr8x)
  int prog_modes;               // Programming interfaces, see #define PM_...
  int mcuid;                    // Unique id in 0..2039 for urclock programmer
  int archnum;                  // Avr-gcc architecture number for the part
  int n_interrupts;             // Number of interrupts, used for vector bootloaders
  int n_page_erase;             // If set, number of pages erased during NVM erase
  int n_boot_sections;          // Number of boot sections
  int boot_section_size;        // Size of (smallest) boot section, if any
  int hvupdi_variant;           // HV pulse on UPDI pin, no pin or RESET pin
  int stk500_devcode;           // Stk500 device code
  int avr910_devcode;           // Avr910 device code
  int chip_erase_delay;         // Microseconds
  unsigned char pagel;          // For parallel programming
  unsigned char bs2;            // For parallel programming
  unsigned char signature[3];   // Expected value of signature bytes
  unsigned short usbpid;        // USB DFU product ID (0 = none)
  int reset_disposition;        // See RESET_ enums
  int retry_pulse;              // Retry program enable by pulsing this pin (PIN_AVR_*)
  unsigned flags;               // See AVRPART_ masks

  // STK500 v2 parameters from ATDF files
  int timeout;
  int stabdelay;
  int cmdexedelay;
  int synchloops;
  int bytedelay;
  int pollindex;
  unsigned char pollvalue;
  int predelay;
  int postdelay;
  int pollmethod;

  enum ctl_stack ctl_stack_type;                 // What to use the ctl stack for
  unsigned char controlstack[CTL_STACK_SIZE];    // Stk500v2 PP/HVSP ctl stack
  unsigned char flash_instr[FLASH_INSTR_SIZE];   // Flash instructions (debugWire, JTAG)
  unsigned char eeprom_instr[EEPROM_INSTR_SIZE]; // EEPROM instructions (debugWire, JTAG)

  // STK500 v2 hv mode parameters
  int hventerstabdelay;
  int progmodedelay;
  int latchcycles;
  int togglevtg;
  int poweroffdelay;
  int resetdelayms;
  int resetdelayus;
  int hvleavestabdelay;
  int resetdelay;
  int chiperasepulsewidth;
  int chiperasepolltimeout;
  int chiperasetime;
  int programfusepulsewidth;
  int programfusepolltimeout;
  int programlockpulsewidth;
  int programlockpolltimeout;
  int synchcycles;
  int hvspcmdexedelay;

  // debugWIRE and/or JTAG ICE mkII XML file parameters
  unsigned char idr;            // I/O address of IDR (OCD) reg
  unsigned char rampz;          // I/O address of RAMPZ reg
  unsigned char spmcr;          // Memory address of SPMCR reg
  unsigned char eecr;           // Memory address of EECR reg
  unsigned char eind;           // Memory address of EIND reg
  unsigned int mcu_base;        // Base address of MCU control block in ATxmega devices
  unsigned int nvm_base;        // Base address of NVM controller in ATxmega devices
  unsigned int ocd_base;        // Base address of OCD module in AVR8X/UPDI devices
  unsigned int syscfg_base;     // Base address of revision ID in AVR8X/UPDI devices
  int ocdrev;                   // OCD revision (JTAGICE3 parameter, from AS6 XML files)

  // Bootloader parameter
  unsigned char autobaud_sync;  // Sync byte for bootloader autobaud, must be <= 0x30
  int factory_fcpu;             // Initial F_CPU after reset assuming factory settings

  OPCODE *op[AVR_OP_MAX];       // Opcodes

  LISTID mem;                   // AVR memory definitions
  LISTID mem_alias;             // Memory alias definitions
  const char *config_file;      // Config file where defined
  int lineno;                   // Config file line number
} AVRPART;

typedef unsigned int Memtype;
typedef struct {
  const char *str;
  Memtype type;
} Memtable;

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
#define MEM_SIGROW      (1<<11) // sigrow prodsig
#define MEM_PRODSIG  MEM_SIGROW
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
#define MEM_USERSIG MEM_USERROW
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
#define avr_locate_prodsig(p) avr_locate_mem_by_type((p), MEM_PRODSIG)
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
#define avr_locate_usersig(p) avr_locate_mem_by_type((p), MEM_USERSIG)
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
#define mem_is_prodsig(mem) (!!((mem)->type & MEM_PRODSIG))
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
#define mem_is_usersig(mem) (!!((mem)->type & MEM_USERSIG))
#define mem_is_io(mem) (!!((mem)->type & MEM_IO))
#define mem_is_sram(mem) (!!((mem)->type & MEM_SRAM))
#define mem_is_sib(mem) (!!((mem)->type & MEM_SIB))

#define mem_is_in_flash(mem) (!!((mem)->type & MEM_IN_FLASH))
#define mem_is_a_fuse(mem) (!!((mem)->type & MEM_IS_A_FUSE))
#define mem_is_in_fuses(mem) (!!((mem)->type & (MEM_FUSES | MEM_IS_A_FUSE))) // If fuses exists!
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

#define mem_fuse_offset(mem) ((mem)->type & MEM_FUSEOFF_MASK)   // Valid if mem_is_a_fuse(mem)

typedef struct avrmem {
  const char *desc;             // Memory description ("flash", "eeprom", etc)
  Memtype type;                 // Internally used type, cannot be set in conf files
  LISTID comments;              // Used by developer options -p*/[ASsr...]
  int paged;                    // 16-bit page addressed, e.g., ATmega flash but not EEPROM
  int size;                     // Total memory size in bytes
  int page_size;                // Size of memory page (if page addressed)
  int num_pages;                // Number of pages (if page addressed)
  int initval;                  // Factory setting of fuses and lock bits
  int bitmask;                  // Bits used in fuses and lock bits
  int n_word_writes;            // TPI only: number words to write at a time
  unsigned int offset;          // Offset in IO memory (ATxmega, UPDI, some classic memories)
  int min_write_delay;          // Microseconds
  int max_write_delay;          // Microseconds
  int pwroff_after_write;       // After this memory is written to, the device must be powered off and back on, see errata
                                // https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/doc1042.pdf
  unsigned char readback[2];    // Polled read-back values

  int mode;                     // Stk500 v2 xml file parameter
  int delay;                    // Stk500 v2 xml file parameter
  int blocksize;                // Stk500 v2 xml file parameter
  int readsize;                 // Stk500 v2 xml file parameter
  int pollindex;                // Stk500 v2 xml file parameter

  unsigned char *buf;           // Pointer to memory buffer
  unsigned char *tags;          // Allocation tags
  OPCODE *op[AVR_OP_MAX];       // Opcodes
} AVRMEM;

typedef struct avrmem_alias {
  const char *desc;             // Alias name, eg, syscfg0
  AVRMEM *aliased_mem;
} AVRMEM_ALIAS;

typedef struct programmer PROGRAMMER;   // Forward declaration

#ifdef __cplusplus
extern "C" {
#endif

  int intlog2(unsigned int n);

  // Functions for OPCODE structures
  OPCODE *avr_new_opcode(void);
  void avr_free_opcode(OPCODE *op);
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

  // Functions for AVRMEM structures
  AVRMEM *avr_new_mem(void);
  AVRMEM *avr_new_memory(const char *name, int size);
  AVRMEM_ALIAS *avr_new_memalias(void);
  const char *avr_mem_name(const AVRPART *p, const AVRMEM *mem);
  int avr_initmem(const AVRPART *p);
  AVRMEM *avr_dup_mem(const AVRMEM *m);
  void avr_free_mem(AVRMEM *m);
  void avr_free_memalias(AVRMEM_ALIAS *m);
  AVRMEM *avr_locate_mem(const AVRPART *p, const char *desc);
  AVRMEM *avr_locate_mem_noalias(const AVRPART *p, const char *desc);
  AVRMEM *avr_locate_fuse_by_offset(const AVRPART *p, unsigned int off);
  AVRMEM *avr_locate_mem_by_type(const AVRPART *p, Memtype type);
  unsigned int avr_data_offset(const AVRPART *p);
  AVRMEM_ALIAS *avr_locate_memalias(const AVRPART *p, const char *desc);
  AVRMEM_ALIAS *avr_find_memalias(const AVRPART *p, const AVRMEM *m_orig);
  void avr_mem_display(FILE *f, const PROGRAMMER *pgm, const AVRPART *p, const char *prefix);

  // Functions for AVRPART structures
  AVRPART *avr_new_part(void);
  AVRPART *avr_dup_part(const AVRPART *d);
  void avr_free_part(AVRPART *d);
  AVRPART *locate_part(const LISTID parts, const char *partdesc);
  AVRPART *locate_part_by_avr910_devcode(const LISTID parts, int devcode);
  AVRPART *locate_part_by_signature(const LISTID parts, unsigned char *sig, int sigsize);
  AVRPART *locate_part_by_signature_pm(const LISTID parts, unsigned char *sig, int sigsize, int prog_modes);
  int avr_sig_compatible(const unsigned char *sig1, const unsigned char *sig2);

  char *avr_prog_modes(int pm), *str_prog_modes(int pm), *dev_prog_modes(int pm);
  void avr_display(FILE *f, const PROGRAMMER *pgm, const AVRPART *p, const char *prefix, int verbose);
  int avr_variants_display(FILE *f, const AVRPART *p, const char *prefix);

  typedef void (*walk_avrparts_cb)(const char *name, const char *desc,
    const char *cfgname, int cfglineno, void *cookie);
  void walk_avrparts(LISTID avrparts, walk_avrparts_cb cb, void *cookie);
  void sort_avrparts(LISTID avrparts);

  // cmp can be, eg, str_caseeq or str_casematch
  int part_eq(AVRPART *p, const char *string, int (*cmp)(const char *, const char *));

  int compare_memory_masked(AVRMEM *m, uint8_t buf1, uint8_t buf2);

#ifdef __cplusplus
}
#endif

// Formerly pindefs.h

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
#define PIN_INVERSE (~(PIN_MASK)) // Flag for inverted pin in serbb
#define PIN_MIN               0   // Smallest allowed pin number
#define PIN_MAX              31   // Largest allowed pin number

#ifdef HAVE_LINUXGPIO

// Embedded systems might have a lot more gpio than only 0-31
#undef PIN_MAX
#define PIN_MAX     1000        // Largest allowed pin number
#endif

// Number of pins in each element of the bitfield
#define PIN_FIELD_ELEMENT_SIZE (sizeof(Pinmask)*8)

// Numer of elements to store the complete bitfield of all pins
#define PIN_FIELD_SIZE ((PIN_MAX+1 + PIN_FIELD_ELEMENT_SIZE-1)/PIN_FIELD_ELEMENT_SIZE)

/*
 * This sets the corresponding bits to 1 or 0, the inverse mask is used to
 * invert the value in necessary. It uses only the lowest element (index=0) of
 * the bitfield, which should be enough for most programmers.
 *
 * @param[in] x       input value
 * @param[in] pgm     the programmer whose pin definitions to use
 * @param[in] pinname the logical name of the pin (PIN_AVR_*, ...)
 * @param[in] level   the logical level (level != 0 => 1, level == 0 => 0),
 *                    if the pin is defined as inverted the resulting bit is
 *                    also inverted
 * @returns           the input value with the relevant bits modified
 */
#define SET_BITS_0(x, pgm, pinname, level) ( ((x) & ~(pgm)->pin[pinname].mask[0]) | \
  ((pgm)->pin[pinname].mask[0] & ((level)? ~(pgm)->pin[pinname].inverse[0]: (pgm)->pin[pinname].inverse[0])) \
)

/*
 * Check if the corresponding bit is set (returns != 0) or cleared. The inverse
 * mask is used, to invert the relevant bits. If the pin definition contains
 * multiple pins, then a single set pin leads to return value != 0. Then you
 * have to check the relevant bits of the returned value, if you need more
 * information. It uses only the lowest element (index=0) of the bitfield,
 * which should be enough for most programmers.
 *
 * @param[in] x       input value
 * @param[in] pgm     the programmer whose pin definitions to use
 * @param[in] pinname the logical name of the pin (PIN_AVR_*, ...)
 * @returns           the input value with only the relevant bits (which are already inverted,
 *                      so you get always the logical level)
 */
#define GET_BITS_0(x, pgm,pinname) (((x) ^ (pgm)->pin[pinname].inverse[0]) & (pgm)->pin[pinname].mask[0])

// Data structure to hold used pins by logical function (PIN_AVR_*, ...)
struct pindef {
  Pinmask mask[PIN_FIELD_SIZE]; ///< bitfield of used pins
  Pinmask inverse[PIN_FIELD_SIZE]; ///< bitfield of inverse/normal usage of used pins
};

// Data structure to define a checklist of valid pins for each function
typedef struct pin_checklist {
  int pinname;                  ///< logical pinname eg. PIN_AVR_SCK
  int mandatory;                ///< is this a mandatory pin
  const struct pindef *valid_pins; ///< mask defines allowed pins, inverse define is they might be used inverted
} Pin_checklist;

/*
 * Adds a pin in the pin definition as normal or inverse pin.
 *
 * @param[out] pindef pin definition to update
 * @param[in] pin number of pin [0..PIN_MAX]
 * @param[in] inverse inverse (true) or normal (false) pin
 */
void pin_set_value(struct pindef *const pindef, const int pin, const bool inverse);

/*
 * Clear all defined pins in pindef
 *
 * @param[out] pindef pin definition to clear
 */
void pin_clear_all(struct pindef *const pindef);

/*
 * Convert for given programmer new pin definitions to old pin definitions.
 *
 * @param[inout] pgm programmer whose pins shall be converted.
 */
int pgm_fill_old_pins(PROGRAMMER *const pgm);

/*
 * This function checks all pin of pgm against the constraints given in the checklist.
 * It checks if
 * @li any invalid pins are used
 * @li valid pins are used inverted when not allowed
 * @li any pins are used by more than one function
 * @li any mandatory pin is not set all.
 *
 * In case of any error it report the wrong function and the pin numbers
 * For verbose >= MSG_NOTICE2 it also reports the possible correct value
 * For verbose >= MSG_DEBUG it shows also which pins were ok
 *
 * @param[in] pgm the programmer to check
 * @param[in] checklist the constraint for the pins
 * @param[in] size the number of entries in checklist
 * @param[in] output false suppresses error messages to the user
 * @returns 0 if all pin definitions are valid, -1 otherwise
 */
int pins_check(const PROGRAMMER *const pgm, const Pin_checklist *const checklist, const int size, const bool output);

/*
 * Returns the name of the pin as string
 *
 * @param pinname the pinname which we want as string.
 * @returns a string with the pinname, or <unknown> if pinname is invalid.
 */
const char *avr_pin_name(int pinname);

/*
 * Returns the name of the pin as lowercase string
 *
 * @param pinname the pinname which we want as string.
 * @returns a lowercase string with the pinname, or <unknown> if pinname is invalid.
 */
const char *avr_pin_lcname(int pinname);

/*
 * This function returns a string of defined pins, eg, ~1, 2, ~4, ~5, 7 or ""
 *
 * @param[in] pindef the pin definition for which we want the string representation
 * @returns a temporary string that lives in closed-circuit space
 */
const char *pins_to_str(const struct pindef *const pindef);

/*
 * This function returns a string representation of pins in the mask, eg, 1, 3, 5-7, 9, 12
 * Consecutive pin numbers are represented as start-end.
 *
 * @param[in] pinmask the pin mask for which we want the string representation
 * @returns a temporary string that lives in closed-circuit space
 */
const char *pinmask_to_str(const Pinmask *const pinmask);

// Formerly serial.h

/*
 * This is the API for the generic serial interface. The implementations are
 * actually provided by the target dependant files:
 *
 * ser_posix.c : posix serial interface.
 * ser_win32.c : native win32 serial interface.
 *
 * The target file will be selected at configure time.
 */

extern long serial_recv_timeout;        // ms
extern long serial_drain_timeout;       // ms

union filedescriptor {
  int ifd;
  void *pfd;
  struct {
    void *handle;
    int rep;                    // Bulk read endpoint
    int wep;                    // Bulk write endpoint
    int eep;                    // Event read endpoint
    int max_xfer;               // Max transfer size
    int use_interrupt_xfer;     // Device uses interrupt transfers
  } usb;
};

#define SERIAL_CS5       0x0000
#define SERIAL_CS6       0x0001
#define SERIAL_CS7       0x0002
#define SERIAL_CS8       0x0004

#define SERIAL_NO_CSTOPB 0x0000
#define SERIAL_CSTOPB    0x0008

#define SERIAL_NO_CREAD  0x0000
#define SERIAL_CREAD     0x0010

#define SERIAL_NO_PARITY 0x0000
#define SERIAL_PARENB    0x0020
#define SERIAL_PARODD    0x0040

#define SERIAL_NO_CLOCAL 0x0000
#define SERIAL_CLOCAL    0x0080

#define SERIAL_8N1 (SERIAL_CS8 | SERIAL_NO_CSTOPB | SERIAL_CREAD | SERIAL_NO_PARITY | SERIAL_CLOCAL)
#define SERIAL_8E1 (SERIAL_CS8 | SERIAL_NO_CSTOPB | SERIAL_CREAD | SERIAL_PARENB    | SERIAL_CLOCAL)
#define SERIAL_8E2 (SERIAL_CS8 | SERIAL_CSTOPB    | SERIAL_CREAD | SERIAL_PARENB    | SERIAL_CLOCAL)

union pinfo {
  struct {
    long baud;
    unsigned long cflags;
  } serialinfo;
  struct {
    unsigned short vid;
    unsigned short pid;
    unsigned short flags;
#define PINFO_FL_USEHID  0x0001
#define PINFO_FL_SILENT  0x0002 // Don't complain if not found
  } usbinfo;
};

struct serial_device {
  // Open should return -1 on error, other values on success
  int (*open)(const char *port, union pinfo pinfo, union filedescriptor *fd);
  int (*setparams)(const union filedescriptor *fd, long baud, unsigned long cflags);
  void (*close)(union filedescriptor *fd);
  void (*rawclose)(union filedescriptor *fd);  // Don't restore terminal attributes (Linux)

  int (*send)(const union filedescriptor *fd, const unsigned char *buf, size_t buflen);
  int (*recv)(const union filedescriptor *fd, unsigned char *buf, size_t buflen);
  int (*drain)(const union filedescriptor *fd, int display);

  int (*set_dtr_rts)(const union filedescriptor *fd, int is_on);

  const char *usbsn;
  const char *usbproduct;
  int flags;
#define SERDEV_FL_NONE        0 // No flags
#define SERDEV_FL_CANSETSPEED 1 // Device can change speed
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
  unsigned char *cont, *copy;   // Current memory contens and device copy of it
  unsigned char *iscached;      // iscached[i] set when page i has been loaded
} AVR_Cache;

// Formerly pgm.h

#define OFF                   0 // Many contexts: reset, power, LEDs, ...
#define ON                    1 // Many contexts

#define PGM_TYPELEN 32

typedef enum {
  EXIT_VCC_UNSPEC,
  EXIT_VCC_ENABLED,
  EXIT_VCC_DISABLED
} Exit_vcc;

typedef enum {
  EXIT_RESET_UNSPEC,
  EXIT_RESET_ENABLED,
  EXIT_RESET_DISABLED
} Exit_reset;

typedef enum {
  EXIT_DATAHIGH_UNSPEC,
  EXIT_DATAHIGH_ENABLED,
  EXIT_DATAHIGH_DISABLED
} Exit_datahigh;

typedef enum {
  CONNTYPE_PARALLEL,
  CONNTYPE_SERIAL,
  CONNTYPE_USB,
  CONNTYPE_SPI,
  CONNTYPE_LINUXGPIO
} Conntype;

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
} Leds;

/*
 * Any changes in PROGRAMMER, please also ensure changes are made in
 *  - lexer.l
 *  - Either Component avr_comp[] of config.c or config_gram.y
 *  - dev_pgm_strct() in developer_opts.c
 *  - pgm_new() in pgm.c for initialisation; note that all const char * must be initialised with ""
 */
typedef struct programmer {
  LISTID id;
  const char *desc;
  void (*initpgm)(PROGRAMMER *pgm);  // Sets up the AVRDUDE programmer
  LISTID comments;              // Used by developer options -c*/[ASsr...]
  const char *parent_id;        // Used by developer options
  int prog_modes;               // Programming interfaces, see #define PM_...
  int is_serialadapter;         // Programmer is also a serialadapter
  int extra_features;
  struct pindef pin[N_PINS];
  Conntype conntype;
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
  const char *port;
  unsigned int pinno[N_PINS];   // TODO: to be removed if old pin data no longer needed
  Exit_vcc exit_vcc;            // Should these be set in avrdude.conf?
  Exit_reset exit_reset;
  Exit_datahigh exit_datahigh;
  int ppidata;
  int ppictrl;
  int ispdelay;                 // ISP clock delay
  int page_size;                // Page size if the programmer supports paged write/load
  double bitclock;              // JTAG ICE clock period in microseconds
  Leds *leds;                   // State of LEDs as tracked by led_...()  functions in leds.c

  int (*rdy_led)(const PROGRAMMER *pgm, int value);
  int (*err_led)(const PROGRAMMER *pgm, int value);
  int (*pgm_led)(const PROGRAMMER *pgm, int value);
  int (*vfy_led)(const PROGRAMMER *pgm, int value);
  int (*initialize)(const PROGRAMMER *pgm, const AVRPART *p); // Sets up the physical programmer
  void (*display)(const PROGRAMMER *pgm, const char *p);
  void (*enable)(PROGRAMMER *pgm, const AVRPART *p);
  void (*disable)(const PROGRAMMER *pgm);
  void (*powerup)(const PROGRAMMER *pgm);
  void (*powerdown)(const PROGRAMMER *pgm);
  int (*program_enable)(const PROGRAMMER *pgm, const AVRPART *p);
  int (*chip_erase)(const PROGRAMMER *pgm, const AVRPART *p);
  int (*unlock)(const PROGRAMMER *pgm, const AVRPART *p);
  int (*cmd)(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res);
  int (*cmd_tpi)(const PROGRAMMER *pgm, const unsigned char *cmd, int cmd_len,
    unsigned char *res, int res_len);
  int (*spi)(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res, int count);
  int (*open)(PROGRAMMER *pgm, const char *port);
  void (*close)(PROGRAMMER *pgm);
  int (*paged_write)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned int pg_size, unsigned int addr, unsigned int n);
  int (*paged_load)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned int pg_size, unsigned int addr, unsigned int n);
  int (*page_erase)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned int addr);
  void (*write_setup)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m);
  int (*write_byte)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned long addr, unsigned char value);
  int (*read_byte)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned long addr, unsigned char *value);
  int (*read_sig_bytes)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m);
  int (*read_sib)(const PROGRAMMER *pgm, const AVRPART *p, char *sib);
  int (*read_chip_rev)(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev);
  int (*term_keep_alive)(const PROGRAMMER *pgm, const AVRPART *p);
  int (*end_programming)(const PROGRAMMER *pgm, const AVRPART *p);

  void (*print_parms)(const PROGRAMMER *pgm, FILE *fp);
  int (*set_vtarget)(const PROGRAMMER *pgm, double v);
  int (*get_vtarget)(const PROGRAMMER *pgm, double *v);
  int (*set_varef)(const PROGRAMMER *pgm, unsigned int chan, double v);
  int (*get_varef)(const PROGRAMMER *pgm, unsigned int chan, double *v);
  int (*set_fosc)(const PROGRAMMER *pgm, double v);
  int (*get_fosc)(const PROGRAMMER *pgm, double *v);
  int (*set_sck_period)(const PROGRAMMER *pgm, double v);
  int (*get_sck_period)(const PROGRAMMER *pgm, double *v);
  int (*setpin)(const PROGRAMMER *pgm, int pinfunc, int value);
  int (*getpin)(const PROGRAMMER *pgm, int pinfunc);
  int (*highpulsepin)(const PROGRAMMER *pgm, int pinfunc);
  int (*parseexitspecs)(PROGRAMMER *pgm, const char *s);
  int (*perform_osccal)(const PROGRAMMER *pgm);
  int (*parseextparams)(const PROGRAMMER *pgm, const LISTID xparams);
  void (*setup)(PROGRAMMER *pgm);
  void (*teardown)(PROGRAMMER *pgm);
  int (*flash_readhook)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *flm,
    const char *fname, int size);

  // Cached r/w API for terminal reads/writes
  int (*write_byte_cached)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned long addr, unsigned char value);
  int (*read_byte_cached)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned long addr, unsigned char *value);
  int (*chip_erase_cached)(const PROGRAMMER *pgm, const AVRPART *p);
  int (*page_erase_cached)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned int addr);
  int (*readonly)(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned int addr);
  int (*flush_cache)(const PROGRAMMER *pgm, const AVRPART *p);
  int (*reset_cache)(const PROGRAMMER *pgm, const AVRPART *p);
  AVR_Cache *cp_flash, *cp_eeprom, *cp_bootrow, *cp_usersig;

  const char *config_file;      // Config file where defined
  int lineno;                   // Config file line number
  void *cookie;                 // For private use by the programmer
  char flag;                    // For use by pgm->initpgm()
} PROGRAMMER;

typedef PROGRAMMER SERIALADAPTER;       // Only a subset is needed for serial adapters
int is_programmer(const PROGRAMMER *p);
int is_serialadapter(const SERIALADAPTER *p);
void list_serialadapters(FILE *fp, const char *prefix, LISTID programmers);
void serialadapter_not_found(const char *sea_id);

#define NO_PIN   (PIN_MAX + 1U) // Magic pinno[] value for unused pins

#ifdef __cplusplus
extern "C" {
#endif

  void pgm_init_functions(PROGRAMMER *pgm);
  PROGRAMMER *pgm_new(void);
  PROGRAMMER *pgm_dup(const PROGRAMMER *src);
  void pgm_free(PROGRAMMER *p);
  void programmer_display(PROGRAMMER *pgm, const char *p);

// Show is a mask like this (1<<PIN_AVR_SCK)|(1<<PIN_AVR_SDO)| ...
#define SHOW_ALL_PINS (~0u)
#define SHOW_PPI_PINS ((1<<PPI_AVR_VCC)|(1<<PPI_AVR_BUFF))
#define SHOW_AVR_PINS ((1<<PIN_AVR_RESET)|(1<<PIN_AVR_SCK)|(1<<PIN_AVR_SDO)|(1<<PIN_AVR_SDI))
#define SHOW_JTAG_PINS ((1<<PIN_JTAG_TCK)|(1<<PIN_JTAG_TDI)|(1<<PIN_JTAG_TDO)|(1<<PIN_JTAG_TMS))
#define SHOW_LED_PINS ((1<<PIN_LED_ERR)|(1<<PIN_LED_RDY)|(1<<PIN_LED_PGM)|(1<<PIN_LED_VFY))
  void pgm_display_generic_mask(const PROGRAMMER *pgm, const char *p, unsigned int show);
  void pgm_display_generic(const PROGRAMMER *pgm, const char *p);

  PROGRAMMER *locate_programmer_set(const LISTID programmers, const char *id,
    const char **setid);
  PROGRAMMER *locate_programmer_starts_set(const LISTID programmers, const char *id,
    const char **setid, AVRPART *prt);
  PROGRAMMER *locate_programmer(const LISTID programmers, const char *configid);

  typedef void (*walk_programmers_cb)(const char *name, const char *desc,
    const char *cfgname, int cfglineno, void *cookie);
  void walk_programmers(LISTID programmers, walk_programmers_cb cb, void *cookie);

  void sort_programmers(LISTID programmers);

#ifdef __cplusplus
}
#endif

// Formerly avr.h

typedef void (*FP_UpdateProgress)(int percent, double etime, const char *hdr, int finish);

extern struct avrpart parts[];
extern Memtable avr_mem_order[100];

extern FP_UpdateProgress update_progress;

#ifdef __cplusplus
extern "C" {
#endif

  int avr_tpi_poll_nvmbsy(const PROGRAMMER *pgm);
  int avr_tpi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
  int avr_tpi_program_enable(const PROGRAMMER *pgm, const AVRPART *p, unsigned char guard_time);
  int avr_sigrow_offset(const AVRPART *p, const AVRMEM *mem, int addr);
  int avr_flash_offset(const AVRPART *p, const AVRMEM *mem, int addr);
  int avr_read_byte_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned long addr, unsigned char *value);
  int avr_read_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, const AVRPART *v);
  int avr_read(const PROGRAMMER *pgm, const AVRPART *p, const char *memstr, const AVRPART *v);
  int avr_write_page(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned long addr);

  uint64_t avr_ustimestamp(void);
  uint64_t avr_mstimestamp(void);
  double avr_timestamp(void);
  void init_cx(PROGRAMMER *pgm);
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
  int avr_verify_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRPART *v, const AVRMEM *a, int size);
  int avr_get_cycle_count(const PROGRAMMER *pgm, const AVRPART *p, int *cycles);
  int avr_put_cycle_count(const PROGRAMMER *pgm, const AVRPART *p, int cycles);

  int avr_mem_exclude(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem);
  int avr_get_mem_type(const char *str);
#ifndef TO_BE_DEPRECATED_IN_2026
  int avr_mem_is_flash_type(const AVRMEM *mem);
  int avr_mem_is_eeprom_type(const AVRMEM *mem);
  int avr_mem_is_usersig_type(const AVRMEM *mem);
#endif
  int avr_mem_cmp(void *mem1, void *mem2);
  int avr_mem_is_known(const char *str);
  int avr_mem_might_be_known(const char *str);
  int avr_mem_hiaddr(const AVRMEM *mem);

  int avr_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
  int avr_unlock(const PROGRAMMER *pgm, const AVRPART *p);
  void report_progress(int completed, int total, const char *hdr);
  void trace_buffer(const char *funstr, const unsigned char *buf, size_t buflen);
  int avr_has_paged_access(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m);
  int avr_read_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    int addr, unsigned char *buf);
  int avr_write_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    int addr, unsigned char *data);
  int avr_is_and(const unsigned char *s1, const unsigned char *s2, const unsigned char *s3, size_t n);

  // Bytewise cached read/write API
  int avr_read_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned long addr, unsigned char *value);
  int avr_write_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned long addr, unsigned char data);
  int avr_chip_erase_cached(const PROGRAMMER *pgm, const AVRPART *p);
  int avr_page_erase_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned int addr);
  int avr_flush_cache(const PROGRAMMER *pgm, const AVRPART *p);
  int avr_reset_cache(const PROGRAMMER *pgm, const AVRPART *p);


typedef struct avr_buildinfo_item {
  const char *const key;
  const char *const value;
} avr_buildinfo_item;

typedef struct avr_buildinfo {
  const char *const name;
  const char *const version;
  avr_buildinfo_item items[];
} avr_buildinfo;

extern const avr_buildinfo libavrdude_buildinfo;


#ifdef __cplusplus
}
#endif

// Formerly fileio.h

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
  int op;
  char *mode;
  char *iodesc;
  char *dir;
  char *rw;
  unsigned int fileoffset;
};

typedef struct {
  int addr, len;
} Segment;

enum {
  FIO_READ,
  FIO_WRITE,
  FIO_READ_FOR_VERIFY,
};

#ifdef __cplusplus
extern "C" {
#endif

  FILEFMT fileio_format(char c);
  FILEFMT fileio_format_with_errmsg(char c, const char *who);
  char *fileio_fmtstr(FILEFMT format);
  int fileio_fmtchr(FILEFMT format);
  AVRMEM *fileio_any_memory(const char *name);
  unsigned fileio_mem_offset(const AVRPART *p, const AVRMEM *mem);
  FILE *fileio_fopenr(const char *fname);
  int fileio_fmt_autodetect_fp(FILE *f);
  int fileio_fmt_autodetect(const char *fname);
  int fileio_mem(int oprwv, const char *filename, FILEFMT format, const AVRPART *p, const AVRMEM *mem, int size);
  int fileio(int oprwv, const char *filename, FILEFMT format, const AVRPART *p, const char *memstr, int size);
  int segment_normalise(const AVRMEM *mem, Segment *segp);
  int fileio_segments(int oprwv, const char *filename, FILEFMT format,
    const AVRPART *p, const AVRMEM *mem, int n, const Segment *seglist);

#ifdef __cplusplus
}
#endif

// Formerly update.h

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
  UF_NOHEADING = 8,
};

typedef struct update {
  const char *cmdline;          // -T line is stored here and takes precedence if it exists
  char *memstr;                 // Memory name for -U
  int op;                       // Symbolic memory operation DEVICE_... for -U
  char *filename;               // Filename for -U, can be -
  int format;                   // File format FMT_...
} UPDATE;

typedef struct {                // File reads for flash can exclude trailing 0xff, which are cut off
  int nbytes,                   // Number of bytes set including 0xff but excluding cut off, trailing 0xff
   nsections,                   // Number of consecutive sections in source excluding cut off, trailing 0xff
   npages,                      // Number of memory pages needed excluding pages solely with trailing 0xff
   nfill,                       // Number of fill bytes to make up full pages that are needed
   ntrailing,                   // Number of trailing 0xff in source
   firstaddr,                   // First address set in [0, mem->size-1]
   lastaddr;                    // Highest address set by input file
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
  int memstats_mem(const AVRPART *p, const AVRMEM *mem, int size, Filestats *fsp);

  // Helper functions for dry run to determine file access
  int update_is_okfile(const char *fn);
  int update_is_writeable(const char *fn);
  int update_is_readable(const char *fn);

  int update_dryrun(const AVRPART *p, UPDATE *upd);

  AVRMEM **memory_list(const char *mstr, const PROGRAMMER *pgm, const AVRPART *p,
    int *np, int *rwvsoftp, int *dry);
  int memlist_contains_flash(const char *mstr, const AVRPART *p);

#ifdef __cplusplus
}
#endif

// Formerly pgm_type.h

typedef struct programmer_type {
  const char *const id;
  void (*initpgm)(PROGRAMMER *pgm);
  const char *const desc;
} PROGRAMMER_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

  const PROGRAMMER_TYPE *locate_programmer_type(const char *id);
  const char *locate_programmer_type_id(void (*initpgm)(PROGRAMMER *pgm));
  typedef void (*walk_programmer_types_cb)(const char *id, const char *desc, void *cookie);
  void walk_programmer_types(walk_programmer_types_cb cb, void *cookie);

#ifdef __cplusplus
}
#endif

// Formerly config.h

extern LISTID part_list;
extern LISTID programmers;
extern const char *avrdude_conf_version;
extern const char *default_programmer;
extern const char *default_parallel;
extern const char *default_serial;
extern const char *default_spi;
extern int default_baudrate;
extern double default_bitclock;
extern char const *default_linuxgpio;
extern int allow_subshells;

// This name is fixed, it's only here for symmetry with default_parallel and default_serial
#define DEFAULT_USB       "usb"

#ifdef __cplusplus
extern "C" {
#endif

  void *cfg_malloc(const char *funcname, size_t n);
  void *cfg_realloc(const char *funcname, void *p, size_t n);
  char *cfg_strdup(const char *funcname, const char *s);
  void mmt_f_free(void *ptr);
  int init_config(void);
  void cleanup_config(void);
  int read_config(const char *file);
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
#define STR_INT8   (STR_1 | STR_SIGNED)   // Signed domain [-2^(n-1), 2^(n-1)-1]
#define STR_INT16  (STR_2 | STR_SIGNED)
#define STR_INT32  (STR_4 | STR_SIGNED)
#define STR_INT64  (STR_8 | STR_SIGNED)
#define STR_UINT8  (STR_1 | STR_UNSIGNED) // Unsigned domain [0, 2^n-1]; -u is same as 2^n-u
#define STR_UINT16 (STR_2 | STR_UNSIGNED) // Unsigned number u is deemed out of range
#define STR_UINT32 (STR_4 | STR_UNSIGNED) // If both u and -u are outside unsigned domain
#define STR_UINT64 (STR_8 | STR_UNSIGNED)
#define STR_XINT8   STR_1  // Unspecified signedness: numbers can occupy the asymmetric union
#define STR_XINT16  STR_2  // of signed domain and unsigned domain: [-2^(n-1), 2^n-1]
#define STR_XINT32  STR_4
#define STR_XINT64  STR_8

// AVR opcodes and disassembly

typedef enum {
  OP_AVRe,
  OP_AVRxm,
  OP_AVRxt,
  OP_AVRrc,
  OP_AVR_cycle_N
} AVR_cycle_index;

typedef struct {
  // Flags how to display lines
  int gcc_source, addresses, opcode_bytes, comments, sreg_flags, cycles;
  int op_names, op_explanations, avrgcc_style, labels;
  int avrlevel;                 // Eg, PART_AVR_XM or PART_AVR_51 (describes opcodes for the part)
  char *tagfile;                // Maps addresses to labels, PGM data, memory and I/O variables
} Dis_options;

typedef struct {
  int from, to, mnemo, labelno, is_func;
} Dis_jumpcall;

typedef struct {
  char *name, *comment;
  int address;
  int type;                     // I: I/O vars, M: mem vars, L: labels, P: PGM vars
  int subtype;                  // B: byte, W: word, A: autoterminated string, S: string
  int count;                    // Array length for tag file variables
  int used;                     // Whether symbol was referenced by disassembly process
  int printed;                  // Whether this L/P label will be printed in pass 2
} Dis_symbol;

// Order of enums must align with avr_opcodes[] table order
typedef enum {
  MNEMO_NONE = -1,
  MNEMO_lsl,      MNEMO_add,      MNEMO_rol,      MNEMO_adc,
  MNEMO_ror,      MNEMO_asr,      MNEMO_adiw,     MNEMO_sub,
  MNEMO_subi,     MNEMO_sbc,      MNEMO_sbci,     MNEMO_sbiw,
  MNEMO_tst,      MNEMO_and,      MNEMO_andi,     MNEMO_cbr,
  MNEMO_or,       MNEMO_ori,      MNEMO_sbr,      MNEMO_clr,
  MNEMO_eor,      MNEMO_com,      MNEMO_neg,      MNEMO_inc,
  MNEMO_dec,      MNEMO_mul,      MNEMO_muls,     MNEMO_mulsu,
  MNEMO_fmul,     MNEMO_fmuls,    MNEMO_fmulsu,   MNEMO_des,
  MNEMO_rjmp,     MNEMO_ijmp,     MNEMO_eijmp,    MNEMO_jmp,
  MNEMO_rcall,    MNEMO_icall,    MNEMO_eicall,   MNEMO_call,
  MNEMO_ret,      MNEMO_reti,     MNEMO_cpse,     MNEMO_cp,
  MNEMO_cpc,      MNEMO_cpi,      MNEMO_sbrc,     MNEMO_sbrs,
  MNEMO_sbic,     MNEMO_sbis,     MNEMO_brcs,     MNEMO_brlo,
  MNEMO_breq,     MNEMO_brmi,     MNEMO_brvs,     MNEMO_brlt,
  MNEMO_brhs,     MNEMO_brts,     MNEMO_brie,     MNEMO_brbs,
  MNEMO_brcc,     MNEMO_brsh,     MNEMO_brne,     MNEMO_brpl,
  MNEMO_brvc,     MNEMO_brge,     MNEMO_brhc,     MNEMO_brtc,
  MNEMO_brid,     MNEMO_brbc,     MNEMO_mov,      MNEMO_movw,
  MNEMO_ser,      MNEMO_ldi,      MNEMO_lds,      MNEMO_ld_x,
  MNEMO_ld_xp,    MNEMO_ld_mx,    MNEMO_ld_y,     MNEMO_ld_yp,
  MNEMO_ld_my,    MNEMO_ldd_y,    MNEMO_ld_z,     MNEMO_ld_zp,
  MNEMO_ld_mz,    MNEMO_ldd_z,    MNEMO_sts,      MNEMO_st_x,
  MNEMO_st_xp,    MNEMO_st_mx,    MNEMO_st_y,     MNEMO_st_yp,
  MNEMO_st_my,    MNEMO_std_y,    MNEMO_st_z,     MNEMO_st_zp,
  MNEMO_st_mz,    MNEMO_std_z,    MNEMO_lpm_0,    MNEMO_lpm_z,
  MNEMO_lpm_zp,   MNEMO_elpm_0,   MNEMO_elpm_z,   MNEMO_elpm_zp,
  MNEMO_spm,      MNEMO_spm_zp,   MNEMO_in,       MNEMO_out,
  MNEMO_push,     MNEMO_pop,      MNEMO_xch,      MNEMO_las,
  MNEMO_lac,      MNEMO_lat,      MNEMO_lsr,      MNEMO_swap,
  MNEMO_sbi,      MNEMO_cbi,      MNEMO_bst,      MNEMO_bld,
  MNEMO_sec,      MNEMO_clc,      MNEMO_sen,      MNEMO_cln,
  MNEMO_sez,      MNEMO_clz,      MNEMO_sei,      MNEMO_cli,
  MNEMO_ses,      MNEMO_cls,      MNEMO_sev,      MNEMO_clv,
  MNEMO_set,      MNEMO_clt,      MNEMO_seh,      MNEMO_clh,
  MNEMO_bset,     MNEMO_bclr,     MNEMO_break,    MNEMO_nop,
  MNEMO_sleep,    MNEMO_wdr,      MNEMO_lds_rc,   MNEMO_sts_rc,
  MNEMO_u_nop_1,  MNEMO_u_nop_2,  MNEMO_u_nop_3,  MNEMO_u_nop_4,
  MNEMO_u_nop_5,  MNEMO_u_nop_6,  MNEMO_u_nop_7,  MNEMO_u_icall,
  MNEMO_u_eicall, MNEMO_u_ret,    MNEMO_u_reti,   MNEMO_u_nop_8,
  MNEMO_u_nop_9,  MNEMO_u_nop_a,  MNEMO_u_ijmp,   MNEMO_u_eijmp,
  MNEMO_u_bld,    MNEMO_u_bst,    MNEMO_u_sbrc,   MNEMO_u_sbrs,
  MNEMO_N
} AVR_mnemo;

typedef enum {
  OP_AVR_RC = 1,                // Reduced-core Tiny only (128 byte STS/LDS)
  OP_AVR1 = 2,                  // All AVR can run this OPCODE
  OP_AVR1nRC = 4,               // All except reduced-core Tiny (TPI) and AT90S1200
  OP_AVR2 = 8,                  // AVR with archnum 2 and above
  OP_AVR2nRC = 16,              // AVR with archnum 2+ but not reduced-core Tiny
  OP_AVR25 = 32,                // AVR with archnum 25 and above
  OP_AVR_M = 64,                // AVR with flash > 8 kB or archnum 3+ (JMP, CALL)
  OP_AVR4 = 128,                // AVR with archnum 4 and above
  OP_AVR_L = 256,               // AVR with flash > 64 kB (ELMP)
  OP_AVR_XL = 512,              // AVR with flash > 128 kB (EIJMP, EICALL)
  OP_AVR_XM = 1024,             // XMEGA only (DES, XCH, LAC, LAS, LAT)
  OP_AVR_XTM = 2048,            // XMEGA and UPDI only (SPM Z+)
  OP_AVR_ILL = 4096,            // Undocumented (illegal) opcodes
} AVR_archlevel;

/*
 * Approximation(!) of which opcodes a part may have given its archnum
 *   - Take with a pinch of salt
 *   - For OP_AVR_XT and OP_AVR_XM one needs to add in OP_AVR_L/ OP_AVR_XL
 *     depending on flash size
 */
#define PART_AVR1   OP_AVR1
#define PART_AVR_RC (OP_AVR1|OP_AVR2|OP_AVR_RC)
#define PART_AVR2   (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC)
#define PART_AVR25  (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25)
#define PART_AVR3   (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR_M)
#define PART_AVR31  (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR_M)
#define PART_AVR4   (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR4|OP_AVR_M)
#define PART_AVR5   (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR4|OP_AVR_M)
#define PART_AVR51  (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR4|OP_AVR_M|OP_AVR_L)
#define PART_AVR6   (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR4|OP_AVR_M|OP_AVR_L|OP_AVR_XL)
#define PART_AVR_XT (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR4|OP_AVR_M|OP_AVR_XTM)
#define PART_AVR_XM (OP_AVR1|OP_AVR1nRC|OP_AVR2|OP_AVR2nRC|OP_AVR25|OP_AVR4|OP_AVR_M|OP_AVR_XM|OP_AVR_XTM)
#define PART_ALL    (PART_AVR_XM|OP_AVR_L|OP_AVR_XL)    // All but RC (the latter conflicts)

// Opcode types
#define OTY_REG_MASK          7 // Register formula mask
#define OTY_RNONE             0 // No registers addressed in this opcode
#define OTY_RALL              1 // Opcode can use all 32 registers (both Rd, Rr)
#define OTY_REVN              2 // Opcode only uses even registers (Rd *= 2, Rr *= 2)
#define OTY_RUPP              3 // Opcode only uses upper registers (Rd += 16, Rr += 16)
#define OTY_RW24              4 // Opcode only uses r24, r26, r28, r30 (Rd = Rd *2 + 24)

#define OTY_EXTERNAL      0x008 // Opcode might r/w either I/O region or memory

#define OTY_TYPE_MASK      0x78 // OPCODE type mask
#define OTY_ITYPE_MASK     0x70 // OPCODE type mask matching OTY_xxxI types
#define OTY_MCUI           0x00 // nop and wdr
#define OTY_MCUX           0x08 // sleep and break
#define OTY_ALBI           0x10 // Arithmetic, logic or bitwise operation
#define OTY_ALBX           0x18 // Arithmetic, logic or bitwise operation (external)
#define OTY_XFRI           0x20 // Data transfer (only affecting registers)
#define OTY_XFRX           0x28 // Data transfer (between external I/O or memory and regs)
#define OTY_JMPI           0x30 // Jump to potentially anywhere in flash (jmp, ijmp, eijmp)
#define OTY_JMPX           0x38 // Jump to potentially anywhere in flash (calls and ret/i)
#define OTY_RJMI           0x40 // Relative jump rjmp, range [.-4096, .+4094] bytes
#define OTY_RJMX           0x48 // Relative call rcall, range [.-4096, .+4094] bytes
#define OTY_BRAI           0x50 // Conditional branch, range [.-128, .+126] bytes
#define OTY_SKPI           0x60 // Conditional skip, range [.+0, .+4] (cpse, sbrc, sbrs)
#define OTY_SKPX           0x68 // Conditional skip, range [.+0, .+4] (sbic, sbis)

#define OTY_ZWORD         0x080 // Opcode uses Z register for word address (ijmp, ical etc)
#define OTY_ALIAS         0x100 // Opcode is a strict alias for another one, eg, sbr == ori
#define OTY_CONSTRAINT    0x200 // Opcode has constraints: Rr == Rd (tst, clr, lsl, rol)

#define OTY_WARN_MASK     0xc00 // OPCODE warning mask
#define OTY_XWRN          0x400 // Operand register must not be r27/r28
#define OTY_YWRN          0x800 // Operand register must not be r29/r30
#define OTY_ZWRN          0xc00 // Operand register must not be r29/r30

typedef struct {
  AVR_mnemo mnemo;              // Eg, MNEMO_add
  const char *idname;           // Unique id, eg, "ldx_1" (for error msgs or debugging)
  int mask, value, nwords;
  AVR_archlevel avrlevel;       // OP_AVR1
  const char *bits;             // "0000 11rd  dddd rrrr"
  int type;                     // OTY_ALBI|OTY_RALL
  const char
  *opcode,                      // "add"
  *operands,                    // "Rd, Rr"
  *description,                 // "add without carry"
  *operation,                   // "Rd <-- Rd + Rr"
  *flags,                       // "--HSVNZC"
  *clock[OP_AVR_cycle_N],       // Timings for AVRe, AVRxm, AVRxt and AVRrc
  *remarks;
} AVR_opcode;

extern const AVR_opcode avr_opcodes[164];

#ifdef __cplusplus
extern "C" {
#endif

  int avr_locate_upidx(const AVRPART *p);
  const Avrintel *avr_locate_uP(const AVRPART *p);
  const Configitem *avr_locate_configitems(const AVRPART *p, int *ncp);
  const char *const *avr_locate_isrtable(const AVRPART *p, int *nip);
  const Register_file *avr_locate_register_file(const AVRPART *p, int *nrp);
  const Register_file *avr_locate_register(const Register_file *rgf, int nr, const char *reg,
    int (*match)(const char *, const char *));
  const Register_file **avr_locate_registerlist(const Register_file *rgf, int nr, const char *reg,
    int (*match)(const char *, const char *));
  const Configitem *avr_locate_config(const Configitem *cfg, int nc, const char *name,
    int (*match)(const char *, const char *));
  const Configitem **avr_locate_configlist(const Configitem *cfg, int nc, const char *name,
    int (*match)(const char *, const char *));
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
  int str_is_in_list(const char *s, const char **l, size_t nl, int (*f)(const char *, const char *));
  char *str_sprintf(const char *fmt, ...)
#if defined(__GNUC__)           // Ask gcc to check whether format and parameters match
    __attribute__((format(printf, 1, 2)))
#endif
    ;
  const char *str_ccprintf(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;
  const char *str_ccstrdup(const char *str);
  char *str_fgets(FILE *fp, const char **errpp);
  size_t str_numc(const char *str, char c);
  const char *str_ltrim(const char *s);
  char *str_nrtrim(char *s, size_t n);
  char *str_rtrim(char *s);
  char *str_ntrim(char *s, size_t n);
  char *str_trim(char *s);
  char *str_lc(char *s);
  char *str_uc(char *s);
  char *str_lcfirst(char *s);
  char *str_ucfirst(char *s);
  char *str_asciiname(char *s);
  char *str_utoa(unsigned n, char *buf, int base);
  char *str_endnumber(const char *str);
  const char *str_plural(int x);
  const char *str_inname(const char *fn);
  const char *str_infilename(const char *fn);
  const char *str_outname(const char *fn);
  const char *str_outfilename(const char *fn);
  const char *str_ccinterval(int a, int b);
  bool is_bigendian(void);
  void change_endian(void *p, int size);
  int is_memset(const void *p, char c, size_t n);
  unsigned long long int str_ull(const char *str, char **endptr, int base);
  int looks_like_number(const char *str);
  Str2data *str_todata(const char *str, int type, const AVRPART *part, const char *memstr);
  void str_freedata(Str2data *sd);
  unsigned long long int str_int(const char *str, int type, const char **errpp);
  int str_membuf(const char *str, int type, unsigned char *buf, int size, const char **errpp);
  char *str_nexttok(char *buf, const char *delim, char **next);
  const char *str_ccfrq(double f, int n);
  const char *str_cchex(const void *buf, size_t len, int add_space);
  int str_levenshtein(const char *str1, const char *str2, int swap, int subst, int add, int del);
  size_t str_weighted_damerau_levenshtein(const char *str1, const char *str2);
  int str_mcunames_signature(const unsigned char *sigs, int pm, char *p, size_t n);
  const char *str_ccmcunames_signature(const unsigned char *sigs, int pm);
  const char *str_ccpgmids(LISTID pgm_id);
  const char *str_ccaddress(int addr, int size);
  char *str_quote_bash(const char *s);
  const char *str_ccsharg(const char *str);

  int led_set(const PROGRAMMER *pgm, int led);
  int led_clr(const PROGRAMMER *pgm, int led);
  int led_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
  int led_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned long addr, unsigned char value);
  int led_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned long addr, unsigned char *value);
  int led_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned int page_size, unsigned int addr, unsigned int n);
  int led_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
    unsigned int page_size, unsigned int addr, unsigned int n);
  int led_page_erase(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned int addr);

  int terminal_mode(const PROGRAMMER *pgm, const AVRPART *p);
  int terminal_mode_noninteractive(const PROGRAMMER *pgm, const AVRPART *p);
  int terminal_line(const PROGRAMMER *pgm, const AVRPART *p, const char *line);
  char *terminal_get_input(const char *prompt);
  void terminal_setup_update_progress(void);

  char *avr_cc_buffer(size_t n);

  int op16_is_mnemo(int op16, AVR_mnemo mnemo);
  int is_opcode32(int op16);
  int op_width(int op16);
  int ldi_Rd(int op16);
  int ldi_K(int op16);
  AVR_mnemo opcode_mnemo(int op16, int avrlevel);
  int op16_is_valid(int op16, int avrlevel);
  int op16_is_benign(int op16, int avrlevel);
  int avr_get_archlevel(const AVRPART *p);
  AVR_cycle_index avr_get_cycle_index(const AVRPART *p);
  const char *mnemo_str(int op16);
  int z_width(int op16, AVR_mnemo *mnenop);
  int op16_target(int here, int op16);
  int dist2rjmp(int dist);

  int disasm(const char *buf, int len, int addr, int leadin, int leadout);
  int disasm_init(const AVRPART *p);
  int disasm_init_tagfile(const AVRPART *p, const char *file);
  void disasm_zap_jumpcalls();

#ifdef __cplusplus
}
#endif

/*
 * Context structure
 *
 * Global and static variables should go here; the only remaining static
 * variables ought to be read-only tables. Access should be via a global
 * pointer libavrdude_context *cx; applications using libavrdude ought to
 * allocate cx = mmt_malloc(sizeof *cx) for each instantiation (and set initial
 * values if needed) and deallocate with mmt_free(cx).
 */

typedef struct {
  // Closed-circuit space for returning strings in a persistent buffer
#define AVR_SAFETY_MARGIN 1024
  char *avr_s, avr_space[32768 + AVR_SAFETY_MARGIN];

  // Static variables from avr.c
  int avr_disableffopt;         // Disables trailing 0xff flash optimisation
  uint64_t avr_epoch;           // Epoch for avr_ustimestamp()
  int avr_epoch_init;           // Whether above epoch is initialised
  int avr_last_percent;         // Last valid percentage for report_progress()
  double avr_start_time;        // Start time in s of report_progress() activity

  // Static variables from bitbang.c
  int bb_delay_decrement;

#if defined(WIN32)
  int bb_has_perfcount;
  uint64_t bb_freq;             // Should be LARGE_INTEGER but what to include?
#else
  int bb_done;                  // Handshake variable in alarm handler
  void (*bb_saved_alarmf)(int); // Saved alarm handler
#endif

  // Static variables from config.c
  char **cfg_hstrings[1 << 12]; // Hash lists for cache_string()
  LISTID cfg_comms;             // A chain of comment lines
  LISTID cfg_prologue;          // Comment lines at start of avrdude.conf
  char *cfg_lkw;                // Last seen keyword
  int cfg_lkw_lineno;           // Line number of that
  LISTID cfg_strctcomms;        // Passed on to config_gram.y
  LISTID cfg_pushedcomms;       // Temporarily pushed main comments
  int cfg_pushed;               // ... for memory sections
  int cfg_init_search;          // Used in cfg_comp_search()

  // Static variable from dfu.c
  uint16_t dfu_wIndex;          // A running number for USB messages

  // Static variable from config_gram.y
  int cfgy_pin_name;            // Temporary variable for grammar parsing

  // Static variable from ppi.c
  unsigned char ppi_shadow[3];

  // Static variables from ser_avrdoper.c
  unsigned char sad_avrdoperRxBuffer[280];      // Buffer for receiving data
  int sad_avrdoperRxLength;     // Amount of valid bytes in rx buffer
  int sad_avrdoperRxPosition;   // Amount of bytes already consumed in rx buffer

  // Static variables from ser_win32.c/ser_posix.c

#if defined(WIN32)
  unsigned char ser_serial_over_ethernet;
#else
  struct termios ser_original_termios;
  int ser_saved_original_termios;
#endif

  // Static variables from term.c
  int term_spi_mode;
  struct mem_addr_len {
    const AVRMEM *mem;
    int addr, len;
  } term_rmem[32];
  int term_mi;
  const PROGRAMMER *term_pgm;
  const AVRPART *term_p;
  int term_running;
  char *term_header;
  int term_tty_last, term_tty_todo;
  int term_notty_last, term_notty_todo;

  // Static variables from update.c
  const char **upd_wrote, **upd_termcmds;
  int upd_nfwritten, upd_nterms;

  // Static variable from fileio.c
  int reccount;

  // Static variables from disasm.c
  int dis_initopts, dis_flashsz, dis_flashsz2, dis_addrwidth, dis_sramwidth;
  int dis_pass, dis_para, dis_cycle_index, dis_io_offset, dis_codewidth;
  Dis_options dis_opts;
  int dis_jumpcallN, dis_symbolN, *dis_jumpable, dis_start, dis_end;
  Dis_jumpcall *dis_jumpcalls;
  Dis_symbol *dis_symbols;

  // Static variables from usb_libusb.c
#include "usbdevs.h"
  char usb_buf[USBDEV_MAX_XFER_3];
  int usb_buflen, usb_bufptr;   // @@@ Check whether usb_buflen needs initialising with -1
  int usb_interface;

  // Variable connecting lexer.l and config_gram.y
  int lex_kw_is_programmer;     // Was the K_PROGRAMMER keyword "programmer"?

  // Global variable indicating usb access problems
  int usb_access_error;
} libavrdude_context;

extern libavrdude_context *cx;

// Formerly confwin.h

#if defined(WIN32)

#ifdef __cplusplus
extern "C" {
#endif

  int win_set_path(char *path, int n, const char *file);

#ifdef __cplusplus
}
#endif
#endif                          // WIN32

#ifndef TO_BE_DEPRECATED_IN_2026

/*
 * AVRDUDE type names ending in _t have been renamed, as POSIX reserves
 * all of these. Below typedefs that give access to some of these _t names
 * but will be withdrawn in future. If you want to update your project
 * code that uses libavrdude feel free to copy and paste the lines below
 * into a file avrdude_t.sed and execute in your code directory
 *
 * $ sed -i -f avrdude_t.sed *.{c,h,cpp,hpp,l,y}
 *

s/\btypedef struct programmer_t\b/typedef struct programmer/g
s/\bstruct programmer_t\b/PROGRAMMER/g
s/\bprogrammer_t\b/programmer/g
s/\bprogrammer_type_t\b/programmer_type/g
s/\bctl_stack_t\b/ctl_stack/g
s/\bpindef_t\b/pindef/g
s/\bpin_checklist_t\b/Pin_checklist/g
s/\bpinmask_t\b/Pinmask/g
s/\bupdate_t\b/update/g
s/\buPcore_t\b/Avrintel/g
s/\bComponent_t\b/Component/g
s/\bValueitem_t\b/Configvalue/g
s/\bConfigitem_t\b/Configitem/g
s/\bRegister_file_t\b/Register_file/g
s/\bconntype_t\b/Conntype/g
s/\bexit_datahigh_t\b/Exit_datahigh/g
s/\bexit_reset_t\b/Exit_reset/g
s/\bexit_vcc_t\b/Exit_vcc/g
s/\bleds_t\b/Leds/g
s/\bmemtable_t\b/Memtable/g
s/\bmemtype_t\b/Memtype/g
s/\bSegment_t\b/Segment/g

 *
 */ typedef Configvalue Valueitem_t;
typedef Configitem Configitem_t;
typedef Register_file Register_file_t;
typedef Avrintel uPcore_t;

typedef Pin_checklist pin_checklist_t;
typedef Pinmask pinmask_t;
typedef Conntype conntype_t;
typedef Exit_datahigh exit_datahigh_t;
typedef Exit_reset exit_reset_t;
typedef Exit_vcc exit_vcc_t;
typedef Leds leds_t;
typedef Memtable memtable_t;
typedef Memtype memtype_t;
typedef Segment Segment_t;
#endif

#endif
