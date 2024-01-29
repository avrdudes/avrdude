/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022 Stefan Rueger <stefan.rueger@urclocks.com>
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

/*
 * The Urclock programmer
 *
 *  - Reads/writes flash/EEPROM of boards directly via the MCU bootloader and a serial connection
 *  - Automatically resets an attached board via RTS/DTR into bootloader mode
 *  - Works best in tandem with the urboot bootloader, but can deal with optiboot and similar
 *  - Implements urprotocol, a communication protocol designed for small bootloader sizes
 *  - Supports vector bootloaders by patching relevant interrupt vectors during upload:
 *     + Vector bootloaders run on all devices, not only those with a dedicated boot section
 *     + Can be considerably smaller than the smallest dedicated boot section of a part, eg,
 *       only 256 bytes for ATmega2560 with an otherwise smallest boot section of 1024 bytes
 *  - Checks sizes of applications so they don't overwrite the bootloader
 *  - Keeps the bootloader alive during interactive terminal sessions
 *  - Provides a 4-byte metadata interface in top flash for
 *     + Allowing applications to utilise unused flash in a similar fashion to EEPROM
 *     + Storing in top flash the file name and last-modified-date of the uploaded application
 *     + Displaying file name and date of the application that was last uploaded
 *
 * As an example, the urboot bootloader including EEPROM r/w for the popular ATmega328p is only 384
 * bytes, which frees up 128 bytes. On an ATmega1284p the urboot bootloader without EEPROM r/w is
 * only 256 bytes, freeing 786 bytes on that device. Urboot bootloaders can be configured to
 *  - Upload and download applications
 *  - Read and write EEPROM
 *  - Provide an application function that writes flash memory pages; as this function is located
 *    at FLASHEND-4+1, no linker information is needed for the application
 *  - Operate dual boot from external SPI flash memory in addition to EEPROM r/w at a slightly
 *    increased bootloader of 512 bytes for a range of devices from the small ATtiny167, via the
 *    popular ATmega328p to the mighty ATmega2560.
 *
 *
 * Urprotocol (the gory details, see also https://github.com/stefanrueger/urboot)
 *
 * The **explicit communication** between an uploader/downloader program (*"the programmer"*) and
 * the bootloader is driven by the programmer, which sends command sequences to the bootloader and
 * evaluates their return sequences. A command sequence starts by a command byte, followed by its
 * parameters, followed by an end-of-parameter byte UR_EOP. In return the bootloader sends a fixed
 * byte UR_INSYNC to acknowledge the command, then executes it, possibly returning data, followed
 * by sending a different fixed byte UR_OK.
 *
 * Although the UR_INSYNC and UR_OK are *fixed constants* for a particular bootloader, they *can
 * vary* between bootloaders to indicate
 *  - Which MCU the bootloader sits on (using one of up to 2040 predefined different MCU IDs)
 *  - Whether or not the bootloader provides a paged read flash command
 *  - Whether or not the bootloader has implemented the chip erase command
 *  - Whether or not writing a memory page memory looks like programming NOR memory
 *  - Two more whether-or-not bits that are currently reserved
 *
 * As UR_INSYNC and UR_OK should always differ, there are 256*255 possible combinations, one of
 * which is reserved for backward compatibility mode where UR_INSYNC and UR_OK coincide with the
 * respective STK500v1 constants. This protocol definition enables the bootloader to pass to the
 * programmer log2(256*255-1) bits = 15.994331... bits of configuration information without having
 * to spend a single additional byte of bootloader code. Subtracting the 5 bits for the "whether or
 * not" info leaves 10.994331... bits which allows 2040 ≈ 2**10.994331... MCU ids.
 *
 * **Parameters.** Paged EEPROM/flash access commands and page erase are the only commands that
 * need parameters. In this case the parameters are the address, followed by the length of the
 * block to read or write and, if needed, followed by the bytes to be written. As in STK500v1,
 * addresses are given as little endian (low byte first) and length as big endian (high byte
 * first). The address always is a byte address (unless in compability mode). It is a 16-bit
 * address for MCUs that have 65536 bytes flash or less, and a 24-bit address for MCUs with larger
 * flash. Zero-length reads or  writes are not supported by the protocol. If the *flash* page size
 * is 256 or less, then the length parameter is sent as one byte (where 0 means 256 bytes).
 * Otherwise the length parameter is sent as two bytes (where 0 means 65536). Note, however, that
 * the only valid length for the write flash page command is the MCU page size; also the *maximum*
 * valid length for EEPROM writes is 256 or the *flash* page size, whichever is higher. EEPROM
 * write page commands should never exceed the size of half of SRAM though. The other two (read)
 * paged-access commands are free to request any length between 1 and 256, and 1 and 65536,
 * respectively. However, the programmer must never ask for an address block that would access
 * bytes outside the range of EEPROM or flash on the device. Whilst  the number of parameter bytes
 * differs between bootloaders, for a particular bootloader the address and length is given always
 * in the same way. This means that the EEPROM address on an MCU with a  large flash will be a
 * 24-bit address even though the EEPROM might only have 8192 bytes. Even though the write flash
 * page command only allows one length, and page erase does not need a page at all, it must always
 * be specified. This is to simplify the bootloader effort to decode the programmer's commands.
 *
 *
 * Urprotocol commands
 *
 *  - **UR_GET_SYNC:** The bootloader does nothing except returning the two protocol bytes. Its
 *    purpose is to synchronise the programmer with the bootloader and to identify the type of
 *    bootloader and (some of) its properties. For synchronisation, the programmer should issue a
 *    number of UR_GET_SYNC commands until it receives consistent UR_INSYNC and UR_OK values.
 *    At this point the programmer knows whether or not to switch to backward compatibility mode
 *    using the STK500v1 protocol as in -c arduino, which MCU is to be programmed etc. It is
 *    advised the programmer sets its read timeout in the synchronisation phase to less than 100 ms
 *    when reading the bootloader reply to avoid triggering the bootloader's watchdog timer. It is
 *    also recommended that the input is "drained" after successfully reading two response bytes to
 *    ensure the response has not been brought about by an application program of the connected
 *    board before the board was reset into bootloader mode. This command can also be used
 *    periodically to prevent the bootloader from timing out.
 *
 *  - **UR_PROG_PAGE_FL:** One flash page is written to the device. In the absence of a
 *    UR_CHIP_ERASE (see below), the bootloader is expected to program the flash page as atomic
 *    page erase, page load and page write. If the bootloader implements UR_CHIP_ERASE, it has the
 *    choice of erasing a flash page before programming it or not. In case the bootloader erases
 *    pages before writing them, the payload of the UR_PROG_PAGE is programmed exactly as is; the
 *    programmer should implement desired sub-page modifications by first reading the flash
 *    contents of the not-to-be-modified page parts to correctly pad the page payload. If the
 *    bootloader does not erase pages before writing them, effectively the payload is *and*ed to
 *    the existing contents of the page thereby exposing the physical property of the underlying
 *    NOR flash memory; sub-page modifications can be carried out by padding the page buffer
 *    payload with 0xff, as programming 0xff is a NOP for AVR NOR flash memories.
 *
 *  - **UR_CHIP_ERASE** (optional): If implemented, the bootloader erases to 0xff all flash
 *    except itself. After issuing the chip erase request it is advised the programmer set its
 *    timeout for reading the next character to more time than the bootloader will need to erase
 *    flash to avoid the programmer resuming communication before the bootloader comes back from
 *    the chip erase. 20 s should be sufficient. If the bootloader does not implement chip erase
 *    then the programmer should ensure that flash is erased to 0xff by, eg, repeated
 *    UR_PROG_PAGE calls with 0xff-only contents or equivalent; this normally takes longer than
 *    bootloader chip erase but is otherwise functionally equivalent to a UR_CHIP_ERASE
 *    implementation in the bootloader. The protocol does not expect EEPROM to be erased in either
 *    case. However, when implementing UR_CHIP_ERASE the bootloader is free to read fuses to
 *    determine whether or not EEPROM should also be erased and erase EEPROM accordingly.
 *
 *  - **UR_READ_PAGE_FL** (optional) returns n=length bytes of flash from the given address
 *
 *  - **UR_READ_PAGE_EE** (optional) returns n=length bytes of EEPROM from the given address
 *
 *  - **UR_PROG_PAGE_EE** (optional) writes n=length bytes to the EEPROM at the given address
 *
 *  - **UR_PAGE_ERASE** (optional) erases to 0xff a page at the given address (length must be given
 *    but is ignored)
 *
 *  - **UR_LEAVE_PROGMODE** (optional): some bootloaders reduce the Watchdog timeout so that the
 *    application is started faster after programming
 *
 *  - **Any other command**, should behave like UR_GET_SYNC, ie, the bootloader returns
 *    UR_INSYNC and UR_OK.
 *
 *
 * **Error handling.** It is generally considered an error if the programmer asks for not
 * implemented functionality, as it knows after synchronisation how the bootloader is configured.
 * Hence, the bootloader WDT should reset on request of an optional, not implemented command.
 * Typically, the bootloader would need to save the payload of EEPROM/flash writes to SRAM; for
 * security reasons the bootloader should trigger a WDT reset if an illegitimate length of a paged
 * write could overwrite the stack (eg, a request for writing 256 bytes EEPROM on a part with only
 * 256 bytes SRAM). A protocol error detected by the bootloader (failure to match UR_EOP) should
 * lead to a WDT reset. Protocol errors detected by the programmer (not matching UR_INSYNC or
 * UR_OK) should normally lead to a termination of programming attempts. Frame errors in serial
 * communication should also lead to a WDT reset or termination of programming, respectively. The
 * bootloader should protect itself from being overwritten through own page writes and page erases.
 *
 *
 * **Implicit communication** of further bootloader properties happens through a small table
 * located at the top of flash. Normally, the programmer can read this table after establishing the
 * MCU id, and therefore the location of top flash of the part for which the bootloader was
 * compiled. The 6-byte table contains (top to bottom):
 *   - Version number: one byte, minor version 0..7 in three lsb, major version 0..31 in the 5 msb
 *   - Capabilities byte detailing, eg, whether the bootloader supports EEPROM r/w, dual boot etc
 *   - Two-byte rjmp to a writepage(ram, flash) function or a ret opcode if not implemented
 *   - Number 1..127 of pages that the bootloader occupies
 *   - Vector number 1..127 used for the r/jmp to the application if it is a vector bootloader
 * If the bootloader does not have read capabilities the user needs to supply necessary information
 * such as the bootloader size to the programmer on the command line via -x extended parameters.
 *
 * **Backward compatibility mode.** When urprotocol after synchronisation with the bootloader
 * settles on UR_INSYNC and UR_OK values that turn out to be the STK500v1 values of 0x14 and 0x10,
 * this triggers a backward compatibility mode. In this instance the programmer behaves (almost)
 * like the STK500v1 implementation in avrdude's arduino programmer, ie, it handles optiboot and
 * legacy bootloaders gracefully: in particular, the programmer can issue STK_READ_SIGN and two
 * STK_UNIVERSAL requests (load extended address and chip erase) that the bootloader must implement
 * in the backward compatibility mode. All EEPROM/flash addresses are sent as two-byte word
 * addresses *little* endian, all length arguments are two-byte *big* endian, etc. Unlike avrdude
 * -c arduino the programmer for the urprotocol should not pass on get and set hardware parameter
 * requests, enquire software and hardware versions etc, as these requests would be wasteful for
 * the bootloader. Under the urprotocol, bootloaders should be assured they do not need to even
 * provide code to ignore these requests, even if they operate in the backwards compatibility mode.
 *
 *
 * **Limitations.** Urprotocol has only provisions for reading EEPROM and flash, for writing EEPROM
 * and for writing flash other than the bootloader area. In particular, urprotocol has no
 * provisions for reading other memories such as the signature (other than in backward
 * compatibility mode), calibration bytes, locks or fuses, and neither for writing lock bytes. The
 * protocol does not consider sub-page flash writes, which are shifted to the programmer. If the
 * bootloader's flash write does *not* look like NOR programming *and* if the bootloader does *not*
 * provide flash read, then sub-page modifications simply cannot be done. Installing a bootloader
 * has security implications as it provides a means to modify flash thus weakening the Harvard
 * architecture of AVR microprocessors. Even bootloader implementations that are hardened against
 * prohibited address and length parameters have, out of necessity, somewhere a code sequence that
 * manipulates flash memory. A flawed application might still give an attacker a way to call these
 * code sequences, so be warned here be dragons.
 *
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "urclock.h"
#include "urclock_private.h"

#define urmax(a, b) ((a) > (b)? (a): (b))
#define urmin(a, b) ((a) < (b)? (a): (b))

static int ur_initstruct(const PROGRAMMER *pgm, const AVRPART *p);
static int ur_readEF(const PROGRAMMER *pgm, const AVRPART *p, uint8_t *buf, uint32_t addr, int len,
  char memchr);
static int readUrclockID(const PROGRAMMER *pgm, const AVRPART *p, uint64_t *idp);
static int urclock_send(const PROGRAMMER *pgm, unsigned char *buf, size_t len);
static int urclock_recv(const PROGRAMMER *pgm, unsigned char *buf, size_t len);
static int urclock_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res);

static int urclock_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);


// Context of the programmer
typedef struct {
  char desc[32];                // Text description of bootloader version and capabilities

  bool urprotocol;              // Bootloader uses the urboot modification of the STK500v1 protocol
  uint8_t urfeatures;           // Bootloader features (chip erase, can read flash, ...)
  int STK_INSYNC, STK_OK;       // Variable but fixed bootloader responses for urprotocol

  struct {
    uint8_t seen, stk_ok, stk_insync;
  } gs;                         // Needed for urclock_getsync()


  unsigned char ext_addr_byte;  // Ext-addr byte for STK500v1 protocol and MCUs with > 128k

  uPcore_t uP;                  // Info about the connected processor (copied from uP_table)

  bool initialised;             // Is this structure initialised?
  bool bleepromrw;              // Bootloader has EEPROM r/w support
  bool emulate_ce;              // Emulate chip erase when bootloader cannot and user wants it
  bool done_ce;                 // Set when flash of chip has been erased after first write

  int sync_silence;             // Temporarily set during start of synchronisation

  // Info needed about bootloader to patch, if needed, the reset vector and one other vector
  int vblvectornum,             // Vector bootloader vector number for jump to application op code
      vbllevel,                 // 0=n/a, 1=patch externally, 2=bl patches, 3=bl patches & verifies
      blurversion,              // Octal byte 076 means v7.6 (minor version number is lowest 3 bit)
                                // Small numbers < 070 probably are optiboot major version number
      bloptiversion,            // Optiboot version as (major<<8) + minor
      blguessed;                // Guessed the bootloader from hash data

  int boothigh;                 // 1: Bootloader sits in high flash; 0: low flash (UPDI parts)
  int32_t blstart, blend;       // Bootloader address range [blstart, blend] for write protection
  int32_t pfstart, pfend;       // Programmable flash address range [pfstart, pfend]

  int idmchr;                   // Either 'E' or 'F' for the memory where the Urclock ID is located
  int idaddr;                   // The address of the Urclock ID
  int idlen;                    // Number 1..8 of Urclock ID bytes (location, see iddesc below)

  int32_t storestart;           // Store (ie, unused flash) start address, same as application size
  int32_t storesize;            // Store size

  // Metadata for free flash memory to be used for store support
  char filename[254];           // Filename of uploaded application, must be max 254 bytes incl nul
  int16_t  yyyy;                // Date stamp of uploaded application file: 4 digit year,
  int8_t mm, dd, hr, mn;        // Month (1..12), day (1..31), hour (0..23) and minute (0..59)
  uint8_t freeflash[3];         // 24-bit little endian number (storesize)
  uint8_t mcode;                // 255 = no metadata, 0 = only freeflash, 1 = freeflash + date,
                                // 2-254 = freeflash + date + that many bytes filename incl nul

  /*
   * Examples:
   *   blend-blstart+1 = bootloader size
   *   FLASHEND+1 = application size + freeflash + nmeta(mcode, flashsize) + bootloader size
   *   Note for "classic" parts the bootloader is in high flash: blend = FLASHEND
   *   blstart = application size + freeflash + nmeta(mcode, flashsize)
   *   For "modern" parts the bootloader is in low flash: blstart = 0
   */

  // Extended parameters for Urclock
  int showall,                  // Show all pieces of info for connected part and exit
      showid,                   //   ... Urclock ID
      showdate,                 //   ... last-modified date of last uploaded application
      showfilename,             //   ... filename of last uploaded application
      showapp,                  //   ... application size
      showstore,                //   ... store size
      showmeta,                 //   ... metadata size
      showboot,                 //   ... bootloader size
      showversion,              //   ... bootloader version and capabilities
      showvector,               //   ... vector bootloader level, vector number and name
      showpart,                 //   ... part for which bootloader was compiled
      xbootsize,                // Manual override for size of bootloader section
      xvectornum,               //   ... for vector number (implies vbllevel = 1)
      xeepromrw,                //   ... for EEPROM r/w capability
      xemulate_ce,              //   ... for making avrdude emulate any chip erase
      initstore,                // Zap store when writing the application, ie, fill with 0xff
//@@@ copystore,                // Copy over store as far as possible when writing the application
      restore,                  // Restore a flash backup exactly as it is trimming the bootloader
      nofilename,               // Don't store application filename when writing the application
      nodate,                   // Don't store application filename and no date either
      nostore,                  // Don't store metadata except a flag saying so
      nometadata,               // Don't support metadata at all
      delay,                    // Additional delay [ms] after resetting the board, can be negative
      strict;                   // Use strict synchronisation protocol

  char title[254];              // Use instead of filename for metadata - same size as filename
  char iddesc[64];              // Location of Urclock ID, eg F.12324.6 or E.-4.4 (default E.257.6)
} Urclock_t;

// Use private programmer data as if they were a global structure ur
#define ur (*(Urclock_t *)(pgm->cookie))

#define Return(...) do { pmsg_error(__VA_ARGS__); msg_error("\n"); return -1; } while (0)


// Return how many bytes metadata are needed given the mcode byte just below bootloader
static int nmeta(int mcode, int flashsize) {
  // The size of the structure that holds info about metadata (sits just below bootloader)
  int nheader = 2*(flashsize > (1<<16)? 4: 2) + 1;

  return mcode == 0xff? 1:      // No metadata except 0xff byte itself saying no further metadata
    mcode > 1? mcode+6+nheader: // Application filename, app date and structure for pgm store
    mcode? 6+nheader:           // Application date and structure describing pgm store
      nheader;                  // Structure describing pgm store
}


// Need to know a bit about avr opcodes, in particular jmp and rjmp for patching vector table

#define ret_opcode 0x9508


// Is the opcode an rjmp, ie, a relative jump [-4094, 4096] bytes from opcode address?
static int isRjmp(uint16_t opcode) {
  return (opcode & 0xf000) == 0xc000;
}


/*
 * Map distances to [-flashsize/2, flashsize/2) for smaller devices. As rjmp can go +/- 4 kB, so
 * smaller flash than 8k (eg, 4k) benefit from wrap around logic.
 */
static int rjmpdistwrap(int addis, int flashsize) {
  int size = flashsize > 8182? 8192: flashsize;

  if((size & (size-1)) == 0) {  // Sanity check to assert size is a power of 2; will be true
    addis &= size-1;
    if(addis >= size/2)
      addis -= size;
  }

  return addis;
}


// Compute from rjmp opcode the relative distance in bytes (rjmp address minus destination address)
static int dist_rjmp(uint16_t rjmp, int flashsize) {
  int16_t dist;

  dist = rjmp & 0xfff;          // Signed 12-bit word distance
  dist = (int16_t)(dist<<4)>>3; // Sign-extend and multiply by 2

  return rjmpdistwrap(dist+2, flashsize);  // Wraps around 0 (eg, in flashes smaller than 8k)
}


// rjmp opcode from byte distance; 0xcfff is an endless loop, 0xc000 is a nop
static uint16_t rjmp_opcode(int dist, int flashsize) {
  dist = rjmpdistwrap(dist, flashsize);
  return 0xc000 | (((dist >> 1) - 1) & 0x0fff);
}


// rjmp opcode from reset to bootloader start; same as above if bl start is in top half of flash
static uint16_t rjmp_bwd_blstart(int blstart, int flashsize) { // flashsize must be power of 2
  return 0xc000 | (((uint16_t)((blstart-flashsize-2)/2)) & 0xfff); // Urboot uses this formula
}


// jmp opcode from byte address
static uint32_t jmp_opcode(int32_t addr) {
  // jmp uses word address; hence, shift by that one extra bit more
  return (((addr>>1) & 0xffff)<<16) | 0x940c | (((addr>>18) & 31)<<4) | (((addr>>17) & 1)<<0);
}


// Byte address from jmp opcode
static int addr_jmp(uint32_t jmp) {
  int addr;

  addr  = jmp >> 16;            // Low 16 bit of word address are in upper word of op code
  addr |= (jmp & 1) << 16;      // Add extra address bits from least significant bytes of op code
  addr |= (jmp & 0x1f0) << (17-4);
  addr <<= 1;                   // Convert to byte address

  return addr;
}


// Is the instruction word the lower 16 bit part of a 32-bit instruction?
static int isop32(uint16_t opcode) {
  return
    (opcode & 0xfe0f) == 0x9200 || // sts
    (opcode & 0xfe0f) == 0x9000 || // lds
    (opcode & 0xfe0e) == 0x940c || // jmp
    (opcode & 0xfe0e) == 0x940e;   // call
}


// Is the instruction word the lower 16 bit part of a jmp instruction?
static int isJmp(uint16_t opcode) {
  return (opcode & 0xfe0e) == 0x940c;
}


// Assemble little endian 32-bit word from buffer
static uint32_t buf2uint32(const unsigned char *buf) {
  return buf[0] | buf[1]<<8 | buf[2]<<16 | buf[3]<<24;
}


// Assemble little endian 16-bit word from buffer
static uint16_t buf2uint16(const unsigned char *buf) {
  return buf[0] | buf[1]<<8;
}


// Write little endian 32-bit word into buffer
void uint32tobuf(unsigned char *buf, uint32_t opcode32) {
  buf[0] = opcode32;
  buf[1] = opcode32>>8;
  buf[2] = opcode32>>16;
  buf[3] = opcode32>>24;
}


// Write little endian 16-bit word into buffer
void uint16tobuf(unsigned char *buf, uint16_t opcode16) {
  buf[0] = opcode16;
  buf[1] = opcode16>>8;
}


// Set filename/title and date for metadata
static void set_date_filename(const PROGRAMMER *pgm, const char *fname) {
  const char *base;
  struct stat b;
  struct tm *t;
  time_t when;

  // Last modification date of file or, if unavailable, current time
  when = fname && *fname && !str_eq(fname, "-") && !stat(fname, &b)? b.st_mtime: time(NULL);
  when += 30;                   // Round to minute
  if((t=localtime(& when))) {
    ur.yyyy = t->tm_year + 1900;
    ur.mm = t->tm_mon+1;
    ur.dd = t->tm_mday;
    ur.hr = t->tm_hour;
    ur.mn = t->tm_min;
  }

  // Compute basename of file unless title was set
  if(*ur.title)
    memcpy(ur.filename, ur.title, sizeof ur.filename);
  else {
    ur.filename[0] = 0;
    if(fname && *fname) {
      if((base=strrchr(fname, '/')))
        base++;
#ifdef WIN32
      else if((base=strrchr(fname, '\\')))
        base++;
#endif
      else
        base = fname;
      strncpy(ur.filename, base, sizeof ur.filename-1);
      ur.filename[sizeof ur.filename-1] = 0;
    }
  }
}



// Put destination address of reset vector jmp or rjmp into addr, return -1 if not an r/jmp
static int reset2addr(const unsigned char *opcode, int vecsz, int flashsize, int *addrp) {
  int op32, addr, rc = 0;
  uint16_t op16;

  op16 = buf2uint16(opcode); // First word of the jmp or the full rjmp
  op32 = vecsz == 2? op16: buf2uint32(opcode);

  if(vecsz == 4 && isJmp(op16)) {
    addr = addr_jmp(op32);      // Accept compiler's destination (do not normalise)
  } else if(isRjmp(op16)) {     // rjmp might be generated for larger parts, too
    addr = dist_rjmp(op16, flashsize);
    while(addr < 0)             // If rjmp was backwards
      addr += flashsize;        // OK for small parts, likely(!) OK if flashsize is a power of 2
    while(addr > flashsize)     // Sanity (should not happen): rjmp jumps over FLASHEND
      addr -= flashsize;
  } else
    rc = -1;

  if(addrp && rc == 0)
    *addrp = addr;

  return rc;
}


// What reset looks like for vector bootloaders
static int set_reset(const PROGRAMMER *pgm, unsigned char *jmptoboot, int vecsz) {
  // Small part or larger flash that is power or 2: urboot P reset vector protection uses this
  if(vecsz == 2 || (ur.uP.flashsize & (ur.uP.flashsize-1)) == 0) {
    uint16tobuf(jmptoboot, rjmp_bwd_blstart(ur.blstart, ur.uP.flashsize));
    return 2;
  }

  uint32tobuf(jmptoboot, jmp_opcode(ur.blstart));
  return 4;
}


// Called after the input file has been read for writing or verifying flash
static int urclock_flash_readhook(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *flm,
  const char *fname, int size) { // size is max memory address + 1

  int nmdata, maxsize, firstbeg, firstlen;
  int vecsz = ur.uP.flashsize <= 8192? 2: 4; // Small parts use rjmp, large parts need 4-byte jmp

  set_date_filename(pgm, fname);

  // Record extent of metadata, given the command line options (default: max possible)
  ur.mcode = ur.nometadata || ur.nostore? 0xff:
    ur.nodate? 0: ur.nofilename? 1: strlen(ur.filename)+1;
  nmdata = ur.nometadata? 0: nmeta(ur.mcode, ur.uP.flashsize);

  maxsize = ur.pfend+1;

  // Compute begin and length of first contiguous block in input
  for(firstbeg=0; firstbeg < size; firstbeg++)
    if(flm->tags[firstbeg] & TAG_ALLOCATED)
      break;
  for(firstlen=0; firstbeg+firstlen < size; firstlen++)
    if(!(flm->tags[firstbeg+firstlen] & TAG_ALLOCATED))
      break;

  pmsg_notice2("%s %04d.%02d.%02d %02d.%02d meta %d boot %d\n", ur.filename,
    ur.yyyy, ur.mm, ur.dd, ur.hr, ur.mn, nmdata, ur.blend > ur.blstart? ur.blend-ur.blstart+1: 0);

  // Force upload of exactly this file, no patching, no metadata update, just trim if too big
  if(ur.restore) {
    if(size > maxsize)
      size = maxsize;

    goto nopatch_nometa;
  }

  // Sanity: no patching and no metadata if bootloader location is unknown
  if(ur.blend <= ur.blstart)
    goto nopatch_nometa;

  // Sanity check the bootloader position
  if(ur.blstart < 0 || ur.blstart >= flm->size || ur.blend < 0 || ur.blend >= flm->size)
    Return("bootloader [0x%04x, 0x%04x] outside flash [0, 0x%04x]",
      ur.blstart, ur.blend, flm->size-1);

  // Check size of uploded application and protect bootloader from being overwritten
  if((ur.boothigh && size > maxsize) || (!ur.boothigh && firstbeg <= ur.blend))
    Return("input [0x%04x, 0x%04x] overlaps bootloader [0x%04x, 0x%04x]; consider -xrestore",
      firstbeg, size-1, ur.blstart, ur.blend);

  if(size > maxsize)
    Return("input [0x%04x, 0x%04x] extends programmable area [0x%04x, 0x%04x]",
      firstbeg, size-1, ur.pfstart, ur.pfend);

  if(!ur.nometadata) {
    if(size == maxsize)
      Return("input [0x%04x, 0x%04x] overlaps metadata code byte at 0x%04x, consider -xnometadata",
        firstbeg, size-1, ur.pfend);

    if(nmdata >= nmeta(0, ur.uP.flashsize) && size > maxsize - nmeta(0, ur.uP.flashsize)) {
      pmsg_warning("input [0x%04x, 0x%04x] overlaps metadata [0x%04x, 0x%04x], selecting -xnostore\n",
       firstbeg, size-1, maxsize-nmdata, ur.pfend);
      ur.mcode = 0xff;
      ur.nostore = 1;
      nmdata = 1;
    }

    if(nmdata >= nmeta(1, ur.uP.flashsize) && size > maxsize - nmeta(1, ur.uP.flashsize)) {
      pmsg_warning("input [0x%04x, 0x%04x] overlaps metadata [0x%04x, 0x%04x], selecting -xnodate\n",
        firstbeg, size-1, maxsize-nmdata, ur.pfend);
      ur.mcode = 0;
      ur.nodate = 1;
      nmdata = nmeta(0, ur.uP.flashsize);
    }

    if(size > maxsize - nmdata) {
      pmsg_warning("input [0x%04x, 0x%04x] overlaps metadata [0x%04x, 0x%04x], selecting -xnofilename\n",
        firstbeg, size-1, maxsize-nmdata, ur.pfend);
      ur.mcode = 1;
      ur.nofilename = 1;
      nmdata = nmeta(1, ur.uP.flashsize);
    }
  }

  if(!ur.boothigh)
    goto nopatch;

  bool llcode = firstbeg == 0 && firstlen > ur.uP.ninterrupts*vecsz; // Looks like code
  bool llvectors = firstbeg == 0 && firstlen >= ur.uP.ninterrupts*vecsz; // Looks like vector table
  for(int i=0; llvectors && i<ur.uP.ninterrupts*vecsz; i+=vecsz) {
    uint16_t op16 = buf2uint16(flm->buf+i);
    if(!isRjmp(op16) && !(vecsz == 4 && isJmp(op16)))
      llvectors = 0;
  }

  if(llcode && !llvectors && ur.vblvectornum > 0 && ur.vbllevel)
    pmsg_warning("not patching jmp to application as input does not start with a vector table\n");

  // Patch vectors if input looks like code and it's a vector bootloader with known vector number
  if(llcode && llvectors && ur.vblvectornum > 0 && ur.vbllevel) {
    // From v7.5 patch all levels but for earlier and unknown versions only patch level 1
    if(ur.blurversion >= 075 || ((ur.blurversion==0 || ur.blurversion >= 072) && ur.vbllevel==1)) {
      uint16_t reset16;
      int reset32, appstart, appvecloc;

      appvecloc = ur.vblvectornum*vecsz; // Location of jump-to-application in vector table
      reset16 = buf2uint16(flm->buf);    // First reset word of to-be-uploaded application
      reset32 = vecsz == 2? reset16: buf2uint32(flm->buf);

      /*
       * Compute where the application starts from the reset vector. The assumptions are that the
       *  - Vector table, and therefore the reset vector, resides at address zero
       *  - Compiler puts either a jmp or an rjmp at address zero
       *  - Compiler does not shorten the vector table if no or few interrupts are used
       *  - Compiler does not utilise unused interrupt vectors to place code there
       * These are not necessarily true, but work for run-of-the-mill setups; the code below makes
       * a reasonable effort to detect whether the assumptions are violated, so at least there is
       * an error thrown if so.
       */

      if(reset2addr(flm->buf, vecsz, flm->size, &appstart) < 0) {
        pmsg_warning("not patching input as opcode word %04x at reset is not a%sjmp\n",
          reset16, vecsz==2? "n r": " ");
        goto nopatch;
      }

      // Only patch if appstart does not already point to the bootloader
      if(appstart != ur.blstart) {
        int vectorsend = vecsz*ur.vblvectornum;
        if(appstart < vectorsend || appstart >= size) { // appstart should be in [vectorsend, size)
          if(appstart != ur.blstart) {
            pmsg_warning("not patching as reset opcode %0*x jumps to 0x%04x,\n",
              vecsz*2, reset32, appstart);
            imsg_warning("ie, outside code area [0x%04x, 0x%04x)\n",
              vectorsend, size);
          }
          goto nopatch;
        }

        // OK, now have bootloader start and application start: patch
        set_reset(pgm, flm->buf+0, vecsz);
        if(vecsz == 4)
          uint32tobuf(flm->buf+appvecloc, jmp_opcode(appstart));
        else
          uint16tobuf(flm->buf+appvecloc, rjmp_opcode(appstart - appvecloc, ur.uP.flashsize));
      }
    }
  }

nopatch:

  if(nmdata) {
    int32_t nfree = maxsize - size;

    if(nfree >= nmdata) {
      unsigned char *p = flm->buf + maxsize - nmdata;

      if(ur.mcode != 0xff) {
        if(ur.mcode > 1) {      // Save filename
          memcpy(p, ur.filename, ur.mcode);
          p += ur.mcode;
        }

        if(ur.mcode >= 1) {     // Save date
          *p++ = ur.yyyy;
          *p++ = ur.yyyy>>8;
          *p++ = ur.mm;
          *p++ = ur.dd;
          *p++ = ur.hr;
          *p++ = ur.mn;
        }

        *p++ = size;            // Save where the pgm store begins
        *p++ = size >> 8;
        if(ur.uP.flashsize > (1<<16)) {
          *p++ = size >> 16;
          *p++ = size >> 24;
        }

        nfree -= nmdata;
        *p++ = nfree;           // Save how much is free
        *p++ = nfree >> 8;
        if(ur.uP.flashsize > (1<<16)) {
          *p++ = nfree >> 16;
          *p++ = nfree >> 24;
        }
      }

      *p++ = ur.mcode;          // Save metadata code

      // Set tags so metadata get burned onto chip
      memset(flm->tags + maxsize - nmdata, TAG_ALLOCATED, nmdata);

      if(ur.initstore)          // Zap the pgm store
        memset(flm->tags + size, TAG_ALLOCATED, nfree);

      size = maxsize;
    }
  }

  // Storing no metadata? Still put a 0xff byte just below bootloader if there is space
  if(size < maxsize && nmdata == 0) {
    flm->buf[ur.pfend] = 0xff;
    flm->tags[ur.pfend] = TAG_ALLOCATED;
    size = ur.pfend+1;
  }

nopatch_nometa:

  // Delete metadata on device (if any) that's between new input and metadata
  if(!ur.urprotocol || (ur.urfeatures & UB_READ_FLASH)) { // Flash readable?
    uint8_t devmcode;           // Metadata marker on the device
    if(ur_readEF(pgm, p, &devmcode, ur.pfend, 1, 'F') == 0) {
      int devnmeta=nmeta(devmcode, ur.uP.flashsize);
      for(int addr=ur.pfend+1-devnmeta; addr < ur.pfend+1; addr++) {
        if(addr >= 0 && addr < flm->size && !(flm->tags[addr] & TAG_ALLOCATED)) {
          flm->tags[addr] |= TAG_ALLOCATED;
          flm->buf[addr] = 0xff;
        }
      }
    }
  }

  // Emulate chip erase if bootloader unable to: mark all bytes for upload on first -U flash:w:...
  if(ur.emulate_ce) {
    for(int ai = 0; ai < maxsize; ai++)
      flm->tags[ai] = TAG_ALLOCATED;
    ur.emulate_ce = 0;
  }


  // Ensure that vector bootloaders have correct r/jmp at address 0
  if(ur.boothigh && ur.blstart && ur.vbllevel == 1) {
    int rc, set=0;
    for(int i=0; i < vecsz; i++)
      if(flm->tags[i] & TAG_ALLOCATED)
        set++;


    // Reset vector not programmed? Or -F? Ensure a jmp to bootloader
    if(ovsigck || set != vecsz) {
      unsigned char jmptoboot[4];
      int resetsize = set_reset(pgm, jmptoboot, vecsz);

      if(!ur.urprotocol || (ur.urfeatures & UB_READ_FLASH)) { // Flash readable?
        int resetdest;

        if(set != vecsz) {
          unsigned char device[4];
          // Read reset vector from device flash
          if((rc = ur_readEF(pgm, p, device, 0, vecsz, 'F')) < 0)
            return rc;

          // Mix with already set bytes
          for(int i=0; i < vecsz; i++)
            if(!(flm->tags[i] & TAG_ALLOCATED))
              flm->buf[i] = device[i];
        }

        if(reset2addr(flm->buf, vecsz, flm->size, &resetdest) < 0 || resetdest != ur.blstart) {
          for(int i=0; i < resetsize; i++) {
            flm->buf[i] = jmptoboot[i];
            flm->tags[i] |= TAG_ALLOCATED;
          }
        }
      } else {                  // Flash not readable: patch reset vector unconditionally
        for(int i=0; i < resetsize; i++) {
          flm->buf[i] = jmptoboot[i];
          flm->tags[i] |= TAG_ALLOCATED;
        }
      }
    } else if(firstbeg < vecsz) { // Double-check reset vector jumps to bootloader
      int resetdest;

      if(reset2addr(flm->buf, vecsz, flm->size, &resetdest) < 0)
        Return("input would overwrite the reset vector bricking the bootloader\n"
          "%*susing -F will try to patch the input but this may not be what is needed",
          (int) strlen(progname)+1, "");

      if(resetdest != ur.blstart)
        Return("input points reset to 0x%04x, not to bootloader at 0x%04x\n"
          "%*susing -F will try to patch the input but this may not be what is needed",
          resetdest, ur.blstart, (int) strlen(progname)+1, "");
    }
  }

  // Effective page size, can be 4*pagesize for 4-page erase parts
  int pgsize = p->n_page_erase > 0? p->n_page_erase*ur.uP.pagesize: ur.uP.pagesize;
  if((pgsize & (pgsize-1)) || pgsize < 1 || pgsize > maxsize || maxsize % pgsize)
    Return("effective page size %d implausible for size %d below bootloader", pgsize, maxsize);

  if(!ur.done_ce) {             // Unless chip erase was just issued (where all mem is 0xff)
    if((ur.urprotocol && !(ur.urfeatures & UB_FLASH_LL_NOR)) || !ur.urprotocol) {
      // Scan the memory for eff pages with unset bytes and read these bytes from device flash
      int ai, npe, addr, nset;

      uint8_t spc[2048];

      for(addr = 0; addr < maxsize; addr += pgsize) {
        // How many bytes are set in this effective page?
        for(ai = addr, nset = 0; ai < addr + pgsize; ai++)
          if(flm->tags[ai] & TAG_ALLOCATED)
            nset++;

        // Holes in this page that needs writing? read them in from the chip
        if(nset && nset != pgsize) {
          for(npe=0; npe < pgsize/ur.uP.pagesize; npe++) {
            // Identify a covering interval for all holes in page
            int istart, isize, beg, end;

            beg = addr + npe*ur.uP.pagesize;
            end = beg + ur.uP.pagesize;

            // Lowest address with unset byte (there might be none)
            for(ai = beg; ai < end; ai++)
              if(!(flm->tags[ai] & TAG_ALLOCATED))
                break;
            istart = ai;

            if(istart < end) {
              // Highest address with unset byte
              for(ai = end - 1; ai >= istart; ai--)
                if(!(flm->tags[ai] & TAG_ALLOCATED))
                  break;
              isize = ai - istart + 1;

              if(isize < 1 || isize > (int) sizeof spc) // Should not happen
                Return("isize=%d out of range (enlarge spc[] and recompile)", isize);

              if(ur_readEF(pgm, p, spc, istart, isize, 'F') == 0) {
                pmsg_debug("padding [0x%04x, 0x%04x]\n", istart, istart+isize-1);

                for(ai = istart; ai < istart + isize; ai++)
                  if(!(flm->tags[ai] & TAG_ALLOCATED)) {
                    flm->tags[ai] |= TAG_ALLOCATED;
                    flm->buf[ai] = spc[ai-istart];
                  }
              } else {
                pmsg_notice2("cannot read flash [0x%04x, 0x%04x] to pad page bytes\n",
                  istart, istart+isize-1);
              }
            }
          }
        }
      }
    }
  }
  ur.done_ce = 0;               // From now on can no longer rely on being deleted

  // Fill remaining holes (chip was erased, could not be read or memory looks like NOR memory)
  int ai, addr, nset;

  for(addr = 0; addr < maxsize; addr += pgsize) {
    for(ai = addr, nset = 0; ai < addr + pgsize; ai++)
      if(flm->tags[ai] & TAG_ALLOCATED)
        nset++;

    if(nset && nset != pgsize) { // Page has holes: fill them
      pmsg_debug("0xff padding page addr 0x%04d\n", addr);
      for(ai = addr, nset = 0; ai < addr + pgsize; ai++)
        if(!(flm->tags[ai] & TAG_ALLOCATED)) {
          flm->tags[ai] |= TAG_ALLOCATED;
          flm->buf[ai] = 0xff;
        }
    }
  }


  return size;
}


// Put version string into a buffer of max 19 characters incl nul (normally 15-16 bytes incl nul)
static void urbootPutVersion(const PROGRAMMER *pgm, char *buf, uint16_t ver, uint16_t rjmpwp) {
  uint8_t hi = ver>>8, type = ver & 0xff, flags;

  if(ver == 0xffff)             // Unknown provenance
    hi = type = 0;

  if(hi >= 072) {               // These are urboot versions
    sprintf(buf, "u%d.%d ", hi>>3, hi&7);
    buf += strlen(buf);
    *buf++ = (hi < 077 && (type & UR_PGMWRITEPAGE)) || (hi >= 077 && rjmpwp != ret_opcode)? 'w': '-';
    *buf++ = type & UR_EEPROM? 'e': '-';
    if(hi >= 076) {             // From urboot version 7.6 URPROTOCOL has its own bit
      *buf++ = type & UR_URPROTOCOL? 'u': 's';
      *buf++ = type & UR_DUAL? 'd': '-';
    } else {
      *buf++ = '-';             // Dummy bit
      flags = (type/UR_DUAL) & 3;
      // D = Dual boot with SE & SPI restoration, d = dual boot with SE, f = dual boot only
      *buf++ = flags==3? 'D': flags==2? 'd': flags? 'f': '-';
    }
    flags = (type/UR_VBL) & 3;
    // V = VBL, patch & verify, v = VBL, patch only, j = VBL, jump only
    *buf++ = flags==3? 'V': flags==2? 'v': flags? 'j': 'h';
    *buf++ = hi < 077? (type & UR_PROTECTME? 'p': '-'): (type & UR_PROTECTME? 'P': 'p');
    *buf++ = (hi < 077 && (type & UR_RESETFLAGS)) || hi >= 077? 'r': '-';
    *buf++ = hi >= 077 && (type & UR_AUTOBAUD)? 'a': '-';                 // - means no
    *buf++ = hi >= 077 && (type & UR_HAS_CE)? 'c': hi >= 077? '-': '.';   // . means don't know
    *buf = 0;
  } else if(hi) {               // Version number in binary from optiboot v4.1
    sprintf(buf, "o%d.%d -%cs-%c-r--", hi, type,
      ur.blguessed? (ur.bleepromrw? 'e': '-'): '?',
      ur.blguessed? "hjvV"[ur.vbllevel & 3]: '?');
  } else
    sprintf(buf, "x0.0 .........");

  return;
}


// Return name of the vector with number num
static const char *vblvecname(const PROGRAMMER *pgm, int num) {
  // This should never happen
  if(num < -1 || num > ur.uP.ninterrupts || !ur.uP.isrtable)
    return("unknown");
  if(num == -1)
    return "none";
  if(num == ur.uP.ninterrupts)
    return "VBL_ADDITIONAL_VECTOR";
  return ur.uP.isrtable[num];
}


// Check protocol bytes and read result if needed
static int urclock_res_check(const PROGRAMMER *pgm, const char *funcname, int ignore,
  unsigned char *res, int expected) {

  unsigned char chr;

  if(urclock_recv(pgm, &chr, 1) < 0)
    return -1;
  if(chr != ur.STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x in %s()\n",
      ur.STK_INSYNC, chr, funcname);
    return -1;
  }

  // Potentially ignore some initial bytes of the reply
  while(ignore--)
    if(urclock_recv(pgm, &chr, 1) < 0)
      return -1;

  // Read the reply from previous command if requested
  if(res && expected > 0)
    if(urclock_recv(pgm, res, expected) < 0)
      return -1;

  if(urclock_recv(pgm, &chr, 1) < 0)
    return -1;
  if(chr != ur.STK_OK) {
    pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x in %s()\n",
      ur.STK_OK, chr, funcname);
    return -1;
  }

  return 0;
}


// set ur.uP from mcuid, potentially overwritten by p
static void set_uP(const PROGRAMMER *pgm, const AVRPART *p, int mcuid, int mcuid_wins) {
  int idx_m = -1, idx_p = -1;

  if(mcuid < 0 && !p)           // This should never happen
    pmsg_warning("cannot set ur.uP as neither mcuid nor part given\n");

  if(mcuid >= 0)
    if((idx_m = upidxmcuid(mcuid)) < 0)
      pmsg_warning("uP_table does not know mcuid %d\n", mcuid);

  if(p) {
    if(p->mcuid >= 0)
      idx_p = upidxmcuid(p->mcuid);
    if(idx_p < 0 && p->desc && *p->desc)
      idx_p = upidxname(p->desc);
    if(idx_p < 0)
      pmsg_warning("uP_table does not know mcuid %d nor part %s\n",
        p->mcuid, p->desc && *p->desc? p->desc: "???");
  }

  ur.uP.name = NULL;
  if(idx_m >= 0 && idx_p >= 0)
    ur.uP = uP_table[mcuid_wins? idx_m: idx_p];
  else if(idx_m >= 0)
    ur.uP = uP_table[idx_m];
  else if(idx_p >= 0)
    ur.uP = uP_table[idx_p];

  if(!ur.uP.name && p) {        // Not found in uP_table? Fill in from p; -1 means unknown
    AVRMEM *mem;

    ur.uP.name = p->desc;
    ur.uP.mcuid = p->mcuid;
    ur.uP.avrarch =
      p->prog_modes & PM_UPDI? F_AVR8X:
      p->prog_modes & PM_PDI? F_XMEGA:
      p->prog_modes & PM_TPI? F_AVR8L:
      p->prog_modes & (PM_ISP | PM_HVPP | PM_HVSP)? F_AVR8: 0;
    memcpy(ur.uP.sigs, p->signature, sizeof ur.uP.sigs);
    if((mem = avr_locate_flash(p))) {
      ur.uP.flashoffset = mem->offset;
      ur.uP.flashsize = mem->size;
      ur.uP.pagesize = mem->page_size;
    } else {
      ur.uP.flashoffset = -1;
      ur.uP.flashsize = -1;
      ur.uP.pagesize = -1;
    }
    ur.uP.nboots = -1;
    ur.uP.bootsize = -1;
    if((mem = avr_locate_eeprom(p))) {
      ur.uP.eepromoffset = mem->offset;
      ur.uP.eepromsize = mem->size;
      ur.uP.eeprompagesize = mem->page_size;
    } else {
      ur.uP.eepromoffset = -1;
      ur.uP.eepromsize = -1;
      ur.uP.eeprompagesize = -1;
    }
    ur.uP.sramstart = -1;
    ur.uP.sramsize = -1;
    ur.uP.nfuses = -1;
    ur.uP.nlocks = -1;
    ur.uP.ninterrupts = p->n_interrupts;
    ur.uP.isrtable = NULL;
  }
}


// https://en.wikipedia.org/wiki/Jenkins_hash_function
static uint32_t jenkins_hash(const uint8_t* key, size_t length) {
  size_t i = 0;
  uint32_t hash = 0;

  while (i != length) {
    hash += key[i++];
    hash += hash << 10;
    hash ^= hash >> 6;
  }
  hash += hash << 3;
  hash ^= hash >> 11;
  hash += hash << 15;

  return hash;
}

typedef struct {
  uint16_t sz, ee;
  uint32_t h256, hash;
} Blhash_t;

static int cmpblhash(const void *va, const void *vb) {
  const Blhash_t *a = va, *b = vb;
  return a->sz > b->sz? 1: a->sz < b->sz? -1: a->hash > b->hash? 1: a->hash < b->hash? -1: 0;
}

static void guessblstart(const PROGRAMMER *pgm, const AVRPART *p) {
  if(ur.urprotocol && !(ur.urfeatures & UB_READ_FLASH)) // Cannot read flash
    return;

  Blhash_t blist[] = {
    // From https://github.com/arduino/ArduinoCore-avr/tree/master/bootloaders
    { 1024, 0, 0x35445c45, 0x9ef77953 }, // ATmegaBOOT-prod-firmware-2009-11-07.hex
    { 1024, 0, 0x32b1376c, 0xceba80bb }, // ATmegaBOOT.hex
    { 2048, 0, 0x08426ba2, 0x29e81e21 }, // ATmegaBOOT_168.hex
    { 4096, 0, 0x1bf8ed1b, 0x272e49ed }, // ATmegaBOOT_168_atmega1280.hex
    { 2048, 0, 0x9774b926, 0x335016ed }, // ATmegaBOOT_168_atmega328.hex
    { 4096, 0, 0x3242ddd3, 0x809632a3 }, // ATmegaBOOT_168_atmega328_bt.hex
    { 2048, 0, 0xc553f5b4, 0x56be91cb }, // ATmegaBOOT_168_atmega328_pro_8MHz.hex
    { 2048, 0, 0x12ab8da0, 0xca46a3ca }, // ATmegaBOOT_168_diecimila.hex
    { 2048, 0, 0x3242ddd3, 0xf3e94dba }, // ATmegaBOOT_168_ng.hex
    { 2048, 0, 0x2eed30b3, 0x47d14ffa }, // ATmegaBOOT_168_pro_8MHz.hex
    { 2048, 0, 0x1cef0d75, 0x6cfbac49 }, // LilyPadBOOT_168.hex
    { 1024, 1, 0x6ca0f37b, 0x21124cde }, // bigboot_328p_8v3_uno_ch340_clone.hex
    { 1024, 1, 0xae42ebb8, 0xeb4b1b71 }, // bigboot_328p_8v0.hex
    { 1024, 1, 0x6ca0f37b, 0x31bae545 }, // bigboot_328.hex
    {  512, 0, 0x035cbc07, 0x24ba435e }, // optiboot_atmega168.hex
    {  512, 0, 0x455050db, 0x1d53065f }, // optiboot_atmega328-Mini.hex
    {  512, 0, 0xd2001ddb, 0x16c9663b }, // optiboot_atmega328.hex v4.4
    {  512, 0, 0x49c1e9a4, 0xa450759b }, // optiboot_atmega328.hex v8.3
    {  512, 0, 0xc54dcd6c, 0x5bfc5d06 }, // optiboot_atmega8.hex

    // From https://github.com/nerdralph/picoboot
    {  256, 0, 0x5a01c55b, 0x5a01c55b }, // picobootArduino168.hex
    {  256, 0, 0x1451061b, 0x1451061b }, // picobootArduino168v3b2.hex
    {  512, 0, 0x3242ddd3, 0x53348738 }, // picobootArduino328.hex
    {  512, 0, 0x858e12de, 0xc80a44a4 }, // picobootArduino328v3beta.hex
    {  512, 0, 0x3242ddd3, 0xc254e344 }, // picobootArduino328v3b2.hex
    {  256, 0, 0xaa62bafc, 0xaa62bafc }, // picobootArduino8v3rc1.hex
    {  256, 0, 0x56263965, 0x56263965 }, // picobootSTK500-168p.hex
    {  512, 0, 0x3242ddd3, 0x5ba5f5f6 }, // picobootSTK500-328p.hex

    // From https://github.com/LGTMCU/Larduino_HSP/tree/master/hardware/LGT/avr/bootloaders/lgt8fx8p
    { 3072, 0, 0x3242ddd3, 0xd3347c5d }, // optiboot_lgt8f328p.hex

    // From https://github.com/Lauszus/Sanguino
    { 1024, 0, 0xe244a3c6, 0xc7ceaadf }, // optiboot_atmega644.hex
    { 1024, 0, 0xe244a3c6, 0x063b24dd }, // optiboot_atmega1284p.hex
    { 1024, 0, 0xe244a3c6, 0x6e5d8d92 }, // optiboot_balanduino644.hex
    { 1024, 0, 0xe244a3c6, 0xed2e78d7 }, // optiboot_atmega1284p_8m.hex
    { 1024, 0, 0xe244a3c6, 0x57215b62 }, // optiboot_atmega644p.hex
    { 1024, 0, 0xe244a3c6, 0x365954f4 }, // optiboot_atmega644p_8m.hex
    { 1024, 0, 0xe244a3c6, 0x6f120e6a }, // optiboot_atmega644_8m.hex
    { 1024, 0, 0xe244a3c6, 0x79b266ae }, // optiboot_balanduino.hex

#include "urclock_hash.h"                // Selected from https://github.com/MCUdude/optiboot_flash
  };

  uint8_t buf[4096], b128[128];

  qsort(blist, sizeof blist/sizeof*blist, sizeof*blist, cmpblhash);
  for(int ii, si = 0, sz = 0, bi = 0; si < (int) (sizeof blist/sizeof*blist); si++) {
    if(blist[si].sz > sz) { // Read in and compare
      sz = blist[si].sz;
      if(sz > ur.uP.flashsize/2 || (sz+127)/128*128 > (int) sizeof buf)
        return;
      while(bi < sz) {
       if(ur_readEF(pgm, p, b128, ur.uP.flashsize-bi-128, 128, 'F') < 0)
         return;
       for(int ti=127; ti >= 0; ti--) // read in backwards
         buf[bi++] = b128[ti];
      }

      // Does the hash for the full size match? OK: found a known bootloader
      uint32_t hash = jenkins_hash(buf, sz);
      for(ii = 0; ii < (int) (sizeof blist/sizeof*blist); ii++)
        if(blist[ii].hash == hash && sz == blist[ii].sz && !(sz & (ur.uP.pagesize-1))) {
          // Page aligned bootloader size matches
          ur.blstart = ur.uP.flashsize - sz;
          ur.blend   = ur.uP.flashsize - 1;
          ur.pfend   = ur.blstart - 1;

          if(blist[ii].ee)
            ur.bleepromrw = 1;
          ur.blguessed = 1;
          return;
        }

      // Can we exclude the top 256 byte flash from the botloader list?
      if(sz == 256) {
        for(ii = 0; ii < (int) (sizeof blist/sizeof*blist); ii++)
          if(hash == blist[ii].h256)
            break;
        if(ii >= (int) (sizeof blist/sizeof*blist))
          return;
      }
    }
  }
}


/*
 * Read signature bytes - Urclock version
 *
 * Piggy back reading urboot specific configuration from chip
 *  - whether it is an urboot bootloader, if so,
 *     + which version
 *     + where the bootloader starts
 *     + whether it is a vector bootloader, if so which vector is used
 *  - urclock board ID from EEPROM
 */
static int urclock_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *sigmem) {
  uint8_t buf[16];
  int conn_idx;

  // Signature byte reads are always 3 bytes
  if(sigmem->size < 3) {
    pmsg_error("memsize too small for sig byte read\n");
    return -1;
  }

  if(ur.urprotocol) {           // Urprotocol's STK_INSYNC/STK_OK have already identified the part
    // Got the signature already
    memcpy(sigmem->buf, ur.uP.sigs, 3);
  } else {
    // Have to ask the bootloader directly
    buf[0] = Cmnd_STK_READ_SIGN;
    buf[1] = Sync_CRC_EOP;
    if(urclock_send(pgm, buf, 2) < 0)
      return -1;
    if(urclock_res_check(pgm, __func__, 0, sigmem->buf, 3) < 0)
      return -1;
  }

  if(ur.initialised)
    return 3;

  if(ovsigck || !ur.uP.name) {  // Keep -p ... MCU when given -F  or when not initialised
    set_uP(pgm, p, -1, 0);
    if(!ur.uP.name)
      Return("cannot identify MCU from part %s", p->desc);
  } else {
    // If signatures of command line part and connected part differ complain and return
    if(memcmp(sigmem->buf, p->signature, 3)) {
      char names[1024];

      if(ur.urprotocol)
        Return("connected part %s differs in signature from -p %s (override with -F or use -p %s)",
          ur.uP.name, p->desc, ur.uP.name);
      if((conn_idx = upidxsig(sigmem->buf)) == -1)
        Return("no uP_table entry from signature %02x %02x %02x (override with -F)",
          sigmem->buf[0], sigmem->buf[1], sigmem->buf[2]);
      if(upmatchingsig(sigmem->buf, names, sizeof names) == 1)
        Return("connected part %s signature does not match -p %s's "
          "(override with -F or use -p %s)",
          uP_table[conn_idx].name, p->desc, uP_table[conn_idx].name);
      Return("connected part's signature %02x%02x%02x is one of %s; neither matches -p %s's "
        "(override with -F or use -p ...)",
        sigmem->buf[0], sigmem->buf[1], sigmem->buf[2], names, p->desc);
    }
  }

  return ur_initstruct(pgm, p);
}

static int ur_initstruct(const PROGRAMMER *pgm, const AVRPART *p) {
  uint8_t spc[2048];
  AVRMEM *flm;
  int rc;

  if(!(flm = avr_locate_flash(p)))
    Return("cannot obtain flash memory for %s", p->desc);

  if(flm->page_size > (int) sizeof spc)
    Return("%s's flash page size %d is too large (enlarge spc[] and recompile)",
      p->desc, flm->page_size);

  if(flm->page_size <= 0)
    Return("cannot deal with %s's flash page size of %d", p->desc, flm->page_size);

  if(flm->page_size & (flm->page_size-1))
    Return("cannot deal with %s's flash page size %d as not a power of 2",
      p->desc, flm->page_size);

  // Bail if command line part and connected part differ in important aspects (should not happen)
  if(ur.uP.flashsize != flm->size)
    Return("connected %s's flash size 0x%04x differs from -p %s's (0x%04x); "
      "use correct -p ... or override with -F",
      ur.uP.name, ur.uP.flashsize, p->desc, flm->size);
  if(ur.uP.pagesize != flm->page_size)
    Return("connected %s's flash page size %d differs from -p %s's (%d); "
      "use correct -p ... or override with -F",
      ur.uP.name, ur.uP.pagesize, p->desc, flm->page_size);
  if(ur.uP.ninterrupts != p->n_interrupts)
    Return("connected %s's number %d of interrupts differs from -p %s's (%d); "
      "use correct -p ... or override with -F",
      ur.uP.name, ur.uP.ninterrupts, p->desc, p->n_interrupts);

  // Initialse so that programmable flash is all flash and no bootloader
  ur.pfstart = 0;
  ur.pfend   = flm->size-1;
  ur.blstart = 0;
  ur.blend = 0;
  ur.vbllevel = 0;
  ur.vblvectornum = -1;
  ur.bleepromrw = 0;

  // No urboot bootloaders on AVR32 parts, neither on really small devices
  if((p->prog_modes & PM_aWire) || flm->size < 512)
    goto alldone;

  // UPDI parts have bootloader in low flash
  ur.boothigh = !(p->prog_modes & PM_UPDI);

  // Manual provision of above bootloader parameters
  if(ur.xbootsize) {
    if(ur.boothigh && ur.xbootsize % ur.uP.pagesize)
      Return("-xbootsize=%d size not a multiple of flash page size %d",
        ur.xbootsize, ur.uP.pagesize);
    if(ur.xbootsize < 64 || ur.xbootsize > urmin(8192, ur.uP.flashsize/4))
      Return("implausible -xbootsize=%d, should be in [64, %d]",
        ur.xbootsize, urmin(8192, ur.uP.flashsize/4));
    if(ur.boothigh) {
      ur.blstart = flm->size - ur.xbootsize;
      ur.blend   = flm->size - 1;
      ur.pfend   = ur.blstart - 1;
    } else {
      ur.blstart = 0;
      ur.blend   = ur.xbootsize - 1;
      ur.pfstart = ur.blend + 1;
    }
  }

  if(ur.boothigh) {
    if((int8_t) ur.uP.ninterrupts >= 0) // valid range is 0..127
      if(ur.xvectornum < -1 || ur.xvectornum > ur.uP.ninterrupts)
        Return("unknown interrupt vector #%d for vector bootloader -- should be in [-1, %d]",
          ur.xvectornum, ur.uP.ninterrupts);
    if(ur.xvectornum > 0) {
      ur.vbllevel = 1;
      ur.vblvectornum = ur.xvectornum;
    }
  } else if(ur.xvectornum != -1) {
    Return("UPDI part %s does not support vector bootloaders", ur.uP.name);
  }

  if(ur.urprotocol && !(ur.urfeatures & UB_READ_FLASH)) // Bootloader that cannot read flash?
    if(ur.blend <= ur.blstart)
      Return("please specify -xbootsize=<num> and, if needed, %s-xeepromrw",
        ur.boothigh? "-xvectornum=<num> or ": "");

  uint16_t v16 = 0xffff, rjmpwp = ret_opcode;

  // Sporting chance that we can read top flash to get intell about bootloader?
  if(ur.boothigh && (!ur.urprotocol || (ur.urfeatures & UB_READ_FLASH))) {
    // Read top 6 bytes from flash memory to obtain extended information about bootloader and type
    if((rc = ur_readEF(pgm, p, spc, flm->size-6, 6, 'F')))
      return rc;

    // In a urboot bootloader (v7.2 onwards) these six are as follows
    uint8_t numpags = spc[0];   // Actually, these two only exist from v7.5 onwards
    uint8_t vectnum = spc[1];
    rjmpwp = buf2uint16(spc+2); // rjmp to bootloader pgm_write_page() or ret opcode
    uint8_t cap = spc[4];       // Capability byte
    uint8_t urver = spc[5];     // Urboot version (low three bits are minor version: 076 is v7.6)
    v16 = buf2uint16(spc+4);    // Combo word for neatly printed version line of urboot bootloader

    // Extensively check this is an urboot bootloader v7.2 .. v12.7 == 0147 and extract properties
    if(urver >= 072 && urver <= 0147 && (isRjmp(rjmpwp) || rjmpwp == ret_opcode)) { // Prob urboot
      ur.blurversion = urver;
      ur.bleepromrw = iseeprom_cap(cap);
      // Vector bootloader: 0 = none, 1 = external patching, 2 = bl patches, 3 = patches + verifies
      if(!ur.vbllevel)          // Unless manually overwritten
        ur.vbllevel = vectorbl_level_cap(cap);
      if(urver >= 075) {        // Urboot v7.5+ encodes # of bootloader pages and vbl vector number
        int blsize = numpags*flm->page_size;
        // Size of urboot bootloader should be in [64, 2048] (in v7.6 these are 224-512 bytes)
        if(blsize >= 64 && blsize <= 2048 && vectnum <= ur.uP.ninterrupts) { // Within range
          int dfromend  = dist_rjmp(rjmpwp, ur.uP.flashsize) - 4;
          // Further check whether writepage() rjmp opcode jumps backwards into bootloader
          if(rjmpwp == ret_opcode || (dfromend >= -blsize && dfromend < -6)) { // Due diligence
            if(ur.xbootsize) {
              if(flm->size - blsize != ur.blstart) {
                pmsg_warning("urboot bootloader size %d explicitly overwritten by -xbootsize=%d\n",
                  blsize, ur.xbootsize);
                if(!ovsigck && ur.vbllevel) {
                  imsg_warning("this can lead to bricking the vector bootloader\n");
                  return -1;
                }
              }
            } else {
              ur.blstart = flm->size - blsize;
              ur.blend   = flm->size - 1;
              ur.pfend   = ur.blstart - 1;
            }

            if(ur.xvectornum != -1) {
              if(ur.vblvectornum != vectnum) {
                pmsg_warning("urboot vector number %d overwritten by -xvectornum=%d\n",
                  vectnum, ur.xvectornum);
                imsg_warning("the application might not start correctly\n");
              }
            } else
              ur.vblvectornum = vectnum;
          }
        }
      }
    } else if(urver != 0xff) {  // Probably optiboot where the version number is two bytes
      ur.bloptiversion = (urver<<8) + cap;
    }

    if(ur.blend <= ur.blstart && ur.vbllevel) { // An older version urboot vector bootloader?
      int vecsz = ur.uP.flashsize <= 8192? 2: 4;

       // Reset vector points to the bootloader and the bootloader has r/jmp to application?
      if((rc = ur_readEF(pgm, p, spc, 0, 4, 'F')))
        return rc;

      uint16_t reset16 = buf2uint16(spc);

      if(isRjmp(reset16)) {     // rjmp op code (could be from a large or a small part)
        if((flm->size & (flm->size-1)) == 0) { // Flash size a power of 2? True for small parts
          int guess = dist_rjmp(reset16, ur.uP.flashsize); // Relative destination to reset vector
          while(guess < 0)      // Convert to absolute address
            guess += flm->size;
          if((guess & (flm->page_size-1)) == 0) // Page aligned? Good
            if(flm->size - guess <= 2048) { // Accept unless size of bootloader exceeds 2048 bytes
              ur.blstart = guess;
              ur.blend   = flm->size - 1;
              ur.pfend   = guess - 1;
            }
        }
      } else if(vecsz == 4 && isJmp(reset16)) { // Jmp op code
        int guess = addr_jmp(buf2uint32(spc));
        if(guess < flm->size)
          if((guess & (flm->page_size-1)) == 0) // Page aligned? Good
            if(flm->size - guess <= 2048) { // Accept unless size of bootloader exceeds 2048 bytes
              ur.blstart = guess;
              ur.blend   = flm->size - 1;
              ur.pfend   = guess - 1;
            }
      }

      if(ur.blend > ur.blstart && ur.vblvectornum > 0)
        goto vblvecfound;

      if(ur.blend > ur.blstart) { // Read bootloader to identify jump to vbl vector
        int i, npages, j, n, toend, dist, wasop32, wasjmp, op16;
        uint8_t *q;
        uint16_t opcode;

        op16 = wasjmp = wasop32 = 0;
        toend = flm->size-ur.blstart; // Number of bytes to FLASHEND
        npages = toend/flm->page_size;
        for(i=0; i<npages; i++) {
          // Read bootloader page by page
          if((rc = ur_readEF(pgm, p, spc, ur.blstart+i*flm->page_size, flm->page_size, 'F')))
            return rc;
          for(n=flm->page_size/2, q=spc, j=0; j<n; j++, q+=2, toend-=2) { // Check 16-bit opcodes
            opcode = buf2uint16(q);
            if(wasjmp) {        // Opcode is the word address of the destination
              wasjmp=0;
              int dest = addr_jmp((opcode<<16) | op16);
              if(dest % vecsz == 0 && dest <= ur.uP.ninterrupts*vecsz) { // "<=" for extended table
                ur.vblvectornum = dest/vecsz; // Solve for the vbl vector number
                goto vblvecfound;
              }
              op16 = 0;
            } else if(wasop32) { // Skip opcode evaluation
              wasop32 = 0;
            } else if(isRjmp(opcode) && toend > 4) { // 4 top bytes of bl are data, not rjmp
              // Does that rjmp end in the vector table?
              if((dist = dist_rjmp(opcode, ur.uP.flashsize)) > toend &&
                  dist <= toend+ur.uP.ninterrupts*vecsz) { // "<=" for extended vector table
                ur.vblvectornum = (dist-toend)/vecsz; // Solve for the vbl vector number
                goto vblvecfound;
              }
            } else if(isJmp(opcode) && toend > 6) { // 4 top bytes are data + 2 the jmp addr
              op16 = opcode;
              wasjmp = 1;       // Look at destination address in next loop iteration
            } else if(isop32(opcode)) { // Skip next opcode, too
              wasop32 = 1;
            }
          }
        }
      }
    }

    // Still no bootloader start address? Read in top flash and guess bootloader start
    if(ur.blend <= ur.blstart)
      guessblstart(pgm, p);

    // Still no bootloader start address?
    if(ur.blend <= ur.blstart) {
      if(ur. bloptiversion)
        Return("bootloader might be optiboot %d.%d? Please use -xbootsize=<num>\n",
          ur.bloptiversion>>8, ur.bloptiversion & 255);
      Return("unknown bootloader ... please specify -xbootsize=<num>\n");
    }
  } else if(!ur.boothigh) { // Fixme: guess bootloader size from low flash
  }

vblvecfound:
  urbootPutVersion(pgm, ur.desc, v16, rjmpwp);

  ur.mcode = 0xff;
  int havemetadata = !ur.nometadata;

  if(havemetadata && ur.pfend >= nmeta(254, flm->size)) {
    int nm = nmeta(1, ur.uP.flashsize); // 6 for date + size of store struct + 1 for mcode byte
    // Showing properties mostly requires examining the bytes below bootloader for metadata
    if(ur.showall || (ur.showid && *ur.iddesc && *ur.iddesc != 'E') || ur.showapp ||
      ur.showstore || ur.showmeta || ur.showboot || ur.showversion || ur.showvector ||
      ur.showpart || ur.showdate || ur.showfilename) {

      if((rc = ur_readEF(pgm, p, spc, ur.pfend+1-nm, nm, 'F')))
        return rc;

      if(spc[nm-1] != 0xff) {
        int32_t storesize = ur.uP.flashsize > (1<<16)? buf2uint32(spc+nm-5): buf2uint16(spc+nm-3);
        int32_t storestart = ur.uP.flashsize > (1<<16)? buf2uint32(spc+nm-9): buf2uint16(spc+nm-5);
        uint8_t mcode = spc[nm-1];
        int nmdata = nmeta(mcode, ur.uP.flashsize);

        // Check plausibility of metadata header just below bootloader
        havemetadata = 0;
        if(storestart > 0 && storestart == ur.pfend+1-nmdata-storesize) {
          ur.storestart = storestart;
          ur.storesize = storesize;
          ur.mcode = mcode;
          if(!mcode) {
            havemetadata = 1;
          } else {
            int16_t  yyyy;
            int8_t mm, dd, hr, mn;
            mn = spc[5];
            hr = spc[4];
            dd = spc[3];
            mm = spc[2];
            yyyy = buf2uint16(spc);

            // Is the date plausible? carry on; note this won't work after the year 2999 CE :-O
            if(yyyy > 0 && yyyy < 3000 && mm > 0 && mm < 13 && dd > 0 && dd < 32 &&
              hr >= 0 && hr < 24 && mn >= 0 && mn < 60) {

              ur.yyyy = yyyy;
              ur.mm = mm;
              ur.dd = dd;
              ur.hr = hr;
              ur.mn = mn;
              havemetadata = 1;
              if(mcode > 1) {   // Copy application name over
                rc = ur_readEF(pgm, p, spc, ur.pfend+1-nmeta(mcode, ur.uP.flashsize), mcode, 'F');
                if(rc < 0)
                  return rc;
                int len = mcode<sizeof ur.filename? mcode: sizeof ur.filename;
                memcpy(ur.filename, spc, len);
                ur.filename[len-1] = 0;
              }
            }
          }
        }
      }
    }
  }

  // Print and exit when option show... was given
  int first=1;
  int single = !ur.showall && (!!ur.showid + !!ur.showapp + !!ur.showstore + !!ur.showmeta +
    !!ur.showboot + !!ur.showversion + !!ur.showvector + !!ur.showpart + !!ur.showdate +
    !!ur.showfilename) == 1;

  if(ur.showid || ur.showall) {
    uint64_t urclockID;
    if((rc = readUrclockID(pgm, p, &urclockID)) == -1)
      return rc;
    term_out("%0*lx", 2*ur.idlen, urclockID), first=0;
  }
  if(havemetadata) {
    if(ur.showdate || ur.showall) {
      term_out(&" %04d-%02d-%02d %02d.%02d"[first], ur.yyyy, ur.mm, ur.dd, ur.hr, ur.mn);
      first=0;
    }
    if(ur.showfilename || ur.showall) {
      term_out(&" %s"[first], *ur.filename? ur.filename: "");
      first=0;
    }
    if(ur.showapp || ur.showall) {
      term_out(&" %s%d"[first], single || *ur.filename? "": "application ", ur.storestart);
      first=0;
    }
    if(ur.showstore || ur.showall) {
      term_out(&" %s%d"[first], single? "": "store ", ur.storesize);
      first=0;
    }
  }
  if(ur.showmeta || ur.showall) {
    int nmdata = havemetadata? nmeta(ur.mcode, ur.uP.flashsize): 0;
    term_out(&" %s%d"[first], single? "": "meta ", nmdata);
    first=0;
  }
  if(ur.showboot || ur.showall) {
    term_out(&" %s%d"[first], single? "": "boot ", ur.blend>ur.blstart? ur.blend-ur.blstart+1: 0);
    first=0;
  }
  if(ur.showversion || ur.showall) {
    term_out(&" %s"[first], ur.desc+(*ur.desc==' '));
    first=0;
  }
  if(ur.showvector || ur.showall) {
    int vnum = ur.vbllevel? ur.vblvectornum & 0x7f: 0;
    term_out(&" vector %d (%s)"[first], vnum, vblvecname(pgm, vnum));
    first=0;
  }
  if(ur.showall || ur.showpart)
    term_out(&" %s"[first], ur.uP.name);
  if(!first) {
    term_out("\n");
    exit(0);
  }

alldone:

  ur.initialised = 1;
  return 3;
}


// STK500 section from stk500.c but modified significantly for use with urboot bootloaders

// STK500v1 load correct address for flash/eeprom, memchr is 'E'/'F'
static int urclock_load_baddr(const PROGRAMMER *pgm, const AVRPART *p, char memchr,
  unsigned int baddr) {

  unsigned char buf[16], ext_byte;

  // For classic parts (think optiboot, avrisp) use word addr, otherwise byte addr (optiboot_x etc)
  int classic = !(p->prog_modes & (PM_UPDI | PM_PDI | PM_aWire));
  unsigned int addr = classic? baddr/2: baddr;
  int effpgsiz = classic? ur.uP.pagesize/2: ur.uP.pagesize;

  // STK500 protocol: support flash > 64k words/bytes with the correct extended-address byte
  if(memchr == 'F' && ur.uP.flashsize > (classic? 128*1024: 64*1024)) {
    ext_byte = (addr >> 16) & 0xff;
    if(ext_byte != ur.ext_addr_byte) {
      // Either this is the first addr load, or a 64k boundary is crossed
      buf[0] = (uint8_t) (Subc_STK_UNIVERSAL_LEXT>>24);
      buf[1] = (uint8_t) (Subc_STK_UNIVERSAL_LEXT>>16);
      buf[2] = ext_byte;
      buf[3] = (uint8_t) (Subc_STK_UNIVERSAL_LEXT);
      urclock_cmd(pgm, buf, buf);
      ur.ext_addr_byte = ext_byte;
    }
    /*
     * Ensure next paged r/w will reload ext addr if page is just below a 64k boundary
     * to iron out a bug in some bootloaders
     */
    if((addr & 0xffff0000) != ((addr+effpgsiz) & 0xffff0000))
      ur.ext_addr_byte = 0xff;
  }

  buf[0] = Cmnd_STK_LOAD_ADDRESS;
  buf[1] = addr & 0xff;
  buf[2] = (addr >> 8) & 0xff;
  buf[3] = Sync_CRC_EOP;

  if(urclock_send(pgm, buf, 4) < 0)
    return -1;

  return urclock_res_check(pgm, __func__, 0, NULL, 0);
}


/*
 * Send a paged cmd to device
 *  - rwop is Cmnd_STK_READ/PROG_PAGE
 *  - badd is the byte address, len the length of data
 *  - mchr is 'F' (flash) or 'E' (EEPROM)
 *  - payload for bytes to write or NULL for read
 */
static int urclock_paged_rdwr(const PROGRAMMER *pgm, const AVRPART *part, char rwop,
  unsigned int badd, int len, char mchr, char *payload) {

  int i;
  uint8_t buf[1024 + 5];

  // STK500v1 only: tell the bootloader which address should be used by next paged command
  if(!ur.urprotocol && urclock_load_baddr(pgm, part, mchr, badd) < 0)
      return -1;

  if(mchr == 'F' && rwop == Cmnd_STK_PROG_PAGE) {
    if(len != ur.uP.pagesize)
      Return("len %d must be page size %d for paged flash writes", len, ur.uP.pagesize);

    if(badd < 4U && ur.boothigh && ur.blstart && ur.vbllevel==1) {
      int vecsz = ur.uP.flashsize <= 8192? 2: 4;
      unsigned char jmptoboot[4];
      int resetsize = set_reset(pgm, jmptoboot, vecsz);

      if(badd < (unsigned int) resetsize) { // Ensure reset vector points to bl
        int n = urmin((unsigned int) resetsize - badd, (unsigned int) len);
        int resetdest;

        if(badd == 0 && len >= vecsz) {
          if(reset2addr((unsigned char *) payload, vecsz, ur.uP.flashsize, &resetdest) < 0 ||
            resetdest != ur.blstart) {

            memcpy(payload, jmptoboot, resetsize);
            pmsg_info("forcing reset vector to point to vector bootloader\n");
          }
        } else if(memcmp(payload, jmptoboot+badd, n)) {
          memcpy(payload, jmptoboot+badd, n);
          pmsg_info("forcing partial reset vector to point to vector bootloader\n");
        }
      }
    }
  }

  if(ur.urprotocol) {
    uint8_t *q = buf, op =
      mchr == 'F' && rwop == Cmnd_STK_PROG_PAGE? Cmnd_UR_PROG_PAGE_FL:
      mchr == 'E' && rwop == Cmnd_STK_PROG_PAGE? Cmnd_UR_PROG_PAGE_EE:
      mchr == 'F' && rwop == Cmnd_STK_READ_PAGE? Cmnd_UR_READ_PAGE_FL:
      mchr == 'E' && rwop == Cmnd_STK_READ_PAGE? Cmnd_UR_READ_PAGE_EE: 0xff;

    if(op == 0xff)
      Return("command not recognised");

    *q++ = op;
    *q++ = badd & 0xff;
    *q++ = (badd >> 8) & 0xff;
    // Flash is larger than 64 kBytes, extend address (even for EEPROM)
    if(ur.uP.flashsize > 0x10000)
      *q++ = (badd >> 16) & 0xff;

    if(ur.uP.pagesize <= 256) {
      if(len > 256)
        Return("urprotocol paged r/w len %d cannot exceed 256", len);
      *q++ = len;               // len==256 is sent as 0
    } else {
      int max = ur.uP.pagesize > 256? ur.uP.pagesize: 256;
      if(len > max)
        Return("urprotocol paged r/w len %d cannot exceed %d for %s", len, max, ur.uP.name);
      *q++ = len>>8;            // Big endian length when needed
      *q++ = len;
    }
    i = q-buf;

  } else {
    int max = ur.uP.pagesize > 256? ur.uP.pagesize: 256;
    if(len > max)
      Return("stk500 paged r/w len %d cannot exceed %d for %s", len, max, ur.uP.name);

    buf[0] = rwop;
    buf[1] = len>>8;            // Big endian length when needed
    buf[2] = len;
    buf[3] = mchr;
    i = 4;
  }

  if(payload) {                 // Bytes to write
    if(len < 0 || len > (int) sizeof buf - 5)
      Return("too small buf[] for len %d (enlarge buf[] and recompile)", len);
    memcpy(buf+i, payload, len);
    i += len;
  }

  buf[i] = Sync_CRC_EOP;

  return urclock_send(pgm, buf, i+1);
}


/*
 * Read len bytes at byte address addr of EEPROM (mchr == 'E') or flash (mchr == 'F') from
 * device fd into buffer buf, using extended addressing if needed (extd); returns 0 on success
 */
static int ur_readEF(const PROGRAMMER *pgm, const AVRPART *p, uint8_t *buf, uint32_t badd, int len,
  char mchr) {

  int classic = !(p->prog_modes & (PM_UPDI | PM_PDI | PM_aWire));

  pmsg_debug("ur_readEF(%s, %s, %s, %p, 0x%06x, %d, %c)\n",
    pgmid, p->desc, mchr=='F'? "flash": "eeprom", buf, badd, len, mchr);

  if(mchr == 'F' && ur.urprotocol && !(ur.urfeatures & UB_READ_FLASH))
    Return("bootloader does not have flash read capability");

  if(mchr == 'E' && !ur.bleepromrw && !ur.xeepromrw)
    Return("bootloader %s not have EEPROM access%s", ur.blurversion? "does": "might",
      ur.blurversion? " capability": "; try -xeepromrw if it has");

  if(len < 1 || len > urmax(ur.uP.pagesize, 256))
    Return("len %d exceeds range [1, %d]", len, urmax(ur.uP.pagesize, 256));

  // Odd byte address under word-address protocol for "classic" parts (optiboot, avrisp etc)
  int odd = !ur.urprotocol && classic && (badd&1);
  if(odd) {                     // Need to read one extra byte
    len++;
    badd &= ~1;
    if(len > urmax(ur.uP.pagesize, 256))
      Return("len+1 = %d odd address exceeds range [1, %d]", len, urmax(ur.uP.pagesize, 256));
  }

  if(urclock_paged_rdwr(pgm, p, Cmnd_STK_READ_PAGE, badd, len, mchr, NULL) < 0)
    return -1;

  return urclock_res_check(pgm, __func__, odd, buf, len-odd);
}


static int parseUrclockID(const PROGRAMMER *pgm) {
  if(*ur.iddesc) {              // User override of ID, eg, -xid=F.-4.2 for penultimate flash word
    char *idstr = cfg_strdup(__func__, ur.iddesc), *idlenp;
    const char *errstr;
    int ad, lg;

    if(!(strchr("EF", *idstr) && idstr[1] == '.')) {
      pmsg_warning("-xid=%s string must start with E. or F.\n", ur.iddesc);
      free(idstr);
      return -1;
    }

    if(!(idlenp = strchr(idstr+2, '.'))) {
      pmsg_warning("-xid=%s string must look like [E|F].<addr>.<len>\n", ur.iddesc);
      free(idstr);
      return -1;
    }
    *idlenp++ = 0;
    ad = str_int(idstr+2, STR_INT32, &errstr);
    if(errstr) {
      pmsg_warning("address %s of -xid=%s: %s\n", idstr+2, ur.iddesc, errstr);
      free(idstr);
      return -1;
    }

    lg = str_int(idlenp, STR_INT32, &errstr);
    if(errstr) {
      pmsg_warning("length %s of -xid=%s string: %s\n", idlenp, ur.iddesc, errstr);
      free(idstr);
      return -1;
    }
    if(!lg || lg > 8) {
      pmsg_warning("length %s of -xid=%s string must be between 1 and 8\n", idlenp, ur.iddesc);
      free(idstr);
      return -1;
    }

    ur.idmchr = *idstr;
    ur.idaddr = ad;
    ur.idlen = lg;

    free(idstr);
  }

  return 0;
}


static int readUrclockID(const PROGRAMMER *pgm, const AVRPART *p, uint64_t *urclockIDp) {
  uint8_t spc[16];
  int mchr, addr, len, size;

  if(ur.idlen)
    mchr = ur.idmchr, addr = ur.idaddr, len = ur.idlen;
  else
    mchr = 'E', addr = 256+1, len = 6; // Default location for unique id on urclock boards

  *urclockIDp = 0;

  // Sanity for small boards in absence of user -xid=... option
  if(!ur.idlen && (addr >= ur.uP.eepromsize || addr+len > ur.uP.eepromsize)) {
    addr = 0;
    if(ur.uP.eepromsize < 8)
      mchr = 'F';
  }

  const char *memstr = mchr == 'E'? "eeprom": "flash";

  size = mchr == 'F'? ur.uP.flashsize: ur.uP.eepromsize;

  if(ur.uP.name && size > 0) {
    if(addr < 0)                // X.-4.4 asks for 4 bytes at top memory
      addr += size;

    if(addr < 0 || addr >= size)
      Return("effective address %d of -xids=%s string out of %s range [0, 0x%04x]\n",
        addr, ur.iddesc, memstr, size-1);

    if(addr+len > size)
      Return("memory range [0x%04x, 0x%04x] of -xid=%s out of %s range [0, 0x%04x]\n",
        addr, addr+len-1, ur.iddesc, memstr, size-1);
  }

  memset(spc, 0, sizeof spc);
  if(mchr == 'E' && !ur.bleepromrw && !ur.xeepromrw)
    return -2;

  if(ur_readEF(pgm, p, spc, addr, len, mchr) < 0)
    return -1;

  // Urclock ID
  for(int i = len-1; i >= 0; i--)
    *urclockIDp <<= 8, *urclockIDp |= spc[i];
  ur.idlen = len;

  return 0;
}


static int urclock_send(const PROGRAMMER *pgm, unsigned char *buf, size_t len) {
  return serial_send(&pgm->fd, buf, len);
}


static int urclock_recv(const PROGRAMMER *pgm, unsigned char *buf, size_t len) {
  int rv;

  rv = serial_recv(&pgm->fd, buf, len);
  if(rv < 0) {
    if(ur.sync_silence < 2)
      pmsg_warning("programmer is not responding%s\n",
        ur.sync_silence? "; try -xstrict and/or vary -xdelay=100": "");
    return -1;
  }

  return 0;
}


#define MAX_SYNC_ATTEMPTS 16

/*
 * The modified protocol makes stk_insync and stk_ok responses variable but fixed for a single
 * programming session, so bootloader can pass on en passant 16 bit info about which part it was
 * compiled for and some of its own properties. Urlcock_getsync() therefore tries a few times to
 * sync until the stk_insync/ok responses coincide with the the most recent responses.
 */
static int urclock_getsync(const PROGRAMMER *pgm) {
  unsigned char iob[2], autobaud_sync;
  int attempt;
  AVRPART *part;

  // Reduce timeout for establishing comms
  serial_recv_timeout = 25;     // ms
  part = partdesc? locate_part(part_list, partdesc): NULL;
  /*
   * The urboot autosync detection uses a loop
   *
   * 2: adiw  r26, 32
   *    sbis  RX_Pin_Port,RX_Bit
   *    rjmp  2
   *
   * The number of cycles in this loop must be the position of the least significant set bit of the
   * first byte sent by AVRDUDE. For a 5-cycle loop (ATmega*) the fifth-lowest bit must be set and
   * the four least significant bit unset, eg, 0x30, which by coincidence is Cmnd_STK_GET_SYNC. For
   * ATxmega* parts, the sync byte could be 0x20. For LGT8F* parts this loop has three cycles, so
   * 0x1c would be appropriate. Care must be taken to not choose a sync byte that is an otherwise
   * legitimate command, ie nothing avove 0x30 or below 0x10 should be chosen.
   */
  autobaud_sync = part && part->autobaud_sync? part->autobaud_sync: Cmnd_STK_GET_SYNC;

  ur.sync_silence = 2;
  serial_drain_timeout = 20;    // ms

  for(attempt = 0; attempt < MAX_SYNC_ATTEMPTS; attempt++) {
    /*
     * The initial byte for autobaud must be the sync byte/Sync_CRC_EOP sequence; thereafter it
     * should normally be Cmnd_STK_GET_SYNC/Sync_CRC_EOP. However, both urboot and optiboot are
     * "permissive" as to the get sync command: anything that is not a valid, known command is
     * acceptable. However, these are less permissive when it comes to the End of command byte
     * Sync_CRC_EOP: if that is wrong the bootloader swiftly enters the application. Old but
     * popular optiboot v4.4 first initialises the USART and *then* entertains the user for 300 ms
     * with a flashing LED, which means that AVRDUDE's initial 2-byte sync sequence will appear as
     * one byte Sync_CRC_EOP (because the first byte is overwritten) getting the communication out
     * of step through a missing byte. If AVRDUDE then sends the next request starting with a
     * Cmnd_STK_GET_SYNC command then optiboot v4.4 will bail as ist's not Sync_CRC_EOP. Hence, the
     * strategy here is to send Sync_CRC_EOP/Sync_CRC_EOP for getting a sync. For those bootloaders
     * that are strict about the protocol, eg, picoboot, the presence of -xstrict implies that
     * comms should use Cmnd_STK_GET_SYNC for getting in sync.
     */
    iob[0] = attempt == 0? autobaud_sync: ur.strict? Cmnd_STK_GET_SYNC: Sync_CRC_EOP;
    iob[1] = Sync_CRC_EOP;
    urclock_send(pgm, iob, 2);
    if(urclock_recv(pgm, iob, 2) == 0) { // Expect bootloader to respond with two bytes
      if(!ur.gs.seen || iob[0] != ur.gs.stk_insync || iob[1] != ur.gs.stk_ok || iob[0] == iob[1]) {
        ur.gs.stk_insync = iob[0];
        ur.gs.stk_ok = iob[1];
        serial_drain(&pgm->fd, 0); // Drain periodically to guard against initial line noise
        ur.gs.seen = 1;
      } else
        break;
    } else {                    // Board not yet out of reset or bootloader twiddles lights
      int slp = 32<<(attempt<3? attempt: 3);
      pmsg_debug("%4ld ms: sleeping for %d ms\n", avr_mstimestamp(), slp);
      usleep(slp*1000);
    }
    if(attempt > 5) {           // Don't report first six attempts
      if(attempt == MAX_SYNC_ATTEMPTS-1)
        ur.sync_silence = 1;
      pmsg_warning("attempt %d of %d: not in sync\n", attempt - 5, MAX_SYNC_ATTEMPTS-6);
    }
  }

  if(!ur.strict) {              // Could be out of step by one byte
    iob[0] = Sync_CRC_EOP;
    urclock_send(pgm, iob, 1);  // If so must send EOP
    if(urclock_recv(pgm, iob, 1) < 0) {
      iob[0] = Sync_CRC_EOP;    // No reply: we were not out of step, but are now
      urclock_send(pgm, iob, 1); // So, send the concluding byte
    }
  }
  serial_drain(&pgm->fd, 0);  // And either way drain the reply

  ur.sync_silence = 0;

  serial_recv_timeout = 500;    // ms

  if(attempt == MAX_SYNC_ATTEMPTS)
    return -1;

  ur.STK_INSYNC = ur.gs.stk_insync;
  ur.STK_OK     = ur.gs.stk_ok;
  memset(&ur.uP, 0, sizeof ur.uP);

  // One of the STK500 protocol bytes different from ordinary? If so, it's the urboot protocol
  if(ur.gs.stk_insync != Resp_STK_INSYNC || ur.gs.stk_ok != Resp_STK_OK) {
    // Regain urboot info from stk_insync and stk_ok
    if(ur.gs.stk_insync == 255 && ur.gs.stk_ok == 254) {
      ur.gs.stk_insync = Resp_STK_INSYNC;
      ur.gs.stk_ok = Resp_STK_OK;
    } else if(ur.gs.stk_ok > ur.gs.stk_insync)
      ur.gs.stk_ok--;

    int16_t bootinfo = ur.gs.stk_insync*255 + ur.gs.stk_ok;
    int mcuid = UB_MCUID(bootinfo);
    ur.urfeatures = UB_FEATURES(bootinfo);
    ur.urprotocol = 1;

    set_uP(pgm, part, mcuid, 1);
    if(!ur.uP.name)
      Return("cannot identify MCU");
    if(!partdesc)               // Provide partdesc info, so user does not have to set it
      partdesc = cache_string(ur.uP.name);
  } else {
    ur.urprotocol = 0;
    if(partdesc) {              // Initialise uP from command line for now
      set_uP(pgm, locate_part(part_list, partdesc), -1, 0);
      if(!ur.uP.name)
        Return("cannot identify MCU from partdesc %s", partdesc);
    }
  }

  return 0;
}


/*
 * The urclock bootloader ignores all but two STK_UNIVERSAL commands (load extended address and
 * chip erase) , so only sending these through. Transmits the device command and returns the
 * results; 'cmd' and 'res' must point to at least a 4 byte data buffer
 */
static int urclock_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  if(cmd[0] == (Subc_STK_UNIVERSAL_LEXT>>24) ||
    (cmd[0] == (Subc_STK_UNIVERSAL_CE>>24) && cmd[1] == (uint8_t)(Subc_STK_UNIVERSAL_CE>>16))) {

    unsigned char buf[32];

    buf[0] = Cmnd_STK_UNIVERSAL;
    buf[1] = cmd[0];
    buf[2] = cmd[1];
    buf[3] = cmd[2];
    buf[4] = cmd[3];
    buf[5] = Sync_CRC_EOP;

    if(urclock_send(pgm, buf, 6) < 0)
      return -1;
    if(urclock_recv(pgm, buf, 1) < 0)
      return -1;
    if(buf[0] != ur.STK_INSYNC) {
      pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", ur.STK_INSYNC, buf[0]);
      return -1;
    }

    res[0] = cmd[1];
    res[1] = cmd[2];
    res[2] = cmd[3];
    if(urclock_recv(pgm, &res[3], 1) < 0)
      return -1;

    if(urclock_recv(pgm, buf, 1) < 0)
      return -1;
    if(buf[0] != ur.STK_OK) {
      pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", ur.STK_OK, buf[0]);
      return -1;
    }

  } else {
    // All other requests: pretend call happened and all is good, returning 0xff each time
    memcpy(res, cmd+1, 3);
    res[3] = 0xff;
  }

  return 0;
}


// Either emulate chip erase or send appropriate command to bootloader
static int urclock_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char buf[16];
  long bak_timeout = serial_recv_timeout;

  // Set timeout to 20 ms per page as chip erase may take a long time
  serial_recv_timeout = ur.uP.pagesize > 2? 500 + ur.uP.flashsize/ur.uP.pagesize * 20: 20000;

  int emulated = 0;

  if(ur.xemulate_ce ||
    (ur.urprotocol && !(ur.urfeatures & UB_CHIP_ERASE)) ||
    ur.bloptiversion || (ur.blurversion && ur.blurversion < 076)) {

    // Bootloader does not implement chip erase: don't send command to bootloader
    ur.emulate_ce = 1;
    emulated = 1;

  } else if(ur.urprotocol) {    // Urprotocol uses chip erase command directly
    pmsg_notice2("chip erase via urprotocol\n");

    buf[0] = Cmnd_STK_CHIP_ERASE;
    buf[1] = Sync_CRC_EOP;

    if(urclock_send(pgm, buf, 2) < 0 || urclock_res_check(pgm, __func__, 0, NULL, 0) < 0) {
      serial_recv_timeout = bak_timeout;
      return -1;
    }
  } else {                      // Legacy bootloaders use universal extension
    pmsg_notice2("chip erase via universal STK500v1 command\n");

    if(pgm->cmd == NULL) {      // Should not happen
      pmsg_error("%s programmer does not provide a cmd() method\n", pgm->type);
      serial_recv_timeout = bak_timeout;
      return -1;
    }

    memset(buf, 0, sizeof(buf));

    buf[0] = (uint8_t) (Subc_STK_UNIVERSAL_CE>>24);
    buf[1] = (uint8_t) (Subc_STK_UNIVERSAL_CE>>16);
    buf[2] = (uint8_t) (Subc_STK_UNIVERSAL_CE>>8);
    buf[3] = (uint8_t) (Subc_STK_UNIVERSAL_CE);

    if(urclock_cmd(pgm, buf, buf+4) < 0) {
      serial_recv_timeout = bak_timeout;
      return -1;
    }
  }

  serial_recv_timeout = bak_timeout;
  ur.done_ce = 1;

  if(!emulated) {               // Write jump to boot section to reset vector
    if(ur.boothigh && ur.blstart && ur.vbllevel==1) {
      AVRMEM *flm = avr_locate_flash(p);
      int vecsz = ur.uP.flashsize <= 8192? 2: 4;
      if(flm && flm->page_size >= vecsz) {
        unsigned char *page = cfg_malloc(__func__, flm->page_size);
        memset(page, 0xff, flm->page_size);
        set_reset(pgm, page, vecsz);
        if(avr_write_page_default(pgm, p, flm, 0, page) < 0) {
          free(page);
          return -1;
        }
        free(page);
      }
    }
  }

  return emulated? LIBAVRDUDE_SOFTFAIL: 0;
}


// Issue the 'program enable' command to the AVR device
static int urclock_program_enable(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  unsigned char buf[16];

  buf[0] = Cmnd_STK_ENTER_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  if(urclock_send(pgm, buf, 2) < 0)
    return -1;

  return urclock_res_check(pgm, __func__, 0, NULL, 0);
}


static void urclock_enable(PROGRAMMER *pgm, const AVRPART *p) {
  AVRMEM *mem;
  if((mem = avr_locate_eeprom(p)))
    if(mem->page_size == 1)     // Change EEPROM page size from 1 to 16 to force paged r/w
      mem->page_size = 16;
  return;
}


// Initialise the AVR device and prepare it to accept commands
static int urclock_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  return pgm->program_enable(pgm, p);
}


static void urclock_disable(const PROGRAMMER *pgm) {
  unsigned char buf[16];

  buf[0] = Cmnd_STK_LEAVE_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  if(urclock_send(pgm, buf, 2) < 0)
    return;
  if(urclock_recv(pgm, buf, 1) < 0)
    return;
  if(buf[0] != ur.STK_INSYNC) {
    pmsg_error("protocol expects sync byte 0x%02x but got 0x%02x\n", ur.STK_INSYNC, buf[0]);
    return;
  }

  if(urclock_recv(pgm, buf, 1) < 0)
    return;
  if(buf[0] == ur.STK_OK)
    return;

  pmsg_error("protocol expects OK byte 0x%02x but got 0x%02x\n", ur.STK_OK, buf[0]);
  return;
}


static int urclock_open(PROGRAMMER *pgm, const char *port) {
  union pinfo pinfo;

  strcpy(pgm->port, port);
  pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: 115200;
  pinfo.serialinfo.cflags = SERIAL_8N1;
  if(serial_open(port, pinfo, &pgm->fd) == -1)
    return -1;

  // This code assumes a negative-logic USB to TTL serial adapter
  // Set RTS/DTR high to discharge the series-capacitor, if present
  serial_set_dtr_rts(&pgm->fd, 0);
  usleep(20*1000);
  // Pull the RTS/DTR line low to reset AVR
  serial_set_dtr_rts(&pgm->fd, 1);
  // Max 100 us: charging a cap longer creates a high reset spike above Vcc
  usleep(100);
  // Set the RTS/DTR line back to high, so direct connection to reset works
  serial_set_dtr_rts(&pgm->fd, 0);

  if((120+ur.delay) > 0)
    usleep((120+ur.delay)*1000); // Wait until board comes out of reset

  pmsg_debug("%4ld ms: enter urclock_getsync()\n", avr_mstimestamp());
  if(urclock_getsync(pgm) < 0)
    return -1;
  pmsg_debug("%4ld ms: all good, ready to rock\n", avr_mstimestamp());

  return 0;
}


static void urclock_close(PROGRAMMER *pgm) {
  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
  if(ur.bloptiversion)          // Optiboot needs a pause between two successive avrdude calls
    usleep(200*1000);
}


static int urclock_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  int mchr, chunk;
  unsigned int n;

  if(n_bytes) {
    // Paged writes only valid for flash and eeprom
    mchr = mem_is_in_flash(m)? 'F': 'E';
    if(mchr == 'E' && !mem_is_eeprom(m))
      return -2;

    if(mchr == 'E' && !ur.bleepromrw && !ur.xeepromrw)
      Return("bootloader %s not have paged EEPROM write%s", ur.blurversion? "does": "might",
        ur.blurversion? " capability": ", try -xeepromrw if it has");

    n = addr + n_bytes;

    for(; addr < n; addr += chunk) {
      chunk = n-addr < page_size? n-addr: page_size;

      if(urclock_paged_rdwr(pgm, p, Cmnd_STK_PROG_PAGE, addr, chunk, mchr, (char *) m->buf+addr)<0)
        return -3;
      if(urclock_res_check(pgm, __func__, 0, NULL, 0) < 0)
        return -4;
    }
  }

  return n_bytes;
}


static int urclock_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  int mchr, chunk;
  unsigned int n;

  if(n_bytes) {
    // Paged reads only valid for flash and eeprom
    mchr = mem_is_in_flash(m)? 'F': 'E';
    if(mchr == 'E' && !mem_is_eeprom(m))
      return -2;

    if(mchr == 'F' && ur.urprotocol && !(ur.urfeatures & UB_READ_FLASH))
      Return("bootloader does not have flash read capability");

    if(mchr == 'E' && !ur.bleepromrw && !ur.xeepromrw)
      Return("bootloader %s not have paged EEPROM read%s", ur.blurversion? "does": "might",
        ur.blurversion? " capability": "; try -xeepromrw if it has");

    n = addr + n_bytes;
    for(; addr < n; addr += chunk) {
      chunk = n-addr < page_size? n-addr: page_size;

      if(urclock_paged_rdwr(pgm, p, Cmnd_STK_READ_PAGE, addr, chunk, mchr, NULL) < 0)
        return -3;
      if(urclock_res_check(pgm, __func__, 0, &m->buf[addr], chunk) < 0)
        return -4;

      if(addr == 0 && mchr == 'F') { // Ensure reset vector points to bl
        int vecsz = ur.uP.flashsize <= 8192? 2: 4;
        if(chunk >= vecsz && ur.boothigh && ur.blstart && ur.vbllevel == 1) {
          unsigned char jmptoboot[4];
          int resetsize = set_reset(pgm, jmptoboot, vecsz);
          int resetdest;

          if(reset2addr(m->buf, vecsz, ur.uP.flashsize, &resetdest) < 0 || resetdest != ur.blstart) {
            memcpy(m->buf, jmptoboot, resetsize);
            pmsg_info("en passant forcing reset vector to point to vector bootloader\n");
            if(urclock_paged_rdwr(pgm, p, Cmnd_STK_PROG_PAGE, 0, chunk, mchr, (char *) m->buf) < 0)
              return -5;
            if(urclock_res_check(pgm, __func__, 0, NULL, 0) < 0)
              return -6;
          }
        }
      }
    }
  }

  return n_bytes;
}


int urclock_write_byte(const PROGRAMMER *pgm_uu, const AVRPART *p_uu, const AVRMEM *mem,
  unsigned long addr_uu, unsigned char data_uu) {

  pmsg_error("bootloader does not implement bytewise write to %s \n", mem->desc);
  return -1;
}

int urclock_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char *value) {

  // Bytewise read only valid for flash and eeprom
  int mchr = mem_is_in_flash(mem)? 'F': 'E';
  if(mchr == 'E' && !mem_is_eeprom(mem)) {
    if(mem_is_signature(mem) && pgm->read_sig_bytes) {
       if((int) addr < 0 || (int) addr >= mem->size) {
         return -1;
       }
       pgm->read_sig_bytes(pgm, p, mem);
       *value = mem->buf[(int) addr];
       return 0;
    }
    pmsg_error("bootloader cannot read from %s \n", mem->desc);
    return -1;
  }

  return ur_readEF(pgm, p, value, (uint32_t) addr, 1, mchr);
}

// Periodic call in terminal mode to keep bootloader alive
static int urclock_term_keep_alive(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  unsigned char buf[16];

  buf[0] = Cmnd_STK_GET_SYNC;
  buf[1] = Sync_CRC_EOP;

  if(urclock_send(pgm, buf, 2) < 0)
    return -1;

  return urclock_res_check(pgm, __func__, 0, NULL, 0);
}


// Display what we know so far (too early in the process to say much)
static void urclock_display(const PROGRAMMER *pgm, const char *p_unused) {
  if(ur.urprotocol) {
    imsg_info("Urboot protocol for %s\n", ur.uP.name);
  } else {
    imsg_info("Bootloader using STK500v1 communication protocol\n");
  }

  return;
}

// End of STK500 section


// Return whether an address is write protected
static int urclock_readonly(const struct programmer_t *pgm, const AVRPART *p_unused,
  const AVRMEM *mem, unsigned int addr) {

  if(mem_is_in_flash(mem)) {
    if(addr > (unsigned int) ur.pfend)
      return 1;
    if(addr < (unsigned int) ur.pfstart)
      return 1;
    if(ur.boothigh && addr < 512 && ur.vbllevel) {
      unsigned int vecsz = ur.uP.flashsize <= 8192? 2u: 4u;
      if(addr < vecsz)
        return 1;
      if(ur.vblvectornum > 0) {
        unsigned int appvecloc = ur.vblvectornum*vecsz;
        if(addr >= appvecloc && addr < appvecloc+vecsz)
          return 1;
      }
    }
  } else if(!mem_is_eeprom(mem))
    return 1;

  return 0;
}

static int urclock_parseextparms(const PROGRAMMER *pgm, LISTID extparms) {
  int help = 0;

  struct {
    const char *name;
    int *optionp;
    int nstrbuf;
    char *strbuf;
    bool assign;
    const char *help;
  } options[] = {
#define ARG  0, NULL, 1
#define NA   0, NULL, 0
    {"showall", &ur.showall, NA,          "Show all info for connected part and exit"},
    {"showid", &ur.showid, NA,            "Show Urclock ID and exit"},
    {"showdate", &ur.showdate, NA,        "Show last-modified date of flash application and exit"},
    {"showfilename", &ur.showfilename, NA,"Show filename of last uploaded application and exit"},
    {"showapp", &ur.showapp, NA,          "Show application size and exit"},
    {"showstore", &ur.showstore, NA,      "Show store size and exit"},
    {"showmeta", &ur.showmeta, NA,        "Show metadata size and exit"},
    {"showboot", &ur.showboot, NA,        "Show bootloader size and exit"},
    {"showversion", &ur.showversion, NA,  "Show bootloader version and capabilities and exit"},
    {"showvector", &ur.showvector, NA,    "Show vector bootloader vector # and name and exit"},
    {"id", NULL, sizeof ur.iddesc, ur.iddesc, 1, "Location of Urclock ID, eg, F.12345.6"},
    {"title", NULL, sizeof ur.title, ur.title, 1, "Title stored and shown in lieu of a filename"},
    {"bootsize", &ur.xbootsize, ARG,      "Override/set bootloader size"},
    {"vectornum", &ur.xvectornum, ARG,    "Treat bootloader as vector b/loader using this vector"},
    {"eepromrw", &ur.xeepromrw, NA,       "Assert bootloader EEPROM read/write capability"},
    {"emulate_ce", &ur.xemulate_ce, NA,   "Emulate chip erase"},
    {"restore", &ur.restore, NA,          "Restore a flash backup and trim the bootloader"},
    {"initstore", &ur.initstore, NA,      "Fill store with 0xff on writing to flash"},
    //@@@  {"copystore", &ur.copystore, NA, "Copy over store on writing to flash"},
    {"nofilename", &ur.nofilename, NA,    "Do not store filename on writing to flash"},
    {"nodate", &ur.nodate, NA,            "Do not store application filename and no date either"},
    {"nostore", &ur.nostore, NA,          "Do not store metadata except a flag saying so"},
    {"nometadata", &ur.nometadata, NA,    "Do not support metadata at all"},
    {"delay", &ur.delay, ARG,             "Add delay [ms] after reset, can be negative"},
    {"strict", &ur.strict, NA,            "Use strict synchronisation protocol"},
    {"help", &help, NA,                   "Show this help menu and exit"},
  };

  int rc = 0;
  for(LNODEID ln = lfirst(extparms); ln; ln = lnext(ln)) {
    const char *extended_param = ldata(ln);
    size_t i, olen, plen = strlen(extended_param);

    for(i=0; i<sizeof options/sizeof*options; i++) {
      olen = strlen(options[i].name);
      if(strncmp(extended_param, options[i].name, olen) == 0) {
        if(!options[i].nstrbuf) {
          if(plen == olen && !options[i].assign) {
            if(options[i].optionp) {
              if(*options[i].optionp < 0)
                 *options[i].optionp = 0;
              (*options[i].optionp)++;
              pmsg_notice2("%s set\n", options[i].name);
            }
            break;
          } else if(plen > olen &&  extended_param[olen] == '=' && options[i].assign) {
            const char *arg = extended_param+olen+1;
            const char *errstr;
            int val = str_int(arg, STR_INT32, &errstr);
            if(errstr) {
             pmsg_error("-x%s: %s\n", extended_param, errstr);
             return -1;
            }
            *options[i].optionp = val;
            pmsg_notice2("%s=%d set\n", options[i].name, (int) val);
            break;
          }
        } else if(options[i].nstrbuf > 0) {
          if(plen <= olen || extended_param[olen] != '=') {
            pmsg_error("missing argument for option %s=...\n", extended_param);
            rc = -1;
          } else {
            if(options[i].strbuf) {
              strncpy(options[i].strbuf, extended_param+olen+1, options[i].nstrbuf-1);
              pmsg_notice2("%s=%s set\n", options[i].name, options[i].strbuf);
            }
            break;
          }
        }
      }
    }
    if(i >= sizeof options/sizeof*options) {
      pmsg_error("invalid extended parameter %s\n", extended_param);
      rc = -1;
    }
  }

  if(help || rc < 0) {
    msg_error("%s -c %s extended options:\n", progname, pgmid);
    for(size_t i=0; i<sizeof options/sizeof*options; i++) {
      msg_error("  -x%s%s%*s%s\n", options[i].name, options[i].assign? "=<arg>": "",
        urmax(0, 16-(int) strlen(options[i].name)-(options[i].assign? 6: 0)), "", options[i].help);
    }
    if(rc == 0)
      exit(0);
  }

  if(parseUrclockID(pgm) < 0)
    return -1;

  return rc;
}


static void urclock_setup(PROGRAMMER *pgm) {
  // Allocate ur
  pgm->cookie = cfg_malloc(__func__, sizeof(Urclock_t));

  ur.xvectornum    = -1;        // Initialise, to ascertain whether user had set to 0
  ur.ext_addr_byte = 0xff;      // So first memory address will load extended address
  ur.STK_INSYNC    = Resp_STK_INSYNC;
  ur.STK_OK        = Resp_STK_OK;
}


static void urclock_teardown(PROGRAMMER *pgm) {
  free(pgm->cookie);
  pgm->cookie = NULL;
}


const char urclock_desc[] = "Urclock programmer for urboot bootloaders (arduino compatible)";

void urclock_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "Urclock");

  pgm->read_sig_bytes = urclock_read_sig_bytes;

  // Mandatory functions
  pgm->initialize = urclock_initialize;
  pgm->display = urclock_display;
  pgm->enable = urclock_enable;
  pgm->disable = urclock_disable;
  pgm->program_enable = urclock_program_enable;
  pgm->chip_erase = urclock_chip_erase;
  pgm->cmd = urclock_cmd;
  pgm->open = urclock_open;
  pgm->close = urclock_close;
  pgm->read_byte = urclock_read_byte;
  pgm->write_byte = urclock_write_byte;

  // Optional functions
  pgm->paged_write = urclock_paged_write;
  pgm->paged_load = urclock_paged_load;
  pgm->setup = urclock_setup;
  pgm->teardown = urclock_teardown;
  pgm->parseextparams = urclock_parseextparms;
  pgm->term_keep_alive = urclock_term_keep_alive;
  pgm->readonly = urclock_readonly;
  pgm->flash_readhook = urclock_flash_readhook;

  disable_trailing_ff_removal();
#if defined(HAVE_LIBREADLINE)
  pmsg_notice2("libreadline is used; avrdude -t -c urclock should work interactively\n");
#else
  pmsg_notice2("compiled without readline library, cannot use avrdude -t -c urclock interactively\n");
  imsg_notice2("but it is still possible to pipe: echo \"d fl 0 32; quit\" | tr \\; \\\\n | avrdude -t -curclock\n");
#endif
}
