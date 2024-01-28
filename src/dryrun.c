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
 * The dryrun programmer emulates a physical programmer by allocating a copy of the part and
 * pretending all operations work well.
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "dryrun.h"
#include "dryrun_private.h"

// Context of the programmer
typedef enum {
  DRY_NOBOOTLOADER,             // No bootloader, taking to an ordinary programmer
  DRY_TOP,                      // Bootloader and it sits at top of flash
  DRY_BOTTOM,                   // Bootloader sits at bottom of flash (UPDI parts)
} dry_prog_t;

typedef struct {
  AVRPART *dp;
  dry_prog_t bl;                // Bootloader and, if so, at top/bottom of flash?
  int blsize;                   // Bootloader size min(flash size/4, 512)
} dryrun_t;

// Use private programmer data as if they were a global structure dry
#define dry (*(dryrun_t *)(pgm->cookie))

#define Return(...) do { pmsg_error(__VA_ARGS__); msg_error("\n"); return -1; } while (0)

static int dryrun_readonly(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int addr);

// Read expected signature bytes from part description
static int dryrun_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *sigmem) {
  pmsg_debug("%s()", __func__);
  // Signature byte reads are always 3 bytes
  if(sigmem->size < 3)
    Return("memory size too small for %s()", __func__);

  memcpy(sigmem->buf, p->signature, 3);
  msg_debug(" returns 0x%02x%02x%02x\n", sigmem->buf[0], sigmem->buf[1], sigmem->buf[2]);
  return 3;
}


// Emulate chip erase (only erase flash, pretend EESAVE fuse is active - FIXME: check EESAVE fuse)
static int dryrun_chip_erase(const PROGRAMMER *pgm, const AVRPART *punused) {
  AVRMEM *flm;

  pmsg_debug("%s()\n", __func__);
  if(!dry.dp)
    Return("no dryrun device?");
  if(!(flm = avr_locate_flash(dry.dp)))
    Return("cannot locate %s flash memory for chip erase", dry.dp->desc);
  if(flm->size < 1)
    Return("cannot erase %s flash memory owing to its size %d", dry.dp->desc, flm->size);

  memset(flm->buf, 0xff, flm->size);

  return 0;
}


// For now pretend all is hunky-dory
static int dryrun_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  int ret = 0;

  pmsg_debug("%s(0x%02x 0x%02x 0x%02x 0x%02x)\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
  // FIXME: do we need to emulate some more commands? For now it's only the STK universal CE
  if(cmd[0] == (Subc_STK_UNIVERSAL_LEXT>>24) ||
    (cmd[0] == (Subc_STK_UNIVERSAL_CE>>24) && cmd[1] == (uint8_t)(Subc_STK_UNIVERSAL_CE>>16))) {

    ret = dryrun_chip_erase(pgm, NULL);
  }
  // Pretend call happened and all is good, returning 0xff each time
  memcpy(res, cmd+1, 3);
  res[3] = 0xff;

  return ret;
}


static int dryrun_program_enable(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  pmsg_debug("%s()\n", __func__);

  return 0;
}


static void dryrun_enable(PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);

  AVRMEM *flm = avr_locate_flash(p);
  if(flm && flm->size >= 1024) {
    if(pgm->prog_modes & PM_SPM)
      dry.bl = (p->prog_modes & PM_UPDI)? DRY_BOTTOM: DRY_TOP;
    dry.blsize = flm->size/4;
    if(dry.blsize > 512)
      dry.blsize = 512;
  }

  if(!dry.dp) {
    unsigned char inifuses[16]; // For fuses, which is made up from fuse0, fuse1, ...
    AVRMEM *fusesm = NULL, *prodsigm = NULL, *calm;
    dry.dp = avr_dup_part(p);   // Allocate dryrun part

    memset(inifuses, 0xff, sizeof inifuses);
    // Initialise the device with factory setting and erase flash/EEPROM to 0xff
    for (LNODEID ln=lfirst(dry.dp->mem); ln; ln=lnext(ln)) {
      AVRMEM *m = ldata(ln);
      if(mem_is_in_flash(m) || mem_is_eeprom(m)) {
        memset(m->buf, 0xff, m->size);
        // Overwrite ficticious bootloader section with block of endless loops
        if(dry.bl && (mem_is_boot(m) || mem_is_flash(m)))
          for(int i = dry.bl == DRY_TOP? m->size-dry.blsize: 0, end = i+dry.blsize, n = 0; i+1 < end; i+=2, n++)
            m->buf[i] = 255-n, m->buf[i+1] = 0xcf; // rjmp .-2, rjmp .-4, ...
      } else if(mem_is_fuses(m)) {
        fusesm = m;
      } else if(mem_is_a_fuse(m) || mem_is_lock(m)) {
        // Lock, eg, can have 4 bytes: still allow initialisation from initval
        if(m->initval != -1 && m->size >=1 && m->size <= (int) sizeof(m->initval)) {
          memcpy(m->buf, &m->initval, m->size); // FIXME: relying on little endian here
          if(mem_is_a_fuse(m)) {
            int fno = mem_fuse_offset(m);
            for(int i = 0; i < m->size && fno+i < (int) sizeof inifuses; i++) // pdicfg has 2 bytes
              inifuses[fno+i] = m->initval >> 8*i;
          }
        } else {
          memset(m->buf, 0xff, m->size);
        }
      } else if(mem_is_signature(m) && (int) sizeof(dry.dp->signature) == m->size) {
        memcpy(m->buf, dry.dp->signature, m->size);
      } else if(mem_is_calibration(m)) {
        memset(m->buf, 'U', m->size); // 'U' for uncalibrated or unknown :)
      } else if(mem_is_osc16err(m)) {
        memset(m->buf, 'e', m->size);
      } else if(mem_is_osc20err(m)) {
        memset(m->buf, 'E', m->size);
      } else if(mem_is_osccal16(m)) {
        memset(m->buf, 'o', m->size);
      } else if(mem_is_osccal20(m)) {
        memset(m->buf, 'O', m->size);
      } else if(mem_is_sib(m)) {
        memset(m->buf, 'S', m->size);
      } else if( mem_is_tempsense(m)) {
        memset(m->buf, 'T', m->size); // 'T' for temperature calibration values
      } else if(mem_is_sernum(m)) {
        for(int i = 0; i < m->size; i++) // Set serial number UTSRQPONM...
          m->buf[i] = 'U'-i >= 'A'? 'U'-i: 0xff;
      } else if(mem_is_sigrow(m) && m->size >= 6) {
        prodsigm = m;
        memset(m->buf, 0xff, m->size);
        if(p->prog_modes & PM_PDI) {
          m->buf[0] = m->buf[1] = 'U';
        } else if(!(p->prog_modes & PM_UPDI)) { // Classic parts: signature at even addresses
          for(int i=0; i<3; i++)
            m->buf[2*i] = dry.dp->signature[i];
        }
      } else if(mem_is_io(m)) { // Initialise reset values (if known)
        int nr;
        const Register_file_t *rf = avr_locate_register_file(p, &nr);
        if(rf)
          for(int i = 0; i < nr; i++)
            if(rf[i].initval != -1 && rf[i].size > 0 && rf[i].size < 5)
              if(rf[i].addr >= 0 && rf[i].addr+rf[i].size <= m->size)
                for(int k = 0; k < rf[i].size; k++) // FIXME: Assume little endian compiler
                  m->buf[rf[i].addr+k] = ((unsigned char *) &rf[i].initval)[k];
      }
    }
    if(prodsigm) {
      if(p->prog_modes & PM_UPDI) {
        for (LNODEID ln=lfirst(dry.dp->mem); ln; ln=lnext(ln)) {
          AVRMEM *m = ldata(ln);
          if(m->buf == prodsigm->buf) // Skip prodsig memory
            continue;
          int off = m->offset - prodsigm->offset;
          int cpy = m->size;
          // Submemory of prodsig, eg, signature and tempsense? Copy into prodsig
          if(off >= 0 && off+cpy <= prodsigm->size)
            memcpy(prodsigm->buf + off, m->buf, cpy);
        }
      }
      if(!(p->prog_modes & (PM_PDI|PM_UPDI)) && (calm = avr_locate_calibration(dry.dp))) {
        // Calibration bytes of classic parts are interspersed with signature
        for(int i=0; i<calm->size; i++)
          if(2*i+1 < prodsigm->size)
            prodsigm->buf[2*i+1] = 'U';
      }
    }
    if(fusesm) {
      size_t fusz = fusesm->size;
      memcpy(fusesm->buf, inifuses, fusz < sizeof inifuses? fusz: sizeof inifuses);
    }
  }

  return;
}


// Initialise the AVR device and prepare it to accept commands
static int dryrun_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);

/*
 * Normally one would select appropriate programming mechanisms here,
 * but for dryrun ignore discrepancies...

  int pm = pgm->prog_modes & p->prog_modes;

  if(!pm)
    Return("programmer %s and part %s have no common programming mode", pgmid, p->desc);
  if(pm & (pm-1))
    Return("%s and %s share multiple programming modes (%s)",
      pgmid, p->desc, avr_prog_modes(pm));
 *
 */

  return pgm->program_enable(pgm, p);
}


static void dryrun_disable(const PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  if(dry.dp) {                  // Deallocate dryrun part
    avr_free_part(dry.dp);
    dry.dp = NULL;
  }

  return;
}


static int dryrun_open(PROGRAMMER *pgm, const char *port) {
  pmsg_debug("%s(%s)\n", __func__, port? port: "NULL");

  return 0;
}


static void dryrun_close(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
}


// Emulate flash NOR-memory
static void *memand(void *dest, const void *src, size_t n) {
  for(size_t i=0; i<n; i++)
    ((char *)dest)[i] &= ((const char *)src)[i];
  return dest;
}

static int dryrun_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  pmsg_debug("%s(%s, %u, 0x%04x, %u)\n", __func__, m->desc, page_size, addr, n_bytes);
  if(!dry.dp)
    Return("no dryrun device?");

  if(n_bytes) {
    AVRMEM *dmem, *dm2;
    int mchr, chunk;
    unsigned int end;

    // Paged writes only valid for flash and eeprom
    mchr = mem_is_in_flash(m)? 'F': 'E';
    if(mchr == 'E' && !mem_is_eeprom(m) && !mem_is_user_type(m))
      return -2;

    if(!(dmem = avr_locate_mem(dry.dp, m->desc)))
      Return("cannot locate %s %s memory for paged write", dry.dp->desc, m->desc);
    if(dmem->size < 1)
      Return("cannot write page to %s %s owing to its size %d", dry.dp->desc, dmem->desc, dmem->size);
    if(dmem->size != m->size)
      Return("cannot write page to %s %s as memory sizes differ: 0x%04x vs 0x%04x",
        dry.dp->desc, dmem->desc, dmem->size, m->size);

    end = addr + n_bytes;
    if(addr >= (unsigned int) dmem->size || end > (unsigned int) dmem->size)
      Return("cannot write page [0x%04x, 0x%04x] to %s %s as it is incompatible with memory [0, 0x%04x]",
        addr, end-1, dry.dp->desc, dmem->desc, dmem->size-1);

    for(; addr < end; addr += chunk) {
      chunk = end-addr < page_size? end-addr: page_size;
      // Return write error for protected bootloader region
      if(dry.bl && (mem_is_boot(m) || mem_is_flash(m)))
        if(dryrun_readonly(pgm, p, m, addr))
          if(memcmp(dmem->buf+addr, m->buf+addr, chunk))
            Return("Write error on protected bootloader region %s [0x%04x, 0x%04x]\n", m->desc,
              dry.bl == DRY_TOP? m->size-dry.blsize: 0, dry.bl == DRY_TOP? m->size-1: dry.blsize-1);

      // Unless it is a bootloader flash looks like NOR-memory
      (mchr == 'F' && !dry.bl? memand: memcpy)(dmem->buf+addr, m->buf+addr, chunk);

      // Copy chunk to overlapping XMEGA's apptable, application, boot and flash memories
      if(mchr == 'F') {
        if(mem_is_flash(dmem)) {
          for(LNODEID ln=lfirst(dry.dp->mem); ln; ln=lnext(ln)) {
            dm2 = ldata(ln);
            if(mem_is_in_flash(dm2) && !mem_is_flash(dm2)) { // Overlapping region?
              unsigned int cpaddr = addr + dmem->offset - dm2->offset;
              if(cpaddr < (unsigned int) dm2->size && cpaddr + chunk <= (unsigned int) dm2->size)
                memcpy(dm2->buf+cpaddr, dmem->buf+addr, chunk);
            }
          }
        } else if((dm2 = avr_locate_flash(dry.dp))) {
          unsigned int cpaddr = addr + dmem->offset - dm2->offset;
          if(cpaddr < (unsigned int) dm2->size && cpaddr + chunk <= (unsigned int) dm2->size)
            memcpy(dm2->buf+cpaddr, dmem->buf+addr, chunk);
        }
      }
    }
  }

  return n_bytes;
}


static int dryrun_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  pmsg_debug("%s(%s, %u, 0x%04x, %u)\n", __func__, m->desc, page_size, addr, n_bytes);
  if(!dry.dp)
    Return("no dryrun device?");

  if(n_bytes) {
    AVRMEM *dmem;
    int mchr, chunk;
    unsigned int end;

    // Paged load only valid for flash and eeprom
    mchr = mem_is_in_flash(m)? 'F': 'E';
    if(mchr == 'E' && !mem_is_eeprom(m) && !mem_is_user_type(m))
      return -2;

    if(!(dmem = avr_locate_mem(dry.dp, m->desc)))
      Return("cannot locate %s %s memory for paged load", dry.dp->desc, m->desc);
    if(dmem->size < 1)
      Return("cannot read page from %s %s owing to mem size %d", dry.dp->desc, dmem->desc, dmem->size);
    if(dmem->size != m->size)
      Return("cannot read page from %s %s as mem sizes differ: 0x%04x vs 0x%04x",
        dry.dp->desc, dmem->desc, dmem->size, m->size);

    end = addr + n_bytes;
    if(addr >= (unsigned int) dmem->size || end > (unsigned int) dmem->size)
      Return("cannot read page [0x%04x, 0x%04x] from %s %s as it is incompatible with memory [0, 0x%04x]",
        addr, end-1, dry.dp->desc, dmem->desc, dmem->size-1);

    for(; addr < end; addr += chunk) {
      chunk = end-addr < page_size? end-addr: page_size;
      memcpy(m->buf+addr, dmem->buf+addr, chunk);
    }
  }

  return n_bytes;
}


int dryrun_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char data) {

  AVRMEM *dmem, *dfuse;

  pmsg_debug("%s(%s, 0x%04lx, 0x%02x)\n", __func__, m->desc, addr, data);
  if(!dry.dp)
    Return("no dryrun device?");
  if(!(dmem = avr_locate_mem(dry.dp, m->desc)))
    Return("cannot locate %s %s memory for bytewise write", dry.dp->desc, m->desc);
  if(dmem->size < 1)
    Return("cannot write byte to %s %s owing to its size %d", dry.dp->desc, dmem->desc, dmem->size);
  if(dmem->size != m->size)
    Return("cannot write byte to %s %s as sizes differ: 0x%04x vs 0x%04x",
      dry.dp->desc, dmem->desc, dmem->size, m->size);
  if(dryrun_readonly(pgm, p, dmem, addr)) {
    unsigned char is;
    if(pgm->read_byte(pgm, p, m, addr, &is) >= 0 && is == data)
      return 0;

    Return("cannot write to write-protected memory %s %s", dry.dp->desc, dmem->desc);
  }

  if(addr >= (unsigned long) dmem->size)
    Return("cannot write byte to %s %s as address 0x%04lx outside range [0, 0x%04x]",
      dry.dp->desc, dmem->desc, addr, dmem->size-1);

  if(!(p->prog_modes & (PM_UPDI | PM_aWire))) { // Initialise unused bits in classic & XMEGA parts
    int bitmask = avr_mem_bitmask(dry.dp, dmem, addr);
    // Read-modify-write for bitmasked memory
    data = (data & bitmask) | (dmem->buf[addr] & ~bitmask);
  }

  dmem->buf[addr] = data;

  if(mem_is_fuses(dmem) && addr < 16) { // Copy the byte to corresponding individual fuse
    for(LNODEID ln=lfirst(dry.dp->mem); ln; ln=lnext(ln)) {
      if(mem_is_a_fuse(dfuse = ldata(ln))) {
        if(addr == mem_fuse_offset(dfuse))
          dfuse->buf[0] = data;
        else if(dfuse->size == 2 && addr-1 == mem_fuse_offset(dfuse)) // High byte of 2-byte fuse
          dfuse->buf[1] = data;
      }
    }
  } else if(mem_is_a_fuse(m) && (dfuse = avr_locate_fuses(dry.dp))) { // Copy fuse to fuses
    int fidx = addr + mem_fuse_offset(m);
    if(fidx >=0 && fidx < dfuse->size)
      dfuse->buf[fidx] = data;
  }

  return 0;
}

int dryrun_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char *value) {

  AVRMEM *dmem;

  pmsg_debug("%s(%s, 0x%04lx)", __func__, m->desc, addr);
  if(!dry.dp)
    Return("no dryrun device?");
  if(!(dmem = avr_locate_mem(dry.dp, m->desc)))
    Return("cannot locate %s %s memory for bytewise read", dry.dp->desc, m->desc);
  if(dmem->size < 1)
    Return("cannot read byte from %s %s owing to its size %d", dry.dp->desc, dmem->desc, dmem->size);
  if(dmem->size != m->size)
    Return("cannot read byte from %s %s as sizes differ: 0x%04x vs 0x%04x",
      dry.dp->desc, dmem->desc, dmem->size, m->size);

  if(addr >= (unsigned long) dmem->size)
    Return("cannot read byte %s %s as address 0x%04lx outside range [0, 0x%04x]",
      dry.dp->desc, dmem->desc, addr, dmem->size-1);

  if(!dry.bl && (mem_is_io(dmem) || mem_is_sram(dmem)) && !(p->prog_modes & (PM_UPDI | PM_PDI)))
    Return("classic part io/sram memories cannot be read externally");

  *value = dmem->buf[addr];

  msg_debug(" returns 0x%02x\n", *value);
  return 0;
}

// Periodic call in terminal mode to keep bootloader alive
static int dryrun_term_keep_alive(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  return 0;
}


static int dryrun_rdy_led(const PROGRAMMER *pgm, int value) {
  pmsg_debug("%s(%d)\n", __func__, value);

  return 0;
}

static int dryrun_err_led(const PROGRAMMER *pgm, int value) {
  pmsg_debug("%s(%d)\n", __func__, value);

  return 0;
}

static int dryrun_pgm_led(const PROGRAMMER *pgm, int value) {
  pmsg_debug("%s(%d)\n", __func__, value);

  return 0;
}

static int dryrun_vfy_led(const PROGRAMMER *pgm, int value) {
  pmsg_debug("%s(%d)\n", __func__, value);

  return 0;
}


static void dryrun_display(const PROGRAMMER *pgm, const char *p_unused) {
  imsg_info("%c%s programmer for %s\n", toupper(*pgmid), pgmid+1, dry.dp? dry.dp->desc: partdesc? partdesc: "???");
  return;
}


// Return whether an address is write protected
static int dryrun_readonly(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int addr) {

  if(mem_is_readonly(mem))
    return 1;

  if(!dry.bl) {                 // io and sram may not be accessible by external programming
    if(mem_is_io(mem) || mem_is_sram(mem))
      return !(p->prog_modes & PM_UPDI); // Can not even read these externally in classic parts
    return 0;
  }

  // Bootloader restictions: emulate a bootloader for dryboot
  if(mem_is_boot(mem) || mem_is_flash(mem))
    if(dry.bl == DRY_TOP? (int) addr >= mem->size-dry.blsize: (int) addr < dry.blsize)
      return 1;

  if(mem_is_in_fuses(mem) || mem_is_lock(mem))
    return 1;

  return 0;
}


static void dryrun_setup(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  // Allocate dry
  pgm->cookie = cfg_malloc(__func__, sizeof(dryrun_t));
}


static void dryrun_teardown(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  free(pgm->cookie);
  pgm->cookie = NULL;
}


const char dryrun_desc[] = "Dryrun programmer for testing avrdude";

void dryrun_initpgm(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);

  strcpy(pgm->type, "Dryrun");

  pgm->read_sig_bytes = dryrun_read_sig_bytes;

  // Mandatory functions
  pgm->rdy_led = dryrun_rdy_led;
  pgm->err_led = dryrun_err_led;
  pgm->pgm_led = dryrun_pgm_led;
  pgm->vfy_led = dryrun_vfy_led;
  pgm->initialize = dryrun_initialize;
  pgm->display = dryrun_display;
  pgm->enable = dryrun_enable;
  pgm->disable = dryrun_disable;
  pgm->program_enable = dryrun_program_enable;
  pgm->chip_erase = dryrun_chip_erase;
  pgm->cmd = dryrun_cmd;
  pgm->open = dryrun_open;
  pgm->close = dryrun_close;
  pgm->read_byte = dryrun_read_byte;
  pgm->write_byte = dryrun_write_byte;

  // Optional functions
  pgm->paged_write = dryrun_paged_write;
  pgm->paged_load = dryrun_paged_load;
  pgm->setup = dryrun_setup;
  pgm->teardown = dryrun_teardown;
  pgm->term_keep_alive = dryrun_term_keep_alive;
  pgm->readonly = dryrun_readonly;
}
