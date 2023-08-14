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
typedef struct {
  AVRPART *dp;
} dryrun_t;

// Use private programmer data as if they were a global structure dry
#define dry (*(dryrun_t *)(pgm->cookie))

#define Return(...) do { pmsg_error(__VA_ARGS__); msg_error("\n"); return -1; } while (0)


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
    Return("no dryrun device? Raise an issue at https://github.com/avrdudes/avrdude/issues");
  if(!(flm = avr_locate_mem(dry.dp, "flash")))
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

  if(!dry.dp) {
    unsigned char inifuses[10]; // For fuses, which is made up from fuse0, fuse1, ...
    AVRMEM *fusesm = NULL;
    dry.dp = avr_dup_part(p);   // Allocate dryrun part

    memset(inifuses, 0xff, sizeof inifuses);
    // Initialise the device with fuse factory setting and erase flash/EEPROM to 0xff
    for (LNODEID ln=lfirst(dry.dp->mem); ln; ln=lnext(ln)) {
      AVRMEM *m = ldata(ln);
      if(avr_mem_is_flash_type(m) || avr_mem_is_eeprom_type(m)) {
        memset(m->buf, 0xff, m->size);
      } else if(str_eq(m->desc, "fuses")) {
        fusesm = m;
      } else if(str_contains(m->desc, "fuse") || str_contains(m->desc, "lock")) {
        // Lock can have 4 bytes: still allow initialisation from initval
        if(m->initval != -1 && m->size >=1 && m->size <= (int) sizeof(m->initval)) {
          memcpy(m->buf, &m->initval, m->size); // FIXME: relying on little endian here
          if(str_starts(m->desc, "fuse") && m->desc[4] && m->size == 1) {
            int fno = m->desc[4]-'0';
            if(fno >= 0 && fno < (int) sizeof inifuses)
              inifuses[fno] = m->initval;
          }
        } else {
          memset(m->buf, 0xff, m->size);
        }
      } else if(str_eq(m->desc, "signature") && (int) sizeof(dry.dp->signature) == m->size) {
        memcpy(m->buf, dry.dp->signature, m->size);
      } else if(str_contains(m->desc, "calibration")) {
        memset(m->buf, 0x55, m->size); // ASCII 0x55 is 'U' for uncalibrated :)
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
    Return("no dryrun device? Raise an issue at https://github.com/avrdudes/avrdude/issues");

  if(n_bytes) {
    AVRMEM *dmem, *dm2;
    int mchr, chunk;
    unsigned int end;

    // Paged writes only valid for flash and eeprom
    mchr = avr_mem_is_flash_type(m)? 'F': 'E';
    if(mchr == 'E' && !avr_mem_is_eeprom_type(m))
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
      (mchr == 'F'? memand: memcpy)(dmem->buf+addr, m->buf+addr, chunk);

      // Copy chunk to overlapping XMEGA's apptable, application, boot and flash memories
      if(mchr == 'F') {
        if(str_eq(dmem->desc, "flash")) {
          for(LNODEID ln=lfirst(dry.dp->mem); ln; ln=lnext(ln)) {
            dm2 = ldata(ln);
            if(avr_mem_is_flash_type(dm2) && !str_eq(dm2->desc, "flash")) { // Overlapping region?
              unsigned int cpaddr = addr + dmem->offset - dm2->offset;
              if(cpaddr < (unsigned int) dm2->size && cpaddr + chunk <= (unsigned int) dm2->size)
                memcpy(dm2->buf+cpaddr, dmem->buf+addr, chunk);
            }
          }
        } else if((dm2 = avr_locate_mem(dry.dp, "flash"))) {
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
    Return("no dryrun device? Raise an issue at https://github.com/avrdudes/avrdude/issues");

  if(n_bytes) {
    AVRMEM *dmem;
    int mchr, chunk;
    unsigned int end;

    // Paged load only valid for flash and eeprom
    mchr = avr_mem_is_flash_type(m)? 'F': 'E';
    if(mchr == 'E' && !avr_mem_is_eeprom_type(m))
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
    Return("no dryrun device? Raise an issue at https://github.com/avrdudes/avrdude/issues");
  if(!(dmem = avr_locate_mem(dry.dp, m->desc)))
    Return("cannot locate %s %s memory for bytewise write", dry.dp->desc, m->desc);
  if(dmem->size < 1)
    Return("cannot write byte to %s %s owing to its size %d", dry.dp->desc, dmem->desc, dmem->size);
  if(dmem->size != m->size)
    Return("cannot write byte to %s %s as sizes differ: 0x%04x vs 0x%04x",
      dry.dp->desc, dmem->desc, dmem->size, m->size);
  if(str_contains(dmem->desc, "calibration") || str_eq(dmem->desc, "signature"))
    Return("cannot write to write-protected memory %s %s", dry.dp->desc, dmem->desc);

  if(addr >= (unsigned long) dmem->size)
    Return("cannot write byte to %s %s as address 0x%04lx outside range [0, 0x%04x]",
      dry.dp->desc, dmem->desc, addr, dmem->size-1);

  if(!(p->prog_modes & (PM_UPDI | PM_aWire))) { // Initialise unused bits in classic & XMEGA parts
    int bitmask = avr_mem_bitmask(dry.dp, dmem, addr);
    // Read-modify-write for bitmasked memory
    data = (data & bitmask) | (dmem->buf[addr] & ~bitmask);
  }

  dmem->buf[addr] = data;

  if(str_eq(dmem->desc, "fuses") && addr < 10) { // Copy the byte to corresponding fuse[0-9]
    char memtype[64];
    sprintf(memtype, "fuse%ld", addr);
    if((dfuse = avr_locate_mem(dry.dp, memtype)) && dfuse->size == 1)
      dfuse->buf[0] = data;
  } else if(str_starts(m->desc, "fuse")) { // Copy fuseN byte into fuses memory
    int fno = m->desc[4]-'0';
    if(fno >= 0 && fno < 10)
      if((dfuse = avr_locate_mem(dry.dp, "fuses")) && dfuse->size > fno)
        dfuse->buf[fno] = data;
  }

  return 0;
}

int dryrun_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char *value) {

  AVRMEM *dmem;

  pmsg_debug("%s(%s, 0x%04lx)", __func__, m->desc, addr);
  if(!dry.dp)
    Return("no dryrun device? Raise an issue at https://github.com/avrdudes/avrdude/issues");
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
  imsg_info("Dryrun programmer for %s\n", dry.dp? dry.dp->desc: partdesc? partdesc: "???");
  return;
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
}
