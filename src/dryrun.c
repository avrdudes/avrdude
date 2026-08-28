/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022- Stefan Rueger <stefan.rueger@urclocks.com>
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

/*
 * The dryrun programmer emulates a physical programmer by allocating a copy of the part and
 * pretending all operations work well.
 */

#include <ac_cfg.h>

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

#define random() rand()         // For platform independence
#define srandom(n) srand(n)

// Context of the programmer
typedef enum {
  DRY_NOBOOTLOADER,             // No bootloader, talking to an ordinary programmer
  DRY_TOP,                      // Bootloader and it sits at top of flash
  DRY_BOTTOM,                   // Bootloader sits at bottom of flash (UPDI parts)
} Dry_prog;

typedef struct {
  AVRPART *dp;
  Dry_prog bl;                  // Bootloader and, if so, at top/bottom of flash?
  int bootsize;                 // Size of boot section (if any)
  int init;                     // Initialise memories with something interesting
  int random;                   // Random initialisation of memories
  int seed;                     // Seed for random number generator
  int holes;                    // Whether eeprom/flash should have holes
  struct {
    int vectornum;              // Vector bootloader vector number for jump to application op code
    int urversion;              // Octal byte 076 means v7.6 (minor version number is lowest 3 bit)
    int32_t blstart, blend;     // Bootloader address range [blstart, blend] for write protection
    int32_t pfstart, pfend;     // Programmable flash address range [pfstart, pfend]
  } urdesc;
} Dryrun_data;

// Use private programmer data as if they were a global structure dry
#define dry (*(Dryrun_data *)(pgm->cookie))
#define ur (dry.urdesc)

#define Return(...) do { pmsg_error(__VA_ARGS__); msg_error("\n"); return -1; } while(0)
#define Retwarning(...) do { pmsg_warning(__VA_ARGS__); \
  msg_warning("; not initialising %s memories\n", p->desc); return -1; } while(0)

static int dryrun_readonly(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned int addr);

// Initialise urboot descriptor once an urboot bootloader is detected
static int dryrun_init_ur(const PROGRAMMER *pgm, const AVRPART *p) {
  // The first urboot bootloader detection freezes the parameters
  if(ur.urversion)
    return 0;

  uint8_t top[6];
  const AVRMEM *flm = avr_locate_flash(p);
  if(!flm)
    Return("cannot locate flash memory for %s\n", p->desc);

  // No urboot bootloaders on AVR32 parts, neither on really small devices
  if(is_awire(p) || flm->size < 1024)
    return 0;

  if(!is_updi(p)) {
    // Check top 6 bytes from flash to obtain intell about bootloader and type
    for(int i = sizeof top - 1; i >= 0; i--) {
      if(pgm->read_byte(pgm, p, flm, flm->size - sizeof top + i, top + i) < 0)
        return -1;
      // Abort if last byte in flash does not indicate urboot v7.5 ... v12.7 == 0147
      if(i == sizeof top - 1 && (top[i] < 075 || top[i] > 0147))
        return 0;
    }

    uint8_t numpags = top[0] & 0x7f; // Number of bootloader pages from v7.5
    uint8_t vectnum = top[1] & 0x7f; // Vector number for application start from v7.5
    uint16_t rjmpwp = buf2uint16(top+2); // rjmp to bootloader pgm_write_page() or ret
    // uint8_t cap = top[4];    // Capability byte not needed
    uint8_t urver = top[5];     // Urboot version; low 3 bits = minor version: 076 = v7.6

    // Could be urboot bootloader v7.5 .. v12.7: check further properties
    if(isop(rjmpwp, rjmp) || isop(rjmpwp, ret)) { // OK, valid rjmpwp opcode
      int blsize = numpags*flm->page_size;
      // Size of urboot bootloader should be in [64, 2048] (in v7.6 these are 224-512 bytes)
      if(blsize >= 64 && blsize <= 2048 && vectnum <= p->n_interrupts) { // Within range
        int dfromend  = dist_rjmp(rjmpwp, flm->size) - 4;
        // Further check whether writepage() rjmp opcode jumps backwards into bootloader
        if(isop(rjmpwp, ret) || (dfromend >= -blsize && dfromend < -6)) { // urboot!
          ur.blstart = flm->size - blsize;
          ur.blend   = flm->size - 1;
          ur.pfstart = 0;
          ur.pfend   = ur.blstart - 1;
          ur.vectornum = vectnum;
          ur.urversion = urver;
        }
      }
    }
  } else {                      // @@@ Fixme: todo when UPDI urboot bootloaders available
  }

  if(ur.urversion) {
    char buf[20];
    urbootPutVersion(buf, (uint16_t *) top);
    pmsg_info("detected urboot bootloader %s in [0x%04x, 0x%04x] with vector=%d\n",
      buf, ur.blstart, ur.blend, ur.vectornum);
  }

  return 0;
}

// At the beginning of every terminal command
static int dryrun_cmdhook(const PROGRAMMER *pgm, const AVRPART *p, int argc_uu, const char *argv_uu[]) {
  return dryrun_init_ur(pgm, p);
}

// At the beginning of every -U command and during execution of terminal backup, restore and verify
static int dryrun_updatehook(const PROGRAMMER *pgm, const AVRPART *p, const UPDATE *upd_uu, int flags_uu) {
  return dryrun_init_ur(pgm, p);
}

// Called after the input file has been read for writing or verifying flash
static int dryrun_flash_readhook(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *flm,
  const char *fname_uu, int size) {

  int maxsize = ur.pfend+1, firstbeg, firstlen;
  const int vecsz = flm->size <= 8192? 2: 4; // Small parts use rjmp, large a 4-byte jmp

  // Compute begin and length of first contiguous block in input
  for(firstbeg=0; firstbeg < size; firstbeg++)
    if(flm->tags[firstbeg] & TAG_ALLOCATED)
      break;
  for(firstlen=0; firstbeg+firstlen < size; firstlen++)
    if(!(flm->tags[firstbeg+firstlen] & TAG_ALLOCATED))
      break;

  // Sanity: no patching if bootloader location is unknown
  if(ur.blend <= ur.blstart)
    goto nopatch;

  // Sanity check the bootloader position
  if(ur.blstart < 0 || ur.blstart >= flm->size || ur.blend < 0 || ur.blend >= flm->size)
    Return("bootloader [0x%04x, 0x%04x] outside flash [0, 0x%04x]",
      ur.blstart, ur.blend, flm->size-1);

  // Check size of uploded application and protect bootloader from being overwritten
  if((!is_updi(p) && size > maxsize) || (is_updi(p) && firstbeg <= ur.blend))
    Return("input [0x%04x, 0x%04x] overlaps b/loader [0x%04x, 0x%04x]",
      firstbeg, size-1, ur.blstart, ur.blend);

  if(size > maxsize)
    Return("input [0x%04x, 0x%04x] extends programmable area [0x%04x, 0x%04x]",
      firstbeg, size-1, ur.pfstart, ur.pfend);

  if(is_updi(p))
    goto nopatch;

  bool llcode = firstbeg == 0 && firstlen > p->n_interrupts*vecsz; // Looks like code
  bool llvectors = firstbeg == 0 && firstlen >= p->n_interrupts*vecsz; // Looks like vector table
  for(int i = 0; llvectors && i < p->n_interrupts*vecsz; i += vecsz) {
    uint16_t op16 = buf2uint16(flm->buf+i);
    if(!isop(op16, rjmp) && !(vecsz == 4 && isop(op16, jmp)))
      llvectors = 0;
  }

  if(llcode && !llvectors && ur.vectornum > 0)
    pmsg_warning("not patching jmp to application as input does not start with a vector table\n");

  // Patch vectors if input looks like code and it's a vector bootloader with known vector number
  if(llcode && llvectors && ur.vectornum > 0) {
    uint16_t reset16;
    int reset32, appstart, appvecloc;

    appvecloc = ur.vectornum*vecsz; // Location of jump-to-application in vector table
    reset16 = buf2uint16(flm->buf); // First reset word of to-be-written application
    reset32 = vecsz == 2? reset16: buf2uint32(flm->buf);

    /*
     * Compute where the application starts from the reset vector. The assumptions are that the
     *  - Vector table, and therefore the reset vector, resides at address zero
     *  - Compiler puts either a jmp or an rjmp at address zero
     *  - Compiler does not shorten the vector table if no or few interrupts are used
     *  - Compiler does not utilise unused interrupt vectors to place code there
     */

    if(reset2addr(flm->buf, vecsz, flm->size, &appstart) < 0) {
      pmsg_warning("not patching input as opcode word %04x at reset is not r/jmp\n", reset16);
      goto nopatch;
    }

    // Only patch if appstart does not already point to the bootloader
    if(appstart != ur.blstart) {
      int vectorsend = vecsz*ur.vectornum;
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
      set_resetvector(ur.blstart, flm->size, flm->buf+0, vecsz, 1);
      if(vecsz == 4)
        uint32tobuf(flm->buf+appvecloc, jmp_opcode(appstart));
      else
        uint16tobuf(flm->buf+appvecloc, rjmp_opcode(appstart - appvecloc, flm->size));
    }
  }

nopatch:

  // Ensure that vector bootloaders have correct r/jmp at address 0
  if(!is_updi(p) && ur.blstart && ur.vectornum > 0) {
    int resetdest, set = 0;
    for(int i = 0; i < vecsz; i++)
      if(flm->tags[i] & TAG_ALLOCATED)
        set++;

    // Reset vector not programmed? Or -F? Ensure a jmp to bootloader
    if(ovsigck || set != vecsz) {
      unsigned char jmptoboot[4];
      int resetsize = set_resetvector(ur.blstart, flm->size, jmptoboot, vecsz, 1);

      if(set != vecsz) {
        unsigned char device[4];
        // Read reset vector from device flash
        for(int i = 0; i < vecsz; i++)
          if(pgm->read_byte(pgm, p, flm, i, device+i) < 0)
            return -1;

        // Mix with already set bytes
        for(int i = 0; i < vecsz; i++)
          if(!(flm->tags[i] & TAG_ALLOCATED))
            flm->buf[i] = device[i];
      }

      if(reset2addr(flm->buf, vecsz, flm->size, &resetdest) < 0 || resetdest != ur.blstart) {
        for(int i=0; i < resetsize; i++) {
          flm->buf[i] = jmptoboot[i];
          flm->tags[i] |= TAG_ALLOCATED;
        }
      }
    } else if(firstbeg < vecsz) { // Double-check reset vector jumps to bootloader
      if(reset2addr(flm->buf, vecsz, flm->size, &resetdest) < 0)
        Return("input would overwrite the reset vector bricking the bootloader\n"
          "  using -F will try to patch the input but this may not be what is needed");
      if(resetdest != ur.blstart)
        Return("input points reset to 0x%04x, not to bootloader at 0x%04x\n"
          "  using -F will try to patch the input but this may not be what is needed",
          resetdest, ur.blstart);
    }
  }

  return size;
}


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

// Emulate chip erase
static int dryrun_chip_erase(const PROGRAMMER *pgm, const AVRPART *punused) {
  AVRMEM *mem;

  pmsg_debug("%s()\n", __func__);
  if(!dry.dp)
    Return("no dryrun device?");
  if(!(mem = avr_locate_flash(dry.dp)))
    Return("cannot locate %s flash memory for chip erase", dry.dp->desc);
  if(mem->size < 1)
    Return("cannot erase %s flash memory owing to its size %d", dry.dp->desc, mem->size);

  if(dry.bl) {                  // Bootloaders won't overwrite themselves
    memset(mem->buf + (dry.bl == DRY_TOP? 0: dry.bootsize), 0xff, mem->size - dry.bootsize);
    return 0;                   // Assume that's all a bootloader does
  }

  memset(mem->buf, 0xff, mem->size);

  int eesave, bakverb = verbose;

  verbose = -123;
  if((mem = avr_locate_eeprom(dry.dp))) // Check whether EEPROM needs erasing
    if(avr_get_config_value(pgm, dry.dp, "eesave", &eesave) == 0 && eesave == !is_updi(dry.dp))
      if(mem->size > 0)
        memset(mem->buf, 0xff, mem->size);
  verbose = bakverb;

  if((mem = avr_locate_bootrow(dry.dp)))        // Also erase bootrow if it's there
    if(mem->size > 0)
      memset(mem->buf, 0xff, mem->size);

  if((mem = avr_locate_lock(dry.dp)))
    if(mem->initval != -1 && mem->size > 0 && mem->size <= (int) sizeof(mem->initval))
      for(int i = 0; i < mem->size; i++)
        mem->buf[i] = mem->initval >> 8*i;

  return 0;
}

// For now pretend all is hunky-dory
static int dryrun_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  int ret = 0;

  pmsg_debug("%s(0x%02x 0x%02x 0x%02x 0x%02x)\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
  // FIXME: do we need to emulate some more commands? For now it's only the STK universal CE
  if(cmd[0] == (Subc_STK_UNIVERSAL_LEXT >> 24) ||
    (cmd[0] == (Subc_STK_UNIVERSAL_CE >> 24) && cmd[1] == (uint8_t) (Subc_STK_UNIVERSAL_CE >> 16))) {

    ret = dryrun_chip_erase(pgm, NULL);
  }
  // Pretend call happened and all is good, returning 0xff each time
  memcpy(res, cmd + 1, 3);
  res[3] = 0xff;

  return ret;
}

static int dryrun_page_erase(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, unsigned int addr) {
  pmsg_debug("%s(%s, 0x%04x)\n", __func__, m->desc, addr);

  if(!dry.dp)
    Return("no dryrun device?");

  AVRMEM *dmem;

  if(!(dmem = avr_locate_mem(dry.dp, m->desc)))
    Return("cannot locate %s %s memory for page erase", dry.dp->desc, m->desc);

  if(!avr_has_paged_access(pgm, dry.dp, dmem) || addr >= (unsigned) dmem->size)
    Return("%s does not support paged access", dmem->desc);
  addr &= ~(dmem->page_size - 1);
  if(addr + dmem->page_size > (unsigned) dmem->size)
    Return("%s page erase of %s reaches outside %s?", dmem->desc,
      str_ccinterval(addr, addr + dmem->page_size - 1), str_ccinterval(0, dmem->size - 1));

  memset(dmem->buf + addr, 0xff, dmem->page_size);

  return 0;
}

static int dryrun_program_enable(const PROGRAMMER *pgm, const AVRPART *p_unused) {
  pmsg_debug("%s()\n", __func__);

  return 0;
}

static void dryrun_enable(PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  AVRMEM *m;

  if(dry.dp)                    // Already configured
    return;

  dry.dp = dryrun_part(p->id, &dry.bootsize, dry.init, dry.random, dry.holes, dry.seed);

  // Initialise urboot descriptor so that all flash is programmable and there is no bootloader
  if((m = avr_locate_flash(p)))
    ur.pfend = m->size-1;

  // Is the programmer a bootloader?
  if((m = avr_locate_flash(p)) && m->size >= 1024 && is_spm(pgm))
    dry.bl = is_updi(p)? DRY_BOTTOM: DRY_TOP;

  // So that dryrun can emulate AVRDUDE page erase
  if(!is_spm(pgm) && (p->prog_modes & (PM_PDI | PM_UPDI)))
    pgm->page_erase = dryrun_page_erase;
}

// Initialise the AVR device and prepare it to accept commands
static int dryrun_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);

/*
 * Normally one would select appropriate programming mechanisms here, but for
 * dryrun ignore discrepancies...

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
  if(pgm->bitclock)
    pmsg_warning("-c %s does not support adjustable bitclock speed; ignoring -B\n", pgmid);

  pmsg_debug("%s(%s)\n", __func__, port? port: "NULL");

  return 0;
}

static void dryrun_close(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
}

// Emulate flash NOR-memory
static void *memand(void *dest, const void *src, size_t n) {
  for(size_t i = 0; i < n; i++)
    ((char *) dest)[i] &= ((const char *) src)[i];
  return dest;
}

// Copy chunk in one flash memory to other overlapping flash memories (think XMEGA)
static void sharedflash(AVRPART *dp, const AVRMEM *fm, unsigned addr, int chunk) {
  for(LNODEID ln = lfirst(dp->mem); ln; ln = lnext(ln)) {
    AVRMEM *m = ldata(ln);

    if(mem_is_in_flash(m) && fm != m) { // Overlapping region?
      unsigned int cpaddr = addr + fm->offset - m->offset;

      if(cpaddr < (unsigned int) m->size && cpaddr + chunk <= (unsigned int) m->size)
        memcpy(m->buf + cpaddr, fm->buf + addr, chunk);
    }
  }
}

static int dryrun_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  pmsg_debug("%s(%s, %u, 0x%04x, %u)\n", __func__, m->desc, page_size, addr, n_bytes);
  if(!dry.dp)
    Return("no dryrun device?");

  if(n_bytes) {
    AVRMEM *dmem;
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
        addr, end - 1, dry.dp->desc, dmem->desc, dmem->size - 1);

    // Protect reset vector just as -c urclock would
    if(dry.bl == DRY_TOP && ur.vectornum > 0 && (mem_is_application(m) || mem_is_flash(m)) && addr == 0)
      for(unsigned vecsz = m->size <= 8192? 2u: 4u, i = 0; i < vecsz && i < n_bytes; i++)
        pgm->read_byte(pgm, p, dmem, i, m->buf+i);

    for(; addr < end; addr += chunk) {
      chunk = minm(end - addr, page_size);

      // Silently skip writing the chunk if that were to overwrite bootloader
      if(dry.bl && mchr == 'F' && !mem_is_apptable(m) && ur.blend > ur.blstart) {
        const AVRMEM *am;
        int testa = addr;

        // Translate XMEGA boot addresses to flash addresses
        if(is_pdi(p) && mem_is_boot(m) && (am = avr_locate_application(p)))
          testa += am->size;

        if(testa >= ur.blstart && testa+chunk-1 <= ur.blend)
          continue;
      }

      // Unless it is a bootloader flash looks like NOR-memory
      (mchr == 'F' && !dry.bl? memand: memcpy) (dmem->buf + addr, m->buf + addr, chunk);

      // Copy chunk to overlapping XMEGA's apptable, application, boot and flash memories
      if(mchr == 'F')
        sharedflash(dry.dp, dmem, addr, chunk);
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
        addr, end - 1, dry.dp->desc, dmem->desc, dmem->size - 1);

    for(; addr < end; addr += chunk) {
      chunk = minm(end - addr, page_size);
      memcpy(m->buf + addr, dmem->buf + addr, chunk);
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
      dry.dp->desc, dmem->desc, addr, dmem->size - 1);

  if(p->prog_modes & (PM_Classic | PM_PDI)) {   // Initialise unused bits in classic & XMEGA parts
    int bitmask = avr_mem_bitmask(dry.dp, dmem, addr);

    // Read-modify-write for bitmasked memory
    data = (data & bitmask) | (dmem->buf[addr] & ~bitmask);
  }

  dmem->buf[addr] = data;

  if(mem_is_fuses(dmem) && addr < 16) { // Copy the byte to corresponding individual fuse
    for(LNODEID ln = lfirst(dry.dp->mem); ln; ln = lnext(ln)) {
      if(mem_is_a_fuse(dfuse = ldata(ln))) {
        if(addr == mem_fuse_offset(dfuse))
          dfuse->buf[0] = data;
        else if(dfuse->size == 2 && addr - 1 == mem_fuse_offset(dfuse)) // High byte of 2-byte fuse
          dfuse->buf[1] = data;
      }
    }
  } else if(mem_is_a_fuse(m) && (dfuse = avr_locate_fuses(dry.dp))) {   // Copy fuse to fuses
    int fidx = addr + mem_fuse_offset(m);

    if(fidx >= 0 && fidx < dfuse->size)
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
      dry.dp->desc, dmem->desc, addr, dmem->size - 1);

  if(!dry.bl && (mem_is_io(dmem) || mem_is_sram(dmem)) && is_classic(p))
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
  // imsg_info("%c%s programmer for %s\n", toupper(*pgmid), pgmid+1, dry.dp? dry.dp->desc: partdesc? partdesc: "???");
  return;
}

// Return whether an address is write protected
static int dryrun_readonly(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned int addr) {
  if(mem_is_readonly(mem))
    return 1;

  if(!dry.bl) {                 // io and sram may not be accessible by external programming
    if(mem_is_io(mem) || mem_is_sram(mem))
      return !is_updi(p);       // Can not even read these externally in classic parts
    return 0;
  }

  // Bootloader
  if(mem_is_in_flash(mem) && !mem_is_apptable(mem)) {
    const AVRMEM *m;

    // Translate XMEGA boot addresses to flash addresses
    if(is_pdi(p) && mem_is_boot(mem) && (m = avr_locate_application(p)))
      addr += m->size;

    if(addr > (unsigned int) ur.pfend)
      return 1;
    if(addr < (unsigned int) ur.pfstart)
      return 1;
    // Protect reset vector once vector bootloader detected
    if(addr < 4 && !is_updi(p) && ur.vectornum > 0)
      if(addr < ((m = avr_locate_flash(p)) && m->size <= 8192? 2u: 4u))
        return 1;
  }
  /* // Below is too realistic as it precludes -U urboot: fuse settings
   * else if(is_classic(p) && !mem_is_eeprom(mem))
   *   return 1;
   */

  if(dry.dp && (mem_is_in_fuses(mem) || mem_is_lock(mem)))
    return 1;

  return 0;
}

static void dryrun_setup(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  // Allocate dry
  pgm->cookie = mmt_malloc(sizeof(Dryrun_data));
}

static void dryrun_teardown(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  mmt_free(pgm->cookie);
  pgm->cookie = NULL;
}

static int dryrun_parseextparams(const PROGRAMMER *pgm, const LISTID extparms) {
  int rc = 0;
  bool help = false;

  for(LNODEID ln = lfirst(extparms); ln; ln = lnext(ln)) {
    const char *xpara = ldata(ln);

    if(str_eq(xpara, "init")) {
      dry.init = 1;
      continue;
    }
    if(str_eq(xpara, "random")) {
      dry.random = 1;
      continue;
    }
    if(str_starts(xpara, "seed=") || str_starts(xpara, "init=") || str_starts(xpara, "random=")) {
      const char *errptr;
      int seed = str_int(strchr(xpara, '=') + 1, STR_INT32, &errptr);

      if(errptr) {
        pmsg_error("cannot parse %s seed value: %s\n", xpara, errptr);
        rc = -1;
        break;
      }
      dry.seed = seed;
      if(str_starts(xpara, "init"))
        dry.init = 1;
      else if(str_starts(xpara, "random"))
        dry.random = 1;
      continue;
    }
    if(str_eq(xpara, "help")) {
      help = true;
      rc = LIBAVRDUDE_EXIT_OK;
    }

    if(!help) {
      pmsg_error("invalid extended parameter -x %s\n", xpara);
      rc = -1;
    }
    msg_error("%s -c %s extended options:\n", progname, pgmid);
    msg_error("  -x init       Initialise memories with human-readable patterns (1, 2, 3)\n");
    msg_error("  -x init=<n>   Shortcut for -x init -x seed=<n>\n");
    msg_error("  -x random     Initialise memories with random code/values (1, 3)\n");
    msg_error("  -x random=<n> Shortcut for -x random -x seed=<n>\n");
    msg_error("  -x seed=<n>   Seed random number generator with <n>, n>0, default time(NULL)\n");
    msg_error("  -x help       Show this help menu and exit\n");
    msg_error("Notes:\n");
    msg_error("  (1) -x init and -x random randomly configure flash wrt boot/data/code length\n");
    msg_error("  (2) Patterns can best be seen with fixed-width font on -U flash:r:-:I\n");
    msg_error("  (3) Choose, eg, -x seed=1 for reproducible flash configuration and output\n");
    return rc;
  }

  return rc;
}

const char dryrun_desc[] = "Dryrun programmer for testing avrdude";

void dryrun_initpgm(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);

  pgm->ptyp = "Dryrun";

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
  pgm->parseextparams = dryrun_parseextparams;
  if(is_spm(pgm)) {
    pgm->flash_readhook = dryrun_flash_readhook;
    pgm->updatehook = dryrun_updatehook;
    pgm->cmdhook = dryrun_cmdhook;
  }
}
