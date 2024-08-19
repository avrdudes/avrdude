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
  DRY_NOBOOTLOADER,             // No bootloader, taking to an ordinary programmer
  DRY_TOP,                      // Bootloader and it sits at top of flash
  DRY_BOTTOM,                   // Bootloader sits at bottom of flash (UPDI parts)
} Dry_prog;

typedef struct {
  AVRPART *dp;
  Dry_prog bl;                  // Bootloader and, if so, at top/bottom of flash?
  int init;                     // Initialise memories with something interesting
  int random;                   // Random initialisation of memories
  int seed;                     // Seed for random number generator
  // Flash configuration irrespective of -c programming is bootloading or not
  int appstart, appsize;        // Start and size of application section
  int datastart, datasize;      // Start and size of application data section (if any)
  int bootstart, bootsize;      // Start and size of boot section (if any)
  int initialised;              // 1 once the part memories are initialised
} Dryrun_data;

// Use private programmer data as if they were a global structure dry
#define dry (*(Dryrun_data *)(pgm->cookie))

#define Return(...) do { pmsg_error(__VA_ARGS__); msg_error("\n"); return -1; } while(0)
#define Retwarning(...) do { pmsg_warning(__VA_ARGS__); \
  msg_warning("; not initialising %s memories\n", p->desc); return -1; } while(0)

static int dryrun_readonly(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned int addr);

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
      memcpy(mem->buf, &mem->initval, mem->size);       // FIXME: relying on little endian here

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
    Return("cannot locate %s %s memory for paged write", dry.dp->desc, m->desc);

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

// Randomly set configuration values for bootloading, bootloader size and codesize, if any
static void randflashconfig(const PROGRAMMER *pgm, const AVRPART *p, const Avrintel *up,
  const Configitem *cp, int nc) {

  if(up && is_updi(p)) {
    int sectorsize = up->bootsize > 0? up->bootsize: 256;
    int nsectors = up->flashsize/sectorsize;
    int bootsize = random()%(nsectors > 4? nsectors/4: nsectors);
    int codesize = !bootsize || random()%3? 0: bootsize + random()%(nsectors - bootsize);

    int size = !!avr_locate_config(cp, nc, "bootsize", str_eq);

    avr_set_config_value(pgm, p, size? "bootsize": "bootend", bootsize);
    avr_set_config_value(pgm, p, size? "codesize": "append", codesize);
  } else if(up && up->nboots > 0 && (p->prog_modes & (PM_Classic | PM_PDI))) {
    avr_set_config_value(pgm, p, "bootrst", random()%2);
    if(up->nboots == 4)
      avr_set_config_value(pgm, p, "bootsz", random()%4);
  }
}

// Compute app, data and boot start/size
static int flashlayout(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *flm,
  const Avrintel *up, const Configitem *cp, int nc) {

  AVRMEM *m;

  if(is_updi(p)) {
    int nbootsec = 0, ncodesec = 0;

    int size = !!avr_locate_config(cp, nc, "bootsize", str_eq);

    avr_get_config_value(pgm, p, size? "bootsize": "bootend", &nbootsec);
    avr_get_config_value(pgm, p, size? "codesize": "append", &ncodesec);
    if(nbootsec == 0 || (ncodesec && ncodesec <= nbootsec)) {   // Treat boot section for code
      dry.bootstart = 0, dry.bootsize = 0;
      dry.appstart = 0, dry.appsize = nbootsec? nbootsec*up->bootsize: up->flashsize;
    } else {                    // Distinct boot and application section
      dry.bootstart = 0, dry.bootsize = nbootsec*up->bootsize;
      dry.appstart = dry.bootsize;
      dry.appsize = ncodesec? (ncodesec - nbootsec)*up->bootsize: up->flashsize - dry.appstart;
    }
    dry.datasize = up->flashsize - dry.bootsize - dry.appsize;  // Remainder is apptable
    dry.datastart = dry.datasize? dry.bootsize + dry.appsize: 0;
  } else if(p->prog_modes & (PM_Classic | PM_PDI)) {
    dry.bootstart = 0, dry.bootsize = 0;
    if(up->nboots) {
      int bootrst = 1;

      avr_get_config_value(pgm, p, "bootrst", &bootrst);
      if(bootrst == 0) {        // Jump to bootloader on reset
        if(is_pdi(p) && (m = avr_locate_boot(p)) && m->size > 0) {
          dry.bootstart = m->offset - flm->offset;
          dry.bootsize = m->size;
        } else if(is_classic(p)) {
          if(up->nboots == 4) {
            int bootsz = 0;

            avr_get_config_value(pgm, p, "bootsz", &bootsz);
            dry.bootsize = (8 >> bootsz)*up->bootsize;
          } else
            dry.bootsize = up->bootsize;
          dry.bootstart = up->flashsize - dry.bootsize;
        }
      }
    }
    dry.datastart = 0, dry.datasize = 0;
    if(is_pdi(p) && (m = avr_locate_apptable(p)) && m->size > 0) {
      dry.datastart = m->offset - flm->offset;
      dry.datasize = up->flashsize - dry.datastart - dry.bootsize;
    }
    dry.appstart = 0, dry.appsize = up->flashsize - dry.datasize - dry.bootsize;
  }

  // Sanity checks
  if(dry.appsize < 0)
    Retwarning("negative application size");
  if(dry.appstart < 0 || dry.appstart + dry.appsize > up->flashsize)
    Retwarning("application section %s outside flash [0, 0x%04x]",
      str_ccinterval(dry.appstart, dry.appstart + dry.appsize - 1), up->flashsize - 1);

  if(dry.datasize < 0)
    Retwarning("negative apptable size");
  if(dry.datastart < 0 || dry.datastart + dry.datasize > up->flashsize)
    Retwarning("apptable section %s outside flash [0, 0x%04x]",
      str_ccinterval(dry.datastart, dry.datastart + dry.datasize - 1), up->flashsize - 1);

  if(dry.bootsize < 0)
    Retwarning("negative boot section size");
  if(dry.bootstart < 0 || dry.bootstart + dry.bootsize > up->flashsize)
    Retwarning("boot section %s outside flassh [0, 0x%04x]",
      str_ccinterval(dry.bootstart, dry.bootstart + dry.bootsize - 1), up->flashsize - 1);

  if(dry.appsize + dry.datasize + dry.bootsize != up->flashsize)
    Retwarning("section sizes do not add up (0x%x) to flash size 0x%x",
      dry.appsize + dry.datasize + dry.bootsize, up->flashsize);

  if(!dry.appsize)
    Retwarning("no application section");

  if(is_updi(p)) {
    if(dry.bootsize && dry.appstart != dry.bootsize)
      Retwarning("application section %s does not touch boot section %s",
        str_ccinterval(dry.appstart, dry.appstart + dry.appsize - 1),
        str_ccinterval(dry.bootstart, dry.bootstart + dry.bootsize - 1));
    if(dry.datasize && dry.datastart != dry.bootsize + dry.appsize)
      Retwarning("apptable section %s does not touch code section %s",
        str_ccinterval(dry.datastart, dry.datastart + dry.appsize - 1),
        str_ccinterval(0, dry.bootsize + dry.appsize - 1));
  } else {
    if(dry.datasize && dry.datastart != dry.appsize && dry.appstart != 0)
      Retwarning("apptable section %s does not touch application section %s",
        str_ccinterval(dry.datastart, dry.datastart + dry.appsize - 1),
        str_ccinterval(dry.appstart, dry.appstart + dry.appsize - 1));
    if(dry.datasize && dry.bootsize && dry.bootstart != dry.appsize + dry.datasize)
      Retwarning("apptable section %s does not touch boot section %s",
        str_ccinterval(dry.datastart, dry.datastart + dry.appsize - 1),
        str_ccinterval(dry.bootstart, dry.bootstart + dry.bootsize - 1));
  }

  return 0;
}

// Write a vector table to flash addr and return number of bytes written
static int putvectortable(const AVRPART *p, const AVRMEM *flm, int addr, int round32) {
  int vecsz = flm->size <= 8192? 2: 4, ret = p->n_interrupts*vecsz;
  int app = (ret + vecsz - 2)/2;      // Distance to application in words

  for(int i = 0; i < ret; i += vecsz) { // First store rjmps to after table
    flm->buf[addr + i] = app;
    flm->buf[addr + i + 1] = 0xc0 + (app >> 8); // rjmp app, rjmp app, ...
    if(vecsz == 4)              // Put nop behind rjmp
      flm->buf[addr + i + 2] = 0, flm->buf[addr + i + 3] = 0;
    app -= vecsz/2;
  }
  for(int i = 0; i < vecsz; i++)        // Leave one vector gap
    flm->buf[addr + ret++] = round32? ' ': 0;

  if(round32) {
    flm->buf[addr + ret++] = 0xff;      // Put endless loop rjmp .-2 as application
    flm->buf[addr + ret++] = 0xcf;

    // Then round up to multiples of 32
    while(ret%32)
      flm->buf[addr + ret++] = ' ';
  }

  return ret;
}

// Human-readable messages in flash shown with, eg, avrdude -c dryrun -p m168 -xinit -Uflash:r:-:I
static const int u384[] = {
  0x00000800, 0x08000800, 0x1c4218ca, 0x08a5284a, 0x1842184e, 0x00000000, 0x00000000, 0x08010000,
  0x08010000, 0x08c53086, 0x00430942, 0x08653082,
}, u512[] = {

  0x20000800, 0x20000800, 0xf71c7b51, 0x28a288d1, 0x28a28851, 0x28a28859, 0xc71c7856, 0x00000000,
  0x80020000, 0x80020000, 0x8f22f1cd, 0x80920a23, 0x870e0a21, 0x08120a21, 0x87a2f1c1, 0x00000000,
}, bdata[] = {

  0x00000000, 0x00000001, 0x00000001, 0x08000001, 0x08000001, 0xfe381c1d, 0x08442223, 0x08824121,
  0x08824121, 0x08824121, 0x08442223, 0xf0381c1d, 0x00000000, 0x00000000, 0x00400000, 0x00400000,
  0x00400000, 0x00400000, 0x785c0e3c, 0x88621102, 0x84422081, 0xfc422081, 0x04422081, 0x04621102,
  0xf85c0e3c, 0x00000000, 0x00000000, 0x00000000,
}, adata[] = {

  0x00000020, 0x00000020, 0x00040020, 0x00040020, 0x3c7f1e2e, 0x40042031, 0x40042021, 0x7c043e21,
  0x42042121, 0x42042131, 0xfc787e2e, 0x00000000, 0x00000000, 0x00000000,
}, rocks[] = {

  0x00000004, 0x0000003c, 0x000000fc, 0x000007fc, 0x00001ffc, 0x0000ffe0, 0x0003ff00, 0x001ffc00,
  0x007fc000, 0x03fe0000, 0x07f00000, 0x07800000, 0x07e00000, 0x07fc0000, 0x03ff0000, 0x007fe000,
  0x001ffc00, 0x0003ff00, 0x0000ffe0, 0x00001ffc, 0x000007fc, 0x000000fc, 0x0000003c, 0x00000004,
  0x04000000, 0x07800000, 0x07e00000, 0x07fc0000, 0x07ff0000, 0x00ffe000, 0x003ff800, 0x0007ff00,
  0x0001ffc0, 0x00003ff8, 0x00000ffc, 0x000001fc, 0x0000007c, 0x000003fc, 0x00001ff8, 0x0000ffc0,
  0x0003ff00, 0x001ff800, 0x00ffe000, 0x03ff0000, 0x07fc0000, 0x07e00000, 0x07800000, 0x04000000,
  0x00000000, 0x00000000, 0x00000000, 0x07fffffc, 0x07fffffc, 0x07fffffc, 0x07007000, 0x07007000,
  0x07007c00, 0x0700fe00, 0x0700ff00, 0x0781ffc0, 0x07c3cfe0, 0x03ffc3f0, 0x03ff81fc, 0x01ff00fc,
  0x007c003c, 0x0000001c, 0x0000000c, 0x00000000, 0x00000000, 0x00000000, 0x07fffffc, 0x07fffffc,
  0x07fffffc, 0x0700001c, 0x0700001c, 0x0700001c, 0x0700003c, 0x03800038, 0x03c00078, 0x03e000f0,
  0x01f803f0, 0x00ffffe0, 0x003fff00, 0x0007f800, 0x00000000, 0x00000000, 0x00000000, 0x07ffff00,
  0x07ffffc0, 0x07fffff0, 0x000000f8, 0x00000038, 0x0000003c, 0x0000001c, 0x0000003c, 0x00000078,
  0x000000f8, 0x07fffff0, 0x07ffffc0, 0x07ffff00, 0x00000000, 0x00000000, 0x00000000, 0x07fffffc,
  0x07fffffc, 0x07fffffc, 0x0700001c, 0x0700001c, 0x0700001c, 0x0700003c, 0x03800038, 0x03c00078,
  0x03e000f0, 0x01f803f0, 0x00ffffe0, 0x003fff00, 0x0007f800, 0x00000000, 0x00000000, 0x00000000,
  0x07fffffc, 0x07fffffc, 0x07fffffc, 0x0700e01c, 0x0700e01c, 0x0700e01c, 0x0700e01c, 0x0700e01c,
  0x0700e01c, 0x0700e01c, 0x0700001c, 0x0700001c, 0x0000001c, 0x00000000, 0x00000000, 0x00000000,
  0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
  0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x007ffffc, 0x007ffffc, 0x007ffffc, 0x00070000,
  0x000e0000, 0x001c0000, 0x00380000, 0x00380000, 0x00700000, 0x00700000, 0x007f0000, 0x007f0000,
  0x00000000, 0x00000000, 0x00000000, 0x0000fe00, 0x0007ffc0, 0x000fffe0, 0x003f01f8, 0x003c0078,
  0x0078003c, 0x0070001c, 0x0070001c, 0x0078003c, 0x003c0078, 0x003f01f8, 0x000fffe0, 0x0007ffc0,
  0x0000fe00, 0x00000000, 0x00000000, 0x00000000, 0x0000fe00, 0x0003ffc0, 0x000fffe0, 0x001f01f0,
  0x003c0078, 0x0078003c, 0x0070001c, 0x0070001c, 0x0070001c, 0x0078003c, 0x00380038, 0x00380038,
  0x00000000, 0x00000000, 0x00000000, 0xfffffffc, 0xfffffffc, 0xfffffffc, 0x00003000, 0x00007800,
  0x0000fc00, 0x0003fe00, 0x0007ff00, 0x000fcf80, 0x001f87c0, 0x007f03f0, 0x007e01f8, 0x007800fc,
  0x0070007c, 0x0060003c, 0x0040001c, 0x0000000c, 0x00000004, 0x00000000, 0x00000000, 0x00000000,
  0x00000078, 0x000f8038, 0x001fc038, 0x003fe03c, 0x003fe01c, 0x0038f01c, 0x0078f01c, 0x0070701c,
  0x0070381c, 0x00703c1c, 0x00703c3c, 0x00701e78, 0x00781ff8, 0x00380fe0, 0x000003c0, 0x00000000,
  0x00000000, 0x00000000, 0x7ffffc3c, 0x7ffffc3c, 0x7ffffc3c, 0x00000000, 0x00000000, 0x00000000,
  0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
  0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

const struct {
  const int *bits, n32;
} banner[] = {
  {u384, sizeof u384/sizeof *u384},
  {u512, sizeof u512/sizeof *u512},
  {bdata, sizeof bdata/sizeof *bdata},
  {adata, sizeof adata/sizeof *adata},
  {rocks, sizeof rocks/sizeof *rocks},
  {rocks, sizeof rocks/sizeof *rocks},        // Sic, dummy entry for RND
};

enum { U384, U512, BDATA, ADATA, ROCKS, RND };

/*
 * Given a bit stream, put a sequence of '@' or ' ' into flash; note they are
 * all benign opcodes that do not touch memory or the I/O area:
 *   "  ": and  r2, r0
 *   "@ ": and  r4, r0
 *   " @": sbci r18, 0
 *   "@@": sbci r20, 0
 */
static void putbanner(const AVRMEM *flm, int addr, int n, int bi) {
  const int *bp = banner[bi].bits, len = n/10 + random()%(9*n/10);

  for(int i = 0; i < n;) {
    int scan = bi == RND? random(): *bp;

    for(int j = 0; j < 32; j++) {
      flm->buf[addr++] = scan & 1? '@': ' ';
      scan >>= 1;
      if(++i == n)
        break;
    }
    if(++bp == banner[bi].bits + banner[bi].n32) {
      bp = banner[bi].bits;
      if(i > len)               // Stop repeating banner after some threshold
        break;
    }
  }
}

// Put single 16-bit opcode into memory
static void putop16(unsigned char *addr, int op) {
  addr[0] = op, addr[1] = op >> 8;
}

// Put n/2 random benign opcodes compatible with part into memory at addr
static void putcode(const AVRPART *p, const AVRMEM *flm, int addr, int n) {
  int i, op, inrange, pc, end = addr + n/2*2, avrlevel = avr_get_archlevel(p);

  for(i = 0; i < n/2; i++) {
    do {
      inrange = 0;
      // Last opcode is a long backward jump; the others are random
      op = i == n/2 - 1? dist2rjmp(-2*(i < 2048? i: 2047)): random() & 0xffff;
      if(op16_is_benign(op, avrlevel))
        inrange = (pc = op16_target(addr + 2*i, op)) >= addr && pc < end;
    } while(!inrange);
    putop16(flm->buf + addr + 2*i, op);
  }
}

// Write valid opcodes to flash (banners for -xinit, random code for -xrandom)
static void putflash(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *flm, int addr, int n, int bi) {
  unsigned char *top = flm->buf + addr + n - 4;

  if(dry.random) {
    switch(bi) {
    case U384:
    case U512:
    case BDATA:                // Bootloader stuff, reduce code length a little
      n -= random()%(n/8);
      break;
    case ADATA:
    case ROCKS:                // Set random code length in [n/4, n]
      n -= random()%(3*n/4);
    }
    if(bi != ADATA) {
      putcode(p, flm, addr, n);
      goto seal;
    }
    bi = RND;                   // Make apptable data random @/space sequences
  }
  putbanner(flm, addr, n, bi);

seal:                          // Put 1-2 endless loops in top memory section
  if(*top == 0xff)
    putop16(top, 0xcfff);
  putop16(top + 2, 0xcfff);
}

// Initialise a user writable memory other than flash or fuses
static void putother(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, const char *str) {
  const char *name = avr_mem_name(p, m);
  int len = strlen(str);

  if(len > m->size)
    len = m->size;
  if(len <= 0)
    return;

  memset(m->buf, 0xff, m->size);

  if(dry.random)
    putbanner(m, 0, m->size, RND);
  else
    for(int i = 0; i < m->size/3; i += len)
      if(m->size - i > len)
        memcpy(m->buf + i, str, len);

  if((len = strlen(name)) > m->size)
    len = m->size;
  memcpy(m->buf + m->size - len, name, len);
  if(len < m->size)
    m->buf[m->size - len - 1] = ' ';
}

// Copy chunk in one flash memory to other overlapping flash memories (think XMEGA)
static void sharedflash(const PROGRAMMER *pgm, const AVRMEM *fm, unsigned addr, int chunk) {
  for(LNODEID ln = lfirst(dry.dp->mem); ln; ln = lnext(ln)) {
    AVRMEM *m = ldata(ln);

    if(mem_is_in_flash(m) && fm != m) { // Overlapping region?
      unsigned int cpaddr = addr + fm->offset - m->offset;

      if(cpaddr < (unsigned int) m->size && cpaddr + chunk <= (unsigned int) m->size)
        memmove(m->buf + cpaddr, fm->buf + addr, chunk);
    }
  }
}

static void dryrun_enable(PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);

  if(dry.dp)                    // Already configured
    return;

  unsigned char inifuses[16];   // For fuses: made up from fuse0, fuse1, ...
  AVRMEM *m, *fusesm = NULL, *prodsigm = NULL, *calm;
  AVRPART *q = dry.dp = avr_dup_part(p);        // Allocate dryrun part and abbreviate with q

  memset(inifuses, 0xff, sizeof inifuses);
  srandom(dry.seed? dry.seed: time(NULL));

  // Initialise the device with factory setting and erase flash/EEPROM to 0xff
  for(LNODEID ln = lfirst(q->mem); ln; ln = lnext(ln)) {
    m = ldata(ln);
    if(mem_is_in_flash(m) || mem_is_eeprom(m)) {
      memset(m->buf, 0xff, m->size);
    } else if(mem_is_fuses(m)) {
      fusesm = m;
    } else if(mem_is_a_fuse(m) || mem_is_lock(m)) {
      // Lock, eg, can have 4 bytes: still allow initialisation from initval
      if(m->initval != -1 && m->size >= 1 && m->size <= (int) sizeof(m->initval)) {
        memcpy(m->buf, &m->initval, m->size);   // FIXME: relying on little endian here
        if(mem_is_a_fuse(m)) {
          int fno = mem_fuse_offset(m);

          for(int i = 0; i < m->size && fno + i < (int) sizeof inifuses; i++) // pdicfg has 2 bytes
            inifuses[fno + i] = m->initval >> 8*i;
        }
      } else {
        memset(m->buf, 0xff, m->size);
      }
    } else if(mem_is_signature(m) && (int) sizeof(q->signature) == m->size) {
      memcpy(m->buf, q->signature, m->size);
    } else if(mem_is_calibration(m)) {
      memset(m->buf, 'U', m->size);     // 'U' for uncalibrated or unknown :)
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
    } else if(mem_is_tempsense(m)) {
      memset(m->buf, 'T', m->size);     // 'T' for temperature calibration values
    } else if(mem_is_sernum(m)) {
      for(int i = 0; i < m->size; i++)  // Set serial number UTSRQPONM...
        m->buf[i] = dry.random? 'A' + random()%26: 'U' - i >= 'A'? 'U' - i: 0xff;
    } else if(mem_is_sigrow(m) && m->size >= 6) {
      prodsigm = m;
      memset(m->buf, 0xff, m->size);
      // Classic parts: signature at even addresses
      int n = is_tpi(q)? 1: 2;  // ... unless it's the TPI parts t102/t104

      if(is_classic(q))
        for(int i = 0; i < 3; i++)
          m->buf[n*i] = q->signature[i];
    } else if(mem_is_io(m)) {   // Initialise reset values (if known)
      int nr;
      const Register_file *rf = avr_locate_register_file(q, &nr);

      if(rf)
        for(int i = 0; i < nr; i++)
          if(rf[i].initval != -1 && rf[i].size > 0 && rf[i].size < 5)
            if(rf[i].addr >= 0 && rf[i].addr + rf[i].size <= m->size)
              for(int k = 0; k < rf[i].size; k++)       // FIXME: Assume little endian compiler
                m->buf[rf[i].addr + k] = ((unsigned char *) &rf[i].initval)[k];
    }
  }
  if(prodsigm) {
    if(q->prog_modes & (PM_UPDI | PM_PDI)) {
      for(LNODEID ln = lfirst(q->mem); ln; ln = lnext(ln)) {
        AVRMEM *m = ldata(ln);

        if(m->buf == prodsigm->buf)     // Skip prodsig memory
          continue;
        int off = m->offset - prodsigm->offset;
        int cpy = m->size;

        // Submemory of prodsig, eg, signature and tempsense? Copy into prodsig
        if(off >= 0 && off + cpy <= prodsigm->size)
          memcpy(prodsigm->buf + off, m->buf, cpy);
      }
    }
    if(is_classic(q) && (calm = avr_locate_calibration(q))) {
      // Calibration bytes of classic parts are interspersed with signature
      int n, tpi = is_tpi(q);   // ... unless it's the TPI parts t102/t104

      for(int i = 0; i < calm->size; i++) {
        if((n = tpi? 3 + i: 2*i + 1) < prodsigm->size)
          prodsigm->buf[n] = 'U';
      }
    }
    if(is_classic(q) && (m = avr_locate_sernum(q))) { // m324pb/m328pb, t102/t104
      int off = m->offset - prodsigm->offset;
      int cpy = m->size;

      if(off >= 0 && off + cpy <= prodsigm->size)
        memcpy(prodsigm->buf + off, m->buf, cpy);
    }
  }
  if(fusesm) {
    size_t fusz = fusesm->size;

    memcpy(fusesm->buf, inifuses, fusz < sizeof inifuses? fusz: sizeof inifuses);
  }

  // Is the programmer a bootloader?
  if((m = avr_locate_flash(q)) && m->size >= 1024 && is_spm(pgm))
    dry.bl = is_updi(q)? DRY_BOTTOM: DRY_TOP;

  // So that dryrun can emulate AVRDUDE page erase
  if(!is_spm(pgm) && (q->prog_modes & (PM_PDI | PM_UPDI)))
    pgm->page_erase = dryrun_page_erase;

  if(!dry.random && !dry.init)  // OK, no further initialisation needed
    return;

  int nc, bakverb = verbose;

  verbose = -123;               // Silently retrieve uP_table[] entry and config list
  const Avrintel *up = avr_locate_uP(q);
  const Configitem *cp = avr_locate_configitems(q, &nc);

  verbose = bakverb;
  AVRMEM *flm = avr_locate_flash(q);
  AVRMEM *ee = avr_locate_eeprom(q);
  int incons = flm && up && (up->flashsize != flm->size || flm->size <= 0 ||
    (ee && (up->eepromsize != ee->size || ee->size <= 0)) ||
    up->nboots != q->n_boot_sections || up->nboots < 0 ||
    up->bootsize != q->boot_section_size || up->bootsize < 0 || memcmp(up->sigs, q->signature, 3)
    );

  // Ensure can use up and cp with impunity
  if(!flm || !up || incons || !cp) {
    pmsg_warning("%s for %s; not initialising memories beyond factory settings\n", !flm? "no flash":
      !up? "no uP_table[] entry": incons? "inconsistent uP_table[] entry": "no config table", q->desc);
    return;
  }

  randflashconfig(pgm, q, up, cp, nc);
  if(flashlayout(pgm, q, flm, up, cp, nc) < 0)
    return;

  int vtb = putvectortable(q, flm, dry.appstart, dry.init), urbtsz = 0;

  int urboot = random()%3 && dry.bootsize <= 512 && flm->size >= 1024 &&
    flm->size >= 4*dry.bootsize && is_classic(q) && is_spm(q);
  if(urboot) {                  // Give some classic parts a small bootloader
    int ps = flm->page_size;

    urbtsz = dry.bootsize? dry.bootsize: flm->size > 32768? 512: flm->size < 16384? 256: 384;
    urbtsz = (urbtsz + ps - 1)/ps*ps;
    if(!dry.bootsize && !dry.datasize) {
      dry.bootsize += urbtsz;
      dry.appsize -= urbtsz;
      dry.bootstart = dry.appsize;
    }
    int ubaddr = dry.bootstart;

    putflash(pgm, dry.dp, flm, ubaddr, urbtsz, urbtsz == 384? U384: U512);
  } else if(dry.bootsize) {
    int btb = 0;

    if(dry.bootsize >= 2048)
      btb = putvectortable(q, flm, dry.bootstart, dry.init);
    putflash(pgm, dry.dp, flm, dry.bootstart + btb, dry.bootsize - btb, BDATA);
  }

  if(dry.datasize)
    putflash(pgm, dry.dp, flm, dry.datastart, dry.datasize, ADATA);

  putflash(pgm, dry.dp, flm, dry.appstart + vtb, dry.appsize - vtb - urbtsz, ROCKS);

  for(int i = 0; i < flm->size; i += flm->page_size)
    sharedflash(pgm, flm, i, flm->page_size);

  if((m = avr_locate_eeprom(q)))
    putother(pgm, q, m, "The quick brown fox jumps over the lazy dog. ");
  if((m = avr_locate_userrow(q)))
    putother(pgm, q, m, "The five boxing wizards jump quickly. ");
  if((m = avr_locate_bootrow(q)))
    putother(pgm, q, m, "Lorem ipsum dolor sit amet. ");

  dry.initialised = 1;
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

    for(; addr < end; addr += chunk) {
      chunk = end - addr < page_size? end - addr: page_size;
      // @@@ Check for bootloader write protection here

      // Unless it is a bootloader flash looks like NOR-memory
      (mchr == 'F' && !dry.bl? memand: memcpy) (dmem->buf + addr, m->buf + addr, chunk);

      // Copy chunk to overlapping XMEGA's apptable, application, boot and flash memories
      if(mchr == 'F')
        sharedflash(pgm, dmem, addr, chunk);
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
      chunk = end - addr < page_size? end - addr: page_size;
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

  // @@@ check for bootloader write protection

  if(dry.initialised && (mem_is_in_fuses(mem) || mem_is_lock(mem)))
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
      rc = LIBAVRDUDE_EXIT;
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
  pgm->parseextparams = dryrun_parseextparams;
}
