/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2026- Stefan Rueger <stefan.rueger@urclocks.com>
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
 * AVRPART *dryrun_part(const char *id, int *bootsizep,
 *   int init, int random, int holes, int seed)
 *
 * Returns a duplicate of the part structure that is known from the
 * configuration file under the given id (eg, m328p). Lock and fuse
 * memories are initialised with with factory values as far as known, 0xff
 * otherwise. The signature memory is set from the configuration file; the
 * calibration memory is filled with U (for uncalibrated); osc16err with e
 * and osc20err with E (for error); osccal16 with o and osccal20 with O;
 * sib with S; tempsense with T; sernum with the downward letter sequence
 * UTSRQP...; and the volatile io memory with reset values if known, 0x00
 * otherwise.
 *
 * If either the init or random parameters are set, then the flash memory
 * is randomly configured in terms of bootloader sections, code and
 * application data sections, and the fuses updated accordingly. In either
 * case, flash (including ATxmega submemories of application, apptable and
 * boot), eeprom, and all other existing memories such as prodsig/sigrow,
 * userrow/usersig and bootrow are updated with random data. Flash is
 * always initialised with benign code, that is its opcodes will not
 * access I/O memories, SRAM or flash.
 *
 * If none of init or random parameters are set, these memories are
 * initialised with 0xff. Note that init and random are not meant to be
 * both set at the same time.
 *
 * If init is set then, the patterns that are used for initialising
 * memories as detailed above are human-readable. These patterns can best
 * be seen with a fixed-width font and the :I format by inspecting the
 * generated hex file or by using, eg, -U flash:r:-:I to dump the patterns
 * on screen. eeprom, userrow/usersig and bootrow memories are filled with
 * pangrams such as The quick brown fox jumps over the lazy dog.
 *
 * If random is set flash is initialised with random opcodes and, if
 * applicable, random application table data. The sernum memory, if it
 * exists, will be initialised with a random upper-letter sequence. Other
 * memories are initialised with a random sequence of at-signs and spaces.
 *
 * If holes is set then dryrun_parts() puts holes into larger memories,
 * ie, longer sequences of 0xff, and adds small islands of code or data.
 * Some of these holes can pose problems for programmers that do not
 * anticipate them. As such these can be used for hardened testing, which
 * is the main purpose of the dryrun programmers
 *
 * The argument seed, if positive, initialises the seed of the pseudo
 * random number generator. If seed is zero, time(NULL) is used for
 * initialisation, ie, subsequent calls of dryrun_part() with the same
 * arguments differ in the initialisation of the part. Use a positive seed
 * for reproducible, but random, initialisation.
 *
 * If the pointer bootsizep is not NULL, the integer pointed to will be
 * set to the size of the configured boot section in bytes and 0 if no
 * boot section was configured.
 *
 * The caller is responsible for deallocating the memory associated with
 * the returned structure pointer by using avr_free_part().
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

#include "avrdude.h"
#include "libavrdude.h"

#define random() rand()         // For platform independence
#define srandom(n) srand(n)

// Use Context data for functions in this file as if they were a global structure me
#define me (*mep)

typedef struct {
  AVRPART *dp;
  unsigned char fuses[16];      // Cache of lfuse, hfuse, efuse or, generally, fuses memory
  int lock;                     // Cache of lock (unused)
  int init;                     // Initialise memories with something interesting
  int random;                   // Random initialisation of memories
  int holes;                    // Whether eeprom/flash should have holes
  int seed;                     // Seed for random number generator
  // Flash configuration irrespective of -c programming is bootloading or not
  int appstart, appsize;        // Start and size of application section
  int datastart, datasize;      // Start and size of application data section (if any)
  int bootstart, bootsize;      // Start and size of boot section (if any)
} Testpart_data;


// Fill in lock/fuse associated with config item and the pointer to Configitem record
static int locate_config_c_value(const Testpart_data *mep, const AVRPART *p,
  const char *cname, const Configitem **cp, int *valp) {

  int nc = 0;
  const Configitem *cfg = avr_locate_configitems(p, &nc);

  if(!cfg || nc < 1) {
    pmsg_error("avrintel.c does not hold configuration information for %s\n", p->desc);
    return -1;
  }

  const Configitem *c = avr_locate_config(cfg, nc, cname, str_contains);

  if(!c) {
    pmsg_error("%s does not have a unique config item matched by %s\n", p->desc, cname);
    return -1;
  }

  *cp = c;                      // @@@ does not work for pdicfg as that spans 2 fuse bytes
  *valp = str_starts(c->memstr, "lock")? me.lock: me.fuses[c->memoffset];
  return 0;
}

// Initialise *valuep with configuration value of named configuration bitfield (does not work for pdicfg)
static int get_config_value(const Testpart_data *mep, const AVRPART *p, const char *cname, int *valuep) {
  const Configitem *c;
  int fusel;

  if(locate_config_c_value(mep, p, cname, &c, &fusel) < 0)
    return -1;

  if(valuep)
    *valuep = (fusel & c->mask) >> c->lsh;
  return 0;
}

// Set to value in mep's lock/fuses the configuration value of named configuration bitfield
static int set_config_value(Testpart_data *mep, const AVRPART *p, const char *cname, int value) {
  const Configitem *c;
  int fusel;

  if(locate_config_c_value(mep, p, cname, &c, &fusel) < 0)
    return -1;

  if((value << c->lsh) & ~c->mask)
    pmsg_warning("value 0x%02x for %s has bits set outside bitfield mask 0x%02x\n", value, cname, c->mask >> c->lsh);

  int newval = (fusel & ~c->mask) | ((value << c->lsh) & c->mask);

  if(str_starts(c->memstr, "lock"))
    me.lock = newval;
  else {
    me.fuses[c->memoffset] = newval; // Does not work for pdicfg

    // Write to the the corresponding fuse of dp and its fuses memory, if any
    AVRMEM *m = avr_locate_fuse_by_offset(me.dp, c->memoffset);
    if(m)
      m->buf[0] = newval;
    if((m = avr_locate_fuses(me.dp)))
      m->buf[c->memoffset] = newval;
  }

  return 0;
}

// Randomly set configuration values for bootloading, bootloader size and codesize, if any
static void randflashconfig(Testpart_data *mep, const AVRPART *p, const Avrintel *up,
  const Configitem *cp, int nc) {

  if(up && is_updi(p)) {
    int sectorsize = up->bootsize > 0? up->bootsize: 256;
    int nsectors = up->flashsize/sectorsize;
    int bootsize = random()%(nsectors > 4? nsectors/4: nsectors);
    int codesize = !bootsize || random()%3? 0: bootsize + random()%(nsectors - bootsize);

    int size = !!avr_locate_config(cp, nc, "bootsize", str_eq);

    set_config_value(mep, p, size? "bootsize": "bootend", bootsize);
    set_config_value(mep, p, size? "codesize": "append", codesize);
  } else if(up && up->nboots > 0 && (p->prog_modes & (PM_Classic | PM_PDI))) {
    set_config_value(mep, p, "bootrst", random()%2);
    if(up->nboots == 4)
      set_config_value(mep, p, "bootsz", random()%4);
  }
}

#define Retwarning(...) do { pmsg_warning(__VA_ARGS__); \
  msg_warning("; not initialising %s memories\n", p->desc); return -1; } while(0)

// Compute app, data and boot start/size
static int flashlayout(Testpart_data *mep, const AVRPART *p, const AVRMEM *flm,
  const Avrintel *up, const Configitem *cp, int nc) {

  AVRMEM *m;

  if(is_updi(p)) {
    int nbootsec = 0, ncodesec = 0;

    int size = !!avr_locate_config(cp, nc, "bootsize", str_eq);

    get_config_value(mep, p, size? "bootsize": "bootend", &nbootsec);
    get_config_value(mep, p, size? "codesize": "append", &ncodesec);
    if(nbootsec == 0 || (ncodesec && ncodesec <= nbootsec)) {   // Treat boot section for code
      me.bootstart = 0, me.bootsize = 0;
      me.appstart = 0, me.appsize = nbootsec? nbootsec*up->bootsize: up->flashsize;
    } else {                    // Distinct boot and application section
      me.bootstart = 0, me.bootsize = nbootsec*up->bootsize;
      me.appstart = me.bootsize;
      me.appsize = ncodesec? (ncodesec - nbootsec)*up->bootsize: up->flashsize - me.appstart;
    }
    me.datasize = up->flashsize - me.bootsize - me.appsize;  // Remainder is apptable
    me.datastart = me.datasize? me.bootsize + me.appsize: 0;
  } else if(p->prog_modes & (PM_Classic | PM_PDI)) {
    me.bootstart = 0, me.bootsize = 0;
    if(up->nboots) {
      int bootrst = 1;

      get_config_value(mep, p, "bootrst", &bootrst);
      if(bootrst == 0) {        // Jump to bootloader on reset
        if(is_pdi(p) && (m = avr_locate_boot(p)) && m->size > 0) {
          me.bootstart = m->offset - flm->offset;
          me.bootsize = m->size;
        } else if(is_classic(p)) {
          if(up->nboots == 4) {
            int bootsz = 0;

            get_config_value(mep, p, "bootsz", &bootsz);
            me.bootsize = (8 >> bootsz)*up->bootsize;
          } else
            me.bootsize = up->bootsize;
          me.bootstart = up->flashsize - me.bootsize;
        }
      }
    }
    me.datastart = 0, me.datasize = 0;
    if(is_pdi(p) && (m = avr_locate_apptable(p)) && m->size > 0) {
      me.datastart = m->offset - flm->offset;
      me.datasize = up->flashsize - me.datastart - me.bootsize;
    }
    me.appstart = 0, me.appsize = up->flashsize - me.datasize - me.bootsize;
  }

  // Sanity checks
  if(me.appsize < 0)
    Retwarning("negative application size");
  if(me.appstart < 0 || me.appstart + me.appsize > up->flashsize)
    Retwarning("application section %s outside flash [0, 0x%04x]",
      str_ccinterval(me.appstart, me.appstart + me.appsize - 1), up->flashsize - 1);

  if(me.datasize < 0)
    Retwarning("negative apptable size");
  if(me.datastart < 0 || me.datastart + me.datasize > up->flashsize)
    Retwarning("apptable section %s outside flash [0, 0x%04x]",
      str_ccinterval(me.datastart, me.datastart + me.datasize - 1), up->flashsize - 1);

  if(me.bootsize < 0)
    Retwarning("negative boot section size");
  if(me.bootstart < 0 || me.bootstart + me.bootsize > up->flashsize)
    Retwarning("boot section %s outside flassh [0, 0x%04x]",
      str_ccinterval(me.bootstart, me.bootstart + me.bootsize - 1), up->flashsize - 1);

  if(me.appsize + me.datasize + me.bootsize != up->flashsize)
    Retwarning("section sizes do not add up (0x%x) to flash size 0x%x",
      me.appsize + me.datasize + me.bootsize, up->flashsize);

  if(!me.appsize)
    Retwarning("no application section");

  if(is_updi(p)) {
    if(me.bootsize && me.appstart != me.bootsize)
      Retwarning("application section %s does not touch boot section %s",
        str_ccinterval(me.appstart, me.appstart + me.appsize - 1),
        str_ccinterval(me.bootstart, me.bootstart + me.bootsize - 1));
    if(me.datasize && me.datastart != me.bootsize + me.appsize)
      Retwarning("apptable section %s does not touch code section %s",
        str_ccinterval(me.datastart, me.datastart + me.appsize - 1),
        str_ccinterval(0, me.bootsize + me.appsize - 1));
  } else {
    if(me.datasize && me.datastart != me.appsize && me.appstart != 0)
      Retwarning("apptable section %s does not touch application section %s",
        str_ccinterval(me.datastart, me.datastart + me.appsize - 1),
        str_ccinterval(me.appstart, me.appstart + me.appsize - 1));
    if(me.datasize && me.bootsize && me.bootstart != me.appsize + me.datasize)
      Retwarning("apptable section %s does not touch boot section %s",
        str_ccinterval(me.datastart, me.datastart + me.appsize - 1),
        str_ccinterval(me.bootstart, me.bootstart + me.bootsize - 1));
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

static const struct {
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
static void putflash(const Testpart_data *mep, const AVRPART *p, const AVRMEM *flm, int addr, int n, int bi) {
  unsigned char *top = flm->buf + addr + n - 4;

  if(me.random) {
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
static void putother(const Testpart_data *mep, const AVRPART *p, const AVRMEM *m, const char *str) {
  const char *name = avr_mem_name(p, m), *hi = me.random? "@  @": "Hello, world!";
  int len = strlen(str);

  if(len > m->size)
    len = m->size;
  if(len <= 0)
    return;

  memset(m->buf, 0xff, m->size);

  if(me.random)
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

  if(me.holes && m->size >= 64) {
    // Remove an initial, a middling and a final section
    int delta[4];               // Random number between -2 and 2
    for(size_t i = 0; i < sizeof delta/sizeof *delta; i++)
      delta[i] = random()%5 - 2;
    memset(m->buf, 0xff, m->size/8 + delta[0]);
    memset(m->buf + m->size/2 + delta[1], 0xff, m->size/4 + delta[2]);
    memcpy(m->buf + m->size/2 + delta[1] + 3, hi, strlen(hi));
    int len = m->size/8 + delta[3];
    memset(m->buf + m->size - len, 0xff, len);
  }
}

AVRPART *dryrun_part(const char *partid, int *bootsizep, int init, int random, int holes, int seed) {
  pmsg_debug("%s()\n", __func__);

  const AVRPART *p = locate_part(part_list, partid);
  if(!p) {
    pmsg_error("cannot find part with id %s\n", partid);
    return NULL;
  }
  Testpart_data *mep = mmt_malloc(sizeof(Testpart_data));
  AVRPART *q = me.dp = avr_dup_part(p);   // Allocate dryrun part and abbreviate with q
  AVRMEM *m, *fusesm = NULL, *prodsigm = NULL, *calm;

  me.init = init;               // Initialise memories with something interesting
  me.random = random;           // Random initialisation of memories
  me.holes = holes;             // Whether eeprom/flash should have holes
  me.seed = seed;               // Seed for random number generator
  memset(me.fuses, 0xff, sizeof me.fuses);
  srandom(me.seed? me.seed: time(NULL));

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
        for(int i = 0; i < m->size; i++)
          m->buf[i] = m->initval >> 8*i;
        if(mem_is_a_fuse(m)) {
          int fno = mem_fuse_offset(m);

          for(int i = 0; i < m->size && fno + i < (int) sizeof me.fuses; i++) // pdicfg has 2 bytes
            me.fuses[fno + i] = m->initval >> 8*i;
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
        m->buf[i] = me.random? 'A' + random()%26: 'U' - i >= 'A'? 'U' - i: 0xff;
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
              for(int k = 0; k < rf[i].size; k++)
                m->buf[rf[i].addr + k] = rf[i].initval >> 8*k;
    }
  }
  if(prodsigm) {
    if(q->prog_modes & (PM_UPDI | PM_PDI)) {
      for(LNODEID ln = lfirst(q->mem); ln; ln = lnext(ln)) {
        m = ldata(ln);
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
  if(fusesm)
    memcpy(fusesm->buf, me.fuses, minm((size_t) fusesm->size, sizeof me.fuses));

  if(!me.random && !me.init)  // OK, no further initialisation needed
    goto finished;

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
    goto finished;
  }

  randflashconfig(mep, q, up, cp, nc);
  if(flashlayout(mep, q, flm, up, cp, nc) < 0)
    goto finished;

  int vtb = putvectortable(q, flm, me.appstart, me.init), urbtsz = 0;

  int urboot = random()%3 && me.bootsize <= 512 && flm->size >= 1024 &&
    flm->size >= 4*me.bootsize && is_classic(q) && is_spm(q);
  if(urboot) {                  // Give some classic parts a small bootloader
    int ps = flm->page_size;

    urbtsz = me.bootsize? me.bootsize: flm->size > 32768? 512: flm->size < 16384? 256: 384;
    urbtsz = (urbtsz + ps - 1)/ps*ps;
    if(!me.bootsize && !me.datasize) {
      me.bootsize += urbtsz;
      me.appsize -= urbtsz;
      me.bootstart = me.appsize;
    }
    int ubaddr = me.bootstart;

    putflash(mep, me.dp, flm, ubaddr, urbtsz, urbtsz == 384? U384: U512);
  } else if(me.bootsize) {
    int btb = 0;

    if(me.bootsize >= 2048)
      btb = putvectortable(q, flm, me.bootstart, me.init);
    putflash(mep, me.dp, flm, me.bootstart + btb, me.bootsize - btb, BDATA);
  }

  if(me.datasize)
    putflash(mep, me.dp, flm, me.datastart, me.datasize, ADATA);

  putflash(mep, me.dp, flm, me.appstart + vtb, me.appsize - vtb - urbtsz, ROCKS);

  if(me.holes && me.appsize >= 128) { // Generate holes in the code section
    int start = me.appstart & ~1, size = me.appsize & ~3;
    int len3 = size/3 & ~3, len4 = size/4 - 1;
    unsigned char *code = flm->buf + start;

    /*
     * Cut away just shy of 1/4 of flash either side deliberately making the
     * hole odd-sized. Overwrite odd boundary with a space (0x20): note that
     * the opcodes 0x20ff (sbrs r18, 0) and  0xff20 (tst r15) are benign. Then
     * cut off the central third of the code section and introduce an island
     * with a single space in the middle (generating a single tst r15 opcode).
     */
    memset(code, 0xff, len4); code[len4] = ' ';
    memset(code + size - len4, 0xff, len4); code[size - len4 - 1] = ' ';
    memset(code + len3, 0xff, len3); code[size/2 - 1] = ' ';
    // Terminate code section with two endless loops
    code[size - 4] = code[size - 2] = 0xff;
    code[size - 3] = code[size - 1] = 0xcf;
  }

  // Initialise other overlapping flash memories from flash (think XMEGA)
  for(LNODEID ln = lfirst(me.dp->mem); ln; ln = lnext(ln)) {
    m = ldata(ln);

    if(mem_is_in_flash(m) && flm != m) { // Overlapping flash memories?
      unsigned int faddr = m->offset - flm->offset;

      if(faddr < (unsigned int) flm->size && faddr + m->size <= (unsigned int) flm->size)
        memcpy(m->buf, flm->buf + faddr, m->size);
    }
  }

  if((m = avr_locate_eeprom(q)))
    putother(mep, q, m, "The quick brown fox jumps over the lazy dog. ");
  if((m = avr_locate_userrow(q)))
    putother(mep, q, m, "The five boxing wizards jump quickly. ");
  if((m = avr_locate_bootrow(q)))
    putother(mep, q, m, "Lorem ipsum dolor sit amet. ");

finished:
  if(bootsizep)
    *bootsizep = me.bootsize;
  AVRPART *ret = me.dp;
  mmt_free(mep);

  return ret;
}
