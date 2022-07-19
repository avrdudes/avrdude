/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022, Stefan Rueger <smr@theblueorange.space>
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
 * Code to program an Atmel AVR device through one of the supported
 * programmers.
 *
 * For parallel port connected programmers, the pin definitions can be
 * changed via a config file.  See the config file for instructions on
 * how to add a programmer definition.
 *
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <whereami.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fnmatch.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "developer_opts.h"
#include "developer_opts_private.h"

static char cmdbitchar(CMDBIT cb) {
  switch(cb.type) {
  case AVR_CMDBIT_IGNORE:
    return 'x';
  case AVR_CMDBIT_VALUE:
    return cb.value? '1': '0';
  case AVR_CMDBIT_ADDRESS:
    return 'a';
  case AVR_CMDBIT_INPUT:
    return 'i';
  case AVR_CMDBIT_OUTPUT:
    return 'o';
  default:
    return '?';
  }
}

static char *cmdbitstr(CMDBIT cb) {
  char space[10];

  *space = cmdbitchar(cb);
  if(*space == 'a')
    sprintf(space+1, "%d", cb.bitno);
  else
    space[1] = 0;

  return strdup(space);
}



static const char *opcodename(int what) {
  switch(what) {
  case AVR_OP_READ:
    return "read";
  case AVR_OP_WRITE:
    return "write";
  case AVR_OP_READ_LO:
    return "read_lo";
  case AVR_OP_READ_HI:
    return "read_hi";
  case AVR_OP_WRITE_LO:
    return "write_lo";
  case AVR_OP_WRITE_HI:
    return "write_hi";
  case AVR_OP_LOADPAGE_LO:
    return "loadpage_lo";
  case AVR_OP_LOADPAGE_HI:
    return "loadpage_hi";
  case AVR_OP_LOAD_EXT_ADDR:
    return "load_ext_addr";
  case AVR_OP_WRITEPAGE:
    return "writepage";
  case AVR_OP_CHIP_ERASE:
    return "chip_erase";
  case AVR_OP_PGM_ENABLE:
    return "pgm_enable";
  default:
    return "???";
  }
}


// Unique string representation of an opcode
static char *opcode2str(OPCODE *op, int opnum, int detailed) {
  char cb, space[1024], *sp = space;
  int compact = 1;

  if(!op)
    return strdup("NULL");

  // Can the opcode be printed in a compact way? Only if address bits are systematic.
  for(int i=31; i >= 0; i--)
    if(op->bit[i].type == AVR_CMDBIT_ADDRESS)
      if(i<8 || i>23 || op->bit[i].bitno != (opnum == AVR_OP_LOAD_EXT_ADDR? i+8: i-8))
        compact = 0;

  if(detailed)
    *sp++ = '"';

  for(int i=31; i >= 0; i--) {
    *sp++ = cb = cmdbitchar(op->bit[i]);
    if(compact) {
      if(i && i%8 == 0)
        *sp++ = '-', *sp++ = '-';
      else if(i && i%4 == 0)
        *sp++ = '.';
    } else {
      if(cb == 'a') {
        sprintf(sp, "%d", op->bit[i].bitno);
        sp += strlen(sp);
      }
      if(i) {
        if(detailed)
          *sp++ = ' ';
        if(i%8 == 0)
          *sp++ = ' ';
      }
    }
  }
  if(detailed)
    *sp++ = '"';
  *sp = 0;

  return strdup(space);
}


// return 0 if op code would encode (essentially) the same SPI command
static int opcodecmp(OPCODE *op1, OPCODE *op2, int opnum) {
  char *opstr1, *opstr2, *p;
  int cmp;

  if(!op1 && !op2)
    return 0;
  if(!op1 || !op2)
    return op1? -1: 1;

  opstr1 = opcode2str(op1, opnum, 1);
  opstr2 = opcode2str(op2, opnum, 1);
  if(!opstr1 || !opstr2) {
    dev_info("%s: out of memory\n", progname);
    exit(1);
  }

  // don't care x and 0 are functionally equivalent
  for(p=opstr1; *p; p++)
    if(*p == 'x')
      *p = '0';
  for(p=opstr2; *p; p++)
    if(*p == 'x')
      *p = '0';

  cmp = strcmp(opstr1, opstr2);
  free(opstr1);
  free(opstr2);

  return cmp;
}




static void printopcode(AVRPART *p, const char *d, OPCODE *op, int what) {
  unsigned char cmd[4];
  int i;

  if(op) {
    memset(cmd, 0, sizeof cmd);
    avr_set_bits(op, cmd);

    dev_info(".op\t%s\t%s\t%s\t0x%02x%02x%02x%02x\t", p->desc, d, opcodename(what), cmd[0], cmd[1], cmd[2], cmd[3]);
    for(i=31; i >= 0; i--) {
      dev_info("%c", cmdbitchar(op->bit[i]));
      if(i%8 == 0)
        dev_info("%c", i? '\t': '\n');
    }
  }
}

static void printallopcodes(AVRPART *p, const char *d, OPCODE **opa) {
  for(int i=0; i<AVR_OP_MAX; i++)
    printopcode(p, d, opa[i], i);
}


// returns position 0..31 of highest bit set or INT_MIN if no bit is set
static int intlog2(unsigned int n) {
  int ret;

  if(!n)
    return INT_MIN;

  for(ret = 0; n >>= 1; ret++)
    continue;

  return ret;
}


// mnemonic characterisation of flags
static char *parttype(AVRPART *p) {
  static char type[1024];

  switch(p->flags & (AVRPART_HAS_PDI | AVRPART_AVR32 | AVRPART_HAS_TPI | AVRPART_HAS_UPDI)) {
  case 0:                strcpy(type, "ISP"); break;
  case AVRPART_HAS_PDI:  strcpy(type, "PDI"); break;
  case AVRPART_AVR32:    strcpy(type, "AVR32"); break;
  case AVRPART_HAS_TPI:  strcpy(type, "TPI"); break;
  case AVRPART_HAS_UPDI: strcpy(type, "UPDI"); break;
  default:               strcpy(type, "UNKNOWN"); break;
  }

  if((p->flags & AVRPART_SERIALOK) == 0)
    strcat(type, "|NOTSERIAL");
  if((p->flags & AVRPART_PARALLELOK) == 0)
    strcat(type, "|NOTPARALLEL");
  if(p->flags & AVRPART_PSEUDOPARALLEL)
    strcat(type, "|PSEUDOPARALLEL");
  if(p->flags & AVRPART_IS_AT90S1200)
    strcat(type, "|IS_AT90S1200");

  if(p->flags & AVRPART_HAS_DW)
    strcat(type, "|DW");

  if(p->flags & AVRPART_HAS_JTAG)
    strcat(type, "|JTAG");
  if(p->flags & AVRPART_ALLOWFULLPAGEBITSTREAM)
    strcat(type, "|PAGEBITSTREAM");
  if((p->flags & AVRPART_ENABLEPAGEPROGRAMMING) == 0)
    strcat(type, "|NOPAGEPROGRAMMING");

  return type;
}


// check whether address bits are where they should be in ISP commands
static void checkaddr(int memsize, int pagesize, int what, OPCODE *op, AVRPART *p, AVRMEM *m) {
  int i, lo, hi;
  const char *whatstr = opcodename(what);

  lo = intlog2(pagesize);
  hi = intlog2(memsize-1);

  // address bits should be between positions lo and hi (and fall in line), outside should be 0 or don't care
  for(i=0; i<16; i++) {         // ISP programming only deals with 16-bit addresses (words for flash, bytes for eeprom)
    if(i < lo || i > hi) {
      if(op->bit[i+8].type != AVR_CMDBIT_IGNORE && !(op->bit[i+8].type == AVR_CMDBIT_VALUE && op->bit[i+8].value == 0)) {
        char *cbs = cmdbitstr(op->bit[i+8]);
        dev_info(".cmderr\t%s\t%s-%s\tbit %d outside addressable space should be x or 0 but is %s\n", p->desc, m->desc, whatstr, i+8, cbs? cbs: "NULL");
        if(cbs)
          free(cbs);
      }
    } else {
      if(op->bit[i+8].type != AVR_CMDBIT_ADDRESS)
        dev_info(".cmderr\t%s\t%s-%s\tbit %d is %c but should be a\n", p->desc, m->desc, whatstr, i+8, cmdbitchar(op->bit[i+8]));
      else if(op->bit[i+8].bitno != i)
        dev_info(".cmderr\t%s\t%s-%s\tbit %d inconsistent: a%d specified as a%d\n", p->desc, m->desc, whatstr, i+8, i, op->bit[i+8].bitno);
    }
  }
  for(i=0; i<32; i++)           // command bits 8..23 should not contain address bits
    if((i<8 || i>23) && op->bit[i].type == AVR_CMDBIT_ADDRESS)
      dev_info(".cmderr\t%s\t%s-%s\tbit %d contains a%d which it shouldn't\n", p->desc, m->desc, whatstr, i, op->bit[i].bitno);
}


static char *dev_sprintf(const char *fmt, ...) {
  int size = 0;
  char *p = NULL;
  va_list ap;

  // compute size
  va_start(ap, fmt);
  size = vsnprintf(p, size, fmt, ap);
  va_end(ap);

  if(size < 0)
    return NULL;

  size++;                       // for temrinating '\0'
  if(!(p = malloc(size)))
   return NULL;

  va_start(ap, fmt);
  size = vsnprintf(p, size, fmt, ap);
  va_end(ap);

  if(size < 0) {
    free(p);
    return NULL;
  }

  return p;
}


static int dev_nprinted;

int dev_message(int msglvl, const char *fmt, ...) {
  va_list ap;
  int rc = 0;

  if(verbose >= msglvl) {
    va_start(ap, fmt);
    rc = vfprintf(stderr, fmt, ap);
    va_end(ap);
    if(rc > 0)
      dev_nprinted += rc;
  }

  return rc;
}



static int dev_part_strct_entry(bool tsv, char *col0, char *col1, char *col2, const char *name, char *cont) {
  const char *n = name? name: "name_error";
  const char *c = cont? cont: "cont_error";

  if(tsv) {                     // tab separated values
    if(col0) {
      dev_info("%s\t", col0);
      if(col1) {
        dev_info("%s\t", col1);
        if(col2) {
          dev_info("%s\t", col2);
        }
      }
    }
    dev_info("%s\t%s\n", n, c);
  } else {                      // grammar conform
    int indent = col2 && strcmp(col2, "part");

    printf("%*s%-*s = %s;\n", indent? 8: 4, "", indent? 15: 19, n, c);
  }

  if(cont)
    free(cont);

  return 1;
}


static const char *dev_controlstack_name(AVRPART *p) {
  return
    p->ctl_stack_type == CTL_STACK_PP? "pp_controlstack":
    p->ctl_stack_type == CTL_STACK_HVSP? "hvsp_controlstack":
    p->ctl_stack_type == CTL_STACK_NONE? "NULL":
    "unknown_controlstack";
}


static void dev_stack_out(bool tsv, AVRPART *p, const char *name, unsigned char *stack, int ns) {
  if(!strcmp(name, "NULL")) {
    name = "pp_controlstack";
    ns = 0;
  }

  if(tsv)
    dev_info(".pt\t%s\t%s\t", p->desc, name);
  else
    dev_info("    %-19s =%s", name, ns <=8? " ": "");

  if(ns <= 0)
    dev_info(tsv? "NULL\n": "NULL;\n");
  else
    for(int i=0; i<ns; i++)
      dev_info("%s0x%02x%s", !tsv && ns > 8 && i%8 == 0? "\n        ": "", stack[i], i+1<ns? ", ": tsv? "\n": ";\n");
}


// order in which memories are processed, runtime adds unknown ones
static const char *mem_order[100] = {
  "eeprom",       "flash",        "application",  "apptable",
  "boot",         "lfuse",        "hfuse",        "efuse",
  "fuse",         "fuse0",        "wdtcfg",       "fuse1",
  "bodcfg",       "fuse2",        "osccfg",       "fuse3",
  "fuse4",        "tcd0cfg",      "fuse5",        "syscfg0",
  "fuse6",        "syscfg1",      "fuse7",        "append",
  "codesize",     "fuse8",        "fuse9",        "bootend",
  "bootsize",     "fuses",        "lock",         "lockbits",
  "tempsense",    "signature",    "prodsig",      "sernum",
  "calibration",  "osccal16",     "osccal20",     "osc16err",
  "osc20err",     "usersig",      "userrow",      "data",
};

static void add_mem_order(const char *str) {
  for(int i=0; i < sizeof mem_order/sizeof *mem_order; i++) {
    if(mem_order[i] && !strcmp(mem_order[i], str))
      return;
    if(!mem_order[i]) {
      mem_order[i] = strdup(str);
      return;
    }
  }
  dev_info("%s: mem_order[] under-dimensioned in developer_opts.c; increase and recompile\n", progname);
  exit(1);
}


static int intcmp(int a, int b) {
  return a-b;
}


// deep copies for comparison and raw output

typedef struct {
  AVRMEM base;
  OPCODE ops[AVR_OP_MAX];
} AVRMEMdeep;

static int avrmem_deep_copy(AVRMEMdeep *d, AVRMEM *m) {
  int len;

  d->base = *m;

  // zap all bytes beyond terminating nul of desc array
  len = strlen(m->desc)+1;
  if(len < sizeof m->desc)
    memset(d->base.desc + len, 0, sizeof m->desc - len);

  // zap address values
  d->base.buf = NULL;
  d->base.tags = NULL;
  for(int i=0; i<AVR_OP_MAX; i++)
    d->base.op[i] = NULL;


  // copy over the SPI operations themselves
  memset(d->base.op, 0, sizeof d->base.op);
  memset(d->ops, 0, sizeof d->ops);
  for(int i=0; i<sizeof d->ops/sizeof *d->ops; i++)
    if(m->op[i])
      d->ops[i] = *m->op[i];

  return 0;
}

static int memorycmp(AVRMEM *m1, AVRMEM *m2) {
  AVRMEMdeep dm1, dm2;

  if(!m1 && !m2)
    return 0;

  if(!m1 || !m2)
    return m1? -1: 1;
  
  avrmem_deep_copy(&dm1, m1);
  avrmem_deep_copy(&dm2, m2);

  return memcmp(&dm1, &dm2, sizeof dm1);
}


typedef struct {
  AVRPART base;
  OPCODE ops[AVR_OP_MAX];
  AVRMEMdeep mems[40];
} AVRPARTdeep;

static int avrpart_deep_copy(AVRPARTdeep *d, AVRPART *p) {
  AVRMEM *m;
  int len, di;

  memset(d, 0, sizeof *d);

  d->base = *p;

  d->base.config_file = NULL;
  d->base.lineno = 0;

  // zap all bytes beyond terminating nul of desc, id and family_id array
  len = strlen(p->desc);
  if(len < sizeof p->desc)
    memset(d->base.desc + len, 0, sizeof p->desc - len);

  len = strlen(p->family_id);
  if(len < sizeof p->family_id)
    memset(d->base.family_id + len, 0, sizeof p->family_id - len);

  len = strlen(p->id);
  if(len < sizeof p->id)
    memset(d->base.id + len, 0, sizeof p->id - len);

  // zap address values
  d->base.mem = NULL;
  d->base.mem_alias = NULL;
  for(int i=0; i<AVR_OP_MAX; i++)
    d->base.op[i] = NULL;

  // copy over the SPI operations
  memset(d->base.op, 0, sizeof d->base.op);
  memset(d->ops, 0, sizeof d->ops);
  for(int i=0; i<AVR_OP_MAX; i++)
    if(p->op[i])
      d->ops[i] = *p->op[i];

  // fill in all memories we got in defined order
  di = 0;
  for(int mi=0; mi < sizeof mem_order/sizeof *mem_order && mem_order[mi]; mi++) {
    m = p->mem? avr_locate_mem(p, mem_order[mi]): NULL;
    if(m) {
      if(di >= sizeof d->mems/sizeof *d->mems) {
        avrdude_message(MSG_INFO, "%s: ran out of mems[] space, increase size in AVRMEMdeep of developer_opts.c and recompile\n", progname);
        exit(1);
      }
      avrmem_deep_copy(d->mems+di, m);
      di++;
    }
  }

  return di;
}

static char txtchar(unsigned char in) {
  in &= 0x7f;
  return in == ' '? '_': in > ' ' && in < 0x7f? in: '.';
}


static void dev_raw_dump(unsigned char *p, int nbytes, const char *name, const char *sub, int idx) {
  unsigned char *end = p+nbytes;
  int n = ((end - p) + 15)/16;

  for(int i=0; i<n; i++, p += 16) {
    dev_info("%s\t%s\t%02x%04x: ", name, sub, idx, i*16);
    for(int j=0; j<16; j++)
      dev_info("%02x", p+i*16+j<end? p[i*16+j]: 0);
    dev_info(" ");
    for(int j=0; j<16; j++)
      dev_info("%c", txtchar(p+i*16+j<end? p[i*16+j]: 0));
    dev_info("\n");
  }
}


static void dev_part_raw(AVRPART *part) {
  AVRPARTdeep dp;
  int di = avrpart_deep_copy(&dp, part);

  dev_raw_dump((unsigned char *) &dp.base, sizeof dp.base, part->desc, "part", 0);
  dev_raw_dump((unsigned char *) &dp.ops, sizeof dp.ops, part->desc, "ops", 1);

  for(int i=0; i<di; i++)
    dev_raw_dump((unsigned char *) (dp.mems+i), sizeof dp.mems[i], part->desc, dp.mems[i].base.desc, i+2);
}


static void dev_part_strct(AVRPART *p, bool tsv, AVRPART *base) {
  dev_info("# %s %d\n", p->config_file, p->lineno);

  if(!tsv)
    dev_info("part\n");

  _if_partout(strcmp, "\"%s\"", desc);
  _if_partout(strcmp, "\"%s\"", id);
  _if_partout(strcmp, "\"%s\"", family_id);
  _if_partout(intcmp, "%d", hvupdi_variant);
  _if_partout(intcmp, "0x%02x", stk500_devcode);
  _if_partout(intcmp, "0x%02x", avr910_devcode);
  _if_partout(intcmp, "%d", chip_erase_delay);
  _if_partout(intcmp, "0x%02x", pagel);
  _if_partout(intcmp, "0x%02x", bs2);
  _if_n_partout_str(memcmp, sizeof p->signature, dev_sprintf("0x%02x 0x%02x 0x%02x", p->signature[0], p->signature[1], p->signature[2]), signature);
  _if_partout(intcmp, "0x%04x", usbpid);

  if(!base || base->reset_disposition != p->reset_disposition)
    _partout_str(strdup(p->reset_disposition == RESET_DEDICATED? "dedicated": p->reset_disposition == RESET_IO? "io": "unknown"), reset);

  _if_partout_str(intcmp, strdup(p->retry_pulse == PIN_AVR_RESET? "reset": p->retry_pulse == PIN_AVR_SCK? "sck": "unknown"), retry_pulse);

  if(!base || base->flags != p->flags) {
    if(tsv) {
      _partout("0x%04x", flags);
    } else {
      _if_flagout(AVRPART_HAS_JTAG, has_jtag);
      _if_flagout(AVRPART_HAS_DW, has_debugwire);
      _if_flagout(AVRPART_HAS_PDI, has_pdi);
      _if_flagout(AVRPART_HAS_UPDI, has_updi);
      _if_flagout(AVRPART_HAS_TPI, has_tpi);
      _if_flagout(AVRPART_IS_AT90S1200, is_at90s1200);
      _if_flagout(AVRPART_AVR32, is_avr32);
      _if_flagout(AVRPART_ALLOWFULLPAGEBITSTREAM, allowfullpagebitstream);
      _if_flagout(AVRPART_ENABLEPAGEPROGRAMMING, enablepageprogramming);
      _if_flagout(AVRPART_SERIALOK, serial);

      if(!base || (base->flags & (AVRPART_PARALLELOK | AVRPART_PSEUDOPARALLEL)) != (p->flags & (AVRPART_PARALLELOK | AVRPART_PSEUDOPARALLEL))) {
        int par = p->flags & (AVRPART_PARALLELOK | AVRPART_PSEUDOPARALLEL);
        _partout_str(strdup(par == 0? "no": par == AVRPART_PSEUDOPARALLEL? "unknown": AVRPART_PARALLELOK? "yes": "pseudo"), parallel);
      }
    }
  }

  _if_partout(intcmp, "%d", timeout);
  _if_partout(intcmp, "%d", stabdelay);
  _if_partout(intcmp, "%d", cmdexedelay);
  _if_partout(intcmp, "%d", synchloops);
  _if_partout(intcmp, "%d", bytedelay);
  _if_partout(intcmp, "%d", pollindex);
  _if_partout(intcmp, "0x%02x", pollvalue);
  _if_partout(intcmp, "%d", predelay);
  _if_partout(intcmp, "%d", postdelay);
  _if_partout(intcmp, "%d", pollmethod);

  if(!base  && p->ctl_stack_type != CTL_STACK_NONE)
    dev_stack_out(tsv, p, dev_controlstack_name(p), p->controlstack, CTL_STACK_SIZE);

  // @@@ may need to remove controlstack and set p->ctl_stack_type to CTL_STACK_NONE if base has controlstack?
  if(base && (p->ctl_stack_type != base->ctl_stack_type || memcmp(base->controlstack, p->controlstack, sizeof base->controlstack)))
    dev_stack_out(tsv, p, dev_controlstack_name(p), p->controlstack, CTL_STACK_SIZE);

  if(!base || memcmp(base->flash_instr, p->flash_instr, sizeof base->flash_instr))
    dev_stack_out(tsv, p, "flash_instr", p->flash_instr, FLASH_INSTR_SIZE);

  if(!base || memcmp(base->eeprom_instr, p->eeprom_instr, sizeof base->eeprom_instr))
    dev_stack_out(tsv, p, "eeprom_instr", p->eeprom_instr, EEPROM_INSTR_SIZE);

  _if_partout(intcmp, "%d", hventerstabdelay);
  _if_partout(intcmp, "%d", progmodedelay);
  _if_partout(intcmp, "%d", latchcycles);
  _if_partout(intcmp, "%d", togglevtg);
  _if_partout(intcmp, "%d", poweroffdelay);
  _if_partout(intcmp, "%d", resetdelayms);
  _if_partout(intcmp, "%d", resetdelayus);
  _if_partout(intcmp, "%d", hvleavestabdelay);
  _if_partout(intcmp, "%d", resetdelay);
  _if_partout(intcmp, "%d", chiperasepulsewidth);
  _if_partout(intcmp, "%d", chiperasepolltimeout);
  _if_partout(intcmp, "%d", chiperasetime);
  _if_partout(intcmp, "%d", programfusepulsewidth);
  _if_partout(intcmp, "%d", programfusepolltimeout);
  _if_partout(intcmp, "%d", programlockpulsewidth);
  _if_partout(intcmp, "%d", programlockpolltimeout);
  _if_partout(intcmp, "%d", synchcycles);
  _if_partout(intcmp, "%d", hvspcmdexedelay);

  _if_partout(intcmp, "0x%02x", idr);
  _if_partout(intcmp, "0x%02x", rampz);
  _if_partout(intcmp, "0x%02x", spmcr);
  _if_partout(intcmp, "0x%02x", eecr);  // why is eecr an unsigned short?
  _if_partout(intcmp, "0x%04x", mcu_base);
  _if_partout(intcmp, "0x%04x", nvm_base);
  _if_partout(intcmp, "0x%04x", ocd_base);
  _if_partout(intcmp, "%d", ocdrev);

  for(int i=0; i < AVR_OP_MAX; i++)
    if(!base || opcodecmp(p->op[i], base->op[i], i))
      dev_part_strct_entry(tsv, ".ptop", p->desc, "part", opcodename(i), opcode2str(p->op[i], i, !tsv));

  for(int mi=0; mi < sizeof mem_order/sizeof *mem_order && mem_order[mi]; mi++) {
    AVRMEM *m, *bm;

    m = p->mem? avr_locate_mem(p, mem_order[mi]): NULL;
    bm = base && base->mem? avr_locate_mem(base, mem_order[mi]): NULL;

    if(!m && bm && !tsv)
      dev_info("\n    memory \"%s\" = NULL;\n", bm->desc);

    if(!m)
      continue;

    if(base && !bm)
      bm = avr_new_memtype();

    if(!tsv) {
      if(!memorycmp(bm, m))     // same memory bit for bit, no need to instantiate
        continue;

      dev_info("\n    memory \"%s\"\n", m->desc);
    }

    _if_memout_yn(paged);
    _if_memout(intcmp, m->size > 8192? "0x%x": "%d", size);
    _if_memout(intcmp, "%d", page_size);
    _if_memout(intcmp, "%d", num_pages); // why can AVRDUDE not compute this?
    _if_memout(intcmp, "0x%x", offset);
    _if_memout(intcmp, "%d", min_write_delay);
    _if_memout(intcmp, "%d", max_write_delay);
    _if_memout_yn(pwroff_after_write);
    _if_n_memout_str(memcmp, 2, dev_sprintf("0x%02x 0x%02x", m->readback[0], m->readback[1]), readback);
    _if_memout(intcmp, "%d", mode);
    _if_memout(intcmp, "%d", delay);
    _if_memout(intcmp, "%d", blocksize);
    _if_memout(intcmp, "%d", readsize);
    _if_memout(intcmp, "%d", pollindex);

    for(int i=0; i < AVR_OP_MAX; i++)
      if(!bm || opcodecmp(bm->op[i], m->op[i], i))
        dev_part_strct_entry(tsv, ".ptmmop", p->desc, m->desc, opcodename(i), opcode2str(m->op[i], i, !tsv));

    if(!tsv)
      dev_info("    ;\n");

    for(LNODEID lnm=lfirst(p->mem_alias); lnm; lnm=lnext(lnm)) {
      AVRMEM_ALIAS *ma = ldata(lnm);
      if(ma->aliased_mem && !strcmp(ma->aliased_mem->desc, m->desc)) {
        if(tsv)
          dev_info(".ptmm\t%s\t%s\talias\t%s\n", p->desc, ma->desc, m->desc);
        else
          dev_info("\n    memory \"%s\"\n        alias \"%s\";\n    ;\n", ma->desc, m->desc);
      }
    }
  }

  if(!tsv)
    dev_info(";\n");
}


// -p */[cdosw*]
void dev_output_part_defs(char *partdesc) {
  bool cmdok, waits, opspi, descs, strct, cmpst, raw, all, tsv;
  char *flags;
  int nprinted;
  AVRPART *nullpart = avr_new_part();

  if((flags = strchr(partdesc, '/')))
    *flags++ = 0;

  if(!flags && !strcmp(partdesc, "*")) // treat -p * as if it was -p */*
    flags = "*";

  if(!*flags || !strchr("cdosSrw*t", *flags)) {
    dev_info("%s: flags for developer option -p <wildcard>/<flags> not recognised\n", progname);
    dev_info(
      "Wildcard examples:\n"
      "         * all known parts\n"
      "  ATtiny10 just this part\n"
      "  *32[0-9] matches ATmega329, ATmega325 and ATmega328\n"
      "      *32? matches ATmega329, ATmega32A, ATmega325 and ATmega328\n"
      "Flags (one or more of the characters below):\n"
      "       c  check address bits in SPI commands and output errors\n"
      "       d  description of core part features\n"
      "       o  opcodes for SPI programming parts and memories\n"
      "       S  show entries of avrdude.conf parts with all values\n"
      "       s  show entries of avrdude.conf parts with necessary values\n"
      "       r  show entries of avrdude.conf parts as raw dump\n"
      "       w  wd_... constants for ISP parts\n"
      "       *  all of the above except s\n"
      "       t  use tab separated values as much as possible\n"
      "Note:\n"
      "  -p *      same as -p */*\n"
    );
    return;
  }

  // redirect stderr to stdout
  fflush(stderr); fflush(stdout); dup2(1, 2);

  all = *flags == '*';
  cmdok = all || !!strchr(flags, 'c');
  descs = all || !!strchr(flags, 'd');
  opspi = all || !!strchr(flags, 'o');
  waits = all || !!strchr(flags, 'w');
  strct = all || !!strchr(flags, 'S');
  raw   = all || !!strchr(flags, 'r');
  cmpst = !!strchr(flags, 's');
  tsv   = !!strchr(flags, 't');


  // go through all memories and add them to the memory order list
  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
    AVRPART *p = ldata(ln1);
    if(p->mem)
      for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm))
         add_mem_order(((AVRMEM *) ldata(lnm))->desc);

    // same for aliased memories (though probably not needed)
    if(p->mem_alias)
      for(LNODEID lnm=lfirst(p->mem_alias); lnm; lnm=lnext(lnm))
         add_mem_order(((AVRMEM_ALIAS *) ldata(lnm))->desc);
  }

  nprinted = dev_nprinted;
  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
    AVRPART *p = ldata(ln1);
    int flashsize, flashoffset, flashpagesize, eepromsize , eepromoffset, eeprompagesize;

    if(!descs || tsv)
      if(dev_nprinted > nprinted) {
        dev_info("\n");
        nprinted = dev_nprinted;
      }

    // pattern match the name of the part with command line: FMP_CASEFOLD not available here :(
    if(fnmatch(partdesc, p->desc, 0) && fnmatch(partdesc, p->id, 0))
      continue;

    if(strct || cmpst)
      dev_part_strct(p, tsv, cmpst? nullpart: NULL);

    if(raw)
      dev_part_raw(p);

    // identify core flash and eeprom parameters

    flashsize = flashoffset = flashpagesize = eepromsize = eepromoffset = eeprompagesize = 0;
    if(p->mem) {
      for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
        AVRMEM *m = ldata(lnm);
        if(!flashsize && m->desc && 0==strcmp(m->desc, "flash")) {
          flashsize = m->size;
          flashpagesize = m->page_size;
          flashoffset = m->offset;
        }
        if(!eepromsize && m->desc && 0==strcmp(m->desc, "eeprom")) {
          eepromsize = m->size;
          eepromoffset = m->offset;
          eeprompagesize = m->page_size;
        }
      }
    }

    // "real" entries don't seem to have a space in their desc (a bit hackey)
    if(flashsize && !index(p->desc, ' ')) {
      int ok, nfuses;
      AVRMEM *m;
      OPCODE *oc;

      ok = 2047;
      nfuses = 0;

      if(!p->op[AVR_OP_PGM_ENABLE])
        ok &= ~DEV_SPI_EN_CE_SIG;

      if(!p->op[AVR_OP_CHIP_ERASE])
        ok &= ~DEV_SPI_EN_CE_SIG;

      if((m = avr_locate_mem(p, "flash"))) {
        if((oc = m->op[AVR_OP_LOAD_EXT_ADDR])) {
          // @@@ to do: check whether address is put at lsb of third byte
        } else
         ok &= ~DEV_SPI_LOAD_EXT_ADDR;

        if((oc = m->op[AVR_OP_READ_HI])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_READ_HI, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_READ_LO])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_READ_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_WRITE_HI])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_WRITE_HI, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_WRITE_LO])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_WRITE_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_LOADPAGE_HI])) {
          if(cmdok)
            checkaddr(m->page_size>>1, 1, AVR_OP_LOADPAGE_HI, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM_PAGED;

        if((oc = m->op[AVR_OP_LOADPAGE_LO])) {
          if(cmdok)
            checkaddr(m->page_size>>1, 1, AVR_OP_LOADPAGE_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM_PAGED;

        if((oc = m->op[AVR_OP_WRITEPAGE])) {
          if(cmdok)
            checkaddr(m->size>>1, m->page_size>>1, AVR_OP_WRITEPAGE, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM_PAGED;
      } else
        ok &= ~(DEV_SPI_PROGMEM_PAGED | DEV_SPI_PROGMEM);

      if((m = avr_locate_mem(p, "eeprom"))) {
        if((oc = m->op[AVR_OP_READ])) {
          if(cmdok)
            checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM;

        if((oc = m->op[AVR_OP_WRITE])) {
          if(cmdok)
            checkaddr(m->size, 1, AVR_OP_WRITE, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM;

        if((oc = m->op[AVR_OP_LOADPAGE_LO])) {
          if(cmdok)
            checkaddr(m->page_size, 1, AVR_OP_LOADPAGE_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM_PAGED;

        if((oc = m->op[AVR_OP_WRITEPAGE])) {
          if(cmdok)
            checkaddr(m->size, m->page_size, AVR_OP_WRITEPAGE, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM_PAGED;
      } else
        ok &= ~(DEV_SPI_EEPROM_PAGED | DEV_SPI_EEPROM);

      if((m = avr_locate_mem(p, "signature")) && (oc = m->op[AVR_OP_READ])) {
        if(cmdok)
          checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
      } else
        ok &= ~DEV_SPI_EN_CE_SIG;

      if((m = avr_locate_mem(p, "calibration")) && (oc = m->op[AVR_OP_READ])) {
        if(cmdok)
          checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
      } else
        ok &= ~DEV_SPI_CALIBRATION;

      // actually, some AT90S... parts cannot read, only write lock bits :-0
      if( ! ((m = avr_locate_mem(p, "lock")) && m->op[AVR_OP_WRITE]))
        ok &= ~DEV_SPI_LOCK;

      if(((m = avr_locate_mem(p, "fuse")) || (m = avr_locate_mem(p, "lfuse"))) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_LFUSE;

      if((m = avr_locate_mem(p, "hfuse")) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_HFUSE;

      if((m = avr_locate_mem(p, "efuse")) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_EFUSE;

      if(descs) {
        int len = 16-strlen(p->desc);
        dev_info("%s '%s' =>%*s [0x%02X, 0x%02X, 0x%02X, 0x%08x, 0x%05x, 0x%03x, 0x%06x, 0x%04x, 0x%03x, %d, 0x%03x, 0x%04x, '%s'], # %s %d\n",
          tsv || all? ".desc": " ",
          p->desc, len > 0? len: 0, "",
          p->signature[0], p->signature[1], p->signature[2],
          flashoffset, flashsize, flashpagesize,
          eepromoffset, eepromsize, eeprompagesize,
          nfuses,
          ok,
          p->flags,
          parttype(p),
          p->config_file, p->lineno
        );
      }
    }

    if(opspi) {
      printallopcodes(p, "part", p->op);
      if(p->mem) {
        for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
          AVRMEM *m = ldata(lnm);
          if(m)
            printallopcodes(p, m->desc, m->op);
        }
      }
    }

    // print wait delays for AVR family parts
    if(waits) {
      if(!(p->flags & (AVRPART_HAS_PDI | AVRPART_HAS_UPDI | AVRPART_HAS_TPI | AVRPART_AVR32)))
        dev_info(".wd_chip_erase %.3f ms %s\n", p->chip_erase_delay/1000.0, p->desc);
      if(p->mem) {
        for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
          AVRMEM *m = ldata(lnm);
          // write delays not needed for read-only calibration and signature memories
          if(strcmp(m->desc, "calibration") && strcmp(m->desc, "signature")) {
            if(!(p->flags & (AVRPART_HAS_PDI | AVRPART_HAS_UPDI | AVRPART_HAS_TPI | AVRPART_AVR32))) {
              if(m->min_write_delay == m->max_write_delay)
                 dev_info(".wd_%s %.3f ms %s\n", m->desc, m->min_write_delay/1000.0, p->desc);
              else {
                 dev_info(".wd_min_%s %.3f ms %s\n", m->desc, m->min_write_delay/1000.0, p->desc);
                 dev_info(".wd_max_%s %.3f ms %s\n", m->desc, m->max_write_delay/1000.0, p->desc);
              }
            }
          }
        }
      }
    }
  }
}
