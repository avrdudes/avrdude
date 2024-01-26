/*
 * avrdude - A Downloader/Uploader for AVR device programmers
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
#include <stddef.h>
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

#include "avrdude.h"
#include "libavrdude.h"
#include "config.h"

#include "developer_opts.h"
#include "developer_opts_private.h"

// Inject part parameters into a semi-automated rewrite of avrdude.conf
//  - Add entries to the tables below; they get written on -p*/si or -c*/si
//  - Use the output in a new avrdude.conf
//  - Output again with -p*/s or -c*/s (no /i) and use that for final avrdude.conf
//  - Remove entries from below tables

static struct {
  const char *pgmid, *var, *value;
} pgminj[] = {
  // Add triples here, eg, {"stk500v2", "prog_modes", "PM_TPI|PM_ISP"},
  {NULL, NULL, NULL},
};

static struct {
  const char *mcu, *var, *value;
} ptinj[] = {
  // Add triples here, eg, {"ATmega328P", "mcuid", "999"},
  {NULL, NULL, NULL},
};

static struct {
  const char *mcu, *mem, *var, *value;
} meminj[] = {
  // Add quadruples here, eg, {"ATmega328P", "flash", "page_size", "128"},
  {NULL, NULL, NULL, NULL},
};


// Return 0 if op code would encode (essentially) the same SPI command
static int opcodecmp(const OPCODE *op1, const OPCODE *op2, int opnum) {
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

  // Don't care x and 0 are functionally equivalent
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


static void printopcode(const AVRPART *p, const char *d, const OPCODE *op, int opnum) {
  unsigned char cmd[4];
  int i;

  if(op) {
    memset(cmd, 0, sizeof cmd);
    avr_set_bits(op, cmd);

    dev_info(".op\t%s\t%s\t%s\t0x%02x%02x%02x%02x\t", p->desc, d, opcodename(opnum), cmd[0], cmd[1], cmd[2], cmd[3]);
    for(i=31; i >= 0; i--) {
      dev_info("%c", cmdbitchar(op->bit[i]));
      if(i%8 == 0)
        dev_info("%c", i? '\t': '\n');
    }
  }
}

static void printallopcodes(const AVRPART *p, const char *d, OPCODE * const *opa) {
  for(int i=0; i<AVR_OP_MAX; i++)
    printopcode(p, d, opa[i], i);
}



// Programming modes

static char *prog_modes_str(int pm) {
  static char type[1024];

  strcpy(type, "0");
  if(pm & PM_SPM)
    strcat(type, " | PM_SPM");
  if(pm & PM_TPI)
    strcat(type, " | PM_TPI");
  if(pm & PM_ISP)
    strcat(type, " | PM_ISP");
  if(pm & PM_PDI)
    strcat(type, " | PM_PDI");
  if(pm & PM_UPDI)
    strcat(type, " | PM_UPDI");
  if(pm & PM_HVSP)
    strcat(type, " | PM_HVSP");
  if(pm & PM_HVPP)
    strcat(type, " | PM_HVPP");
  if(pm & PM_debugWIRE)
    strcat(type, " | PM_debugWIRE");
  if(pm & PM_JTAG)
    strcat(type, " | PM_JTAG");
  if(pm & PM_JTAGmkI)
    strcat(type, " | PM_JTAGmkI");
  if(pm & PM_XMEGAJTAG)
    strcat(type, " | PM_XMEGAJTAG");
  if(pm & PM_AVR32JTAG)
    strcat(type, " | PM_AVR32JTAG");
  if(pm & PM_aWire)
    strcat(type, " | PM_aWire");

  return type + (type[1] == 0? 0: 4);
}

static char *extra_features_str(int m) {
  static char mode[1024];

  strcpy(mode, "0");
  if(m & HAS_SUFFER)
    strcat(mode, " | HAS_SUFFER");
  if(m & HAS_VTARG_SWITCH)
    strcat(mode, " | HAS_VTARG_SWITCH");
  if(m & HAS_VTARG_ADJ)
    strcat(mode, " | HAS_VTARG_ADJ");
  if(m & HAS_VTARG_READ)
    strcat(mode, " | HAS_VTARG_READ");
  if(m & HAS_FOSC_ADJ)
    strcat(mode, " | HAS_FOSC_ADJ");
  if(m & HAS_VAREF_ADJ)
    strcat(mode, " | HAS_VAREF_ADJ");

  return mode + (mode[1] == 0? 0: 4);
}

// Check whether address bits are where they should be in ISP commands
static void checkaddr(int memsize, int pagesize, int opnum, const OPCODE *op, const AVRPART *p, const AVRMEM *m) {
  int i, lo, hi;
  const char *opstr = opcodename(opnum);

  lo = intlog2(pagesize);
  hi = intlog2(memsize-1);

  // Address bits should be between positions lo and hi (and fall in line), outside should be 0 or don't care
  for(i=0; i<16; i++) {         // ISP programming only deals with 16-bit addresses (words for flash, bytes for eeprom)
    if(i < lo || i > hi) {
      if(op->bit[i+8].type != AVR_CMDBIT_IGNORE && !(op->bit[i+8].type == AVR_CMDBIT_VALUE && op->bit[i+8].value == 0)) {
        char *cbs = cmdbitstr(op->bit[i+8]);
        dev_info(".cmderr\t%s\t%s-%s\tbit %d outside addressable space should be x or 0 but is %s\n",
          p->desc, m->desc, opstr, i+8, cbs? cbs: "NULL");
        if(cbs)
          free(cbs);
      }
    } else {
      if(op->bit[i+8].type != AVR_CMDBIT_ADDRESS)
        dev_info(".cmderr\t%s\t%s-%s\tbit %d is %c but should be a\n", p->desc, m->desc, opstr, i+8, cmdbitchar(op->bit[i+8]));
      else if(op->bit[i+8].bitno != i)
        dev_info(".cmderr\t%s\t%s-%s\tbit %d inconsistent: a%d specified as a%d\n", p->desc, m->desc, opstr, i+8, i, op->bit[i+8].bitno);
    }
  }
  for(i=0; i<32; i++)           // Command bits 8..23 should not contain address bits
    if((i<8 || i>23) && op->bit[i].type == AVR_CMDBIT_ADDRESS)
      dev_info(".cmderr\t%s\t%s-%s\tbit %d contains a%d which it shouldn't\n", p->desc, m->desc, opstr, i, op->bit[i].bitno);
}



static char *dev_sprintf(const char *fmt, ...) {
  int size = 0;
  char *p = NULL;
  va_list ap;

  // Compute size
  va_start(ap, fmt);
  size = vsnprintf(p, size, fmt, ap);
  va_end(ap);

  if(size < 0)
    return cfg_strdup("dev_sprintf()", "");

  size++;                       // For terminating '\0'
  p = cfg_malloc("dev_sprintf()", size);

  va_start(ap, fmt);
  size = vsnprintf(p, size, fmt, ap);
  va_end(ap);

  if(size < 0)
    *p = 0;

  return p;
}


static int dev_nprinted;

int dev_message(int msglvl, const char *fmt, ...) {
  va_list ap;
  int rc = 0;

  if(verbose >= msglvl) {
    va_start(ap, fmt);
    rc = vfprintf(stdout, fmt, ap);
    va_end(ap);
    if(rc > 0)
      dev_nprinted += rc;
  }

  return rc;
}


// Any of the strings in the list contains subs as substring?
int dev_has_subsstr_comms(const LISTID comms, const char *subs) {
  if(comms)
    for(LNODEID ln=lfirst(comms); ln; ln=lnext(ln))
       if(str_contains((char *) ldata(ln), subs))
         return 1;
  return 0;
}

// Print a chained list of strings
void dev_print_comment(const LISTID comms) {
  if(comms)
    for(LNODEID ln=lfirst(comms); ln; ln=lnext(ln))
       dev_info("%s", (char *) ldata(ln));
}

// Conditional output of part, memory or programmer's comments field
static void dev_cout(const LISTID comms, const char *name, int rhs, int elself) {
  COMMENT *cp;

  if(elself == 2)
    dev_info("\n");
  if((cp = locate_comment(comms, name, rhs)))
    dev_print_comment(cp->comms);
  else if(elself == 1)
    dev_info("\n");
}

// Print part->comments, mem->comments or pgm->comments (for debugging)
void dev_print_kw_comments(const LISTID comms) {
  if(comms)
    for(LNODEID ln=lfirst(comms); ln; ln=lnext(ln)) {
      COMMENT *n = ldata(ln);
      if(n && n->comms) {
        dev_info(">>> %s %c\n", n->kw, n->rhs? '>': '<');
        dev_print_comment(n->comms);
      }
    }
}

// Ideally all assignment outputs run via this function
static int dev_part_strct_entry(bool tsv,               // Print as spreadsheet?
  const char *col0, const char *col1, const char *col2, // Descriptors of item
  const char *name, char *cont, const LISTID comms) {   // Name, contents and comments

  const char *n = name? name: "name_error";
  const char *c = cont? cont: "cont_error";

  if(tsv) {                     // Tab separated values
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
  } else {                      // Grammar conform
    int indent = col2 && !str_eq(col2, "part");
    dev_cout(comms, n, 0, 0);   // Print comments before the line
    dev_info("%*s%-*s = %s;", indent? 8: 4, "", indent? 18: 22, n, c);
    dev_cout(comms, n, 1, 1);   // Print comments on rhs
  }

  if(cont)
    free(cont);

  return 1;
}


static const char *dev_controlstack_name(const AVRPART *p) {
  return
    p->ctl_stack_type == CTL_STACK_PP? "pp_controlstack":
    p->ctl_stack_type == CTL_STACK_HVSP? "hvsp_controlstack":
    p->ctl_stack_type == CTL_STACK_NONE? "NULL":
    "unknown_controlstack";
}


static void dev_stack_out(bool tsv, const AVRPART *p, const char *name, const unsigned char *stack, int ns) {
  if(str_eq(name, "NULL")) {
    name = "pp_controlstack";
    ns = 0;
  }

  if(tsv)
    dev_info(".pt\t%s\t%s\t", p->desc, name);
  else {
    dev_cout(p->comments, name, 0, 0);
    dev_info("    %-22s =%s", name, ns <=8? " ": "");
  }

  if(ns <= 0)
    dev_info(tsv? "NULL\n": "NULL;");
  else
    for(int i=0; i<ns; i++)
      dev_info("%s0x%02x%s", !tsv && ns > 8 && i%8 == 0? "\n        ": " ", stack[i], i+1<ns? ",": tsv? "\n": ";");

  dev_cout(p->comments, name, 1, 1);
}


static int intcmp(int a, int b) {
  return a-b;
}

static int boolcmp(int a, int b) {
  return !!a-!!b;
}


// Deep copies for comparison and raw output

typedef struct {
  char descbuf[32];
  AVRMEM base;
  OPCODE ops[AVR_OP_MAX];
} AVRMEMdeep;

static int avrmem_deep_copy(AVRMEMdeep *d, const AVRMEM *m) {
  d->base = *m;

  // Note memory desc (name, really) is limited to 31 char here
  memset(d->descbuf, 0, sizeof d->descbuf);
  strncpy(d->descbuf, m->desc, sizeof d->descbuf-1);

  // Zap address values
  d->base.comments = NULL;
  d->base.buf = NULL;
  d->base.tags = NULL;
  d->base.desc = NULL;
  for(int i=0; i<AVR_OP_MAX; i++)
    d->base.op[i] = NULL;

  // Copy over the SPI operations themselves
  memset(d->ops, 0, sizeof d->ops);
  for(size_t i=0; i<AVR_OP_MAX; i++)
    if(m->op[i])
      d->ops[i] = *m->op[i];

  return 0;
}

static int memorycmp(const AVRMEM *m1, const AVRMEM *m2) {
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
  char descbuf[64];
  char idbuf[32];
  char family_idbuf[16];
  char variants[4096-16-32-64];
  AVRPART base;
  OPCODE ops[AVR_OP_MAX];
  AVRMEMdeep mems[40];
} AVRPARTdeep;


// Return memory iff its desc matches str exactly
static AVRMEM *dev_locate_mem(const AVRPART *p, const char *str) {
  AVRMEM *m = p->mem? avr_locate_mem_noalias(p, str): NULL;
  return m && str_eq(m->desc, str)? m: NULL;
}

static int avrpart_deep_copy(AVRPARTdeep *d, const AVRPART *p) {
  AVRMEM *m;
  size_t di;

  memset(d, 0, sizeof *d);

  d->base = *p;

  d->base.comments = NULL;
  d->base.parent_id = NULL;
  d->base.config_file = NULL;
  d->base.lineno = 0;

  // Copy over desc, id, and family_id
  strncpy(d->descbuf, p->desc, sizeof d->descbuf-1);
  strncpy(d->idbuf, p->id, sizeof d->idbuf-1);
  strncpy(d->family_idbuf, p->family_id, sizeof d->family_idbuf-1);
  char *vp = d->variants;
  for(LNODEID ln=lfirst(p->variants); ln; ln=lnext(ln)) {
    if(vp < d->variants+sizeof d->variants-2) {
      strncpy(vp, (char *)ldata(ln), d->variants+sizeof d->variants-vp-1);
      vp += strlen(vp)+1;
    }
  }

  // Zap address values
  d->base.desc = NULL;
  d->base.id = NULL;
  d->base.family_id = NULL;
  d->base.mem = NULL;
  d->base.mem_alias = NULL;
  d->base.variants = NULL;
  for(int i=0; i<AVR_OP_MAX; i++)
    d->base.op[i] = NULL;

  // Copy over all used SPI operations
  for(int i=0; i<AVR_OP_MAX; i++)
    if(p->op[i])
      d->ops[i] = *p->op[i];

  // Fill in all memories we got in defined order
  di = 0;
  for(size_t mi=0; mi < sizeof avr_mem_order/sizeof *avr_mem_order && avr_mem_order[mi].str; mi++) {
    m = dev_locate_mem(p, avr_mem_order[mi].str);
    if(m) {
      if(di >= sizeof d->mems/sizeof *d->mems) {
        pmsg_error("ran out of mems[] space, increase size in AVRMEMdeep of developer_opts.c and recompile\n");
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
  return in == 0? '.': in > ' ' && in < 0x7f? in: '_';
}

static void dev_raw_dump(const void *v, int nbytes, const char *name, const char *sub, int idx) {
  const unsigned char *p = v;
  int n = (nbytes + 31)/32;

  for(int i=0; i<n; i++, p += 32, nbytes -= 32) {
    dev_info("%s\t%s\t%02x.%03x0: ", name, sub, idx, 2*i);
    for(int j=0; j<32; j++) {
      if(j && j%8 == 0)
        dev_info(" ");
      if(j < nbytes)
        dev_info("%02x", p[j]);
      else
        dev_info("  ");
    }
    dev_info(" ");
    for(int j=0; j<32 && j < nbytes; j++)
      dev_info("%c", txtchar(p[j]));
    dev_info("\n");
  }
}

static int _is_all_zero(const void *p, size_t n) {
  const char *q = (const char *) p;
  return n <= 0 || (*q == 0 && memcmp(q, q+1, n-1) == 0);
}

static char *opsnm(const char *pre, int opnum) {
  static char ret[128];
  sprintf(ret, "%.31s.%.95s", pre, opcodename(opnum));
  return ret;
}

static void dev_part_raw(const AVRPART *part) {
  AVRPARTdeep dp;
  int di = avrpart_deep_copy(&dp, part);

  dev_raw_dump(&dp, (char *)&dp.base-(char *)&dp, part->desc, "part.intro", 0);
  dev_raw_dump(&dp.base, sizeof dp.base, part->desc, "part", 0);
  for(int i=0; i<AVR_OP_MAX; i++)
    if(!_is_all_zero(dp.ops+i, sizeof*dp.ops))
      dev_raw_dump(dp.ops+i, sizeof*dp.ops, part->desc, opsnm("part", i), 1);

  for(int i=0; i<di; i++) {
    char *nm = dp.mems[i].descbuf;

    dev_raw_dump(nm, sizeof dp.mems[i].descbuf, part->desc, nm, i+2);
    dev_raw_dump(&dp.mems[i].base, sizeof dp.mems[i].base, part->desc, nm, i+2);
    for(int j=0; j<AVR_OP_MAX; j++)
      if(!_is_all_zero(dp.mems[i].ops+j, sizeof(OPCODE)))
        dev_raw_dump(dp.mems[i].ops+j, sizeof(OPCODE), part->desc, opsnm(nm, j), i+2);
  }
}


static void dev_part_strct(const AVRPART *p, bool tsv, const AVRPART *base, bool injct) {
  char *descstr = cfg_escape(p->desc);
  COMMENT *cp;

  if(!tsv) {
    const char *del = "#------------------------------------------------------------";
    cp = locate_comment(p->comments, "*", 0);

    if(!cp || !dev_has_subsstr_comms(cp->comms, del)) {
      dev_info("%s\n", del);
      dev_info("# %.*s\n", strlen(descstr)-2, descstr+1); // Remove double quotes
      dev_info("%s\n\n", del);
    }
    if(cp)
      dev_print_comment(cp->comms);

    if(p->parent_id && *p->parent_id)
      dev_info("part parent \"%s\" # %s\n", p->parent_id, p->id);
    else
      dev_info("part # %s\n", p->id);
  }

  _if_partout_str(strcmp, descstr, desc);
  _if_partout_str(strcmp, cfg_escape(p->id), id);

  if(lsize(p->variants)) {      // Variants are never inherited, so print if they exist
    int firstid = 1;
    if(tsv)
      dev_info(".pt\t%s\tvariants\t", p->desc);
    else {
      dev_cout(p->comments, "variants", 0, 0);
      dev_info("    %-22s =\n", "variants");
    }
    for(LNODEID ln=lfirst(p->variants); ln; ln=lnext(ln)) {
      if(!firstid)
        dev_info(tsv? ", ": ",\n");
      firstid = 0;
      char *str = cfg_escape(ldata(ln));
      dev_info("%*s%s", tsv? 0: 8, "", str);
      free(str);
    }
    if(tsv)
      dev_info("\n");
    else {
      dev_info(";");
      dev_cout(p->comments, "variants", 1, 1);
    }
  } else if(!base) {            // Print NULL for /S option
    if(tsv)
      dev_info(".pt\t%s\tvariants\tNULL\n", p->desc);
    else {
      dev_cout(p->comments, "variants", 0, 0);
      dev_info("    %-22s = NULL;\n", "variants");
      dev_cout(p->comments, "variants", 1, 1);
    }
  }

  _if_partout_str(strcmp, cfg_escape(p->family_id), family_id);
  _if_partout_str(intcmp, cfg_strdup("dev_part_strct()", prog_modes_str(p->prog_modes)), prog_modes);
  if(p->mcuid == 21) {
    _if_partout_str(intcmp, cfg_strdup(__func__, "XVII + IV"), mcuid);
  } else {
    _if_partout(intcmp, "%d", mcuid);
  }
  _if_partout(intcmp, "%d", n_interrupts);
  _if_partout(intcmp, "%d", n_page_erase);
  _if_partout(intcmp, "%d", n_boot_sections);
  _if_partout(intcmp, "%d", boot_section_size);
  _if_partout(intcmp, "%d", hvupdi_variant);
  _if_partout(intcmp, "0x%02x", stk500_devcode);
  _if_partout(intcmp, "0x%02x", avr910_devcode);
  _if_partout(intcmp, "%d", chip_erase_delay);
  _if_partout(intcmp, "0x%02x", pagel);
  _if_partout(intcmp, "0x%02x", bs2);
  _if_n_partout_str(memcmp, sizeof p->signature, dev_sprintf("0x%02x 0x%02x 0x%02x", p->signature[0], p->signature[1], p->signature[2]), signature);
  _if_partout(intcmp, "0x%04x", usbpid);

  if(!base || base->reset_disposition != p->reset_disposition)
    _partout_str(cfg_strdup("dev_part_strct()",
       p->reset_disposition == RESET_DEDICATED? "dedicated": p->reset_disposition == RESET_IO? "io": "unknown"),
       reset);

  _if_partout_str(intcmp, cfg_strdup("dev_part_strct()",
     p->retry_pulse == PIN_AVR_RESET? "reset": p->retry_pulse == PIN_AVR_SCK? "sck": "unknown"),
     retry_pulse);

  if(!base || base->flags != p->flags) {
    if(tsv) {
      _partout("0x%04x", flags);
    } else {
      _if_flagout(AVRPART_IS_AT90S1200, is_at90s1200);
      _if_flagout(AVRPART_ALLOWFULLPAGEBITSTREAM, allowfullpagebitstream);
      _if_flagout(AVRPART_ENABLEPAGEPROGRAMMING, enablepageprogramming);
      _if_flagout(AVRPART_SERIALOK, serial);

      if(!base || (base->flags & (AVRPART_PARALLELOK | AVRPART_PSEUDOPARALLEL)) != (p->flags & (AVRPART_PARALLELOK | AVRPART_PSEUDOPARALLEL))) {
        int par = p->flags & (AVRPART_PARALLELOK | AVRPART_PSEUDOPARALLEL);
        _partout_str(cfg_strdup("dev_part_strct()",
          par == 0? "no": par == AVRPART_PSEUDOPARALLEL? "unknown": AVRPART_PARALLELOK? "yes": "pseudo"),
          parallel);
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
  _if_partout(intcmp, "0x%02x", eecr);
  _if_partout(intcmp, "0x%02x", eind);
  _if_partout(intcmp, "0x%04x", mcu_base);
  _if_partout(intcmp, "0x%04x", nvm_base);
  _if_partout(intcmp, "0x%04x", ocd_base);
  _if_partout(intcmp, "0x%04x", syscfg_base);
  _if_partout(intcmp, "%d", ocdrev);
  _if_partout(intcmp, "0x%02x", autobaud_sync);
  _if_partout(intcmp, "%d", factory_fcpu);

  for(int i=0; i < AVR_OP_MAX; i++)
    if(!base || opcodecmp(p->op[i], base->op[i], i))
      dev_part_strct_entry(tsv, ".ptop", p->desc, "part", opcodename(i), opcode2str(p->op[i], i, !tsv), p->comments);

  for(size_t mi=0; mi < sizeof avr_mem_order/sizeof *avr_mem_order && avr_mem_order[mi].str; mi++) {
    AVRMEM *m, *bm;

    m = dev_locate_mem(p, avr_mem_order[mi].str);
    bm = base? dev_locate_mem(base, avr_mem_order[mi].str): NULL;

    if(!m && bm && !tsv)
      dev_info("\n    memory \"%s\" %*s= NULL;\n", bm->desc, 13 > strlen(bm->desc)? 13-strlen(bm->desc): 0, "");

    if(!m)
      continue;

    if(base && !bm)
      bm = avr_new_mem();

    if(!tsv) {
      if(!memorycmp(bm, m)) {   // Same memory bit for bit, only instantiate on injected parameters
        int haveinjct = 0;
        if(injct)
          for(size_t i=0; i<sizeof meminj/sizeof*meminj; i++)
            if(meminj[i].mcu && str_casematch(meminj[i].mcu, p->desc) && str_match(meminj[i].mem, m->desc))
              haveinjct = 1;
        if(!haveinjct)
          continue;
      }

      dev_cout(m->comments, "*", 0, 2);
      dev_info("    memory \"%s\"\n", m->desc);
    }

    _if_memout_yn(paged);
    _if_memout(intcmp, m->size > 8192? "0x%x": "%d", size);
    _if_memout(intcmp, "%d", page_size);
    _if_memout(intcmp, "%d", num_pages);
    _if_memout(intcmp, m->initval == -1? "%d": "0x%02x", initval);
    _if_memout(intcmp, m->bitmask == -1? "%d": "0x%02x", bitmask);
    _if_memout(intcmp, "%d", n_word_writes);
    _if_memout(intcmp, "0x%x", offset);
    _if_memout(intcmp, "%d", min_write_delay);
    _if_memout(intcmp, "%d", max_write_delay);
    _if_memout_yn(pwroff_after_write);
    _if_n_memout_str(memcmp, 2, dev_sprintf("0x%02x 0x%02x", m->readback[0], m->readback[1]), readback);
    _if_memout(intcmp, "0x%02x", mode);
    _if_memout(intcmp, "%d", delay);
    _if_memout(intcmp, "%d", blocksize);
    _if_memout(intcmp, "%d", readsize);
    _if_memout(intcmp, "%d", pollindex);

    for(int i=0; i < AVR_OP_MAX; i++)
      if(!bm || opcodecmp(bm->op[i], m->op[i], i))
        dev_part_strct_entry(tsv, ".ptmmop", p->desc, m->desc, opcodename(i), opcode2str(m->op[i], i, !tsv), m->comments);

    if(injct)
      for(size_t i=0; i<sizeof meminj/sizeof*meminj; i++)
        if(meminj[i].mcu && str_casematch(meminj[i].mcu, p->desc))
          if(str_match(meminj[i].mem, m->desc)) {
            dev_part_strct_entry(tsv, ".ptmm", p->desc, m->desc,
              meminj[i].var, cfg_strdup("meminj", meminj[i].value), NULL);
            meminj[i].mcu = NULL;
          }

    if(!tsv) {
      dev_cout(m->comments, ";", 0, 0);
      dev_info("    ;\n");
    }

    for(LNODEID lnm=lfirst(p->mem_alias); lnm; lnm=lnext(lnm)) {
      AVRMEM_ALIAS *ma = ldata(lnm);
      if(ma->aliased_mem && str_eq(ma->aliased_mem->desc, m->desc)) {
        // There is a memory that's aliased to the current memory: is it inherited?
        if(base) {
          int basehasalias = 0;
          for(LNODEID lnb=lfirst(base->mem_alias); lnb; lnb=lnext(lnb)) {
            AVRMEM_ALIAS *mab = ldata(lnb);
            if(str_eq(mab->desc, ma->desc) && mab->aliased_mem && str_eq(mab->aliased_mem->desc, m->desc))
              basehasalias = 1;
          }
          if(basehasalias)
            continue;
        }
        if(tsv)
          dev_info(".ptmm\t%s\t%s\talias\t%s\n", p->desc, ma->desc, m->desc);
        else
          dev_info("\n    memory \"%s\"\n        alias \"%s\";\n    ;\n", ma->desc, m->desc);
      }
    }
  }

  if(injct) {
    for(size_t i=0; i<sizeof ptinj/sizeof*ptinj; i++)
      if(ptinj[i].mcu)
        if(str_casematch(ptinj[i].mcu, p->desc))
          dev_part_strct_entry(tsv, ".pt", p->desc, NULL,
            ptinj[i].var, cfg_strdup("ptinj", ptinj[i].value), NULL);

    for(size_t i=0; i<sizeof meminj/sizeof*meminj; i++)
      if(meminj[i].mcu && str_casematch(meminj[i].mcu, p->desc)) {
        if(!tsv)
          dev_info("    memory \"%s\"\n", meminj[i].mem);
        dev_part_strct_entry(tsv, ".ptmm", p->desc, meminj[i].mem,
          meminj[i].var, cfg_strdup("meminj", meminj[i].value), NULL);
        meminj[i].mcu = NULL;
        if(!tsv)
          dev_info("    ;\n");
      }
  }

  if(!tsv) {
    dev_cout(p->comments, ";", 0, 0);
    dev_info(";\n");
  }
}


void dev_output_pgm_part(int dev_opt_c, const char *programmer, int dev_opt_p, const char *partdesc) {
  if(dev_opt_c == 2 && dev_opt_p == 2) {
    char *p;

    dev_print_comment(cfg_get_prologue());
    dev_info("avrdude_conf_version = %s;\n\n", p = cfg_escape(avrdude_conf_version)); free(p);
    dev_info("default_programmer = %s;\n", p = cfg_escape(default_programmer)); free(p);
    dev_info("default_parallel   = %s;\n", p = cfg_escape(default_parallel)); free(p);
    dev_info("default_serial     = %s;\n", p = cfg_escape(default_serial)); free(p);
    dev_info("default_spi        = %s;\n", p = cfg_escape(default_spi)); free(p);
    dev_info("default_baudrate   = %d;\n", default_baudrate);
    dev_info("default_bitclock   = %7.5f;\n", default_bitclock);
    dev_info("default_linuxgpio  = %s;\n", p = cfg_escape(default_linuxgpio)); free(p);
    dev_info("allow_subshells    = %s;\n", allow_subshells? "yes": "no");

    dev_info("\n#\n# PROGRAMMER DEFINITIONS\n#\n\n");
  }

  if(dev_opt_c)
    dev_output_pgm_defs(cfg_strdup("main()", programmer));

  if(dev_opt_p == 2 && dev_opt_c)
    dev_info("\n");
  if(dev_opt_p == 2)
    dev_info("#\n# PART DEFINITIONS\n#\n");

  if(dev_opt_p)
    dev_output_part_defs(cfg_strdup("main()", partdesc));
}


// -p */[dASsrcow*ti]
void dev_output_part_defs(char *partdesc) {
  bool cmdok, waits, opspi, descs, astrc, strct, cmpst, injct, raw, all, tsv;
  char *flags;
  int nprinted;
  AVRPART *nullpart = avr_new_part();

  if((flags = strchr(partdesc, '/')))
    *flags++ = 0;

  if(!flags && str_eq(partdesc, "*")) // Treat -p * as if it was -p */s
    flags = "s";

  if(!*flags || !strchr("cdoASsrw*ti", *flags)) {
    dev_info("%s: flags for developer option -p <wildcard>/<flags> not recognised\n", progname);
    dev_info(
      "Wildcard examples (these need protecting in the shell through quoting):\n"
      "         * all known parts\n"
      "  ATtiny10 just this part\n"
      "  *32[0-9] matches ATmega329, ATmega325 and ATmega328\n"
      "      *32? matches ATmega329, ATmega32A, ATmega325 and ATmega328\n"
      "Flags (one or more of the characters below):\n"
      "       d  description of core part features\n"
      "       A  show entries of avrdude.conf parts with all values\n"
      "       S  show entries of avrdude.conf parts with necessary values\n"
      "       s  show short entries of avrdude.conf parts using parent\n"
      "       r  show entries of avrdude.conf parts as raw dump\n"
      "       c  check and report errors in address bits of SPI commands\n"
      "       o  opcodes for SPI programming parts and memories\n"
      "       w  wd_... constants for ISP parts\n"
      "       *  all of the above except s and S\n"
      "       t  use tab separated values as much as possible\n"
      "       i  inject assignments from source code table\n"
      "Examples:\n"
      "  $ avrdude -p ATmega328P/s\n"
      "  $ avrdude -p m328*/st | grep chip_erase_delay\n"
      "  avrdude -p*/r | sort\n"
      "Notes:\n"
      "  -p * is the same as -p */s\n"
      "  This help message is printed using any unrecognised flag, eg, -p/h\n"
      "  Leaving no space after -p can be an OK substitute for quoting in shells\n"
      "  /s, /S and /A outputs are designed to be used as input in avrdude.conf\n"
      "  Sorted /r output should stay invariant when rearranging avrdude.conf\n"
      "  The /c, /o and /w flags are less generic and may be removed sometime\n"
      "  These options are just to help development, so not further documented\n"
    );
    return;
  }

  all = *flags == '*';
  cmdok = all || !!strchr(flags, 'c');
  descs = all || !!strchr(flags, 'd');
  opspi = all || !!strchr(flags, 'o');
  waits = all || !!strchr(flags, 'w');
  astrc = all || !!strchr(flags, 'A');
  raw   = all || !!strchr(flags, 'r');
  strct = !!strchr(flags, 'S');
  cmpst = !!strchr(flags, 's');
  tsv   = !!strchr(flags, 't');
  injct = !!strchr(flags, 'i');


  // Go through all memories and add them to the memory order list
  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
    AVRPART *p = ldata(ln1);
    if(p->mem)
      for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm))
        avr_get_mem_type(((AVRMEM *) ldata(lnm))->desc);

    // Same for aliased memories (though probably not needed)
    if(p->mem_alias)
      for(LNODEID lnm=lfirst(p->mem_alias); lnm; lnm=lnext(lnm))
        avr_get_mem_type(((AVRMEM_ALIAS *) ldata(lnm))->desc);
  }

  if((nprinted = dev_nprinted)) {
    dev_info("\n");
    nprinted = dev_nprinted;
  }
  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
    AVRPART *p = ldata(ln1);
    int flashsize, flashoffset, flashpagesize, eepromsize , eepromoffset, eeprompagesize;

    if(!descs || tsv)
      if(dev_nprinted > nprinted) {
        dev_info("\n");
        nprinted = dev_nprinted;
      }

    if(!part_eq(p, partdesc, str_casematch))
      continue;

    if(astrc || strct || cmpst)
      dev_part_strct(p, tsv,
        astrc? NULL:
        strct? nullpart:
        p->parent_id && *p->parent_id? locate_part(part_list, p->parent_id): nullpart,
        injct);

    if(raw)
      dev_part_raw(p);

    // Identify core flash and eeprom parameters

    flashsize = flashoffset = flashpagesize = eepromsize = eepromoffset = eeprompagesize = 0;
    if(p->mem) {
      for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
        AVRMEM *m = ldata(lnm);
        if(!flashsize && mem_is_flash(m)) {
          flashsize = m->size;
          flashpagesize = m->page_size;
          flashoffset = m->offset;
        }
        if(!eepromsize && mem_is_eeprom(m)) {
          eepromsize = m->size;
          eepromoffset = m->offset;
          eeprompagesize = m->page_size;
        }
      }
    }

    // "Real" entries don't seem to have a space in their desc (a bit hackey)
    if(flashsize && !strchr(p->desc, ' ')) {
      int ok, nfuses;
      AVRMEM *m;
      OPCODE *oc;

      ok = 2047;
      nfuses = 0;

      if(!p->op[AVR_OP_PGM_ENABLE])
        ok &= ~DEV_SPI_EN_CE_SIG;

      if(!p->op[AVR_OP_CHIP_ERASE])
        ok &= ~DEV_SPI_EN_CE_SIG;

      if((m = avr_locate_flash(p))) {
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

      if((m = avr_locate_eeprom(p))) {
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

      if((m = avr_locate_signature(p)) && (oc = m->op[AVR_OP_READ])) {
        if(cmdok)
          checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
      } else
        ok &= ~DEV_SPI_EN_CE_SIG;

      if((m = avr_locate_calibration(p)) && (oc = m->op[AVR_OP_READ])) {
        if(cmdok)
          checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
      } else
        ok &= ~DEV_SPI_CALIBRATION;

      // Actually, some AT90S... parts cannot read, only write lock bits :-0
      if( !((m = avr_locate_lock(p)) && m->op[AVR_OP_WRITE]))
        ok &= ~DEV_SPI_LOCK;

      if((m = avr_locate_fuse(p)) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_LFUSE;

      if((m = avr_locate_hfuse(p)) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_HFUSE;

      if((m = avr_locate_efuse(p)) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_EFUSE;

      if(descs) {
        int len = 16-strlen(p->desc);
        dev_info("%s '%s' =>%*s [0x%02X, 0x%02X, 0x%02X, 0x%08x, 0x%05x, 0x%03x, 0x%06x, 0x%04x, 0x%03x, %d, 0x%03x, 0x%04x, '%s'], # %s %d\n",
          tsv || all? ".desc": "   ",
          p->desc, len > 0? len: 0, "",
          p->signature[0], p->signature[1], p->signature[2],
          flashoffset, flashsize, flashpagesize,
          eepromoffset, eepromsize, eeprompagesize,
          nfuses,
          ok,
          p->flags,
          prog_modes_str(p->prog_modes),
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

    // Print wait delays for AVR family parts
    if(waits) {
      if(p->prog_modes & PM_ISP)
        dev_info(".wd_chip_erase %.3f ms %s\n", p->chip_erase_delay/1000.0, p->desc);
      if(p->mem) {
        for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
          AVRMEM *m = ldata(lnm);
          // Write delays not needed for read-only calibration and signature memories
          if(!mem_is_readonly(m)) {
            if(p->prog_modes & PM_ISP) {
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


static void dev_pgm_raw(const PROGRAMMER *pgm) {
  PROGRAMMER dp;
  int len, idx;
  char *id = ldata(lfirst(pgm->id));
  LNODEID ln;

  memcpy(&dp, pgm, sizeof dp);

  // Dump id, usbpid and hvupdi_support lists
  for(idx=0, ln=lfirst(dp.id); ln; ln=lnext(ln))
    dev_raw_dump(ldata(ln), strlen(ldata(ln))+1, id, "id", idx++);
  for(idx=0, ln=lfirst(dp.usbpid); ln; ln=lnext(ln))
    dev_raw_dump(ldata(ln), sizeof(int), id, "usbpid", idx++);
  for(idx=0, ln=lfirst(dp.hvupdi_support); ln; ln=lnext(ln))
    dev_raw_dump(ldata(ln), sizeof(int), id, "hvupdi_", idx++);

  if(dp.desc)
    dev_raw_dump(dp.desc, strlen(dp.desc)+1, id, "desc", 0);
  // Dump cache_string values
  if(dp.usbdev && *dp.usbdev)
    dev_raw_dump(dp.usbdev, strlen(dp.usbdev)+1, id, "usbdev", 0);
  if(dp.usbsn && *dp.usbsn)
    dev_raw_dump(dp.usbsn, strlen(dp.usbsn)+1, id, "usbsn", 0);
  if(dp.usbvendor && *dp.usbvendor)
    dev_raw_dump(dp.usbvendor, strlen(dp.usbvendor)+1, id, "usbvend", 0);
  if(dp.usbproduct && *dp.usbproduct)
    dev_raw_dump(dp.usbproduct, strlen(dp.usbproduct)+1, id, "usbprod", 0);

  // Zap all bytes beyond terminating nul of desc, type and port array
  if((len = (int) strlen(dp.type)+1) < (int) sizeof dp.type)
    memset(dp.type + len, 0, sizeof dp.type - len);
  if((len = (int) strlen(dp.port)+1) < (int) sizeof dp.port)
    memset(dp.port + len, 0, sizeof dp.port - len);

  // Zap address values
  dp.desc = NULL;
  dp.id = NULL;
  dp.comments = NULL;
  dp.parent_id = NULL;
  dp.initpgm = NULL;
  dp.usbpid = NULL;
  dp.usbdev = NULL;
  dp.usbsn = NULL;
  dp.usbvendor = NULL;
  dp.usbproduct = NULL;
  dp.hvupdi_support = NULL;

  // Only dump contents of PROGRAMMER struct up to and excluding the fd component
  dev_raw_dump((char *) &dp, offsetof(PROGRAMMER, fd), id, "pgm", 0);
}


static const char *connstr(conntype_t conntype) {
  switch(conntype) {
  case CONNTYPE_LINUXGPIO: return "linuxgpio";
  case CONNTYPE_PARALLEL: return "parallel";
  case CONNTYPE_SERIAL: return "serial";
  case CONNTYPE_USB: return "usb";
  case CONNTYPE_SPI: return "spi";
  default: return "<unknown>";
  }
}


static char *dev_usbpid_liststr(const PROGRAMMER *pgm) {
  char spc[1024];  int firstid = 1;

  spc[0] = 0;
  if(pgm->usbpid)
    for(LNODEID ln=lfirst(pgm->usbpid); ln; ln=lnext(ln)) {
      if(strlen(spc) > sizeof spc - 20)
        break;
      if(!firstid)
        strcat(spc, ", ");
      firstid = 0;
      sprintf(spc + strlen(spc), "0x%04x", *(unsigned int *) ldata(ln));
    }

  return cfg_strdup(__func__, *spc? spc: "NULL");
}

static char *dev_hvupdi_support_liststr(const PROGRAMMER *pgm) {
  char spc[1024];  int firstid = 1;

  spc[0] = 0;
  if(pgm->hvupdi_support)
    for(LNODEID ln=lfirst(pgm->hvupdi_support); ln; ln=lnext(ln)) {
      if(strlen(spc) > sizeof spc - 20)
        break;
      if(!firstid)
        strcat(spc, ", ");
      firstid = 0;
      sprintf(spc + strlen(spc), "%d", *(unsigned int *) ldata(ln));
    }

  return cfg_strdup(__func__, *spc? spc: "NULL");
}


static void dev_pgm_strct(const PROGRAMMER *pgm, bool tsv, const PROGRAMMER *base, bool injct) {
  char *id = ldata(lfirst(pgm->id));
  LNODEID ln;
  COMMENT *cp;
  int firstid;

  if(!tsv) {
    const char *del = "#------------------------------------------------------------";
    cp = locate_comment(pgm->comments, "*", 0);

    if(!cp || !dev_has_subsstr_comms(cp->comms, del)) {
      dev_info("%s\n# ", del);
      for(firstid=1, ln=lfirst(pgm->id); ln; ln=lnext(ln)) {
        if(!firstid)
          dev_info("/");
        firstid = 0;
        dev_info("%s", ldata(ln));
      }
      dev_info("\n%s\n\n", del);
    }
    if(cp)
      dev_print_comment(cp->comms);

    const char *prog_sea = is_programmer(pgm)? "programmer": is_serialadapter(pgm)? "serialadapter": "programmer";
    if(pgm->parent_id && *pgm->parent_id)
      dev_info("%s parent \"%s\" # %s\n", prog_sea, pgm->parent_id, ldata(lfirst(pgm->id)));
    else
      dev_info("%s # %s\n", prog_sea, ldata(lfirst(pgm->id)));
  }

  if(tsv)
    dev_info(".prog\t%s\tid\t", id);
  else {
    dev_cout(pgm->comments, "id", 0, 0);
    dev_info("    %-22s = ", "id");
  }
  for(firstid=1, ln=lfirst(pgm->id); ln; ln=lnext(ln)) {
    if(!firstid)
      dev_info(", ");
    firstid = 0;
    char *str = cfg_escape(ldata(ln));
    dev_info("%s", str);
    free(str);
  }
  if(tsv)
    dev_info("\n");
  else {
    dev_info(";");
    dev_cout(pgm->comments, "id", 1, 1);
  }

  _if_pgmout_str(strcmp, cfg_escape(pgm->desc), desc);
  if(!base || base->initpgm != pgm->initpgm)
    _pgmout_fmt("type", "\"%s\"", locate_programmer_type_id(pgm->initpgm));
  _if_pgmout_str(intcmp, cfg_strdup("dev_pgm_strct()", prog_modes_str(pgm->prog_modes)), prog_modes);
  _if_pgmout_str(boolcmp, cfg_strdup("dev_pgm_strct()", pgm->is_serialadapter? "yes": "no"), is_serialadapter);
  _if_pgmout_str(intcmp, cfg_strdup("dev_pgm_strct()", extra_features_str(pgm->extra_features)), extra_features);
  if(!base || base->conntype != pgm->conntype)
    _pgmout_fmt("connection_type", "%s", connstr(pgm->conntype));
  _if_pgmout(intcmp, "%d", baudrate);

  _if_pgmout(intcmp, "0x%04x", usbvid);

  char *pgmstr = dev_usbpid_liststr(pgm);
  int show = !base;

  if(base) {
    char *basestr = dev_usbpid_liststr(base);
    show = !str_eq(basestr, pgmstr);
    free(basestr);
  }
  if(show)
    dev_part_strct_entry(tsv, ".prog", id, NULL, "usbpid", pgmstr, pgm->comments);
  else                          // dev_part_strct_entry() frees pgmstr
    free(pgmstr);

  _if_pgmout_str(strcmp, cfg_escape(pgm->usbdev), usbdev);
  _if_pgmout_str(strcmp, cfg_escape(pgm->usbsn), usbsn);
  _if_pgmout_str(strcmp, cfg_escape(pgm->usbvendor), usbvendor);
  _if_pgmout_str(strcmp, cfg_escape(pgm->usbproduct), usbproduct);

  for(int i=0; i<N_PINS; i++) {
    char *str = pins_to_strdup(pgm->pin+i);
    char *bstr = base? pins_to_strdup(base->pin+i): NULL;
    if(!base || !str_eq(bstr, str))
      _pgmout_fmt(avr_pin_lcname(i), "%s", str);

    free(str);
    if(bstr)
      free(bstr);
  }

  pgmstr = dev_hvupdi_support_liststr(pgm);
  show = !base;

  if(base) {
    char *basestr = dev_hvupdi_support_liststr(base);
    show = !str_eq(basestr, pgmstr);
    free(basestr);
  }
  if(show)
    dev_part_strct_entry(tsv, ".prog", id, NULL, "hvupdi_support", pgmstr, pgm->comments);
  else                          // dev_part_strct_entry() frees pgmstr
    free(pgmstr);

  if(injct)
    for(size_t i=0; i<sizeof pgminj/sizeof*pgminj; i++)
      if(pgminj[i].pgmid)
        for(LNODEID *ln=lfirst(pgm->id); ln; ln=lnext(ln))
          if(str_casematch(pgminj[i].pgmid, ldata(ln)))
            dev_part_strct_entry(tsv, ".prog", ldata(ln), NULL,
              pgminj[i].var, cfg_strdup("pgminj", pgminj[i].value), NULL);

  if(!tsv) {
    dev_cout(pgm->comments, ";", 0, 0);
    dev_info(";\n");
  }
}


// -c */[ASsrti]
void dev_output_pgm_defs(char *pgmidcp) {
  bool astrc, strct, cmpst, raw, tsv, injct;
  char *flags;
  int nprinted;
  PROGRAMMER *nullpgm = pgm_new();

  if((flags = strchr(pgmidcp, '/')))
    *flags++ = 0;

  if(!flags && str_eq(pgmidcp, "*")) // Treat -c * as if it was -c */s
    flags = "s";

  if(!*flags || !strchr("ASsrti", *flags)) {
    dev_info("%s: flags for developer option -c <wildcard>/<flags> not recognised\n", progname);
    dev_info(
      "Wildcard examples (these need protecting in the shell through quoting):\n"
      "         * all known programmers\n"
      "   avrftdi just this programmer\n"
      "  jtag*pdi matches jtag2pdi, jtag3pdi, jtag3updi and jtag2updi\n"
      "  jtag?pdi matches jtag2pdi and jtag3pdi\n"
      "Flags (one or more of the characters below):\n"
      "       A  show entries of avrdude.conf programmers with all values\n"
      "       S  show entries of avrdude.conf programmers with necessary values\n"
      "       s  show short entries of avrdude.conf programmers using parent\n"
      "       r  show entries of avrdude.conf programmers as raw dump\n"
      "       t  use tab separated values as much as possible\n"
      "       i  inject assignments from source code table\n"
      "Examples:\n"
      "  $ avrdude -c usbasp/s\n"
      "  $ avrdude -c */st | grep baudrate\n"
      "  $ avrdude -c */r | sort\n"
      "Notes:\n"
      "  -c * is the same as -c */s\n"
      "  This help message is printed using any unrecognised flag, eg, -c/h\n"
      "  Leaving no space after -c can be an OK substitute for quoting in shells\n"
      "  /s, /S and /A outputs are designed to be used as input in avrdude.conf\n"
      "  Sorted /r output should stay invariant when rearranging avrdude.conf\n"
      "  These options are just to help development, so not further documented\n"
    );
    return;
  }

  astrc = !!strchr(flags, 'A');
  strct = !!strchr(flags, 'S');
  cmpst = !!strchr(flags, 's');
  raw   = !!strchr(flags, 'r');
  tsv   = !!strchr(flags, 't');
  injct = !!strchr(flags, 'i');

  nprinted = dev_nprinted;

  LNODEID ln1, ln2;
  for(ln1=lfirst(programmers); ln1; ln1=lnext(ln1)) {
    PROGRAMMER *pgm = ldata(ln1);
    int matched = 0;
    for(ln2=lfirst(pgm->id); ln2; ln2=lnext(ln2)) {
      if(str_casematch(pgmidcp, ldata(ln2))) {
        matched = 1;
        break;
      }
    }
    if(!matched)
      continue;

    if(dev_nprinted > nprinted) {
      dev_info("\n");
      nprinted = dev_nprinted;
    }

    if(astrc || strct || cmpst)
      dev_pgm_strct(pgm, tsv,
        astrc? NULL:
        strct? nullpgm:
        pgm->parent_id && *pgm->parent_id? locate_programmer(programmers, pgm->parent_id): nullpgm,
        injct);

    if(raw)
      dev_pgm_raw(pgm);
  }
}
