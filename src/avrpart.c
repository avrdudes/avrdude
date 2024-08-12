/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004 Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
 * Copyright (C) 2022- Stefan Rueger <stefan.rueger@urclocks.com>
 * Copyright (C) 2023- Hans Eirik Bull
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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <ac_cfg.h>

#include "avrdude.h"
#include "libavrdude.h"

/***
 *** Elementary functions dealing with OPCODE structures
 ***/

OPCODE *avr_new_opcode(void) {
  return (OPCODE *) mmt_malloc(sizeof(OPCODE));
}

static OPCODE *avr_dup_opcode(const OPCODE *op) {
  if(op == NULL)                // Caller wants NULL if op == NULL
    return NULL;

  OPCODE *m = (OPCODE *) mmt_malloc(sizeof(*m));
  memcpy(m, op, sizeof(*m));

  return m;
}

void avr_free_opcode(OPCODE *op) {
  mmt_free(op);
}


// returns position 0..31 of highest bit set or INT_MIN if no bit is set
int intlog2(unsigned int n) {
  int ret;

  if(!n)
    return INT_MIN;

  for(ret = 0; n >>= 1; ret++)
    continue;

  return ret;
}


/*
 * avr_set_bits()
 *
 * Set instruction bits in the specified command based on the opcode.
 */
int avr_set_bits(const OPCODE *op, unsigned char *cmd) {
  int i, j, bit;
  unsigned char mask;

  for (i=0; i<32; i++) {
    if (op->bit[i].type == AVR_CMDBIT_VALUE || op->bit[i].type == AVR_CMDBIT_IGNORE) {
      j = 3 - i / 8;
      bit = i % 8;
      mask = 1 << bit;
      if (op->bit[i].value && op->bit[i].type == AVR_CMDBIT_VALUE)
        cmd[j] = cmd[j] | mask;
      else
        cmd[j] = cmd[j] & ~mask;
    }
  }

  return 0;
}


/*
 * avr_set_addr()
 *
 * Set address bits in the specified command based on the opcode, and
 * the address.
 */
int avr_set_addr(const OPCODE *op, unsigned char *cmd, unsigned long addr) {
  int i, j, bit;
  unsigned long value;
  unsigned char mask;

  for (i=0; i<32; i++) {
    if (op->bit[i].type == AVR_CMDBIT_ADDRESS) {
      j = 3 - i / 8;
      bit = i % 8;
      mask = 1 << bit;
      value = addr >> op->bit[i].bitno & 0x01;
      if (value)
        cmd[j] = cmd[j] | mask;
      else
        cmd[j] = cmd[j] & ~mask;
    }
  }

  return 0;
}


/*
 * avr_set_addr_mem()
 *
 * Set address bits in the specified command based on the memory, opcode and
 * address; addr must be a word address for flash or, for all other memories,
 * a byte address; returns 0 on success and -1 on error (no memory or no
 * opcode) or, if positive, bn+1 where bn is bit number of the highest
 * necessary bit that the opcode does not provide.
 */
int avr_set_addr_mem(const AVRMEM *mem, int opnum, unsigned char *cmd, unsigned long addr) {
  int ret, isflash, lo, hi, memsize, pagesize;
  OPCODE *op;

  if(!mem)
    return -1;

  if(!(op = mem->op[opnum]))
    return -1;

  isflash = mem_is_in_flash(mem);
  memsize = mem->size >> isflash;        // word addresses for flash
  pagesize = mem->page_size >> isflash;

  // compute range lo..hi of needed address bits
  switch(opnum) {
  case AVR_OP_READ:
  case AVR_OP_WRITE:
  case AVR_OP_READ_LO:
  case AVR_OP_READ_HI:
  case AVR_OP_WRITE_LO:
  case AVR_OP_WRITE_HI:
    lo = 0;
    hi = intlog2(memsize-1);    // memsize = 1 implies no addr bit is needed
    break;

  case AVR_OP_LOADPAGE_LO:
  case AVR_OP_LOADPAGE_HI:
    lo = 0;
    hi = intlog2(pagesize-1);
    break;

  case AVR_OP_LOAD_EXT_ADDR:
    lo = 16;
    hi = intlog2(memsize-1);
    break;

  case AVR_OP_WRITEPAGE:
    lo = intlog2(pagesize);
    hi = intlog2(memsize-1);
    break;

  case AVR_OP_CHIP_ERASE:
  case AVR_OP_PGM_ENABLE:
  default:
    lo = 0;
    hi = -1;
    break;
  }

  // Unless it's load extended address, ISP chips only deal with 16 bit addresses
  if(opnum != AVR_OP_LOAD_EXT_ADDR && hi > 15)
    hi = 15;

  unsigned char avail[32];
  memset(avail, 0, sizeof avail);

  for(int i=0; i<32; i++) {
    if(op->bit[i].type == AVR_CMDBIT_ADDRESS) {
      int bitno, j, bit;
      unsigned char mask;

      bitno = op->bit[i].bitno & 31;
      j = 3 - i / 8;
      bit = i % 8;
      mask = 1 << bit;
      avail[bitno] = 1;

      // 'a' bit with number outside bit range [lo, hi] is set to 0
      if (bitno >= lo && bitno <= hi? (addr >> bitno) & 1: 0)
        cmd[j] = cmd[j] | mask;
      else
        cmd[j] = cmd[j] & ~mask;
    }
  }

  ret = 0;
  if(lo >= 0 && hi < 32 && lo <= hi)
    for(int bn=lo; bn <= hi; bn++)
      if(!avail[bn])            // necessary bit bn misses in opcode
        ret = bn+1;

  return ret;
}


/*
 * avr_set_input()
 *
 * Set input data bits in the specified command based on the opcode,
 * and the data byte.
 */
int avr_set_input(const OPCODE *op, unsigned char *cmd, unsigned char data) {
  int i, j, bit;
  unsigned char value;
  unsigned char mask;

  for (i=0; i<32; i++) {
    if (op->bit[i].type == AVR_CMDBIT_INPUT) {
      j = 3 - i / 8;
      bit = i % 8;
      mask = 1 << bit;
      value = data >> op->bit[i].bitno & 0x01;
      if (value)
        cmd[j] = cmd[j] | mask;
      else
        cmd[j] = cmd[j] & ~mask;
    }
  }

  return 0;
}


/*
 * avr_get_output()
 *
 * Retrieve output data bits from the command results based on the
 * opcode data.
 */
int avr_get_output(const OPCODE *op, const unsigned char *res, unsigned char *data) {
  int i, j, bit;
  unsigned char value;
  unsigned char mask;

  for (i=0; i<32; i++) {
    if (op->bit[i].type == AVR_CMDBIT_OUTPUT) {
      j = 3 - i / 8;
      bit = i % 8;
      mask = 1 << bit;
      value = ((res[j] & mask) >> bit) & 0x01;
      value = value << op->bit[i].bitno;
      if (value)
        *data = *data | value;
      else
        *data = *data & ~value;
    }
  }

  return 0;
}


/*
 * avr_get_output_index()
 *
 * Calculate the byte number of the output data based on the
 * opcode data.
 */
int avr_get_output_index(const OPCODE *op) {
  int i, j;

  for (i=0; i<32; i++) {
    if (op->bit[i].type == AVR_CMDBIT_OUTPUT) {
      j = 3 - i / 8;
      return j;
    }
  }

  return -1;
}


/***
 *** Elementary functions dealing with AVRMEM structures
 ***/

AVRMEM *avr_new_mem(void) {
  AVRMEM *m = (AVRMEM *) mmt_malloc(sizeof(*m));
  m->desc = cache_string("");
  m->page_size = 1;             // Ensure not 0
  m->initval = -1;              // Unknown value represented as -1
  m->bitmask = -1;              // Default to -1

  return m;
}

// Create memory from name and size
AVRMEM *avr_new_memory(const char *name, int size) {
  AVRMEM *m = (AVRMEM *) mmt_malloc(sizeof(*m));
  m->desc = cache_string(name);
  m->page_size = 1;             // Ensure not 0
  m->size = size;
  m->buf = mmt_malloc(size);
  m->tags = mmt_malloc(size);
  m->initval = -1;              // Unknown value represented as -1
  m->bitmask = -1;              // Default to -1

  return m;
}

AVRMEM_ALIAS *avr_new_memalias(void) {
  AVRMEM_ALIAS *m = (AVRMEM_ALIAS *) mmt_malloc(sizeof *m);
  m->desc = cache_string("");
  return m;
}

// Return longer name of memory including alias if any, eg, fuse7/codesize
const char *avr_mem_name(const AVRPART *p, const AVRMEM *mem) {
  char ret[1024];
  const int n = sizeof ret - 1;

  strncpy(ret, mem->desc, n/2);
  ret[n/2] = 0;

  AVRMEM_ALIAS *alias = avr_find_memalias(p, mem);
  if(alias && alias->desc && *alias->desc) {
    int l = strlen(ret);
    ret[l] = '/';
    strncpy(ret+l+1, alias->desc, n-l-1);
    ret[n] = 0;
  }
  return cache_string(ret);
}

/*
 * Allocate and initialize memory buffers for each of the device's
 * defined memory regions.
 */
int avr_initmem(const AVRPART *p) {
  if(p == NULL || p->mem == NULL)
    return -1;

  for (LNODEID ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    AVRMEM *m = ldata(ln);
    m->buf  = mmt_malloc(m->size);
    m->tags = mmt_malloc(m->size);
  }

  return 0;
}


AVRMEM *avr_dup_mem(const AVRMEM *m) {
  AVRMEM *n = avr_new_mem();

  if(m) {
    *n = *m;

    if(m->buf) {
      n->buf = mmt_malloc(n->size);
      memcpy(n->buf, m->buf, n->size);
    }

    if(m->tags) {
      n->tags = (unsigned char *) mmt_malloc(n->size);
      memcpy(n->tags, m->tags, n->size);
    }

    for(int i = 0; i < AVR_OP_MAX; i++)
      n->op[i] = avr_dup_opcode(n->op[i]);
  }

  return n;
}

AVRMEM_ALIAS *avr_dup_memalias(const AVRMEM_ALIAS *m) {
  AVRMEM_ALIAS *n = avr_new_memalias();

  if(m)
    *n = *m;

  return n;
}

void avr_free_mem(AVRMEM * m) {
  if(m == NULL)
    return;

  if(m->buf) {
    mmt_free(m->buf);
    m->buf = NULL;
  }
  if(m->tags) {
    mmt_free(m->tags);
    m->tags = NULL;
  }
  for(size_t i=0; i<sizeof(m->op)/sizeof(m->op[0]); i++) {
    if(m->op[i]) {
      avr_free_opcode(m->op[i]);
      m->op[i] = NULL;
    }
  }
  mmt_free(m);
}

void avr_free_memalias(AVRMEM_ALIAS *m) {
  mmt_free(m);
}

AVRMEM_ALIAS *avr_locate_memalias(const AVRPART *p, const char *desc) {
  AVRMEM_ALIAS *m, *match;
  LNODEID ln;
  int matches, d1;
  size_t l;

  if(!p || !desc || !(d1 = *desc) || !p->mem_alias)
    return NULL;

  l = strlen(desc);
  matches = 0;
  match = NULL;
  for(ln=lfirst(p->mem_alias); ln; ln=lnext(ln)) {
    m = ldata(ln);
    if(d1 == *m->desc && !strncmp(m->desc, desc, l)) { // Partial initial match
      match = m;
      matches++;
      if(m->desc[l] == 0)       // Exact match; return straight away
        return m;
    }
  }

  return matches == 1? match: NULL;
}

AVRMEM *avr_locate_mem_noalias(const AVRPART *p, const char *desc) {
  AVRMEM *m, *match;
  LNODEID ln;
  int matches, d1;
  size_t l;

  if(!p || !desc || !(d1 = *desc) || !p->mem)
    return NULL;

  l = strlen(desc);
  matches = 0;
  match = NULL;
  for(ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    m = ldata(ln);
    if(d1 == *m->desc && !strncmp(m->desc, desc, l)) { // Partial initial match
      match = m;
      matches++;
      if(m->desc[l] == 0)       // Exact match; return straight away
        return m;
    }
  }

  return matches == 1? match: NULL;
}


AVRMEM *avr_locate_mem(const AVRPART *p, const char *desc) {
  AVRMEM *m = avr_locate_mem_noalias(p, desc);

  if(m)
    return m;

  // Not yet found: look for matching alias name
  AVRMEM_ALIAS *a = avr_locate_memalias(p, desc);
  return a? a->aliased_mem: NULL;
}

// Return the first fuse which has off as offset or which has high byte and off-1 as offset
AVRMEM *avr_locate_fuse_by_offset(const AVRPART *p, unsigned int off) {
  AVRMEM *m;

  if(p && p->mem)
    for(LNODEID ln=lfirst(p->mem); ln; ln=lnext(ln))
      if(mem_is_a_fuse(m = ldata(ln)))
        if(off == mem_fuse_offset(m) || (m->size == 2 && off-1 == mem_fuse_offset(m)))
          return m;

  return NULL;
}

// Return the first memory that shares the type incl any fuse identified by offset in fuses
AVRMEM *avr_locate_mem_by_type(const AVRPART *p, Memtype type) {
  AVRMEM *m;
  Memtype off = type & MEM_FUSEOFF_MASK;
  type &= ~(Memtype) MEM_FUSEOFF_MASK;

  if(p && p->mem)
    for(LNODEID ln=lfirst(p->mem); ln; ln=lnext(ln))
      if((m = ldata(ln))->type & type)
        if(type != MEM_IS_A_FUSE || off == mem_fuse_offset(m))
          return m;

  return NULL;
}

// Return offset of memory data
unsigned int avr_data_offset(const AVRPART *p) {
  return p->prog_modes & (PM_PDI | PM_UPDI)? 0x1000000: 0;
}

AVRMEM_ALIAS *avr_find_memalias(const AVRPART *p, const AVRMEM *m_orig) {
  if(p && p->mem_alias && m_orig)
    for(LNODEID ln=lfirst(p->mem_alias); ln; ln=lnext(ln)) {
      AVRMEM_ALIAS *m = ldata(ln);
      if(m->aliased_mem == m_orig)
        return m;
    }

  return NULL;
}

// Return index in uP_table for part or -1
int avr_locate_upidx(const AVRPART *p) {
  int idx = -1;

  if(!p)
    return -1;
  if(p->mcuid >= 0)
    idx = upidxmcuid(p->mcuid);
  if(idx < 0 && p->desc && *p->desc)
    idx = upidxname(p->desc);

  if(idx < 0)
    pmsg_error("uP_table neither knows mcuid %d nor part %s\n",
      p->mcuid, p->desc && *p->desc? p->desc: "???");

  return idx;
}

// Return pointer to uP_table entry for part p
const Avrintel *avr_locate_uP(const AVRPART *p) {
  int idx = avr_locate_upidx(p);

  return idx < 0? NULL: uP_table + idx;
}


// Return pointer to config table for the part and set number of config bitfields
const Configitem *avr_locate_configitems(const AVRPART *p, int *ncp) {
  int idx = avr_locate_upidx(p);
  if(idx < 0)
    return NULL;

  *ncp = uP_table[idx].nconfigs;
  return uP_table[idx].cfgtable;
}

// Return pointer to ISR table for the part and set number of interrupts
const char * const *avr_locate_isrtable(const AVRPART *p, int *nip) {
  int idx = avr_locate_upidx(p);
  if(idx < 0)
    return NULL;

  *nip = uP_table[idx].ninterrupts;
  return uP_table[idx].isrtable;
}

// Return pointer to register file for the part and set number of registers
const Register_file *avr_locate_register_file(const AVRPART *p, int *nrp) {
  int idx = avr_locate_upidx(p);
  if(idx < 0)
    return NULL;

  *nrp = uP_table[idx].nregisters;
  return uP_table[idx].regf;
}

/*
 * Return pointer to a register that uniquely matches the argument reg or
 * NULL if no or more than one register matches the reg argument.
 *
 * Register names have the form module.name or module.instance.name. The
 * caller provides a matching function which can be str_eq, str_starts,
 * str_matched_by etc. If reg is a full, existing register name, eg,
 * porta.out then a pointer to that register entry is returned irrespective
 * of the matching function. avr_locate_register() also tries to match the
 * last colon-separated segments (instance.name or name) using the provided
 * matching function. If reg is the same as instance.name or name then the
 * matching function switches to str_eq(). This allows the only ADC register
 * adc.adc to be addressed by adc under a lax str_begins() matching even
 * though there are other registers that start with adc, eg, adc.adcsra.
 */

const Register_file *avr_locate_register(const Register_file *rgf, int nr, const char *reg,
  int (*match)(const char *, const char*)) {

  if(!rgf || nr < 1 || !reg || !match)
    return NULL;

  const Register_file *ret = NULL;
  int nmatches = 0, eqmatch = match == str_eq;

  for(int i = 0; i < nr; i++) {
    int reg_matched = 0;
    // Match against module.instance.name, instance.name or name
    for(const char *p = rgf[i].reg; p; p = strchr(p, '.'), p = p? p+1: p)
      if(match(p, reg)) {
        if(p == rgf[i].reg && (eqmatch || str_eq(p, reg))) // Reg is full name: return straight away
          return rgf+i;
        if(!eqmatch && str_eq(p, reg)) // reg same as segment: switch to str_eq() matching
          return avr_locate_register(rgf, nr, reg, str_eq);
        if(!reg_matched++)      // Record a matching register only once
          nmatches++, ret = rgf+i;
      }
  }

  return nmatches == 1? ret: NULL;
}

/*
 * Return a NULL terminated malloc'd list of pointers to matching registers
 *
 * Register names have the form module.name or module.instance.name. The
 * caller provides a matching function which can be str_eq, str_starts,
 * str_matched_by etc. If reg is a full, existing register name, eg,
 * porta.out then the returned list is confined to this specific entry
 * irrespective of the matching function. avr_locate_registerlist() also
 * tries to match the last colon-separated segments (instance.name or name)
 * using the provided matching function. If the argument reg is the same as
 * instance.name or name then the matching function switches to str_eq()
 * reducing the returned list to those that match that full segment. This
 * behaviour can be suppressed by specifying a pattern for reg, eg, adc*
 * together with the matching function str_matched_by.
 */
const Register_file **avr_locate_registerlist(const Register_file *rgf, int nr, const char *reg,
  int (*match)(const char *, const char*)) {

  const Register_file **ret = mmt_malloc(sizeof rgf*(nr>0? nr+1: 1)), **r = ret;
  int eqmatch = match == str_eq;

  if(rgf && reg && match)
    for(int i = 0; i < nr; i++) {
      int reg_matched = 0;
      // Match against module.instance.name, instance.name or name
      for(const char *p = rgf[i].reg; p; p = strchr(p, '.'), p = p? p+1: p)
        if(match(p, reg)) {
          if(p == rgf[i].reg && (eqmatch || str_eq(p, reg))) { // Reg is full name: return only that
            ret[0] = rgf+i;
            ret[1] = NULL;
            return ret;
          }
          if(!eqmatch && str_eq(p, reg)) { // reg same as segment: switch to str_eq() match
            mmt_free(ret);
            return avr_locate_registerlist(rgf, nr, reg, str_eq);
          }
          if(!reg_matched++)      // Record a matching register only once
            *r++ = rgf+i;
        }
    }
  *r = NULL;

  return ret;
}

/*
 * Return pointer to a configuration bitfield that uniquely matches the
 * argument name. Return NULL if none matches or more than one do.
 *
 * The caller provides a matching function which can be str_eq, str_starts,
 * str_matched_by etc. If name is the full name of a configuration bitfield
 * then a pointer to that is returned irrespective of the matching function.
 */
const Configitem *avr_locate_config(const Configitem *cfg, int nc, const char *name,
  int (*match)(const char *, const char*)) {

  if(!cfg || nc < 1 || !name || !match)
    return NULL;

  const Configitem *ret = NULL;
  int nmatches = 0;

  for(int i = 0; i < nc; i++) {
    if(match(cfg[i].name, name)) {
      if(match == str_eq || str_eq(cfg[i].name, name)) // Full name specified: return straight away
        return cfg+i;
      nmatches++, ret = cfg+i;
    }
  }

  return nmatches == 1? ret: NULL;
}

/*
 * Return a NULL terminated malloc'd list of pointers to config bitfields
 *
 * The caller provides a matching function which can be str_eq, str_starts,
 * str_matched_by etc. If name is a full, existing config name then the
 * returned list is confined to this specific entry irrespective of the
 * matching function.
 */
const Configitem **avr_locate_configlist(const Configitem *cfg, int nc, const char *name,
  int (*match)(const char *, const char*)) {

  const Configitem **ret = mmt_malloc(sizeof cfg*(nc>0? nc+1: 1)), **r = ret;

  if(cfg && name && match) {
    for(int i = 0; i < nc; i++)
      if(match(cfg[i].name, name)) {
        if(match == str_eq || str_eq(cfg[i].name, name)) { // Full name specified: return straight away
          ret[0] = cfg+i;
          ret[1] = NULL;
          return ret;
        }
        *r++ = cfg+i;
      }
  }
  *r = NULL;

  return ret;
}

// Return memory associated with config item and fill in pointer to Configitem record
static AVRMEM *avr_locate_config_mem_c_value(const PROGRAMMER *pgm, const AVRPART *p,
  const char *cname, const Configitem **cp, int *valp) {

  int nc = 0;
  const Configitem *cfg = avr_locate_configitems(p, &nc);

  if(!cfg || nc < 1) {
    pmsg_error("avrintel.c does not hold configuration information for %s\n", p->desc);
    return NULL;
  }

  const Configitem *c = avr_locate_config(cfg, nc, cname, str_contains);
  if(!c) {
    pmsg_error("%s does not have a unique config item matched by %s\n", p->desc, cname);
    return NULL;
  }

  AVRMEM *mem = str_starts(c->memstr, "lock")? avr_locate_lock(p): avr_locate_fuse_by_offset(p, c->memoffset);
  if(!mem)
    mem = avr_locate_mem(p, c->memstr);
  if(!mem) {
    pmsg_error("%s does not have the memory %s needed for config item %s\n", p->desc, c->memstr, cname);
    return NULL;
  }

  if(mem->size < 1 || mem->size > 4) {
    pmsg_error("cannot handle size %d of %s's memory %s for config item %s\n", mem->size, p->desc, c->memstr, cname);
    return NULL;
  }

  int fusel = 0;
  for(int i = 0; i < mem->size; i++)
    if(led_read_byte(pgm, p, mem, i, (unsigned char *) &fusel + i) < 0) {
      pmsg_error("cannot read from  %s's %s memory\n", p->desc, mem->desc);
      return NULL;
    }

  *cp = c;
  *valp = fusel;
  return mem;
}

// Initialise *valuep with configuration value of named configuration bitfield
int avr_get_config_value(const PROGRAMMER *pgm, const AVRPART *p, const char *cname, int *valuep) {
  const Configitem *c;
  int fusel;

  if(!avr_locate_config_mem_c_value(pgm, p, cname, &c, &fusel))
    return -1;

  if(valuep)
    *valuep = (fusel & c->mask) >> c->lsh;
  return 0;
}

// Set configuration value of named configuration bitfield to value
int avr_set_config_value(const PROGRAMMER *pgm, const AVRPART *p, const char *cname, int value) {
  AVRMEM *mem;
  const Configitem *c;
  int fusel;

  if(!(mem=avr_locate_config_mem_c_value(pgm, p, cname, &c, &fusel)))
    return -1;

  if((value << c->lsh) & ~c->mask)
    pmsg_warning("value 0x%02x has bits set outside bitfield mask 0x%02x\n", value, c->mask >> c->lsh);

  int newval = (fusel & ~c->mask) | ((value << c->lsh) & c->mask);

  if(newval != fusel) {
    for(int i = 0; i < mem->size; i++)
      if(led_write_byte(pgm, p, mem, i, ((unsigned char *) &newval)[i]) < 0) {
        pmsg_error("cannot write to %s's %s memory\n", p->desc, mem->desc);
        return -1;
      }
  }

  return 0;
}


static const char *print_num(const char *fmt, int n) {
  return str_ccprintf(n<10? "%d": fmt, n);
}

static int num_len(const char *fmt, int n) {
  return strlen(print_num(fmt, n));
}

void avr_mem_display(FILE *f, const PROGRAMMER *pgm, const AVRPART *p, const char *prefix) {
  const char *table_colum[] = {"Memory", "Size", "Pg size", "Offset"};
  const char *table_padding = "-------------------------------";
  const int memory_col = 0, offset_col = 3;
  int m_char_max[4];
  AVRMEM *m;

  for(int i = 0; i < 4; i++)
    m_char_max[i] = strlen(table_colum[i]);

  for (LNODEID ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    if(avr_mem_exclude(pgm, p, (m = ldata(ln))))
      continue;

    int m_size[] = {0, m->size, m->page_size, m->offset};

    // Max column widths
    for(int i = 0; i < 4; i++) {
      int len = i == memory_col? (int) strlen(avr_mem_name(p, m)):
        num_len(i == offset_col? "0x%04x": "%d", m_size[i]); // size/pgsize/offset
      if(m_char_max[i] < len)
        m_char_max[i] = len;
    }
  }

  // Print memory table header
  if(p->prog_modes & (PM_PDI | PM_UPDI)) {
    fprintf(f,
      "\n%s%-*s  %*s  %-*s  %*s\n"
      "%s%.*s--%.*s--%.*s--%.*s\n",
      prefix,
      m_char_max[0], table_colum[0],
      m_char_max[1], table_colum[1],
      m_char_max[2], table_colum[2],
      m_char_max[3], table_colum[3],
      prefix,
      m_char_max[0], table_padding,
      m_char_max[1], table_padding,
      m_char_max[2], table_padding,
      m_char_max[3], table_padding);
  } else {
    fprintf(f,
      "\n%s%-*s  %*s  %-*s\n"
      "%s%.*s--%.*s--%.*s\n",
      prefix,
      m_char_max[0], table_colum[0],
      m_char_max[1], table_colum[1],
      m_char_max[2], table_colum[2],
      prefix,
      m_char_max[0], table_padding,
      m_char_max[1], table_padding,
      m_char_max[2], table_padding);
  }

  for (LNODEID ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    if(avr_mem_exclude(pgm, p, (m = ldata(ln))))
      continue;

    // Print memory table content
    if(p->prog_modes & (PM_PDI | PM_UPDI)) {
      fprintf(f, "%s%-*s  %*d  %*d  %*s \n",
        prefix,
        m_char_max[0], avr_mem_name(p, m),
        m_char_max[1], m->size,
        m_char_max[2], m->page_size,
        m_char_max[3], print_num("0x%04x", m->offset));
    } else {
      fprintf(f, "%s%-*s  %*d  %*d\n",
        prefix,
        m_char_max[0], avr_mem_name(p, m),
        m_char_max[1], m->size,
        m_char_max[2], m->page_size);
    }
  }
}

int avr_variants_display(FILE *f, const AVRPART *p, const char *prefix) {
  const char *table_padding = "-------------------------------";
  const char *var_table_column[] = {"Variants", "Package", "F max", "T range", "V range"};
  char var_tok[5][50];
  int var_tok_len[5];

  for(int i = 0; i < 5; i++)
    var_tok_len[i] = strlen(var_table_column[i]);

  if(lsize(p->variants)) {
    // Split, eg, "ATtiny841-SSU:  SOIC14, Fmax=16 MHz, T=[-40 C, 85 C], Vcc=[1.7 V, 5.5 V]"
    for(LNODEID ln=lfirst(p->variants); ln; ln=lnext(ln))
      if(5 == sscanf(ldata(ln), "%49[^:]: %49[^,], Fmax=%49[^,], T=%48[^]]], Vcc=%48[^]]]",
        var_tok[0], var_tok[1], var_tok[2], var_tok[3], var_tok[4]))
        for(int i = 0; i < 5; i++)
          if(var_tok_len[i] < (int) strlen(var_tok[i]))
            var_tok_len[i] = strlen(var_tok[i]) + (i>2); // Add 1 for closing interval bracket

    // Print variants table header
    fprintf(f,
      "\n%s%-*s  %-*s  %-*s  %-*s  %-*s\n"
        "%s%.*s--%.*s--%.*s--%.*s--%.*s\n",
      prefix,
      var_tok_len[0], var_table_column[0],
      var_tok_len[1], var_table_column[1],
      var_tok_len[2], var_table_column[2],
      var_tok_len[3], var_table_column[3],
      var_tok_len[4], var_table_column[4],
      prefix,
      var_tok_len[0], table_padding,
      var_tok_len[1], table_padding,
      var_tok_len[2], table_padding,
      var_tok_len[3], table_padding,
      var_tok_len[4], table_padding);

    // Print variants table content
    for(LNODEID ln=lfirst(p->variants); ln; ln=lnext(ln))
      if(5 == sscanf(ldata(ln), "%49[^:]: %49[^,], Fmax=%49[^,], T=%48[^]]], Vcc=%48[^]]]",
        var_tok[0], var_tok[1], var_tok[2], var_tok[3], var_tok[4])) {
        strcat(var_tok[3], "]");
        strcat(var_tok[4], "]");
        fprintf(f,
          "%s%-*s  %-*s  %-*s  %-*s  %-*s\n",
          prefix,
          var_tok_len[0], var_tok[0],
          var_tok_len[1], var_tok[1],
          var_tok_len[2], var_tok[2],
          var_tok_len[3], var_tok[3],
          var_tok_len[4], var_tok[4]);
      }

    return 0;
  }
  return -1;
}


/*
 * Elementary functions dealing with AVRPART structures
 */

AVRPART *avr_new_part(void) {
  AVRPART *p = (AVRPART *) mmt_malloc(sizeof(AVRPART));
  const char *nulp = cache_string("");

  memset(p, 0, sizeof(*p));

  // Initialise const char * and LISTID entities
  p->desc = nulp;
  p->id = nulp;
  p->parent_id = nulp;
  p->family_id = nulp;
  p->config_file = nulp;
  p->mem = lcreat(NULL, 0);
  p->mem_alias = lcreat(NULL, 0);
  p->variants = lcreat(NULL, 0);

  // Default values
  p->mcuid = -1;
  p->hvupdi_variant = -1;
  p->autobaud_sync = 0x30; // STK_GET_SYNC
  memset(p->signature, 0xFF, 3);
  p->reset_disposition = RESET_DEDICATED;
  p->retry_pulse = PIN_AVR_SCK;
  p->flags = AVRPART_SERIALOK | AVRPART_PARALLELOK | AVRPART_ENABLEPAGEPROGRAMMING;
  p->ctl_stack_type = CTL_STACK_NONE;
  p->ocdrev = -1;
  p->lineno = 0;

  return p;
}


AVRPART *avr_dup_part(const AVRPART *d) {
  AVRPART *p = avr_new_part();

  if(d) {
    *p = *d;

    // Leave variants list empty but duplicate the memory and alias chains
    p->variants = lcreat(NULL, 0);
    p->mem = lcreat(NULL, 0);
    p->mem_alias = lcreat(NULL, 0);
    for(LNODEID ln=lfirst(d->mem); ln; ln=lnext(ln)) {
      AVRMEM *m = ldata(ln);
      AVRMEM *m2 = avr_dup_mem(m);
      ladd(p->mem, m2);
      // See if there is any alias for it
      for(LNODEID ln2=lfirst(d->mem_alias); ln2; ln2=lnext(ln2)) {
        AVRMEM_ALIAS *a = ldata(ln2);
        if (a->aliased_mem == m) {
          // Yes, duplicate it, adjust the pointer and add to new list
          AVRMEM_ALIAS *a2 = avr_dup_memalias(a);
          a2->aliased_mem = m2;
          ladd(p->mem_alias, a2);
        }
      }
    }

    for(int i = 0; i < AVR_OP_MAX; i++)
      p->op[i] = avr_dup_opcode(p->op[i]);
  }

  return p;
}

void avr_free_part(AVRPART * d) {
  ldestroy_cb(d->mem, (void(*)(void *)) avr_free_mem);
  d->mem = NULL;
  ldestroy_cb(d->mem_alias, (void(*)(void *)) avr_free_memalias);
  d->mem_alias = NULL;
  ldestroy_cb(d->variants, mmt_f_free);
  d->variants = NULL;

  /* do not free d->parent_id and d->config_file */
  for(size_t i=0; i<sizeof(d->op)/sizeof(d->op[0]); i++) {
    if (d->op[i] != NULL) {
      avr_free_opcode(d->op[i]);
      d->op[i] = NULL;
    }
  }
  mmt_free(d);
}

AVRPART *locate_part(const LISTID parts, const char *partdesc) {
  AVRPART * p = NULL;
  int found = 0;

  if(!parts || !partdesc)
    return NULL;

  for (LNODEID ln1=lfirst(parts); ln1 && !found; ln1=lnext(ln1)) {
    p = ldata(ln1);
    if(part_eq(p, partdesc, str_caseeq))
      found = 1;
  }

  return found? p: NULL;
}

AVRPART *locate_part_by_avr910_devcode(const LISTID parts, int devcode) {
  if(parts)
    for (LNODEID ln1=lfirst(parts); ln1; ln1=lnext(ln1)) {
      AVRPART * p = ldata(ln1);
      if (p->avr910_devcode == devcode)
        return p;
    }

  return NULL;
}

// Return pointer to first part that has signature sig (unless all 0xff or all 0x00); NULL if no match
AVRPART *locate_part_by_signature_pm(const LISTID parts, unsigned char *sig, int sigsize, int prog_modes) {
  if(parts && sigsize == 3) {
    for(LNODEID ln=lfirst(parts); ln; ln=lnext(ln)) {
      AVRPART *p = ldata(ln);
      if(!*p->id || *p->id == '.') // Skip stump entries
        continue;
      if(!is_memset(p->signature, 0xff, 3) && !is_memset(p->signature, 0, 3))
        if(!memcmp(p->signature, sig, 3) && p->prog_modes & prog_modes)
          return p;
    }
  }
  return NULL;
}

AVRPART *locate_part_by_signature(const LISTID parts, unsigned char *sig, int sigsize) {
  return locate_part_by_signature_pm(parts, sig, sigsize, PM_ALL);
}

// Return whether two signatures represent SW-compatible parts
int avr_sig_compatible(const unsigned char *sig1, const unsigned char *sig2) {
  // SW-compatible parts (same memories, interrupts and regfiles) despite different signatures
  static const struct { unsigned char sig[3], equ[3]; } compat[] = {
    {{0x1e, 0x97, 0x06}, {0x1e, 0x97, 0x05}}, // ATmega1284 vs ATmega1284P
    {{0x1e, 0xa7, 0x03}, {0x1e, 0xa7, 0x02}}, // ATmega1284RFR2 vs ATmega128RFR2
    {{0x1e, 0x94, 0x0f}, {0x1e, 0x94, 0x0a}}, // ATmega164A vs ATmega164P=ATmega164PA
    {{0x1e, 0x94, 0x10}, {0x1e, 0x94, 0x07}}, // ATmega165A vs ATmega165=ATmega165=ATmega165PA
    {{0x1e, 0x94, 0x06}, {0x1e, 0x94, 0x0b}}, // ATmega168=ATmega168A vs ATmega168P=ATmega168PA
    {{0x1e, 0x94, 0x11}, {0x1e, 0x94, 0x05}}, // ATmega169A vs ATmega169=ATmega169P=ATmega169PA
    {{0x1e, 0xa8, 0x03}, {0x1e, 0xa8, 0x02}}, // ATmega2564RFR2 vs ATmega256RFR2
    {{0x1e, 0x95, 0x15}, {0x1e, 0x95, 0x08}}, // ATmega324A vs ATmega324P
    {{0x1e, 0x95, 0x15}, {0x1e, 0x95, 0x11}}, // ATmega324A vs ATmega324PA
    {{0x1e, 0x95, 0x08}, {0x1e, 0x95, 0x11}}, // ATmega324P vs ATmega324PA
    {{0x1e, 0x95, 0x06}, {0x1e, 0x95, 0x0e}}, // ATmega3250=ATmega3250A vs ATmega3250P=ATmega3250PA
    {{0x1e, 0x95, 0x05}, {0x1e, 0x95, 0x0d}}, // ATmega325=ATmega325A vs ATmega325P=ATmega325PA
    {{0x1e, 0x95, 0x04}, {0x1e, 0x95, 0x0c}}, // ATmega3290=ATmega3290A vs ATmega3290P=ATmega3290PA
    {{0x1e, 0x95, 0x03}, {0x1e, 0x95, 0x0b}}, // ATmega329=ATmega329A vs ATmega329P=ATmega329PA
    {{0x1e, 0x92, 0x05}, {0x1e, 0x92, 0x0a}}, // ATmega48=ATmega48A vs ATmega48P=ATmega48PA
    {{0x1e, 0x92, 0x05}, {0x1e, 0x92, 0x0a}}, // ATmega48=ATmega48A vs ATmega48P=ATmega48PA
    {{0x1e, 0x96, 0x09}, {0x1e, 0x96, 0x0a}}, // ATmega644=ATmega644A vs ATmega644P=ATmega644PA
    {{0x1e, 0xa6, 0x03}, {0x1e, 0xa6, 0x02}}, // ATmega644RFR2 vs ATmega64RFR2
    {{0x1e, 0x96, 0x06}, {0x1e, 0x96, 0x0e}}, // ATmega6450=ATmega6450A vs ATmega6450P
    {{0x1e, 0x96, 0x05}, {0x1e, 0x96, 0x0d}}, // ATmega645=ATmega645A vs ATmega645P
    {{0x1e, 0x96, 0x04}, {0x1e, 0x96, 0x0c}}, // ATmega6490=ATmega6490A vs ATmega6490P
    {{0x1e, 0x96, 0x03}, {0x1e, 0x96, 0x0b}}, // ATmega649=ATmega649A vs ATmega649P
    {{0x1e, 0x93, 0x0a}, {0x1e, 0x93, 0x0f}}, // ATmega88=ATmega88A=ATA6612C vs ATmega88P=ATmega88PA
  };

  if(!memcmp(sig1, sig2, 3))
    return 1;

  for(size_t i = 0; i < sizeof compat/sizeof *compat; i++) {
    if(!memcmp(sig1, compat[i].sig, 3) && !memcmp(sig2, compat[i].equ, 3))
      return 1;
    if(!memcmp(sig2, compat[i].sig, 3) && !memcmp(sig1, compat[i].equ, 3))
      return 1;
  }

  return 0;
}

/*
 * Iterate over the list of avrparts given as "avrparts", and
 * call the callback function cb for each entry found.  cb is being
 * passed the following arguments:
 * . the name of the avrpart (for -p)
 * . the descriptive text given in the config file
 * . the name of the config file this avrpart has been defined in
 * . the line number of the config file this avrpart has been defined at
 * . the "cookie" passed into walk_avrparts() (opaque client data)
 */
void walk_avrparts(LISTID avrparts, walk_avrparts_cb cb, void *cookie)
{
  LNODEID ln1;
  AVRPART * p;

  for (ln1 = lfirst(avrparts); ln1; ln1 = lnext(ln1)) {
    p = ldata(ln1);
    cb(p->id, p->desc, p->config_file, p->lineno, cookie);
  }
}

/*
 * Compare function to sort a list of parts
 */
static int sort_avrparts_compare(const AVRPART *p1, const AVRPART *p2) {
  if(p1 == NULL || p1->desc == NULL || p2 == NULL || p2->desc == NULL)
    return 0;

  return strcasecmp(p1->desc, p2->desc);
}

/*
 * Sort the list avrparts of parts
 */
void sort_avrparts(LISTID avrparts)
{
  lsort(avrparts,(int (*)(void*, void*)) sort_avrparts_compare);
}


void avr_display(FILE *f, const PROGRAMMER *pgm, const AVRPART *p, const char *prefix, int verbose) {
  fprintf(f, "%sAVR part              : %s\n", prefix, p->desc);
  fprintf(f, "%sProgramming modes     : %s\n", prefix, str_prog_modes(p->prog_modes));

  if(verbose > 1) {
    avr_mem_display(f, pgm, p, prefix);
    avr_variants_display(f, p, prefix);
  }
}


char cmdbitchar(CMDBIT cb) {
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


char *cmdbitstr(CMDBIT cb) {
  char space[32];

  *space = cmdbitchar(cb);
  if(*space == 'a')
    sprintf(space+1, "%d", cb.bitno);
  else
    space[1] = 0;

  return mmt_strdup(space);
}


const char *opcodename(int opnum) {
  switch(opnum) {
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
char *opcode2str(const OPCODE *op, int opnum, int detailed) {
  char cb, space[1024], *sp = space;
  int compact = 1, printbit;

  if(!op)
    return mmt_strdup("NULL");

  // Can the opcode be printed in a compact way? Only if i, o and a bits are systematic.
  for(int i=31; i >= 0; i--)
    switch(op->bit[i].type) {
    case AVR_CMDBIT_ADDRESS:
      if(i<8 || i>23 || op->bit[i].bitno != (opnum == AVR_OP_LOAD_EXT_ADDR? i+8: i-8))
        compact = 0;
      break;
    case AVR_CMDBIT_INPUT:
    case AVR_CMDBIT_OUTPUT:
      if(op->bit[i].bitno != i%8)
        compact = 0;
    }

  if(detailed)
    *sp++ = '"';

  for(int i=31; i >= 0; i--) {
    cb = cmdbitchar(op->bit[i]);
    printbit = cb == 'a' || ((strchr("io", cb) && op->bit[i].bitno != i%8));
    *sp++ = !detailed && !compact && printbit? toupper(cb): cb; // Disambiguate tsv output
    if(!compact && printbit) {
      sprintf(sp, "%d", op->bit[i].bitno);
      sp += strlen(sp);
    }
    if(compact || !detailed) {
      if(i && i%8 == 0)
        *sp++ = '-', *sp++ = '-';
      else if(i && i%4 == 0)
        *sp++ = '.';
    } else {
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

  return mmt_strdup(space);
}


// Returns 1 if the part pointed to by p matches the string or pattern s under the function cmp(s, ...)
int part_eq(AVRPART *p, const char *s, int (*cmp)(const char *, const char *)) {
  // Matching id or desc? OK
  if(cmp(s, p->id) || cmp(s, p->desc))
    return 1;

  // Check against all variants, either up to colon or up to dash
  size_t desclen = strlen(p->desc), variantlen, dashlen;
  char query[1024];
  for(LNODEID ln = lfirst(p->variants); ln; ln = lnext(ln)) {
    const char *q = (const char *) ldata(ln), *qdash = strchr(q, '-'), *qcolon = strchr(q, ':');
    variantlen = qcolon? (size_t) (qcolon-q): strlen(q);
    dashlen = qdash? (size_t) (qdash-q): variantlen;
    if(variantlen < sizeof query) { // Sanity: should not expect such long strings
      // Variant names should be unique order numbers, but don't check (again) if it's the same as p->desc
      if(variantlen != desclen || memcmp(q, p->desc, desclen)) {
        memcpy(query, q, variantlen); query[variantlen] = 0;
        if(cmp(s, query))
          return 1;
        // The name before dash should normally be p->desc and the dash is meant to come before the colon
        if(dashlen > desclen && dashlen < variantlen) {
          query[dashlen] = 0;
          if(cmp(s, query))
            return 1;
        }
      }
    }
  }
  return 0;
}
