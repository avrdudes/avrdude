/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "ac_cfg.h"
#include "avrdude.h"
#include "libavrdude.h"

/***
 *** Elementary functions dealing with OPCODE structures
 ***/

OPCODE *avr_new_opcode(void) {
  return (OPCODE *) cfg_malloc("avr_new_opcode()", sizeof(OPCODE));
}

static OPCODE *avr_dup_opcode(const OPCODE *op) {
  if(op == NULL)                // Caller wants NULL if op == NULL
    return NULL;

  OPCODE *m = (OPCODE *) cfg_malloc("avr_dup_opcode()", sizeof(*m));
  memcpy(m, op, sizeof(*m));

  return m;
}

void avr_free_opcode(OPCODE *op) {
  if(op)
    free(op);
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


static char * avr_op_str(int op)
{
  switch (op) {
    case AVR_OP_READ        : return "READ"; break;
    case AVR_OP_WRITE       : return "WRITE"; break;
    case AVR_OP_READ_LO     : return "READ_LO"; break;
    case AVR_OP_READ_HI     : return "READ_HI"; break;
    case AVR_OP_WRITE_LO    : return "WRITE_LO"; break;
    case AVR_OP_WRITE_HI    : return "WRITE_HI"; break;
    case AVR_OP_LOADPAGE_LO : return "LOADPAGE_LO"; break;
    case AVR_OP_LOADPAGE_HI : return "LOADPAGE_HI"; break;
    case AVR_OP_LOAD_EXT_ADDR : return "LOAD_EXT_ADDR"; break;
    case AVR_OP_WRITEPAGE   : return "WRITEPAGE"; break;
    case AVR_OP_CHIP_ERASE  : return "CHIP_ERASE"; break;
    case AVR_OP_PGM_ENABLE  : return "PGM_ENABLE"; break;
    default : return "<unknown opcode>"; break;
  }
}


static char * bittype(int type)
{
  switch (type) {
    case AVR_CMDBIT_IGNORE  : return "IGNORE"; break;
    case AVR_CMDBIT_VALUE   : return "VALUE"; break;
    case AVR_CMDBIT_ADDRESS : return "ADDRESS"; break;
    case AVR_CMDBIT_INPUT   : return "INPUT"; break;
    case AVR_CMDBIT_OUTPUT  : return "OUTPUT"; break;
    default : return "<unknown bit type>"; break;
  }
}


/***
 *** Elementary functions dealing with AVRMEM structures
 ***/

AVRMEM *avr_new_mem(void) {
  AVRMEM *m = (AVRMEM *) cfg_malloc("avr_new_mem()", sizeof(*m));
  m->desc = cache_string("");
  m->page_size = 1;             // Ensure not 0
  m->initval = -1;              // Unknown value represented as -1
  m->bitmask = -1;              // Default to -1

  return m;
}

AVRMEM_ALIAS *avr_new_memalias(void) {
  AVRMEM_ALIAS *m = (AVRMEM_ALIAS *) cfg_malloc("avr_new_memalias()", sizeof*m);
  m->desc = cache_string("");
  return m;
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
    m->buf  = (unsigned char *) cfg_malloc("avr_initmem()", m->size);
    m->tags = (unsigned char *) cfg_malloc("avr_initmem()", m->size);
  }

  return 0;
}


AVRMEM *avr_dup_mem(const AVRMEM *m) {
  AVRMEM *n = avr_new_mem();

  if(m) {
    *n = *m;

    if(m->buf) {
      n->buf = (unsigned char *) cfg_malloc("avr_dup_mem()", n->size);
      memcpy(n->buf, m->buf, n->size);
    }

    if(m->tags) {
      n->tags = (unsigned char *) cfg_malloc("avr_dup_mem()", n->size);
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
    free(m->buf);
    m->buf = NULL;
  }
  if(m->tags) {
    free(m->tags);
    m->tags = NULL;
  }
  for(size_t i=0; i<sizeof(m->op)/sizeof(m->op[0]); i++) {
    if(m->op[i]) {
      avr_free_opcode(m->op[i]);
      m->op[i] = NULL;
    }
  }
  free(m);
}

void avr_free_memalias(AVRMEM_ALIAS *m) {
  if(m)
    free(m);
}

AVRMEM_ALIAS *avr_locate_memalias(const AVRPART *p, const char *desc) {
  AVRMEM_ALIAS * m, * match;
  LNODEID ln;
  int matches;
  size_t l;

  if(!p || !desc || !p->mem_alias)
    return NULL;

  l = strlen(desc);
  matches = 0;
  match = NULL;
  for (ln=lfirst(p->mem_alias); ln; ln=lnext(ln)) {
    m = ldata(ln);
    if(l && str_starts(m->desc, desc)) { // Partial initial match
      match = m;
      matches++;
      if(m->desc[l] == 0)       // Exact match; return straight away
        return m;
    }
  }

  return matches == 1? match: NULL;
}

AVRMEM *avr_locate_mem_noalias(const AVRPART *p, const char *desc) {
  AVRMEM * m, * match;
  LNODEID ln;
  int matches;
  size_t l;

  if(!p || !desc || !p->mem)
    return NULL;

  l = strlen(desc);
  matches = 0;
  match = NULL;
  for (ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    m = ldata(ln);
    if(l && str_starts(m->desc, desc)) { // Partial initial match
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

AVRMEM_ALIAS *avr_find_memalias(const AVRPART *p, const AVRMEM *m_orig) {
  if(p && p->mem_alias && m_orig)
    for(LNODEID ln=lfirst(p->mem_alias); ln; ln=lnext(ln)) {
      AVRMEM_ALIAS *m = ldata(ln);
      if(m->aliased_mem == m_orig)
        return m;
    }

  return NULL;
}


void avr_mem_display(const char *prefix, FILE *f, const AVRMEM *m,
                     const AVRPART *p, int verbose) {
  static unsigned int prev_mem_offset;
  static int prev_mem_size;
  int i, j;
  char * optr;

  if (m == NULL || verbose > 2) {
      fprintf(f,
              "%s                                Block Poll               Page                       Polled\n"
              "%sMemory Type Alias    Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack\n"
              "%s----------- -------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------\n",
            prefix, prefix, prefix);
  }

  if (m != NULL) {
    // Only print memory section if the previous section printed isn't identical
    if(prev_mem_offset != m->offset || prev_mem_size != m->size || str_eq(p->family_id, "")) {
      prev_mem_offset = m->offset;
      prev_mem_size = m->size;
      AVRMEM_ALIAS *ap = avr_find_memalias(p, m);
      /* Show alias if the current and the next memory section has the same offset
      and size, we're not out of band and a family_id is present */
      const char *mem_desc_alias = ap? ap->desc: "";
      fprintf(f,
              "%s%-11s %-8s %4d %5d %5d %4d %-6s %6d %4d %6d %5d %5d 0x%02x 0x%02x\n",
              prefix,
              m->desc,
              mem_desc_alias,
              m->mode, m->delay, m->blocksize, m->pollindex,
              m->paged ? "yes" : "no",
              m->size,
              m->page_size,
              m->num_pages,
              m->min_write_delay,
              m->max_write_delay,
              m->readback[0],
              m->readback[1]);
    }
    if (verbose > 4) {
      msg_trace2("%s  Memory Ops:\n"
                      "%s    Operation    Inst Bit  Bit Type  Bitno  Value\n"
                      "%s    -----------  --------  --------  -----  -----\n",
                      prefix, prefix, prefix);
      for (i=0; i<AVR_OP_MAX; i++) {
        if (m->op[i]) {
          for (j=31; j>=0; j--) {
            if (j==31)
              optr = avr_op_str(i);
            else
              optr = " ";
          fprintf(f,
                  "%s    %-11s  %8d  %8s  %5d  %5d\n",
                  prefix, optr, j,
                  bittype(m->op[i]->bit[j].type),
                  m->op[i]->bit[j].bitno,
                  m->op[i]->bit[j].value);
          }
        }
      }
    }
  }
}


/*
 * Elementary functions dealing with AVRPART structures
 */

AVRPART *avr_new_part(void) {
  AVRPART *p = (AVRPART *) cfg_malloc("avr_new_part()", sizeof(AVRPART));
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

void avr_free_part(AVRPART * d)
{
  ldestroy_cb(d->mem, (void(*)(void *))avr_free_mem);
  d->mem = NULL;
  ldestroy_cb(d->mem_alias, (void(*)(void *))avr_free_memalias);
  d->mem_alias = NULL;
  ldestroy_cb(d->variants, free);
  d->variants = NULL;

  /* do not free d->parent_id and d->config_file */
  for(size_t i=0; i<sizeof(d->op)/sizeof(d->op[0]); i++) {
    if (d->op[i] != NULL) {
      avr_free_opcode(d->op[i]);
      d->op[i] = NULL;
    }
  }
  free(d);
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

AVRPART *locate_part_by_signature(const LISTID parts, unsigned char *sig, int sigsize) {
  if(parts && sigsize == 3)
    for(LNODEID ln1=lfirst(parts); ln1; ln1=lnext(ln1)) {
      AVRPART *p = ldata(ln1);
      int i;
      for(i=0; i<3; i++)
        if(p->signature[i] != sig[i])
          break;
      if(i == 3)
        return p;
    }

  return NULL;
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
 * Compare function to sort the list of programmers
 */
static int sort_avrparts_compare(const AVRPART *p1, const AVRPART *p2) {
  if(p1 == NULL || p1->desc == NULL || p2 == NULL || p2->desc == NULL)
    return 0;

  return strcasecmp(p1->desc, p2->desc);
}

/*
 * Sort the list of programmers given as "programmers"
 */
void sort_avrparts(LISTID avrparts)
{
  lsort(avrparts,(int (*)(void*, void*)) sort_avrparts_compare);
}


static char * reset_disp_str(int r)
{
  switch (r) {
    case RESET_DEDICATED : return "dedicated";
    case RESET_IO        : return "possible i/o";
    default              : return "<invalid>";
  }
}


void avr_display(FILE *f, const AVRPART *p, const char *prefix, int verbose) {
  char * buf;
  const char * px;
  LNODEID ln;
  AVRMEM * m;

  fprintf(  f, "%sAVR Part                      : %s\n", prefix, p->desc);
  if (p->chip_erase_delay)
    fprintf(f, "%sChip Erase delay              : %d us\n", prefix, p->chip_erase_delay);
  if (p->pagel)
    fprintf(f, "%sPAGEL                         : P%02X\n", prefix, p->pagel);
  if (p->bs2)
    fprintf(f, "%sBS2                           : P%02X\n", prefix, p->bs2);
  fprintf(  f, "%sRESET disposition             : %s\n", prefix, reset_disp_str(p->reset_disposition));
  fprintf(  f, "%sRETRY pulse                   : %s\n", prefix, avr_pin_name(p->retry_pulse));
  fprintf(  f, "%sSerial program mode           : %s\n", prefix, (p->flags & AVRPART_SERIALOK) ? "yes" : "no");
  fprintf(  f, "%sParallel program mode         : %s\n", prefix, (p->flags & AVRPART_PARALLELOK) ?
         ((p->flags & AVRPART_PSEUDOPARALLEL) ? "pseudo" : "yes") : "no");
  if(p->timeout)
    fprintf(f, "%sTimeout                       : %d\n", prefix, p->timeout);
  if(p->stabdelay)
    fprintf(f, "%sStabDelay                     : %d\n", prefix, p->stabdelay);
  if(p->cmdexedelay)
    fprintf(f, "%sCmdexeDelay                   : %d\n", prefix, p->cmdexedelay);
  if(p->synchloops)
    fprintf(f, "%sSyncLoops                     : %d\n", prefix, p->synchloops);
  if(p->bytedelay)
    fprintf(f, "%sByteDelay                     : %d\n", prefix, p->bytedelay);
  if(p->pollindex)
    fprintf(f, "%sPollIndex                     : %d\n", prefix, p->pollindex);
  if(p->pollvalue)
    fprintf(f, "%sPollValue                     : 0x%02x\n", prefix, p->pollvalue);
  fprintf(  f, "%sMemory Detail                 :\n\n", prefix);

  px = prefix;
  buf = (char *) cfg_malloc("avr_display()", strlen(prefix) + 5);
  strcpy(buf, prefix);
  strcat(buf, "  ");
  px = buf;

  if (verbose <= 2)
    avr_mem_display(px, f, NULL, p, verbose);

  for (ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    m = ldata(ln);
    avr_mem_display(px, f, m, p, verbose);
  }

  if (buf)
    free(buf);
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

  return cfg_strdup("cmdbitstr()", space);
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
    return cfg_strdup("opcode2str()", "NULL");

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

  return cfg_strdup("opcode2str()", space);
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
