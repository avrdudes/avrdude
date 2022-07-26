
/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
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

#include "ac_cfg.h"
#include "avrdude.h"
#include "libavrdude.h"

/***
 *** Elementary functions dealing with OPCODE structures
 ***/

OPCODE * avr_new_opcode(void)
{
  OPCODE * m;

  m = (OPCODE *)malloc(sizeof(*m));
  if (m == NULL) {
    avrdude_message(MSG_INFO, "avr_new_opcode(): out of memory\n");
    exit(1);
  }

  memset(m, 0, sizeof(*m));

  return m;
}

static OPCODE * avr_dup_opcode(OPCODE * op)
{
  OPCODE * m;
  
  /* this makes life easier */
  if (op == NULL) {
    return NULL;
  }

  m = (OPCODE *)malloc(sizeof(*m));
  if (m == NULL) {
    avrdude_message(MSG_INFO, "avr_dup_opcode(): out of memory\n");
    exit(1);
  }

  memcpy(m, op, sizeof(*m));

  return m;
}

void avr_free_opcode(OPCODE * op)
{
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
int avr_set_bits(OPCODE * op, unsigned char * cmd)
{
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
int avr_set_addr(OPCODE * op, unsigned char * cmd, unsigned long addr)
{
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
int avr_set_addr_mem(AVRMEM *mem, int opnum, unsigned char *cmd, unsigned long addr) {
  int ret, isflash, lo, hi, memsize, pagesize;
  OPCODE *op;

  if(!mem)
    return -1;

  if(!(op = mem->op[opnum]))
    return -1;

  isflash = !strcmp(mem->desc, "flash"); // ISP parts have only one flash-like memory
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
int avr_set_input(OPCODE * op, unsigned char * cmd, unsigned char data)
{
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
 * Retreive output data bits from the command results based on the
 * opcode data.
 */
int avr_get_output(OPCODE * op, unsigned char * res, unsigned char * data)
{
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
int avr_get_output_index(OPCODE * op)
{
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

AVRMEM * avr_new_memtype(void)
{
  AVRMEM * m;

  m = (AVRMEM *)malloc(sizeof(*m));
  if (m == NULL) {
    avrdude_message(MSG_INFO, "avr_new_memtype(): out of memory\n");
    exit(1);
  }

  memset(m, 0, sizeof(*m));
  m->page_size = 1; // ensure not 0

  return m;
}

AVRMEM_ALIAS * avr_new_memalias(void)
{
  AVRMEM_ALIAS * m;

  m = (AVRMEM_ALIAS *)malloc(sizeof(*m));
  if (m == NULL) {
    avrdude_message(MSG_INFO, "avr_new_memalias(): out of memory\n");
    exit(1);
  }

  memset(m, 0, sizeof(*m));

  return m;
}


/*
 * Allocate and initialize memory buffers for each of the device's
 * defined memory regions.
 */
int avr_initmem(AVRPART * p)
{
  LNODEID ln;
  AVRMEM * m;

  for (ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    m = ldata(ln);
    m->buf = (unsigned char *) malloc(m->size);
    if (m->buf == NULL) {
      avrdude_message(MSG_INFO, "%s: can't alloc buffer for %s size of %d bytes\n",
              progname, m->desc, m->size);
      return -1;
    }
    m->tags = (unsigned char *) malloc(m->size);
    if (m->tags == NULL) {
      avrdude_message(MSG_INFO, "%s: can't alloc buffer for %s size of %d bytes\n",
              progname, m->desc, m->size);
      return -1;
    }
  }

  return 0;
}


AVRMEM * avr_dup_mem(AVRMEM * m)
{
  AVRMEM * n;
  int i;

  n = avr_new_memtype();

  *n = *m;

  if (m->buf != NULL) {
    n->buf = (unsigned char *)malloc(n->size);
    if (n->buf == NULL) {
      avrdude_message(MSG_INFO, "avr_dup_mem(): out of memory (memsize=%d)\n",
                      n->size);
      exit(1);
    }
    memcpy(n->buf, m->buf, n->size);
  }

  if (m->tags != NULL) {
    n->tags = (unsigned char *)malloc(n->size);
    if (n->tags == NULL) {
      avrdude_message(MSG_INFO, "avr_dup_mem(): out of memory (memsize=%d)\n",
                      n->size);
      exit(1);
    }
    memcpy(n->tags, m->tags, n->size);
  }

  for (i = 0; i < AVR_OP_MAX; i++) {
    n->op[i] = avr_dup_opcode(n->op[i]);
  }

  return n;
}

AVRMEM_ALIAS * avr_dup_memalias(AVRMEM_ALIAS * m)
{
  AVRMEM_ALIAS * n;

  n = avr_new_memalias();

  *n = *m;

  return n;
}

void avr_free_mem(AVRMEM * m)
{
    if (m->buf != NULL) {
      free(m->buf);
      m->buf = NULL;
    }
    if (m->tags != NULL) {
      free(m->tags);
      m->tags = NULL;
    }
    for(size_t i=0; i<sizeof(m->op)/sizeof(m->op[0]); i++)
    {
      if (m->op[i] != NULL)
      {
        avr_free_opcode(m->op[i]);
        m->op[i] = NULL;
      }
    }
    free(m);
}

void avr_free_memalias(AVRMEM_ALIAS * m)
{
  free(m);
}

AVRMEM_ALIAS * avr_locate_memalias(AVRPART * p, const char * desc)
{
  AVRMEM_ALIAS * m, * match;
  LNODEID ln;
  int matches;
  int l;

  if(!p || !desc || !p->mem_alias)
    return NULL;

  l = strlen(desc);
  matches = 0;
  match = NULL;
  for (ln=lfirst(p->mem_alias); ln; ln=lnext(ln)) {
    m = ldata(ln);
    if (strncmp(desc, m->desc, l) == 0) {
      match = m;
      matches++;
    }
  }

  if (matches == 1)
    return match;

  return NULL;
}

AVRMEM * avr_locate_mem_noalias(AVRPART * p, const char * desc)
{
  AVRMEM * m, * match;
  LNODEID ln;
  int matches;
  int l;

  if(!p || !desc || !p->mem)
    return NULL;

  l = strlen(desc);
  matches = 0;
  match = NULL;
  for (ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    m = ldata(ln);
    if (strncmp(desc, m->desc, l) == 0) {
      match = m;
      matches++;
    }
  }

  if (matches == 1)
    return match;

  return NULL;
}


AVRMEM * avr_locate_mem(AVRPART * p, const char * desc)
{
  AVRMEM * m, * match;
  AVRMEM_ALIAS * alias;
  LNODEID ln;
  int matches;
  int l;

  if(!p || !desc)
    return NULL;

  l = strlen(desc);
  matches = 0;
  match = NULL;
  if(p->mem) {
    for (ln=lfirst(p->mem); ln; ln=lnext(ln)) {
      m = ldata(ln);
      if (strncmp(desc, m->desc, l) == 0) {
        match = m;
        matches++;
      }
    }
  }

  if (matches == 1)
    return match;

  /* not yet found: look for matching alias name */
  alias = avr_locate_memalias(p, desc);
  if (alias != NULL)
    return alias->aliased_mem;

  return NULL;
}

AVRMEM_ALIAS * avr_find_memalias(AVRPART * p, AVRMEM * m_orig)
{
  AVRMEM_ALIAS * m;
  LNODEID ln;

  for (ln=lfirst(p->mem_alias); ln; ln=lnext(ln)) {
    m = ldata(ln);
    if (m->aliased_mem == m_orig)
      return m;
  }

  return NULL;
}


void avr_mem_display(const char * prefix, FILE * f, AVRMEM * m, AVRPART * p,
                     int type, int verbose)
{
  static unsigned int prev_mem_offset;
  static int prev_mem_size;
  int i, j;
  char * optr;

  if (m == NULL) {
      fprintf(f,
              "%s                                Block Poll               Page                       Polled\n"
              "%sMemory Type Alias    Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack\n"
              "%s----------- -------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------\n",
            prefix, prefix, prefix);
  }
  else {
    if (verbose > 2) {
      fprintf(f,
              "%s                                Block Poll               Page                       Polled\n"
              "%sMemory Type Alias    Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack\n"
              "%s----------- -------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------\n",
              prefix, prefix, prefix);
    }

    // Only print memory section if the previous section printed isn't identical
    if(prev_mem_offset != m->offset || prev_mem_size != m->size || (strcmp(p->family_id, "") == 0)) {
      prev_mem_offset = m->offset;
      prev_mem_size = m->size;
      AVRMEM_ALIAS *ap = avr_find_memalias(p, m);
      /* Show alias if the current and the next memory section has the same offset
      and size, we're not out of band and a family_id is present */
      char * mem_desc_alias = ap? ap->desc: "";
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
      avrdude_message(MSG_TRACE2, "%s  Memory Ops:\n"
                      "%s    Oeration     Inst Bit  Bit Type  Bitno  Value\n"
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

AVRPART * avr_new_part(void)
{
  AVRPART * p;

  p = (AVRPART *)malloc(sizeof(AVRPART));
  if (p == NULL) {
    avrdude_message(MSG_INFO, "new_part(): out of memory\n");
    exit(1);
  }

  memset(p, 0, sizeof(*p));

  p->id[0]   = 0;
  p->desc[0] = 0;
  p->reset_disposition = RESET_DEDICATED;
  p->retry_pulse = PIN_AVR_SCK;
  p->flags = AVRPART_SERIALOK | AVRPART_PARALLELOK | AVRPART_ENABLEPAGEPROGRAMMING;
  p->parent_id = NULL;
  p->config_file = NULL;
  p->lineno = 0;
  memset(p->signature, 0xFF, 3);
  p->ctl_stack_type = CTL_STACK_NONE;
  p->ocdrev = -1;
  p->hvupdi_variant = -1;

  p->mem = lcreat(NULL, 0);
  p->mem_alias = lcreat(NULL, 0);

  return p;
}


AVRPART * avr_dup_part(AVRPART * d)
{
  AVRPART * p;
  LISTID save, save2;
  LNODEID ln, ln2;
  int i;

  p = avr_new_part();
  save = p->mem;
  save2 = p->mem_alias;

  *p = *d;

  p->mem = save;
  p->mem_alias = save2;
  for (ln=lfirst(d->mem); ln; ln=lnext(ln)) {
    AVRMEM *m = ldata(ln);
    AVRMEM *m2 = avr_dup_mem(m);
    ladd(p->mem, m2);
    // see if there is any alias for it
    for (ln2=lfirst(d->mem_alias); ln2; ln2=lnext(ln2)) {
      AVRMEM_ALIAS *a = ldata(ln2);
      if (a->aliased_mem == m) {
        // yes, duplicate it
        AVRMEM_ALIAS *a2 = avr_dup_memalias(a);
        // ... adjust the pointer ...
        a2->aliased_mem = m2;
        // ... and add to new list
        ladd(p->mem_alias, a2);
      }
    }
  }

  for (i = 0; i < AVR_OP_MAX; i++) {
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
  /* do not free d->parent_id and d->config_file */
  for(size_t i=0; i<sizeof(d->op)/sizeof(d->op[0]); i++) {
    if (d->op[i] != NULL) {
      avr_free_opcode(d->op[i]);
      d->op[i] = NULL;
    }
  }
  free(d);
}

AVRPART * locate_part(LISTID parts, const char * partdesc)
{
  LNODEID ln1;
  AVRPART * p = NULL;
  int found;

  if(!parts || !partdesc)
    return NULL;

  found = 0;

  for (ln1=lfirst(parts); ln1 && !found; ln1=lnext(ln1)) {
    p = ldata(ln1);
    if ((strcasecmp(partdesc, p->id) == 0) ||
        (strcasecmp(partdesc, p->desc) == 0))
      found = 1;
  }

  if (found)
    return p;

  return NULL;
}

AVRPART * locate_part_by_avr910_devcode(LISTID parts, int devcode)
{
  LNODEID ln1;
  AVRPART * p = NULL;

  for (ln1=lfirst(parts); ln1; ln1=lnext(ln1)) {
    p = ldata(ln1);
    if (p->avr910_devcode == devcode)
      return p;
  }

  return NULL;
}

AVRPART * locate_part_by_signature(LISTID parts, unsigned char * sig,
                                   int sigsize)
{
  LNODEID ln1;
  AVRPART * p = NULL;
  int i;

  if (sigsize == 3) {
    for (ln1=lfirst(parts); ln1; ln1=lnext(ln1)) {
      p = ldata(ln1);
      for (i=0; i<3; i++)
        if (p->signature[i] != sig[i])
          break;
      if (i == 3)
        return p;
    }
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
static int sort_avrparts_compare(AVRPART * p1,AVRPART * p2)
{
  if(p1 == NULL || p2 == NULL) {
    return 0;
  }
  return strncasecmp(p1->desc,p2->desc,AVR_DESCLEN);
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


void avr_display(FILE * f, AVRPART * p, const char * prefix, int verbose)
{
  int i;
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
  i = strlen(prefix) + 5;
  buf = (char *)malloc(i);
  if (buf == NULL) {
    /* ugh, this is not important enough to bail, just ignore it */
  }
  else {
    strcpy(buf, prefix);
    strcat(buf, "  ");
    px = buf;
  }

  if (verbose <= 2) {
    avr_mem_display(px, f, NULL, p, 0, verbose);
  }
  for (ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    m = ldata(ln);
    avr_mem_display(px, f, m, p, i, verbose);
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

  return strdup(space);
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
char *opcode2str(OPCODE *op, int opnum, int detailed) {
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


/*
 * Match STRING against the partname pattern PATTERN, returning 1 if it
 * matches, 0 if not. NOTE: part_match() is a modified old copy of !fnmatch()
 * from the GNU C Library (published under GLP v2). Used for portability.
 */

inline static int fold(int c) {
  return (c >= 'A' && c <= 'Z')? c+('a'-'A'): c;
}

int part_match(const char *pattern, const char *string) {
  unsigned char c;
  const char *p = pattern, *n = string;

  if(!*n)                       // AVRDUDE specialty: empty string never matches
    return 0;

  while((c = fold(*p++))) {
    switch(c) {
    case '?':
      if(*n == 0)
        return 0;
      break;

    case '\\':
      c = fold(*p++);
      if(fold(*n) != c)
        return 0;
      break;

    case '*':
      for(c = *p++; c == '?' || c == '*'; c = *p++)
        if(c == '?' && *n++ == 0)
          return 0;

      if(c == 0)
        return 1;

      {
        unsigned char c1 = fold(c == '\\'? *p : c); // This char

        for(--p; *n; ++n)       // Recursively check reminder of string for *
          if((c == '[' || fold(*n) == c1) && part_match(p, n) == 1)
            return 1;
        return 0;
      }

    case '[':
      {
        int negate;

        if(*n == 0)
          return 0;

        negate = (*p == '!' || *p == '^');
        if(negate)
          ++p;

        c = *p++;
        for(;;) {
          unsigned char cstart = c, cend = c;

          if(c == '\\')
            cstart = cend = *p++;

          cstart = cend = fold(cstart);

          if(c == 0)            // [ (unterminated)
            return 0;

          c = *p++;
          c = fold(c);

          if(c == '-' && *p != ']') {
            cend = *p++;
            if(cend == '\\')
              cend = *p++;
            if(cend == 0)
              return 0;
            cend = fold(cend);

            c = *p++;
          }

          if(fold(*n) >= cstart && fold(*n) <= cend)
            goto matched;

          if(c == ']')
            break;
        }
        if(!negate)
          return 0;
        break;

      matched:;
        while(c != ']') {       // Skip the rest of the [...] that already matched

          if(c == 0)            // [... (unterminated)
            return 0;

          c = *p++;
          if(c == '\\')         // XXX 1003.2d11 is unclear if this is right
            ++p;
        }
        if(negate)
          return 0;
      }
      break;

    default:
      if(c != fold(*n))
        return 0;
    }

    ++n;
  }

  return *n == 0;
}
