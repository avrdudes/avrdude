
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */

#include <stdlib.h>
#include <string.h>

#include "avrdude.h"
#include "avrpart.h"
#include "pindefs.h"

/***
 *** Elementary functions dealing with OPCODE structures
 ***/

OPCODE * avr_new_opcode(void)
{
  OPCODE * m;

  m = (OPCODE *)malloc(sizeof(*m));
  if (m == NULL) {
    fprintf(stderr, "avr_new_opcode(): out of memory\n");
    exit(1);
  }

  memset(m, 0, sizeof(*m));

  return m;
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
    if (op->bit[i].type == AVR_CMDBIT_VALUE) {
      j = 3 - i / 8;
      bit = i % 8;
      mask = 1 << bit;
      if (op->bit[i].value)
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


char * avr_op_str(int op)
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


char * bittype(int type)
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
    fprintf(stderr, "avr_new_memtype(): out of memory\n");
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
      fprintf(stderr, "%s: can't alloc buffer for %s size of %d bytes\n",
              progname, m->desc, m->size);
      return -1;
    }
  }

  return 0;
}


AVRMEM * avr_dup_mem(AVRMEM * m)
{
  AVRMEM * n;

  n = avr_new_memtype();

  *n = *m;

  n->buf = (unsigned char *)malloc(n->size);
  if (n->buf == NULL) {
    fprintf(stderr,
            "avr_dup_mem(): out of memory (memsize=%d)\n",
            n->size);
    exit(1);
  }
  memset(n->buf, 0, n->size);

  return n;
}


AVRMEM * avr_locate_mem(AVRPART * p, char * desc)
{
  AVRMEM * m, * match;
  LNODEID ln;
  int matches;
  int l;

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


void avr_mem_display(char * prefix, FILE * f, AVRMEM * m, int type,
                     int verbose)
{
  int i, j;
  char * optr;

  if (m == NULL) {
      fprintf(f,
              "%s                       Block Poll               Page                       Polled\n"
              "%sMemory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack\n"
              "%s----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------\n",
            prefix, prefix, prefix);
  }
  else {
    if (verbose > 2) {
      fprintf(f,
              "%s                       Block Poll               Page                       Polled\n"
              "%sMemory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack\n"
              "%s----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------\n",
              prefix, prefix, prefix);
    }
    fprintf(f,
            "%s%-11s %4d %5d %5d %4d %-6s %6d %4d %6d %5d %5d 0x%02x 0x%02x\n",
            prefix, m->desc, m->mode, m->delay, m->blocksize, m->pollindex,
            m->paged ? "yes" : "no",
            m->size,
            m->page_size,
            m->num_pages,
            m->min_write_delay,
            m->max_write_delay,
            m->readback[0],
            m->readback[1]);
    if (verbose > 4) {
      fprintf(stderr,
              "%s  Memory Ops:\n"
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
    fprintf(stderr, "new_part(): out of memory\n");
    exit(1);
  }

  memset(p, 0, sizeof(*p));

  p->id[0]   = 0;
  p->desc[0] = 0;
  p->reset_disposition = RESET_DEDICATED;
  p->retry_pulse = PIN_AVR_SCK;
  p->flags = AVRPART_SERIALOK | AVRPART_PARALLELOK | AVRPART_ENABLEPAGEPROGRAMMING;
  p->config_file[0] = 0;
  p->lineno = 0;
  memset(p->signature, 0xFF, 3);
  p->ctl_stack_type = CTL_STACK_NONE;

  p->mem = lcreat(NULL, 0);

  return p;
}


AVRPART * avr_dup_part(AVRPART * d)
{
  AVRPART * p;
  LISTID save;
  LNODEID ln;

  p = avr_new_part();
  save = p->mem;

  *p = *d;

  p->mem = save;

  for (ln=lfirst(d->mem); ln; ln=lnext(ln)) {
    ladd(p->mem, avr_dup_mem(ldata(ln)));
  }

  return p;
}


AVRPART * locate_part(LISTID parts, char * partdesc)
{
  LNODEID ln1;
  AVRPART * p = NULL;
  int found;

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

void list_parts(FILE * f, char * prefix, LISTID parts)
{
  LNODEID ln1;
  AVRPART * p;

  for (ln1=lfirst(parts); ln1; ln1=lnext(ln1)) {
    p = ldata(ln1);
    fprintf(f, "%s%-4s = %-15s [%s:%d]\n",
            prefix, p->id, p->desc, p->config_file, p->lineno);
  }

  return;
}


char * reset_disp_str(int r)
{
  switch (r) {
    case RESET_DEDICATED : return "dedicated";
    case RESET_IO        : return "possible i/o";
    default              : return "<invalid>";
  }
}


char * pin_name(int pinno)
{
  switch (pinno) {
    case PIN_AVR_RESET : return "RESET";
    case PIN_AVR_MISO  : return "MISO";
    case PIN_AVR_MOSI  : return "MOSI";
    case PIN_AVR_SCK   : return "SCK";
    default : return "<unknown>";
  }
}


void avr_display(FILE * f, AVRPART * p, char * prefix, int verbose)
{
  int i;
  char * buf;
  char * px;
  LNODEID ln;
  AVRMEM * m;

  fprintf(f,
          "%sAVR Part              : %s\n"
          "%sChip Erase delay      : %d us\n"
          "%sPAGEL                 : P%02X\n"
          "%sBS2                   : P%02X\n"
          "%sRESET disposition     : %s\n"
          "%sRETRY pulse           : %s\n"
          "%sserial program mode   : %s\n"
          "%sparallel program mode : %s\n"
          "%sTimeout               : %d\n"
          "%sStabDelay             : %d\n"
          "%sCmdexeDelay           : %d\n"
          "%sSyncLoops             : %d\n"
          "%sByteDelay             : %d\n"
          "%sPollIndex             : %d\n"
          "%sPollValue             : 0x%02x\n"
          "%sMemory Detail         :\n\n",
          prefix, p->desc,
          prefix, p->chip_erase_delay,
          prefix, p->pagel,
          prefix, p->bs2,
          prefix, reset_disp_str(p->reset_disposition),
          prefix, pin_name(p->retry_pulse),
          prefix, (p->flags & AVRPART_SERIALOK) ? "yes" : "no",
          prefix, (p->flags & AVRPART_PARALLELOK) ?
            ((p->flags & AVRPART_PSEUDOPARALLEL) ? "psuedo" : "yes") : "no",
          prefix, p->timeout,
          prefix, p->stabdelay,
          prefix, p->cmdexedelay,
          prefix, p->synchloops,
          prefix, p->bytedelay,
          prefix, p->pollindex,
          prefix, p->pollvalue,
          prefix);

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
    avr_mem_display(px, f, NULL, 0, verbose);
  }
  for (ln=lfirst(p->mem); ln; ln=lnext(ln)) {
    m = ldata(ln);
    avr_mem_display(px, f, m, i, verbose);
  }

  if (buf)
    free(buf);
}
