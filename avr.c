/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


#include "avr.h"
#include "config.h"
#include "lists.h"
#include "pindefs.h"
#include "ppi.h"

#define DEBUG 0

extern char       * progname;
extern char         progbuf[];
extern PROGRAMMER * pgm;


extern int do_cycles;


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
  p->flags = AVRPART_SERIALOK | AVRPART_PARALLELOK;
  p->config_file[0] = 0;
  p->lineno = 0;

  p->mem = lcreat(NULL, 0);

  return p;
}



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


/*
 * read a byte of data from the indicated memory region
 */
int avr_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem, 
                  unsigned long addr, unsigned char * value)
{
  unsigned char cmd[4];
  unsigned char res[4];
  unsigned char data;
  OPCODE * readop;

  pgm->pgm_led(pgm, ON);
  pgm->err_led(pgm, OFF);

  /*
   * figure out what opcode to use
   */
  if (mem->op[AVR_OP_READ_LO]) {
    if (addr & 0x00000001)
      readop = mem->op[AVR_OP_READ_HI];
    else
      readop = mem->op[AVR_OP_READ_LO];
    addr = addr / 2;
  }
  else {
    readop = mem->op[AVR_OP_READ];
  }

  if (readop == NULL) {
#if DEBUG
    fprintf(stderr, 
            "avr_read_byte(): operation not supported on memory type \"%s\"\n",
            p->desc);
#endif
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(readop, cmd);
  avr_set_addr(readop, cmd, addr);
  pgm->cmd(pgm, cmd, res);
  data = 0;
  avr_get_output(readop, res, &data);

  pgm->pgm_led(pgm, OFF);

  *value = data;

  return 0;
}


/*
 * Read the entirety of the specified memory type into the
 * corresponding buffer of the avrpart pointed to by 'p'.  If size =
 * 0, read the entire contents, otherwise, read 'size' bytes.
 *
 * Return the number of bytes read, or < 0 if an error occurs.  
 */
int avr_read(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size, 
             int verbose)
{
  unsigned char    rbyte;
  unsigned long    i;
  unsigned char  * buf;
  AVRMEM * mem;
  int rc;
  int printed;

  mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    fprintf(stderr, "No \"%s\" memory for part %s\n",
            memtype, p->desc);
    return -1;
  }

  buf  = mem->buf;
  if (size == 0) {
    size = mem->size;
  }

  if ((strcmp(mem->desc, "flash")==0) || (strcmp(mem->desc, "eeprom")==0)) {
    if (pgm->paged_load != NULL) {
      /*
       * the programmer supports a paged mode read, perhaps more
       * efficiently than we can read it directly, so use its routine
       * instead
       */
      if (mem->paged) {
        return pgm->paged_load(pgm, p, mem, mem->page_size, size);
      }
      else {
        return pgm->paged_load(pgm, p, mem, pgm->page_size, size);
      }
    }
  }


  printed = 0;

  for (i=0; i<size; i++) {
    rc = avr_read_byte(pgm, p, mem, i, &rbyte);
    if (rc != 0) {
      fprintf(stderr, "avr_read(): error reading address 0x%04lx\n", i);
      if (rc == -1) 
        fprintf(stderr, 
                "    read operation not supported for memory \"%s\"\n",
                memtype);
      return -2;
    }
    buf[i] = rbyte;
    if (verbose) {
      if ((i % 16 == 0)||(i == (size-1))) {
        printed = 1;
        fprintf(stderr, "\r        \r%6lu", i);
      }
    }
  }

  if (printed) {
    fprintf(stderr, "\n");
  }

  return i;
}


/*
 * write a page data at the specified address
 */
int avr_write_page(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem, 
                   unsigned long addr)
{
  unsigned char cmd[4];
  unsigned char res[4];
  OPCODE * wp;

  wp = mem->op[AVR_OP_WRITEPAGE];
  if (wp == NULL) {
    fprintf(stderr, 
            "avr_write_page(): memory \"%s\" not configured for page writes\n",
            mem->desc);
    return -1;
  }

  /*
   * if this memory is word-addressable, adjust the address
   * accordingly
   */
  if ((mem->op[AVR_OP_LOADPAGE_LO]) || (mem->op[AVR_OP_READ_LO]))
    addr = addr / 2;

  pgm->pgm_led(pgm, ON);
  pgm->err_led(pgm, OFF);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(wp, cmd);
  avr_set_addr(wp, cmd, addr);
  pgm->cmd(pgm, cmd, res);

  /*
   * since we don't know what voltage the target AVR is powered by, be
   * conservative and delay the max amount the spec says to wait 
   */
  usleep(mem->max_write_delay);

  pgm->pgm_led(pgm, OFF);
  return 0;
}


/*
 * write a byte of data at the specified address
 */
int avr_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                   unsigned long addr, unsigned char data)
{
  unsigned char cmd[4];
  unsigned char res[4];
  unsigned char r;
  int ready;
  int tries;
  unsigned char b;
  unsigned short caddr;
  OPCODE * writeop;
  int rc;
  int readok=0;

  if (!mem->paged) {
    /* 
     * check to see if the write is necessary by reading the existing
     * value and only write if we are changing the value; we can't
     * use this optimization for paged addressing.
     */
    rc = avr_read_byte(pgm, p, mem, addr, &b);
    if (rc != 0) {
      if (rc != -1) {
        return -2;
      }
      /*
       * the read operation is not support on this memory type
       */
    }
    else {
      readok = 1;
      if (b == data) {
        return 0;
      }
    }
  }

  /*
   * determine which memory opcode to use
   */
  if (mem->op[AVR_OP_WRITE_LO]) {
    if (addr & 0x01)
      writeop = mem->op[AVR_OP_WRITE_HI];
    else
      writeop = mem->op[AVR_OP_WRITE_LO];
    caddr = addr / 2;
  }
  else if (mem->op[AVR_OP_LOADPAGE_LO]) {
    if (addr & 0x01)
      writeop = mem->op[AVR_OP_LOADPAGE_HI];
    else
      writeop = mem->op[AVR_OP_LOADPAGE_LO];
    caddr = addr / 2;
  }
  else {
    writeop = mem->op[AVR_OP_WRITE];
    caddr = addr;
  }

  if (writeop == NULL) {
#if DEBUG
    fprintf(stderr, 
            "avr_write_byte(): write not supported for memory type \"%s\"\n",
            mem->desc);
#endif
    return -1;
  }


  pgm->pgm_led(pgm, ON);
  pgm->err_led(pgm, OFF);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(writeop, cmd);
  avr_set_addr(writeop, cmd, caddr);
  avr_set_input(writeop, cmd, data);
  pgm->cmd(pgm, cmd, res);

  if (mem->paged) {
    /*
     * in paged addressing, single bytes to be written to the memory
     * page complete immediately, we only need to delay when we commit
     * the whole page via the avr_write_page() routine.
     */
    pgm->pgm_led(pgm, OFF);
    return 0;
  }

  if (readok == 0) {
    /*
     * read operation not supported for this memory type, just wait
     * the max programming time and then return 
     */
    usleep(mem->max_write_delay); /* maximum write delay */
    pgm->pgm_led(pgm, OFF);
    return 0;
  }

  tries = 0;
  ready = 0;
  while (!ready) {
    usleep(mem->min_write_delay);
    rc = avr_read_byte(pgm, p, mem, addr, &r);
    if (rc != 0) {
      pgm->pgm_led(pgm, OFF);
      pgm->err_led(pgm, ON);
      return -4;
    }

    if ((data == mem->readback[0]) ||
        (data == mem->readback[1])) {
      /* 
       * use an extra long delay when we happen to be writing values
       * used for polled data read-back.  In this case, polling
       * doesn't work, and we need to delay the worst case write time
       * specified for the chip.
       */
      usleep(mem->max_write_delay);
      rc = avr_read_byte(pgm, p, mem, addr, &r);
      if (rc != 0) {
        pgm->pgm_led(pgm, OFF);
        pgm->err_led(pgm, OFF);
        return -5;
      }
    }
    
    if (r == data) {
      ready = 1;
    }
    else if (mem->pwroff_after_write) {
      /*
       * The device has been flagged as power-off after write to this
       * memory type.  The reason we don't just blindly follow the
       * flag is that the power-off advice may only apply to some
       * memory bits but not all.  We only actually power-off the
       * device if the data read back does not match what we wrote.
       */
      usleep(mem->max_write_delay); /* maximum write delay */
      pgm->pgm_led(pgm, OFF);
      fprintf(stderr,
              "%s: this device must be powered off and back on to continue\n",
              progname);
      if (pgm->pinno[PPI_AVR_VCC]) {
        fprintf(stderr, "%s: attempting to do this now ...\n", progname);
        pgm->powerdown(pgm);
        usleep(250000);
        rc = pgm->initialize(pgm, p);
        if (rc < 0) {
          fprintf(stderr, "%s: initialization failed, rc=%d\n", progname, rc);
          fprintf(stderr, 
                  "%s: can't re-initialize device after programming the "
                  "%s bits\n", progname, mem->desc);
          fprintf(stderr,
                  "%s: you must manually power-down the device and restart\n"
                  "%s:   %s to continue.\n",
                  progname, progname, progname);
          return -3;
        }
        
        fprintf(stderr, "%s: device was successfully re-initialized\n",
                progname);
        return 0;
      }
    }

    tries++;
    if (!ready && tries > 5) {
      /*
       * we wrote the data, but after waiting for what should have
       * been plenty of time, the memory cell still doesn't match what
       * we wrote.  Indicate a write error.
       */
      pgm->pgm_led(pgm, OFF);
      pgm->err_led(pgm, ON);
      
      return -6;
    }
  }

  pgm->pgm_led(pgm, OFF);
  return 0;
}


/*
 * Write the whole memory region of the specified memory from the
 * corresponding buffer of the avrpart pointed to by 'p'.  Write up to
 * 'size' bytes from the buffer.  Data is only written if the new data
 * value is different from the existing data value.  Data beyond
 * 'size' bytes is not affected.
 *
 * Return the number of bytes written, or -1 if an error occurs.
 */
int avr_write(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size, 
              int verbose)
{
  int              rc;
  int              wsize;
  unsigned long    i;
  unsigned char    data;
  int              werror;
  AVRMEM         * m;
  int              printed;

  m = avr_locate_mem(p, memtype);
  if (m == NULL) {
    fprintf(stderr, "No \"%s\" memory for part %s\n",
            memtype, p->desc);
    return -1;
  }

  pgm->err_led(pgm, OFF);

  if ((strcmp(m->desc, "flash")==0) || (strcmp(m->desc, "eeprom")==0)) {
    if (pgm->paged_write != NULL) {
      /*
       * the programmer supports a paged mode write, perhaps more
       * efficiently than we can read it directly, so use its routine
       * instead
       */
      if (m->paged) {
        return pgm->paged_write(pgm, p, m, m->page_size, size);
      }
      else {
        return pgm->paged_write(pgm, p, m, pgm->page_size, size);
      }
    }
  }

  printed = 0;
  werror  = 0;

  wsize = m->size;
  if (size < wsize) {
    wsize = size;
  }
  else if (size > wsize) {
    fprintf(stderr, 
            "%s: WARNING: %d bytes requested, but memory region is only %d bytes\n"
            "%sOnly %d bytes will actually be written\n",
            progname, size, wsize,
            progbuf, wsize);
  }

  for (i=0; i<wsize; i++) {
    data = m->buf[i];
    if (verbose) {
      if ((i % 16 == 0)||(i == (wsize-1))) {
        fprintf(stderr, "\r      \r%6lu", i);
        printed = 1;
      }
    }
    rc = avr_write_byte(pgm, p, m, i, data);
    if (rc) {
      fprintf(stderr, " ***failed;  ");
      fprintf(stderr, "\n");
      pgm->err_led(pgm, ON);
      werror = 1;
    }

    if (m->paged) {
      /*
       * check to see if it is time to flush the page with a page
       * write
       */
      if (((i % m->page_size) == m->page_size-1) ||
          (i == wsize-1)) {
        rc = avr_write_page(pgm, p, m, i);
        if (rc) {
          fprintf(stderr,
                  " *** page %ld (addresses 0x%04lx - 0x%04lx) failed "
                  "to write\n",
                  i % m->page_size, 
                  i - m->page_size + 1, i);
          fprintf(stderr, "\n");
          pgm->err_led(pgm, ON);
          werror = 1;
        }
      }
    }

    if (werror) {
      /* 
       * make sure the error led stay on if there was a previous write
       * error, otherwise it gets cleared in avr_write_byte() 
       */
      pgm->err_led(pgm, ON);
    }
  }

  if (printed)
    fprintf(stderr, "\n");

  return i;
}



/*
 * read the AVR device's signature bytes
 */
int avr_signature(PROGRAMMER * pgm, AVRPART * p)
{
  int rc;

  rc = avr_read(pgm, p, "signature", 0, 0);
  if (rc < 0) {
    fprintf(stderr, 
            "%s: error reading signature data for part \"%s\", rc=%d\n",
            progname, p->desc, rc);
    return -1;
  }

  return 0;
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


/*
 * Verify the memory buffer of p with that of v.  The byte range of v,
 * may be a subset of p.  The byte range of p should cover the whole
 * chip's memory size.
 *
 * Return the number of bytes verified, or -1 if they don't match.  
 */
int avr_verify(AVRPART * p, AVRPART * v, char * memtype, int size)
{
  int i;
  unsigned char * buf1, * buf2;
  int vsize;
  AVRMEM * a, * b;

  a = avr_locate_mem(p, memtype);
  if (a == NULL) {
    fprintf(stderr, 
            "avr_verify(): memory type \"%s\" not defined for part %s\n",
            memtype, p->desc);
    return -1;
  }

  b = avr_locate_mem(v, memtype);
  if (b == NULL) {
    fprintf(stderr, 
            "avr_verify(): memory type \"%s\" not defined for part %s\n",
            memtype, v->desc);
    return -1;
  }

  buf1  = a->buf;
  buf2  = b->buf;
  vsize = a->size;

  if (vsize < size) {
    fprintf(stderr, 
            "%s: WARNING: requested verification for %d bytes\n"
            "%s%s memory region only contains %d bytes\n"
            "%sOnly %d bytes will be verified.\n",
            progname, size,
            progbuf, memtype, vsize,
            progbuf, vsize);
    size = vsize;
  }

  for (i=0; i<size; i++) {
    if (buf1[i] != buf2[i]) {
      fprintf(stderr, 
              "%s: verification error, first mismatch at byte 0x%04x\n"
              "%s0x%02x != 0x%02x\n",
              progname, i, 
              progbuf, buf1[i], buf2[i]);
      return -1;
    }
  }

  return size;
}


int avr_get_cycle_count(PROGRAMMER * pgm, AVRPART * p, int * cycles)
{
  AVRMEM * a;
  int cycle_count;
  unsigned char v1, v2, v3, v4;
  int rc;

  a = avr_locate_mem(p, "eeprom");
  if (a == NULL) {
    return -1;
  }

  rc = avr_read_byte(pgm, p, a, a->size-4, &v1);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't read memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
  }

  rc = avr_read_byte(pgm, p, a, a->size-3, &v2);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't read memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
  }

  rc = avr_read_byte(pgm, p, a, a->size-2, &v3);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't read memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
  }

  rc = avr_read_byte(pgm, p, a, a->size-1, &v4);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't read memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
  }

  if ((v1 == 0xff) && (v2 == 0xff) && (v3 != 0xff) && (v4 != 0xff)) {
    v1 = 0;
    v2 = 0;
  }

  cycle_count = (((unsigned int)v1) << 24) | 
    (((unsigned int)v2) << 16) |
    (((unsigned int)v3) << 8) |
    v4;

  *cycles = cycle_count;

  return 0;
}


int avr_put_cycle_count(PROGRAMMER * pgm, AVRPART * p, int cycles)
{
  AVRMEM * a;
  unsigned char v1, v2, v3, v4;
  int rc;

  a = avr_locate_mem(p, "eeprom");
  if (a == NULL) {
    return -1;
  }

  v4 = cycles & 0x0ff;
  v3 = (cycles & 0x0ff00) >> 8;
  v2 = (cycles & 0x0ff0000) >> 16;
  v1 = (cycles & 0x0ff000000) >> 24;

  rc = avr_write_byte(pgm, p, a, a->size-4, v1);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't write memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
  }
  rc = avr_write_byte(pgm, p, a, a->size-3, v2);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't write memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
  }
  rc = avr_write_byte(pgm, p, a, a->size-2, v3);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't write memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
  }
  rc = avr_write_byte(pgm, p, a, a->size-1, v4);
  if (rc < 0) {
    fprintf(stderr, "%s: WARNING: can't write memory for cycle count, rc=%d\n",
            progname, rc);
    return -1;
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



void avr_mem_display(char * prefix, FILE * f, AVRMEM * m, int type, 
                     int verbose)
{
  int i, j;
  char * optr;

  if (m == NULL) {
    fprintf(f, 
            "%s                          Page                       Polled\n"
            "%sMemory Type Paged  Size   Size #Pages MinW  MaxW   ReadBack\n"
            "%s----------- ------ ------ ---- ------ ----- ----- ---------\n",
            prefix, prefix, prefix);
  }
  else {
    if (verbose > 2) {
      fprintf(f, 
              "%s                          Page                       Polled\n"
              "%sMemory Type Paged  Size   Size #Pages MinW  MaxW   ReadBack\n"
              "%s----------- ------ ------ ---- ------ ----- ----- ---------\n",
              prefix, prefix, prefix);
    }
    fprintf(f,
            "%s%-11s %-6s %6d %4d %5d %5d %5d 0x%02x 0x%02x\n",
            prefix, m->desc,
            m->paged ? "yes" : "no",
            m->size, 
            m->page_size, 
            m->num_pages, 
            m->min_write_delay, 
            m->max_write_delay,
            m->readback[0], 
            m->readback[1]);
    if (verbose > 2) {
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


