/*
 * Copyright (c) 2000, 2001, 2002  Brian S. Dean <bsd@bsdhome.com>
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BRIAN S. DEAN ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BRIAN S. DEAN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * 
 */

/* $Id$ */

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


char * avr_version = "$Id$";


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
 * transmit and receive a byte of data to/from the AVR device
 */
unsigned char avr_txrx(int fd, unsigned char byte)
{
  int i;
  unsigned char r, b, rbyte;

  rbyte = 0;
  for (i=0; i<8; i++) {
    b = (byte >> (7-i)) & 0x01;

    /* 
     * read the result bit (it is either valid from a previous clock
     * pulse or it is ignored in the current context)
     */
    r = ppi_getpin(fd, pgm->pinno[PIN_AVR_MISO]);
    
    /* set the data input line as desired */
    ppi_setpin(fd, pgm->pinno[PIN_AVR_MOSI], b);
    
    /* 
     * pulse the clock line, clocking in the MOSI data, and clocking out
     * the next result bit
     */
    ppi_pulsepin(fd, pgm->pinno[PIN_AVR_SCK]);

    rbyte = rbyte | (r << (7-i));
  }

  return rbyte;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
int avr_cmd(int fd, unsigned char cmd[4], unsigned char res[4])
{
  int i;

  for (i=0; i<4; i++) {
    res[i] = avr_txrx(fd, cmd[i]);
  }

#if 0
  fprintf(stderr, "avr_cmd(): [ ");
  for (i=0; i<4; i++)
    fprintf(stderr, "%02x ", cmd[i]);
  fprintf(stderr, "] [ ");
  for (i=0; i<4; i++)
    fprintf(stderr, "%02x ", res[i]);
  fprintf(stderr, "]\n");
#endif

  return 0;
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
int avr_read_byte(int fd, AVRPART * p, AVRMEM * mem, unsigned long addr, 
                  unsigned char * value)
{
  unsigned char cmd[4];
  unsigned char res[4];
  unsigned char data;
  OPCODE * readop;

  LED_ON(fd, pgm->pinno[PIN_LED_PGM]);
  LED_OFF(fd, pgm->pinno[PIN_LED_ERR]);

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
  avr_cmd(fd, cmd, res);
  data = 0;
  avr_get_output(readop, res, &data);

  LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);

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
int avr_read(int fd, AVRPART * p, char * memtype, int size, int verbose)
{
  unsigned char    rbyte;
  unsigned long    i;
  unsigned char  * buf;
  AVRMEM * mem;
  int rc;

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

  for (i=0; i<size; i++) {
    rc = avr_read_byte(fd, p, mem, i, &rbyte);
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
        fprintf(stderr, "\r        \r%6lu", i);
      }
    }
  }

  fprintf(stderr, "\n");

  return i;
}


/*
 * write a page data at the specified address
 */
int avr_write_page(int fd, AVRPART * p, AVRMEM * mem,
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
  if (mem->op[AVR_OP_LOADPAGE_LO])
    addr = addr / 2;

  LED_ON(fd, pgm->pinno[PIN_LED_PGM]);
  LED_OFF(fd, pgm->pinno[PIN_LED_ERR]);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(wp, cmd);
  avr_set_addr(wp, cmd, addr);
  avr_cmd(fd, cmd, res);

  /*
   * since we don't know what voltage the target AVR is powered by, be
   * conservative and delay the max amount the spec says to wait 
   */
  usleep(mem->max_write_delay);

  LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
  return 0;
}


/*
 * write a byte of data at the specified address
 */
int avr_write_byte(int fd, AVRPART * p, AVRMEM * mem,
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
    rc = avr_read_byte(fd, p, mem, addr, &b);
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


  LED_ON(fd, pgm->pinno[PIN_LED_PGM]);
  LED_OFF(fd, pgm->pinno[PIN_LED_ERR]);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(writeop, cmd);
  avr_set_addr(writeop, cmd, caddr);
  avr_set_input(writeop, cmd, data);
  avr_cmd(fd, cmd, res);

  if (mem->paged) {
    /*
     * in paged addressing, single bytes to be written to the memory
     * page complete immediately, we only need to delay when we commit
     * the whole page via the avr_write_page() routine.
     */
    LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
    return 0;
  }

  if (readok == 0) {
    /*
     * read operation not supported for this memory type, just wait
     * the max programming time and then return 
     */
    usleep(mem->max_write_delay); /* maximum write delay */
    LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
    return 0;
  }

  tries = 0;
  ready = 0;
  while (!ready) {
    usleep(mem->min_write_delay);
    rc = avr_read_byte(fd, p, mem, addr, &r);
    if (rc != 0) {
      LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
      LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
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
      rc = avr_read_byte(fd, p, mem, addr, &r);
      if (rc != 0) {
        LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
        LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
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
      LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
      fprintf(stderr,
              "%s: this device must be powered off and back on to continue\n",
              progname);
      if (pgm->pinno[PPI_AVR_VCC]) {
        fprintf(stderr, "%s: attempting to do this now ...\n", progname);
        avr_powerdown(fd);
        usleep(250000);
        rc = avr_initialize(fd, p);
        if (rc < 0) {
          fprintf(stderr, "%s: initialization failed, rc=%d\n", progname, rc);
          fprintf(stderr, 
                  "%s: can't re-initialize device after programming the "
                  "%s bits\n", progname, mem->desc);
          fprintf(stderr,
                  "%s: you must manually power-down the device and restart\n"
                  "%s:   avrprog to continue.\n",
                  progname, progname);
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
      LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
      LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
      
      return -6;
    }
  }

  LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
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
int avr_write(int fd, AVRPART * p, char * memtype, int size, int verbose)
{
  int              rc;
  int              wsize;
  unsigned long    i;
  unsigned char    data;
  int              werror;
  AVRMEM         * m;

  m = avr_locate_mem(p, memtype);
  if (m == NULL) {
    fprintf(stderr, "No \"%s\" memory for part %s\n",
            memtype, p->desc);
    return -1;
  }

  LED_OFF(fd, pgm->pinno[PIN_LED_ERR]);

  werror = 0;

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
      }
    }
    rc = avr_write_byte(fd, p, m, i, data);
    if (rc) {
      fprintf(stderr, " ***failed;  ");
      fprintf(stderr, "\n");
      LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
      werror = 1;
    }

    if (m->paged) {
      /*
       * check to see if it is time to flush the page with a page
       * write
       */
      if (((i % m->page_size) == m->page_size-1) ||
          (i == wsize-1)) {
        rc = avr_write_page(fd, p, m, i);
        if (rc) {
          fprintf(stderr,
                  " *** page %ld (addresses 0x%04lx - 0x%04lx) failed "
                  "to write\n",
                  i % m->page_size, 
                  i - m->page_size + 1, i);
          fprintf(stderr, "\n");
          LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
          werror = 1;
        }
      }
    }

    if (werror) {
      /* 
       * make sure the error led stay on if there was a previous write
       * error, otherwise it gets cleared in avr_write_byte() 
       */
      LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
    }
  }


  fprintf(stderr, "\n");

  return i;
}


/*
 * issue the 'program enable' command to the AVR device
 */
int avr_program_enable(int fd, AVRPART * p)
{
  unsigned char cmd[4];
  unsigned char res[4];

  if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
    fprintf(stderr, "program enable instruction not defined for part \"%s\"\n",
            p->desc);
    return -1;
  }

  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
  avr_cmd(fd, cmd, res);

  if (res[2] != cmd[1])
    return -2;

  return 0;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
int avr_chip_erase(int fd, AVRPART * p)
{
  unsigned char cmd[4];
  unsigned char res[4];

  if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
    fprintf(stderr, "chip erase instruction not defined for part \"%s\"\n",
            p->desc);
    return -1;
  }

  LED_ON(fd, pgm->pinno[PIN_LED_PGM]);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  avr_cmd(fd, cmd, res);
  usleep(p->chip_erase_delay);
  avr_initialize(fd, p);

  LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);

  return 0;
}


/*
 * read the AVR device's signature bytes
 */
int avr_signature(int fd, AVRPART * p)
{
  int rc;

  rc = avr_read(fd, p, "signature", 0, 0);
  if (rc < 0) {
    fprintf(stderr, 
            "%s: error reading signature data for part \"%s\", rc=%d\n",
            progname, p->desc, rc);
    return -1;
  }

  return 0;
}


/*
 * apply power to the AVR processor
 */
void avr_powerup(int fd)
{
  ppi_set(fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power up */
  usleep(100000);
}


/*
 * remove power from the AVR processor
 */
void avr_powerdown(int fd)
{
  ppi_clr(fd, PPIDATA, pgm->pinno[PPI_AVR_VCC]);    /* power down */
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
int avr_initialize(int fd, AVRPART * p)
{
  int rc;
  int tries;

  avr_powerup(fd);
  usleep(20000);

  ppi_setpin(fd, pgm->pinno[PIN_AVR_SCK], 0);
  ppi_setpin(fd, pgm->pinno[PIN_AVR_RESET], 0);
  usleep(20000);

  ppi_pulsepin(fd, pgm->pinno[PIN_AVR_RESET]);

  usleep(20000); /* 20 ms XXX should be a per-chip parameter */

  /*
   * Enable programming mode.  If we are programming an AT90S1200, we
   * can only issue the command and hope it worked.  If we are using
   * one of the other chips, the chip will echo 0x53 when issuing the
   * third byte of the command.  In this case, try up to 32 times in
   * order to possibly get back into sync with the chip if we are out
   * of sync.
   */
  if (strcmp(p->desc, "AT90S1200")==0) {
    avr_program_enable(fd, p);
  }
  else {
    tries = 0;
    do {
      rc = avr_program_enable(fd, p);
      if ((rc == 0)||(rc == -1))
        break;
      ppi_pulsepin(fd, pgm->pinno[PIN_AVR_SCK]);
      tries++;
    } while (tries < 65);

    /*
     * can't sync with the device, maybe it's not attached?
     */
    if (rc) {
      fprintf(stderr, "%s: AVR device not responding\n", progname);
      return -1;
    }
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



void avr_display(FILE * f, AVRPART * p, char * prefix, int verbose)
{
  int i;
  char * buf;
  char * px;
  LNODEID ln;
  AVRMEM * m;

  fprintf(f, 
          "%sAVR Part         : %s\n"
          "%sChip Erase delay : %d us\n"
          "%sMemory Detail    :\n\n",
          prefix, p->desc,
          prefix, p->chip_erase_delay,
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


