/*
 * Copyright 2001  Brian S. Dean <bsd@bsdhome.com>
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
#include "pindefs.h"
#include "ppi.h"


extern char * progname;
extern char   progbuf[];


char * avr_version = "$Id$";


/* Need to add information for 2323, 2343, and 4414 */

struct avrpart parts[] = {
  {"AT90S1200", "1200", 20000, 
   {{0,     64,   0,   0,  9000, 20000, {0x00, 0xff }, NULL},   /* eeprom */
    {0,   1024,   0,   0,  9000, 20000, {0xff,    0 }, NULL}}}, /* flash  */
  
  {"AT90S2313", "2313", 20000, 
   {{0,    128,   0,   0,  9000, 20000, {0x80, 0x7f }, NULL},   /* eeprom */
    {0,   2048,   0,   0,  9000, 20000, {0x7f,    0 }, NULL}}}, /* flash  */
  
  {"AT90S2333", "2333", 20000, 
   {{0,    128,   0,   0,  9000, 20000, {0x00, 0xff }, NULL},   /* eeprom */
    {0,   2048,   0,   0,  9000, 20000, {0xff,    0 }, NULL}}}, /* flash  */
  
  {"AT90S4433", "4433", 20000, 
   {{0,    256,   0,   0,  9000, 20000, {0x00, 0xff }, NULL},   /* eeprom */
    {0,   4096,   0,   0,  9000, 20000, {0xff,    0 }, NULL}}}, /* flash  */
  
  {"AT90S4434", "4434", 20000, 
   {{0,    256,   0,   0,  9000, 20000, {0x00, 0xff }, NULL},   /* eeprom */
    {0,   4096,   0,   0,  9000, 20000, {0xff,    0 }, NULL}}}, /* flash  */
  
  {"AT90S8515", "8515", 20000, 
   {{0,    512,   0,   0,  9000, 20000, {0x80, 0x7f }, NULL},   /* eeprom */
    {0,   8192,   0,   0,  9000, 20000, {0x7f, 0x00 }, NULL}}}, /* flash  */
  
  {"AT90S8535", "8535", 20000, 
   {{0,    512,   0,   0,  9000, 20000, {0x00, 0xff }, NULL},   /* eeprom */
    {0,   8192,   0,   0,  9000, 20000, {0xff, 0x00 }, NULL}}}, /* flash  */
  
  {"ATMEGA103", "103", 56000*2, 
   {{0,   4096,   0,   0, 64000, 69000, {0x80, 0x7f }, NULL},   /* eeprom */
    {1, 131072, 256, 512, 22000, 56000, {0xff, 0x00 }, NULL}}}, /* flash  */

};

#define N_AVRPARTS (sizeof(parts)/sizeof(struct avrpart))




int avr_list_parts(FILE * f, char * prefix)
{
  int i;

  for (i=0; i<N_AVRPARTS; i++) {
    fprintf(f, "%s%s = %s\n", 
            prefix, parts[i].optiontag, parts[i].partdesc);
  }

  return i;
}


struct avrpart * avr_find_part(char * p)
{
  int i;

  for (i=0; i<N_AVRPARTS; i++) {
    if (strcmp(parts[i].optiontag, p)==0) {
      return &parts[i];
    }
  }

  return NULL;
}


/*
 * transmit and receive a bit of data to/from the AVR device
 */
int avr_txrx_bit(int fd, int bit)
{
  int r;

  /* 
   * read the result bit (it is either valid from a previous clock
   * pulse or it is ignored in the current context)
   */
  r = ppi_getpin(fd, pinno[PIN_AVR_MISO]);

  /* set the data input line as desired */
  ppi_setpin(fd, pinno[PIN_AVR_MOSI], bit);

  /* 
   * pulse the clock line, clocking in the MOSI data, and clocking out
   * the next result bit
   */
  ppi_pulsepin(fd, pinno[PIN_AVR_SCK]);

  return r;
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
    r = avr_txrx_bit(fd, b);
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

  return 0;
}


/*
 * read a byte of data from the indicated memory region
 */
unsigned char avr_read_byte(int fd, struct avrpart * p,
                              int memtype, unsigned long addr)
{
  unsigned short offset;
  unsigned char cmd[4];
  unsigned char res[4];
  /* order here is very important, AVR_EEPROM, AVR_FLASH, AVR_FLASH+1 */
  static unsigned char cmdbyte[3] = { 0xa0, 0x20, 0x28 };

  LED_ON(fd, pinno[PIN_LED_PGM]);
  LED_OFF(fd, pinno[PIN_LED_ERR]);

  offset = 0;

  if (memtype == AVR_M_FLASH) {
    offset = addr & 0x01;
    addr   = addr / 2;
  }

  cmd[0] = cmdbyte[memtype + offset];
  cmd[1] = addr >> 8;     /* high order bits of address       */
  cmd[2] = addr & 0x0ff;  /* low order bits of address        */
  cmd[3] = 0;             /* don't care                       */

  avr_cmd(fd, cmd, res);

  LED_OFF(fd, pinno[PIN_LED_PGM]);

  return res[3];
}


/*
 * Read the entirety of the specified memory type into the
 * corresponding buffer of the avrpart pointed to by 'p'.
 *
 * Return the number of bytes read, or -1 if an error occurs.  
 */
int avr_read(int fd, struct avrpart * p, int memtype)
{
  unsigned char    rbyte;
  unsigned long    i;
  unsigned char  * buf;
  int              size;

  buf  = p->mem[memtype].buf;
  size = p->mem[memtype].size;

  for (i=0; i<size; i++) {
    rbyte = avr_read_byte(fd, p, memtype, i);
    fprintf(stderr, "                    \r%4lu  0x%02x", i, rbyte);
    buf[i] = rbyte;
  }

  fprintf(stderr, "\n");

  return i;
}


/*
 * write a byte of data to the indicated memory region
 */
int avr_write_bank(int fd, struct avrpart * p, int memtype, 
                   unsigned short bank)
{
  unsigned char cmd[4];
  unsigned char res[4];

  LED_ON(fd, pinno[PIN_LED_PGM]);
  LED_OFF(fd, pinno[PIN_LED_ERR]);

  cmd[0] = 0x4c;
  cmd[1] = bank >> 8;     /* high order bits of address */
  cmd[2] = bank & 0x0ff;  /* low order bits of address  */
  cmd[3] = 0;             /* these bits are ignored     */

  avr_cmd(fd, cmd, res);

  /*
   * since we don't know what voltage the target AVR is powered by, be
   * conservative and delay the max amount the spec says to wait 
   */
  usleep(p->mem[memtype].max_write_delay);

  LED_OFF(fd, pinno[PIN_LED_PGM]);
  return 0;
}


/*
 * write a byte of data to the indicated memory region
 */
int avr_write_byte(int fd, struct avrpart * p, int memtype, 
                     unsigned long addr, unsigned char data)
{
  unsigned char cmd[4];
  unsigned char res[4];
  unsigned char r;
  int ready;
  int tries;
  unsigned char b;
  unsigned short offset;
  unsigned short caddr;
  /* order here is very important, AVR_M_EEPROM, AVR_M_FLASH, AVR_M_FLASH+1 */
  static unsigned char cmdbyte[3] = { 0xc0, 0x40, 0x48 };

  if (!p->mem[memtype].banked) {
    /* 
     * check to see if the write is necessary by reading the existing
     * value and only write if we are changing the value; we can't
     * use this optimization for banked addressing.
     */
    b = avr_read_byte(fd, p, memtype, addr);
    if (b == data) {
      return 0;
    }
  }
  else {
    addr = addr % p->mem[memtype].bank_size;
  }

  LED_ON(fd, pinno[PIN_LED_PGM]);
  LED_OFF(fd, pinno[PIN_LED_ERR]);

  offset = 0;

  caddr = addr;
  if (memtype == AVR_M_FLASH) {
    offset = addr & 0x01;
    caddr  = addr / 2;
  }

  cmd[0] = cmdbyte[memtype + offset];
  cmd[1] = caddr >> 8;     /* high order bits of address       */
  cmd[2] = caddr & 0x0ff;  /* low order bits of address        */
  cmd[3] = data;           /* data                             */

  avr_cmd(fd, cmd, res);

  if (p->mem[memtype].banked) {
    /*
     * in banked addressing, single bytes to written to the memory
     * page complete immediately, we only need to delay when we commit
     * the whole page via the avr_write_bank() routine.
     */
    LED_OFF(fd, pinno[PIN_LED_PGM]);
    return 0;
  }

  tries = 0;
  ready = 0;
  while (!ready) {
    usleep(p->mem[memtype].min_write_delay); /* typical write delay */
    r = avr_read_byte(fd, p, memtype, addr);
    if ((data == p->mem[memtype].readback[0]) ||
        (data == p->mem[memtype].readback[1])) {
      /* 
       * use an extra long delay when we happen to be writing values
       * used for polled data read-back.  In this case, polling
       * doesn't work, and we need to delay the worst case write time
       * specified for the chip.
       */
      usleep(p->mem[memtype].max_write_delay);
      r = avr_read_byte(fd, p, memtype, addr);
    }

    if (r == data) {
      ready = 1;
    }

    tries++;
    if (!ready && tries > 5) {
      /*
       * we couldn't write the data, indicate our displeasure by
       * returning an error code 
       */
      LED_OFF(fd, pinno[PIN_LED_PGM]);
      LED_ON(fd, pinno[PIN_LED_ERR]);

      return -1;
    }
  }

  LED_OFF(fd, pinno[PIN_LED_PGM]);
  return 0;
}


/*
 * Write the whole memory region (flash or eeprom, specified by
 * 'memtype') from the corresponding buffer of the avrpart pointed to
 * by 'p'.  Write up to 'size' bytes from the buffer.  Data is only
 * written if the new data value is different from the existing data
 * value.  Data beyond 'size' bytes is not affected.
 *
 * Return the number of bytes written, or -1 if an error occurs.
 */
int avr_write(int fd, struct avrpart * p, int memtype, int size)
{
  int              rc;
  int              wsize;
  unsigned long    i;
  unsigned char    data;
  int              werror;

  LED_OFF(fd, pinno[PIN_LED_ERR]);

  werror = 0;

  wsize = p->mem[memtype].size;
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
    /* eeprom or low byte of flash */
    data = p->mem[memtype].buf[i];
    rc = avr_write_byte(fd, p, memtype, i, data);
    fprintf(stderr, "                      \r%4lu 0x%02x", i, data);
    if (rc) {
      fprintf(stderr, " ***failed;  ");
      fprintf(stderr, "\n");
      LED_ON(fd, pinno[PIN_LED_ERR]);
      werror = 1;
    }

    if (p->mem[memtype].banked) {
      if (((i % p->mem[memtype].bank_size) == p->mem[memtype].bank_size-1) ||
          (i == wsize-1)) {
        rc = avr_write_bank(fd, p, memtype, i/p->mem[memtype].bank_size);
        if (rc) {
          fprintf(stderr,
                  " *** bank %ld (addresses 0x%04lx - 0x%04lx) failed to write\n",
                  i % p->mem[memtype].bank_size, 
                  i-p->mem[memtype].bank_size+1, i);
          fprintf(stderr, "\n");
          LED_ON(fd, pinno[PIN_LED_ERR]);
          werror = 1;
        }
      }
    }

    if (werror) {
      /* 
       * make sure the error led stay on if there was a previous write
       * error, otherwise it gets cleared in avr_write_byte() 
       */
      LED_ON(fd, pinno[PIN_LED_ERR]);
    }
  }


  fprintf(stderr, "\n");

  return i;
}


/*
 * issue the 'program enable' command to the AVR device
 */
int avr_program_enable(int fd)
{
  unsigned char cmd[4] = {0xac, 0x53, 0x00, 0x00};
  unsigned char res[4];

  avr_cmd(fd, cmd, res);

  if (res[2] != cmd[1])
    return -1;

  return 0;
}


/*
 * issue the 'chip erase' command to the AVR device
 */
int avr_chip_erase(int fd, struct avrpart * p)
{
  unsigned char data[4] = {0xac, 0x80, 0x00, 0x00};
  unsigned char res[4];

  LED_ON(fd, pinno[PIN_LED_PGM]);

  avr_cmd(fd, data, res);
  usleep(p->chip_erase_delay);
  avr_initialize(fd, p);

  LED_OFF(fd, pinno[PIN_LED_PGM]);

  return 0;
}


/*
 * read the AVR device's signature bytes
 */
int avr_signature(int fd, unsigned char sig[4])
{
  unsigned char cmd[4] = {0x30, 0x00, 0x00, 0x00};
  unsigned char res[4];
  int i;

  for (i=0; i<4; i++) {
    cmd[2] = i;
    avr_cmd(fd, cmd, res);
    sig[i] = res[3];
  }

  return 0;
}


/*
 * apply power to the AVR processor
 */
void avr_powerup(int fd)
{
  ppi_set(fd, PPIDATA, PPI_AVR_VCC);    /* power up */
  usleep(100000);
}


/*
 * remove power from the AVR processor
 */
void avr_powerdown(int fd)
{
  ppi_clr(fd, PPIDATA, PPI_AVR_VCC);    /* power down */
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
int avr_initialize(int fd, struct avrpart * p)
{
  int rc;
  int tries;

  avr_powerup(fd);


  ppi_setpin(fd, pinno[PIN_AVR_SCK], 0);
  ppi_setpin(fd, pinno[PIN_AVR_RESET], 0);
  ppi_pulsepin(fd, pinno[PIN_AVR_RESET]);

  usleep(20000); /* 20 ms XXX should be a per-chip parameter */

  /*
   * Enable programming mode.  If we are programming an AT90S1200, we
   * can only issue the command and hope it worked.  If we are using
   * one of the other chips, the chip will echo 0x53 when issuing the
   * third byte of the command.  In this case, try up to 32 times in
   * order to possibly get back into sync with the chip if we are out
   * of sync.
   */
  if (strcmp(p->partdesc, "AT90S1200")==0) {
    avr_program_enable(fd);
  }
  else {
    tries = 0;
    do {
      rc = avr_program_enable(fd);
      if (rc == 0)
        break;
      ppi_pulsepin(fd, pinno[PIN_AVR_SCK]);
      tries++;
    } while (tries < 32);

    /*
     * can't sync with the device, maybe it's not attached?
     */
    if (tries == 32) {
      fprintf(stderr, "%s: AVR device not responding\n", progname);
      return -1;
    }
  }

  return 0;
}



char * avr_memtstr(int memtype)
{
  switch (memtype) {
    case AVR_M_EEPROM : return "eeprom"; break;
    case AVR_M_FLASH  : return "flash"; break;
    default         : return "unknown-memtype"; break;
  }
}


int avr_initmem(struct avrpart * p)
{
  int i;

  for (i=0; i<AVR_MAXMEMTYPES; i++) {
    p->mem[i].buf = (unsigned char *) malloc(p->mem[i].size);
    if (p->mem[i].buf == NULL) {
      fprintf(stderr, "%s: can't alloc buffer for %s size of %d bytes\n",
              progname, avr_memtstr(i), p->mem[i].size);
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
int avr_verify(struct avrpart * p, struct avrpart * v, int memtype, int size)
{
  int i;
  unsigned char * buf1, * buf2;
  int vsize;

  buf1  = p->mem[memtype].buf;
  buf2  = v->mem[memtype].buf;
  vsize = p->mem[memtype].size;

  if (vsize < size) {
    fprintf(stderr, 
            "%s: WARNING: requested verification for %d bytes\n"
            "%s%s memory region only contains %d bytes\n"
            "%sOnly %d bytes will be verified.\n",
            progname, size,
            progbuf, avr_memtstr(memtype), vsize,
            progbuf, vsize);
    size = vsize;
  }

  for (i=0; i<size; i++) {
    if (buf1[i] != buf2[i]) {
      fprintf(stderr, 
              "%s: verification error, first mismatch at byte %d\n"
              "%s0x%02x != 0x%02x\n",
              progname, i, 
              progbuf, buf1[i], buf2[i]);
      return -1;
    }
  }

  return size;
}



void avr_mem_display(char * prefix, FILE * f, AVRMEM * m, int type)
{
  if (m == NULL) {
    fprintf(f, 
            "%sMem                  Bank\n"
            "%sType   Banked Size   Size #Banks MinW  MaxW   ReadBack\n"
            "%s------ ------ ------ ---- ------ ----- ----- ---------\n",
            prefix, prefix, prefix);
  }
  else {
    fprintf(f,
            "%s%-6s %-6s %6d %4d %6d %5d %5d 0x%02x 0x%02x\n",
            prefix, avr_memtstr(type), m->banked ? "yes" : "no",
            m->size, m->bank_size, m->num_banks, 
            m->min_write_delay, m->max_write_delay,
            m->readback[0], m->readback[1]);
  }
}



void avr_display(FILE * f, struct avrpart * p, char * prefix)
{
  int i;
  char * buf;
  char * px;

  fprintf(f, 
          "%sAVR Part         : %s\n"
          "%sChip Erase delay : %d us\n"
          "%sMemory Detail    :\n\n",
          prefix, p->partdesc,
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
  
  avr_mem_display(px, f, NULL, 0);
  for (i=0; i<AVR_MAXMEMTYPES; i++) {
    avr_mem_display(px, f, &p->mem[i], i);
  }

  if (buf)
    free(buf);
}


