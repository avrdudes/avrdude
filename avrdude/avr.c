/*
 * Copyright 2000  Brian S. Dean <bsd@bsdhome.com>
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


#include "avr.h"
#include "ppi.h"


extern char * progname;
extern char   progbuf[];




/* Need to add information for 2323, 2343, and 4414 */

struct avrpart parts[] = {
  { "AT90S1200", "1200", 1024,  64, 0xff, { 0x00, 0xff }, 
    9000, 20000, 20000, NULL, NULL },

  { "AT90S2313", "2313", 2048, 128, 0x7f, { 0x80, 0x7f }, 
    9000, 20000, 20000, NULL, NULL },

  { "AT90S2333", "2333", 2048, 128, 0xff, { 0x00, 0xff },
    9000, 20000, 20000, NULL, NULL },

  { "AT90S4433", "4433", 4096, 256, 0xff, { 0x00, 0xff },
    9000, 20000, 20000, NULL, NULL },

  { "AT90S4434", "4434", 4096, 256, 0xff, { 0x00, 0xff },
    9000, 20000, 20000, NULL, NULL },

  { "AT90S8515", "8515", 8192, 512, 0x7f, { 0x80, 0x7f },
    9000, 20000, 20000, NULL, NULL },

  { "AT90S8535", "8535", 8192, 512, 0xff, { 0x00, 0xff },
    9000, 20000, 20000, NULL, NULL },
};

#define N_AVRPARTS (sizeof(parts)/sizeof(struct avrpart))




int avr_list_parts ( FILE * f, char * prefix )
{
  int i;

  for (i=0; i<N_AVRPARTS; i++) {
    fprintf(f, "%s%s = %s\n", 
            prefix, parts[i].optiontag, parts[i].partdesc);
  }

  return i;
}

struct avrpart * avr_find_part ( char * p )
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
int avr_txrx_bit ( int fd, int bit )
{
  int r;

  /* 
   * read the result bit (it is either valid from a previous clock
   * pulse or it is ignored in the current context)
   */
  r = ppi_get(fd, PPISTATUS, AVR_DATA);

  /* set the data input line as desired */
  if (bit)
    ppi_set(fd, PPIDATA, AVR_INSTR);
  else
    ppi_clr(fd, PPIDATA, AVR_INSTR);

  /* 
   * pulse the clock line, clocking in the MOSI data, and clocking out
   * the next result bit
   */
  ppi_pulse(fd, PPIDATA, AVR_CLOCK);

  return r;
}


/*
 * transmit and receive a byte of data to/from the AVR device
 */
unsigned char avr_txrx ( int fd, unsigned char byte )
{
  int i;
  unsigned char r, b, rbyte;

  rbyte = 0;
  for (i=0; i<8; i++) {
    b = (byte >> (7-i)) & 0x01;
    r = avr_txrx_bit ( fd, b );
    rbyte = rbyte | (r << (7-i));
  }

  return rbyte;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
int avr_cmd ( int fd, unsigned char cmd[4], unsigned char res[4] )
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
unsigned char avr_read_byte ( int fd, struct avrpart * p,
                              AVRMEM memtype, unsigned short addr )
{
  unsigned char cmd[4];
  unsigned char res[4];

  switch (memtype) {
    case AVR_FLASH_LO: 
      cmd[0] = 0x20;
      break;
    case AVR_FLASH_HI: 
      cmd[0] = 0x28;
      break;
    case AVR_EEPROM: 
      cmd[0] = 0xa0;
      break;
    default:
      fprintf(stderr, 
              "%s: avr_read_byte(); internal error: invalid memtype=%d\n",
              progname, memtype);
      exit(1);
      break;
  }

  cmd[1] = addr >> 8;     /* high order bits of address       */
  cmd[2] = addr & 0x0ff;  /* low order bits of address        */
  cmd[3] = 0;             /* don't care                       */

  avr_cmd(fd, cmd, res);

  return res[3];
}


/*
 * read the entirety of the specified memory type into the
 * corresponding buffer of the avrpart pointed to by 'p'.  
 */
int avr_read ( int fd, struct avrpart * p, AVRMEM memtype )
{
  unsigned char rbyte, memt;
  unsigned short n, start, end, i, bi;
  unsigned char * buf;
  int bufsize;

  start = 0;

  switch (memtype) {
    case AVR_FLASH  :
      memt    = AVR_FLASH_LO;
      buf     = p->flash;
      n       = p->flash_size/2;
      bufsize = p->flash_size;
      break;

    case AVR_EEPROM : 
      memt    = memtype;
      buf     = p->eeprom;
      n       = p->eeprom_size;
      bufsize = p->eeprom_size;
      break;

    default:
      fprintf(stderr, "%s: avr_read(); internal error: invalid memtype=%d\n",
              progname, memtype);
      exit(1);
      break;
  }

  end = start+n;

  bi = 0;

  for (i=start; i<end; i++) {
    /* eeprom or low byte of flash */
    rbyte = avr_read_byte(fd, p, memt, i);
    fprintf ( stderr, "                    \r%4u  0x%02x", i, rbyte );
    if (bi < bufsize) {
      buf[bi++] = rbyte;
    }

    if (memtype == AVR_FLASH) {
      /* flash high byte */
      rbyte = avr_read_byte(fd, p, AVR_FLASH_HI, i);
      fprintf ( stderr, " 0x%02x", rbyte );
      if (bi < bufsize) {
        buf[bi++] = rbyte;
      }
    }
  }

  fprintf ( stderr, "\n" );

  return 0;
}


/*
 * write a byte of data to the indicated memory region
 */
int avr_write_byte ( int fd, struct avrpart * p, AVRMEM memtype, 
                     unsigned short addr, unsigned char data )
{
  unsigned char cmd[4], res[4];
  unsigned char r;
  int ready;
  int tries;
  unsigned char b;

  /* 
   * check to see if the write is necessary by reading the existing
   * value and only write if we are changing the value 
   */
  b = avr_read_byte(fd, p, memtype, addr);
  if (b == data) {
    return 0;
  }

  switch (memtype) {
    case AVR_FLASH_LO: 
      cmd[0] = 0x40;
      break;
    case AVR_FLASH_HI: 
      cmd[0] = 0x48;
      break;
    case AVR_EEPROM: 
      cmd[0] = 0xc0;
      break;
    default:
      fprintf(stderr,
              "%s: avr_write_byte(); internal error: invalid memtype=%d\n",
              progname, memtype);
      exit(1);
      break;
  }

  cmd[1] = addr >> 8;     /* high order bits of address       */
  cmd[2] = addr & 0x0ff;  /* low order bits of address        */
  cmd[3] = data;          /* data                             */

  avr_cmd(fd, cmd, res);

  tries = 0;
  ready = 0;
  while (!ready) {
    usleep(p->min_write_delay); /* typical flash/eeprom write delay */
    r = avr_read_byte(fd, p, memtype, addr);
    if ((data == p->f_readback) ||
        (data == p->e_readback[0]) || (data == p->e_readback[1])) {
      /* 
       * use an extra long delay when we happen to be writing values
       * used for polled data read-back.  In this case, polling
       * doesn't work, and we need to delay the worst case write time
       * specified for the chip.
       */
      usleep(p->max_write_delay);
      ready = 1;
    }
    else if (r == data) {
      ready = 1;
    }

    tries++;
    if (!ready && tries > 10) {
      /*
       * we couldn't write the data, indicate our displeasure by
       * returning an error code 
       */
      return -1;
    }
  }

  return 0;
}


/*
 * Write the whole memory region (flash or eeprom, specified by
 * 'memtype') from the corresponding buffer of the avrpart pointed to
 * by 'p'.  All of the memory is updated, however, input data of 0xff
 * is not actually written out, because empty flash and eeprom
 * contains 0xff, and you can't actually write 1's, only 0's.
 */
int avr_write ( int fd, struct avrpart * p, AVRMEM memtype )
{
  unsigned char data, memt;
  unsigned short start, end, i, bi;
  int nl;
  int rc;
  unsigned char * buf;
  int bufsize;

  start = 0;

  switch (memtype) {
    case AVR_FLASH  : 
      buf     = p->flash;
      bufsize = p->flash_size;
      end     = start+bufsize/2;
      memt    = AVR_FLASH_LO;
      break;
    case AVR_EEPROM : 
      buf     = p->eeprom;
      bufsize = p->eeprom_size;
      end     = start+bufsize;
      memt    = memtype;
      break;
    default:
      fprintf(stderr, "%s: avr_write(); internal error: invalid memtype=%d\n",
              progname, memtype);
      exit(1);
      break;
  }

  bi = 0;

  for (i=start; i<end; i++) {
    /* eeprom or low byte of flash */
    data = buf[bi++];
    nl = 0;
    rc = avr_write_byte(fd, p, memt, i, data );
    fprintf(stderr, "                      \r%4u 0x%02x", i, data);
    if (rc) {
      fprintf(stderr, " ***failed;  ");
      nl = 1;
    }
    
    if (memtype == AVR_FLASH) {
      /* high byte of flash */
      data = buf[bi++];
      rc = avr_write_byte(fd, p, AVR_FLASH_HI, i, data );
      fprintf(stderr, " 0x%02x", data);
      if (rc) {
        fprintf(stderr, " ***failed;  " );
        nl = 1;
      }
    }
    if (nl)
      fprintf(stderr, "\n");
  }

  fprintf ( stderr, "\n" );

  return 0;
}


/*
 * issue the 'program enable' command to the AVR device
 */
int avr_program_enable ( int fd )
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
int avr_chip_erase ( int fd, struct avrpart * p )
{
  unsigned char data[4] = {0xac, 0x80, 0x00, 0x00};
  unsigned char res[4];

  avr_cmd(fd, data, res);
  usleep(p->chip_erase_delay);
  avr_initialize(fd, p);

  return 0;
}


/*
 * read the AVR device's signature bytes
 */
int avr_signature ( int fd, unsigned char sig[4] )
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
void avr_powerup ( int fd )
{
  ppi_set(fd, PPIDATA, AVR_POWER);    /* power up */
  usleep(100000);
}


/*
 * remove power from the AVR processor
 */
void avr_powerdown ( int fd )
{
  ppi_clr(fd, PPIDATA, AVR_POWER);    /* power down */
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
int avr_initialize ( int fd, struct avrpart * p )
{
  int rc;
  int tries;

  avr_powerup(fd);

  ppi_clr(fd, PPIDATA, AVR_CLOCK);
  ppi_clr(fd, PPIDATA, AVR_RESET);
  ppi_pulse(fd, PPIDATA, AVR_RESET);

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
    avr_program_enable ( fd );
  }
  else {
    tries = 0;
    do {
      rc = avr_program_enable ( fd );
      if (rc == 0)
        break;
      ppi_pulse(fd, PPIDATA, AVR_CLOCK);
      tries++;
    } while (tries < 32);

    /*
     * can't sync with the device, maybe it's not attached?
     */
    if (tries == 32) {
      fprintf ( stderr, "%s: AVR device not responding\n", progname );
      return -1;
    }
  }

  return 0;
}



char * avr_memtstr ( AVRMEM memtype )
{
  switch (memtype) {
    case AVR_EEPROM : return "eeprom"; break;
    case AVR_FLASH  : return "flash"; break;
    default         : return "unknown-memtype"; break;
  }
}



int avr_initmem ( struct avrpart * p )
{
  p->flash = (unsigned char *) malloc(p->flash_size);
  if (p->flash == NULL) {
    fprintf(stderr, "%s: can't alloc buffer for flash size of %d bytes\n",
            progname, p->flash_size);
    exit(1);
  }

  p->eeprom = (unsigned char *) malloc(p->eeprom_size);
  if (p->eeprom == NULL) {
    fprintf(stderr, "%s: can't alloc buffer for eeprom size of %d bytes\n",
            progname, p->eeprom_size);
    exit(1);
  }

  return 0;
}


int avr_verify(struct avrpart * p, struct avrpart * v, AVRMEM memtype)
{
  int i;
  unsigned char * buf1, * buf2;
  int size;

  switch (memtype) {
    case AVR_FLASH:
      buf1 = p->flash;
      buf2 = v->flash;
      size = p->flash_size;
      break;

    case AVR_EEPROM:
      buf1 = p->eeprom;
      buf2 = v->eeprom;
      size = p->eeprom_size;
      break;

    default:
      fprintf(stderr, "%s: invalid memory type = %d for data verification\n",
              progname, memtype);
      return -1;
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

  return 0;
}


void avr_display ( FILE * f, struct avrpart * p, char * prefix )
{
  fprintf(f, 
          "%sAVR Part               = %s\n"
          "%sFlash memory size      = %d bytes\n"
          "%sEEPROM memory size     = %d bytes\n"
          "%sMin/Max program delay  = %d/%d us\n"
          "%sChip Erase delay       = %d us\n"
          "%sFlash Polled Readback  = 0x%02x\n"
          "%sEEPROM Polled Readback = 0x%02x, 0x%02x\n",
          prefix, p->partdesc,
          prefix, p->flash_size, 
          prefix, p->eeprom_size,
          prefix, p->min_write_delay, p->max_write_delay, 
          prefix, p->chip_erase_delay,
          prefix, p->f_readback,
          prefix, p->e_readback[0], p->e_readback[1]);
}

