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

/*
 * Code to program an Atmel AVR AT90S device using the parallel port.
 *
 * Make the following connections:
 *
 *  Parallel Port      Atmel AVR
 *  -------------      ----------------------------
 *    Pin  2       ->   Vcc
 *    Pin  3       ->   PB7(SCK)  CLOCK IN
 *    Pin  4       ->   PB5(MOSI) Instruction input
 *    Pin  5       ->   /RESET
 *    Pin 10       <-   PB6(MISO) Data out
 *
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <sys/stat.h>
#include </sys/dev/ppbus/ppi.h>
#include </sys/dev/ppbus/ppbconf.h>

#define PARALLEL "/dev/ppi0"

char * progname;


#define AVR_POWER 0x01  /* bit 0 of data register */
#define AVR_CLOCK 0x02  /* bit 1 of data register */
#define AVR_INSTR 0x04  /* bit 2 of data register */
#define AVR_RESET 0x08  /* bit 3 of data register */
#define AVR_DATA  0x40  /* bit 6 of status register */


enum {
  PPIDATA,
  PPICTRL,
  PPISTATUS
};


enum {
  AVR_EEPROM,
  AVR_FLASH,
  AVR_FLASH_LO,
  AVR_FLASH_HI
};


int ppi_getops ( int reg, unsigned long * get, unsigned long * set )
{
  switch (reg) {
    case PPIDATA:
      *set = PPISDATA;
      *get = PPIGDATA;
      break;
    case PPICTRL:
      *set = PPISCTRL;
      *get = PPIGCTRL;
      break;
    case PPISTATUS:
      *set = PPISSTATUS;
      *get = PPIGSTATUS;
      break;
    default:
      fprintf ( stderr, "%s: avr_set(): invalid register=%d\n",
                progname, reg );
      return -1;
      break;
  }

  return 0;
}


int ppi_set ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v |= bit;
  ioctl(fd, set, &v);

  return 0;
}


int ppi_clr ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v &= ~bit;
  ioctl(fd, set, &v);

  return 0;
}


int ppi_get ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v &= bit;

  return (v == bit);
}


int ppi_toggle ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v |= bit;
  ioctl(fd, set, &v);

  v &= ~bit;
  ioctl(fd, set, &v);

  return 0;
}


int avr_txrx_bit ( int fd, int bit )
{
  unsigned char d;
  int r;

  ioctl(fd, PPIGDATA, &d);

  r = ppi_get(fd, PPISTATUS, AVR_DATA);

  if (bit)
    ppi_set(fd, PPIDATA, AVR_INSTR);
  else
    ppi_clr(fd, PPIDATA, AVR_INSTR);

  ppi_toggle(fd, PPIDATA, AVR_CLOCK);

  return r;
}


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


int avr_cmd ( int fd, unsigned char * cmd, unsigned char * res )
{
  int i;

  for (i=0; i<4; i++) {
    res[i] = avr_txrx(fd, cmd[i]);
  }

  return 0;
}


unsigned char avr_read_byte ( int fd, int memtype, unsigned short addr )
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
      addr &= 0x7f;
      break;
    default:
      fprintf(stderr, "%s: avr_read_byte(); internal error: invalid memtype=%d\n",
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


int avr_read ( int fd, int memtype, unsigned start, unsigned n, 
               unsigned char * buf, int bufsize )
{
  unsigned char rbyte, memt;
  unsigned short end, i, bi;

  switch (memtype) {
    case AVR_FLASH  :
      memt = AVR_FLASH_LO;
      break;
    case AVR_EEPROM : 
      memt = memtype;
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
    rbyte = avr_read_byte(fd, memt, i);
    fprintf ( stderr, "                    \r%4u  0x%02x", i, rbyte );
    if (bi < bufsize) {
      buf[bi++] = rbyte;
    }

    if (memtype == AVR_FLASH) {
      /* flash high byte */
      rbyte = avr_read_byte(fd, AVR_FLASH_HI, i);
      fprintf ( stderr, " 0x%02x", rbyte );
      if (bi < bufsize) {
        buf[bi++] = rbyte;
      }
    }
  }

  fprintf ( stderr, "\n" );

  return 0;
}


int avr_write_byte ( int fd, int memtype, unsigned short addr, unsigned char data )
{
  unsigned char cmd[4], res[4];
  unsigned char r;
  int ready;
  int tries;

  switch (memtype) {
    case AVR_FLASH_LO: 
      cmd[0] = 0x40;
      break;
    case AVR_FLASH_HI: 
      cmd[0] = 0x48;
      break;
    case AVR_EEPROM: 
      cmd[0] = 0xc0;
      addr &= 0x7f;
      break;
    default:
      fprintf(stderr, "%s: avr_write_byte(); internal error: invalid memtype=%d\n",
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
    usleep(5000);               /* flash write delay */
    r = avr_read_byte(fd, memtype, addr);
    if (data == 0x7f) {
      usleep(20000);    /* long delay for 0x7f since polling doesn't work */
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


int avr_write ( int fd, int memtype, unsigned start, 
                unsigned char * buf, int bufsize )
{
  unsigned char data, memt;
  unsigned short end, i, bi;
  int nl;
  int rc;

  switch (memtype) {
    case AVR_FLASH  : 
      end = start+bufsize/2;
      memt = AVR_FLASH_LO;
      break;
    case AVR_EEPROM : 
      end = start+bufsize;
      memt = memtype;
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
    rc = avr_write_byte(fd, memt, i, data );
    fprintf(stderr, "                      \r%4u 0x%02x", i, data);
    if (rc) {
      fprintf(stderr, " ***failed;  ");
      nl = 1;
    }
    
    if (memtype == AVR_FLASH) {
      /* high byte of flash */
      data = buf[bi++];
      rc = avr_write_byte(fd, AVR_FLASH_HI, i, data );
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


int avr_program_enable ( int fd )
{
  unsigned char cmd[4] = {0xac, 0x53, 0x00, 0x00};
  unsigned char res[4];

  avr_cmd(fd, cmd, res);

  if (res[2] != cmd[1])
    return -1;

  return 0;
}


int avr_chip_erase ( int fd )
{
  unsigned char data[4] = {0xac, 0x80, 0x00, 0x00};
  unsigned char res[4];

  avr_cmd(fd, data, res);

  usleep(20000);

  return 0;
}


int avr_signature ( int fd, char sig[4] )
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


void avr_powerup ( int fd )
{
  ppi_set(fd, PPIDATA, AVR_POWER);    /* power up */
  usleep(100000);
}


void avr_powerdown ( int fd )
{
  ppi_clr(fd, PPIDATA, AVR_POWER);    /* power down */
}


int avr_initialize ( int fd )
{
  int rc;
  int tries;

  avr_powerup(fd);

  ppi_clr(fd, PPIDATA, AVR_CLOCK);
  ppi_clr(fd, PPIDATA, AVR_RESET);
  ppi_toggle(fd, PPIDATA, AVR_RESET);

  usleep(20000); /* 20 ms */

  tries = 0;
  do {
    rc = avr_program_enable ( fd );
    if (rc == 0)
      break;
    ppi_toggle(fd, PPIDATA, AVR_CLOCK);
    tries++;
  } while (tries < 32);
  if (tries == 32) {
    fprintf ( stderr, "%s: AVR device not responding\n", progname );
    return -1;
  }
  return 0;
}


int ppi_sense_test ( int fd )
{
  unsigned char v, pv;

  pv = 1;
  do {
    v = ppi_get(fd, PPISTATUS, AVR_DATA);
    if (v != pv) {
      fprintf ( stderr, "PPISTATUS bit = %d\n", v );
    }
    pv = v;
  } while(1);

  return 0;
}



/* vars for getopt() */
char *optarg;
int optind;
int optopt;
int opterr;
int optreset;


void usage ( void )
{
  fprintf ( stderr, 
            "\nUsage:  %s [-r] [-e|-f] [-u InputFile|-o Outputfile]\n"
            "\n"
            "  Available Options:\n"
            "    -r            : erase the flash and eeprom (required before programming)\n"
            "    -e            : select eeprom for reading or writing\n"
            "    -f            : select flash for reading or writing\n"
            "    -u InputFile  : write data from this file\n"
            "    -o OutputFile : write data to this file\n"
            "\n",
            progname );
}


int main ( int argc, char * argv [] )
{
  int fd;
  int rc;
  unsigned char buf[2048];
  unsigned char sig[4];
  int ch;
  int iofd;
  int flash, eeprom, doread, erase, dosig;
  int size;
  char * outputf;
  char * inputf;

  iofd    = -1;
  outputf = NULL;
  inputf  = NULL;
  doread  = 1;
  eeprom  = 0;
  flash   = 0;
  erase   = 0;
  dosig   = 0;

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];

  if (argc == 1) {
    usage();
    return 0;
  }

  while ((ch = getopt(argc,argv,"?efo:rsu:")) != -1) {
    switch (ch) {
      case 'e':
        if (flash) {
          fprintf(stderr,"%s: -e and -f are incompatible\n", progname);
          return 1;
        }
        eeprom = 1;
        break;
      case 'r':
        erase = 1;
        break;
      case 's':
        dosig = 1;
        break;
      case 'f':
        if (eeprom) {
          fprintf(stderr,"%s: -e and -f are incompatible\n", progname);
          return 1;
        }
        flash = 1;
        break;
      case 'o':
        if (inputf) {
          fprintf(stderr,"%s: -o and -u are incompatible\n", progname);
          return 1;
        }
        doread = 1;
        outputf = optarg;
        if (strcmp(outputf,"-")==0) {
          iofd = fileno(stdout);
        }
        else {
          iofd = open ( outputf, O_WRONLY|O_CREAT|O_TRUNC,
                        S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
          if (iofd < 0) {
            fprintf(stderr, "%s: can't open output file \"%s\": %s\n",
                    progname, outputf, strerror(errno));
            return 1;
          }
        }
        break;
      case 'u':
        if (outputf) {
          fprintf(stderr,"%s: -o and -u are incompatible\n", progname);
          return 1;
        }
        doread = 0;
        inputf = optarg;
        iofd = open ( inputf, O_RDONLY, 0);
        if (iofd < 0) {
          fprintf(stderr, "%s: can't open input file \"%s\": %s\n",
                  progname, inputf, strerror(errno));
          return 1;
        }
        break;
      case '?':
        usage();
        return 1;
        break;
      default:
        fprintf(stderr, "%s: invalid option -%c\n", progname, ch);
        usage();
        return 1;
        break;
    }
  }

  fd = open ( PARALLEL, O_RDWR );
  if (fd < 0) {
    fprintf ( stderr, "%s: can't open device \"%s\": %s\n",
              progname, PARALLEL, strerror(errno) );
    return 1;
  }

  fprintf ( stderr, "%s: initializing\n", progname );
  rc = avr_initialize(fd);
  if (rc < 0) {
    fprintf ( stderr, "%s: initialization failed, rc=%d\n", progname, rc );
    avr_powerdown(fd);
    return 1;
  }

  fprintf ( stderr, "%s: AVR device initialized and ready to accept instructions\n",
            progname );

  if (erase) {
    fprintf(stderr, "%s: erasing chip\n", progname );
    avr_chip_erase(fd);
    avr_initialize(fd);
    fprintf(stderr, "%s: done.\n", progname );
  }

  if (dosig) {
    int i;
    fprintf(stderr, "%s: reading signature bytes: ", progname );
    avr_signature(fd, sig);
    for (i=0; i<4; i++)
      fprintf(stderr, "0x%02x ", sig[i]);
    fprintf(stderr, "\n");
  }

  if (iofd < 0) {
    if (eeprom||flash) {
      fprintf(stderr, "%s: you must specify an input or an output file\n",
              progname);
    }
    avr_powerdown(fd);
    close(fd);
    return 1;
  }

  if (!(eeprom||flash)) {
    fprintf(stderr, 
            "%s: please specify either the eeprom (-e) or the flash (-f) memory\n",
            progname);
    avr_powerdown(fd);
    return 1;
  }

  if (doread) {
    /*
     * read device memory 
     */
    if (flash) {
      size = 2048;
      fprintf ( stderr, "%s: reading flash memory:\n", progname );
      rc = avr_read ( fd, AVR_FLASH, 0, size/2, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to read all of flash memory, rc=%d\n", 
                  progname, rc );
        avr_powerdown(fd);
        return 1;
      }
    }
    else if (eeprom) {
      size = 128;
      fprintf ( stderr, "%s: reading eeprom memory:\n", progname );
      rc = avr_read ( fd, AVR_EEPROM, 0, size, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to read all of eeprom memory, rc=%d\n", 
                  progname, rc );
        avr_powerdown(fd);
        return 1;
      }
    }

    rc = write ( iofd, buf, size );
    if (rc < 0) {
      fprintf(stderr, "%s: write error: %s\n", progname, strerror(errno));
      avr_powerdown(fd);
      return 1;
    }
    else if (rc != size) {
      fprintf(stderr, "%s: wrote only %d bytes of the expected %d\n", 
              progname, rc, size);
      avr_powerdown(fd);
      return 1;
    }
  }
  else {
    /*
     * write device memory 
     */
    if (flash) {
      size = 2048;
    }
    else if (eeprom) {
      size = 128;
    }

    /* read in the data file */
    rc = read(iofd, buf, size);
    if (rc < 0) {
      fprintf(stderr, "%s: read error from \"%s\": %s\n", 
              progname, inputf, strerror(errno));
      avr_powerdown(fd);
      return 1;
    }

    size = rc;

    if (flash) {
      fprintf(stderr, "%s: writing %d bytes into flash memory:\n", 
              progname, size);
      rc = avr_write ( fd, AVR_FLASH, 0, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to write flash memory, rc=%d\n", 
                  progname, rc );
        avr_powerdown(fd);
        return 1;
      }
    }
    else if (eeprom) {
      fprintf(stderr, "%s: writing %d bytes into eeprom memory:\n", 
              progname, size);
      rc = avr_write ( fd, AVR_EEPROM, 0, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to write eeprom memory, rc=%d\n", 
                  progname, rc );
        avr_powerdown(fd);
        return 1;
      }
    }
  }

  avr_powerdown(fd);

  close(fd);
  close(iofd);

  return 0;
}


