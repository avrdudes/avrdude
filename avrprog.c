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
 *    Pin 18       <-   GND
 *
 */

#include <stdio.h>
#include <stdlib.h>
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

char * version = "$Id$";

char * progname;


/*
 * bit definitions for AVR device connections
 */
#define AVR_POWER 0x01  /* bit 0 of data register */
#define AVR_CLOCK 0x02  /* bit 1 of data register */
#define AVR_INSTR 0x04  /* bit 2 of data register */
#define AVR_RESET 0x08  /* bit 3 of data register */
#define AVR_DATA  0x40  /* bit 6 of status register */


/*
 * PPI registers
 */
enum {
  PPIDATA,
  PPICTRL,
  PPISTATUS
};


/*
 * AVR memory designations
 */
typedef enum {
  AVR_EEPROM,
  AVR_FLASH,
  AVR_FLASH_LO,
  AVR_FLASH_HI
} AVRMEM;


struct avrpart {
  char * partdesc;
  char * optiontag;
  int flash_size;
  int eeprom_size;
};


struct avrpart parts[] = {
  { "AT90S8515", "8515", 8192, 512 },
  { "AT90S2313", "2313", 2048, 128 }
};



/*
 * variable declarations required for getopt() 
 */
char *optarg;
int optind;
int optopt;
int opterr;
int optreset;


/*
 * set 'get' and 'set' appropriately for subsequent passage to ioctl()
 * to get/set the specified PPI registers.  
 */
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


/*
 * set the indicated bit of the specified register.
 */
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


/*
 * clear the indicated bit of the specified register.
 */
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


/*
 * get the indicated bit of the specified register.
 */
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


/*
 * toggle the indicated bit of the specified register.
 */
int ppi_toggle ( int fd, int reg, int bit )
{
  unsigned char v;
  unsigned long get, set;
  int rc;

  rc = ppi_getops ( reg, &get, &set );
  if (rc)
    return -1;

  ioctl(fd, get, &v);
  v ^= bit;
  ioctl(fd, set, &v);

  return 0;
}


/*
 * pulse the indicated bit of the specified register.
 */
int ppi_pulse ( int fd, int reg, int bit )
{
  ppi_toggle(fd, reg, bit);
  ppi_toggle(fd, reg, bit);

  return 0;
}


/*
 * transmit and receive a bit of data to/from the AVR device
 */
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
unsigned char avr_read_byte ( int fd, AVRMEM memtype, unsigned short addr )
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
      /* addr &= 0x7f; */
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


/*
 * read 'n' words of data from the indicated memory region.  If the
 * flash memory is being read, n*2 bytes will be read into 'buf'; if
 * the eeprom is being read, 'n' bytes will be read into 'buf'.
 */
int avr_read ( int fd, AVRMEM memtype, unsigned start, unsigned n, 
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


/*
 * write a byte of data to the indicated memory region
 */
int avr_write_byte ( int fd, AVRMEM memtype, unsigned short addr, unsigned char data )
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
      /* addr &= 0x7f; */
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


/*
 * write 'bufsize' bytes of data to the indicated memory region.
 */
int avr_write ( int fd, AVRMEM memtype, unsigned start, 
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
int avr_chip_erase ( int fd )
{
  unsigned char data[4] = {0xac, 0x80, 0x00, 0x00};
  unsigned char res[4];

  avr_cmd(fd, data, res);

  usleep(20000);

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
int avr_initialize ( int fd )
{
  int rc;
  int tries;

  avr_powerup(fd);

  ppi_clr(fd, PPIDATA, AVR_CLOCK);
  ppi_clr(fd, PPIDATA, AVR_RESET);
  ppi_pulse(fd, PPIDATA, AVR_RESET);

  usleep(20000); /* 20 ms */

  /*
   * enable programming mode, try up to 32 times in order to possibly
   * get back into sync with the chip if we are out of sync.  
   */
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

  return 0;
}



/*
 * infinite loop, sensing on the pin that we use to read data out of
 * the device; this is a debugging aid, you can insert a call to this
 * function in 'main()' and can use it to determine whether your sense
 * pin is actually sensing.  
 */
int ppi_sense_test ( int fd )
{
  unsigned char v, pv;

  pv = 1;
  do {
    usleep(100000); /* check every 100 ms */
    v = ppi_get(fd, PPISTATUS, AVR_DATA);
    if (v != pv) {
      fprintf ( stderr, "sense bit = %d\n", v );
    }
    pv = v;
  } while(1);

  return 0;
}


/*
 * usage message
 */
void usage ( void )
{
  fprintf ( stderr, 
            "\nUsage:  %s [-r] [-e|-f] [-u InputFile|-o Outputfile]\n"
            "\n"
            "  Available Options:\n"
            "    -r            : erase the flash and eeprom (required before programming)\n"
            "    -e            : select eeprom for reading or writing\n"
            "    -f            : select flash for reading or writing\n"
            "    -p Part       : 8515 or 2313\n"
            "    -u InputFile  : write data from this file\n"
            "    -o OutputFile : write data to this file\n"
            "\n",
            progname );
}


/*
 * main routine
 */
int main ( int argc, char * argv [] )
{
  int fd;
  int rc, exitrc;
  int i;
  unsigned char * buf;
  unsigned char sig[4];
  int ch;
  int iofd;
  int flash, eeprom, doread, erase, dosig;
  int size;
  char * outputf;
  char * inputf;
  char * p1, * p2;
  struct avrpart * p;

  iofd    = -1;
  outputf = NULL;
  inputf  = NULL;
  doread  = 1;
  eeprom  = 0;
  flash   = 0;
  erase   = 0;
  dosig   = 0;
  p       = NULL;

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];


  /*
   * Print out an identifying string so folks can tell what version
   * they are running
   */
  p1 = strchr(version,',');
  if (p1 == NULL)
    p1 = version;
  else
    p1 += 3;

  p2 = strrchr(p1,':');
  if (p2 == NULL)
    p2 = &p1[strlen(p1)];
  else
    p2 += 3;

  fprintf(stderr, "\n");
  fprintf(stderr, "AVRProg: Copyright 2000 Brian Dean, bsd@bsdhome.com\n");
  fprintf(stderr, "         Revision " );
  for (i=0; i<p2-p1; i++)
    fprintf(stderr, "%c", p1[i]);
  fprintf(stderr, "\n\n");

  /*
   * check for no arguments
   */
  if (argc == 1) {
    usage();
    return 0;
  }


  /*
   * process command line arguments
   */
  while ((ch = getopt(argc,argv,"?efo:p:rsu:")) != -1) {
    switch (ch) {
      case 'e': /* select eeprom memory */
        if (flash) {
          fprintf(stderr,"%s: -e and -f are incompatible\n", progname);
          return 1;
        }
        eeprom = 1;
        break;
      case 'r': /* perform a chip erase */
        erase = 1;
        break;
      case 's': /* read out the signature bytes */
        dosig = 1;
        break;
      case 'f': /* select flash memory */
        if (eeprom) {
          fprintf(stderr,"%s: -e and -f are incompatible\n", progname);
          return 1;
        }
        flash = 1;
        break;
      case 'o': /* specify output file */
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
      case 'p' : /* specify AVR part */
        p = NULL;
        for (i=0; i<sizeof(parts)/sizeof(parts[0]); i++) {
          if (strcmp(parts[i].optiontag, optarg)==0) {
            p = &parts[i];
            break;
          }
        }
        if (p == NULL) {
          fprintf(stderr, "%s: AVR Part \"%s\" not found.  Valid parts are:\n\n",
                  progname, optarg );
          for (i=0; i<sizeof(parts)/sizeof(parts[0]); i++) {
            fprintf(stderr, "    \"%s\" = %s\n", 
                    parts[i].optiontag, parts[i].partdesc);
          }
          fprintf(stderr, "\n");
          return 1;
        }
        break;
      case 'u': /* specify input (upload) file */
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
      case '?': /* help */
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


  if (p == NULL) {
    fprintf(stderr, 
            "%s: No AVR part has been specified, use \"-p Part\"\n\n"
            "  Valid Parts are:\n\n",
            progname );
    for (i=0; i<sizeof(parts)/sizeof(parts[0]); i++) {
      fprintf(stderr, "    \"%s\" = %s\n", 
              parts[i].optiontag, parts[i].partdesc);
    }
    fprintf(stderr,"\n");
    return 1;
  }

  fprintf(stderr, "%s: Using AVR Part %s: flash=%d, eeprom=%d\n",
          progname, p->partdesc, p->flash_size, p->eeprom_size);
  fprintf(stderr, "\n");

  if (p->flash_size >= p->eeprom_size)
    size = p->flash_size;
  else
    size = p->eeprom_size;

  buf = (unsigned char *) malloc(size);
  if (buf == NULL) {
    fprintf(stderr, 
            "%s: out of memory allocating %d bytes for on-chip memory buffer\n",
            progname, size);
    return 1;
  }


  /*
   * open the parallel port
   */
  fd = open ( PARALLEL, O_RDWR );
  if (fd < 0) {
    fprintf ( stderr, "%s: can't open device \"%s\": %s\n",
              progname, PARALLEL, strerror(errno) );
    return 1;
  }

  exitrc = 0;


  /*
   * initialize the chip in preperation for accepting commands
   */
  rc = avr_initialize(fd);
  if (rc < 0) {
    fprintf ( stderr, "%s: initialization failed, rc=%d\n", progname, rc );
    exitrc = 1;
    goto main_exit;
  }

  fprintf ( stderr, "%s: AVR device initialized and ready to accept instructions\n",
            progname );


  if (erase) {
    /*
     * erase the chip's flash and eeprom memories, this is required
     * before the chip can accept new programming
     */
    fprintf(stderr, "%s: erasing chip\n", progname );
    avr_chip_erase(fd);
    avr_initialize(fd);
    fprintf(stderr, "%s: done.\n", progname );
  }


  if (dosig) {
    /*
     * read out the on-chip signature bytes
     */
    fprintf(stderr, "%s: reading signature bytes: ", progname );
    avr_signature(fd, sig);
    for (i=0; i<4; i++)
      fprintf(stderr, "0x%02x ", sig[i]);
    fprintf(stderr, "\n");
  }


  if (iofd < 0) {
    /*
     * Check here to see if any other operations were selected and
     * generate an error message because if they were, we need either
     * an input or and output file, but one was not selected.
     * Otherwise, we just shut down.  
     */
    if (eeprom||flash) {
      fprintf(stderr, "%s: you must specify an input or an output file\n",
              progname);
    }
    exitrc = 1;
    goto main_exit;
  }


  if (!(eeprom||flash)) {
    /*
     * an input file or an output file was specified, but the memory
     * type (eeprom or flash) was not specified.  
     */
    fprintf(stderr, 
            "%s: please specify either the eeprom (-e) or the flash (-f) memory\n",
            progname);
    exitrc = 1;
    goto main_exit;
  }


  if (doread) {
    /*
     * read out the specified device memory and write it to a file 
     */
    if (flash) {
      size = p->flash_size;
      fprintf ( stderr, "%s: reading flash memory:\n", progname );
      rc = avr_read ( fd, AVR_FLASH, 0, size/2, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to read all of flash memory, rc=%d\n", 
                  progname, rc );
        exitrc = 1;
        goto main_exit;
      }
    }
    else if (eeprom) {
      size = p->eeprom_size;
      fprintf ( stderr, "%s: reading eeprom memory:\n", progname );
      rc = avr_read ( fd, AVR_EEPROM, 0, size, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to read all of eeprom memory, rc=%d\n", 
                  progname, rc );
        exitrc = 1;
        goto main_exit;
      }
    }

    /*
     * write it out to the specified file
     */
    rc = write ( iofd, buf, size );
    if (rc < 0) {
      fprintf(stderr, "%s: write error: %s\n", progname, strerror(errno));
      exitrc = 1;
      goto main_exit;
    }
    else if (rc != size) {
      fprintf(stderr, "%s: wrote only %d bytes of the expected %d\n", 
              progname, rc, size);
      exitrc = 1;
      goto main_exit;
    }
  }
  else {
    /*
     * write the selected device memory using data from a file
     */
    if (flash) {
      size = p->flash_size;
    }
    else if (eeprom) {
      size = p->eeprom_size;
    }

    /*
     * read in the data file that will be used to write into the chip
     */
    rc = read(iofd, buf, size);
    if (rc < 0) {
      fprintf(stderr, "%s: read error from \"%s\": %s\n", 
              progname, inputf, strerror(errno));
      exitrc = 1;
      goto main_exit;
    }

    size = rc;

    /*
     * write the buffer contents to the selected memory type
     */
    if (flash) {
      fprintf(stderr, "%s: writing %d bytes into flash memory:\n", 
              progname, size);
      rc = avr_write ( fd, AVR_FLASH, 0, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to write flash memory, rc=%d\n", 
                  progname, rc );
        exitrc = 1;
        goto main_exit;
      }
    }
    else if (eeprom) {
      fprintf(stderr, "%s: writing %d bytes into eeprom memory:\n", 
              progname, size);
      rc = avr_write ( fd, AVR_EEPROM, 0, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to write eeprom memory, rc=%d\n", 
                  progname, rc );
        exitrc = 1;
        goto main_exit;
      }
    }
  }

 main_exit:

  /*
   * program complete
   */

  avr_powerdown(fd);
  ppi_clr(fd, PPIDATA, 0xff);
  ppi_set(fd, PPIDATA, AVR_RESET);

  close(fd);
  close(iofd);

  fprintf(stderr, "\n" );

  return exitrc;
}

