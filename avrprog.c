/*
 * Copyright 2000, Brian Dean
 * All Rights Reserved.
 */

/* $Id$ */

/*
 * Code to program an Atmel AVR device using the parallel port.
 *
 * Make the following connections:
 *
 *  Pin  2 -> PB7(SCK)  CLOCK IN             (data bit 0)
 *  Pin  3 -> PB5(MOSI) Instruction input    (data bit 1)
 *  Pin  4 -> /RESET                         (data bit 2)
 *  Pin 10 <- PB6(MISO) Data out             (status bit 6)
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


#define AVR_CLOCK 0x01  /* bit 0 of data register */
#define AVR_INSTR 0x02  /* bit 1 of data register */
#define AVR_RESET 0x04  /* bit 2 of data register */
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


int dprintf ( FILE * f, char * format, ... )
{
#if DEBUG
  va_list ap;
  int rc;

  va_start(ap,format);
  rc = vfprintf(f,format,ap);
  va_end(ap);

  return rc;
#else
  return 0;
#endif
}


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


unsigned char avr_read_byte ( int fd, unsigned short addr, int memtype )
{
  unsigned char r;

  switch (memtype) {
    case AVR_FLASH_LO: 
      avr_txrx(fd, 0x20); 
      break;
    case AVR_FLASH_HI: 
      avr_txrx(fd, 0x28); 
      break;
    case AVR_EEPROM: 
      avr_txrx(fd, 0xa0); 
      addr &= 0x7f;
      break;
    default:
      fprintf(stderr, "%s: avr_read_byte(); internal error: invalid memtype=%d\n",
              progname, memtype);
      exit(1);
      break;
  }

  avr_txrx(fd, addr >> 8);     /* high order bits of address       */
  avr_txrx(fd, addr & 0x0ff);  /* low order bits of address        */
  r = avr_txrx(fd, 0);         /* don't care                       */

  return r;
}


int avr_read ( int fd, int memtype, unsigned start, unsigned n, 
               unsigned char * buf, int bufsize )
{
  unsigned char rbyte;
  unsigned short data;
  unsigned short end, i, bi;

  switch (memtype) {
    case AVR_FLASH  : 
    case AVR_EEPROM : 
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
    if (memtype == AVR_FLASH) {
      rbyte = avr_read_byte(fd, i, AVR_FLASH_LO);  /* flash low byte */
      fprintf ( stderr, "                    \r%4u  0x%02x", i, rbyte );
      data = rbyte;
      if (bi < bufsize) {
        buf[bi++] = rbyte;
      }

      rbyte = avr_read_byte(fd, i, AVR_FLASH_HI);  /* flash high byte */
      fprintf ( stderr, " 0x%02x", rbyte );
      data |= (rbyte << 8);
      if (bi < bufsize) {
        buf[bi++] = rbyte;
      }
    }
    else {
      rbyte = avr_read_byte(fd, i, memtype);  /* eeprom byte */
      fprintf ( stderr, "          \r%4u  0x%02x", i, rbyte );
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
  unsigned char r;
  int ready;
  int tries;

  switch (memtype) {
    case AVR_FLASH_LO: 
      avr_txrx(fd, 0x40); 
      break;
    case AVR_FLASH_HI: 
      avr_txrx(fd, 0x48); 
      break;
    case AVR_EEPROM: 
      avr_txrx(fd, 0xc0); 
      addr &= 0x7f;
      break;
    default:
      fprintf(stderr, "%s: avr_write_byte(); internal error: invalid memtype=%d\n",
              progname, memtype);
      exit(1);
      break;
  }

  avr_txrx(fd, addr >> 8);     /* high order bits of address       */
  avr_txrx(fd, addr & 0x0ff);  /* low order bits of address        */
  avr_txrx(fd, data);          /* data                             */

  tries = 0;
  ready = 0;
  while (!ready) {
    usleep(5000);               /* flash write delay */
    r = avr_read_byte ( fd, addr, memtype );
    if (data == 0x7f) {
      usleep(20000);
      ready = 1;
    }
    else if (r == data) {
      ready = 1;
    }
    tries++;
    if (!ready && tries > 10) {
      fprintf(stderr, "**" );
      ready = 1;
    }
  }

  return 0;
}


int avr_write ( int fd, int memtype, unsigned start, 
                unsigned char * buf, int size )
{
  unsigned char data;
  unsigned short end, i, bi;

  switch (memtype) {
    case AVR_FLASH  : 
      end = start+size/2;
      break;
    case AVR_EEPROM : 
      end = start+size;
      break;
    default:
      fprintf(stderr, "%s: avr_write(); internal error: invalid memtype=%d\n",
              progname, memtype);
      exit(1);
      break;
  }

  bi = 0;

  for (i=start; i<end; i++) {
    if (memtype == AVR_FLASH) {
      /* low byte */
      data = buf[bi++];
      avr_write_byte(fd, AVR_FLASH_LO, i, data );
      fprintf ( stderr, "                      \r%4u 0x%02x", i, data );

      /* high byte */
      data = buf[bi++];
      avr_write_byte(fd, AVR_FLASH_HI, i, data );
      fprintf ( stderr, " 0x%02x", data );
    }
    else {
      data = buf[bi++];
      avr_write_byte(fd, memtype, i, data );
      fprintf ( stderr, "            \r%4u 0x%02x", i, data );
    }

  }

  fprintf ( stderr, "\n" );

  return 0;
}


int avr_program_enable ( int fd )
{
  unsigned char data[4] = {0xac, 0x53, 0x00, 0x00};
  unsigned char byte, rbyte;
  int avrok;
  int i;

  avrok = 0;

  for (i=0; i<4; i++) {
    byte = data[i];
    rbyte = avr_txrx ( fd, byte );
    if (i == 2) {
      if (rbyte == data[1])
        avrok = 1;
    }
  }

  if (!avrok)
    return -1;

  return 0;
}


int avr_chip_erase ( int fd )
{
  unsigned char data[4] = {0xac, 0x80, 0x00, 0x00};
  int i;

  for (i=0; i<4; i++) {
    avr_txrx ( fd, data[i] );
  }

  usleep(20000);

  return 0;
}


int avr_initialize ( int fd )
{
  int rc;
  int tries;

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
  int ch;
  int iofd;
  int flash, eeprom, doread, erase;
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

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];

  while ((ch = getopt(argc,argv,"?efo:ru:")) != -1) {
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

  if (iofd < 0) {
    fprintf(stderr, "%s: you must specify an input or an output file\n",
            progname);
    return 1;
  }

  if (!(eeprom||flash)) {
    fprintf(stderr, 
            "%s: please specify either the eeprom (-e) or the flash (-f) memory\n",
            progname);
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
        return 1;
      }
    }

    rc = write ( iofd, buf, size );
    if (rc < 0) {
      fprintf(stderr, "%s: write error: %s\n", progname, strerror(errno));
      return 1;
    }
    else if (rc != size) {
      fprintf(stderr, "%s: wrote only %d bytes of the expected %d\n", 
              progname, rc, size);
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
      return 1;
    }
    else if (rc != size) {
      fprintf(stderr, "%s: read only %d bytes of the expected %d from \"%s\"\n", 
              progname, rc, size, inputf);
      return 1;
    }

    if (flash) {
      fprintf ( stderr, "%s: writing flash memory:\n", progname );
      rc = avr_write ( fd, AVR_FLASH, 0, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to write flash memory, rc=%d\n", 
                  progname, rc );
        return 1;
      }
    }
    else if (eeprom) {
      fprintf ( stderr, "%s: writing eeprom memory:\n", progname );
      rc = avr_write ( fd, AVR_EEPROM, 0, buf, size );
      if (rc) {
        fprintf ( stderr, "%s: failed to write eeprom memory, rc=%d\n", 
                  progname, rc );
        return 1;
      }
    }
  }

  close(fd);
  close(iofd);

  return 0;
}


