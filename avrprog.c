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
 *    Pin  2       ->   Vcc      (see NOTE below)
 *    Pin  3       ->   SCK      CLOCK IN
 *    Pin  4       ->   MOSI     Instruction input
 *    Pin  5       ->   /RESET
 *    Pin  6,7,8,9 ->   Vcc      (Can be tied together with Schottky diodes)
 *    Pin 10       <-   MISO     Data out
 *    Pin 18       <-   GND
 *
 *  NOTE on Vcc connection: make sure your parallel port can supply an
 *  adequate amount of current to power your device.  6-10 mA is
 *  common for parallel port signal lines, but is not guaranteed,
 *  especially for notebook computers.  Optionally, you can tie pins
 *  6, 7, 8, and 9 also to Vcc with Schottky diodes to supply
 *  additional current.  If in doubt, don't risk damaging your
 *  parallel port, use an external power supply.
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
#include <limits.h>
#include <ctype.h>

#define DEFAULT_PARALLEL "/dev/ppi0"

char * version = "$Id$";

char * progname;


/*
 * bit definitions for AVR device connections
 */
#define AVR_POWER 0xf1  /* bit 0 and 4...7 of data register */
#define AVR_CLOCK 0x02  /* bit 1 of data register           */
#define AVR_INSTR 0x04  /* bit 2 of data register           */
#define AVR_RESET 0x08  /* bit 3 of data register           */
#define AVR_DATA  0x40  /* bit 6 of status register         */


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

typedef enum {
  FMT_AUTO,
  FMT_SREC,
  FMT_IHEX,
  FMT_RBIN
} FILEFMT;

struct avrpart {
  char          * partdesc;         /* long part name */
  char          * optiontag;        /* short part name */
  int             flash_size;       /* size in bytes of flash */
  int             eeprom_size;      /* size in bytes of eeprom */
  unsigned char   f_readback;       /* flash write polled readback value */
  unsigned char   e_readback[2];    /* eeprom write polled readback values */
  int             min_write_delay;  /* microseconds */
  int             max_write_delay;  /* microseconds */
  int             chip_erase_delay; /* microseconds */
  unsigned char * flash;
  unsigned char * eeprom;
};


struct avrpart parts[] = {
  { "AT90S8515", "8515", 8192, 512, 0x7f, { 0x80, 0x7f },
    9000, 20000, 20000, NULL, NULL },

  { "AT90S2313", "2313", 2048, 128, 0x7f, { 0x80, 0x7f }, 
    9000, 20000, 20000, NULL, NULL },

  { "AT90S1200", "1200", 1024,  64, 0x7f, { 0x80, 0x7f }, 
    9000, 20000, 20000, NULL, NULL }
};

#define N_AVRPARTS (sizeof(parts)/sizeof(struct avrpart))


struct fioparms {
  int    op;
  char * mode;
  char * iodesc;
  char * dir;
  char * rw;
};

enum {
  FIO_READ,
  FIO_WRITE
};



#define MAX_LINE_LEN 256  /* max line length for ASCII format input files */


char * usage_text =
"\n"
"Usage:  avrprog [options]\n"
"\n"
"  Available Options:\n"
"\n"
"    -m MemType    : select memory type for reading or writing\n"
"                      \"e\", \"eeprom\" = EEPROM\n"
"                      \"f\", \"flash\"  = FLASH (default)\n"
"\n"
"    -i Filename   : select input file, \"-\" = stdin\n"
"\n"
"    -o Filename   : select output file, \"-\" = stdout\n"
"\n"
"    -f Format     : select input / output file format\n"
"                      \"i\" = Intel Hex\n"
"                      \"s\" = Motorola S-Record\n"
"                      \"r\" = Raw binary (default for output)\n"
"                      \"a\" = Auto detect (default for input)\n"
"                              (valid for input only)\n"
"                              \n"
"\n"
"    -p Part       : select Atmel part number (see below for valid parts)\n"
"\n"
"    -P Parallel   : select parallel port device name (default = /dev/ppi0)\n"
"\n"
"    -F            : override invalid device signature check\n"
"\n"
"    -c            : enter interactive command mode (or read commands\n"
"                    from stdin)\n"
"\n"
"    -e            : perform a chip erase (required before programming)\n"
"\n";



int list_valid_parts ( FILE * f, char * prefix )
{
  int i;

  for (i=0; i<N_AVRPARTS; i++) {
    fprintf(f, "%s%s = %s\n", 
            prefix, parts[i].optiontag, parts[i].partdesc);
  }

  return i;
}




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
    if (data != 0xff)
      rc = avr_write_byte(fd, p, memt, i, data );
    else
      rc = 0;
    fprintf(stderr, "                      \r%4u 0x%02x", i, data);
    if (rc) {
      fprintf(stderr, " ***failed;  ");
      nl = 1;
    }
    
    if (memtype == AVR_FLASH) {
      /* high byte of flash */
      data = buf[bi++];
      if (data != 0xff)
        rc = avr_write_byte(fd, p, AVR_FLASH_HI, i, data );
      else
        rc = 0;
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

  usleep(20000); /* 20 ms */

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

  fprintf ( stderr, "%s", usage_text );

  fprintf(stderr, "  Valid Parts for the -p option are:\n");
  list_valid_parts(stderr, "    ");
  fprintf(stderr, "\n");

}


char * fmtstr ( FILEFMT format )
{
  switch (format) {
    case FMT_AUTO : return "auto-detect"; break;
    case FMT_SREC : return "Motorola S-Record"; break;
    case FMT_IHEX : return "Intel Hex"; break;
    case FMT_RBIN : return "raw binary"; break;
    default       : return "invalid format"; break;
  };
}



int b2ihex ( unsigned char * inbuf, int bufsize, 
             int recsize, int startaddr,
             char * outfile, FILE * outf )
{
  unsigned char * buf;
  unsigned int nextaddr;
  int n;
  int i;
  unsigned char cksum;

  if (recsize > 255) {
    fprintf ( stderr, "%s: recsize=%d, must be < 256\n",
              progname, recsize );
    return -1;
  }

  nextaddr = startaddr;

  buf = inbuf;
  while (bufsize) {
    n = recsize;
    if (n > bufsize)
      n = bufsize;

    if (n) {
      cksum = 0;
      fprintf ( outf, ":%02X%04X00", n, nextaddr );
      cksum += n + ((nextaddr >> 8) & 0x0ff) + (nextaddr & 0x0ff);
      for (i=0; i<n; i++) {
        fprintf ( outf, "%02X", buf[i] );
        cksum += buf[i];
      }
      cksum = -cksum;
      fprintf ( outf, "%02X\n", cksum );
      
      nextaddr += n;
    }

    /* advance to next 'recsize' bytes */
    buf += n;
    bufsize -= n;
  }

  /*-----------------------------------------------------------------
    add the trailing zero data line
    -----------------------------------------------------------------*/
  cksum = 0;
  n = 0;
  fprintf ( outf, ":%02X%04X00", n, nextaddr );
  cksum += n + ((nextaddr >> 8) & 0x0ff) + (nextaddr & 0x0ff);
  cksum = -cksum;
  fprintf ( outf, "%02X\n", cksum );

  return 0;
}


int ihex2b ( char * infile, FILE * inf,
             unsigned char * outbuf, int bufsize )
{
  unsigned char buffer [ MAX_LINE_LEN ];
  unsigned char * buf;
  unsigned int prevaddr, nextaddr;
  unsigned int b;
  int n;
  int i, j;
  unsigned int cksum, rectype;
  int lineno;

  lineno   = 0;
  prevaddr = 0;
  buf      = outbuf;

  while (fgets((char *)buffer,MAX_LINE_LEN,inf)!=NULL) {
    lineno++;
    if (buffer[0] != ':')
      continue;
    if (sscanf((char *)&buffer[1], 
               "%02x%04x%02x", &n, &nextaddr, &rectype) != 3) {
      fprintf(stderr, "%s: invalid record at line %d of \"%s\"\n",
              progname, lineno, infile);
      exit(1);
    }

    if (rectype != 0) {
      fprintf(stderr, 
              "%s: don't know how to deal with rectype=%d " 
              "at line %d of %s\n",
              progname, rectype, lineno, infile);
      exit(1);
    }

    if (n && ((nextaddr + n) > bufsize)) {
      fprintf(stderr, "%s: address 0x%04x out of range at line %d of %s\n",
              progname, nextaddr+n, lineno, infile);
      return -1;
    }

    /* start computing a checksum */
    cksum = n + ((nextaddr >> 8 ) & 0x0ff) + (nextaddr & 0x0ff);

    for (i=0; i<n; i++) {
      if (sscanf((char *)&buffer[i*2+9], "%02x", &b) != 1) {
        fprintf(stderr, "%s: can't scan byte number %d at line %d of %s\n",
                progname, i, lineno, infile);
        /* display the buffer and the position of the scan error */
        fprintf(stderr, "%s", buffer);
        for (j=0; j<9+2*i; j++) {
          fprintf(stderr, " ");
        }
        fprintf(stderr, "^\n");
        return -1;
      }

      buf[nextaddr + i] = b;
      cksum += b;
    }

    /*-----------------------------------------------------------------
      read the cksum value from the record and compare it with our
      computed value
      -----------------------------------------------------------------*/
    if (sscanf((char *)&buffer[n*2+9], "%02x", &b) != 1) {
      fprintf(stderr, "%s: can't scan byte number %d at line %d of %s\n",
              progname, i, lineno, infile);
      /* display the buffer and the position of the scan error */
      fprintf(stderr, "%s", buffer);
      for (j=0; j<9+2*i; j++) {
        fprintf(stderr, " ");
      }
      fprintf(stderr, "^\n");
      return -1;
    }

    cksum = -cksum & 0xff;
    if (cksum != b) {
      fprintf(stderr, 
              "%s: cksum error for line %d of \"%s\": computed=%02x "
              "found=%02x\n",
              progname, lineno, infile, cksum, b);
      return -1;
    }

    prevaddr = nextaddr + n;
  }

  return 0;
}



int fileio_rbin ( struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size )
{
  int rc;

  switch (fio->op) {
    case FIO_READ:
      rc = fread(buf, 1, size, f);
      break;
    case FIO_WRITE:
      rc = fwrite(buf, 1, size, f);
      break;
  }

  if (rc < size) {
    fprintf(stderr, 
            "%s: %s error %s %s: %s; %s %d of the expected %d bytes\n", 
            progname, fio->iodesc, fio->dir, filename, strerror(errno),
            fio->rw, rc, size);
    return -1;
  }

  return rc;
}


int fileio_ihex ( struct fioparms * fio, 
                  char * filename, FILE * f, unsigned char * buf, int size )
{
  int rc;

  switch (fio->op) {
    case FIO_WRITE:
      rc = b2ihex(buf, size, 32, 0, filename, f);
      if (rc) {
        return -1;
      }
      break;

    case FIO_READ:
      rc = ihex2b(filename, f, buf, size);
      if (rc)
        return -1;
      break;

    default:
      fprintf(stderr, "%s: invalid Intex Hex file I/O operation=%d\n",
              progname, fio->op);
      return -1;
      break;
  }

  return 0;
}


int fileio_srec ( struct fioparms * fio,
                  char * filename, FILE * f, unsigned char * buf, int size )
{
  fprintf(stderr, "%s: Motorola S-Record %s format not yet supported\n",
          progname, fio->iodesc);
  return -1;
}


int fileio_setparms ( int op, struct fioparms * fp )
{
  fp->op = op;

  switch (op) {
    case FIO_READ:
      fp->mode   = "r";
      fp->iodesc = "input";
      fp->dir    = "from";
      fp->rw     = "read";
      break;

    case FIO_WRITE:
      fp->mode   = "w";
      fp->iodesc = "output";
      fp->dir    = "to";
      fp->rw     = "wrote";
      break;

    default:
      fprintf(stderr, "%s: invalid I/O operation %d\n",
              progname, op);
      return -1;
      break;
  }

  return 0;
}



int fmt_autodetect ( char * fname )
{
  FILE * f;
  unsigned char buf[MAX_LINE_LEN];
  int i;
  int len;
  int found;

  f = fopen(fname, "r");
  if (f == NULL) {
    fprintf(stderr, "%s: error opening %s: %s\n",
            progname, fname, strerror(errno));
    return -1;
  }

  while (fgets((char *)buf, MAX_LINE_LEN, f)!=NULL) {
    buf[MAX_LINE_LEN-1] = 0;
    len = strlen((char *)buf);
    if (buf[len-1] == '\n')
      buf[--len] = 0;

    /* check for binary data */
    found = 0;
    for (i=0; i<len; i++) {
      if (buf[i] > 127) {
        found = 1;
        break;
      }
    }
    if (found)
      return FMT_RBIN;

    /* check for lines that look like intel hex */
    if ((buf[0] == ':') && (len >= 11)) {
      found = 1;
      for (i=1; i<len; i++) {
        if (!isxdigit(buf[1])) {
          found = 0;
          break;
        }
      }
      if (found)
        return FMT_IHEX;
    }

    /* check for lines that look like motorola s-record */
    if ((buf[0] == 'S') && (len >= 10) && isdigit(buf[1])) {
      found = 1;
      for (i=1; i<len; i++) {
        if (!isxdigit(buf[1])) {
          found = 0;
          break;
        }
      }
      if (found)
        return FMT_SREC;
    }
  }

  return -1;
}



int fileio ( int op, char * filename, FILEFMT format, 
             struct avrpart * p, AVRMEM memtype )
{
  int rc;
  FILE * f;
  char * fname;
  unsigned char * buf;
  int size;
  struct fioparms fio;
  int i;

  rc = fileio_setparms(op, &fio);
  if (rc < 0)
    return -1;

  if (strcmp(filename, "-")==0) {
    if (fio.op == FIO_READ) {
      fname = "<stdin>";
      f = stdin;
    }
    else {
      fname = "<stdout>";
      f = stdout;
    }
  }
  else {
    fname = filename;
    f = fopen(fname, fio.mode);
    if (f == NULL) {
      fprintf(stderr, "%s: can't open %s file %s: %s\n",
              progname, fio.iodesc, fname, strerror(errno));
      return -1;
    }
  }

  switch (memtype) {
    case AVR_EEPROM:
      buf = p->eeprom;
      size = p->eeprom_size;
      break;

    case AVR_FLASH:
      buf = p->flash;
      size = p->flash_size;
      break;
      
    default:
      fprintf(stderr, "%s: invalid memory type for %s: %d\n",
              progname, fio.iodesc, memtype);
      return -1;
  }

  if (fio.op == FIO_READ) {
    /* 0xff fill unspecified memory */
    for (i=0; i<size; i++) {
      buf[i] = 0xff;
    }
  }

  if (format == FMT_AUTO) {
    format = fmt_autodetect(fname);
    if (format < 0) {
      fprintf(stderr, 
              "%s: can't determine file format for %s, specify explicitly\n",
              progname, fname);
      return -1;
    }

    fprintf(stderr, "%s: %s file %s auto detected as %s\n\n", 
            progname, fio.iodesc, fname, fmtstr(format));
  }

  switch (format) {
    case FMT_IHEX:
      rc = fileio_ihex(&fio, fname, f, buf, size);
      break;

    case FMT_SREC:
      rc = fileio_srec(&fio, fname, f, buf, size);
      break;

    case FMT_RBIN:
      rc = fileio_rbin(&fio, fname, f, buf, size);
      break;

    default:
      fprintf(stderr, "%s: invalid %s file format: %d\n",
              progname, fio.iodesc, format);
      return -1;
  }

  return rc;
}


char * memtypestr ( AVRMEM memtype )
{
  switch (memtype) {
    case AVR_EEPROM : return "eeprom"; break;
    case AVR_FLASH  : return "flash"; break;
    default         : return "unknown-memtype"; break;
  }
}


/*
 * main routine
 */
int main ( int argc, char * argv [] )
{
  int              fd;          /* file descriptor for parallel port */
  int              rc;          /* general return code checking */
  int              exitrc;      /* exit code for main() */
  int              i;           /* general loop counter */
  int              ch;          /* options flag */
  int              size;        /* size of memory region */
  int              len;         /* length for various strings */
  char             pbuf[PATH_MAX]; /* temporary buffer */
  char           * p1;          /* used to parse CVS Id */
  char           * p2;          /* used to parse CVS Ed */
  unsigned char    sig[4];      /* AVR signature bytes */
  unsigned char    nulldev[4];  /* 0xff signature bytes for comparison */
  struct avrpart * p;           /* which avr part we are programming */
  int              readorwrite; /* true if a chip read/write op was selected */

  /* options / operating mode variables */
  int     memtype;     /* AVR_FLASH or AVR_EEPROM */
  int     doread;      /* 0=reading, 1=writing */
  int     erase;       /* 1=erase chip, 0=don't */
  char  * outputf;     /* output file name */
  char  * inputf;      /* input file name */
  int     ovsigck;     /* 1=override sig check, 0=don't */
  char  * parallel;    /* parallel port device */
  int     interactive; /* 1=enter interactive command mode, 0=don't */
  FILEFMT filefmt;     /* FMT_AUTO, FMT_IHEX, FMT_SREC, FMT_RBIN */

  readorwrite = 0;
  parallel    = DEFAULT_PARALLEL;
  outputf     = NULL;
  inputf      = NULL;
  doread      = 1;
  memtype     = AVR_FLASH;
  erase       = 0;
  p           = NULL;
  ovsigck     = 0;
  interactive = 0;
  filefmt     = FMT_AUTO;

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];

  len = strlen(progname) + 2;
  for (i=0; i<len; i++)
    pbuf[i] = ' ';
  pbuf[i] = 0;

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
  fprintf(stderr, "%s: Copyright 2000 Brian Dean, bsd@bsdhome.com\n"
          "%sRevision ", progname, pbuf);
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
  while ((ch = getopt(argc,argv,"?cef:Fi:m:o:p:P:")) != -1) {

    switch (ch) {
      case 'm': /* select memory type to operate on */
        if ((strcasecmp(optarg,"e")==0)||(strcasecmp(optarg,"eeprom")==0)) {
          memtype = AVR_EEPROM;
        }
        else if ((strcasecmp(optarg,"f")==0)||
                 (strcasecmp(optarg,"flash")==0)) {
          memtype = AVR_FLASH;
        }
        else {
          fprintf(stderr, "%s: invalid memory type \"%s\"\n\n", 
                  progname, optarg);
          usage();
          exit(1);
        }
        readorwrite = 1;
        break;

      case 'F': /* override invalid signature check */
        ovsigck = 1;
        break;

      case 'o': /* specify output file */
        if (inputf || interactive) {
          fprintf(stderr,"%s: -i, -o, and -c are incompatible\n\n", progname);
          return 1;
        }
        doread = 1;
        outputf = optarg;
        if (filefmt == FMT_AUTO)
          filefmt = FMT_IHEX;
        break;

      case 'p' : /* specify AVR part */
        p = NULL;
        for (i=0; i<N_AVRPARTS; i++) {
          if (strcmp(parts[i].optiontag, optarg)==0) {
            p = &parts[i];
            break;
          }
        }
        if (p == NULL) {
          fprintf(stderr, 
                  "%s: AVR Part \"%s\" not found.  Valid parts are:\n\n",
                  progname, optarg );
          list_valid_parts(stderr,"    ");
          fprintf(stderr, "\n");
          return 1;
        }
        break;

      case 'e': /* perform a chip erase */
        erase = 1;
        break;

      case 'i': /* specify input file */
        if (outputf || interactive) {
          fprintf(stderr,"%s: -o, -i, and -c are incompatible\n\n", progname);
          return 1;
        }
        doread = 0;
        inputf = optarg;
        break;

      case 'f':   /* specify file format */
        if (strlen(optarg) != 1) {
          fprintf(stderr, "%s: invalid file format \"%s\"\n",
                  progname, optarg);
          usage();
          exit(1);
        }
        switch (optarg[0]) {
          case 'a' : filefmt = FMT_AUTO; break;
          case 'i' : filefmt = FMT_IHEX; break;
          case 'r' : filefmt = FMT_RBIN; break;
          case 's' :
            fprintf(stderr, 
                    "%s: Motorola S-Record format not yet supported\n\n",
                    progname);
            exit(1);
            break;
          default :
            fprintf(stderr, "%s: invalid file format \"%s\"\n\n",
                    progname, optarg);
            usage();
            exit(1);
        }
        break;

      case 'c': /* enter interactive command mode */
        if (!((inputf == NULL)||(outputf == NULL))) {
          fprintf(stderr, 
                  "%s: interactive mode is not compatible with -i or -o\n\n",
                  progname);
          usage();
          exit(1);
        }
        interactive = 1;
        break;

      case 'P':
        parallel = optarg;
        break;

      case '?': /* help */
        usage();
        exit(0);
        break;

      default:
        fprintf(stderr, "%s: invalid option -%c\n\n", progname, ch);
        usage();
        exit(1);
        break;
    }

  }

  if (p == NULL) {
    fprintf(stderr, 
            "%s: No AVR part has been specified, use \"-p Part\"\n\n"
            "  Valid Parts are:\n\n",
            progname );
    list_valid_parts(stderr, "    ");
    fprintf(stderr,"\n");
    return 1;
  }

  fprintf(stderr, 
          "%sAVR Part               = %s\n"
          "%sFlash memory size      = %d bytes\n"
          "%sEEPROM memory size     = %d bytes\n"
          "%sMin/Max program delay  = %d/%d us\n"
          "%sChip Erase delay       = %d us\n"
          "%sFlash Polled Readback  = 0x%02x\n"
          "%sEEPROM Polled Readback = 0x%02x, 0x%02x\n",
          pbuf, p->partdesc,
          pbuf, p->flash_size, 
          pbuf, p->eeprom_size,
          pbuf, p->min_write_delay, p->max_write_delay, 
          pbuf, p->chip_erase_delay,
          pbuf, p->f_readback,
          pbuf, p->e_readback[0], p->e_readback[1]);
  fprintf(stderr, "\n");

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

  /*
   * open the parallel port
   */
  fd = open ( parallel, O_RDWR );
  if (fd < 0) {
    fprintf ( stderr, "%s: can't open device \"%s\": %s\n\n",
              progname, parallel, strerror(errno) );
    return 1;
  }

  exitrc = 0;


  /*
   * initialize the chip in preperation for accepting commands
   */
  rc = avr_initialize(fd,p);
  if (rc < 0) {
    fprintf ( stderr, "%s: initialization failed, rc=%d\n", progname, rc );
    exitrc = 1;
    goto main_exit;
  }

  fprintf ( stderr, 
            "%s: AVR device initialized and ready to accept instructions\n",
            progname );

  /*
   * Let's read the signature bytes to make sure there is at least a
   * chip on the other end that is responding correctly.  A check
   * against 0xffffffff should ensure that the signature bytes are
   * valid.  
   */
  avr_signature(fd, sig);
  fprintf(stderr, "%s: Device signature = 0x", progname);
  for (i=0; i<4; i++)
    fprintf(stderr, "%02x", sig[i]);
  fprintf(stderr, "\n");

  memset(nulldev,0xff,4);
  if (memcmp(sig,nulldev,4)==0) {
    fprintf(stderr, 
            "%s: Yikes!  Invalid device signature.\n", progname);
    if (!ovsigck) {
      fprintf(stderr, 
              "%sDouble check connections and try again, or use -F to override\n"
              "%sthis check.\n\n",
              pbuf, pbuf );
      exit(1);
    }
  }

  fprintf(stderr, "\n");

  if (erase) {
    /*
     * erase the chip's flash and eeprom memories, this is required
     * before the chip can accept new programming
     */
    fprintf(stderr, "%s: erasing chip\n", progname );
    avr_chip_erase(fd,p);
    avr_initialize(fd,p);
    fprintf(stderr, "%s: done.\n", progname );
  }


  if ((inputf==NULL) && (outputf==NULL)) {
    /*
     * Check here to see if any other operations were selected and
     * generate an error message because if they were, we need either
     * an input or and output file, but one was not selected.
     * Otherwise, we just shut down.  
     */
    if (readorwrite) {
      fprintf(stderr, "%s: you must specify an input or an output file\n",
              progname);
      exitrc = 1;
    }
    goto main_exit;
  }


  if (doread) {
    /*
     * read out the specified device memory and write it to a file 
     */
    fprintf ( stderr, "%s: reading %s memory:\n", 
              progname, memtypestr(memtype) );
    rc = avr_read ( fd, p, memtype );
    if (rc) {
      fprintf ( stderr, "%s: failed to read all of %s memory, rc=%d\n", 
                progname, memtypestr(memtype), rc );
      exitrc = 1;
      goto main_exit;
    }

    rc = fileio(FIO_WRITE, outputf, filefmt, p, memtype);
    if (rc < 0) {
      fprintf(stderr, "%s: terminating\n", progname);
      exitrc = 1;
      goto main_exit;
    }

  }
  else {
    /*
     * write the selected device memory using data from a file; first
     * read the data from the specified file
     */
    rc = fileio(FIO_READ, inputf, filefmt, p, memtype );
    if (rc < 0) {
      fprintf(stderr, "%s: terminating\n", progname);
      exitrc = 1;
      goto main_exit;
    }
    size = rc;

    /*
     * write the buffer contents to the selected memory type
     */
    fprintf(stderr, "%s: writing %s:\n", 
            progname, memtypestr(memtype));
#if 1
    rc = avr_write ( fd, p, memtype );
#else
    /* 
     * test mode, don't actually write to the chip, output the buffer
     * to stdout in intel hex instead 
     */
    rc = fileio(FIO_WRITE, "-", FMT_IHEX, p, memtype);
#endif
    if (rc) {
      fprintf ( stderr, "%s: failed to write flash memory, rc=%d\n", 
                progname, rc );
      exitrc = 1;
      goto main_exit;
    }
  }

 main_exit:

  /*
   * program complete
   */

  avr_powerdown(fd);
  ppi_clr(fd, PPIDATA, 0xff);
  ppi_clr(fd, PPIDATA, AVR_RESET);

  close(fd);

  fprintf(stderr, "\n" );

  return exitrc;
}

