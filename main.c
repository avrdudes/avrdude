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
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include "avr.h"
#include "fileio.h"
#include "ppi.h"
#include "term.h"


#define DEFAULT_PARALLEL "/dev/ppi0"


char * version = "1.1";

char * progname;
char   progbuf[PATH_MAX]; /* temporary buffer of spaces the same
                             length as progname; used for lining up
                             multiline messages */

/*
 * usage message
 */
void usage ( void )
{
  fprintf(stderr,
          "\nUsage: %s -p partno [-e] [-E exitspec[,exitspec]] [-f format] [-F]\n"
          "      %s[-i filename] [-m memtype] [-o filename] [-P parallel] [-t]\n\n",
          progname, progbuf);

#if 0
  fprintf(stderr, "  Valid Parts for the -p option are:\n");
  avr_list_parts(stderr, "    ");
  fprintf(stderr, "\n");
#endif

}


/*
 * parse the -E string
 */
int getexitspecs ( char *s, int *set, int *clr )
{
  char *cp;

  while ((cp = strtok(s, ","))) {
    if (strcmp(cp, "reset") == 0) {
      *clr |= AVR_RESET; 
    }
    else if (strcmp(cp, "noreset") == 0) {
      *set |= AVR_RESET; 
    }
    else if (strcmp(cp, "vcc") == 0) { 
      *set |= AVR_POWER; 
    }
    else if (strcmp(cp, "novcc") == 0) {
      *clr |= AVR_POWER; 
    }
    else {
      return -1;
    }
    s = 0; /* strtok() should be called with the actual string only once */
  }

  return 0;
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
  unsigned char    sig[4];      /* AVR signature bytes */
  unsigned char    nulldev[4];  /* 0xff signature bytes for comparison */
  struct avrpart * p, ap1;      /* which avr part we are programming */
  struct avrpart * v, ap2;      /* used for verify */
  int              readorwrite; /* true if a chip read/write op was selected */
  int              ppidata;	/* cached value of the ppi data register */
  int              vsize=-1;    /* number of bytes to verify */

  /* options / operating mode variables */
  int     memtype;     /* AVR_FLASH or AVR_EEPROM */
  int     doread;      /* 0=reading, 1=writing */
  int     erase;       /* 1=erase chip, 0=don't */
  char  * outputf;     /* output file name */
  char  * inputf;      /* input file name */
  int     ovsigck;     /* 1=override sig check, 0=don't */
  char  * parallel;    /* parallel port device */
  int     terminal;    /* 1=enter terminal mode, 0=don't */
  FILEFMT filefmt;     /* FMT_AUTO, FMT_IHEX, FMT_SREC, FMT_RBIN */
  int     nowrite;     /* don't actually write anything to the chip */
  int     verify;      /* perform a verify operation */
  int     ppisetbits;  /* bits to set in ppi data register at exit */
  int     ppiclrbits;  /* bits to clear in ppi data register at exit */

  readorwrite = 0;
  parallel    = DEFAULT_PARALLEL;
  outputf     = NULL;
  inputf      = NULL;
  doread      = 1;
  memtype     = AVR_FLASH;
  erase       = 0;
  p           = NULL;
  ovsigck     = 0;
  terminal = 0;
  filefmt     = FMT_AUTO;
  nowrite     = 0;
  verify      = 1;        /* on by default; XXX can't turn it off */
  ppisetbits  = ppiclrbits = 0;

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];

  len = strlen(progname) + 2;
  for (i=0; i<len; i++)
    progbuf[i] = ' ';
  progbuf[i] = 0;

  /*
   * Print out an identifying string so folks can tell what version
   * they are running
   */
  fprintf(stderr, "\n%s: Copyright 2000 Brian Dean, bsd@bsdhome.com\n"
          "%sVersion %s\n\n", progname, progbuf, version);

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
  while ((ch = getopt(argc,argv,"?eE:f:Fi:m:no:p:P:tv")) != -1) {

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

      case 'n':
        nowrite = 1;
        break;

      case 'o': /* specify output file */
        if (inputf || terminal) {
          fprintf(stderr,"%s: -i, -o, and -t are incompatible\n\n", progname);
          return 1;
        }
        doread = 1;
        outputf = optarg;
        if (filefmt == FMT_AUTO)
          filefmt = FMT_RBIN;
        break;

      case 'p' : /* specify AVR part */
        p = avr_find_part(optarg);
        if (p == NULL) {
          fprintf(stderr, 
                  "%s: AVR Part \"%s\" not found.  Valid parts are:\n\n",
                  progname, optarg );
          avr_list_parts(stderr,"    ");
          fprintf(stderr, "\n");
          return 1;
        }
        break;

      case 'e': /* perform a chip erase */
        erase = 1;
        break;

      case 'E':
	if (getexitspecs(optarg, &ppisetbits, &ppiclrbits) < 0) {
	  usage();
	  exit(1);
	}
	break;

      case 'i': /* specify input file */
        if (outputf || terminal) {
          fprintf(stderr,"%s: -o, -i, and -t are incompatible\n\n", progname);
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

      case 't': /* enter terminal mode */
        if (!((inputf == NULL)||(outputf == NULL))) {
          fprintf(stderr, 
                  "%s: terminal mode is not compatible with -i or -o\n\n",
                  progname);
          usage();
          exit(1);
        }
        terminal = 1;
        break;

      case 'P':
        parallel = optarg;
        break;

      case 'v':
        verify = 1;
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
    avr_list_parts(stderr, "    ");
    fprintf(stderr,"\n");
    return 1;
  }

  /* 
   * set up seperate instances of the avr part, one for use in
   * programming, one for use in verifying.  These are separate
   * because they need separate flash and eeprom buffer space 
   */
  ap1 = *p;
  v   = p;
  p   = &ap1;
  ap2 = *v;
  v   = &ap2;

  avr_initmem(p);
  avr_initmem(v);

  avr_display(stderr, p, progbuf);

  fprintf(stderr, "\n");

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

  ppidata = ppi_getall(fd, PPIDATA);
  if (ppidata < 0) {
    fprintf ( stderr, "%s: error reading status of ppi data port\n", progname);
    exitrc = 1;
    ppidata = 0; /* clear all bits at exit */
    goto main_exit;
  }
  ppidata &= ~ppiclrbits;
  ppidata |= ppisetbits;

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
              progbuf, progbuf );
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
    fprintf(stderr, "%s: done.\n", progname );
  }


  if (!terminal && ((inputf==NULL) && (outputf==NULL))) {
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

  if (terminal) {
    /*
     * terminal mode
     */
    exitrc = terminal_mode(fd, p);
  }
  else if (doread) {
    /*
     * read out the specified device memory and write it to a file 
     */
    fprintf(stderr, "%s: reading %s memory:\n", 
            progname, avr_memtstr(memtype));
    rc = avr_read ( fd, p, memtype );
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, avr_memtstr(memtype), rc);
      exitrc = 1;
      goto main_exit;
    }
    size = rc;

    fprintf(stderr, "%s: writing output file \"%s\"\n",
            progname, outputf);
    rc = fileio(FIO_WRITE, outputf, filefmt, p, memtype, size);
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
    fprintf(stderr, "%s: reading input file \"%s\"\n",
            progname, inputf);
    rc = fileio(FIO_READ, inputf, filefmt, p, memtype, -1);
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
            progname, avr_memtstr(memtype));

    if (!nowrite) {
      rc = avr_write ( fd, p, memtype, size );
    }
    else {
      /* 
       * test mode, don't actually write to the chip, output the buffer
       * to stdout in intel hex instead 
       */
      rc = fileio(FIO_WRITE, "-", FMT_IHEX, p, memtype, size);
    }

    if (rc < 0) {
      fprintf ( stderr, "%s: failed to write flash memory, rc=%d\n", 
                progname, rc );
      exitrc = 1;
      goto main_exit;
    }

    vsize = rc;

    fprintf(stderr, "%s: %d bytes of %s written\n", progname, 
            vsize, avr_memtstr(memtype));

  }

  if (!doread && verify) {
    /* 
     * verify that the in memory file (p->flash or p->eeprom) is the
     * same as what is on the chip 
     */
    fprintf(stderr, "%s: verifying %s memory against %s:\n", 
            progname, avr_memtstr(memtype), inputf);
    fprintf(stderr, "%s: reading on-chip %s data:\n", 
            progname, avr_memtstr(memtype));
    rc = avr_read ( fd, v, memtype );
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, avr_memtstr(memtype), rc);
      exitrc = 1;
      goto main_exit;
    }

    fprintf(stderr, "%s: verifying ...\n", progname);
    rc = avr_verify(p, v, memtype, vsize);
    if (rc < 0) {
      fprintf(stderr, "%s: verification error; content mismatch\n", 
              progname);
      exitrc = 1;
      goto main_exit;
    }
    
    fprintf(stderr, "%s: %d bytes of %s verified\n", 
            progname, rc, avr_memtstr(memtype));
  }



 main_exit:

  /*
   * program complete
   */

  avr_powerdown(fd);
  ppi_setall(fd, PPIDATA, ppidata);

  close(fd);

  fprintf(stderr, "\n%s done.  Thank you.\n\n", progname);

  return exitrc;
}

