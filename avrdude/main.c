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

/*
 * Code to program an Atmel AVR AT90S device using the parallel port.
 *
 * Pin definitions can be changed via a config file.  Below is the
 * default pin configuration in the absence of a config definition
 * which lists "default" as one of its ids.
 *
 *  Parallel Port      Programmer Function
 *  -------------      -----------------------------
 *       Pins 2-5  ->  Vcc (see note below)
 *       Pin    7  ->  AVR /RESET
 *       Pin    8  ->  AVR SCK  (clock input)
 *       Pin    9  ->  AVR MOSI (instruction in)
 *       Pin   10  <-  AVR MISO (data out)
 *       Pin   18      Signal Ground
 *
 * Additionally, the following connections can be made to enable
 * additional features, however, to enable these features use the
 * pin configuration id "alf" ("-c alf" on the command line):
 *
 *  Parallel Port      Programmer Function
 *  -------------      -----------------------------
 *       Pin    1      STATUS LED, active low (program or verify error)
 *       Pin    6  ->  /ENABLE ('367 bus driver)
 *       Pin   14      STATUS LED, active low (ready)
 *       Pin   16      STATUS LED, active low (programming)
 *       Pin   17      STATUS LED, active low (verifying)
 *
 *  Pin 6 can be tied to the enable line of a 74HC367 in order to
 *  isolate and buffer the data to and from the PC parallel port.
 *  This is useful for connecting to a device in-circuit, and keeps
 *  the state of the parallel port pins from interfering with the
 *  normal operation of the target system.  When programming is
 *  complete, this pin is driven high, causing to pins of the '367 to
 *  float.
 *
 *  NOTE on Vcc connection: make sure your parallel port can supply an
 *  adequate amount of current to power your device.  6-10 mA per pin
 *  is common for parallel port signal lines, but is not guaranteed,
 *  especially for notebook computers.  For additional power, use
 *  multiple pins tied together with Schottky diodes.  If in doubt,
 *  don't risk damaging your parallel port, use an external power
 *  supply.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>

#include "avr.h"
#include "config.h"
#include "fileio.h"
#include "pindefs.h"
#include "ppi.h"
#include "term.h"


#define DEFAULT_PARALLEL "/dev/ppi0"

extern char * avr_version;
extern char * config_version;
extern char * fileio_version;
extern char * lists_version;
extern char * main_version;
extern char * ppi_version;
extern char * term_version;

#define N_MODULES 7

char ** modules[N_MODULES] = { 
  &avr_version,
  &config_version,
  &fileio_version,
  &lists_version,
  &main_version, 
  &ppi_version, 
  &term_version 
};

char * version      = "2.1.3";

char * main_version = "$Id$";

char * progname;
char   progbuf[PATH_MAX]; /* temporary buffer of spaces the same
                             length as progname; used for lining up
                             multiline messages */

PROGRAMMER * pgm = NULL;

PROGRAMMER compiled_in_pgm;

/*
 * global options
 */
int do_cycles;   /* track erase-rewrite cycles */


/*
 * usage message
 */
void usage(void)
{
  fprintf(stderr,
          "\nUsage: %s -p partno [-e] [-E exitspec[,exitspec]] [-f format] "
          "[-F]\n"
          "      %s[-i filename] [-m memtype] [-o filename] [-P parallel] "
          "[-t]\n"
          "      %s[-c programmer] [-C config-file] [-v [-v]] [-n]\n\n",
          progname, progbuf, progbuf);

}


/*
 * parse the -E string
 */
int getexitspecs(char *s, int *set, int *clr)
{
  char *cp;

  while ((cp = strtok(s, ","))) {
    if (strcmp(cp, "reset") == 0) {
      *clr |= ppi_getpinmask(pgm->pinno[PIN_AVR_RESET]);
    }
    else if (strcmp(cp, "noreset") == 0) {
      *set |= ppi_getpinmask(pgm->pinno[PIN_AVR_RESET]);
    }
    else if (strcmp(cp, "vcc") == 0) { 
      if (pgm->pinno[PPI_AVR_VCC])
        *set |= pgm->pinno[PPI_AVR_VCC];
    }
    else if (strcmp(cp, "novcc") == 0) {
      if (pgm->pinno[PPI_AVR_VCC])
        *clr |= pgm->pinno[PPI_AVR_VCC];
    }
    else {
      return -1;
    }
    s = 0; /* strtok() should be called with the actual string only once */
  }

  return 0;
}


int parse_cvsid(char * cvsid, char * name, char * rev, char * datetime)
{
  int i, j;

  if (strncmp(cvsid,"$Id: ",5) != 0)
    return -1;

  name[0]     = 0;
  rev[0]      = 0;
  datetime[0] = 0;

  i = 0;
  j = 5;
  while (cvsid[j] && cvsid[j] != ',')
    name[i++] = cvsid[j++];
  name[i] = 0;

  while (cvsid[j] && cvsid[j] != ' ')
    j++;

  if (cvsid[j])
    j++;

  i = 0;
  while (cvsid[j] && cvsid[j] != ' ')
    rev[i++] = cvsid[j++];
  rev[i] = 0;

  if (cvsid[j])
    j++;

  i = 0;
  while (cvsid[j] && cvsid[j] != ' ')
    datetime[i++] = cvsid[j++];
  if (cvsid[j] == ' ') {
    datetime[i++] = cvsid[j++];
    while (cvsid[j] && cvsid[j] != ' ')
      datetime[i++] = cvsid[j++];
  }
  datetime[i] = 0;

  return 0;
}


int print_module_versions(FILE * outf, char * timestamp)
{
  char name[64], rev[16], datetime[64];
  int y, m, d, h, min, s;
  int i;
  int rc;
  int maxtime;
  struct tm t;
  time_t now;

  maxtime = 0;
  for (i=0; i<N_MODULES; i++) {
    parse_cvsid(*modules[i], name, rev, datetime);
    rc = sscanf(datetime, "%d/%d/%d %d:%d:%d", &y, &m, &d, &h, &min, &s);
    if (rc != 6) {
      fprintf(stderr, "%s: module version scan error, rc=%d\n", progname, rc);
    }
    else if (timestamp) {
      now = time(NULL);
      gmtime_r(&now, &t);
      t.tm_sec  = s;
      t.tm_min  = min;
      t.tm_hour = h;
      t.tm_mday = d;
      t.tm_mon  = m-1;
      t.tm_year = y-1900;
      now = timegm(&t);
      if (now > maxtime) {
        maxtime = now;
        strcpy(timestamp, datetime);
        strcat(timestamp, " GMT");
      }
    }
    if (outf)
      fprintf(outf, "  %-10s  %-5s  %s\n", name, rev, datetime);
  }

  if (outf)
    fprintf(outf, "\n");

#if 0
  gmtime_r(&maxtime, &t);
  fprintf(stderr, "Latest revision date is %04d/%02d/%02d %02d:%02d:%02d\n",
          t.tm_year+1900, t.tm_mon, t.tm_mday, 
          t.tm_hour, t.tm_min, t.tm_sec);
#endif

  if (outf)
    fprintf(outf, "\n");

  return 0;
}



int read_config(char * file)
{
  FILE * f;

  f = fopen(file, "r");
  if (f == NULL) {
    fprintf(stderr, "%s: can't open config file \"%s\": %s\n",
            progname, file, strerror(errno));
    return -1;
  }

  infile = file;
  yyin   = f;

  yyparse();

  fclose(f);

  return 0;
}




static char vccpins_buf[64];
char * vccpins_str(unsigned int pmask)
{
  unsigned int mask;
  int pin;
  char b2[8];
  char * b;

  b = vccpins_buf;

  b[0] = 0;
  for (pin = 2, mask = 1; mask < 0x80; mask = mask << 1, pin++) {
    if (pmask & mask) {
      sprintf(b2, "%d", pin);
      if (b[0] != 0)
        strcat(b, ",");
      strcat(b, b2);
    }
  }

  return b;
}


void pinconfig_display(char * p)
{
  char vccpins[64];
  char buffpins[64];

  if (pgm->pinno[PPI_AVR_VCC]) {
    snprintf(vccpins, sizeof(vccpins), " = pins %s", 
             vccpins_str(pgm->pinno[PPI_AVR_VCC]));
  }
  else {
    strcpy(vccpins, " (not used)");
  }

  if (pgm->pinno[PPI_AVR_BUFF]) {
    snprintf(buffpins, sizeof(buffpins), " = pins %s", 
             vccpins_str(pgm->pinno[PPI_AVR_BUFF]));
  }
  else {
    strcpy(buffpins, " (not used)");
  }

  fprintf(stderr, "%sProgrammer Pin Configuration: %s (%s)\n", p, 
          (char *)ldata(lfirst(pgm->id)), pgm->desc);

  fprintf(stderr, 
          "%s  VCC     = 0x%02x%s\n"
          "%s  BUFF    = 0x%02x%s\n"
          "%s  RESET   = %d\n"
          "%s  SCK     = %d\n"
          "%s  MOSI    = %d\n"
          "%s  MISO    = %d\n"
          "%s  ERR LED = %d\n"
          "%s  RDY LED = %d\n"
          "%s  PGM LED = %d\n"
          "%s  VFY LED = %d\n",
          p, pgm->pinno[PPI_AVR_VCC], vccpins,
          p, pgm->pinno[PPI_AVR_BUFF], buffpins,
          p, pgm->pinno[PIN_AVR_RESET],
          p, pgm->pinno[PIN_AVR_SCK],
          p, pgm->pinno[PIN_AVR_MOSI],
          p, pgm->pinno[PIN_AVR_MISO],
          p, pgm->pinno[PIN_LED_ERR],
          p, pgm->pinno[PIN_LED_RDY],
          p, pgm->pinno[PIN_LED_PGM],
          p, pgm->pinno[PIN_LED_VFY]);
}



void verify_pin_assigned(int pin, char * desc)
{
  if (pgm->pinno[pin] == 0) {
    fprintf(stderr, "%s: error: no pin has been assigned for %s\n",
            progname, desc);
    exit(1);
  }
}



PROGRAMMER * locate_pinconfig(LISTID programmers, char * configid)
{
  LNODEID ln1, ln2;
  PROGRAMMER * p = NULL;
  char * id;
  int found;

  found = 0;

  for (ln1=lfirst(programmers); ln1 && !found; ln1=lnext(ln1)) {
    p = ldata(ln1);
    for (ln2=lfirst(p->id); ln2 && !found; ln2=lnext(ln2)) {
      id = ldata(ln2);
      if (strcasecmp(configid, id) == 0)
        found = 1;
    }  
  }

  if (found)
    return p;

  return NULL;
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


void list_parts(FILE * f, char * prefix, LISTID parts)
{
  LNODEID ln1;
  AVRPART * p;

  for (ln1=lfirst(parts); ln1; ln1=lnext(ln1)) {
    p = ldata(ln1);
    fprintf(f, "%s%-4s = %s\n", prefix, p->id, p->desc);
  }

  return;
}



/*
 * main routine
 */
int main(int argc, char * argv [])
{
  int              fd;          /* file descriptor for parallel port */
  int              rc;          /* general return code checking */
  int              exitrc;      /* exit code for main() */
  int              i;           /* general loop counter */
  int              ch;          /* options flag */
  int              size;        /* size of memory region */
  int              len;         /* length for various strings */
  struct avrpart * p;           /* which avr part we are programming */
  struct avrpart * v;           /* used for verify */
  int              readorwrite; /* true if a chip read/write op was selected */
  int              ppidata;	/* cached value of the ppi data register */
  int              vsize=-1;    /* number of bytes to verify */
  char             timestamp[64];
  AVRMEM         * sig;         /* signature data */

  /* options / operating mode variables */
  char *  memtype;     /* "flash", "eeprom", etc */
  int     doread;      /* 1=reading AVR, 0=writing AVR */
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
  char  * exitspecs;   /* exit specs string from command line */
  int     verbose;     /* verbose output */
  char  * pinconfig;   /* programmer id */
  char  * partdesc;    /* part id */
  char    configfile[PATH_MAX]; /* pin configuration file */
  int     cycles;      /* erase-rewrite cycles */
  int     set_cycles;  /* value to set the erase-rewrite cycles to */
  char  * e;

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];

  init_config();

  partdesc      = NULL;
  readorwrite   = 0;
  parallel      = DEFAULT_PARALLEL;
  outputf       = NULL;
  inputf        = NULL;
  doread        = 1;
  memtype       = "flash";
  erase         = 0;
  p             = NULL;
  ovsigck       = 0;
  terminal      = 0;
  filefmt       = FMT_AUTO;
  nowrite       = 0;
  verify        = 1;        /* on by default */
  ppisetbits    = 0;
  ppiclrbits    = 0;
  exitspecs     = NULL;
  pgm           = NULL;
  pinconfig     = "avrprog"; /* compiled-in default */
  verbose       = 0;
  do_cycles     = 0;
  set_cycles    = -1;

  strcpy(configfile, CONFIG_DIR);
  i = strlen(configfile);
  if (i && (configfile[i-1] != '/'))
    strcat(configfile, "/");
  strcat(configfile, "avrprog.conf");

  /*
   * initialize compiled-in default programmer 
   */
  pgm = &compiled_in_pgm;
  pgm->id = lcreat(NULL, 0);
  ladd(pgm->id, dup_string("avrprog"));
  strcpy(pgm->desc, "avrprog compiled-in default");
  for (i=0; i<N_PINS; i++)
    pgm->pinno[i] = 0;
  pgm->pinno[PPI_AVR_VCC]   = 0x0f;  /* ppi pins 2-5, data reg bits 0-3 */
  pgm->pinno[PPI_AVR_BUFF]  =  0;
  pgm->pinno[PIN_AVR_RESET] =  7;
  pgm->pinno[PIN_AVR_SCK]   =  8;
  pgm->pinno[PIN_AVR_MOSI]  =  9;
  pgm->pinno[PIN_AVR_MISO]  = 10;
  pgm->pinno[PIN_LED_ERR]   =  0;
  pgm->pinno[PIN_LED_RDY]   =  0;
  pgm->pinno[PIN_LED_PGM]   =  0;
  pgm->pinno[PIN_LED_VFY]   =  0;

  len = strlen(progname) + 2;
  for (i=0; i<len; i++)
    progbuf[i] = ' ';
  progbuf[i] = 0;

  print_module_versions(NULL, timestamp);

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
  while ((ch = getopt(argc,argv,"?c:C:eE:f:Fi:m:no:p:P:tvVyY:")) != -1) {

    switch (ch) {
      case 'c': /* pin configuration */
        pinconfig = optarg;
        break;

      case 'C': /* pin configuration file */
        strncpy(configfile, optarg, PATH_MAX);
        configfile[PATH_MAX-1] = 0;
        break;

      case 'm': /* select memory type to operate on */
        if ((strcasecmp(optarg,"e")==0)||(strcasecmp(optarg,"eeprom")==0)) {
          memtype = "eeprom";
        }
        else if ((strcasecmp(optarg,"f")==0)||
                 (strcasecmp(optarg,"flash")==0)) {
          memtype = "flash";
        }
        else {
          memtype = optarg;
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
        partdesc = optarg;
        break;

      case 'e': /* perform a chip erase */
        erase = 1;
        break;

      case 'E':
        exitspecs = optarg;
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
        verbose++;
        break;

      case 'V':
        verify = 0;
        break;

      case 'y':
        do_cycles = 1;
        break;

      case 'Y':
        set_cycles = strtol(optarg, &e, 0);
        if ((e == optarg) || (*e != 0)) {
          fprintf(stderr, "%s: invalid cycle count '%s'\n",
                  progname, optarg);
          exit(1);
        }
        do_cycles = 1;
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

  if (verbose) {
    /*
     * Print out an identifying string so folks can tell what version
     * they are running
     */
    fprintf(stderr, 
            "\n%s: Copyright (c) 2000-2002 Brian Dean, bsd@bsdhome.com\n"
            "%sVersion %s  Revision Timestamp %s\n\n", 
            progname, progbuf, version, timestamp);
    
    if (verbose > 1) {
      print_module_versions(stderr, NULL);
    }
  }

  rc = read_config(configfile);
  if (rc) {
    fprintf(stderr, "%s: error reading \"%s\" configuration from \"%s\"\n",
            progname, pinconfig, configfile);
    exit(1);
  }

  if (strcmp(pinconfig, "avrprog") == 0) {
    pgm = locate_pinconfig(programmers, "default");
    if (pgm == NULL) {
      /* no default config listed, use the compile-in default */
      pgm = &compiled_in_pgm;
    }
  }
  else {
    pgm = locate_pinconfig(programmers, pinconfig);
    if (pgm == NULL) {
      fprintf(stderr, 
              "%s: Can't find pin config id \"%s\"\n",
              progname, pinconfig);
      fprintf(stderr,"\n");
      exit(1);
    }
  }

  if (partdesc == NULL) {
    fprintf(stderr, 
            "%s: No AVR part has been specified, use \"-p Part\"\n\n"
            "  Valid Parts are:\n\n",
            progname);
    list_parts(stderr, "    ", part_list);
    fprintf(stderr,"\n");
    exit(1);
  }


  p = locate_part(part_list, partdesc);
  if (p == NULL) {
    fprintf(stderr, 
            "%s: AVR Part \"%s\" not found.  Valid parts are:\n\n",
            progname, partdesc);
    list_parts(stderr, "    ", part_list);
    fprintf(stderr, "\n");
    exit(1);
  }


  if (exitspecs != NULL) {
    if (getexitspecs(exitspecs, &ppisetbits, &ppiclrbits) < 0) {
      usage();
      exit(1);
    }
  }


  /* 
   * set up seperate instances of the avr part, one for use in
   * programming, one for use in verifying.  These are separate
   * because they need separate flash and eeprom buffer space 
   */
  p = avr_dup_part(p);
  v = avr_dup_part(p);

  if (verbose) {
    avr_display(stderr, p, progbuf, verbose);
    fprintf(stderr, "\n");
    pinconfig_display(progbuf);
  }

  fprintf(stderr, "\n");

  verify_pin_assigned(PIN_AVR_RESET, "AVR RESET");
  verify_pin_assigned(PIN_AVR_SCK,   "AVR SCK");
  verify_pin_assigned(PIN_AVR_MISO,  "AVR MISO");
  verify_pin_assigned(PIN_AVR_MOSI,  "AVR MOSI");

  /*
   * open the parallel port
   */
  fd = open(parallel, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "%s: can't open device \"%s\": %s\n\n",
              progname, parallel, strerror(errno));
    return 1;
  }

  exitrc = 0;

#if 0
  ppi_sense(fd);
#endif

  ppidata = ppi_getall(fd, PPIDATA);
  if (ppidata < 0) {
    fprintf(stderr, "%s: error reading status of ppi data port\n", progname);
    exitrc = 1;
    ppidata = 0; /* clear all bits at exit */
    goto main_exit;
  }

  ppidata &= ~ppiclrbits;
  ppidata |= ppisetbits;

  /* 
   * turn off all the status leds
   */
  LED_OFF(fd, pgm->pinno[PIN_LED_RDY]);
  LED_OFF(fd, pgm->pinno[PIN_LED_ERR]);
  LED_OFF(fd, pgm->pinno[PIN_LED_PGM]);
  LED_OFF(fd, pgm->pinno[PIN_LED_VFY]);

  /*
   * enable the 74367 buffer, if connected; this signal is active low
   */
  /*ppi_setpin(fd, pgm->pinno[PIN_AVR_BUFF], 0);*/
  ppi_clr(fd, PPIDATA, pgm->pinno[PPI_AVR_BUFF]);

  /*
   * initialize the chip in preperation for accepting commands
   */
  rc = avr_initialize(fd,p);
  if (rc < 0) {
    fprintf(stderr, "%s: initialization failed, rc=%d\n", progname, rc);
    exitrc = 1;
    goto main_exit;
  }

  /* indicate ready */
  LED_ON(fd, pgm->pinno[PIN_LED_RDY]);

  fprintf(stderr, 
            "%s: AVR device initialized and ready to accept instructions\n",
            progname);

  /*
   * Let's read the signature bytes to make sure there is at least a
   * chip on the other end that is responding correctly.  A check
   * against 0xffffffff should ensure that the signature bytes are
   * valid.  
   */
  rc = avr_signature(fd, p);
  if (rc != 0) {
    fprintf(stderr, "%s: error reading signature data, rc=%d\n",
            progname, rc);
    exit(1);
  }

  sig = avr_locate_mem(p, "signature");
  if (sig == NULL) {
    fprintf(stderr,
            "%s: WARNING: signature data not defined for device \"%s\"\n",
            progname, p->desc);
  }

  if (sig != NULL) {
    int ff;

    fprintf(stderr, "%s: Device signature = 0x", progname);
    ff = 1;
    for (i=0; i<sig->size; i++) {
      fprintf(stderr, "%02x", sig->buf[i]);
      if (sig->buf[i] != 0xff)
        ff = 0;
    }
    fprintf(stderr, "\n");

    if (ff) {
      fprintf(stderr, 
              "%s: Yikes!  Invalid device signature.\n", progname);
      if (!ovsigck) {
        fprintf(stderr, "%sDouble check connections and try again, "
                "or use -F to override\n"
                "%sthis check.\n\n",
                progbuf, progbuf);
        exitrc = 1;
        goto main_exit;
      }
    }
  }

  if (set_cycles != -1) {
    cycles = avr_get_cycle_count(fd, p);
    if (cycles != -1) {
      /*
       * only attempt to update the cycle counter if we can actually
       * read the old value
       */
      cycles = set_cycles;
      fprintf(stderr, "%s: setting erase-rewrite cycle count to %d\n", 
              progname, cycles);
      rc = avr_put_cycle_count(fd, p, cycles);
      if (rc < 0) {
        fprintf(stderr, 
                "%s: WARNING: failed to update the erase-rewrite cycle "
                "counter\n",
                progname);
      }
    }
  }

  if (erase) {
    /*
     * erase the chip's flash and eeprom memories, this is required
     * before the chip can accept new programming
     */
    fprintf(stderr, "%s: erasing chip\n", progname);
    avr_chip_erase(fd,p);
    fprintf(stderr, "%s: done.\n", progname);
  }
  else if (set_cycles == -1) {
    /*
     * The erase routine displays this same information, so don't
     * repeat it if an erase was done.  Also, don't display this if we
     * set the cycle count (due to -Y).
     *
     * see if the cycle count in the last two bytes of eeprom seems
     * reasonable 
     */
    cycles = avr_get_cycle_count(fd, p);
    if ((cycles != -1) && (cycles != 0x00ffff)) {
      fprintf(stderr,
              "%s: current erase-rewrite cycle count is %d%s\n",
              progname, cycles, 
              do_cycles ? "" : " (if being tracked)");
    }
  }



  if (!terminal && ((inputf==NULL) && (outputf==NULL))) {
    /*
     * Check here to see if any other operations were selected and
     * generate an error message because if they were, we need either
     * an input or an output file, but one was not selected.
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
            progname, memtype);
    rc = avr_read(fd, p, memtype, 0, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, memtype, rc);
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
    fprintf(stderr, "%s: writing %s (%d bytes):\n", 
            progname, memtype, size);

    if (!nowrite) {
      rc = avr_write(fd, p, memtype, size, 1);
    }
    else {
      /* 
       * test mode, don't actually write to the chip, output the buffer
       * to stdout in intel hex instead 
       */
      rc = fileio(FIO_WRITE, "-", FMT_IHEX, p, memtype, size);
    }

    if (rc < 0) {
      fprintf(stderr, "%s: failed to write %s memory, rc=%d\n", 
                progname, memtype, rc);
      exitrc = 1;
      goto main_exit;
    }

    vsize = rc;

    fprintf(stderr, "%s: %d bytes of %s written\n", progname, 
            vsize, memtype);

  }

  if (!doread && verify) {
    /* 
     * verify that the in memory file (p->mem[AVR_M_FLASH|AVR_M_EEPROM])
     * is the same as what is on the chip 
     */
    LED_ON(fd, pgm->pinno[PIN_LED_VFY]);

    fprintf(stderr, "%s: verifying %s memory against %s:\n", 
            progname, memtype, inputf);
    fprintf(stderr, "%s: reading on-chip %s data:\n", 
            progname, memtype);
    rc = avr_read(fd, v, memtype, vsize, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, memtype, rc);
      LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
      exitrc = 1;
      goto main_exit;
    }

    fprintf(stderr, "%s: verifying ...\n", progname);
    rc = avr_verify(p, v, memtype, vsize);
    if (rc < 0) {
      fprintf(stderr, "%s: verification error; content mismatch\n", 
              progname);
      LED_ON(fd, pgm->pinno[PIN_LED_ERR]);
      exitrc = 1;
      goto main_exit;
    }
    
    fprintf(stderr, "%s: %d bytes of %s verified\n", 
            progname, rc, memtype);

    LED_OFF(fd, pgm->pinno[PIN_LED_VFY]);
  }



 main_exit:

  /*
   * program complete
   */

  avr_powerdown(fd);

  ppi_setall(fd, PPIDATA, ppidata);

  /*
   * disable the 74367 buffer, if connected; this signal is active low 
   */
  /* ppi_setpin(fd, pgm->pinno[PIN_AVR_BUFF], 1); */
  ppi_set(fd, PPIDATA, pgm->pinno[PPI_AVR_BUFF]);

  LED_OFF(fd, pgm->pinno[PIN_LED_RDY]);

  close(fd);

  fprintf(stderr, "\n%s done.  Thank you.\n\n", progname);

  return exitrc;
}

