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
 * Make the following (minimal) connections:
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
 * Additionally, the following conntections can be made to enable
 * additional features:
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
#include "fileio.h"
#include "pindefs.h"
#include "ppi.h"
#include "term.h"


#define DEFAULT_PARALLEL "/dev/ppi0"

extern char * avr_version;
extern char * fileio_version;
extern char * main_version;
extern char * ppi_version;
extern char * term_version;

#define N_MODULES 5

char ** modules[5] = { 
  &avr_version, 
  &fileio_version, 
  &main_version, 
  &ppi_version, 
  &term_version 
};

char * version      = "1.2";

char * main_version = "$Id$";

char * progname;
char   progbuf[PATH_MAX]; /* temporary buffer of spaces the same
                             length as progname; used for lining up
                             multiline messages */

unsigned int pinno[N_PINS];

/*
 * usage message
 */
void usage ( void )
{
  fprintf(stderr,
          "Usage: %s -p partno [-e] [-E exitspec[,exitspec]] [-f format] "
          "[-F] [-V]\n"
          "      %s[-i filename] [-m memtype] [-o filename] [-P parallel] "
          "[-t]\n\n",
          progname, progbuf);

}


/*
 * parse the -E string
 */
int getexitspecs ( char *s, int *set, int *clr )
{
  char *cp;

  while ((cp = strtok(s, ","))) {
    if (strcmp(cp, "reset") == 0) {
      *clr |= ppi_getpinmask(pinno[PIN_AVR_RESET]);
    }
    else if (strcmp(cp, "noreset") == 0) {
      *set |= ppi_getpinmask(pinno[PIN_AVR_RESET]);
    }
    else if (strcmp(cp, "vcc") == 0) { 
      if (pinno[PPI_AVR_VCC])
        *set |= pinno[PPI_AVR_VCC];
    }
    else if (strcmp(cp, "novcc") == 0) {
      if (pinno[PPI_AVR_VCC])
        *clr |= pinno[PPI_AVR_VCC];
    }
    else {
      return -1;
    }
    s = 0; /* strtok() should be called with the actual string only once */
  }

  return 0;
}


int parse_cvsid ( char * cvsid, char * name, char * rev, char * datetime )
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


int print_module_versions ( FILE * outf, char * timestamp )
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


#define MAX_LINE_LEN 1024
#define MAX_PIN_NAME 64

int parse_config(int lineno, char * infile, char * config, char * s, 
                 unsigned int * pinno, char * desc, int desclen)
{
  char pin_name[MAX_PIN_NAME];
  char * p;
  int i;
  int pins;
  unsigned int value;
  unsigned int v;
  char * e;

  pins = 0;
  p = s;

  while (1) {

    while (*p && isspace(*p))
      p++;

    if (*p == 0) {
      if (pins == 0) {
        fprintf(stderr, 
                "%s: warning: no pins configured using config entry \"%s\" "
                "at line %d of %s\n",
                progname, config, lineno, infile);
      }
      return 0;
    }

    /*
     * parse the pin name
     */
    pin_name[0] = 0;
    i = 0;
    while (*p && (i<MAX_PIN_NAME) && !((*p == '=')||isspace(*p))) {
      pin_name[i++] = *p;
      p++;
    }

    if (i == MAX_PIN_NAME) {
      fprintf(stderr, "%s: pin name too long at line %d of \"%s\"\n",
              progname, lineno, infile);
      return -1;
    }
    pin_name[i] = 0;

    /* skip over spaces and equals sign */
    while (*p && (isspace(*p)||(*p == '=')))
      p++;

    if (strcasecmp(pin_name, "desc") == 0) {
      i = 0;
      while (*p && (i<desclen) && (*p != ':')) {
        desc[i++] = *p;
        p++;
      }
      if (i == desclen) {
        fprintf(stderr, 
                "%s: error at line %d of %s: description is too "
                "long (max = %d chars)\n",
                progname, lineno, infile, desclen);
        return -1;
      }
      desc[i] = 0;
    }
    else {
      /*
       * parse pin value
       */
      value = 0;
      while (*p && (*p != ':')) {
        
        if (strcasecmp(pin_name, "desc") == 0) {
          i = 0;
          while (*p && (i<desclen) && (*p != ':')) {
            desc[i++] = *p;
            p++;
          }
          if (i == desclen) {
            fprintf(stderr, 
                    "%s: error at line %d of %s: description is too "
                    "long (max = %d chars)\n",
                    progname, lineno, infile, desclen);
            return -1;
          }
          desc[i] = 0;
        }
        else {
          v = strtoul(p, &e, 0);
          if (e == p) {
            fprintf(stderr, 
                    "%s: can't parse pin value at line %d of \"%s\" "
                    "starting with \"%s\"\n",
                    progname, lineno, infile, p);
            return -1;
          }
          
          if (strcasecmp(pin_name, "VCC")==0) {
            /*
             * VCC is a bit mask of pins for the data register, pins 2-9
             */
            if ((v < 2) || (v > 9)) {
              fprintf(stderr, 
                      "%s: error at line %d of %s: VCC must be one or more "
                      "pins from the range 2-9\n",
                      progname, lineno, infile);
              return -1;
            }
            value |= (1 << (v-2));
          }
          else {
            if ((v <= 0) || (v >= 18)) {
              fprintf(stderr, 
                      "%s: error at line %d of %s: pin must be in the "
                      "range 1-17\n",
                      progname, lineno, infile);
              return -1;
            }
            value = v;
          }
          
          p = e;
          while (*p && (isspace(*p)||(*p == ',')))
            p++;
        }
        
        if (strcasecmp(pin_name, "VCC")==0)
          pinno[PPI_AVR_VCC] = value;
        else if (strcasecmp(pin_name, "BUFF")==0)
          pinno[PIN_AVR_BUFF] = value;
        else if (strcasecmp(pin_name, "RESET")==0)
          pinno[PIN_AVR_RESET] = value;
        else if (strcasecmp(pin_name, "SCK")==0)
          pinno[PIN_AVR_SCK] = value;
        else if (strcasecmp(pin_name, "MOSI")==0)
          pinno[PIN_AVR_MOSI] = value;
        else if (strcasecmp(pin_name, "MISO")==0)
          pinno[PIN_AVR_MISO] = value;
        else if (strcasecmp(pin_name, "ERRLED")==0)
          pinno[PIN_LED_ERR] = value;
        else if (strcasecmp(pin_name, "RDYLED")==0)
          pinno[PIN_LED_RDY] = value;
        else if (strcasecmp(pin_name, "PGMLED")==0)
          pinno[PIN_LED_PGM] = value;
        else if (strcasecmp(pin_name, "VFYLED")==0)
          pinno[PIN_LED_VFY] = value;
        else {
          fprintf(stderr, 
                  "%s: error at line %d of %s: invalid pin name \"%s\"\n",
                  progname, lineno, infile,  pin_name);
          return -1;
        } 
        
        pins++;
      }
    }

    while (*p && (*p == ':'))
      p++;
  }

  return 0;
}

  

int read_config(char * infile, char * config, unsigned int * pinno, 
                char * desc, int desclen)
{
  FILE * f;
  char line[MAX_LINE_LEN];
  char buf[MAX_LINE_LEN];
  char configname[MAX_PIN_NAME];
  int len, lineno, rc, cont;
  char * p, * q;
  int i;

  for (i=0; i<N_PINS; i++)
    pinno[i] = 0;

  f = fopen(infile, "r");
  if (f == NULL) {
    fprintf(stderr, "%s: can't open config file \"%s\": %s\n",
            progname, infile, strerror(errno));
    return -1;
  }

  lineno = 0;
  buf[0] = 0;
  cont   = 0;
  while (fgets(line, MAX_LINE_LEN, f) != NULL) {
    lineno++;

    p = line;
    while (isspace(*p))
      p++;

    /*
     * skip empty lines and lines that start with '#'
     */
    if ((*p == '#')||(*p == '\n')||(*p == 0))
      continue;

    len = strlen(p);
    if (p[len-1] == '\n') {
      p[len-1] = 0;
      len--;
    }

    /*
     * we're only interested in pin configuration data which begin
     * with "c:" 
     */
    if (((len < 3) || (p[0] != 'c')) && !cont)
      continue;


    /*
     * skip over the "c:"
     */
    if (!cont) {
      p++;
      while (*p && isspace(*p))
        p++;

      if (*p != ':') {
        fprintf(stderr, "line %d:\n%s\n",
                lineno, line);
        for (i=0; i<(p-line); i++) {
          fprintf(stderr, "-");
        }
        fprintf(stderr, "^\n");
        fprintf(stderr, "error at column %d, line %d of %s: expecting ':'\n",
                p-line+1, lineno, infile);
        return -1;
      }
      p++;
      len = strlen(p);
    }

    cont = 0;

    if (p[len-1] == '\\') {
      cont = 1;              /* flag that this is a continuation line */

      /* trim trailing white space before continuation character */
      q = &p[len-2];
      while (isspace(*q))
        q--;
      q++;
      *q = 0;
    }

    rc = strlcat(buf, p, MAX_LINE_LEN);
    if (rc >= MAX_LINE_LEN) {
      fprintf(stderr, 
              "%s: buffer length of %d exceed at line %d of \"%s\"\n",
              progname, MAX_LINE_LEN, lineno, infile);
      return -2;
    }

    if (cont)
      continue;  /* continuation line, keep going */

    /*
     * read the configuration name from the beginning of the line
     */
    p = buf;
    i = 0;
    while (*p && (i < MAX_PIN_NAME) && (!(isspace(*p)||(*p == ':')))) {
      configname[i++] = *p;
      p++;
    }
    if (i == MAX_PIN_NAME) {
      fprintf(stderr, "%s: configuration name too long at line %d of \"%s\"\n",
              progname, lineno, infile);
      return -3;
    }
    configname[i] = 0;

    /*
     * position 'p' to the beginning of the pin information
     */
    while (*p && (isspace(*p) || (*p == ':')))
      p++;

    if (strcasecmp(configname, config) == 0) {
      strlcpy(desc, "no description", desclen);
      rc = parse_config(lineno, infile, config, p, pinno, desc, desclen);
      if (rc) {
        fprintf(stderr, "%s: error parsing config file \"%s\" at line %d\n",
                progname, infile, lineno);
        return -3;
      }

      return 0;
    }

    buf[0] = 0;
  }

  /*
   * config entry not found
   */

  fprintf(stderr, "%s: config entry \"%s\" not found in file \"%s\"\n",
          progname, config, infile);

  return -5;
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


void pinconfig_display(char * p, char * config, char * desc)
{
  char vccpins[64];

  if (pinno[PPI_AVR_VCC]) {
    snprintf(vccpins, sizeof(vccpins), " = pins %s", 
             vccpins_str(pinno[PPI_AVR_VCC]));
  }
  else {
    vccpins[0] = 0;
  }

  fprintf(stderr, "%sProgrammer Pin Configuration: %s (%s)\n", p, 
          config ? config : "DEFAULT", desc);

  fprintf(stderr, 
          "%s  VCC     = 0x%02x %s\n"
          "%s  BUFF    = %d\n"
          "%s  RESET   = %d\n"
          "%s  SCK     = %d\n"
          "%s  MOSI    = %d\n"
          "%s  MISO    = %d\n"
          "%s  ERR LED = %d\n"
          "%s  RDY LED = %d\n"
          "%s  PGM LED = %d\n"
          "%s  VFY LED = %d\n",
          p, pinno[PPI_AVR_VCC], vccpins,
          p, pinno[PIN_AVR_BUFF],
          p, pinno[PIN_AVR_RESET],
          p, pinno[PIN_AVR_SCK],
          p, pinno[PIN_AVR_MOSI],
          p, pinno[PIN_AVR_MISO],
          p, pinno[PIN_LED_ERR],
          p, pinno[PIN_LED_RDY],
          p, pinno[PIN_LED_PGM],
          p, pinno[PIN_LED_VFY]);
}



void verify_pin_assigned(int pin, char * desc)
{
  if (pinno[pin] == 0) {
    fprintf(stderr, "%s: error: no pin has been assigned for %s\n",
            progname, desc);
    exit(1);
  }
}

    

#define MAX_DESC_LEN 80

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
  char             timestamp[64];
  char             configfile[PATH_MAX]; /* pin configuration file */
  char *           pinconfig;
  char             desc[MAX_DESC_LEN];

  /* options / operating mode variables */
  int     memtype;     /* AVR_FLASH or AVR_EEPROM */
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

  readorwrite   = 0;
  parallel      = DEFAULT_PARALLEL;
  outputf       = NULL;
  inputf        = NULL;
  doread        = 1;
  memtype       = AVR_FLASH;
  erase         = 0;
  p             = NULL;
  ovsigck       = 0;
  terminal      = 0;
  filefmt       = FMT_AUTO;
  nowrite       = 0;
  verify        = 1;        /* on by default; XXX can't turn it off */
  ppisetbits    = ppiclrbits = 0;
  pinconfig     = NULL;

  strcpy(configfile, CONFIG_DIR);
  i = strlen(configfile);
  if (i && (configfile[i-1] != '/'))
    strcat(configfile, "/");
  strcat(configfile, "avrprog.conf");

  for (i=0; i<N_PINS; i++)
    pinno[i] = 0;

  /*
   * default pin configuration
   */
  pinno[PPI_AVR_VCC]   = 0x0f;  /* ppi pins 2-5, data reg bits 0-3 */
  pinno[PIN_AVR_BUFF]  =  0;
  pinno[PIN_AVR_RESET] =  7;
  pinno[PIN_AVR_SCK]   =  8;
  pinno[PIN_AVR_MOSI]  =  9;
  pinno[PIN_AVR_MISO]  = 10;
  pinno[PIN_LED_ERR]   =  0;
  pinno[PIN_LED_RDY]   =  0;
  pinno[PIN_LED_PGM]   =  0;
  pinno[PIN_LED_VFY]   =  0;

  strcpy(desc, "compiled in default");

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];

  len = strlen(progname) + 2;
  for (i=0; i<len; i++)
    progbuf[i] = ' ';
  progbuf[i] = 0;

  print_module_versions(NULL, timestamp);

  /*
   * Print out an identifying string so folks can tell what version
   * they are running
   */
  fprintf(stderr, "\n%s: Copyright 2000 Brian Dean, bsd@bsdhome.com\n"
          "%sVersion %s  Revision Timestamp %s\n\n", 
          progname, progbuf, version, timestamp);

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
  while ((ch = getopt(argc,argv,"?c:C:eE:f:Fi:m:no:p:P:tV")) != -1) {

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

      case 'V':
        print_module_versions(stderr, NULL);
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
   * read the parallel port pin configuration to use if requested
   */
  if (pinconfig != NULL) {
    rc = read_config(configfile, pinconfig, pinno, desc, MAX_DESC_LEN);
    if (rc) {
      fprintf(stderr, "%s: error reading \"%s\" configuration from \"%s\"\n",
              progname, pinconfig, configfile);
      exit(1);
    }
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
#if 1
  pinconfig_display(progbuf, pinconfig, desc);
#endif

  fprintf(stderr, "\n");

  verify_pin_assigned(PIN_AVR_RESET, "AVR RESET");
  verify_pin_assigned(PIN_AVR_SCK,   "AVR SCK");
  verify_pin_assigned(PIN_AVR_MISO,  "AVR MISO");
  verify_pin_assigned(PIN_AVR_MOSI,  "AVR MOSI");

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

#if 0
  ppi_sense(fd);
#endif

  ppidata = ppi_getall(fd, PPIDATA);
  if (ppidata < 0) {
    fprintf ( stderr, "%s: error reading status of ppi data port\n", progname);
    exitrc = 1;
    ppidata = 0; /* clear all bits at exit */
    goto main_exit;
  }

  fprintf(stderr, "initial port data = 0x%02x, pins %s\n",
          ppidata, vccpins_str(ppidata));

  ppidata &= ~ppiclrbits;
  ppidata |= ppisetbits;

  fprintf(stderr, "apply exit specs, port data = 0x%02x, pins %s\n",
          ppidata, vccpins_str(ppidata));

  /* 
   * turn off all the status leds
   */
  LED_OFF(fd, pinno[PIN_LED_RDY]);
  LED_OFF(fd, pinno[PIN_LED_ERR]);
  LED_OFF(fd, pinno[PIN_LED_PGM]);
  LED_OFF(fd, pinno[PIN_LED_VFY]);

  /*
   * enable the 74367 buffer, if connected; this signal is active low
   */
  ppi_setpin(fd, pinno[PIN_AVR_BUFF], 0);

  /*
   * initialize the chip in preperation for accepting commands
   */
  rc = avr_initialize(fd,p);
  if (rc < 0) {
    fprintf ( stderr, "%s: initialization failed, rc=%d\n", progname, rc );
    exitrc = 1;
    goto main_exit;
  }

  /* indicate ready */
  LED_ON(fd, pinno[PIN_LED_RDY]);

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
      fprintf(stderr, "%sDouble check connections and try again, "
              "or use -F to override\n"
              "%sthis check.\n\n",
              progbuf, progbuf );
      exitrc = 1;
      goto main_exit;
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
     * verify that the in memory file (p->mem[AVR_FLASH|AVR_EEPROM])
     * is the same as what is on the chip 
     */
    LED_ON(fd, pinno[PIN_LED_VFY]);

    fprintf(stderr, "%s: verifying %s memory against %s:\n", 
            progname, avr_memtstr(memtype), inputf);
    fprintf(stderr, "%s: reading on-chip %s data:\n", 
            progname, avr_memtstr(memtype));
    rc = avr_read ( fd, v, memtype );
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, avr_memtstr(memtype), rc);
      LED_ON(fd, pinno[PIN_LED_ERR]);
      exitrc = 1;
      goto main_exit;
    }

    fprintf(stderr, "%s: verifying ...\n", progname);
    rc = avr_verify(p, v, memtype, vsize);
    if (rc < 0) {
      fprintf(stderr, "%s: verification error; content mismatch\n", 
              progname);
      LED_ON(fd, pinno[PIN_LED_ERR]);
      exitrc = 1;
      goto main_exit;
    }
    
    fprintf(stderr, "%s: %d bytes of %s verified\n", 
            progname, rc, avr_memtstr(memtype));

    LED_OFF(fd, pinno[PIN_LED_VFY]);
  }



 main_exit:

  /*
   * program complete
   */

  avr_powerdown(fd);
  fprintf(stderr, "port data = 0x%02x, pins %s\n",
          ppidata, vccpins_str(ppidata));
  ppi_setall(fd, PPIDATA, ppidata);

  /*
   * disable the 74367 buffer, if connected; this signal is active low 
   */
  ppi_setpin(fd, pinno[PIN_AVR_BUFF], 1);

  LED_OFF(fd, pinno[PIN_LED_RDY]);

  close(fd);

  fprintf(stderr, "\n%s done.  Thank you.\n\n", progname);

  return exitrc;
}

