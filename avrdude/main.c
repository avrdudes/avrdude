/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */

/*
 * Code to program an Atmel AVR AT90S device using the parallel port.
 *
 * For parallel port connected programmers, the pin definitions can be
 * changed via a config file.  See the config file for instructions on
 * how to add a programmer definition.
 *  
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "avr.h"
#include "config.h"
#include "confwin.h"
#include "fileio.h"
#include "lists.h"
#include "par.h"
#include "pindefs.h"
#include "ppi.h"
#include "term.h"


enum {
  DEVICE_READ, 
  DEVICE_WRITE,
  DEVICE_VERIFY
};


typedef struct update_t {
  char * memtype;
  int    op;
  char * filename;
  int    format;
} UPDATE;



/* Get VERSION from ac_cfg.h */
char * version      = VERSION;

int    verbose;     /* verbose output */
char * progname;
char   progbuf[PATH_MAX]; /* temporary buffer of spaces the same
                             length as progname; used for lining up
                             multiline messages */

PROGRAMMER * pgm = NULL;

LISTID updates;

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
    "Usage: %s [options]\n"
    "Options:\n"
    "  -p <partno>                Required. Specify AVR device.\n"
    "  -C <config-file>           Specify location of configuration file.\n"
    "  -c <programmer>            Specify programmer type.\n"
    "  -P <port>                  Specify connection port.\n"
    "  -F                         Override invalid signature check.\n"
    "  -e                         Perform a chip erase.\n"
    "  -m <memtype>               Memory type to operate on.\n"
    "  -i <filename>              Write device. Specify an input file.\n"
    "  -o <filename>              Read device. Specify an output file.\n"
    "  -f <format>                Specify the file format.\n"
    "  -U <memtype>:r|w|v:<filename>[:format]\n"
    "                             Alternate memory operation specification.\n"
    "                             Multiple -U options are allowed, each request\n"
    "                             is performed in the order specified.\n"
    "  -n                         Do not write anything to the device.\n"
    "  -V                         Do not verify.\n"
    "  -t                         Enter terminal mode.\n"
    "  -E <exitspec>[,<exitspec>] List programmer exit specifications.\n"
    "  -v                         Verbose output. -v -v for more.\n"
    "  -q                         Quell progress output.\n"
    "  -?                         Display this usage.\n"
    "\navrdude project: <URL:http://savannah.nongnu.org/projects/avrdude>\n"
    ,progname);
}


/*
 * parse the -E string
 */
int getexitspecs(char *s, int *set, int *clr)
{
  char *cp;

  while ((cp = strtok(s, ","))) {
    if (strcmp(cp, "reset") == 0) {
      *clr |= par_getpinmask(pgm->pinno[PIN_AVR_RESET]);
    }
    else if (strcmp(cp, "noreset") == 0) {
      *set |= par_getpinmask(pgm->pinno[PIN_AVR_RESET]);
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



int read_config(char * file)
{
  FILE * f;

  f = fopen(file, "r");
  if (f == NULL) {
    fprintf(stderr, "%s: can't open config file \"%s\": %s\n",
            progname, file, strerror(errno));
    return -1;
  }

  lineno = 1;
  infile = file;
  yyin   = f;

  yyparse();

  fclose(f);

  return 0;
}




void programmer_display(char * p)
{
  fprintf(stderr, "%sProgrammer Type : %s\n", p, pgm->type);
  fprintf(stderr, "%sDescription     : %s\n", p, pgm->desc);

  pgm->display(pgm, p);
}



void verify_pin_assigned(int pin, char * desc)
{
  if (pgm->pinno[pin] == 0) {
    fprintf(stderr, "%s: error: no pin has been assigned for %s\n",
            progname, desc);
    exit(1);
  }
}



PROGRAMMER * locate_programmer(LISTID programmers, char * configid)
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
    fprintf(f, "%s%-4s = %-15s [%s:%d]\n", 
            prefix, p->id, p->desc, p->config_file, p->lineno);
  }

  return;
}

void list_programmers(FILE * f, char * prefix, LISTID programmers)
{
  LNODEID ln1;
  PROGRAMMER * p;

  for (ln1=lfirst(programmers); ln1; ln1=lnext(ln1)) {
    p = ldata(ln1);
    fprintf(f, "%s%-8s = %-30s [%s:%d]\n", 
            prefix, (char *)ldata(lfirst(p->id)), p->desc, 
            p->config_file, p->lineno);
  }

  return;
}

typedef void (*FP_UpdateProgress)(int percent, double etime, char *hdr);

static FP_UpdateProgress update_progress;

/* Report the progress of a read or write operation from/to the device.

   The first call of report_progress() should look like this (for a write op):

     report_progress (0, 1, "Writing"); 

   Then hdr should be passed NULL on subsequent calls while the operation is
   progressing. Once the operation is complete, a final call should be made as
   such to ensure proper termination of the progress report:

     report_progress (1, 1, NULL);

   It would be nice if we could reduce the usage to one and only one call for
   each of start, during and end cases. As things stand now, that is not
   possible and makes maintenance a bit more work. */

void report_progress (int completed, int total, char *hdr)
{
  static int last = 0;
  static double start_time;
  int percent = (completed * 100) / total;
  struct timeval tv;
  double t;

  if (update_progress == NULL)
    return;

  gettimeofday(&tv, NULL);
  t = tv.tv_sec + ((double)tv.tv_usec)/1000000;

  if (hdr) {
    last = 0;
    start_time = t;
    update_progress (percent, t - start_time, hdr);
  }

  if (percent > 100)
    percent = 100;

  if (percent > last) {
    last = percent;
    update_progress (percent, t - start_time, hdr);
  }

  if (percent == 100)
    last = 0;                   /* Get ready for next time. */
}

static void update_progress_tty (int percent, double etime, char *hdr)
{
  static char hashes[51];
  static char *header;
  static int last = 0;
  int i;

  hashes[50] = 0;

  memset (hashes, ' ', 50);
  for (i=0; i<percent; i+=2) {
    hashes[i/2] = '#';
  }

  if (hdr) {
    fprintf (stderr, "\n");
    last = 0;
    header = hdr;
  }

  if (last == 0) {
    fprintf(stderr, "\r%s | %s | %d%% %0.2fs", 
            header, hashes, percent, etime);
  }

  if (percent == 100) {
    last = 1;
    fprintf (stderr, "\n\n");
  }
}

static void update_progress_no_tty (int percent, double etime, char *hdr)
{
  static int last = 0;
  int cnt = (percent>>1)*2;

  if (hdr) {
    fprintf (stderr, "\n%s | ", hdr);
    last = 0;
  }
  else {
    while (cnt > last) {
      fprintf (stderr, "#");
      cnt -=  2;
    }
  }

  if ((percent == 100) && (last != 0)) {
    fprintf (stderr, " | 100%% %0.2fs\n\n", etime);
    last = 0;
  }
  else
    last = (percent>>1)*2;    /* Make last a multiple of 2. */
}


UPDATE * parse_op(char * s)
{
  char buf[1024];
  char * p;
  UPDATE * upd;
  int i;

  upd = (UPDATE *)malloc(sizeof(UPDATE));
  if (upd == NULL) {
    fprintf(stderr, "%s: out of memory\n", progname);
    exit(1);
  }

  i = 0;
  p = s;
  while ((i < (sizeof(buf)-1) && *p && (*p != ':')))
    buf[i++] = *p++;

  if (*p != ':') {
    fprintf(stderr, "%s: invalid update specification\n", progname);
    free(upd);
    return NULL;
  }

  buf[i] = 0;

  upd->memtype = (char *)malloc(strlen(buf)+1);
  if (upd->memtype == NULL) {
    fprintf(stderr, "%s: out of memory\n", progname);
    exit(1);
  }
  strcpy(upd->memtype, buf);

  p++;
  if (*p == 'r') {
    upd->op = DEVICE_READ;
  }
  else if (*p == 'w') {
    upd->op = DEVICE_WRITE;
  }
  else if (*p == 'v') {
    upd->op = DEVICE_VERIFY;
  }
  else {
    fprintf(stderr, "%s: invalid I/O mode '%c' in update specification\n",
            progname, *p);
    fprintf(stderr, 
            "  allowed values are:\n"
            "    r = read device\n"
            "    w = write device\n"
            "    v = verify device\n");
    free(upd->memtype);
    free(upd);
    return NULL;
  }

  p++;

  if (*p != ':') {
    fprintf(stderr, "%s: invalid update specification\n", progname);
    free(upd->memtype);
    free(upd);
    return NULL;
  }

  p++;

  i = 0;
  while ((i < (sizeof(buf)-1) && *p && (*p != ':')))
    buf[i++] = *p++;
  
  if (!((*p == ':')||(*p == 0))) {
    fprintf(stderr, "%s: invalid update specification\n", progname);
    free(upd->memtype);
    free(upd);
    return NULL;
  }

  buf[i] = 0;

  upd->filename = (char *)malloc(strlen(buf)+1);
  if (upd->filename == NULL) {
    fprintf(stderr, "%s: out of memory\n", progname);
    free(upd->memtype);
    free(upd);
    return NULL;
  }
  strcpy(upd->filename, buf);

  upd->format = FMT_AUTO;

  if (*p == ':') {
    p++;
    switch (*p) {
      case 'a': upd->format = FMT_AUTO; break;
      case 's': upd->format = FMT_SREC; break;
      case 'i': upd->format = FMT_IHEX; break;
      case 'r': upd->format = FMT_RBIN; break;
      case 'm': upd->format = FMT_IMM; break;
      default:
        fprintf(stderr, "%s: invalid file format '%c' in update specifier\n",
                progname, *p);
        free(upd->memtype);
        free(upd->filename);
        free(upd);
        return NULL;
    }
    p++;
  }

  if (*p != 0) {
    fprintf(stderr, 
            "%s: WARNING, extraneous data (%s) in update specifier ignored\n",
            progname, p);
  }
  
  return upd;
}


UPDATE * dup_update(UPDATE * upd)
{
  UPDATE * u;

  u = (UPDATE *)malloc(sizeof(UPDATE));
  if (u == NULL) {
    fprintf(stderr, "%s: out of memory\n", progname);
    exit(1);
  }

  memcpy(u, upd, sizeof(UPDATE));

  u->memtype = strdup(upd->memtype);
  u->filename = strdup(upd->filename);

  return u;
}



UPDATE * new_update(int op, char * memtype, int filefmt, char * filename)
{
  UPDATE * u;

  u = (UPDATE *)malloc(sizeof(UPDATE));
  if (u == NULL) {
    fprintf(stderr, "%s: out of memory\n", progname);
    exit(1);
  }

  u->memtype = strdup(memtype);
  u->filename = strdup(filename);
  u->op = op;
  u->format = filefmt;

  return u;
}



int do_op(PROGRAMMER * pgm, struct avrpart * p, UPDATE * upd, int nowrite, 
          int verify)
{
  struct avrpart * v;
  AVRMEM * mem;
  long size, vsize;
  int rc;

  mem = avr_locate_mem(p, upd->memtype);
  if (mem == NULL) {
    fprintf(stderr, "\"%s\" memory type not defined for part \"%s\"\n",
            upd->memtype, p->desc);
    return -1;
  }

  if (upd->op == DEVICE_READ) {
    /*
     * read out the specified device memory and write it to a file 
     */
    fprintf(stderr, "%s: reading %s memory:\n", 
            progname, upd->memtype);
    report_progress(0,1,"Reading");
    rc = avr_read(pgm, p, upd->memtype, 0, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, upd->memtype, rc);
      return -1;
    }
    report_progress(1,1,NULL);
    size = rc;

    fprintf(stderr, 
            "%s: writing output file \"%s\"\n",
            progname, 
            strcmp(upd->filename, "-")==0 ? "<stdout>" : upd->filename);
    rc = fileio(FIO_WRITE, upd->filename, upd->format, p, upd->memtype, size);
    if (rc < 0) {
      fprintf(stderr, "%s: write to file '%s' failed\n", 
              progname, upd->filename);
      return -1;
    }
  }
  else if (upd->op == DEVICE_WRITE) {
   /*
     * write the selected device memory using data from a file; first
     * read the data from the specified file
     */
    fprintf(stderr, 
            "%s: reading input file \"%s\"\n",
            progname, 
            strcmp(upd->filename, "-")==0 ? "<stdin>" : upd->filename);
    rc = fileio(FIO_READ, upd->filename, upd->format, p, upd->memtype, -1);
    if (rc < 0) {
      fprintf(stderr, "%s: write to file '%s' failed\n", 
              progname, upd->filename);
      return -1;
    }
    size = rc;

    /*
     * write the buffer contents to the selected memory type
     */
    fprintf(stderr, "%s: writing %s (%d bytes):\n", 
            progname, upd->memtype, size);

    if (!nowrite) {
      report_progress(0,1,"Writing");
      rc = avr_write(pgm, p, upd->memtype, size, 1);
      report_progress(1,1,NULL);
    }
    else {
      /* 
       * test mode, don't actually write to the chip, output the buffer
       * to stdout in intel hex instead 
       */
      rc = fileio(FIO_WRITE, "-", FMT_IHEX, p, upd->memtype, size);
    }

    if (rc < 0) {
      fprintf(stderr, "%s: failed to write %s memory, rc=%d\n", 
                progname, upd->memtype, rc);
      return -1;
    }

    vsize = rc;

    fprintf(stderr, "%s: %d bytes of %s written\n", progname, 
            vsize, upd->memtype);

  }
  else if (upd->op == DEVICE_VERIFY) {
    /* 
     * verify that the in memory file (p->mem[AVR_M_FLASH|AVR_M_EEPROM])
     * is the same as what is on the chip 
     */
    pgm->vfy_led(pgm, ON);

    v = avr_dup_part(p);

    fprintf(stderr, "%s: verifying %s memory against %s:\n", 
            progname, upd->memtype, upd->filename);

    fprintf(stderr, "%s: load data %s data from input file %s:\n",
            progname, upd->memtype, upd->filename);

    rc = fileio(FIO_READ, upd->filename, upd->format, p, upd->memtype, -1);
    if (rc < 0) {
      fprintf(stderr, "%s: read from file '%s' failed\n", 
              progname, upd->filename);
      return -1;
    }
    size = rc;
    fprintf(stderr, "%s: input file %s contains %ld bytes\n", 
            progname, upd->filename, size);



    fprintf(stderr, "%s: reading on-chip %s data:\n", 
            progname, upd->memtype);

    report_progress (0,1,"Reading");
    rc = avr_read(pgm, v, upd->memtype, size, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, upd->memtype, rc);
      pgm->err_led(pgm, ON);
      return -1;
    }
    report_progress (1,1,NULL);



    fprintf(stderr, "%s: verifying ...\n", progname);
    rc = avr_verify(p, v, upd->memtype, size);
    if (rc < 0) {
      fprintf(stderr, "%s: verification error; content mismatch\n", 
              progname);
      pgm->err_led(pgm, ON);
      return -1;
    }
    
    fprintf(stderr, "%s: %d bytes of %s verified\n", 
            progname, rc, upd->memtype);

    pgm->vfy_led(pgm, OFF);
  }
  else {
    fprintf(stderr, "%s: invalid update operation (%d) requested\n",
            progname, upd->op);
    return -1;
  }

  return 0;
}

/*
 * main routine
 */
int main(int argc, char * argv [])
{
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
  AVRMEM         * sig;         /* signature data */
  struct stat      sb;
  UPDATE         * upd;
  LNODEID        * ln;

  /* options / operating mode variables */
  char *  memtype;     /* "flash", "eeprom", etc */
  int     doread;      /* 1=reading AVR */
  int     dowrite;     /* 1=writing AVR */
  int     erase;       /* 1=erase chip, 0=don't */
  char  * outputf;     /* output file name */
  char  * inputf;      /* input file name */
  int     ovsigck;     /* 1=override sig check, 0=don't */
  char  * port;        /* device port (/dev/xxx) */
  int     terminal;    /* 1=enter terminal mode, 0=don't */
  FILEFMT filefmt;     /* FMT_AUTO, FMT_IHEX, FMT_SREC, FMT_RBIN */
  int     nowrite;     /* don't actually write anything to the chip */
  int     verify;      /* perform a verify operation */
  int     ppisetbits;  /* bits to set in ppi data register at exit */
  int     ppiclrbits;  /* bits to clear in ppi data register at exit */
  char  * exitspecs;   /* exit specs string from command line */
  char  * programmer;  /* programmer id */
  char  * partdesc;    /* part id */
  char    sys_config[PATH_MAX]; /* system wide config file */
  char    usr_config[PATH_MAX]; /* per-user config file */
  int     cycles;      /* erase-rewrite cycles */
  int     set_cycles;  /* value to set the erase-rewrite cycles to */
  char  * e;           /* for strtol() error checking */
  int     quell_progress;

  progname = rindex(argv[0],'/');
  if (progname)
    progname++;
  else
    progname = argv[0];

  default_parallel[0] = 0;
  default_serial[0]   = 0;

  init_config();

  updates = lcreat(NULL, 0);
  if (updates == NULL) {
    fprintf(stderr, "%s: cannot initialize updater list\n", progname);
    exit(1);
  }

  partdesc      = NULL;
  readorwrite   = 0;
  port          = default_parallel;
  outputf       = NULL;
  inputf        = NULL;
  doread        = 0;
  dowrite       = 0;
  memtype       = "flash";
  erase         = 0;
  p             = NULL;
  ovsigck       = 0;
  terminal      = 0;
  filefmt       = FMT_AUTO;
  nowrite       = 0;
  verify        = 1;        /* on by default */
  quell_progress = 0;
  ppisetbits    = 0;
  ppiclrbits    = 0;
  exitspecs     = NULL;
  pgm           = NULL;
  programmer    = default_programmer;
  verbose       = 0;
  do_cycles     = 0;
  set_cycles    = -1;


  #if defined(__CYGWIN__)

  win_sys_config_set(sys_config);
  win_usr_config_set(usr_config);
  
  #else
  char  * homedir;

  strcpy(sys_config, CONFIG_DIR);
  i = strlen(sys_config);
  if (i && (sys_config[i-1] != '/'))
    strcat(sys_config, "/");
  strcat(sys_config, "avrdude.conf");

  usr_config[0] = 0;
  homedir = getenv("HOME");
  if (homedir != NULL) {
    strcpy(usr_config, homedir);
    i = strlen(usr_config);
    if (i && (usr_config[i-1] != '/'))
      strcat(usr_config, "/");
    strcat(usr_config, ".avrduderc");
  }
  
  #endif

  len = strlen(progname) + 2;
  for (i=0; i<len; i++)
    progbuf[i] = ' ';
  progbuf[i] = 0;

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
  while ((ch = getopt(argc,argv,"?c:C:eE:f:Fi:I:m:no:p:P:qtvU:VyY:")) != -1) {

    switch (ch) {
      case 'c': /* programmer id */
        programmer = optarg;
        break;

      case 'C': /* system wide configuration file */
        strncpy(sys_config, optarg, PATH_MAX);
        sys_config[PATH_MAX-1] = 0;
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

      case 'q' : /* Quell progress output */
        quell_progress = 1;
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
        dowrite = 1;
        inputf = optarg;
        break;

      case 'I': /* specify input file and assume 'immediate mode' */
        if (outputf || terminal) {
          fprintf(stderr,"%s: -o, -I, and -t are incompatible\n\n", progname);
          return 1;
        }
        dowrite = 1;
        inputf = optarg;
        filefmt = FMT_IMM;
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
          case 's' : filefmt = FMT_SREC; break;
          case 'm' : filefmt = FMT_IMM; break;
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
        port = optarg;
        break;

      case 'U':
        upd = parse_op(optarg);
        if (upd == NULL) {
          fprintf(stderr, "%s: error parsing update operation '%s'\n",
                  progname, optarg);
          exit(1);
        }
        ladd(updates, upd);

        if (verify && upd->op == DEVICE_WRITE) {
          upd = dup_update(upd);
          upd->op = DEVICE_VERIFY;
          ladd(updates, upd);
        }
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

  if (quell_progress == 0) {
    if (isatty (STDERR_FILENO))
      update_progress = update_progress_tty;
    else
      update_progress = update_progress_no_tty;
  }

  if (verbose) {
    /*
     * Print out an identifying string so folks can tell what version
     * they are running
     */
    fprintf(stderr, 
            "\n%s: Version %s\n"
            "%sCopyright (c) 2000-2003 Brian Dean, bsd@bsdhome.com\n\n", 
            progname, version, progbuf);
  }

  if (verbose) {
    fprintf(stderr, "%sSystem wide configuration file is \"%s\"\n",
            progbuf, sys_config);
  }

  rc = read_config(sys_config);
  if (rc) {
    fprintf(stderr, 
            "%s: error reading system wide configuration file \"%s\"\n",
            progname, sys_config);
    exit(1);
  }

  if (usr_config[0] != 0) {
    if (verbose) {
      fprintf(stderr, "%sUser configuration file is \"%s\"\n",
              progbuf, usr_config);
    }

    rc = stat(usr_config, &sb);
    if ((rc < 0) || ((sb.st_mode & S_IFREG) == 0)) {
      if (verbose) {
        fprintf(stderr,
                "%sUser configuration file does not exist or is not a "
                "regular file, skipping\n",
                progbuf);
      }
    }
    else {
      rc = read_config(usr_config);
      if (rc) {
        fprintf(stderr, "%s: error reading user configuration file \"%s\"\n",
                progname, usr_config);
        exit(1);
      }
    }
  }

  if (verbose) {
    fprintf(stderr, "\n");
  }

  if (partdesc) {
    if (strcmp(partdesc, "?") == 0) {
      fprintf(stderr, "\n");
      fprintf(stderr,"Valid parts are:\n");
      list_parts(stderr, "  ", part_list);
      fprintf(stderr, "\n");
      exit(1);
    }
  }

  if (programmer) {
    if (strcmp(programmer, "?") == 0) {
      fprintf(stderr, "\n");
      fprintf(stderr,"Valid programmers are:\n");
      list_programmers(stderr, "  ", programmers);
      fprintf(stderr,"\n");
      exit(1);
    }
  }


  if (programmer[0] == 0) {
    fprintf(stderr, 
            "\n%s: no programmer has been specified on the command line "
            "or the config file\n", 
            progname);
    fprintf(stderr, 
            "%sSpecify a programmer using the -c option and try again\n\n",
            progbuf);
    exit(1);
  }

  pgm = locate_programmer(programmers, programmer);
  if (pgm == NULL) {
    fprintf(stderr,"\n");
    fprintf(stderr, 
            "%s: Can't find programmer id \"%s\"\n",
            progname, programmer);
    fprintf(stderr,"\nValid programmers are:\n");
    list_programmers(stderr, "  ", programmers);
    fprintf(stderr,"\n");
    exit(1);
  }

  if ((strcmp(pgm->type, "STK500") == 0)
      || (strcmp(pgm->type, "avr910") == 0)){
    if (port == default_parallel) {
      port = default_serial;
    }
  }

  if (partdesc == NULL) {
    fprintf(stderr, 
            "%s: No AVR part has been specified, use \"-p Part\"\n\n",
            progname);
    fprintf(stderr,"Valid parts are:\n");
    list_parts(stderr, "  ", part_list);
    fprintf(stderr, "\n");
    exit(1);
  }


  p = locate_part(part_list, partdesc);
  if (p == NULL) {
    fprintf(stderr, 
            "%s: AVR Part \"%s\" not found.\n\n",
            progname, partdesc);
    fprintf(stderr,"Valid parts are:\n");
    list_parts(stderr, "  ", part_list);
    fprintf(stderr, "\n");
    exit(1);
  }


  if (exitspecs != NULL) {
    if (strcmp(pgm->type, "PPI") != 0) {
      fprintf(stderr, 
              "%s: WARNING: -E option is only valid with \"PPI\" "
              "programmer types\n",
              progname);
      exitspecs = NULL;
    }
    else if (getexitspecs(exitspecs, &ppisetbits, &ppiclrbits) < 0) {
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

  if (strcmp(pgm->type, "PPI") == 0) {
    verify_pin_assigned(PIN_AVR_RESET, "AVR RESET");
    verify_pin_assigned(PIN_AVR_SCK,   "AVR SCK");
    verify_pin_assigned(PIN_AVR_MISO,  "AVR MISO");
    verify_pin_assigned(PIN_AVR_MOSI,  "AVR MOSI");
  }

  /*
   * open the programmer
   */
  if (port[0] == 0) {
    fprintf(stderr, "\n%s: no port has been specified on the command line "
            "or the config file\n", 
            progname);
    fprintf(stderr, "%sSpecify a port using the -P option and try again\n\n",
            progbuf);
    exit(1);
  }

  if (verbose) {
    fprintf(stderr, "%sUsing Port            : %s\n", progbuf, port);
    fprintf(stderr, "%sUsing Programmer      : %s\n", progbuf, programmer);
  }

  pgm->open(pgm, port);

  if (verbose) {
    avr_display(stderr, p, progbuf, verbose);
    fprintf(stderr, "\n");
    programmer_display(progbuf);
  }

  fprintf(stderr, "\n");

  exitrc = 0;

  /*
   * allow the programmer to save its state
   */
  rc = pgm->save(pgm);
  if (rc < 0) {
    exitrc = 1;
    ppidata = 0; /* clear all bits at exit */
    goto main_exit;
  }

  if (strcmp(pgm->type, "PPI") == 0) {
    pgm->ppidata &= ~ppiclrbits;
    pgm->ppidata |= ppisetbits;
  }

  /*
   * enable the programmer
   */
  pgm->enable(pgm);

  /*
   * turn off all the status leds
   */
  pgm->rdy_led(pgm, OFF);
  pgm->err_led(pgm, OFF);
  pgm->pgm_led(pgm, OFF);
  pgm->vfy_led(pgm, OFF);

  /*
   * initialize the chip in preperation for accepting commands
   */
  rc = pgm->initialize(pgm, p);
  if (rc < 0) {
    fprintf(stderr, "%s: initialization failed, rc=%d\n", progname, rc);
    exitrc = 1;
    goto main_exit;
  }

  /* indicate ready */
  pgm->rdy_led(pgm, ON);

  fprintf(stderr, 
            "%s: AVR device initialized and ready to accept instructions\n",
            progname);

  /*
   * Let's read the signature bytes to make sure there is at least a
   * chip on the other end that is responding correctly.  A check
   * against 0xffffffff should ensure that the signature bytes are
   * valid.  
   */
  rc = avr_signature(pgm, p);
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
    rc = avr_get_cycle_count(pgm, p, &cycles);
    if (rc == 0) {
      /*
       * only attempt to update the cycle counter if we can actually
       * read the old value
       */
      cycles = set_cycles;
      fprintf(stderr, "%s: setting erase-rewrite cycle count to %d\n", 
              progname, cycles);
      rc = avr_put_cycle_count(pgm, p, cycles);
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
    pgm->chip_erase(pgm, p);
    fprintf(stderr, "%s: done.\n", progname);
  }
  else if (set_cycles == -1) {
    /*
     * The erase routine displays this same information, so don't
     * repeat it if an erase was done.  Also, don't display this if we
     * set the cycle count (due to -Y).
     *
     * see if the cycle count in the last four bytes of eeprom seems
     * reasonable 
     */
    rc = avr_get_cycle_count(pgm, p, &cycles);
    if ((rc >= 0) && (cycles != 0xffffffff)) {
      fprintf(stderr,
              "%s: current erase-rewrite cycle count is %d%s\n",
              progname, cycles, 
              do_cycles ? "" : " (if being tracked)");
    }
  }



  if (!terminal && (lsize(updates) == 0) &&
      ((inputf==NULL) && (outputf==NULL))) {
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

  if (doread && dowrite) {
    fprintf(stderr, "%s: can't be reading and writing from/to the same file\n",
            progname);
    exitrc = 1;
    goto main_exit;
  }

  if (terminal) {
    /*
     * terminal mode
     */
    exitrc = terminal_mode(pgm, p);
  }
  else if (doread) {
    upd = new_update(DEVICE_READ, memtype, filefmt, outputf);
    ladd(updates, upd);
  }
  else if (dowrite) {
    upd = new_update(DEVICE_WRITE, memtype, filefmt, inputf);
    ladd(updates, upd);
  }

  if (!doread && verify && inputf) {
    upd = new_update(DEVICE_VERIFY, memtype, filefmt, inputf);
    ladd(updates, upd);
  }

  for (ln=lfirst(updates); ln; ln=lnext(ln)) {
    upd = ldata(ln);
    rc = do_op(pgm, p, upd, nowrite, verify);
    if (rc) {
      exitrc = 1;
      break;
    }
  }

 main_exit:

  /*
   * program complete
   */

  pgm->powerdown(pgm);

  /*
   * restore programmer state
   */
  pgm->restore(pgm);

  pgm->disable(pgm);

  pgm->rdy_led(pgm, OFF);

  pgm->close(pgm);

  fprintf(stderr, "\n%s done.  Thank you.\n\n", progname);

  return exitrc;
}

