/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005  Brian S. Dean <bsd@bsdhome.com>
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
 * Code to program an Atmel AVR device through one of the supported
 * programmers.
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
#include "term.h"
#include "safemode.h"


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
int    quell_progress; /* un-verebose output */
int    ovsigck;     /* 1=override sig check, 0=don't */
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
 "  -b <baudrate>              Override RS-232 baud rate.\n"
 "  -B <bitclock>              Specify JTAG/STK500v2 bit clock period (us).\n"
 "  -C <config-file>           Specify location of configuration file.\n"
 "  -c <programmer>            Specify programmer type.\n"
 "  -D                         Disable auto erase for flash memory\n"
 "  -i <delay>                 ISP Clock Delay [in microseconds]\n"
 "  -P <port>                  Specify connection port.\n"
 "  -F                         Override invalid signature check.\n"
 "  -e                         Perform a chip erase.\n"
 "  -O                         Perform RC oscillator calibration (see AVR053). \n"
 "  -U <memtype>:r|w|v:<filename>[:format]\n"
 "                             Memory operation specification.\n"
 "                             Multiple -U options are allowed, each request\n"
 "                             is performed in the order specified.\n"
 "  -n                         Do not write anything to the device.\n"
 "  -V                         Do not verify.\n"
 "  -u                         Disable safemode, default when running from a script.\n"
 "  -s                         Silent safemode operation, will not ask you if\n"
 "                             fuses should be changed back.\n"
 "  -t                         Enter terminal mode.\n"
 "  -E <exitspec>[,<exitspec>] List programmer exit specifications.\n"
 "  -y                         Count # erase cycles in EEPROM.\n"
 "  -Y <number>                Initialize erase cycle # in EEPROM.\n"
 "  -v                         Verbose output. -v -v for more.\n"
 "  -q                         Quell progress output. -q -q for less.\n"
 "  -?                         Display this usage.\n"
 "\navrdude project: <URL:http://savannah.nongnu.org/projects/avrdude>\n"
          ,progname);
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

/*
 * Report the progress of a read or write operation from/to the
 * device.
 * 
 * The first call of report_progress() should look like this (for a write op):
 * 
 * report_progress (0, 1, "Writing");
 *
 * Then hdr should be passed NULL on subsequent calls while the
 * operation is progressing. Once the operation is complete, a final
 * call should be made as such to ensure proper termination of the
 * progress report:
 * 
 * report_progress (1, 1, NULL);
 *
 * It would be nice if we could reduce the usage to one and only one
 * call for each of start, during and end cases. As things stand now,
 * that is not possible and makes maintenance a bit more work.
 */
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
  static int done = 0;
  static int last = 0;
  int cnt = (percent>>1)*2;

  if (hdr) {
    fprintf (stderr, "\n%s | ", hdr);
    last = 0;
    done = 0;
  }
  else {
    while ((cnt > last) && (done == 0)) {
      fprintf (stderr, "#");
      cnt -=  2;
    }
  }

  if ((percent == 100) && (done == 0)) {
    fprintf (stderr, " | 100%% %0.2fs\n\n", etime);
    last = 0;
    done = 1;
  }
  else
    last = (percent>>1)*2;    /* Make last a multiple of 2. */
}


UPDATE * parse_op(char * s)
{
  char buf[1024];
  char * p, * cp, c;
  UPDATE * upd;
  int i;
  size_t fnlen;

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
    upd->memtype = (char *)malloc(strlen("flash")+1);
    if (upd->memtype == NULL) {
      outofmem:
      fprintf(stderr, "%s: out of memory\n", progname);
      exit(1);
    }
    strcpy(upd->memtype, "flash");
    upd->op = DEVICE_WRITE;
    upd->filename = (char *)malloc(strlen(buf) + 1);
    if (upd->filename == NULL)
      goto outofmem;
    strcpy(upd->filename, buf);
    upd->format = FMT_AUTO;
    return upd;
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

  /*
   * Now, parse the filename component.  Instead of looking for the
   * leftmost possible colon delimiter, we look for the rightmost one.
   * If we found one, we do have a trailing :format specifier, and
   * process it.  Otherwise, the remainder of the string is our file
   * name component.  That way, the file name itself is allowed to
   * contain a colon itself (e. g. C:/some/file.hex), except the
   * optional format specifier becomes mandatory then.
   */
  cp = p;
  p = strrchr(cp, ':');
  if (p == NULL) {
    upd->format = FMT_AUTO;
    fnlen = strlen(cp);
    upd->filename = (char *)malloc(fnlen + 1);
  } else {
    fnlen = p - cp;
    upd->filename = (char *)malloc(fnlen +1);
    c = *++p;
    if (c && p[1])
      /* More than one char - force failure below. */
      c = '?';
    switch (c) {
      case 'a': upd->format = FMT_AUTO; break;
      case 's': upd->format = FMT_SREC; break;
      case 'i': upd->format = FMT_IHEX; break;
      case 'r': upd->format = FMT_RBIN; break;
      case 'm': upd->format = FMT_IMM; break;
      case 'b': upd->format = FMT_BIN; break;
      case 'd': upd->format = FMT_DEC; break;
      case 'h': upd->format = FMT_HEX; break;
      case 'o': upd->format = FMT_OCT; break;
      default:
        fprintf(stderr, "%s: invalid file format '%s' in update specifier\n",
                progname, p);
        free(upd->memtype);
        free(upd);
        return NULL;
    }
  }

  if (upd->filename == NULL) {
    fprintf(stderr, "%s: out of memory\n", progname);
    free(upd->memtype);
    free(upd);
    return NULL;
  }
  memcpy(upd->filename, cp, fnlen);
  upd->filename[fnlen] = 0;

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
  int size, vsize;
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
    if (quell_progress < 2) {
      fprintf(stderr, "%s: reading %s memory:\n", 
            progname, mem->desc);
	  }
    report_progress(0,1,"Reading");
    rc = avr_read(pgm, p, upd->memtype, 0, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, mem->desc, rc);
      return -1;
    }
    report_progress(1,1,NULL);
    size = rc;

    if (quell_progress < 2) {
      fprintf(stderr, 
            "%s: writing output file \"%s\"\n",
            progname, 
            strcmp(upd->filename, "-")==0 ? "<stdout>" : upd->filename);
    }
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
    if (quell_progress < 2) {
      fprintf(stderr, 
            "%s: reading input file \"%s\"\n",
            progname, 
            strcmp(upd->filename, "-")==0 ? "<stdin>" : upd->filename);
    }
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
    if (quell_progress < 2) {
      fprintf(stderr, "%s: writing %s (%d bytes):\n", 
            progname, mem->desc, size);
	  }

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
              progname, mem->desc, rc);
      return -1;
    }

    vsize = rc;

    if (quell_progress < 2) {
      fprintf(stderr, "%s: %d bytes of %s written\n", progname, 
            vsize, mem->desc);
    }

  }
  else if (upd->op == DEVICE_VERIFY) {
    /* 
     * verify that the in memory file (p->mem[AVR_M_FLASH|AVR_M_EEPROM])
     * is the same as what is on the chip 
     */
    pgm->vfy_led(pgm, ON);

    v = avr_dup_part(p);

    if (quell_progress < 2) {
      fprintf(stderr, "%s: verifying %s memory against %s:\n", 
            progname, mem->desc, upd->filename);

      fprintf(stderr, "%s: load data %s data from input file %s:\n",
            progname, mem->desc, upd->filename);
    }

    rc = fileio(FIO_READ, upd->filename, upd->format, p, upd->memtype, -1);
    if (rc < 0) {
      fprintf(stderr, "%s: read from file '%s' failed\n", 
              progname, upd->filename);
      return -1;
    }
    size = rc;
    if (quell_progress < 2) {
      fprintf(stderr, "%s: input file %s contains %d bytes\n", 
            progname, upd->filename, size);
      fprintf(stderr, "%s: reading on-chip %s data:\n", 
            progname, mem->desc);
    }

    report_progress (0,1,"Reading");
    rc = avr_read(pgm, v, upd->memtype, size, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: failed to read all of %s memory, rc=%d\n", 
              progname, mem->desc, rc);
      pgm->err_led(pgm, ON);
      return -1;
    }
    report_progress (1,1,NULL);



    if (quell_progress < 2) {
      fprintf(stderr, "%s: verifying ...\n", progname);
    }
    rc = avr_verify(p, v, upd->memtype, size);
    if (rc < 0) {
      fprintf(stderr, "%s: verification error; content mismatch\n", 
              progname);
      pgm->err_led(pgm, ON);
      return -1;
    }

    if (quell_progress < 2) {
      fprintf(stderr, "%s: %d bytes of %s verified\n", 
              progname, rc, mem->desc);
    }

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
  int              len;         /* length for various strings */
  struct avrpart * p;           /* which avr part we are programming */
  struct avrpart * v;           /* used for verify */
  AVRMEM         * sig;         /* signature data */
  struct stat      sb;
  UPDATE         * upd;
  LNODEID        * ln;

  /* options / operating mode variables */
  int     erase;       /* 1=erase chip, 0=don't */
  int     calibrate;   /* 1=calibrate RC oscillator, 0=don't */
  int     auto_erase;  /* 0=never erase unless explicity told to do
                          so, 1=erase if we are going to program flash */
  char  * port;        /* device port (/dev/xxx) */
  int     terminal;    /* 1=enter terminal mode, 0=don't */
  int     nowrite;     /* don't actually write anything to the chip */
  int     verify;      /* perform a verify operation */
  char  * exitspecs;   /* exit specs string from command line */
  char  * programmer;  /* programmer id */
  char  * partdesc;    /* part id */
  char    sys_config[PATH_MAX]; /* system wide config file */
  char    usr_config[PATH_MAX]; /* per-user config file */
  int     cycles;      /* erase-rewrite cycles */
  int     set_cycles;  /* value to set the erase-rewrite cycles to */
  char  * e;           /* for strtol() error checking */
  int     baudrate;    /* override default programmer baud rate */
  double  bitclock;    /* Specify programmer bit clock (JTAG ICE) */
  int     ispdelay;    /* Specify the delay for ISP clock */
  int     safemode;    /* Enable safemode, 1=safemode on, 0=normal */
  int     silentsafe;  /* Don't ask about fuses, 1=silent, 0=normal */
  unsigned char safemode_lfuse = 0xff;
  unsigned char safemode_hfuse = 0xff;
  unsigned char safemode_efuse = 0xff;
  unsigned char safemode_fuse = 0xff;

  char * safemode_response;
  int fuses_specified = 0;
  int fuses_updated = 0;
#if !defined(WIN32NATIVE)
  char  * homedir;
#endif

  progname = strrchr(argv[0],'/');

#if defined (WIN32NATIVE)
  /* take care of backslash as dir sep in W32 */
  if (!progname) progname = strrchr(argv[0],'\\');
#endif /* WIN32NATIVE */

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
  port          = default_parallel;
  erase         = 0;
  calibrate     = 0;
  auto_erase    = 1;
  p             = NULL;
  ovsigck       = 0;
  terminal      = 0;
  nowrite       = 0;
  verify        = 1;        /* on by default */
  quell_progress = 0;
  exitspecs     = NULL;
  pgm           = NULL;
  programmer    = default_programmer;
  verbose       = 0;
  do_cycles     = 0;
  set_cycles    = -1;
  baudrate      = 0;
  bitclock      = 0.0;
  ispdelay      = 0;
  safemode      = 1;       /* Safemode on by default */
  silentsafe    = 0;       /* Ask by default */
  
  if (isatty(STDIN_FILENO) == 0)
      safemode  = 0;       /* Turn off safemode if this isn't a terminal */



#if defined(WIN32NATIVE)

  win_sys_config_set(sys_config);
  win_usr_config_set(usr_config);

#else

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
  while ((ch = getopt(argc,argv,"?b:B:c:C:DeE:Fi:np:OP:qstU:uvVyY:")) != -1) {

    switch (ch) {
      case 'b': /* override default programmer baud rate */
        baudrate = strtol(optarg, &e, 0);
        if ((e == optarg) || (*e != 0)) {
          fprintf(stderr, "%s: invalid baud rate specified '%s'\n",
                  progname, optarg);
          exit(1);
        }
        break;

      case 'B':	/* specify JTAG ICE bit clock period */
	bitclock = strtod(optarg, &e);
	if ((e == optarg) || (*e != 0) || bitclock == 0.0) {
	  fprintf(stderr, "%s: invalid bit clock period specified '%s'\n",
                  progname, optarg);
          exit(1);
        }
        break;

      case 'i':	/* specify isp clock delay */
	ispdelay = strtol(optarg, &e,10);
	if ((e == optarg) || (*e != 0) || ispdelay == 0) {
	  fprintf(stderr, "%s: invalid isp clock delay specified '%s'\n",
                  progname, optarg);
          exit(1);
        }
        break;

      case 'c': /* programmer id */
        programmer = optarg;
        break;

      case 'C': /* system wide configuration file */
        strncpy(sys_config, optarg, PATH_MAX);
        sys_config[PATH_MAX-1] = 0;
        break;

      case 'D': /* disable auto erase */
        auto_erase = 0;
        break;

      case 'e': /* perform a chip erase */
        erase = 1;
        break;

      case 'E':
        exitspecs = optarg;
        break;

      case 'F': /* override invalid signature check */
        ovsigck = 1;
        break;

      case 'n':
        nowrite = 1;
        break;

      case 'O': /* perform RC oscillator calibration */
	calibrate = 1;
	break;

      case 'p' : /* specify AVR part */
        partdesc = optarg;
        break;

      case 'P':
        port = optarg;
        break;

      case 'q' : /* Quell progress output */
        quell_progress++ ;
        break;

      case 's' : /* Silent safemode */
        silentsafe = 1;
        safemode = 1;
        break;
        
      case 't': /* enter terminal mode */
        terminal = 1;
        break;

      case 'u' : /* Disable safemode */
        safemode = 0;
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
    else {
      update_progress = update_progress_no_tty;
      /* disable all buffering of stderr for compatibility with
         software that captures and redirects output to a GUI
         i.e. Programmers Notepad */
      setvbuf( stderr, NULL, _IONBF, 0 );
      setvbuf( stdout, NULL, _IONBF, 0 );
    }
  }

  if (verbose) {
    /*
     * Print out an identifying string so folks can tell what version
     * they are running
     */
    fprintf(stderr,
            "\n%s: Version %s, compiled on %s at %s\n"
            "%sCopyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/\n\n",
            progname, version, __DATE__, __TIME__, progbuf);
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

  if ((strcmp(pgm->type, "STK500") == 0) ||
      (strcmp(pgm->type, "avr910") == 0) ||
      (strcmp(pgm->type, "STK500V2") == 0) ||
      (strcmp(pgm->type, "JTAGMKII") == 0)) {
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
    if (pgm->parseexitspecs == NULL) {
      fprintf(stderr,
              "%s: WARNING: -E option not supported by this programmer type\n",
              progname);
      exitspecs = NULL;
    }
    else if (pgm->parseexitspecs(pgm, exitspecs) < 0) {
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

  if (baudrate != 0) {
    if (verbose) {
      fprintf(stderr, "%sOverriding Baud Rate  : %d\n", progbuf, baudrate);
    }
    pgm->baudrate = baudrate;
  }

  if (bitclock != 0.0) {
    if (verbose) {
      fprintf(stderr, "%sSetting bit clk period: %.1f\n", progbuf, bitclock);
    }

    pgm->bitclock = bitclock * 1e-6;
  }

  if (ispdelay != 0) {
    if (verbose) {
      fprintf(stderr, "%sSetting isp clock delay: %3i\n", progbuf, ispdelay);
    }
    pgm->ispdelay = ispdelay;
  }

  rc = pgm->open(pgm, port);
  if (rc < 0) {
    exitrc = 1;
    pgm->ppidata = 0; /* clear all bits at exit */
    goto main_exit;
  }

  if (calibrate) {
    /*
     * perform an RC oscillator calibration
     * as outlined in appnote AVR053
     */
    fprintf(stderr, "%s: performing RC oscillator calibration\n", progname);
    exitrc = pgm->perform_osccal(pgm);
    if (exitrc == 0 && quell_progress < 2) {
      fprintf(stderr,
              "%s: calibration value is now stored in EEPROM at address 0\n",
              progname);
    }
    goto main_exit;
  }

  if (verbose) {
    avr_display(stderr, p, progbuf, verbose);
    fprintf(stderr, "\n");
    programmer_display(progbuf);
  }

  if (quell_progress < 2) {
    fprintf(stderr, "\n");
  }

  exitrc = 0;

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
    if (!ovsigck) {
      fprintf(stderr, "%sDouble check connections and try again, "
              "or use -F to override\n"
              "%sthis check.\n\n",
              progbuf, progbuf);
      exitrc = 1;
      goto main_exit;
    }
  }

  /* indicate ready */
  pgm->rdy_led(pgm, ON);

  if (quell_progress < 2) {
    fprintf(stderr,
            "%s: AVR device initialized and ready to accept instructions\n",
            progname);
  }

  /*
   * Let's read the signature bytes to make sure there is at least a
   * chip on the other end that is responding correctly.  A check
   * against 0xffffff / 0x000000 should ensure that the signature bytes
   * are valid.
   */
  rc = avr_signature(pgm, p);
  if (rc != 0) {
    fprintf(stderr, "%s: error reading signature data, rc=%d\n",
            progname, rc);
    exitrc = 1;
    goto main_exit;
  }

  sig = avr_locate_mem(p, "signature");
  if (sig == NULL) {
    fprintf(stderr,
            "%s: WARNING: signature data not defined for device \"%s\"\n",
            progname, p->desc);
  }

  if (sig != NULL) {
    int ff, zz;

    if (quell_progress < 2) {
      fprintf(stderr, "%s: Device signature = 0x", progname);
    }
    ff = zz = 1;
    for (i=0; i<sig->size; i++) {
      if (quell_progress < 2) {
        fprintf(stderr, "%02x", sig->buf[i]);
      }
      if (sig->buf[i] != 0xff)
        ff = 0;
      if (sig->buf[i] != 0x00)
        zz = 0;
    }
    if (quell_progress < 2) {
      fprintf(stderr, "\n");
    }

    if (ff || zz) {
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

    if (sig->size != 3 ||
	sig->buf[0] != p->signature[0] ||
	sig->buf[1] != p->signature[1] ||
	sig->buf[2] != p->signature[2]) {
      fprintf(stderr,
	      "%s: Expected signature for %s is %02X %02X %02X\n",
	      progname, p->desc,
	      p->signature[0], p->signature[1], p->signature[2]);
      if (!ovsigck) {
        fprintf(stderr, "%sDouble check chip, "
		"or use -F to override this check.\n",
                progbuf);
        exitrc = 1;
        goto main_exit;
      }
    }
  }

  if (safemode == 1) {
    /* If safemode is enabled, go ahead and read the current low, high,
       and extended fuse bytes as needed */

    if (safemode_readfuses(&safemode_lfuse, &safemode_hfuse,
                           &safemode_efuse, &safemode_fuse, pgm, p, verbose) != 0) {
      fprintf(stderr, "%s: safemode: To protect your AVR the programming "
              "will be aborted\n",
              progname);
      exitrc = 1;
      goto main_exit;
    }

    //Save the fuses as default
    safemode_memfuses(1, &safemode_lfuse, &safemode_hfuse, &safemode_efuse, &safemode_fuse);
  }




  if ((erase == 0) && (auto_erase == 1)) {
    AVRMEM * m;

    for (ln=lfirst(updates); ln; ln=lnext(ln)) {
      upd = ldata(ln);
      m = avr_locate_mem(p, upd->memtype);
      if (m == NULL)
        continue;
      if ((strcasecmp(m->desc, "flash") == 0) && (upd->op == DEVICE_WRITE)) {
        erase = 1;
        if (quell_progress < 2) {
          fprintf(stderr,
                "%s: NOTE: FLASH memory has been specified, an erase cycle "
                "will be performed\n"
                "%sTo disable this feature, specify the -D option.\n",
                progname, progbuf);
        }
        break;
      }
    }
  }

  /*
   * Display cycle count, if and only if it is not set later on.
   *
   * The cycle count will be displayed anytime it will be changed later.
   */
  if ((set_cycles == -1) && ((erase == 0) || (do_cycles == 0))) {
    /*
     * see if the cycle count in the last four bytes of eeprom seems
     * reasonable
     */
    rc = avr_get_cycle_count(pgm, p, &cycles);
    if (quell_progress < 2) {
      if ((rc >= 0) && (cycles != 0)) {
        fprintf(stderr,
              "%s: current erase-rewrite cycle count is %d%s\n",
              progname, cycles,
              do_cycles ? "" : " (if being tracked)");
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
      if (quell_progress < 2) {
        fprintf(stderr, "%s: setting erase-rewrite cycle count to %d\n",
              progname, cycles);
      }
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
    if (quell_progress < 2) {
      fprintf(stderr, "%s: erasing chip\n", progname);
    }
    avr_chip_erase(pgm, p);
  }


  if (terminal) {
    /*
     * terminal mode
     */         
    exitrc = terminal_mode(pgm, p);
  }


  for (ln=lfirst(updates); ln; ln=lnext(ln)) {
    upd = ldata(ln);
    rc = do_op(pgm, p, upd, nowrite, verify);
    if (rc) {
      exitrc = 1;
      break;
    }
  }

  /* Right before we exit programming mode, which will make the fuse
     bits active, check to make sure they are still correct */
  if (safemode == 1){
    /* If safemode is enabled, go ahead and read the current low,
     * high, and extended fuse bytes as needed */
    unsigned char safemodeafter_lfuse = 0xff;
    unsigned char safemodeafter_hfuse = 0xff;
    unsigned char safemodeafter_efuse = 0xff;
    unsigned char safemodeafter_fuse  = 0xff;
    unsigned char failures = 0;
    char yes[1] = {'y'};

    if (quell_progress < 2) {
      fprintf(stderr, "\n");
    }

    //Restore the default fuse values
    safemode_memfuses(0, &safemode_lfuse, &safemode_hfuse, &safemode_efuse, &safemode_fuse);

    /* Try reading back fuses, make sure they are reliable to read back */
    if (safemode_readfuses(&safemodeafter_lfuse, &safemodeafter_hfuse,
                           &safemodeafter_efuse, &safemodeafter_fuse, pgm, p, verbose) != 0) {
      /* Uh-oh.. try once more to read back fuses */
      if (safemode_readfuses(&safemodeafter_lfuse, &safemodeafter_hfuse,
                             &safemodeafter_efuse, &safemodeafter_fuse, pgm, p, verbose) != 0) { 
        fprintf(stderr,
                "%s: safemode: Sorry, reading back fuses was unreliable. "
                "I have given up and exited programming mode\n",
                progname);
        exitrc = 1;
        goto main_exit;		  
      }
    }
    
    /* Now check what fuses are against what they should be */
    if (safemodeafter_fuse != safemode_fuse) {
      fuses_updated = 1;
      fprintf(stderr, "%s: safemode: fuse changed! Was %x, and is now %x\n",
              progname, safemode_fuse, safemodeafter_fuse);

              
      /* Ask user - should we change them */
       
       if (silentsafe == 0)
            safemode_response = terminal_get_input("Would you like this fuse to be changed back? [y/n] ");
       else
            safemode_response = yes;
       
       if (tolower(safemode_response[0]) == 'y') {
              
            /* Enough chit-chat, time to program some fuses and check them */
            if (safemode_writefuse (safemode_fuse, "fuse", pgm, p,
                                    10, verbose) == 0) {
                fprintf(stderr, "%s: safemode: and is now rescued\n", progname);
            }
            else {
                fprintf(stderr, "%s: and COULD NOT be changed\n", progname);
                failures++;
            }
      }
    }

    /* Now check what fuses are against what they should be */
    if (safemodeafter_lfuse != safemode_lfuse) {
      fuses_updated = 1;
      fprintf(stderr, "%s: safemode: lfuse changed! Was %x, and is now %x\n",
              progname, safemode_lfuse, safemodeafter_lfuse);

              
      /* Ask user - should we change them */
       
       if (silentsafe == 0)
            safemode_response = terminal_get_input("Would you like this fuse to be changed back? [y/n] ");
       else
            safemode_response = yes;
       
       if (tolower(safemode_response[0]) == 'y') {
              
            /* Enough chit-chat, time to program some fuses and check them */
            if (safemode_writefuse (safemode_lfuse, "lfuse", pgm, p,
                                    10, verbose) == 0) {
                fprintf(stderr, "%s: safemode: and is now rescued\n", progname);
            }
            else {
                fprintf(stderr, "%s: and COULD NOT be changed\n", progname);
                failures++;
            }
      }
    }

    /* Now check what fuses are against what they should be */
    if (safemodeafter_hfuse != safemode_hfuse) {
      fuses_updated = 1;
      fprintf(stderr, "%s: safemode: hfuse changed! Was %x, and is now %x\n",
              progname, safemode_hfuse, safemodeafter_hfuse);
              
      /* Ask user - should we change them */
       if (silentsafe == 0)
            safemode_response = terminal_get_input("Would you like this fuse to be changed back? [y/n] ");
       else
            safemode_response = yes;
       if (tolower(safemode_response[0]) == 'y') {

            /* Enough chit-chat, time to program some fuses and check them */
            if (safemode_writefuse(safemode_hfuse, "hfuse", pgm, p,
                                    10, verbose) == 0) {
                fprintf(stderr, "%s: safemode: and is now rescued\n", progname);
            }
            else {
                fprintf(stderr, "%s: and COULD NOT be changed\n", progname);
                failures++;
            }
      }
    }

    /* Now check what fuses are against what they should be */
    if (safemodeafter_efuse != safemode_efuse) {
      fuses_updated = 1;
      fprintf(stderr, "%s: safemode: efuse changed! Was %x, and is now %x\n",
              progname, safemode_efuse, safemodeafter_efuse);

      /* Ask user - should we change them */
       if (silentsafe == 0)
            safemode_response = terminal_get_input("Would you like this fuse to be changed back? [y/n] ");
       else
            safemode_response = yes;
       if (tolower(safemode_response[0]) == 'y') {
              
            /* Enough chit-chat, time to program some fuses and check them */
            if (safemode_writefuse (safemode_efuse, "efuse", pgm, p,
                                    10, verbose) == 0) {
                fprintf(stderr, "%s: safemode: and is now rescued\n", progname);
            }
            else {
                fprintf(stderr, "%s: and COULD NOT be changed\n", progname);
                failures++;
            }
       }
    }

    if (quell_progress < 2) {
      fprintf(stderr, "%s: safemode: ", progname);
      if (failures == 0) {
        fprintf(stderr, "Fuses OK\n");
      }
      else {
        fprintf(stderr, "Fuses not recovered, sorry\n");
      }
    }

    if (fuses_updated && fuses_specified) {
      exitrc = 1;
    }

  }


main_exit:

  /*
   * program complete
   */


  pgm->powerdown(pgm);

  pgm->disable(pgm);

  pgm->rdy_led(pgm, OFF);

  pgm->close(pgm);

  if (quell_progress < 2) {
    fprintf(stderr, "\n%s done.  Thank you.\n\n", progname);
  }

  return exitrc;
}
