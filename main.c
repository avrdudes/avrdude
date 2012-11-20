/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005  Brian S. Dean <bsd@bsdhome.com>
 * Copyright 2007-2009 Joerg Wunsch <j@uriah.heep.sax.de>
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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
#include "update.h"
#include "pgm_type.h"


/* Get VERSION from ac_cfg.h */
char * version      = VERSION;

char * progname;
char   progbuf[PATH_MAX]; /* temporary buffer of spaces the same
                             length as progname; used for lining up
                             multiline messages */

struct list_walk_cookie
{
    FILE *f;
    const char *prefix;
};

static LISTID updates = NULL;

static LISTID extended_params = NULL;

static LISTID additional_config_files = NULL;

static PROGRAMMER * pgm;

/*
 * global options
 */
int    do_cycles;   /* track erase-rewrite cycles */
int    verbose;     /* verbose output */
int    quell_progress; /* un-verebose output */
int    ovsigck;     /* 1=override sig check, 0=don't */


/*
 * usage message
 */
static void usage(void)
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
 "  -x <extended_param>        Pass <extended_param> to programmer.\n"
 "  -y                         Count # erase cycles in EEPROM.\n"
 "  -Y <number>                Initialize erase cycle # in EEPROM.\n"
 "  -v                         Verbose output. -v -v for more.\n"
 "  -q                         Quell progress output. -q -q for less.\n"
 "  -?                         Display this usage.\n"
 "\navrdude version %s, URL: <http://savannah.nongnu.org/projects/avrdude/>\n"
          ,progname, version);
}


static void update_progress_tty (int percent, double etime, char *hdr)
{
  static char hashes[51];
  static char *header;
  static int last = 0;
  int i;

  setvbuf(stderr, (char*)NULL, _IONBF, 0);

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
    if (!last) fprintf (stderr, "\n\n");
    last = 1;
  }

  setvbuf(stderr, (char*)NULL, _IOLBF, 0);
}

static void update_progress_no_tty (int percent, double etime, char *hdr)
{
  static int done = 0;
  static int last = 0;
  int cnt = (percent>>1)*2;

  setvbuf(stderr, (char*)NULL, _IONBF, 0);

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

  setvbuf(stderr, (char*)NULL, _IOLBF, 0);
}

static void list_programmers_callback(const char *name, const char *desc,
                                      const char *cfgname, int cfglineno,
                                      void *cookie)
{
    struct list_walk_cookie *c = (struct list_walk_cookie *)cookie;
    if (verbose){
        fprintf(c->f, "%s%-16s = %-30s [%s:%d]\n",
                c->prefix, name, desc, cfgname, cfglineno);
    } else {
        fprintf(c->f, "%s%-16s = %-s\n",
                c->prefix, name, desc);
    }
}

static void list_programmers(FILE * f, const char *prefix, LISTID programmers)
{
    struct list_walk_cookie c;

    c.f = f;
    c.prefix = prefix;

    sort_programmers(programmers);

    walk_programmers(programmers, list_programmers_callback, &c);
}

static void list_programmer_types_callback(const char *name, const char *desc,
                                      void *cookie)
{
    struct list_walk_cookie *c = (struct list_walk_cookie *)cookie;
    fprintf(c->f, "%s%-16s = %-s\n",
                c->prefix, name, desc);
}

static void list_programmer_types(FILE * f, const char *prefix)
{
    struct list_walk_cookie c;

    c.f = f;
    c.prefix = prefix;

    walk_programmer_types(list_programmer_types_callback, &c);
}

static void list_avrparts_callback(const char *name, const char *desc,
                                   const char *cfgname, int cfglineno,
                                   void *cookie)
{
    struct list_walk_cookie *c = (struct list_walk_cookie *)cookie;

    /* hide ids starting with '.' */
    if ((verbose < 2) && (name[0] == '.'))
        return;

    if (verbose) {
        fprintf(c->f, "%s%-8s = %-18s [%s:%d]\n",
                c->prefix, name, desc, cfgname, cfglineno);
    } else {
        fprintf(c->f, "%s%-8s = %s\n",
                c->prefix, name, desc);
    }
}

static void list_parts(FILE * f, const char *prefix, LISTID avrparts)
{
    struct list_walk_cookie c;

    c.f = f;
    c.prefix = prefix;

    sort_avrparts(avrparts);

    walk_avrparts(avrparts, list_avrparts_callback, &c);
}

static void exithook(void)
{
    if (pgm->teardown)
        pgm->teardown(pgm);
}

static void cleanup_main(void)
{
    if (updates) {
        ldestroy_cb(updates, (void(*)(void*))free_update);
        updates = NULL;
    }
    if (extended_params) {
        ldestroy(extended_params);
        extended_params = NULL;
    }
    if (additional_config_files) {
        ldestroy(additional_config_files);
        additional_config_files = NULL;
    }

    cleanup_config();
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
  AVRMEM         * sig;         /* signature data */
  struct stat      sb;
  UPDATE         * upd;
  LNODEID        * ln;


  /* options / operating mode variables */
  int     erase;       /* 1=erase chip, 0=don't */
  int     calibrate;   /* 1=calibrate RC oscillator, 0=don't */
  char  * port;        /* device port (/dev/xxx) */
  int     terminal;    /* 1=enter terminal mode, 0=don't */
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
  int     init_ok;     /* Device initialization worked well */
  int     is_open;     /* Device open succeeded */
  enum updateflags uflags = UF_AUTO_ERASE; /* Flags for do_op() */
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

  /*
   * Set line buffering for file descriptors so we see stdout and stderr
   * properly interleaved.
   */
  setvbuf(stdout, (char*)NULL, _IOLBF, 0);
  setvbuf(stderr, (char*)NULL, _IOLBF, 0);

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
  default_bitclock    = 0.0;

  init_config();

  atexit(cleanup_main);

  updates = lcreat(NULL, 0);
  if (updates == NULL) {
    fprintf(stderr, "%s: cannot initialize updater list\n", progname);
    exit(1);
  }

  extended_params = lcreat(NULL, 0);
  if (extended_params == NULL) {
    fprintf(stderr, "%s: cannot initialize extended parameter list\n", progname);
    exit(1);
  }

  additional_config_files = lcreat(NULL, 0);
  if (additional_config_files == NULL) {
    fprintf(stderr, "%s: cannot initialize additional config files list\n", progname);
    exit(1);
  }

  partdesc      = NULL;
  port          = NULL;
  erase         = 0;
  calibrate     = 0;
  p             = NULL;
  ovsigck       = 0;
  terminal      = 0;
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
  is_open       = 0;

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
  while ((ch = getopt(argc,argv,"?b:B:c:C:DeE:Fi:np:OP:qstU:uvVx:yY:")) != -1) {

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
        if (optarg[0] == '+') {
          ladd(additional_config_files, optarg+1);
        } else {
          strncpy(sys_config, optarg, PATH_MAX);
          sys_config[PATH_MAX-1] = 0;
        }
        break;

      case 'D': /* disable auto erase */
        uflags &= ~UF_AUTO_ERASE;
        break;

      case 'e': /* perform a chip erase */
        erase = 1;
        uflags &= ~UF_AUTO_ERASE;
        break;

      case 'E':
        exitspecs = optarg;
        break;

      case 'F': /* override invalid signature check */
        ovsigck = 1;
        break;

      case 'n':
        uflags |= UF_NOWRITE;
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

      case 'x':
        ladd(extended_params, optarg);
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
            "%sCopyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/\n"
	    "%sCopyright (c) 2007-2009 Joerg Wunsch\n\n",
            progname, version, __DATE__, __TIME__, progbuf, progbuf);
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

  if (lsize(additional_config_files) > 0) {
    LNODEID ln1;
    const char * p = NULL;

    for (ln1=lfirst(additional_config_files); ln1; ln1=lnext(ln1)) {
      p = ldata(ln1);
      if (verbose) {
        fprintf(stderr, "%sAdditional configuration file is \"%s\"\n",
                progbuf, p);
      }

      rc = read_config(p);
      if (rc) {
        fprintf(stderr,
                "%s: error reading additional configuration file \"%s\"\n",
                progname, p);
        exit(1);
      }
    }
  }

  // set bitclock from configuration files unless changed by command line
  if (default_bitclock > 0 && bitclock == 0.0) {
    bitclock = default_bitclock;
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
    if (strcmp(programmer, "?type") == 0) {
      fprintf(stderr, "\n");
      fprintf(stderr,"Valid programmer types are:\n");
      list_programmer_types(stderr, "  ");
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

  if (pgm->initpgm) {
    pgm->initpgm(pgm);
  } else {
    fprintf(stderr,
            "\n%s: Can't initialize the programmer.\n\n",
            progname);
    exit(1);
  }

  if (pgm->setup) {
    pgm->setup(pgm);
  }
  if (pgm->teardown) {
    atexit(exithook);
  }

  if (lsize(extended_params) > 0) {
    if (pgm->parseextparams == NULL) {
      fprintf(stderr,
              "%s: WARNING: Programmer doesn't support extended parameters,"
              " -x option(s) ignored\n",
              progname);
    } else {
      if (pgm->parseextparams(pgm, extended_params) < 0) {
        fprintf(stderr,
              "%s: Error parsing extended parameter list\n",
              progname);
        exit(1);
      }
    }
  }

  if (port == NULL) {
    switch (pgm->conntype)
    {
      case CONNTYPE_PARALLEL:
        port = default_parallel;
        break;

      case CONNTYPE_SERIAL:
        port = default_serial;
        break;

      case CONNTYPE_USB:
        port = DEFAULT_USB;
        break;
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

  if(p->flags & AVRPART_AVR32) {
    safemode = 0;
  }

  if(p->flags & (AVRPART_HAS_PDI | AVRPART_HAS_TPI)) {
    safemode = 0;
  }


  if (avr_initmem(p) != 0)
  {
    fprintf(stderr, "\n%s: failed to initialize memories\n",
            progname);
    exit(1);
  }

  /*
   * Now that we know which part we are going to program, locate any
   * -U options using the default memory region, and fill in the
   * device-dependent default region name, either "application" (for
   * Xmega devices), or "flash" (everything else).
   */
  for (ln=lfirst(updates); ln; ln=lnext(ln)) {
    upd = ldata(ln);
    if (upd->memtype == NULL) {
      const char *mtype = (p->flags & AVRPART_HAS_PDI)? "application": "flash";
      if (verbose >= 2) {
        fprintf(stderr,
                "%s: defaulting memtype in -U %c:%s option to \"%s\"\n",
                progname,
                (upd->op == DEVICE_READ)? 'r': (upd->op == DEVICE_WRITE)? 'w': 'v',
                upd->filename, mtype);
      }
      if ((upd->memtype = strdup(mtype)) == NULL) {
        fprintf(stderr, "%s: out of memory\n", progname);
        exit(1);
      }
    }
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
    fprintf(stderr, "%sUsing Port                    : %s\n", progbuf, port);
    fprintf(stderr, "%sUsing Programmer              : %s\n", progbuf, programmer);
    if ((strcmp(pgm->type, "avr910") == 0)) {
	  fprintf(stderr, "%savr910_devcode (avrdude.conf) : ", progbuf);
      if(p->avr910_devcode)fprintf(stderr, "0x%x\n", p->avr910_devcode);
	  else fprintf(stderr, "none\n");
    }  
  }

  if (baudrate != 0) {
    if (verbose) {
      fprintf(stderr, "%sOverriding Baud Rate          : %d\n", progbuf, baudrate);
    }
    pgm->baudrate = baudrate;
  }

  if (bitclock != 0.0) {
    if (verbose) {
      fprintf(stderr, "%sSetting bit clk period        : %.1f\n", progbuf, bitclock);
    }
    pgm->bitclock = bitclock * 1e-6;
  }

  if (ispdelay != 0) {
    if (verbose) {
      fprintf(stderr, "%sSetting isp clock delay        : %3i\n", progbuf, ispdelay);
    }
    pgm->ispdelay = ispdelay;
  }

  rc = pgm->open(pgm, port);
  if (rc < 0) {
    exitrc = 1;
    pgm->ppidata = 0; /* clear all bits at exit */
    goto main_exit;
  }
  is_open = 1;

  if (calibrate) {
    /*
     * perform an RC oscillator calibration
     * as outlined in appnote AVR053
     */
    if (pgm->perform_osccal == 0) {
      fprintf(stderr,
              "%s: programmer does not support RC oscillator calibration\n",
	      progname);
      exitrc = 1;
    } else {
      fprintf(stderr, "%s: performing RC oscillator calibration\n", progname);
      exitrc = pgm->perform_osccal(pgm);
    }
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
    programmer_display(pgm, progbuf);
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
  init_ok = (rc = pgm->initialize(pgm, p)) >= 0;
  if (!init_ok) {
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
  if(!(p->flags & AVRPART_AVR32)) {
    int attempt = 0;
    int waittime = 10000;       /* 10 ms */

  sig_again:
    usleep(waittime);
    if (init_ok) {
      rc = avr_signature(pgm, p);
      if (rc != 0) {
        fprintf(stderr, "%s: error reading signature data, rc=%d\n",
          progname, rc);
        exitrc = 1;
        goto main_exit;
      }
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
      if (ff || zz) {
        if (++attempt < 3) {
          waittime *= 5;
          if (quell_progress < 2) {
              fprintf(stderr, " (retrying)\n");
          }
          goto sig_again;
        }
        if (quell_progress < 2) {
            fprintf(stderr, "\n");
        }
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
      } else {
        if (quell_progress < 2) {
          fprintf(stderr, "\n");
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
  }

  if (init_ok && safemode == 1) {
    /* If safemode is enabled, go ahead and read the current low, high,
       and extended fuse bytes as needed */

    rc = safemode_readfuses(&safemode_lfuse, &safemode_hfuse,
                           &safemode_efuse, &safemode_fuse, pgm, p, verbose);

    if (rc != 0) {

	  //Check if the programmer just doesn't support reading
  	  if (rc == -5)
			{
			  if (verbose > 0)
				{
				fprintf(stderr, "%s: safemode: Fuse reading not support by programmer.\n"
                	            "              Safemode disabled.\n", progname);
				}
	  		safemode = 0;
			}
      else
			{

      		fprintf(stderr, "%s: safemode: To protect your AVR the programming "
            				    "will be aborted\n",
               					 progname);
      		exitrc = 1;
		    goto main_exit;
			}
    } else {
      //Save the fuses as default
      safemode_memfuses(1, &safemode_lfuse, &safemode_hfuse, &safemode_efuse, &safemode_fuse);
    }
  }

  if (uflags & UF_AUTO_ERASE) {
    if ((p->flags & AVRPART_HAS_PDI) && pgm->page_erase != NULL &&
        lsize(updates) > 0) {
      if (quell_progress < 2) {
        fprintf(stderr,
                "%s: NOTE: Programmer supports page erase for Xmega devices.\n"
                "%sEach page will be erased before programming it, but no chip erase is performed.\n"
                "%sTo disable page erases, specify the -D option; for a chip-erase, use the -e option.\n",
                progname, progbuf, progbuf);
      }
    } else {
      AVRMEM * m;
      const char *memname = (p->flags & AVRPART_HAS_PDI)? "application": "flash";
      for (ln=lfirst(updates); ln; ln=lnext(ln)) {
        upd = ldata(ln);
        m = avr_locate_mem(p, upd->memtype);
        if (m == NULL)
          continue;
        if ((strcasecmp(m->desc, memname) == 0) && (upd->op == DEVICE_WRITE)) {
          erase = 1;
          uflags &= ~UF_AUTO_ERASE;
          if (quell_progress < 2) {
            fprintf(stderr,
                    "%s: NOTE: \"%s\" memory has been specified, an erase cycle "
                    "will be performed\n"
                    "%sTo disable this feature, specify the -D option.\n",
                    progname, memname, progbuf);
          }
          break;
        }
      }
    }
  }

  /*
   * Display cycle count, if and only if it is not set later on.
   *
   * The cycle count will be displayed anytime it will be changed later.
   */
  if (init_ok && !(p->flags & AVRPART_AVR32) && do_cycles) {
    /*
     * see if the cycle count in the last four bytes of eeprom seems
     * reasonable
     */
    rc = avr_get_cycle_count(pgm, p, &cycles);
    if (quell_progress < 2) {
      if ((rc >= 0) && (cycles != 0)) {
        fprintf(stderr,
              "%s: current erase-rewrite cycle count is %d\n",
              progname, cycles);
      }
    }
  }

  if (init_ok && set_cycles != -1 && !(p->flags & AVRPART_AVR32)) {
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

  if (init_ok && erase) {
    /*
     * erase the chip's flash and eeprom memories, this is required
     * before the chip can accept new programming
     */
    if (uflags & UF_NOWRITE) {
      fprintf(stderr,
	      "%s: conflicting -e and -n options specified, NOT erasing chip\n",
	      progname);
    } else {
      if (quell_progress < 2) {
      	fprintf(stderr, "%s: erasing chip\n", progname);
      }
      exitrc = avr_chip_erase(pgm, p);
      if(exitrc) goto main_exit;
    }
  }

  if (terminal) {
    /*
     * terminal mode
     */
    exitrc = terminal_mode(pgm, p);
  }

  if (!init_ok) {
    /*
     * If we came here by the -tF options, bail out now.
     */
    exitrc = 1;
    goto main_exit;
  }


  for (ln=lfirst(updates); ln; ln=lnext(ln)) {
    upd = ldata(ln);
    rc = do_op(pgm, p, upd, uflags);
    if (rc) {
      exitrc = 1;
      break;
    }
  }

  /* Right before we exit programming mode, which will make the fuse
     bits active, check to make sure they are still correct */
  if (safemode == 1) {
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
       
       if (tolower((int)(safemode_response[0])) == 'y') {
              
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
       
       if (tolower((int)(safemode_response[0])) == 'y') {
              
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
       if (tolower((int)(safemode_response[0])) == 'y') {

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
       if (tolower((int)(safemode_response[0])) == 'y') {
              
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

  if (is_open) {
    pgm->powerdown(pgm);

    pgm->disable(pgm);

    pgm->rdy_led(pgm, OFF);
  }

  pgm->close(pgm);

  if (quell_progress < 2) {
    fprintf(stderr, "\n%s done.  Thank you.\n\n", progname);
  }

  return exitrc;
}
