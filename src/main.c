/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005  Brian S. Dean <bsd@bsdhome.com>
 * Copyright Joerg Wunsch <j@uriah.heep.sax.de>
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
#include <whereami.h>
#include <stdarg.h>
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

#include "avrdude.h"
#include "libavrdude.h"

#include "term.h"
#include "developer_opts.h"

/* Get VERSION from ac_cfg.h */
char * version      = VERSION;

char * progname;
char   progbuf[PATH_MAX]; /* temporary buffer of spaces the same
                             length as progname; used for lining up
                             multiline messages */

int avrdude_message(const int msglvl, const char *format, ...)
{
    int rc = 0;
    va_list ap;
    if (verbose >= msglvl) {
        va_start(ap, format);
        rc = vfprintf(stderr, format, ap);
        va_end(ap);
    }
    return rc;
}


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
int    verbose;     /* verbose output */
int    quell_progress; /* un-verebose output */
int    ovsigck;     /* 1=override sig check, 0=don't */




/*
 * usage message
 */
static void usage(void)
{
  avrdude_message(MSG_INFO,
 "Usage: %s [options]\n"
 "Options:\n"
 "  -p <partno>                Required. Specify AVR device.\n"
 "  -b <baudrate>              Override RS-232 baud rate.\n"
 "  -B <bitclock>              Specify JTAG/STK500v2 bit clock period (us).\n"
 "  -C <config-file>           Specify location of configuration file.\n"
 "  -c <programmer>            Specify programmer type.\n"
 "  -A                         Disable trailing-0xff removal from file and AVR read.\n"
 "  -D                         Disable auto erase for flash memory; implies -A.\n"
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
 "  -t                         Enter terminal mode.\n"
 "  -E <exitspec>[,<exitspec>] List programmer exit specifications.\n"
 "  -x <extended_param>        Pass <extended_param> to programmer.\n"
 "  -v                         Verbose output. -v -v for more.\n"
 "  -q                         Quell progress output. -q -q for less.\n"
 "  -l logfile                 Use logfile rather than stderr for diagnostics.\n"
 "  -?                         Display this usage.\n"
 "\navrdude version %s, URL: <https://github.com/avrdudes/avrdude>\n"
          ,progname, version);
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

static void replace_backslashes(char *s)
{
  // Replace all backslashes with forward slashes
  for (size_t i = 0; i < strlen(s); i++) {
    if (s[i] == '\\') {
      s[i] = '/';
    }
  }
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
  char  * exitspecs;   /* exit specs string from command line */
  char  * programmer;  /* programmer id */
  char  * partdesc;    /* part id */
  char    sys_config[PATH_MAX]; /* system wide config file */
  char    usr_config[PATH_MAX]; /* per-user config file */
  char    executable_abspath[PATH_MAX]; /* absolute path to avrdude executable */
  char    executable_dirpath[PATH_MAX]; /* absolute path to folder with executable */
  bool    executable_abspath_found = false; /* absolute path to executable found */
  bool    sys_config_found = false; /* 'avrdude.conf' file found */
  char  * e;           /* for strtol() error checking */
  int     baudrate;    /* override default programmer baud rate */
  double  bitclock;    /* Specify programmer bit clock (JTAG ICE) */
  int     ispdelay;    /* Specify the delay for ISP clock */
  int     init_ok;     /* Device initialization worked well */
  int     is_open;     /* Device open succeeded */
  char  * logfile;     /* Use logfile rather than stderr for diagnostics */
  enum updateflags uflags = UF_AUTO_ERASE | UF_VERIFY; /* Flags for do_op() */

#if !defined(WIN32)
  char  * homedir;
#endif

#ifdef _MSC_VER
  _set_printf_count_output(1);
#endif

  /*
   * Set line buffering for file descriptors so we see stdout and stderr
   * properly interleaved.
   */
  setvbuf(stdout, (char*)NULL, _IOLBF, 0);
  setvbuf(stderr, (char*)NULL, _IOLBF, 0);

  sys_config[0] = '\0';

  progname = strrchr(argv[0],'/');

#if defined (WIN32)
  /* take care of backslash as dir sep in W32 */
  if (!progname) progname = strrchr(argv[0],'\\');
#endif /* WIN32 */

  if (progname)
    progname++;
  else
    progname = argv[0];

  default_parallel[0] = 0;
  default_serial[0]   = 0;
  default_spi[0]      = 0;
  default_bitclock    = 0.0;

  init_config();

  atexit(cleanup_main);

  updates = lcreat(NULL, 0);
  if (updates == NULL) {
    avrdude_message(MSG_INFO, "%s: cannot initialize updater list\n", progname);
    exit(1);
  }

  extended_params = lcreat(NULL, 0);
  if (extended_params == NULL) {
    avrdude_message(MSG_INFO, "%s: cannot initialize extended parameter list\n", progname);
    exit(1);
  }

  additional_config_files = lcreat(NULL, 0);
  if (additional_config_files == NULL) {
    avrdude_message(MSG_INFO, "%s: cannot initialize additional config files list\n", progname);
    exit(1);
  }

  partdesc      = NULL;
  port          = NULL;
  erase         = 0;
  calibrate     = 0;
  p             = NULL;
  ovsigck       = 0;
  terminal      = 0;
  quell_progress = 0;
  exitspecs     = NULL;
  pgm           = NULL;
  programmer    = default_programmer;
  verbose       = 0;
  baudrate      = 0;
  bitclock      = 0.0;
  ispdelay      = 0;
  is_open       = 0;
  logfile       = NULL;

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
  while ((ch = getopt(argc,argv,"?Ab:B:c:C:DeE:Fi:l:np:OP:qstU:uvVx:yY:")) != -1) {

    switch (ch) {
      case 'b': /* override default programmer baud rate */
        baudrate = strtol(optarg, &e, 0);
        if ((e == optarg) || (*e != 0)) {
          avrdude_message(MSG_INFO, "%s: invalid baud rate specified '%s'\n",
                  progname, optarg);
          exit(1);
        }
        break;

      case 'B':	/* specify JTAG ICE bit clock period */
	bitclock = strtod(optarg, &e);
	if (*e != 0) {
	  /* trailing unit of measure present */
	  int suffixlen = strlen(e);
	  switch (suffixlen) {
	  case 2:
	    if ((e[0] != 'h' && e[0] != 'H') || e[1] != 'z')
	      bitclock = 0.0;
	    else
	      /* convert from Hz to microseconds */
	      bitclock = 1E6 / bitclock;
	    break;

	  case 3:
	    if ((e[1] != 'h' && e[1] != 'H') || e[2] != 'z')
	      bitclock = 0.0;
	    else {
	      switch (e[0]) {
	      case 'M':
	      case 'm':		/* no Millihertz here :) */
		bitclock = 1.0 / bitclock;
		break;

	      case 'k':
		bitclock = 1E3 / bitclock;
		break;

	      default:
		bitclock = 0.0;
		break;
	      }
	    }
	    break;

	  default:
	    bitclock = 0.0;
	    break;
	  }
	  if (bitclock == 0.0)
	    avrdude_message(MSG_INFO, "%s: invalid bit clock unit of measure '%s'\n",
			    progname, e);
	}
	if ((e == optarg) || bitclock == 0.0) {
	  avrdude_message(MSG_INFO, "%s: invalid bit clock period specified '%s'\n",
                  progname, optarg);
          exit(1);
        }
        break;

      case 'i':	/* specify isp clock delay */
	ispdelay = strtol(optarg, &e,10);
	if ((e == optarg) || (*e != 0) || ispdelay == 0) {
	  avrdude_message(MSG_INFO, "%s: invalid isp clock delay specified '%s'\n",
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
        /* fall through */

      case 'A': /* explicit disabling of trailing-0xff removal */
        disable_trailing_ff_removal();
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

      case 'l':
	logfile = optarg;
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

      case 't': /* enter terminal mode */
        terminal = 1;
        break;

      case 's':
      case 'u':
        avrdude_message(MSG_INFO, "%s: \"safemode\" feature no longer supported\n",
                progname);
        break;

      case 'U':
        upd = parse_op(optarg);
        if (upd == NULL) {
          avrdude_message(MSG_INFO, "%s: error parsing update operation '%s'\n",
                  progname, optarg);
          exit(1);
        }
        ladd(updates, upd);
        break;

      case 'v':
        verbose++;
        break;

      case 'V':
        uflags &= ~UF_VERIFY;
        break;

      case 'x':
        ladd(extended_params, optarg);
        break;

      case 'y':
        avrdude_message(MSG_INFO, "%s: erase cycle counter no longer supported\n",
                progname);
        break;

      case 'Y':
        avrdude_message(MSG_INFO, "%s: erase cycle counter no longer supported\n",
                progname);
        break;

      case '?': /* help */
        usage();
        exit(0);
        break;

      default:
        avrdude_message(MSG_INFO, "%s: invalid option -%c\n\n", progname, ch);
        usage();
        exit(1);
        break;
    }

  }

  if (logfile != NULL) {
    FILE *newstderr = freopen(logfile, "w", stderr);
    if (newstderr == NULL) {
      /* Help!  There's no stderr to complain to anymore now. */
      printf("Cannot create logfile \"%s\": %s\n",
	     logfile, strerror(errno));
      return 1;
    }
  }

  /* search for system configuration file unless -C conffile was given */
  if (strlen(sys_config) == 0) {
    /*
     * EXECUTABLE ABSPATH
     * ------------------
     * Determine the absolute path to avrdude executable. This will be used to
     * locate the 'avrdude.conf' file later.
     */
    int executable_dirpath_len;
    int executable_abspath_len = wai_getExecutablePath(
      executable_abspath,
      PATH_MAX,
      &executable_dirpath_len
    );
    if (
        (executable_abspath_len != -1) &&
        (executable_abspath_len != 0) &&
        (executable_dirpath_len != -1) &&
        (executable_dirpath_len != 0)
        ) {
      // All requirements satisfied, executable path was found
      executable_abspath_found = true;

      // Make sure the string is null terminated
      executable_abspath[executable_abspath_len] = '\0';

      replace_backslashes(executable_abspath);

      // Define 'executable_dirpath' to be the path to the parent folder of the
      // executable.
      strcpy(executable_dirpath, executable_abspath);
      executable_dirpath[executable_dirpath_len] = '\0';

      // Debug output
      avrdude_message(MSG_DEBUG, "executable_abspath = %s\n", executable_abspath);
      avrdude_message(MSG_DEBUG, "executable_abspath_len = %i\n", executable_abspath_len);
      avrdude_message(MSG_DEBUG, "executable_dirpath = %s\n", executable_dirpath);
      avrdude_message(MSG_DEBUG, "executable_dirpath_len = %i\n", executable_dirpath_len);
    }

    /*
     * SYSTEM CONFIG
     * -------------
     * Determine the location of 'avrdude.conf'. Check in this order:
     *  1. <dirpath of executable>/../etc/avrdude.conf
     *  2. <dirpath of executable>/avrdude.conf
     *  3. CONFIG_DIR/avrdude.conf
     *
     * When found, write the result into the 'sys_config' variable.
     */
    if (executable_abspath_found) {
      // 1. Check <dirpath of executable>/../etc/avrdude.conf
      strcpy(sys_config, executable_dirpath);
      sys_config[PATH_MAX - 1] = '\0';
      i = strlen(sys_config);
      if (i && (sys_config[i - 1] != '/'))
        strcat(sys_config, "/");
      strcat(sys_config, "../etc/" SYSTEM_CONF_FILE);
      sys_config[PATH_MAX - 1] = '\0';
      if (access(sys_config, F_OK) == 0) {
        sys_config_found = true;
      }
      else {
        // 2. Check <dirpath of executable>/avrdude.conf
        strcpy(sys_config, executable_dirpath);
        sys_config[PATH_MAX - 1] = '\0';
        i = strlen(sys_config);
        if (i && (sys_config[i - 1] != '/'))
          strcat(sys_config, "/");
        strcat(sys_config, SYSTEM_CONF_FILE);
        sys_config[PATH_MAX - 1] = '\0';
        if (access(sys_config, F_OK) == 0) {
          sys_config_found = true;
        }
      }
    }
    if (!sys_config_found) {
      // 3. Check CONFIG_DIR/avrdude.conf
#if defined(WIN32)
      win_sys_config_set(sys_config);
#else
      strcpy(sys_config, CONFIG_DIR);
      i = strlen(sys_config);
      if (i && (sys_config[i - 1] != '/'))
        strcat(sys_config, "/");
      strcat(sys_config, SYSTEM_CONF_FILE);
#endif
      if (access(sys_config, F_OK) == 0) {
        sys_config_found = true;
      }
    }
  }
  // Debug output
  avrdude_message(MSG_DEBUG, "sys_config = %s\n", sys_config);
  avrdude_message(MSG_DEBUG, "sys_config_found = %s\n", sys_config_found ? "true" : "false");
  avrdude_message(MSG_DEBUG, "\n");

  /*
   * USER CONFIG
   * -----------
   * Determine the location of '.avrduderc'.
   */
#if defined(WIN32)
  win_usr_config_set(usr_config);
#else
  usr_config[0] = 0;
  homedir = getenv("HOME");
  if (homedir != NULL) {
    strcpy(usr_config, homedir);
    i = strlen(usr_config);
    if (i && (usr_config[i - 1] != '/'))
      strcat(usr_config, "/");
    strcat(usr_config, USER_CONF_FILE);
  }
#endif

  if (quell_progress == 0)
    terminal_setup_update_progress();

  /*
   * Print out an identifying string so folks can tell what version
   * they are running
   */
  avrdude_message(MSG_NOTICE, "\n%s: Version %s\n"
                    "%sCopyright (c) Brian Dean, http://www.bdmicro.com/\n"
                    "%sCopyright (c) Joerg Wunsch\n\n",
                    progname, version, progbuf, progbuf);
  avrdude_message(MSG_NOTICE, "%sSystem wide configuration file is \"%s\"\n",
            progbuf, sys_config);

  rc = read_config(sys_config);
  if (rc) {
    avrdude_message(MSG_INFO, "%s: error reading system wide configuration file \"%s\"\n",
                    progname, sys_config);
    exit(1);
  }

  if (usr_config[0] != 0) {
    avrdude_message(MSG_NOTICE, "%sUser configuration file is \"%s\"\n",
              progbuf, usr_config);

    rc = stat(usr_config, &sb);
    if ((rc < 0) || ((sb.st_mode & S_IFREG) == 0)) {
      avrdude_message(MSG_NOTICE, "%sUser configuration file does not exist or is not a "
                      "regular file, skipping\n",
                      progbuf);
    }
    else {
      rc = read_config(usr_config);
      if (rc) {
        avrdude_message(MSG_INFO, "%s: error reading user configuration file \"%s\"\n",
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
      avrdude_message(MSG_NOTICE, "%sAdditional configuration file is \"%s\"\n",
                      progbuf, p);

      rc = read_config(p);
      if (rc) {
        avrdude_message(MSG_INFO, "%s: error reading additional configuration file \"%s\"\n",
                        progname, p);
        exit(1);
      }
    }
  }

  // set bitclock from configuration files unless changed by command line
  if (default_bitclock > 0 && bitclock == 0.0) {
    bitclock = default_bitclock;
  }


  int dev_opts = 0;
  // Developer option -c <wildcard>/[ASsrt] prints programmer description(s) and exits
  if(programmer && (strcmp(programmer, "*") == 0 || strchr(programmer, '/'))) {
    dev_output_pgm_defs(programmer);
    dev_opts = 1;
  }
  // Developer option -p <wildcard>/[dASsrcow*t] prints part description(s) and exits
  if(partdesc && (strcmp(partdesc, "*") == 0 || strchr(partdesc, '/'))) {
    dev_output_part_defs(partdesc);
    dev_opts = 1;
  }
  if(dev_opts)
    exit(0);

  avrdude_message(MSG_NOTICE, "\n");

  if (partdesc) {
    if (strcmp(partdesc, "?") == 0) {
      avrdude_message(MSG_INFO, "\n");
      avrdude_message(MSG_INFO, "Valid parts are:\n");
      list_parts(stderr, "  ", part_list);
      avrdude_message(MSG_INFO, "\n");
      exit(1);
    }
  }

  if (programmer) {
    if (strcmp(programmer, "?") == 0) {
      avrdude_message(MSG_INFO, "\n");
      avrdude_message(MSG_INFO, "Valid programmers are:\n");
      list_programmers(stderr, "  ", programmers);
      avrdude_message(MSG_INFO, "\n");
      exit(1);
    }
    if (strcmp(programmer, "?type") == 0) {
      avrdude_message(MSG_INFO, "\n");
      avrdude_message(MSG_INFO, "Valid programmer types are:\n");
      list_programmer_types(stderr, "  ");
      avrdude_message(MSG_INFO, "\n");
      exit(1);
    }
  }


  if (programmer[0] == 0) {
    avrdude_message(MSG_INFO, "\n%s: no programmer has been specified on the command line "
                    "or the config file\n",
                    progname);
    avrdude_message(MSG_INFO, "%sSpecify a programmer using the -c option and try again\n\n",
                    progbuf);
    exit(1);
  }

  pgm = locate_programmer(programmers, programmer);
  if (pgm == NULL) {
    avrdude_message(MSG_INFO, "\n");
    avrdude_message(MSG_INFO, "%s: Can't find programmer id \"%s\"\n",
                    progname, programmer);
    avrdude_message(MSG_INFO, "\nValid programmers are:\n");
    list_programmers(stderr, "  ", programmers);
    avrdude_message(MSG_INFO, "\n");
    exit(1);
  }

  if (pgm->initpgm) {
    pgm->initpgm(pgm);
  } else {
    avrdude_message(MSG_INFO, "\n%s: Can't initialize the programmer.\n\n",
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
      avrdude_message(MSG_INFO, "%s: WARNING: Programmer doesn't support extended parameters,"
                      " -x option(s) ignored\n",
                      progname);
    } else {
      if (pgm->parseextparams(pgm, extended_params) < 0) {
        avrdude_message(MSG_INFO, "%s: Error parsing extended parameter list\n",
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

      case CONNTYPE_SPI:
#ifdef HAVE_LINUXSPI
        port = *default_spi? default_spi: "unknown";
#endif
        break;
    }
  }

  if (partdesc == NULL) {
    avrdude_message(MSG_INFO, "%s: No AVR part has been specified, use \"-p Part\"\n\n",
                    progname);
    avrdude_message(MSG_INFO, "Valid parts are:\n");
    list_parts(stderr, "  ", part_list);
    avrdude_message(MSG_INFO, "\n");
    exit(1);
  }


  p = locate_part(part_list, partdesc);
  if (p == NULL) {
    avrdude_message(MSG_INFO, "%s: AVR Part \"%s\" not found.\n\n",
                    progname, partdesc);
    avrdude_message(MSG_INFO, "Valid parts are:\n");
    list_parts(stderr, "  ", part_list);
    avrdude_message(MSG_INFO, "\n");
    exit(1);
  }


  if (exitspecs != NULL) {
    if (pgm->parseexitspecs == NULL) {
      avrdude_message(MSG_INFO, "%s: WARNING: -E option not supported by this programmer type\n",
                      progname);
      exitspecs = NULL;
    }
    else if (pgm->parseexitspecs(pgm, exitspecs) < 0) {
      usage();
      exit(1);
    }
  }

  if (avr_initmem(p) != 0)
  {
    avrdude_message(MSG_INFO, "\n%s: failed to initialize memories\n",
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
      avrdude_message(MSG_NOTICE2, "%s: defaulting memtype in -U %c:%s option to \"%s\"\n",
                      progname,
                      (upd->op == DEVICE_READ)? 'r': (upd->op == DEVICE_WRITE)? 'w': 'v',
                      upd->filename, mtype);
      upd->memtype = cfg_strdup("main()", mtype);
    }

    if (!avr_mem_might_be_known(upd->memtype)) {
      avrdude_message(MSG_INFO, "%s: unknown memory type %s\n", progname, upd->memtype);
      exit(1);
    }
    // TODO: check whether filename other than "-" is readable/writable
  }

  /*
   * open the programmer
   */
  if (port[0] == 0) {
    avrdude_message(MSG_INFO, "\n%s: no port has been specified on the command line "
            "or in the config file\n",
            progname);
    avrdude_message(MSG_INFO, "%sSpecify a port using the -P option and try again\n\n",
            progbuf);
    exit(1);
  }

  if (verbose) {
    avrdude_message(MSG_NOTICE, "%sUsing Port                    : %s\n", progbuf, port);
    avrdude_message(MSG_NOTICE, "%sUsing Programmer              : %s\n", progbuf, programmer);
    if ((strcmp(pgm->type, "avr910") == 0)) {
	  avrdude_message(MSG_NOTICE, "%savr910_devcode (avrdude.conf) : ", progbuf);
      if(p->avr910_devcode)avrdude_message(MSG_INFO, "0x%x\n", p->avr910_devcode);
	  else avrdude_message(MSG_NOTICE, "none\n");
    }
  }

  if (baudrate != 0) {
    avrdude_message(MSG_NOTICE, "%sOverriding Baud Rate          : %d\n", progbuf, baudrate);
    pgm->baudrate = baudrate;
  }

  if (bitclock != 0.0) {
    avrdude_message(MSG_NOTICE, "%sSetting bit clk period        : %.1f\n", progbuf, bitclock);
    pgm->bitclock = bitclock * 1e-6;
  }

  if (ispdelay != 0) {
    avrdude_message(MSG_NOTICE, "%sSetting isp clock delay        : %3i\n", progbuf, ispdelay);
    pgm->ispdelay = ispdelay;
  }

  rc = pgm->open(pgm, port);
  if (rc < 0) {
    avrdude_message(MSG_INFO,
                    "%s: opening programmer \"%s\" on port \"%s\" failed\n",
                    progname, programmer, port);
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
      avrdude_message(MSG_INFO, "%s: programmer does not support RC oscillator calibration\n",
                      progname);
      exitrc = 1;
    } else {
      avrdude_message(MSG_INFO, "%s: performing RC oscillator calibration\n", progname);
      exitrc = pgm->perform_osccal(pgm);
    }
    if (exitrc == 0 && quell_progress < 2) {
      avrdude_message(MSG_INFO, "%s: calibration value is now stored in EEPROM at address 0\n",
                      progname);
    }
    goto main_exit;
  }

  if (verbose) {
    avr_display(stderr, p, progbuf, verbose);
    avrdude_message(MSG_NOTICE, "\n");
    programmer_display(pgm, progbuf);
  }

  if (quell_progress < 2) {
    avrdude_message(MSG_INFO, "\n");
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
   * initialize the chip in preparation for accepting commands
   */
  init_ok = (rc = pgm->initialize(pgm, p)) >= 0;
  if (!init_ok) {
    avrdude_message(MSG_INFO, "%s: initialization failed, rc=%d\n", progname, rc);
    if (!ovsigck) {
      avrdude_message(MSG_INFO, "%sDouble check connections and try again, "
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
    avrdude_message(MSG_INFO, "%s: AVR device initialized and ready to accept instructions\n",
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
      if (rc != LIBAVRDUDE_SUCCESS) {
        if ((rc == LIBAVRDUDE_SOFTFAIL) && (p->flags & AVRPART_HAS_UPDI) && (attempt < 1)) {
          attempt++;
          if (pgm->read_sib) {
             // Read SIB and compare FamilyID
             char sib[AVR_SIBLEN + 1];
             pgm->read_sib(pgm, p, sib);
             avrdude_message(MSG_NOTICE, "%s: System Information Block: \"%s\"\n",
                             progname, sib);
            if (quell_progress < 2) {
              avrdude_message(MSG_INFO, "%s: Received FamilyID: \"%.*s\"\n", progname, AVR_FAMILYIDLEN, sib);
            }
            if (strncmp(p->family_id, sib, AVR_FAMILYIDLEN)) {
              avrdude_message(MSG_INFO, "%s: Expected FamilyID: \"%s\"\n", progname, p->family_id);
              if (!ovsigck) {
                avrdude_message(MSG_INFO, "%sDouble check chip, "
                        "or use -F to override this check.\n",
                        progbuf);
                exitrc = 1;
                goto main_exit;
              }
            }
          }
          if(erase) {
            erase = 0;
            if (uflags & UF_NOWRITE) {
              avrdude_message(MSG_INFO, "%s: conflicting -e and -n options specified, NOT erasing chip\n",
                              progname);
            } else {
              if (quell_progress < 2) {
                avrdude_message(MSG_INFO, "%s: erasing chip\n", progname);
              }
              exitrc = avr_unlock(pgm, p);
              if(exitrc) goto main_exit;
              goto sig_again;
            }
          }
        }
        avrdude_message(MSG_INFO, "%s: error reading signature data, rc=%d\n",
          progname, rc);
        exitrc = 1;
        goto main_exit;
      }
    }

    sig = avr_locate_mem(p, "signature");
    if (sig == NULL) {
      avrdude_message(MSG_INFO, "%s: WARNING: signature data not defined for device \"%s\"\n",
                      progname, p->desc);
    }

    if (sig != NULL) {
      int ff, zz;

      if (quell_progress < 2) {
        avrdude_message(MSG_INFO, "%s: Device signature = 0x", progname);
      }
      ff = zz = 1;
      for (i=0; i<sig->size; i++) {
        if (quell_progress < 2) {
          avrdude_message(MSG_INFO, "%02x", sig->buf[i]);
        }
        if (sig->buf[i] != 0xff)
          ff = 0;
        if (sig->buf[i] != 0x00)
          zz = 0;
      }

      bool signature_matches =
          sig->size == 3 &&
          sig->buf[0] == p->signature[0] &&
          sig->buf[1] == p->signature[1] &&
          sig->buf[2] == p->signature[2];

      if (quell_progress < 2) {
        AVRPART * part;

        part = locate_part_by_signature(part_list, sig->buf, sig->size);
        if (part) {
          avrdude_message(MSG_INFO, " (probably %s)", signature_matches ? p->id : part->id);
        }
      }
      if (ff || zz) {
        if (++attempt < 3) {
          waittime *= 5;
          if (quell_progress < 2) {
              avrdude_message(MSG_INFO, " (retrying)\n");
          }
          goto sig_again;
        }
        if (quell_progress < 2) {
            avrdude_message(MSG_INFO, "\n");
        }
        avrdude_message(MSG_INFO, "%s: Yikes!  Invalid device signature.\n", progname);
        if (!ovsigck) {
          avrdude_message(MSG_INFO, "%sDouble check connections and try again, "
                  "or use -F to override\n"
                  "%sthis check.\n\n",
                  progbuf, progbuf);
          exitrc = 1;
          goto main_exit;
        }
      } else {
        if (quell_progress < 2) {
          avrdude_message(MSG_INFO, "\n");
        }
      }

      if (!signature_matches) {
        avrdude_message(MSG_INFO, "%s: Expected signature for %s is %02X %02X %02X\n",
                        progname, p->desc,
                        p->signature[0], p->signature[1], p->signature[2]);
        if (!ovsigck) {
          avrdude_message(MSG_INFO, "%sDouble check chip, "
                  "or use -F to override this check.\n",
                  progbuf);
          exitrc = 1;
          goto main_exit;
        }
      }
    }
  }

  if (uflags & UF_AUTO_ERASE) {
    if ((p->flags & AVRPART_HAS_PDI) && pgm->page_erase != NULL &&
        lsize(updates) > 0) {
      if (quell_progress < 2) {
        avrdude_message(MSG_INFO, "%s: NOTE: Programmer supports page erase for Xmega devices.\n"
                        "%sEach page will be erased before programming it, but no chip erase is performed.\n"
                        "%sTo disable page erases, specify the -D option; for a chip-erase, use the -e option.\n",
                        progname, progbuf, progbuf);
      }
    } else {
      AVRMEM * m;
      const char *memname = (p->flags & AVRPART_HAS_PDI)? "application": "flash";

      uflags &= ~UF_AUTO_ERASE;
      for (ln=lfirst(updates); ln; ln=lnext(ln)) {
        upd = ldata(ln);
        m = avr_locate_mem(p, upd->memtype);
        if (m == NULL)
          continue;
        if ((strcasecmp(m->desc, memname) == 0) && (upd->op == DEVICE_WRITE)) {
          erase = 1;
          if (quell_progress < 2) {
            avrdude_message(MSG_INFO, "%s: NOTE: \"%s\" memory has been specified, an erase cycle "
                            "will be performed\n"
                            "%sTo disable this feature, specify the -D option.\n",
                            progname, memname, progbuf);
          }
          break;
        }
      }
    }
  }

  if (init_ok && erase) {
    /*
     * erase the chip's flash and eeprom memories, this is required
     * before the chip can accept new programming
     */
    if (uflags & UF_NOWRITE) {
      avrdude_message(MSG_INFO, "%s: conflicting -e and -n options specified, NOT erasing chip\n",
                      progname);
    } else {
      if (quell_progress < 2) {
      	avrdude_message(MSG_INFO, "%s: erasing chip\n", progname);
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
    if (rc && rc != LIBAVRDUDE_SOFTFAIL) {
      exitrc = 1;
      break;
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

    pgm->close(pgm);
  }

  if (quell_progress < 2) {
    avrdude_message(MSG_INFO, "\n%s done.  Thank you.\n\n", progname);
  }

  return exitrc;
}
