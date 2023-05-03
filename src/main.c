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
#include "config.h"
#include "term.h"
#include "developer_opts.h"

/* Get VERSION from ac_cfg.h */
char * version      = VERSION;

char * progname;
char   progbuf[PATH_MAX]; /* temporary buffer of spaces the same
                             length as progname; used for lining up
                             multiline messages */

// Old (deprecated) message routine
int avrdude_message(int msglvl, const char *format, ...)
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

static const char *avrdude_message_type(int msglvl) {
  switch(msglvl) {
  case MSG_EXT_ERROR: return "OS error";
  case MSG_ERROR: return "error";
  case MSG_WARNING: return "warning";
  case MSG_INFO: return "info";
  case MSG_NOTICE: return "notice";
  case MSG_NOTICE2: return "notice2";
  case MSG_DEBUG: return "debug";
  case MSG_TRACE: return "trace";
  case MSG_TRACE2: return "trace2";
  default: return "unknown msglvl";
  }
}


/*
 * Core msg_xyz() routine
 * See #define lines in avrdude.h of how it is normally called
 * Side note: if format starts with \v print \n but only if *not* at beginning of line
 */
int avrdude_message2(FILE *fp, int lno, const char *file, const char *func, int msgmode, int msglvl, const char *format, ...) {
    int rc = 0;
    va_list ap;

    static struct {             // Memorise whether last print ended at beginning of line
      FILE *fp;
      int bol;                  // Are we at the beginning of a line for this fp stream?
    } bols[5+1];                // Cater for up to 5 different FILE pointers plus one catch-all

    size_t bi = 0;              // bi is index to bols[] array
    for(bi=0; bi < sizeof bols/sizeof*bols -1; bi++) { // Note the -1, so bi is valid after loop
      if(!bols[bi].fp) {        // First free space
        bols[bi].fp = fp;       // Insert fp in first free space
        bols[bi].bol = 1;       // Assume beginning of line on first use
      }
      if(bols[bi].fp == fp)
        break;
    }

    if(msglvl <= MSG_ERROR)     // Serious error? Free progress bars (if any)
      report_progress(1, -1, NULL);

    if(msgmode & MSG2_FLUSH) {
        fflush(stdout);
        fflush(stderr);
    }

    // Reduce effective verbosity level by number of -q above one when printing to stderr
    if ((quell_progress < 2 || fp != stderr? verbose: verbose+1-quell_progress) >= msglvl) {
        if(msgmode & MSG2_PROGNAME) {
          if(!bols[bi].bol)
            fprintf(fp, "\n");
          fprintf(fp, "%s", progname);
          if(verbose >= MSG_NOTICE && (msgmode & MSG2_FUNCTION))
            fprintf(fp, " %s()", func);
          if(verbose >= MSG_DEBUG && (msgmode & MSG2_FILELINE)) {
            const char *pr = strrchr(file, '/'); // Only print basename
#if defined (WIN32)
            if(!pr)
              pr =  strrchr(file, '\\');
#endif
            pr = pr? pr+1: file;
            fprintf(fp, " [%s:%d]", pr, lno);
          }
          if(msgmode & MSG2_TYPE)
            fprintf(fp, " %s", avrdude_message_type(msglvl));
          fprintf(fp, ": ");
          bols[bi].bol = 0;
        } else if(msgmode & MSG2_INDENT1) {
          fprintf(fp, "%*s", (int) strlen(progname)+1, "");
          bols[bi].bol = 0;
        } else if(msgmode & MSG2_INDENT2) {
          fprintf(fp, "%*s", (int) strlen(progname)+2, "");
          bols[bi].bol = 0;
        }

        // Vertical tab at start of format string is a conditional new line
        if(*format == '\v') {
          format++;
          if(!bols[bi].bol) {
            fprintf(fp, "\n");
            bols[bi].bol = 1;
          }
        }

        // Figure out whether this print will leave us at beginning of line

        // Determine required size first
        va_start(ap, format);
        rc = vsnprintf(NULL, 0, format, ap);
        va_end(ap);

        if(rc < 0)              // Some errror?
          return 0;

        rc++;                   // Accommodate terminating nul
        char *p = cfg_malloc(__func__, rc);
        va_start(ap, format);
        rc = vsnprintf(p, rc, format, ap);
        va_end(ap);

        if(rc < 0) {
          free(p);
          return 0;
        }

        if(*p) {
          fprintf(fp, "%s", p); // Finally: print!
          bols[bi].bol = p[strlen(p)-1] == '\n';
        }
        free(p);
    }

    if(msgmode & MSG2_FLUSH)
        fflush(fp);

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
int verbose;                    // Verbose output
int quell_progress;             // Quell progress report and un-verbose output
int ovsigck;                    // 1 = override sig check, 0 = don't
const char *partdesc;           // Part id


/*
 * usage message
 */
static void usage(void)
{
  msg_error(
    "Usage: %s [options]\n"
    "Options:\n"
    "  -p <partno>                Specify AVR device\n"
    "  -p <wildcard>/<flags>      Run developer options for matched AVR devices\n"
    "  -b <baudrate>              Override RS-232 baud rate\n"
    "  -B <bitclock>              Specify bit clock period (us)\n"
    "  -C <config-file>           Specify location of configuration file\n"
    "  -c <programmer>            Specify programmer type\n"
    "  -c <wildcard>/<flags>      Run developer options for matched programmers\n"
    "  -A                         Disable trailing-0xff removal from file and AVR read\n"
    "  -D                         Disable auto erase for flash memory; implies -A\n"
    "  -i <delay>                 ISP Clock Delay [in microseconds]\n"
    "  -P <port>                  Specify connection port\n"
    "  -F                         Override invalid signature or initialisation check\n"
    "  -e                         Perform a chip erase\n"
    "  -O                         Perform RC oscillator calibration (see AVR053)\n"
    "  -U <memtype>:r|w|v:<filename>[:format]\n"
    "                             Memory operation specification\n"
    "                             Multiple -U options are allowed, each request\n"
    "                             is performed in the order specified\n"
    "  -n                         Do not write to the device whilst processing -U\n"
    "  -V                         Do not automatically verify during -U\n"
    "  -t                         Enter terminal mode\n"
    "  -E <exitspec>[,<exitspec>] List programmer exit specifications\n"
    "  -x <extended_param>        Pass <extended_param> to programmer, see -xhelp\n"
    "  -v                         Verbose output; -v -v for more\n"
    "  -q                         Quell progress output; -q -q for less\n"
    "  -l logfile                 Use logfile rather than stderr for diagnostics\n"
    "  -?                         Display this usage\n"
    "\navrdude version %s, URL: <https://github.com/avrdudes/avrdude>\n",
    progname, version);
}


// Potentially shorten copy of prog description if it's the suggested mode
static void pmshorten(char *desc, const char *modes) {
  struct { const char *end, *mode; } pairs[] = {
    {" in parallel programming mode", "HVPP"},
    {" in PP mode", "HVPP"},
    {" in high-voltage serial programming mode", "HVSP"},
    {" in HVSP mode", "HVSP"},
    {" in ISP mode", "ISP"},
    {" in debugWire mode", "debugWIRE"},
    {" in AVR32 mode", "aWire"},
    {" in PDI mode", "PDI"},
    {" in UPDI mode", "UPDI"},
    {" in JTAG mode", "JTAG"},
    {" in JTAG mode", "JTAGmkI"},
    {" in JTAG mode", "XMEGAJTAG"},
    {" in JTAG mode", "AVR32JTAG"},
    {" for bootloader", "bootloader"},
  };
  size_t len = strlen(desc);

  for(size_t i=0; i<sizeof pairs/sizeof*pairs; i++) {
    size_t elen = strlen(pairs[i].end);
    if(len > elen && strcasecmp(desc+len-elen, pairs[i].end) == 0 && strcmp(modes, pairs[i].mode) == 0) {
      desc[len-elen] = 0;
      break;
    }
  }
}

static void list_programmers(FILE *f, const char *prefix, LISTID programmers, int pm) {
  LNODEID ln1;
  LNODEID ln2;
  PROGRAMMER *pgm;
  int maxlen=0, len;

  sort_programmers(programmers);

  // Compute max length of programmer names
  for(ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    pgm = ldata(ln1);
    for(ln2=lfirst(pgm->id); ln2; ln2=lnext(ln2))
      if(!pm || !pgm->prog_modes || (pm & pgm->prog_modes)) {
        const char *id = ldata(ln2);
        if(*id == 0 || *id == '.')
          continue;
        if((len = strlen(id)) > maxlen)
          maxlen = len;
      }
  }

  for(ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    pgm = ldata(ln1);
    for(ln2=lfirst(pgm->id); ln2; ln2=lnext(ln2)) {
      // List programmer if pm or prog_modes uninitialised or if they are compatible otherwise
      if(!pm || !pgm->prog_modes || (pm & pgm->prog_modes)) {
        const char *id = ldata(ln2);
        char *desc = cfg_strdup("list_programmers()", pgm->desc);
        const char *modes = avr_prog_modes(pm & pgm->prog_modes);

        if(pm != ~0)
          pmshorten(desc, modes);

        if(*id == 0 || *id == '.')
          continue;
        if(verbose)
          fprintf(f, "%s%-*s = %-30s [%s:%d]", prefix, maxlen, id, desc, pgm->config_file, pgm->lineno);
        else
          fprintf(f, "%s%-*s = %-s", prefix, maxlen, id, desc);
        if(pm != ~0)
          fprintf(f, " via %s",  modes);
        fprintf(f, "\n");

        free(desc);
      }
    }
  }
}

static void list_programmer_types_callback(const char *name, const char *desc,
                                      void *cookie)
{
    struct list_walk_cookie *c = (struct list_walk_cookie *)cookie;
    fprintf(c->f, "%s%-16s = %-s\n", c->prefix, name, desc);
}

static void list_programmer_types(FILE * f, const char *prefix)
{
    struct list_walk_cookie c;

    c.f = f;
    c.prefix = prefix;

    walk_programmer_types(list_programmer_types_callback, &c);
}


static void list_parts(FILE *f, const char *prefix, LISTID avrparts, int pm) {
  LNODEID ln1;
  AVRPART *p;
  int maxlen=0, len;

  sort_avrparts(avrparts);

  // Compute max length of part names
  for(ln1 = lfirst(avrparts); ln1; ln1 = lnext(ln1)) {
    p = ldata(ln1);
    // List part if pm or prog_modes uninitialised or if they are compatible otherwise
    if(!pm || !p->prog_modes || (pm & p->prog_modes)) {
      if(verbose < 2 && p->id[0] == '.') // hide ids starting with '.'
        continue;
      if((len = strlen(p->id)) > maxlen)
        maxlen = len;
    }
  }

  for(ln1 = lfirst(avrparts); ln1; ln1 = lnext(ln1)) {
    p = ldata(ln1);
    // List part if pm or prog_modes uninitialised or if they are compatible otherwise
    if(!pm || !p->prog_modes || (pm & p->prog_modes)) {
      if(verbose < 2 && p->id[0] == '.') // hide ids starting with '.'
        continue;
      if(verbose)
        fprintf(f, "%s%-*s = %-18s [%s:%d]", prefix, maxlen, p->id, p->desc, p->config_file, p->lineno);
      else
        fprintf(f, "%s%-*s = %s", prefix, maxlen, p->id, p->desc);
      if(pm != ~0)
        fprintf(f, " via %s", avr_prog_modes(pm & p->prog_modes));
      fprintf(f, "\n");
      if(verbose)
        for(LNODEID ln = lfirst(p->variants); ln; ln = lnext(ln))
          fprintf(f, "%s%s- %s\n", prefix, prefix, (char *) ldata(ln));
    }
  }
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

// Return 2 if string is * or starts with */, 1 if string contains /, 0 otherwise
static int dev_opt(const char *str) {
  return
    !str? 0:
    !strcmp(str, "*") || !strncmp(str, "*/", 2)? 2:
    !!strchr(str, '/');
}


static void programmer_not_found(const char *programmer) {
  msg_error("\n");
  if(programmer && *programmer)
    pmsg_error("cannot find programmer id %s\n", programmer);
  else {
    pmsg_error("no programmer has been specified on the command line or in the\n");
    imsg_error("config file(s); specify one using the -c option and try again\n");
  }

  msg_error("\nValid programmers are:\n");
  list_programmers(stderr, "  ", programmers, ~0);
  msg_error("\n");
}

static void part_not_found(const char *partdesc) {
  msg_error("\n");
  if(partdesc && *partdesc)
    pmsg_error("AVR part %s not found\n", partdesc);
  else
    pmsg_error("no AVR part has been specified; use -p part\n");

  msg_error("\nValid parts are:\n");
  list_parts(stderr, "  ", part_list, ~0);
  msg_error("\n");
}


#if !defined(WIN32)
// Safely concatenate dir/file into dst that has size n
static char *concatpath(char *dst, char *dir, char *file, size_t n) {
  // Dir or file empty?
  if(!dir || !*dir || !file || !*file)
    return NULL;

  size_t len = strlen(dir);

  // Insufficient space?
  if(len + (dir[len-1] != '/') + strlen(file) > n-1)
    return NULL;

  if(dst != dir)
    strcpy(dst, dir);

  if(dst[len-1] != '/')
    strcat(dst, "/");

  strcat(dst, file);

  return dst;
}
#endif


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
  const char *exitspecs; /* exit specs string from command line */
  const char *programmer; /* programmer id */
  int     explicit_c;  /* 1=explicit -c on command line, 0=not spcified  there */
  int     explicit_e;  /* 1=explicit -e on command line, 0=not spcified  there */
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
  int     ce_delayed;  /* Chip erase delayed */
  char  * logfile;     /* Use logfile rather than stderr for diagnostics */
  enum updateflags uflags = UF_AUTO_ERASE | UF_VERIFY; /* Flags for do_op() */

  (void) avr_ustimestamp();

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

  progname = strrchr(argv[0], '/');

#if defined (WIN32)
  /* take care of backslash as dir sep in W32 */
  if (!progname)
    progname = strrchr(argv[0], '\\');
#endif /* WIN32 */

  if (progname)
    progname++;
  else
    progname = argv[0];

  // Remove trailing .exe
  if(strlen(progname) > 4 && strcmp(progname+strlen(progname)-4, ".exe") == 0) {
    progname = cfg_strdup("main()", progname); // Don't write to argv[0]
    progname[strlen(progname)-4] = 0;
  }

  default_programmer = "";
  default_parallel   = "";
  default_serial     = "";
  default_spi        = "";
  default_bitclock   = 0.0;
  default_linuxgpio  = "";

  init_config();

  atexit(cleanup_main);

  updates = lcreat(NULL, 0);
  if (updates == NULL) {
    pmsg_error("cannot initialize updater list\n");
    exit(1);
  }

  extended_params = lcreat(NULL, 0);
  if (extended_params == NULL) {
    pmsg_error("cannot initialize extended parameter list\n");
    exit(1);
  }

  additional_config_files = lcreat(NULL, 0);
  if (additional_config_files == NULL) {
    pmsg_error("cannot initialize additional config files list\n");
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
  programmer    = "";
  explicit_c    = 0;
  explicit_e    = 0;
  verbose       = 0;
  baudrate      = 0;
  bitclock      = 0.0;
  ispdelay      = 0;
  is_open       = 0;
  ce_delayed    = 0;
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
          pmsg_error("invalid baud rate specified '%s'\n", optarg);
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
	    pmsg_error("invalid bit clock unit of measure '%s'\n", e);
	}
	if ((e == optarg) || bitclock == 0.0) {
	  pmsg_error("invalid bit clock period specified '%s'\n", optarg);
          exit(1);
        }
        break;

      case 'i':	/* specify isp clock delay */
	ispdelay = strtol(optarg, &e,10);
	if ((e == optarg) || (*e != 0) || ispdelay == 0) {
	  pmsg_error("invalid isp clock delay specified '%s'\n", optarg);
          exit(1);
        }
        break;

      case 'c': /* programmer id */
        programmer = optarg;
        explicit_c = 1;
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
        explicit_e = 1;
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
        pmsg_error("\"safemode\" feature no longer supported\n");
        break;

      case 'U':
        upd = parse_op(optarg);
        if (upd == NULL) {
          pmsg_error("unable to parse update operation '%s'\n", optarg);
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
        pmsg_error("erase cycle counter no longer supported\n");
        break;

      case 'Y':
        pmsg_error("erase cycle counter no longer supported\n");
        break;

      case '?': /* help */
        usage();
        exit(0);
        break;

      default:
        pmsg_error("invalid option -%c\n\n", ch);
        usage();
        exit(1);
        break;
    }

  }

  if (logfile != NULL) {
    FILE *newstderr = freopen(logfile, "w", stderr);
    if (newstderr == NULL) {
      /* Help!  There's no stderr to complain to anymore now. */
      printf("Cannot create logfile %s: %s\n", logfile, strerror(errno));
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
      msg_trace2("executable_abspath = %s\n", executable_abspath);
      msg_trace2("executable_abspath_len = %i\n", executable_abspath_len);
      msg_trace2("executable_dirpath = %s\n", executable_dirpath);
      msg_trace2("executable_dirpath_len = %i\n", executable_dirpath_len);
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
  msg_trace2("sys_config = %s\n", sys_config);
  msg_trace2("sys_config_found = %s\n", sys_config_found ? "true" : "false");
  msg_trace2("\n");

  /*
   * USER CONFIG
   * -----------
   * Determine the location of '.avrduderc'.
   */
#if defined(WIN32)
  win_usr_config_set(usr_config);
#else
  usr_config[0] = 0;
  if(!concatpath(usr_config, getenv("XDG_CONFIG_HOME"), XDG_USER_CONF_FILE, sizeof usr_config))
    concatpath(usr_config, getenv("HOME"), ".config/" XDG_USER_CONF_FILE, sizeof usr_config);
  if(stat(usr_config, &sb) < 0 || (sb.st_mode & S_IFREG) == 0)
    concatpath(usr_config, getenv("HOME"), USER_CONF_FILE, sizeof usr_config);
#endif

  if (quell_progress == 0)
    terminal_setup_update_progress();

  /*
   * Print out an identifying string so folks can tell what version
   * they are running
   */
  msg_notice("\n");
  pmsg_notice("Version %s\n", version);
  imsg_notice("Copyright the AVRDUDE authors;\n");
  imsg_notice("see https://github.com/avrdudes/avrdude/blob/main/AUTHORS\n\n");

  if(*sys_config) {
    char *real_sys_config = realpath(sys_config, NULL);
    if(real_sys_config) {
     imsg_notice("System wide configuration file is %s\n", real_sys_config);
    } else
      pmsg_warning("cannot determine realpath() of config file %s: %s\n", sys_config, strerror(errno));

    rc = read_config(real_sys_config);
    if (rc) {
      pmsg_error("unable to process system wide configuration file %s\n", real_sys_config);
      exit(1);
    }
    free(real_sys_config);
  }

  if (usr_config[0] != 0) {
    imsg_notice("User configuration file is %s\n", usr_config);

    rc = stat(usr_config, &sb);
    if ((rc < 0) || ((sb.st_mode & S_IFREG) == 0))
      imsg_notice("User configuration file does not exist or is not a regular file, skipping\n");
    else {
      rc = read_config(usr_config);
      if (rc) {
        pmsg_error("unable to process user configuration file %s\n", usr_config);
        exit(1);
      }
    }
  }

  if (lsize(additional_config_files) > 0) {
    LNODEID ln1;
    const char * p = NULL;

    for (ln1=lfirst(additional_config_files); ln1; ln1=lnext(ln1)) {
      p = ldata(ln1);
      imsg_notice("additional configuration file is %s\n", p);

      rc = read_config(p);
      if (rc) {
        pmsg_error("unable to process additional configuration file %s\n", p);
        exit(1);
      }
    }
  }

  // set bitclock from configuration files unless changed by command line
  if (default_bitclock > 0 && bitclock == 0.0) {
    bitclock = default_bitclock;
  }

  if(!(programmer && *programmer) && *default_programmer)
    programmer = cache_string(default_programmer);

  // Developer options to print parts and/or programmer entries of avrdude.conf
  int dev_opt_c = dev_opt(programmer); // -c <wildcard>/[sSArt]
  int dev_opt_p = dev_opt(partdesc);   // -p <wildcard>/[dsSArcow*t]

  if(dev_opt_c || dev_opt_p) {  // See -c/h and or -p/h
    dev_output_pgm_part(dev_opt_c, programmer, dev_opt_p, partdesc);
    exit(0);
  }

  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
    AVRPART *p = ldata(ln1);
    for(LNODEID ln2 = lfirst(programmers); ln2; ln2 = lnext(ln2)) {
      PROGRAMMER *pgm = ldata(ln2);
      int pm = pgm->prog_modes & p->prog_modes;
      if(pm & (pm-1))
        pmsg_warning("%s and %s share multiple modes (%s)\n",
          pgm->id? (char *) ldata(lfirst(pgm->id)): "???", p->desc, avr_prog_modes(pm));
    }
  }

  if (partdesc) {
    if (strcmp(partdesc, "?") == 0) {
      if(programmer && *programmer && explicit_c) {
        PROGRAMMER *pgm = locate_programmer(programmers, programmer);
        if(!pgm) {
          programmer_not_found(programmer);
          exit(1);
        }
        msg_error("\nValid parts for programmer %s are:\n", programmer);
        list_parts(stderr, "  ", part_list, pgm->prog_modes);
      } else {
        msg_error("\nValid parts are:\n");
        list_parts(stderr, "  ", part_list, ~0);
      }
      msg_error("\n");
      exit(1);
    }
  }

  if (programmer) {
    if (strcmp(programmer, "?") == 0) {
      if(partdesc && *partdesc) {
        AVRPART *p = locate_part(part_list, partdesc);
        if(!p) {
          part_not_found(partdesc);
          exit(1);
        }
        msg_error("\nValid programmers for part %s are:\n", p->desc);
        list_programmers(stderr, "  ", programmers, p->prog_modes);
      }  else {
        msg_error("\nValid programmers are:\n");
        list_programmers(stderr, "  ", programmers, ~0);
      }
      msg_error("\n");
      exit(1);
    }

    if (strcmp(programmer, "?type") == 0) {
      msg_error("\nValid programmer types are:\n");
      list_programmer_types(stderr, "  ");
      msg_error("\n");
      exit(1);
    }
  }

  msg_notice("\n");

  if (!programmer || !*programmer) {
    programmer_not_found(NULL);
    exit(1);
  }

  pgm = locate_programmer(programmers, programmer);
  if (pgm == NULL) {
    programmer_not_found(programmer);
    exit(1);
  }

  if (pgm->initpgm) {
    pgm->initpgm(pgm);
  } else {
    msg_error("\n");
    pmsg_error("cannot initialize the programmer\n\n");
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
      for (LNODEID ln = lfirst(extended_params); ln; ln = lnext(ln)) {
        const char *extended_param = ldata(ln);
        if (str_eq(extended_param, "help")) {
          char *prg = (char *)ldata(lfirst(pgm->id));
          msg_error("%s -c %s extended options:\n", progname, prg);
          msg_error("  -xhelp    Show this help menu and exit\n");
          exit(0);
        }
        else
          pmsg_error("programmer does not support extended parameter -x %s, option ignored\n", extended_param);
      }
    } else {
      if (pgm->parseextparams(pgm, extended_params) < 0) {
        pmsg_error("unable to parse extended parameter list\n");
        exit(1);
      }
    }
  }

  if (port == NULL) {
    switch (pgm->conntype)
    {
      case CONNTYPE_PARALLEL:
        port = cfg_strdup("main()", default_parallel);
        break;

      case CONNTYPE_SERIAL:
        port = cfg_strdup("main()", default_serial);
        break;

      case CONNTYPE_USB:
        port = DEFAULT_USB;
        break;

      case CONNTYPE_SPI:
#ifdef HAVE_LINUXSPI
        port = cfg_strdup("main()", *default_spi? default_spi: "unknown");
#endif
        break;

      case CONNTYPE_LINUXGPIO:
        port = cfg_strdup("main()", default_linuxgpio);
        break;

    }
  }

  /*
   * open the programmer
   */
  if (port[0] == 0) {
    msg_error("\n");
    pmsg_error("no port has been specified on the command line or in the config file\n");
    imsg_error("specify a port using the -P option and try again\n\n");
    exit(1);
  }

  if (verbose) {
    imsg_notice("Using Port                    : %s\n", port);
    imsg_notice("Using Programmer              : %s\n", programmer);
  }

  if (baudrate != 0) {
    imsg_notice("Overriding Baud Rate          : %d\n", baudrate);
    pgm->baudrate = baudrate;
  }

  if (bitclock != 0.0) {
    imsg_notice("Setting bit clk period        : %.1f\n", bitclock);
    pgm->bitclock = bitclock * 1e-6;
  }

  if (ispdelay != 0) {
    imsg_notice("Setting isp clock delay        : %3i\n", ispdelay);
    pgm->ispdelay = ispdelay;
  }

  rc = pgm->open(pgm, port);
  if (rc < 0) {
    pmsg_error("unable to open programmer %s on port %s\n", programmer, port);
    exitrc = 1;
    pgm->ppidata = 0; /* clear all bits at exit */
    goto main_exit;
  }
  is_open = 1;

  if (partdesc == NULL) {
    part_not_found(NULL);
    exitrc = 1;
    goto main_exit;
  }

  p = locate_part(part_list, partdesc);
  if (p == NULL) {
    part_not_found(partdesc);
    exitrc = 1;
    goto main_exit;
  }

  if (exitspecs != NULL) {
    if (pgm->parseexitspecs == NULL) {
      pmsg_warning("-E option not supported by this programmer type\n");
      exitspecs = NULL;
    }
    else if (pgm->parseexitspecs(pgm, exitspecs) < 0) {
      usage();
      exitrc = 1;
      goto main_exit;
    }
  }

  if (avr_initmem(p) != 0) {
    msg_error("\n");
    pmsg_error("unable to initialize memories\n");
    exitrc = 1;
    goto main_exit;
  }

  if(verbose) {
    if ((strcmp(pgm->type, "avr910") == 0)) {
      imsg_notice("avr910_devcode (avrdude.conf) : ");
      if(p->avr910_devcode)
        msg_notice("0x%02x\n", (uint8_t) p->avr910_devcode);
      else
        msg_notice("none\n");
    }
  }

  /*
   * Now that we know which part we are going to program, locate any -U
   * options using the default memory region, fill in the device-dependent
   * default region name ("application" for Xmega parts or "flash" otherwise)
   * and check for basic problems with memory names or file access with a
   * view to exit before programming.
   */
  int doexit = 0;
  for (ln=lfirst(updates); ln; ln=lnext(ln)) {
    upd = ldata(ln);
    if (upd->memtype == NULL) {
      const char *mtype = p->prog_modes & PM_PDI? "application": "flash";
      pmsg_notice2("defaulting memtype in -U %c:%s option to \"%s\"\n",
        (upd->op == DEVICE_READ)? 'r': (upd->op == DEVICE_WRITE)? 'w': 'v',
        upd->filename, mtype);
      upd->memtype = cfg_strdup("main()", mtype);
    }

    rc = update_dryrun(p, upd);
    if (rc && rc != LIBAVRDUDE_SOFTFAIL)
      doexit = 1;
  }
  if(doexit) {
    exitrc = 1;
    goto main_exit;
  }

  if (calibrate) {
    /*
     * perform an RC oscillator calibration
     * as outlined in appnote AVR053
     */
    if (pgm->perform_osccal == 0) {
      pmsg_error("programmer does not support RC oscillator calibration\n");
      exitrc = 1;
    } else {
      pmsg_info("performing RC oscillator calibration\n");
      exitrc = pgm->perform_osccal(pgm);
    }
    if (exitrc == 0)
      pmsg_info("calibration value is now stored in EEPROM at address 0\n");
    goto main_exit;
  }

  if (verbose && quell_progress < 2) {
    avr_display(stderr, p, progbuf, verbose);
    msg_notice("\n");
    programmer_display(pgm, progbuf);
  }

  msg_info("\v");

  exitrc = 0;

  /*
   * enable the programmer
   */
  pgm->enable(pgm, p);

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
    pmsg_error("initialization failed, rc=%d\n", rc);
    if (rc == -2)
      imsg_error("the programmer ISP clock is too fast for the target\n");
    else
      imsg_error("- double check the connections and try again\n");

    if (strcmp(pgm->type, "serialupdi") == 0 || strcmp(pgm->type, "SERBB") == 0)
      imsg_error("- use -b to set lower baud rate, e.g. -b %d\n", baudrate? baudrate/2: 57600);
    else
      imsg_error("- use -B to set lower the bit clock frequency, e.g. -B 125kHz\n");

    if (!ovsigck) {
      imsg_error("- use -F to override this check\n");
      exitrc = 1;
      goto main_exit;
    }
  }

  /* indicate ready */
  pgm->rdy_led(pgm, ON);

  pmsg_info("AVR device initialized and ready to accept instructions\n");

  /*
   * Let's read the signature bytes to make sure there is at least a
   * chip on the other end that is responding correctly.  A check
   * against 0xffffff / 0x000000 should ensure that the signature bytes
   * are valid.
   */
  if(!(p->prog_modes & PM_aWire)) { // not AVR32
    int attempt = 0;
    int waittime = 10000;       /* 10 ms */

  sig_again:
    usleep(waittime);
    if (init_ok) {
      rc = avr_signature(pgm, p);
      if (rc != LIBAVRDUDE_SUCCESS) {
        if (rc == LIBAVRDUDE_SOFTFAIL && (p->prog_modes & PM_UPDI) && attempt < 1) {
          attempt++;
          if (pgm->read_sib) {
            // Read SIB and compare FamilyID
            char sib[AVR_SIBLEN + 1];
            pgm->read_sib(pgm, p, sib);
            pmsg_notice("System Information Block: %s\n", sib);
            pmsg_info("received FamilyID: \"%.*s\"\n", AVR_FAMILYIDLEN, sib);
            if (strncmp(p->family_id, sib, AVR_FAMILYIDLEN))
              pmsg_error("expected FamilyID: \"%s\"\n", p->family_id);
          }
          if(erase) {
            erase = 0;
            if (uflags & UF_NOWRITE) {
              pmsg_warning("conflicting -e and -n options specified, NOT erasing chip\n");
            } else {
              pmsg_info("erasing chip\n");
              exitrc = avr_unlock(pgm, p);
              if(exitrc)
                goto main_exit;
              goto sig_again;
            }
          }
          if (!ovsigck) {
            imsg_error("double check chip or use -F to override this check\n");
            exitrc = 1;
            goto main_exit;
          }
        }
        pmsg_error("unable to read signature data, rc=%d\n", rc);
        exitrc = 1;
        goto main_exit;
      }
    }

    sig = avr_locate_mem(p, "signature");
    if (sig == NULL)
      pmsg_warning("signature memory not defined for device %s\n", p->desc);

    if (sig != NULL) {
      int ff, zz;

      pmsg_info("device signature = 0x");
      ff = zz = 1;
      for (i=0; i<sig->size; i++) {
        msg_info("%02x", sig->buf[i]);
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
        if((part = locate_part_by_signature(part_list, sig->buf, sig->size)))
          msg_info(" (probably %s)", signature_matches ? p->id : part->id);
      }
      if (ff || zz) {
        if (++attempt < 3) {
          waittime *= 5;
          msg_info(" (retrying)\n");
          goto sig_again;
        }
        msg_info("\n");
        pmsg_error("Yikes!  Invalid device signature.\n");
        if (!ovsigck) {
          pmsg_error("expected signature for %s is %02X %02X %02X\n", p->desc,
            p->signature[0], p->signature[1], p->signature[2]);
          imsg_error("Double check connections and try again, or use -F to override\n");
          imsg_error("this check.\n\n");
          exitrc = 1;
          goto main_exit;
        }
      } else {
        msg_info("\n");
      }

      if (!signature_matches) {
        if (ovsigck) {
          pmsg_warning("expected signature for %s is %02X %02X %02X\n", p->desc,
            p->signature[0], p->signature[1], p->signature[2]);
        } else {
          pmsg_error("expected signature for %s is %02X %02X %02X\n", p->desc,
            p->signature[0], p->signature[1], p->signature[2]);
          imsg_error("double check chip or use -F to override this check\n");
          exitrc = 1;
          goto main_exit;
        }
      }
    }
  }

  if (uflags & UF_AUTO_ERASE) {
    if ((p->prog_modes & PM_PDI) && pgm->page_erase && lsize(updates) > 0) {
      pmsg_info("Note: programmer supports page erase for Xmega devices.\n");
      imsg_info("Each page will be erased before programming it, but no chip erase is performed.\n");
      imsg_info("To disable page erases, specify the -D option; for a chip-erase, use the -e option.\n");
    } else {
      AVRMEM * m;
      const char *memname = p->prog_modes & PM_PDI? "application": "flash";

      uflags &= ~UF_AUTO_ERASE;
      for (ln=lfirst(updates); ln; ln=lnext(ln)) {
        upd = ldata(ln);
        m = avr_locate_mem(p, upd->memtype);
        if (m == NULL)
          continue;
        if ((strcmp(m->desc, memname) == 0) && (upd->op == DEVICE_WRITE)) {
          erase = 1;
          pmsg_info("Note: %s memory has been specified, an erase cycle will be performed.\n", memname);
          imsg_info("To disable this feature, specify the -D option.\n");
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
      pmsg_warning("%s-n specified, NOT erasing chip\n", explicit_e? "conflicting -e and ": "");
    } else {
      pmsg_info("erasing chip\n");
      exitrc = avr_chip_erase(pgm, p);
      if(exitrc == LIBAVRDUDE_SOFTFAIL) {
        imsg_info("delaying chip erase until first -U upload to flash\n");
        ce_delayed = 1;
        exitrc = 0;
      } else if(exitrc)
        goto main_exit;
    }
  }

  if (terminal) {
    /*
     * terminal mode
     */
    if (uflags & UF_NOWRITE)
      pmsg_warning("the terminal ignores option -n, that is, it writes to the device\n");
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
    } else if(rc == 0 && upd->op == DEVICE_WRITE && avr_memtype_is_flash_type(upd->memtype))
      ce_delayed = 0;           // Redeemed chip erase promise
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

  msg_info("\n%s done.  Thank you.\n\n", progname);

  return ce_delayed? 1: exitrc;
}
