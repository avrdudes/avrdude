/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005 Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) Joerg Wunsch <j@uriah.heep.sax.de>
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

/*
 * Program an Atmel AVR device through one of the supported programmers
 *
 * For parallel port connected programmers, the pin definitions can be changed
 * via a config file.  See the config file for instructions on how to add a
 * programmer definition.
 *
 */

// For AVRDUDE_FULL_VERSION and possibly others
#include <ac_cfg.h>

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

#if !defined(WIN32)
#include <dirent.h>
#endif

#include "avrdude.h"
#include "libavrdude.h"
#include "config.h"
#include "developer_opts.h"

char *progname = "avrdude";

static const char *avrdude_message_type(int msglvl) {
  switch(msglvl) {
  case MSG_EXT_ERROR:
    return "OS error";
  case MSG_ERROR:
    return "error";
  case MSG_WARNING:
    return "warning";
  case MSG_INFO:
    return "info";
  case MSG_NOTICE:
    return "notice";
  case MSG_NOTICE2:
    return "notice2";
  case MSG_DEBUG:
    return "debug";
  case MSG_TRACE:
    return "trace";
  case MSG_TRACE2:
    return "trace2";
  default:
    return "unknown msglvl";
  }
}

/*
 * Core messaging routine for msg_xyz(), [pli]msg_xyz() and term_out()
 * See #define lines in avrdude.h of how it is normally called
 *
 * Named that way as there used to be a now gone different avrdude_message()
 */
int avrdude_message2(FILE *fp, int lno, const char *file, const char *func,
  int msgmode, int msglvl, const char *format, ...) {

  int rc = 0;
  va_list ap;

  static struct {               // Memorise whether last print ended at beginning of line
    FILE *fp;
    int bol;                    // Are we at the beginning of a line for this fp stream?
  } bols[5 + 1];                // Cater for up to 5 different FILE pointers plus one catch-all

  size_t bi = 0;                // bi is index to bols[] array

  for(bi = 0; bi < sizeof bols/sizeof *bols - 1; bi++) {      // Note the -1, so bi is valid after loop
    if(!bols[bi].fp) {          // First free space
      bols[bi].fp = fp;         // Insert fp in first free space
      bols[bi].bol = 1;         // Assume beginning of line on first use
    }
    if(bols[bi].fp == fp)
      break;
  }

  if(msglvl <= MSG_ERROR)       // Serious error? Free progress bars (if any)
    report_progress(1, -1, NULL);

  if(msgmode & MSG2_FLUSH) {
    fflush(stdout);
    fflush(stderr);
  }
  // Reduce effective verbosity level by number of -q above one when printing to stderr
  if((quell_progress < 2 || fp != stderr? verbose: verbose + 1 - quell_progress) >= msglvl) {
    if(msgmode & MSG2_LEFT_MARGIN && !bols[bi].bol) {
      fprintf(fp, "\n");
      bols[bi].bol = 1;
    }
    // Keep vertical tab at start of format string as conditional new line
    if(*format == '\v') {
      format++;
      if(!bols[bi].bol) {
        fprintf(fp, "\n");
        bols[bi].bol = 1;
      }
    }

    if(msgmode & (MSG2_PROGNAME | MSG2_TYPE)) {
      if(msgmode & MSG2_PROGNAME) {
        fprintf(fp, "%s", progname);
        bols[bi].bol = 0;
      }
      if(msgmode & MSG2_TYPE) {
        const char *mt = avrdude_message_type(msglvl);

        if(bols[bi].bol)
          fprintf(fp, "%c%s", msgmode & (MSG2_UCFIRST)? toupper(*mt & 0xff): *mt, mt + 1);
        else
          fprintf(fp, " %s", mt);
        bols[bi].bol = 0;
      }
      if(verbose >= MSG_NOTICE2) {
        const char *bfname = strrchr(file, '/');        // Only print basename

#if defined (WIN32)
        if(!bfname)
          bfname = strrchr(file, '\\');
#endif

        bfname = bfname? bfname + 1: file;
        if(msgmode & MSG2_FUNCTION)
          fprintf(fp, " %s()", func);
        if(msgmode & MSG2_FILELINE)
          fprintf(fp, " %s %d", bfname, lno);
      }
      fprintf(fp, ": ");
    } else if(msgmode & MSG2_INDENT1) {
      fprintf(fp, "%*s", (int) strlen(progname) + 1, "");
      bols[bi].bol = 0;
    } else if(msgmode & MSG2_INDENT2) {
      fprintf(fp, "%*s", (int) strlen(progname) + 2, "");
      bols[bi].bol = 0;
    }
    // Figure out whether this print will leave us at beginning of line

    // Determine required size first
    va_start(ap, format);
    rc = vsnprintf(NULL, 0, format, ap);
    va_end(ap);

    if(rc < 0)                  // Some errror?
      return 0;

    rc++;                       // Accommodate terminating nul
    char *p = mmt_malloc(rc);

    va_start(ap, format);
    rc = vsnprintf(p, rc, format, ap);
    va_end(ap);

    if(rc < 0) {
      mmt_free(p);
      return 0;
    }

    if(*p) {                    // Finally: print!
      if(bols[bi].bol && (msgmode & MSG2_UCFIRST))
        fprintf(fp, "%c%s", toupper(*p & 0xff), p + 1);
      else
        fprintf(fp, "%s", p);
      bols[bi].bol = p[strlen(p) - 1] == '\n';
    }
    mmt_free(p);
  }

  if(msgmode & MSG2_FLUSH)
    fflush(fp);

  return rc;
}

struct list_walk_cookie {
  FILE *f;
  const char *prefix;
};

libavrdude_context *cx;         // Context pointer, eventually the only global variable

static LISTID updates = NULL;

static LISTID extended_params = NULL;

static LISTID additional_config_files = NULL;

static PROGRAMMER *pgm;

// Global options
int verbose;                    // Verbose output
int quell_progress;             // Quell progress report and un-verbose output
int ovsigck;                    // 1 = override sig check, 0 = don't
const char *partdesc;           // Part -p string
const char *pgmid;              // Programmer -c string

static char usr_config[PATH_MAX];       // Per-user config file

// Usage message
static void usage(void) {
  char *home = getenv("HOME");
  size_t l = home? strlen(home): 0;
  char *cfg = home && str_casestarts(usr_config, home)?
    mmt_sprintf("~/%s", usr_config + l + (usr_config[l] == '/')): mmt_sprintf("%s", usr_config);

  msg_error("Usage: %s [options]\n"
    "Options:\n"
    "  -p <partno>            Specify AVR device; -p ? lists all known parts\n"
    "  -p <wildcard>/<flags>  Run developer options for matched AVR devices,\n"
    "                         e.g., -p ATmega328P/s or /S for part definition\n"
    "  -b <baudrate>          Override RS-232 baud rate\n"
    "  -B <bitclock>          Specify bit clock period (us)\n"
    "  -C <config-file>       Specify location of configuration file\n"
    "  -C +<config-file>      Specify additional config file, can be repeated\n"
    "  -N                     Do not load %s%s\n"
    "  -c <programmer>        Specify programmer; -c ? and -c ?type list all\n"
    "  -c <wildcard>/<flags>  Run developer options for matched programmers,\n"
    "                         e.g., -c 'ur*'/s for programmer info/definition\n"
    "  -A                     Disable trailing-0xff removal for file/AVR read\n"
    "  -D                     Disable auto-erase for flash memory; implies -A\n"
    "  -i <delay>             ISP Clock Delay [in microseconds]\n"
    "  -P <port>              Connection; -P ?s or -P ?sa lists serial ones\n"
    "  -r                     Reconnect to -P port after \"touching\" it; wait\n"
    "                         400 ms for each -r; needed for some USB boards\n"
    "  -F                     Override invalid signature or initial checks\n"
    "  -e                     Perform a chip erase at the beginning\n"
    "  -O                     Perform RC oscillator calibration (see AVR053)\n"
    "  -t                     Run an interactive terminal when it is its turn\n"
    "  -T <terminal cmd line> Run terminal line when it is its turn\n"
    "  -U <memstr>:r|w|v:<filename>[:format]\n"
    "                         Carry out memory operation when it is its turn\n"
    "                         Multiple -t, -T and -U options can be specified\n"
    "  -n                     Do not write to the device whilst processing -U\n"
    "  -V                     Do not automatically verify during -U\n"
    "  -E <exitsp>[,<exitsp>] List programmer exit specifications\n"
    "  -x <extended_param>    Pass <extended_param> to programmer, see -x help\n"
    "  -v                     Verbose output; -v -v for more\n"
    "  -q                     Quell progress output; -q -q for less\n"
    "  -l logfile             Use logfile rather than stderr for diagnostics\n"
    "  -?                     Display this usage\n"
    "\navrdude version %s, https://github.com/avrdudes/avrdude\n",
    progname, strlen(cfg) < 24? "config file ": "", cfg, AVRDUDE_FULL_VERSION);

  mmt_free(cfg);
}

// Potentially shorten copy of prog description if it's the suggested mode
static void pmshorten(char *desc, const char *modes) {
  struct {
    const char *end, *mode;
  } pairs[] = {
    {" in parallel programming mode", "HVPP"},
    {" in PP mode", "HVPP"},
    {" in high-voltage serial programming mode", "HVSP"},
    {" in HV serial programming mode", "HVSP"},
    {" in HVSP mode", "HVSP"},
    {" in ISP mode", "ISP"},
    {" in ISP mode", "TPI, ISP"},
    {" in TPI mode", "TPI"},
    {" in debugWire mode", "debugWIRE"},
    {" in AVR32 mode", "aWire"},
    {" in PDI mode", "PDI"},
    {" in UPDI mode", "UPDI"},
    {" in JTAG mode", "JTAG"},
    {" in JTAG mode", "JTAGmkI"},
    {" in JTAG mode", "XMEGAJTAG"},
    {" in JTAG mode", "AVR32JTAG"},
    {" in JTAG mode", "JTAG, XMEGAJTAG, AVR32JTAG"},
    {" in JTAG mode", "JTAG, XMEGAJTAG"},
    {" for bootloader", "bootloader"},
  };
  size_t len = strlen(desc);

  for(size_t i = 0; i < sizeof pairs/sizeof *pairs; i++) {
    size_t elen = strlen(pairs[i].end);

    if(len > elen && str_caseeq(desc + len - elen, pairs[i].end) && str_eq(modes, pairs[i].mode)) {
      desc[len - elen] = 0;
      break;
    }
  }
}

static void list_programmers(FILE *f, const char *prefix, LISTID programmers, int pm) {
  LNODEID ln1;
  LNODEID ln2;
  PROGRAMMER *pgm;
  int maxlen = 0, len;
  PROGRAMMER *dry = locate_programmer(programmers, "dryrun");

  sort_programmers(programmers);

  // Compute max length of programmer names
  for(ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    pgm = ldata(ln1);
    if(!is_programmer(pgm))
      continue;
    for(ln2 = lfirst(pgm->id); ln2; ln2 = lnext(ln2))
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
    if(!is_programmer(pgm))
      continue;
    for(ln2 = lfirst(pgm->id); ln2; ln2 = lnext(ln2)) {
      // List programmer if pm or prog_modes uninitialised or if they are compatible otherwise
      if(!pm || !pgm->prog_modes || (pm & pgm->prog_modes)) {
        const char *id = ldata(ln2);
        if(*id == 0 || *id == '.')
          continue;

        char *desc = mmt_strdup(pgm->desc);
        const char *strmodes = str_prog_modes(pgm->prog_modes);
        pmshorten(desc, avr_prog_modes(pm & pgm->prog_modes));
        if(verbose > 0)
          fprintf(f, "%s%-*s = %s (%s) [%s:%d]", prefix, maxlen, id, desc,
            strmodes, pgm->config_file, pgm->lineno);
        else
          fprintf(f, "%s%-*s = %s (%s)", prefix, maxlen, id, desc, strmodes);
        if(pm != ~0 && strchr(strmodes, ' ') && !(dry && pgm->initpgm == dry->initpgm))
          fprintf(f, " via %s", str_prog_modes(pm & pgm->prog_modes));
        fprintf(f, "\n");

        mmt_free(desc);
      }
    }
  }
}

static void list_programmer_types_callback(const char *name, const char *desc, void *cookie) {
  struct list_walk_cookie *c = (struct list_walk_cookie *) cookie;

  fprintf(c->f, "%s%-16s = %-s\n", c->prefix, name, desc);
}

static void list_programmer_types(FILE *f, const char *prefix) {
  struct list_walk_cookie c;

  c.f = f;
  c.prefix = prefix;

  walk_programmer_types(list_programmer_types_callback, &c);
}

// Return a list of long names for part followed by prog modes in brackets
static const char *part_ccdesc(const AVRPART *p) {
  char *name[5];               // Max 5 alternative names
  int nn = 0, i;
  char *pmodes = mmt_strdup(str_prog_modes(p->prog_modes));
  char ret[6*(64+2) + 256 + 20], *r = ret;

  // Create list name[] of alternative names to p->desc
  for(LNODEID ln = lfirst(p->variants); ln; ln = lnext(ln)) {
    const char *alt = ldata(ln), *end, *q;
    if((end = strchr(alt, ':')) && end > alt && *alt != '-') {
      if((q = strchr(alt, '-')) && q < end)
        end = q;
      if(strncasecmp(p->desc, alt, end-alt) || p->desc[end-alt]) { // Variant's base is not p->desc
// printf("X %.*s", (int) (end-alt), alt);
        for(i = 0; i < nn; i++)
          if(!strncasecmp(name[i], alt, end-alt) && !name[i][end-alt])
            break;
        if(i == nn && nn < 5)
          name[nn++] = str_sprintf("%.*s", (int) (end-alt), alt);
      }
    }
  }
  sprintf(r, "%.64s", p->desc), r += strlen(r);
  for(i = 0; i < nn; i++) {
    sprintf(r, ", %.64s", name[i]), r += strlen(r);
    mmt_free(name[i]);
  }
  sprintf(r, " (%.256s)", pmodes);
  mmt_free(pmodes);

  return str_ccprintf("%s", ret);
}

static void list_parts(FILE *f, const char *prefix, LISTID avrparts, int pm) {
  LNODEID ln1;
  AVRPART *p;
  int maxlen = 0, len;

  sort_avrparts(avrparts);

  // Compute max length of part names
  for(ln1 = lfirst(avrparts); ln1; ln1 = lnext(ln1)) {
    p = ldata(ln1);
    // List part if pm or prog_modes uninitialised or if they are compatible otherwise
    if(!pm || !p->prog_modes || (pm & p->prog_modes)) {
      if(verbose < MSG_NOTICE2 && p->id[0] == '.')      // Hide ids starting with '.'
        continue;
      if((len = strlen(p->id)) > maxlen)
        maxlen = len;
    }
  }

  for(ln1 = lfirst(avrparts); ln1; ln1 = lnext(ln1)) {
    p = ldata(ln1);
    // List part if pm or prog_modes uninitialised or if they are compatible otherwise
    if(!pm || !p->prog_modes || (pm & p->prog_modes)) {
      if(verbose < MSG_NOTICE2 && p->id[0] == '.')      // Hide ids starting with '.'
        continue;
      if(verbose > 0)
        fprintf(f, "%s%-*s = %-18s [%s:%d]", prefix, maxlen, p->id, part_ccdesc(p),
          p->config_file, p->lineno);
      else
        fprintf(f, "%s%-*s = %s", prefix, maxlen, p->id, part_ccdesc(p));
      if(pm != ~0)
        fprintf(f, " via %s", avr_prog_modes(pm & p->prog_modes));
      fprintf(f, "\n");
      if(verbose > 0)
        for(LNODEID ln = lfirst(p->variants); ln; ln = lnext(ln))
          fprintf(f, "%s%s- %s\n", prefix, prefix, (char *) ldata(ln));
    }
  }
}

static void exithook(void) {
  if(pgm->teardown)
    pgm->teardown(pgm);
}

static void cleanup_main(void) {
  if(updates) {
    ldestroy_cb(updates, (void (*)(void *)) free_update);
    updates = NULL;
  }
  if(extended_params) {
    ldestroy(extended_params);
    extended_params = NULL;
  }
  if(additional_config_files) {
    ldestroy(additional_config_files);
    additional_config_files = NULL;
  }

  cleanup_config();
}

static void replace_backslashes(char *s) {
  // Replace all backslashes with forward slashes
  for(size_t i = 0; i < strlen(s); i++) {
    if(s[i] == '\\') {
      s[i] = '/';
    }
  }
}

// Return whether a part/programmer string is a developer option and if so which type
static int dev_opt(const char *str) {
  return !str? 0: str_eq(str, "*") || str_starts(str, "*/s")? 2:    // Print PART DEFINITIONS comment as well
    strchr(str, '/') && !locate_part(part_list, str);
}

typedef struct {
  size_t dist;
  int common_modes;
  const char *pgmid;
  const char *desc;
} pgm_distance;

static int cmp_pgmid(const void *a, const void *b) {
  const pgm_distance *pa = a, *pb = b;

  int ret = pa->dist - pb->dist;

  if(ret)
    return ret;
  return strcmp(pa->pgmid, pb->pgmid);
}

static int suggest_programmers(const char *programmer, LISTID programmers) {
  const size_t max_distance = 64;       // Don't show suggestions if they are way far out

  int nid = 0;                  // Number of possible programmer ids

  for(LNODEID ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    PROGRAMMER *pgm = ldata(ln1);

    if(is_programmer(pgm))
      for(LNODEID ln2 = lfirst(pgm->id); ln2; ln2 = lnext(ln2))
        nid++;
  }

  pgm_distance *d = mmt_malloc(nid*sizeof *d);

  // Fill d[] struct
  int idx = 0;
  AVRPART *p = locate_part(part_list, partdesc);

  for(LNODEID ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    PROGRAMMER *pgm = ldata(ln1);

    if(!is_programmer(pgm))
      continue;
    for(LNODEID ln2 = lfirst(pgm->id); ln2; ln2 = lnext(ln2)) {
      if(idx < nid) {
        d[idx].pgmid = ldata(ln2);
        d[idx].desc = pgm->desc;
        d[idx].dist = str_weighted_damerau_levenshtein(d[idx].pgmid, programmer);
        d[idx].common_modes = pgm->prog_modes & (p? p->prog_modes: ~0);
        idx++;
      }
    }
  }

  int n = 0, pgmid_maxlen = 0, comp = 0, len;

  if(nid) {                     // Sort list so programmers according to string distance
    qsort(d, nid, sizeof(*d), cmp_pgmid);
    size_t dst = d[nid > 2? 2: nid - 1].dist;

    if(dst > max_distance)
      dst = max_distance;
    for(; n < nid && d[n].dist <= dst; n++)
      if(d[n].common_modes) {
        if((len = strlen(d[n].pgmid)) > pgmid_maxlen)
          pgmid_maxlen = len;
        comp++;
      }
  }
  if(comp) {
    msg_info("similar programmer name%s:\n", str_plural(comp));
    for(int i = 0; i < n; i++)
      if(d[i].common_modes)
        msg_info("  %-*s = %s\n", pgmid_maxlen, d[i].pgmid, d[i].desc);
  }
  mmt_free(d);
  return n;
}

static void programmer_not_found(const char *programmer, const PROGRAMMER *pgm, const AVRPART *pt) {
  int pmode = pt? pt->prog_modes: ~0;

  if(!programmer || !*programmer) {
    pmsg_error("no programmer has been specified on the command line or in the\n");
    imsg_error("config file(s); specify one using the -c option and try again\n");
    return;
  }

  if(str_eq(programmer, "?")) {
    lmsg_error("Valid programmers are:\n");
    list_programmers(stderr, "  ", programmers, ~0);
    msg_error("\n");
    return;
  }

  sort_programmers(programmers);

  // If there were partial matches then they were not unique: count and list them
  int pmatches = 0, maxlen = 0, len;

  for(LNODEID ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    PROGRAMMER *pg = ldata(ln1);

    if(is_programmer(pg) && (pg->prog_modes & pmode))
      for(LNODEID ln2 = lfirst(pg->id); ln2; ln2 = lnext(ln2)) {
        const char *id = (const char *) ldata(ln2);

        if(str_casestarts(id, programmer)) {    // Partial initial match
          pmatches++;
          if((len = strlen(id)) > maxlen)
            maxlen = len;
        }
      }
  }
  if(pmatches) {
    pmsg_error("%s is not a unique start of a programmer name; consider:\n", programmer);
    for(LNODEID ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
      PROGRAMMER *pg = ldata(ln1);

      if(is_programmer(pg) && (pg->prog_modes & pmode))
        for(LNODEID ln2 = lfirst(pg->id); ln2; ln2 = lnext(ln2)) {
          const char *id = (const char *) ldata(ln2);

          if(str_casestarts(id, programmer))
            msg_error("  %-*s = %s\n", maxlen, id, pg->desc);
        }
    }
  } else if(!pgm || !pgm->id || !lsize(pgm->id)) {
    PROGRAMMER *pg = locate_programmer(programmers, programmer);

    if(!pgm && pt && pg && !(pg->prog_modes & pmode)) {
      pmsg_error("programmer %s and part %s have no programming modes in common\n", programmer, pt->desc);
      msg_info("use -c? -p %s to see all possible programmers for %s\n", pt->desc, pt->desc);
    } else {
      pmsg_error("cannot find programmer id %s\n", programmer);
      suggest_programmers(programmer, programmers);
      msg_info("use -c? to see all possible programmers\n");
    }
  } else
    pmsg_error("programmer %s lacks %s setting\n", programmer,
      !pgm->prog_modes? "prog_modes": !pgm->initpgm? "type": "some");
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
  if(len + (dir[len - 1] != '/') + strlen(file) > n - 1)
    return NULL;

  if(dst != dir)
    strcpy(dst, dir);

  if(dst[len - 1] != '/')
    strcat(dst, "/");

  strcat(dst, file);

  return dst;
}
#endif


int main(int argc, char *argv[]) {
  int rc;                       // General return code checking
  int exitrc;                   // Exit code for main()
  int i;                        // General loop counter
  int ch;                       // Options flag
  struct avrpart *p;            // Which avr part we are programming
  AVRMEM *sig;                  // Signature data
  struct stat sb;
  UPDATE *upd;
  LNODEID *ln;

  // Options/operating mode variables
  int erase;                    // 1=erase chip, 0=don't
  int flashread;                // 1=flash is going to be read, 0=no flash reads
  int calibrate;                // 1=calibrate RC oscillator, 0=don't
  int no_avrduderc;             // 1=don't load personal conf file
  char *port;                   // Device port (/dev/xxx)
  const char *exitspecs;        // Exit specs string from command line
  int explicit_c;               // 1=explicit -c on command line, 0=not specified there
  int explicit_e;               // 1=explicit -e on command line, 0=not specified there
  char sys_config[PATH_MAX];    // System wide config file
  char executable_abspath[PATH_MAX];     // Absolute path to avrdude executable
  char executable_dirpath[PATH_MAX];     // Absolute path to folder with executable
  bool executable_abspath_found = false; // Absolute path to executable found
  bool sys_config_found = false;         // avrdude.conf file found
  char *e;                      // For strtod() error checking
  const char *errstr;           // For str_int() error checking
  int baudrate;                 // Override default programmer baud rate
  int touch_1200bps;            // Touch serial port prior to programming
  double bitclock;              // Specify programmer bit clock (JTAG ICE)
  int ispdelay;                 // Specify the delay for ISP clock
  int init_ok;                  // Device initialization worked well
  int is_open;                  // Device open succeeded
  int ce_delayed;               // Chip erase delayed
  char *logfile;                // Use logfile rather than stderr for diagnostics
  enum updateflags uflags = UF_AUTO_ERASE | UF_VERIFY;  // Flags for do_op()

  init_cx(NULL);

#ifdef _MSC_VER
  _set_printf_count_output(1);
#endif

  // Set line buffering for file descriptors so we see stdout and stderr properly interleaved
  setvbuf(stdout, (char *) NULL, _IOLBF, 0);
  setvbuf(stderr, (char *) NULL, _IOLBF, 0);

  sys_config[0] = '\0';

  progname = strrchr(argv[0], '/');

#if defined (WIN32)
  // Take care of backslash as dir sep in W32
  if(!progname)
    progname = strrchr(argv[0], '\\');
#endif                          // WIN32

  if(progname)
    progname++;
  else
    progname = argv[0];

  // Remove trailing .exe
  if(str_ends(progname, ".exe")) {
    progname = mmt_strdup(progname);    // Don't write to argv[0]
    progname[strlen(progname) - 4] = 0;
  }

  avrdude_conf_version = "";

  default_programmer = "";
  default_parallel = "";
  default_serial = "";
  default_spi = "";
  default_baudrate = 0;
  default_bitclock = 0.0;
  default_linuxgpio = "";
  allow_subshells = 0;

  init_config();

  atexit(cleanup_main);

  updates = lcreat(NULL, 0);
  if(updates == NULL) {
    pmsg_error("cannot initialize updater list\n");
    exit(1);
  }

  extended_params = lcreat(NULL, 0);
  if(extended_params == NULL) {
    pmsg_error("cannot initialize extended parameter list\n");
    exit(1);
  }

  additional_config_files = lcreat(NULL, 0);
  if(additional_config_files == NULL) {
    pmsg_error("cannot initialize additional config files list\n");
    exit(1);
  }

  partdesc = NULL;
  port = NULL;
  erase = 0;
  flashread = 0;
  calibrate = 0;
  no_avrduderc = 0;
  p = NULL;
  ovsigck = 0;
  quell_progress = 0;
  exitspecs = NULL;
  pgm = NULL;
  pgmid = "";
  explicit_c = 0;
  explicit_e = 0;
  verbose = 0;
  baudrate = 0;
  touch_1200bps = 0;
  bitclock = 0.0;
  ispdelay = 0;
  is_open = 0;
  ce_delayed = 0;
  logfile = NULL;

  if(argc == 1) {               // No arguments?
    usage();
    return 0;
  }

  // Determine the location of personal configuration file

#if defined(WIN32)
  win_set_path(usr_config, sizeof usr_config, USER_CONF_FILE);
#else
  usr_config[0] = 0;
  if(!concatpath(usr_config, getenv("XDG_CONFIG_HOME"), XDG_USER_CONF_FILE, sizeof usr_config))
    concatpath(usr_config, getenv("HOME"), ".config/" XDG_USER_CONF_FILE, sizeof usr_config);
  if(stat(usr_config, &sb) < 0 || (sb.st_mode & S_IFREG) == 0)
    concatpath(usr_config, getenv("HOME"), USER_CONF_FILE, sizeof usr_config);
#endif

  // Process command line arguments
  while((ch = getopt(argc, argv, "?Ab:B:c:C:DeE:Fi:l:nNp:OP:qrtT:U:vVx:")) != -1) {
    switch(ch) {
    case 'b':                  // Override default programmer baud rate
      baudrate = str_int(optarg, STR_INT32, &errstr);
      if(errstr) {
        pmsg_error("invalid baud rate %s specified: %s\n", optarg, errstr);
        exit(1);
      }
      break;

    case 'B':                  // Specify bit clock period
      bitclock = strtod(optarg, &e);
      if((e == optarg) || bitclock <= 0.0) {
        pmsg_error("invalid bit clock period %s\n", optarg);
        exit(1);
      }
      while(*e && isascii(*e & 0xff) && isspace(*e & 0xff))
        e++;
      if(*e == 0 || str_caseeq(e, "us"))        // us is optional and the default
        ;
      else if(str_caseeq(e, "m") || str_caseeq(e, "mhz"))
        bitclock = 1/bitclock;
      else if(str_caseeq(e, "k") || str_caseeq(e, "khz"))
        bitclock = 1e3/bitclock;
      else if(str_caseeq(e, "hz"))
        bitclock = 1e6/bitclock;
      else {
        pmsg_error("invalid bit clock unit %s\n", e);
        exit(1);
      }
      break;

    case 'i':                  // Specify isp clock delay
      ispdelay = str_int(optarg, STR_INT32, &errstr);
      if(errstr || ispdelay == 0) {
        pmsg_error("invalid isp clock delay %s specified", optarg);
        if(errstr)
          msg_error(": %s\n", errstr);
        else
          msg_error("\n");
        exit(1);
      }
      break;

    case 'c':                  // Programmer id
      pgmid = optarg;
      explicit_c = 1;
      break;

    case 'C':                  // System wide configuration file
      if(optarg[0] == '+') {
        ladd(additional_config_files, optarg + 1);
      } else {
        strncpy(sys_config, optarg, PATH_MAX);
        sys_config[PATH_MAX - 1] = 0;
      }
      break;

    case 'D':                  // Disable auto-erase
      uflags &= ~UF_AUTO_ERASE;
      // Fall through

    case 'A':                  // Explicit disabling of trailing-0xff removal
      cx->avr_disableffopt = 1;
      break;

    case 'e':                  // Perform a chip erase
      erase = 1;
      explicit_e = 1;
      uflags &= ~UF_AUTO_ERASE;
      break;

    case 'E':
      exitspecs = optarg;
      break;

    case 'F':                  // Override invalid signature check
      ovsigck = 1;
      break;

    case 'l':
      logfile = optarg;
      break;

    case 'n':
      uflags |= UF_NOWRITE;
      break;

    case 'N':
      no_avrduderc = 1;
      break;

    case 'O':                  // Perform RC oscillator calibration
      calibrate = 1;
      break;

    case 'p':                  // Specify AVR part
      partdesc = optarg;
      break;

    case 'P':
      port = mmt_strdup(optarg);
      break;

    case 'q':                  // Quell progress output
      quell_progress++;
      break;

    case 'r':
      touch_1200bps++;
      break;

    case 't':                  // Enter terminal mode
      ladd(updates, cmd_update("interactive terminal"));
      break;

    case 'T':
      ladd(updates, cmd_update(optarg));
      break;

    case 'U':
      upd = parse_op(optarg);
      if(upd == NULL) {
        pmsg_error("unable to parse update operation %s\n", optarg);
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

    case '?':                  // Help
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

  if(logfile != NULL) {
    FILE *newstderr = freopen(logfile, "w", stderr);

    if(newstderr == NULL) {
      // Help!  There's no stderr to complain to anymore now
      printf("Cannot create logfile %s: %s\n", logfile, strerror(errno));
      return 1;
    }
  }

  msg_debug("$ ");              // Record command line
  for(int i = 0; i < argc; i++)
    msg_debug("%s%c", str_ccsharg(argv[i]), i == argc - 1? '\n': ' ');

  size_t ztest;

  if(1 != sscanf("42", "%zi", &ztest) || ztest != 42)
    pmsg_warning("linked C library does not conform to C99; %s may not work as expected\n", progname);

  // Search for system configuration file unless -C conffile was given
  if(strlen(sys_config) == 0) {
    /*
     * Executable abspath: Determine the absolute path to avrdude executable.
     * This will be used to locate the avrdude.conf file later.
     */
    int executable_dirpath_len;
    int executable_abspath_len = wai_getExecutablePath(executable_abspath,
      PATH_MAX,
      &executable_dirpath_len);

    if(
      (executable_abspath_len != -1) &&
      (executable_abspath_len != 0) && (executable_dirpath_len != -1) && (executable_dirpath_len != 0)
      ) {
      // All requirements satisfied, executable path was found
      executable_abspath_found = true;

      // Make sure the string is null terminated
      executable_abspath[executable_abspath_len] = '\0';

      replace_backslashes(executable_abspath);

      // Define executable_dirpath to be the path to the parent folder of the executable
      strcpy(executable_dirpath, executable_abspath);
      executable_dirpath[executable_dirpath_len] = '\0';

      // Debug output
      msg_trace2("executable_abspath = %s\n", executable_abspath);
      msg_trace2("executable_abspath_len = %i\n", executable_abspath_len);
      msg_trace2("executable_dirpath = %s\n", executable_dirpath);
      msg_trace2("executable_dirpath_len = %i\n", executable_dirpath_len);
    }

    /*
     * System config
     * -------------
     * Determine the location of avrdude.conf. Check in this order:
     *  1. <dirpath of executable>/../etc/avrdude.conf
     *  2. <dirpath of executable>/avrdude.conf
     *  3. CONFIG_DIR/avrdude.conf
     *
     * When found, write the result into the 'sys_config' variable.
     */
    if(executable_abspath_found) {
      // 1. Check <dirpath of executable>/../etc/avrdude.conf
      strcpy(sys_config, executable_dirpath);
      sys_config[PATH_MAX - 1] = '\0';
      i = strlen(sys_config);
      if(i && (sys_config[i - 1] != '/'))
        strcat(sys_config, "/");
      strcat(sys_config, "../etc/" SYSTEM_CONF_FILE);
      sys_config[PATH_MAX - 1] = '\0';
      if(access(sys_config, F_OK) == 0) {
        sys_config_found = true;
      } else {
        // 2. Check <dirpath of executable>/avrdude.conf
        strcpy(sys_config, executable_dirpath);
        sys_config[PATH_MAX - 1] = '\0';
        i = strlen(sys_config);
        if(i && (sys_config[i - 1] != '/'))
          strcat(sys_config, "/");
        strcat(sys_config, SYSTEM_CONF_FILE);
        sys_config[PATH_MAX - 1] = '\0';
        if(access(sys_config, F_OK) == 0) {
          sys_config_found = true;
        }
      }
    }
    if(!sys_config_found) {
      // 3. Check CONFIG_DIR/avrdude.conf

#if defined(WIN32)
      win_set_path(sys_config, sizeof sys_config, SYSTEM_CONF_FILE);
#else
      strcpy(sys_config, CONFIG_DIR);
      i = strlen(sys_config);
      if(i && (sys_config[i - 1] != '/'))
        strcat(sys_config, "/");
      strcat(sys_config, SYSTEM_CONF_FILE);
#endif

      if(access(sys_config, F_OK) == 0) {
        sys_config_found = true;
      }
    }
  }
  // Debug output
  msg_trace2("sys_config = %s\n", sys_config);
  msg_trace2("sys_config_found = %s\n", sys_config_found? "true": "false");
  msg_trace2("\n");

  if(quell_progress == 0)
    terminal_setup_update_progress();

  // Print out an identifying string so folks can tell what version they are running
  pmsg_notice("%s version %s\n", progname, AVRDUDE_FULL_VERSION);
  pmsg_notice("Copyright see https://github.com/avrdudes/avrdude/blob/main/AUTHORS\n\n");

  if(*sys_config) {
    char *real_sys_config = realpath(sys_config, NULL);

    if(real_sys_config) {
      pmsg_notice("system wide configuration file is %s\n", real_sys_config);
    } else
      pmsg_warning("cannot determine realpath() of config file %s: %s\n", sys_config, strerror(errno));

    rc = read_config(real_sys_config);
    if(rc) {
      pmsg_error("unable to process system wide configuration file %s\n", real_sys_config);
      exit(1);
    }
    mmt_free(real_sys_config);
  }

  if(usr_config[0] != 0 && !no_avrduderc) {
    int ok = (rc = stat(usr_config, &sb)) >= 0 && (sb.st_mode & S_IFREG);

    pmsg_notice("user configuration file %s%s%s\n", ok? "is ": "", usr_config,
      rc < 0? " does not exist": !(sb.st_mode & S_IFREG)? " is not a regular file, skipping": "");

    if(ok) {
      rc = read_config(usr_config);
      if(rc) {
        pmsg_error("unable to process user configuration file %s\n", usr_config);
        exit(1);
      }
    }
  }

  if(!str_eq(avrdude_conf_version, AVRDUDE_FULL_VERSION)) {
    pmsg_warning("system wide configuration file version (%s)\n", avrdude_conf_version);
    imsg_warning("does not match Avrdude build version (%s)\n", AVRDUDE_FULL_VERSION);
  }

  if(lsize(additional_config_files) > 0) {
    LNODEID ln1;
    const char *p = NULL;

    for(ln1 = lfirst(additional_config_files); ln1; ln1 = lnext(ln1)) {
      p = ldata(ln1);
      pmsg_notice("additional configuration file is %s\n", p);

      rc = read_config(p);
      if(rc) {
        pmsg_error("unable to process additional configuration file %s\n", p);
        exit(1);
      }
    }
  }

  // Sort memories of all parts in canonical order
  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1))
    if((p = ldata(ln1))->mem)
      lsort(p->mem, avr_mem_cmp);

  // Set bitclock from configuration files unless changed by command line
  if(default_bitclock > 0 && bitclock == 0.0) {
    bitclock = default_bitclock;
  }

  if(!(pgmid && *pgmid) && *default_programmer)
    pgmid = cache_string(default_programmer);

  // Developer options to print parts and/or programmer entries of avrdude.conf
  int dev_opt_c = dev_opt(pgmid);       // -c <wildcard>/[duASsrtiBUPTIJWHQ]
  int dev_opt_p = dev_opt(partdesc);    // -p <wildcard>/[cdoASsrw*tiBUPTIJWHQ]

  if(dev_opt_c || dev_opt_p) {  // See -c/h and or -p/h
    dev_output_pgm_part(dev_opt_c, pgmid, dev_opt_p, partdesc);
    exit(0);
  }

  PROGRAMMER *dry = locate_programmer(programmers, "dryrun");

  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
    AVRPART *p = ldata(ln1);

    for(LNODEID ln2 = lfirst(programmers); ln2; ln2 = lnext(ln2)) {
      PROGRAMMER *pgm = ldata(ln2);

      if(!is_programmer(pgm))
        continue;
      const char *pnam = pgm->id? ldata(lfirst(pgm->id)): "???";
      int pm = pgm->prog_modes & p->prog_modes;

      if((pm & (pm - 1)) && !str_eq(pnam, "dryrun") && !(dry && pgm->initpgm == dry->initpgm))
        pmsg_warning("%s and %s share multiple modes (%s)\n", pnam, p->desc, avr_prog_modes(pm));
    }
  }

  if(port) {
    if(str_eq(port, "?s")) {
      list_available_serialports(programmers);
      exit(0);
    } else if(str_eq(port, "?sa")) {
      lmsg_error("Valid serial adapters are:\n");
      list_serialadapters(stderr, "  ", programmers);
      exit(0);
    }
  }

  if(partdesc) {
    if(str_eq(partdesc, "?")) {
      if(pgmid && *pgmid && explicit_c) {
        PROGRAMMER *pgm = locate_programmer_starts_set(programmers, pgmid, &pgmid, NULL);

        if(!pgm || !is_programmer(pgm)) {
          programmer_not_found(pgmid, pgm, NULL);
          exit(1);
        }
        msg_error("\nValid parts for programmer %s are:\n", pgmid);
        list_parts(stderr, "  ", part_list, pgm->prog_modes);
      } else {
        msg_error("\nValid parts are:\n");
        list_parts(stderr, "  ", part_list, ~0);
      }
      msg_error("\n");
      exit(1);
    }
  }

  if(pgmid) {
    if(str_eq(pgmid, "?")) {
      if(partdesc && *partdesc) {
        AVRPART *p = locate_part(part_list, partdesc);

        if(!p) {
          part_not_found(partdesc);
          exit(1);
        }
        msg_error("\nValid programmers for part %s are:\n", p->desc);
        list_programmers(stderr, "  ", programmers, p->prog_modes);
      } else {
        msg_error("\nValid programmers are:\n");
        list_programmers(stderr, "  ", programmers, ~0);
      }
      msg_error("\n");
      exit(1);
    }

    if(str_eq(pgmid, "?type")) {
      msg_error("\nValid programmer types are:\n");
      list_programmer_types(stderr, "  ");
      msg_error("\n");
      exit(1);
    }
  }

  msg_notice("\n");

  if(!pgmid || !*pgmid) {
    programmer_not_found(NULL, NULL, NULL);
    exit(1);
  }

  p = partdesc && *partdesc? locate_part(part_list, partdesc): NULL;
  pgm = locate_programmer_starts_set(programmers, pgmid, &pgmid, p);
  if(pgm == NULL || !is_programmer(pgm)) {
    programmer_not_found(pgmid, pgm, p);
    exit(1);
  }

  if(p && !(p->prog_modes & pgm->prog_modes)) {
    pmsg_error("-c %s cannot program %s for lack of a common programming mode\n", pgmid, p->desc);
    if(!ovsigck) {
      imsg_error("use -F to override this check\n");
      exit(1);
    }
  }

  if(pgm->initpgm) {
    pgm->initpgm(pgm);
  } else {
    msg_error("\n");
    pmsg_error("cannot initialize the programmer\n\n");
    exit(1);
  }

  if(pgm->setup) {
    pgm->setup(pgm);
  }
  if(pgm->teardown) {
    atexit(exithook);
  }

  if(lsize(extended_params) > 0) {
    if(pgm->parseextparams == NULL) {
      for(LNODEID ln = lfirst(extended_params); ln; ln = lnext(ln)) {
        const char *extended_param = ldata(ln);

        if(str_eq(extended_param, "help")) {
          msg_error("%s -c %s extended options:\n", progname, pgmid);
          msg_error("  -x help  Show this help menu and exit\n");
          exit(0);
        } else
          pmsg_error("programmer does not support extended parameter -x %s, option ignored\n", extended_param);
      }
    } else {
      int rc = pgm->parseextparams(pgm, extended_params);

      if(rc == LIBAVRDUDE_EXIT)
        exit(0);
      if(rc < 0) {
        pmsg_error("unable to parse list of -x parameters\n");
        exit(1);
      }
    }
  }

  if(port == NULL) {
    switch(pgm->conntype) {
    case CONNTYPE_PARALLEL:
      port = mmt_strdup(default_parallel);
      break;

    case CONNTYPE_SERIAL:
      port = mmt_strdup(default_serial);
      break;

    case CONNTYPE_USB:
      port = mmt_strdup(DEFAULT_USB);
      break;

    case CONNTYPE_SPI:

#ifdef HAVE_LINUXSPI
      port = mmt_strdup(*default_spi? default_spi: "unknown");
#else
      port = mmt_strdup("unknown");
#endif

      break;

    case CONNTYPE_LINUXGPIO:
      port = mmt_strdup(default_linuxgpio);
      break;

    default:
      port = mmt_strdup("unknown");
      break;

    }
  }

  int is_dryrun = str_eq(pgm->type, "dryrun") || (dry && pgm->initpgm == dry->initpgm);

  if((port[0] == 0 || str_eq(port, "unknown")) && !is_dryrun) {
    msg_error("\n");
    pmsg_error("no port has been specified on the command line or in the config file;\n");
    imsg_error("specify a port using the -P option and try again\n");
    exit(1);
  }

  /*
   * Divide a serialadapter port string into tokens separated by colons.
   * There are two ways such a port string can be presented:
   *   1) -P <serialadapter>[:<sernum>]
   *   2) -P usb:<usbvid>:<usbpid>[:<sernum>]
   * In either case the serial number is optional. The USB vendor and
   * product ids are hexadecimal numbers.
   */
  bool print_ports = true;
  SERIALADAPTER *ser = NULL;

  if(pgm->conntype == CONNTYPE_SERIAL) {
    char *portdup = mmt_strdup(port);
    char *port_tok[4], *tok = portdup;

    for(int t = 0, maxt = str_starts(portdup, DEFAULT_USB ":")? 4: 2; t < 4; t++) {
      char *save = tok && t < maxt? tok: "";

      if(t < maxt - 1 && tok && (tok = strchr(tok, ':')))
        *tok++ = 0;
      port_tok[t] = mmt_strdup(save);
    }
    mmt_free(portdup);

    // Use libserialport to find the actual serial port
    ser = locate_programmer(programmers, port_tok[0]);
    if(is_serialadapter(ser)) {

#ifdef HAVE_LIBSERIALPORT
      int rv = setport_from_serialadapter(&port, ser, port_tok[1]);

      if(rv == -1) {
        pmsg_warning("serial adapter %s", port_tok[0]);
        if(port_tok[1][0])
          msg_warning(" with serial number %s", port_tok[1]);
        else if(ser->usbsn && ser->usbsn[0])
          msg_warning(" with serial number %s", ser->usbsn);
        msg_warning(" not connected to host\n");
      } else if(rv == -2)
        print_ports = false;
      if(rv)
        ser = NULL;
#endif
    } else if(str_eq(port_tok[0], DEFAULT_USB)) {
      // Port or usb:[vid]:[pid]
      int vid, pid;

      if(sscanf(port_tok[1], "%x", &vid) > 0 && sscanf(port_tok[2], "%x", &pid) > 0) {
        int rv = setport_from_vid_pid(&port, vid, pid, port_tok[3]);

        if(rv == -1) {
          if(port_tok[3][0])
            pmsg_warning("serial adapter with USB VID %s and PID %s and serial number %s not connected\n", port_tok[1],
              port_tok[2], port_tok[3]);
          else
            pmsg_warning("serial adapter with USB VID %s and PID %s not connected\n", port_tok[1], port_tok[2]);
        } else if(rv == -2)
          print_ports = false;
      }
    }
    for(int i = 0; i < 4; i++)
      mmt_free(port_tok[i]);
    if(touch_1200bps && touch_serialport(&port, 1200, touch_1200bps) < 0)
      goto skipopen;
  }

  // Open the programmer
  if(verbose > 0) {
    if(!is_dryrun)
      pmsg_notice("using port            : %s\n", port);
    pmsg_notice("using programmer      : %s\n", pgmid);
  }

  if(baudrate && !pgm->baudrate && !default_baudrate) { // None set
    pmsg_notice("setting baud rate     : %d\n", baudrate);
    pgm->baudrate = baudrate;
  } else if(baudrate && ((pgm->baudrate && pgm->baudrate != baudrate)
      || (!pgm->baudrate && default_baudrate != baudrate))) {
    pmsg_notice("overriding baud rate  : %d\n", baudrate);
    pgm->baudrate = baudrate;
  } else if(!pgm->baudrate && default_baudrate) {
    pmsg_notice("default baud rate     : %d\n", default_baudrate);
    pgm->baudrate = default_baudrate;
  } else if(ser && ser->baudrate) {
    pmsg_notice("serial baud rate      : %d\n", ser->baudrate);
    pgm->baudrate = ser->baudrate;
  } else if(pgm->baudrate != 0)
    pmsg_notice("programmer baud rate  : %d\n", pgm->baudrate);

  if(bitclock != 0.0) {
    pmsg_notice("setting bit clk period: %.1f us\n", bitclock);
    pgm->bitclock = bitclock*1e-6;
  }

  if(ispdelay != 0) {
    pmsg_notice("setting ISP clk delay : %3i us\n", ispdelay);
    pgm->ispdelay = ispdelay;
  }

  rc = pgm->open(pgm, port);
  if(rc < 0) {
    if(rc == LIBAVRDUDE_EXIT) {
      exitrc = 0;
      goto main_exit;
    }

    pmsg_error("unable to open port %s for programmer %s\n", port, pgmid);
  skipopen:
    if(print_ports && pgm->conntype == CONNTYPE_SERIAL) {

#ifdef HAVE_LIBSERIALPORT
      list_available_serialports(programmers);
      if(touch_1200bps == 1)
        pmsg_info("alternatively, try -rr or -rrr for longer delays\n");
#endif
    }
    exitrc = 1;
    pgm->ppidata = 0;           // Clear all bits at exit
    goto main_exit;
  }
  is_open = 1;

  if(partdesc == NULL) {
    part_not_found(NULL);
    exitrc = 1;
    goto main_exit;
  }

  p = locate_part(part_list, partdesc);
  if(p == NULL) {
    part_not_found(partdesc);
    exitrc = 1;
    goto main_exit;
  }

  if(exitspecs != NULL) {
    if(pgm->parseexitspecs == NULL) {
      pmsg_warning("-E option not supported by this programmer type\n");
      exitspecs = NULL;
    } else {
      int rc = pgm->parseexitspecs(pgm, exitspecs);

      if(rc == LIBAVRDUDE_EXIT)
        exit(0);
      if(rc < 0) {
        pmsg_error("unable to parse list of -E parameters\n");
        exit(1);
      }
    }
  }

  if(avr_initmem(p) != 0) {
    msg_error("\n");
    pmsg_error("unable to initialize memories\n");
    exitrc = 1;
    goto main_exit;
  }

  if(verbose > 0) {
    if((str_eq(pgm->type, "avr910"))) {
      imsg_notice("avr910_devcode (avrdude.conf) : ");
      if(p->avr910_devcode)
        msg_notice("0x%02x\n", (uint8_t) p->avr910_devcode);
      else
        msg_notice("none\n");
    }
  }

  /*
   * Now that we know which part we are going to program, locate any -U options
   * using the default memory region, fill in the device-dependent default
   * region name ("application" for Xmega parts or "flash" otherwise) and check
   * for basic problems with memory names or file access with a view to exit
   * before programming.
   */
  int doexit = 0;

  for(ln = lfirst(updates); ln; ln = lnext(ln)) {
    upd = ldata(ln);
    if(upd->memstr == NULL && upd->cmdline == NULL) {
      const char *mtype = is_pdi(p)? "application": "flash";

      pmsg_notice2("defaulting memstr in -U %c:%s option to \"%s\"\n",
        (upd->op == DEVICE_READ)? 'r': (upd->op == DEVICE_WRITE)? 'w': 'v', upd->filename, mtype);
      upd->memstr = mmt_strdup(mtype);
    }
    rc = update_dryrun(p, upd);
    if(rc && rc != LIBAVRDUDE_SOFTFAIL)
      doexit = 1;
  }
  if(doexit) {
    exitrc = 1;
    goto main_exit;
  }

  if(calibrate) {
    // Perform an RC oscillator calibration as outlined in appnote AVR053
    if(pgm->perform_osccal == 0) {
      pmsg_error("programmer does not support RC oscillator calibration\n");
      exitrc = 1;
    } else {
      pmsg_notice2("performing RC oscillator calibration\n");
      exitrc = pgm->perform_osccal(pgm);
    }
    if(exitrc)
      pmsg_error("RC calibration unsuccesful\n");
    else
      pmsg_notice("calibration value is now stored in EEPROM at address 0\n");

    goto main_exit;
  }

  if(verbose > 0 && quell_progress < 2) {
    avr_display(stderr, pgm, p, progbuf, verbose);
    msg_notice2("\n");
    programmer_display(pgm, progbuf);
  }

  lmsg_info("");

  exitrc = 0;

  // Enable the programmer
  pgm->enable(pgm, p);

  // Turn off all the status LEDs and reset LED states
  led_set(pgm, LED_BEG);

  // Initialize the chip in preparation for accepting commands
  init_ok = (rc = pgm->initialize(pgm, p)) >= 0;
  if(!init_ok) {
    if(rc == LIBAVRDUDE_EXIT) {
      exitrc = 0;
      goto main_exit;
    }
    pmsg_error("initialization failed  (rc = %d)\n", rc);
    if(rc == -2)
      imsg_error(" - the programmer ISP clock is too fast for the target\n");
    else
      imsg_error(" - double check the connections and try again\n");

    if(str_eq(pgm->type, "serialupdi") || str_eq(pgm->type, "SERBB"))
      imsg_error(" - use -b to set lower baud rate, e.g. -b %d\n", baudrate? baudrate/2: 57600);
    else
      imsg_error(" - use -B to set lower the bit clock frequency, e.g. -B 125kHz\n");

    if(str_starts(pgm->type, "pickit5"))
      imsg_error(" - reset the programmer by unplugging it");

    if(!ovsigck) {
      imsg_error(" - use -F to override this check\n");
      exitrc = 1;
      goto main_exit;
    }
  }

  // Indicate programmer is ready
  led_set(pgm, LED_RDY);

  msg_notice("\n");
  pmsg_notice("AVR device initialized and ready to accept instructions\n");

  /*
   * Let's read the signature bytes to make sure there is at least a chip on
   * the other end that is responding correctly.  A check against
   * 0xffffff/0x000000 should ensure that the signature bytes are valid.
   */
  if(!is_awire(p)) {            // Not AVR32
    int attempt = 0;
    int waittime = 10000;       // 10 ms

  sig_again:
    usleep(waittime);
    if(init_ok) {
      rc = avr_signature(pgm, p);
      if(rc == LIBAVRDUDE_EXIT) {
        exitrc = 0;
        goto main_exit;
      }
      if(rc != LIBAVRDUDE_SUCCESS) {
        if(rc == LIBAVRDUDE_SOFTFAIL && is_updi(p) && attempt < 1) {
          attempt++;
          if(erase) {
            erase = 0;
            if(uflags & UF_NOWRITE) {
              pmsg_warning("conflicting -e and -n options specified, NOT erasing chip\n");
            } else {
              pmsg_info("unlocking the chip");
              exitrc = avr_unlock(pgm, p);
              if(exitrc)
                goto main_exit;
              msg_info(" and trying again\n");
              goto sig_again;
            }
          }
          if(!ovsigck) {
            pmsg_error("double check chip or use -F to override this check\n");
            exitrc = 1;
            goto main_exit;
          }
        }
        pmsg_error("unable to read signature data (rc = %d)\n", rc);
        if(!ovsigck) {
          imsg_error("use -F to override this check\n");
          exitrc = 1;
          goto main_exit;
        }
      }
    }

    sig = avr_locate_signature(p);
    if(sig == NULL)
      pmsg_warning("signature memory not defined for device %s\n", p->desc);
    else {
      const char *mculist = str_ccmcunames_signature(sig->buf, pgm->prog_modes);

      if(!*mculist) {           // No matching signatures?
        if(is_updi(p)) {        // UPDI parts have different(!) offsets for signature
          int k, n = 0;         // Gather list of known different signature offsets
          unsigned myoff = sig->offset, offlist[10];

          for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
            AVRMEM *m = avr_locate_signature(ldata(ln1));

            if(m && m->offset != myoff) {
              for(k = 0; k < n; k++)
                if(m->offset == offlist[k])
                  break;
              if(k == n && k < (int) (sizeof offlist/sizeof *offlist))
                offlist[n++] = m->offset;
            }
          }
          // Now go through the list of other(!) sig offsets and try these
          for(k = 0; k < n; k++) {
            sig->offset = offlist[k];
            if(avr_signature(pgm, p) >= 0)
              if(*(mculist = str_ccmcunames_signature(sig->buf, pgm->prog_modes)))
                break;
          }
          sig->offset = myoff;
        }
      }

      int ff = 1, zz = 1;

      for(i = 0; i < sig->size; i++) {
        if(sig->buf[i] != 0xff)
          ff = 0;
        if(sig->buf[i] != 0x00)
          zz = 0;
      }
      bool signature_matches = sig->size >= 3 && !memcmp(sig->buf, p->signature, 3);
      int showsig = !signature_matches || ff || zz || verbose > 0;

      if(showsig)
        pmsg_info("device signature =%s", str_cchex(sig->buf, sig->size, 1));
      if(*mculist && showsig)
        msg_info(" (%s)", is_dryrun? p->desc: mculist);

      if(ff || zz) {            // All three bytes are 0xff or all three bytes are 0x00
        if(++attempt < 3) {
          waittime *= 5;
          msg_info(" (retrying)\n");
          goto sig_again;
        }
        msg_info("\n");
        pmsg_error("invalid device signature\n");
        if(!ovsigck) {
          pmsg_error("expected signature for %s is%s\n", p->desc, str_cchex(p->signature, 3, 1));
          imsg_error("  - double check connections and try again, or use -F to carry on regardless\n");
          exitrc = 1;
          goto main_exit;
        }
      } else if(showsig) {
        msg_info("\n");
      }

      if(!signature_matches) {
        if(ovsigck) {
          pmsg_warning("expected signature for %s is%s\n", p->desc, str_cchex(p->signature, 3, 1));
        } else {
          pmsg_error("expected signature for %s is%s\n", p->desc, str_cchex(p->signature, 3, 1));
          imsg_error("  - double check chip or use -F to carry on regardless\n");
          exitrc = 1;
          goto main_exit;
        }
      }
    }
  }

  if(uflags & UF_AUTO_ERASE) {
    if((p->prog_modes & (PM_PDI | PM_UPDI)) && pgm->page_erase && lsize(updates) > 0) {
      for(ln = lfirst(updates); ln; ln = lnext(ln)) {
        upd = ldata(ln);
        if(upd->memstr && upd->op == DEVICE_WRITE && memlist_contains_flash(upd->memstr, p)) {
          cx->avr_disableffopt = 1;     // Must write full flash file including trailing 0xff
          pmsg_notice("NOT erasing chip as page erase will be used for new flash%s contents;\n",
            avr_locate_bootrow(p)? "/bootrow": "");
          imsg_notice("unprogrammed flash contents remains: use -e for an explicit chip-erase\n");
          break;
        }
      }
    } else {
      uflags &= ~UF_AUTO_ERASE;
      for(ln = lfirst(updates); ln; ln = lnext(ln)) {
        upd = ldata(ln);
        if(upd->cmdline && *str_ltrim(upd->cmdline) && str_starts("erase", str_ltrim(upd->cmdline)))
          break;                // -T erase already erases the chip: no auto-erase needed

        if(upd->cmdline || (upd->memstr &&      // Might be reading flash?
            (upd->op == DEVICE_READ || upd->op == DEVICE_VERIFY) && memlist_contains_flash(upd->memstr, p)))
          flashread = 1;

        if(upd->memstr && upd->op == DEVICE_WRITE && memlist_contains_flash(upd->memstr, p)) {
          if(flashread) {
            pmsg_info("NOT auto-erasing chip as flash might need reading before writing to it\n");
          } else {
            erase = 1;
            pmsg_notice("auto-erasing chip as flash memory needs programming (-U %s:w:...)\n", upd->memstr);
            imsg_notice("specify the -D option to disable this feature\n");
          }
          break;
        }
      }
    }
  }

  if(init_ok && erase) {
    /*
     * Erase the chip's flash and eeprom memories, this is required before the
     * chip can accept new programming
     */
    if(uflags & UF_NOWRITE) {
      if(explicit_e)
        pmsg_warning("conflicting -e and -n specified, NOT erasing chip\n");
      else
        pmsg_notice("-n specified, NOT erasing chip\n");
    } else {
      exitrc = avr_chip_erase(pgm, p);
      if(exitrc == LIBAVRDUDE_SOFTFAIL) {
        pmsg_notice("delaying chip erase until first -U upload to flash\n");
        ce_delayed = 1;
        exitrc = 0;
      } else if(exitrc) {
        pmsg_error("chip erase failed\n");
        goto main_exit;
      } else
        pmsg_notice("erased chip\n");
    }
  }

  if(!init_ok && !ovsigck) {    // Bail out on failed initialisation unless -F was given
    exitrc = 1;
    goto main_exit;
  }

  int wrmem = 0, terminal = 0;

  if(lsize(updates) <= 1)
    uflags |= UF_NOHEADING;
  for(ln = lfirst(updates); ln; ln = lnext(ln)) {
    const AVRMEM *m;

    upd = ldata(ln);
    if(upd->cmdline && wrmem) { // Invalidate cache if device was written to
      wrmem = 0;
      pgm->reset_cache(pgm, p);
    } else if(!upd->cmdline) {  // Flush cache before any device memory access
      pgm->flush_cache(pgm, p);
      wrmem |= upd->op == DEVICE_WRITE;
    }
    if((uflags & UF_NOWRITE) && upd->cmdline && !terminal++)
      pmsg_warning("the terminal ignores option -n, that is, it writes to the device\n");
    rc = do_op(pgm, p, upd, uflags);
    if(rc && rc != LIBAVRDUDE_SOFTFAIL) {
      exitrc = 1;
      break;
    } else if(rc == 0 && upd->op == DEVICE_WRITE && (m = avr_locate_mem(p, upd->memstr)) && mem_is_in_flash(m))
      ce_delayed = 0;           // Redeemed chip erase promise
  }
  pgm->flush_cache(pgm, p);

  if(pgm->end_programming)
    if(pgm->end_programming(pgm, p) < 0)
      pmsg_error("could not end programming, aborting\n");

main_exit:

  // Program complete
  if(is_open) {
    // Clear rdy LED and summarise interaction in err, pgm and vfy LEDs
    led_set(pgm, LED_END);
    pgm->powerdown(pgm);
    pgm->disable(pgm);
    pgm->close(pgm);
  }

  if(cx->usb_access_error) {
    pmsg_info("\nUSB access errors detected; this could have many reasons; if it is\n"
      "USB permission problems, avrdude is likely to work when run as root\n"
      "but this is not good practice; instead you might want to\n");

#if 0 && !defined(WIN32)
    DIR *dir;

    if((dir = opendir("/etc/udev/rules.d"))) {  // Linux udev land
      closedir(dir);
      imsg_info("run the command below to show udev rules recitifying USB access\n" "$ %s -c %s/u\n", progname, pgmid);
    } else
#endif

      imsg_info("check out USB port permissions on your OS and set them correctly\n");
  }

  msg_info("\n");
  pmsg_info("%s done.  Thank you.\n", progname);

  return ce_delayed? 1: exitrc;
}
