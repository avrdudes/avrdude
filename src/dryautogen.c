/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2026 Stefan Rueger <stefan.rueger@urclocks.com>
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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>

#include "avrdude.h"
#include <libavrdude.h>

#define Return(...) do { \
  if(silent) \
    return -1; \
  if(verbose > 0 || rethelp) \
    autogen_help(); \
  pmsg_error("(test) "); \
  msg_error(__VA_ARGS__); \
  msg_error("\n"); \
  return -1; \
} while (0)

static void autogen_help(void) {
  msg_error("%s",
    "Test file features are specified in an underscore-separated list of the\n"
    "filename dry:[<part>][_<memlist>]{_<feature>}[.hex] in arbitrary order,\n"
    "eg, \"dry:random=1_holes\". Features are:\n"
    "  init           Initialise memories with human-readable patterns\n"
    "  init=<n>       Shortcut for init_seed=<n>\n"
    "  random         Initialise memories with random code/values\n"
    "  random=<n>     Shortcut for random_seed=<n>\n"
    "  holes          Put holes into the code of flash, eeprom, userrow and bootrow\n"
    "  seed=<n>       Seed random number generator with n > 0, default time(NULL)\n"
    "  save=file.hex  Save test file with chosen name\n"
    "  save           Save test file with canonical file name\n"
    "  help           Show this help message and return\n"
    "Notes:\n"
    "  Init and random randomly configure flash wrt code, data and boot sections.\n"
    "  Generated Flash code will be benign, ie, not acceess I/O, SRAM or flash.\n"
    "  Choose, eg, seed=1 for reproducible flash configuration and output.\n"
    "  The default <memlist> comprises of all writeable memories and signature.\n"
  );
}

typedef struct {
  const char *mcu;              // MCU id
  char *memlist;                // Optional memory list
  int init;                     // Initialise memories with human-readable patterns
  int random;                   // Random initialisation of memories
  int seed;                     // Seed for random number generator
  int holes;                    // Generate holes in flash, eeprom, userrow and bootrow
  int save;
  FILEFMT savefmt;
  char *savefname;
} Testparams;

static char *dry_filename(const Testparams *ppp, const char *ext) {
  char ret[512], *p = ret;

  sprintf(p, "dry_%.64s", ppp->mcu);
  if(!str_eq("test", ppp->memlist))
    sprintf((p+=strlen(p)), "_%.256s", ppp->memlist);

  if(ppp->random && ppp->seed)
    sprintf((p+=strlen(p)), "_random=%d", ppp->seed);
  else if(ppp->random)
    strcpy((p+=strlen(p)), "_random");
  else if(ppp->init && ppp->seed)
    sprintf((p+=strlen(p)), "_init=%d", ppp->seed);
  else if(ppp->init)
    strcpy((p+=strlen(p)), "_init");
  else if(ppp->seed)
    sprintf((p+=strlen(p)), "_seed=%d", ppp->seed);

  if(ppp->holes)
    strcpy((p+=strlen(p)), "_holes");

  sprintf((p+=strlen(p)), "%.32s", ext);

  return mmt_strdup(ret);
}

static int dryautogen_parse(const AVRPART *part, char *testname, Testparams *ppp, int silent) {
  char *p, *q, *tok;
  int beyond = 0, rethelp = 0;

  memset(ppp, 0, sizeof *ppp);
  ppp->savefmt = FMT_IHXC;      // Saved file defaults to Intel Hex with comments

  ppp->mcu = part->id;

  // Quick and dirty attempt at gleaning a help further down the line
  rethelp = (q = strstr(testname, "_help")) && q[-1] != '\\' && strchr("._", q[5]);

  if(!str_starts(testname, "dry:"))
    Return("%s does not start with dry:", testname);

  // Remove hex unless last para is save=...; first find last unescaped underscore
  if((p = strrchr(testname, '_'))) {
    do {
      for(q = p--; *p == '\\'; --p)
        continue;
      if((q-p) & 1)             // Even number of escape chars
        break;
      while(*p != '_' && p > testname + 6)
        --p;
    } while(*p == '_');         // Check again for unescaped underscore
  }
  if(!p || !str_starts(q, "_save="))
    if(str_caseends(testname, ".hex"))
      testname[strlen(testname)-4] = 0;

  AVRPART *testp = NULL;
  p = testname + 4;
  while(*(tok = str_nexttok(p, "_", &p))) {
    if(beyond++ < 2) {          // Accept part/memlist only within first two elements
      if(beyond == 1 && !testp && (testp = locate_part(part_list, tok))) {
        if(!str_eq(ppp->mcu, testp->id))
          Return("-p %s part is incompatible with dry:%s name", part->desc, tok);
        continue;
      }
      if((beyond == 1 && !testp) || (beyond == 2 && testp)) {
        int nmem = 0, ret = 0, bakverb = verbose;
        verbose = -123;
        mmt_free(memory_list(tok, locate_programmer(programmers, pgmid), part, &nmem, NULL, &ret));
        verbose = bakverb;
        if(nmem > 0 && ret == 0) {
          ppp->memlist = mmt_strdup(tok);
          continue;
        }
      }
    }

    if(str_eq(tok, "init")) {
      ppp->init = 1;
      continue;
    }
    if(str_eq(tok, "random")) {
      ppp->random = 1;
      continue;
    }
    if(str_eq(tok, "holes")) {
      ppp->holes = 1;
      continue;
    }
    if(str_starts(tok, "init=") || str_starts(tok, "random=") || str_starts(tok, "seed=")) {
      const char *errptr;
      int seed = str_int(strchr(tok, '=') + 1, STR_INT32, &errptr);

      if(errptr)
        Return("cannot parse %s seed value", tok);
      if(seed < 0)
        Return("seed value %d in %s must not be negative", seed, tok);

      ppp->seed = seed;
      if(str_starts(tok, "init"))
        ppp->init = 1;
      else if(str_starts(tok, "random"))
        ppp->random = 1;

      continue;
    }

    if(str_starts(tok, "save")) {
      ppp->save = 1;
      if(tok[4] == '=') {
        ppp->savefname = mmt_strdup(tok+5);
        cfg_unescape(ppp->savefname, ppp->savefname);
        size_t fnlen = strlen(ppp->savefname);
        if(fnlen > 2 && ppp->savefname[fnlen-2] == ':') {
          ppp->savefname[fnlen-2] = 0;
          FILEFMT sfmt = fileio_format_with_errmsg(ppp->savefname[fnlen - 1], "");
          if(sfmt == FMT_ERROR)
            return -1;
          if(sfmt != FMT_AUTO)
            ppp->savefmt = sfmt;
        }
        if(!*ppp->savefname) {
          mmt_free(ppp->savefname);
          ppp->savefname = NULL;
        }
      }
      continue;
    }

    if(str_eq(tok, "help")) {
      if(!silent)
        autogen_help();
      return -1;
    }

    Return("unable to parse _%s segment", tok);
  }

  if(ppp->init && ppp->random)
    Return("init and random paramters are mutually exclusive");

  if(!ppp->memlist)
    ppp->memlist = mmt_strdup("test");

  return 0;
}

int dry_has_contents(const AVRPART *part, const char *filename) {
  Testparams pp;
  char *testname = mmt_strdup(filename);

  // Silently parse the dry:... string
  int ret = dryautogen_parse(part, testname, &pp, 1) == 0;

  mmt_free(testname);
  mmt_free(pp.memlist);
  mmt_free(pp.savefname);

  return ret;
}

static void addseg(int *np, Segment **segp, int addr, int len) {
  if(*np%32 == 0)
    *segp = (Segment *) mmt_realloc(*segp, sizeof**segp * (*np + 32));
  (*segp)[*np].addr = addr;
  (*segp)[*np].len = len;
  (*np)++;
}

// Set memory to autogenerated test contents as if read from a file
int dryautogen(const AVRPART *part, const AVRMEM *mem, const char *filename) {
  int ret = -1;
  char *testname = mmt_strdup(filename);
  AVRPART *dp = NULL;
  AVRMEM *any = NULL;
  int n_segs = 0;
  Segment *segs = NULL;

  Testparams pp;
  if(dryautogen_parse(part, testname, &pp, 0) < 0)
    goto done;

  if(!(dp = dryrun_part(part->id, NULL, pp.init, pp.random, pp.holes, pp.seed))) {
    pmsg_error("(test) cannot autogenerate %s", filename);
    goto done;
  }

  // Assemble a multi-memory memory any with memories from the list
  any = fileio_any_memory("any:dryautogen");
  memset(any->buf, 0xff, any->size);
  memset(any->tags, 0, any->size);
  int nm = 0, softfail = 0;
  AVRMEM **ml = memory_list(pp.memlist, locate_programmer(programmers, pgmid), dp, &nm, &softfail, NULL);
  for(int mi = 0; mi < nm; mi++) {
    AVRMEM *m = ml[mi];

    int offset = fileio_mem_offset(dp, m);
    if(offset == (int) ~0U) {
      pmsg_error("(test) unknown multimem offset for memory %s\n", m->desc);
      goto done;
    }
    if(offset < 0 || offset + m->size > any->size) {
      pmsg_error("(test) multimem segment for memory %s is out of bounds\n", m->desc);
      goto done;
    }
    unsigned char *b = any->buf + offset, *t = any->tags + offset;

    memcpy(b, m->buf, m->size);
    memset(t, TAG_ALLOCATED, m->size);
    if(m->size < 64 || mem_is_readonly(m) || mem_is_io(m) || mem_is_sram(m)) {
      addseg(&n_segs, &segs, offset, m->size);
    } else {
      // Zap tags in any-memory that are holes, ie, sequences of two or more 0xff
      for(int j, a = 0, i = 0, s = m->size; i < s; ) {
        while(i < s && (i == s-1 || b[i] != 0xff || b[i+1] != 0xff))
          i++;
        if((i&1) && !pp.holes && mem_is_in_flash(m) && i < s)
          i++;
        if(i - a)
          addseg(&n_segs, &segs, offset + a, i - a);

        for(j = i; j < s; j++)
          if(b[j] != 0xff)
            break;
        if((j&1) && j > i && !pp.holes && mem_is_in_flash(m))
          j--;
        a = j;                  // Start of next contents streak
        if(i == 0 && j >= s) {  // Complete memory is a hole, record at least one byte
          i++;
          addseg(&n_segs, &segs, offset, 1);
        }
        memset(t + i, 0, j - i); // Clear tags of 0xff sequence
        i = j;
      }
    }
  }
  const Segment seg = { 0, mem->size };
  ret = fileio_any2mem(dp, mem, &seg, any, any->size);

  if(pp.save) {
    if(!pp.savefname)
      pp.savefname = dry_filename(&pp, ".hex");
    pmsg_notice("writing autogenerated test contents to %s\n", pp.savefname);
    if(n_segs)
      fileio_segments(FIO_WRITE, pp.savefname, pp.savefmt, part, any, n_segs, segs);
  }

done:
  mmt_free(testname);
  mmt_free(segs);
  mmt_free(pp.savefname);
  mmt_free(pp.memlist);
  avr_free_part(dp);
  avr_free_mem(any);

  return ret;
}
