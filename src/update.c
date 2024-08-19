/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005 Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2007 Joerg Wunsch
 * Copyright (C) 2022- Stefan Rueger <stefan.rueger@urclocks.com>
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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <ac_cfg.h>

#include "avrdude.h"
#include "libavrdude.h"

// Is s a multi-memory string (comma-separates list, all, ALL, etc or list subtraction)?
static int is_multimem(const char *s) {
  return str_eq(s, "ALL") || str_eq(s, "all") || str_eq(s, "etc") || strpbrk(s, "-,\\");
}

/*
 * Parsing of [<memory>:<op>:<file>[:<fmt>] | <file>[:<fmt>]]
 *
 * As memory names don't contain colons and the r/w/v operation <op> is a
 * single character, check whether the first two colons sandwich one character.
 * If not, treat the argument as a filename (defaulting to flash write). This
 * allows colons in filenames other than those for enclosing <op> and
 * separating <fmt>, eg, C:/some/file.hex
 */
UPDATE *parse_op(const char *s) {
  // Assume -U <file>[:<fmt>] first
  UPDATE *upd = (UPDATE *) mmt_malloc(sizeof *upd);

  upd->memstr = NULL;           // Defaults to flash or application
  upd->op = DEVICE_WRITE;
  const char *fn = s;

  // Check for <memory>:c: start in which case override defaults
  const char *fc = strchr(s, ':');

  if(fc && fc[1] && fc[2] == ':') {
    if(!strchr("rwv", fc[1])) {
      pmsg_error("invalid I/O mode :%c: in -U %s\n", fc[1], s);
      imsg_error("I/O mode can be r, w or v for read, write or verify device\n");
      mmt_free(upd->memstr);
      mmt_free(upd);
      return NULL;
    }

    upd->memstr = memcpy(mmt_malloc(fc - s + 1), s, fc - s);
    upd->op = fc[1] == 'r'? DEVICE_READ: fc[1] == 'w'? DEVICE_WRITE: DEVICE_VERIFY;
    fn = fc + 3;
  }
  // Autodetect for file reads, and hex (multi-mem)/raw (single mem) for file writes
  upd->format = upd->op != DEVICE_READ? FMT_AUTO: is_multimem(upd->memstr)? FMT_IHXC: FMT_RBIN;

  // Filename: last char is format if the penultimate char is a colon
  size_t len = strlen(fn);

  if(len > 2 && fn[len - 2] == ':') {   // Assume format specified
    upd->format = fileio_format_with_errmsg(fn[len - 1], "");
    if(upd->format == FMT_ERROR) {
      mmt_free(upd->memstr);
      mmt_free(upd);
      return NULL;
    }
    len -= 2;
  }

  upd->filename = memcpy(mmt_malloc(len + 1), fn, len);

  return upd;
}

UPDATE *dup_update(const UPDATE *upd) {
  UPDATE *u = (UPDATE *) mmt_malloc(sizeof *u);

  memcpy(u, upd, sizeof *u);
  u->memstr = upd->memstr? mmt_strdup(upd->memstr): NULL;
  u->filename = mmt_strdup(upd->filename);

  return u;
}

UPDATE *new_update(int op, const char *memstr, int filefmt, const char *fname) {
  UPDATE *u = (UPDATE *) mmt_malloc(sizeof *u);

  u->memstr = mmt_strdup(memstr);
  u->filename = mmt_strdup(fname);
  u->op = op;
  u->format = filefmt;

  return u;
}

UPDATE *cmd_update(const char *cmd) {
  UPDATE *u = (UPDATE *) mmt_malloc(sizeof *u);

  u->cmdline = cmd;

  return u;
}

void free_update(UPDATE *u) {
  if(u) {
    mmt_free(u->memstr);
    mmt_free(u->filename);
    memset(u, 0, sizeof *u);
    mmt_free(u);
  }
}

char *update_str(const UPDATE *upd) {
  if(upd->cmdline)
    return mmt_sprintf("-%c %s", str_eq("interactive terminal", upd->cmdline)? 't': 'T', upd->cmdline);
  return mmt_sprintf("-U %s:%c:%s:%c", upd->memstr,
    upd->op == DEVICE_READ? 'r': upd->op == DEVICE_WRITE? 'w': 'v',
    upd->filename, fileio_fmtchr(upd->format));
}

// Memory statistics considering holes after a file read returned size bytes
int memstats(const AVRPART *p, const char *memstr, int size, Filestats *fsp) {
  AVRMEM *mem = avr_locate_mem(p, memstr);

  if(!mem) {
    pmsg_error("%s %s undefined\n", p->desc, memstr);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return memstats_mem(p, mem, size, fsp);
}

int memstats_mem(const AVRPART *p, const AVRMEM *mem, int size, Filestats *fsp) {
  Filestats ret = { 0 };

  if(!mem->buf || !mem->tags) {
    pmsg_error("%s %s is not set\n", p->desc, mem->desc);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  int pgsize = mem->page_size;

  if(pgsize < 1)
    pgsize = 1;

  if(size < 0 || size > mem->size) {
    pmsg_error("size %d at odds with %s %s size %d\n", size, p->desc, mem->desc, mem->size);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  ret.lastaddr = -1;
  int firstset = 0, insection = 0;

  // Scan all memory
  for(int addr = 0; addr < mem->size;) {
    int pageset = 0;

    // Go page by page
    for(int pgi = 0; pgi < pgsize; pgi++, addr++) {
      if(mem->tags[addr] & TAG_ALLOCATED) {
        if(!firstset) {
          firstset = 1;
          ret.firstaddr = addr;
        }
        ret.lastaddr = addr;
        // Size can be smaller than tags suggest owing to flash trailing-0xff
        if(addr < size) {
          ret.nbytes++;
          if(!pageset) {
            pageset = 1;
            ret.nfill += pgi;
            ret.npages++;
          }
          if(!insection) {
            insection = 1;
            ret.nsections++;
          }
        } else {                // Now beyond size returned by input file read
          ret.ntrailing++;
          if(pageset)
            ret.nfill++;
        }
      } else {                  // In a hole or beyond input file
        insection = 0;
        if(pageset)
          ret.nfill++;
      }
    }
  }

  if(fsp)
    *fsp = ret;

  return LIBAVRDUDE_SUCCESS;
}

// Helper functions for dry run to determine file access

int update_is_okfile(const char *fn) {
  struct stat info;

  // File exists and is a regular file or a character file, eg, /dev/urandom
  return fn && *fn && stat(fn, &info) == 0 && !!(info.st_mode & (S_IFREG | S_IFCHR));
}

int update_is_writeable(const char *fn) {
  if(!fn || !*fn)
    return 0;

  // Assume writing to stdout will be OK
  if(str_eq(fn, "-"))
    return 1;

  // File exists? If so return whether it's readable and an OK file type
  if(access(fn, F_OK) == 0)
    return access(fn, W_OK) == 0 && update_is_okfile(fn);

  // File does not exist: try to create it
  FILE *test = fopen(fn, "w");

  if(test) {
    unlink(fn);
    fclose(test);
  }
  return !!test;
}

int update_is_readable(const char *fn) {
  if(!fn || !*fn)
    return 0;

  // Assume reading from stdin will be OK
  if(str_eq(fn, "-"))
    return 1;

  // File exists, is readable by the process and an OK file type?
  return access(fn, R_OK) == 0 && update_is_okfile(fn);
}

static void ioerror(const char *iotype, const UPDATE *upd) {
  int errnocp = errno;

  pmsg_ext_error("file %s is not %s: ", str_outfilename(upd->filename), iotype);
  if(errnocp)
    msg_ext_error("%s", strerror(errnocp));
  else if(upd->filename && *upd->filename)
    msg_ext_error("(not a regular or character file?)");
  msg_ext_error("\n");
}

// Whether a memory should be returned for ALL: exclude IO/SRAM
static int is_interesting_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem) {
  return !mem_is_io(mem) && !mem_is_sram(mem) && !(pgm && avr_mem_exclude(pgm, p, mem));
}

// Whether a memory should be backup-ed: exclude sub-memories
static int is_backup_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem) {
  return mem_is_in_flash(mem)? mem_is_flash(mem):
    mem_is_in_sigrow(mem)? mem_is_sigrow(mem):
    mem_is_in_fuses(mem)? mem_is_fuses(mem) || !avr_locate_fuses(p): is_interesting_mem(pgm, p, mem);
}

// Add (not == 0) or subtract (not == 1) a memory from list
static int memadd(AVRMEM **mlist, int nm, int not, AVRMEM *m) {
  for(int i = 0; i < nm; i++)
    if(mlist[i] == m) {
      if(not)
        mlist[i] = NULL;
      return nm;
    }
  if(!not)
    mlist[nm++] = m;
  return nm;
}

/*
 * Generate a memory list from string mstr, part p and intended programming
 * modes pm; then put number of memories into *np.
 *
 * Memory list can be sth like ee,fl,all,-cal,efuse. -mem or /mem removes it
 * from the list. Normal use is to pass NULL for dry and let the function write
 * to *np and *rwvsoftfail indicating unknown memories for this part. If dry is
 * set then -1 will be written to *dry when a generally unknown memory is used.
 */
AVRMEM **memory_list(const char *mstr, const PROGRAMMER *pgm, const AVRPART *p, int *np, int *rwvsoftp, int *dry) {

  int not, nm = (lsize(p->mem) + 1)*((int) str_numc(mstr, ',') + 1);  // Upper limit
  AVRMEM *m, **umemlist = mmt_malloc(nm*sizeof *umemlist);
  char *dstr = mmt_strdup(mstr), *s = dstr, *e;

  nm = 0;                       // Now count how many there really are mentioned
  // Parse comma-separated list of memories incl memory all
  for(e = strchr(s, ','); 1; e = strchr(s, ',')) {
    if(e)
      *e = 0;
    s = str_trim(s);
    if((not = *s == '\\' || *s == '-')) // \mem or -mem removes the memory
      s++;
    if(str_eq(s, "ALL")) {
      for(LNODEID lm = lfirst(p->mem); lm; lm = lnext(lm))
        if(is_interesting_mem(pgm, p, (m = ldata(lm))))
          nm = memadd(umemlist, nm, not, m);
    } else if(str_eq(s, "all") || str_eq(s, "etc")) {
      for(LNODEID lm = lfirst(p->mem); lm; lm = lnext(lm))
        if(is_backup_mem(pgm, p, (m = ldata(lm))))
          nm = memadd(umemlist, nm, not, m);
    } else if(!*s) {            // Ignore empty list elements
    } else {
      if(dry) {
        // Reject an update if memory name is not known amongst any part (suspect a typo)
        if(!avr_mem_might_be_known(s)) {
          pmsg_error("unknown memory %s in -U %s:...\n", s, mstr);
          *dry = LIBAVRDUDE_GENERAL_FAILURE;
          mmt_free(dstr);
          goto done;
        } else if(!avr_locate_mem(p, s))
          *dry = LIBAVRDUDE_SOFTFAIL;
      }
      if((m = avr_locate_mem(p, s)))
        nm = memadd(umemlist, nm, not, m);
      else if(rwvsoftp) {
        pmsg_warning("skipping unknown memory %s in list -U %s:...\n", s, mstr);
        *rwvsoftp = 1;
      }
    }
    if(!e)
      break;
    s = e + 1;
  }
  mmt_free(dstr);

  int nj = 0;

  for(int i = 0; i < nm; i++)   // Remove NULL entries
    if((umemlist[nj] = umemlist[i]))
      nj++;
  nm = nj;

done:
  if(np)
    *np = nm;

  return umemlist;
}

// Returns whether or not the memory list contains a flash memory
int memlist_contains_flash(const char *mstr, const AVRPART *p) {
  int ret = 0, nm = 0;
  AVRMEM **mlist = memory_list(mstr, NULL, p, &nm, NULL, NULL);

  for(int i = 0; i < nm; i++)
    if(mem_is_in_flash(mlist[i]))
      ret = 1;

  mmt_free(mlist);
  return ret;
}

// Basic checks to reveal serious failure before programming (and on autodetect set format)
int update_dryrun(const AVRPART *p, UPDATE *upd) {
  int known, format_detect, ret = LIBAVRDUDE_SUCCESS;

  if(upd->cmdline) {            // Todo: parse terminal command line?
    cx->upd_termcmds = mmt_realloc(cx->upd_termcmds, sizeof(*cx->upd_termcmds)*(cx->upd_nterms + 1));
    cx->upd_termcmds[cx->upd_nterms++] = upd->cmdline;
    return 0;
  }

  mmt_free(memory_list(upd->memstr, NULL, p, NULL, NULL, &ret));

  known = 0;
  // Necessary to check whether the file is readable?
  if(upd->op == DEVICE_VERIFY || upd->op == DEVICE_WRITE || upd->format == FMT_AUTO) {
    if(upd->format != FMT_IMM) {
      // Need to read the file: was it written before, so will be known?
      for(int i = 0; i < cx->upd_nfwritten; i++)
        if(!cx->upd_wrote || (upd->filename && str_eq(cx->upd_wrote[i], upd->filename)))
          known = 1;
      // Could a -T terminal command have created the file?
      for(int i = 0; i < cx->upd_nterms; i++)
        if(!cx->upd_termcmds || (upd->filename && str_contains(cx->upd_termcmds[i], upd->filename)))
          known = 1;
      // Any -t interactive terminal could have created it
      for(int i = 0; i < cx->upd_nterms; i++)
        if(!cx->upd_termcmds || str_eq(cx->upd_termcmds[i], "interactive terminal"))
          known = 1;

      errno = 0;
      if(!known && !update_is_readable(upd->filename)) {
        ioerror("readable", upd);
        ret = LIBAVRDUDE_SOFTFAIL;      // Even so it might still be there later on
        known = 1;              // Pretend we know it, so no auto detect needed
      }
    }
  }

  if(!known && upd->format == FMT_AUTO) {
    if(str_eq(upd->filename, "-")) {
      pmsg_error("cannot auto detect file format for stdin/out, specify explicitly\n");
      ret = LIBAVRDUDE_GENERAL_FAILURE;
    } else if((format_detect = fileio_fmt_autodetect(upd->filename)) < 0) {
      pmsg_warning("cannot determine file format for %s, specify explicitly\n", upd->filename);
      ret = LIBAVRDUDE_SOFTFAIL;
    } else {
      // Set format now (but might be wrong in edge cases, where user needs to specify explicity)
      upd->format = format_detect;
      if(quell_progress < 2)
        pmsg_notice2("%s file %s auto detected as %s\n",
          upd->op == DEVICE_READ? "output": "input", upd->filename, fileio_fmtstr(upd->format));
    }
  }

  switch(upd->op) {
  case DEVICE_READ:
    if(upd->format == FMT_IMM) {
      pmsg_error("invalid file format 'immediate' for output\n");
      ret = LIBAVRDUDE_GENERAL_FAILURE;
    } else {
      errno = 0;
      if(!update_is_writeable(upd->filename)) {
        ioerror("writeable", upd);
        ret = LIBAVRDUDE_SOFTFAIL;
      } else if(upd->filename) {        // Record filename (other than stdout) is available for future reads
        if(!str_eq(upd->filename, "-") &&
          (cx->upd_wrote = mmt_realloc(cx->upd_wrote, sizeof(*cx->upd_wrote)*(cx->upd_nfwritten + 1))))
          cx->upd_wrote[cx->upd_nfwritten++] = upd->filename;
      }
    }
    break;

  case DEVICE_VERIFY:          // Already checked that file is readable
  case DEVICE_WRITE:
    break;

  default:
    pmsg_error("invalid update operation (%d) requested\n", upd->op);
    ret = LIBAVRDUDE_GENERAL_FAILURE;
  }

  return ret;
}

static int update_avr_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  const UPDATE *upd, enum updateflags flags, int size, int multiple) {

  int rc = 0, pbar = (mem->size > 32 || verbose > 1) && update_progress;
  Filestats fs, fs_patched;
  const char *m_name = avr_mem_name(p, mem);

  if(memstats_mem(p, mem, size, &fs) < 0)
    return -1;
  if(multiple)                  // Single file writes multiple memories, say which ones
    pmsg_notice("%d byte%s %s ", fs.nbytes, str_plural(fs.nbytes), m_name);
  else
    imsg_notice("");
  msg_notice("in %d section%s %s%s", fs.nsections, str_plural(fs.nsections),
    fs.nsections == 1? "": "of ", str_ccinterval(fs.firstaddr, fs.lastaddr));
  if(mem->page_size > 1) {
    msg_notice(": %d page%s and %d pad byte%s",
      fs.npages, str_plural(fs.npages), fs.nfill, str_plural(fs.nfill));
    if(fs.ntrailing)
      imsg_notice("cutting off %d trailing 0xff byte%s (use -A to keep trailing 0xff)",
        fs.ntrailing, str_plural(fs.ntrailing));
  }
  msg_notice("\n");

  // Patch flash input, eg, for vector bootloaders
  if(pgm->flash_readhook && mem_is_flash(mem)) {
    if((size = pgm->flash_readhook(pgm, p, mem, upd->filename, size)) < 0) {
      pmsg_error("readhook for file %s failed\n", str_infilename(upd->filename));
      return -1;
    }
    if(memstats_mem(p, mem, size, &fs_patched) < 0)
      return -1;
    if(memcmp(&fs_patched, &fs, sizeof fs)) {
      pmsg_notice("preparing flash input for device%s\n", is_spm(pgm)? " bootloader": "");
      imsg_notice("%d byte%s in %d section%s %s%s",
        fs_patched.nbytes, str_plural(fs_patched.nbytes),
        fs_patched.nsections, str_plural(fs_patched.nsections),
        fs_patched.nsections == 1? "": "of ", str_ccinterval(fs_patched.firstaddr, fs_patched.lastaddr));
      if(mem->page_size > 1) {
        msg_notice(": %d page%s and %d pad byte%s",
          fs_patched.npages, str_plural(fs_patched.npages), fs_patched.nfill, str_plural(fs_patched.nfill));
        if(fs_patched.ntrailing)
          imsg_notice("note %d trailing 0xff byte%s", fs_patched.ntrailing, str_plural(fs_patched.ntrailing));
      }
      msg_notice("\n");
      memcpy(&fs, &fs_patched, sizeof fs);
    }
  }
  // Write the buffer contents to the selected memory
  int spellout = size > 0 && size <= 4 && fs.nbytes == size;

  pmsg_info("writing %d byte%s %sto %s", fs.nbytes, str_plural(fs.nbytes),
    spellout? str_ccprintf("(0x%s) ", str_cchex(mem->buf, size, 1) + 1): "", m_name);

  if(flags & UF_NOWRITE) {
    // Test mode: write to stdout in intel hex rather than to the chip
    rc = fileio_mem(FIO_WRITE, "-", FMT_IHEX, p, mem, size);
  } else {
    if(pbar)
      report_progress(0, 1, str_ccprintf("%*sWriting", (int) strlen(progbuf), ""));
    rc = avr_write_mem(pgm, p, mem, size, (flags & UF_AUTO_ERASE) != 0);
    report_progress(1, 1, NULL);
  }

  if(rc < 0)
    return -1;
  // @@@ has there has been output in the meantime to make the ", x bytes written" look out of place?
  if(pbar && !(flags & UF_VERIFY))
    pmsg_info("%d byte%s of %s written", fs.nbytes, str_plural(fs.nbytes), m_name);
  else if(!pbar)
    msg_info(", %d byte%s written", fs.nbytes, str_plural(fs.nbytes));

  return rc;                    // Highest memory address written plus 1
}

static int update_avr_verify(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  const UPDATE *upd, int size, const char *caption) {

  int retval = LIBAVRDUDE_GENERAL_FAILURE, pbar = (mem->size > 32 || verbose > 1) && update_progress;
  Filestats fs;
  AVRPART *v = avr_dup_part(p);
  const char *m_name = avr_mem_name(p, mem);

  if(memstats_mem(p, mem, size, &fs) < 0)
    goto error;

  led_set(pgm, LED_VFY);
  if(pbar)
    report_progress(0, 1, caption);
  int rc = avr_read_mem(pgm, p, mem, v);

  report_progress(1, 1, NULL);
  if(rc < 0) {
    pmsg_error("unable to read all of %s (rc = %d)\n", m_name, rc);
    led_set(pgm, LED_ERR);
    goto error;
  }

  rc = avr_verify_mem(pgm, p, v, mem, size);
  if(rc < 0) {
    pmsg_error("%s verification mismatch\n", mem->desc);
    led_set(pgm, LED_ERR);
    goto error;
  }
  // @@@ has there has been output in the meantime to make the ", x bytes verified" look out of place?
  int verified = fs.nbytes + fs.ntrailing;

  if(pbar || upd->op == DEVICE_VERIFY)
    pmsg_info("%d byte%s of %s verified\n", verified, str_plural(verified), m_name);
  else
    msg_info(", %d verified\n", verified);

  retval = LIBAVRDUDE_SUCCESS;

error:
  led_clr(pgm, LED_VFY);
  avr_free_part(v);
  return retval;
}

static int update_mem_from_all(const UPDATE *upd, const AVRPART *p, const AVRMEM *m, const AVRMEM *all, int allsize) {

  const char *m_name = avr_mem_name(p, m);
  int off = fileio_mem_offset(p, m);

  if(off < 0) {
    pmsg_warning("cannot map %s to flat address space, skipping ...\n", m_name);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }
  // Copy input file contents into memory
  int size = m->size;

  if(allsize - off < size)      // Clip to available data in input
    size = allsize > off? allsize - off: 0;
  if(is_memset(all->tags + off, 0, size)) // Nothing set? This memory was not present
    size = 0;
  if(size == 0)
    pmsg_warning("%s has no data for %s, skipping ...\n", str_infilename(upd->filename), m_name);

  memcpy(m->buf, all->buf + off, size);
  memcpy(m->tags, all->tags + off, size);

  return size;
}

static int update_all_from_file(const UPDATE *upd, const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *all, const char *mem_desc, Filestats *fsp) {
  // On writing to the device trailing 0xff might be cut off
  int op = upd->op == DEVICE_WRITE? FIO_READ: FIO_READ_FOR_VERIFY;
  int allsize = fileio_mem(op, upd->filename, upd->format, p, all, -1);

  if(allsize < 0) {
    pmsg_error("reading from file %s failed\n", str_infilename(upd->filename));
    return -1;
  }
  if(memstats_mem(p, all, allsize, fsp) < 0)
    return -1;
  pmsg_info(upd->op == DEVICE_WRITE?
    "reading %d byte%s for %s from input file %s\n":
    "verifying %d byte%s of %s against input file %s\n",
    fsp->nbytes, str_plural(fsp->nbytes), mem_desc, str_infilename(upd->filename)
    );

  return allsize;
}

int do_op(const PROGRAMMER *pgm, const AVRPART *p, const UPDATE *upd, enum updateflags flags) {
  int retval = LIBAVRDUDE_GENERAL_FAILURE, rwvproblem = 0, rwvsoftfail = 0;
  AVRMEM *mem, **umemlist = NULL, *m;
  Segment *seglist = NULL;
  Filestats fs;
  const char *umstr = upd->memstr;

  if(!(flags & UF_NOHEADING)) {
    char *heading = update_str(upd);

    lmsg_info("\n");            // Ensure an empty line for visual separation of operations
    pmsg_info("processing %s\n", heading);
    mmt_free(heading);
  }

  if(upd->cmdline) {
    if(!str_eq(upd->cmdline, "interactive terminal"))
      return terminal_line(pgm, p, upd->cmdline);
    // Interactive terminal shell
    clearerr(stdin);
    return terminal_mode(pgm, p);
  }

  int allsize, len, maxrlen = 0, ns = 0;

  if(is_multimem(umstr)) {
    umemlist = memory_list(umstr, pgm, p, &ns, &rwvsoftfail, NULL);

    if(!ns) {                   // ns is number of memories listed
      pmsg_warning("skipping -U %s:... as no memory in part %s available\n", umstr, p->desc);
      mmt_free(umemlist);
      return LIBAVRDUDE_SOFTFAIL;
    }
    // Maximum length of memory names for to-be-processed memories
    for(int i = 0; i < ns; i++)
      if((len = strlen(avr_mem_name(p, umemlist[i]))) > maxrlen)
        maxrlen = len;

    seglist = mmt_malloc(ns*sizeof *seglist);
  }

  mem = umemlist? fileio_any_memory("any"): avr_locate_mem(p, umstr);
  if(mem == NULL) {
    pmsg_warning("skipping -U %s:... as memory not defined for part %s\n", umstr, p->desc);
    return LIBAVRDUDE_SOFTFAIL;
  }

  const char *rcap = str_ccprintf("%*sReading", (int) strlen(progbuf), "");
  const char *mem_desc = !umemlist? avr_mem_name(p, mem): ns == 1? avr_mem_name(p, umemlist[0]): "multiple memories";
  int rc = 0;

  switch(upd->op) {
  case DEVICE_READ:
    // Read out the specified device memory and write it to a file
    if(upd->format == FMT_IMM) {
      pmsg_error("invalid file format 'immediate' for output\n");
      goto error;
    }
    if(umemlist) {
      /*
       * Writing to all memories or a list of memories. It is crucial not to
       * skip empty flash memory: otherwise the output file cannot distinguish
       * between flash having been deliberately dropped by the user or it
       * having been empty. Therefore the code switches temporarily off
       * trailing 0xff optimisation. In theory, the output file could only
       * store those pages that are non-empty for a paged memory, and if it was
       * all empty, store only the first empty page to indicate the memory was
       * selected. However, file space on a PC is cheap and fast; the main use
       * case for saving "all/etc" memory is a backup, and AVRDUDE does not
       * want to rely on the uploader to know that the backup file requires the
       * paged memories to be erased first, so the code goes the full hog.
       */
      int dffo = cx->avr_disableffopt;

      cx->avr_disableffopt = 1;
      if(upd->format != FMT_IHEX && upd->format != FMT_IHXC && upd->format != FMT_SREC && upd->format != FMT_ELF) {
        pmsg_warning("generating %s file format with multiple memories that cannot\n", fileio_fmtstr(upd->format));
        imsg_warning("be read by %s; consider using :I :i or :s instead\n", progname);
      }
      pmsg_info("reading %s ...\n", mem_desc);
      int nn = 0, nbytes = 0;

      for(int ii = 0; ii < ns; ii++) {
        m = umemlist[ii];
        const char *m_name = avr_mem_name(p, m);
        const char *cap = str_ccprintf("%*s - %-*s", (int) strlen(progbuf), "", maxrlen, m_name);

        report_progress(0, 1, cap);
        int ret = avr_read_mem(pgm, p, m, NULL);

        report_progress(1, 1, NULL);
        if(ret < 0) {
          pmsg_warning("unable to read %s (ret = %d), skipping...\n", m_name, ret);
          rwvproblem = 1;
          continue;
        }
        unsigned off = fileio_mem_offset(p, m);

        if(off == ~0U) {
          pmsg_warning("cannot map %s to flat address space, skipping ...\n", m_name);
          rwvproblem = 1;
          continue;
        }
        if(ret > 0) {
          // Copy individual memory into multi memory
          memcpy(mem->buf + off, m->buf, ret);
          seglist[nn].addr = off;
          seglist[nn].len = ret;
          nbytes += ret;
          nn++;
        }
      }

      pmsg_info("writing %d byte%s to output file %s\n", nbytes, str_plural(nbytes), str_outfilename(upd->filename));
      if(nn)
        rc = fileio_segments(FIO_WRITE, upd->filename, upd->format, p, mem, nn, seglist);
      else
        pmsg_notice("empty memory, resulting file has no contents\n");
      cx->avr_disableffopt = dffo;
    } else {                    // Regular file
      pmsg_info("reading %s memory ...\n", mem_desc);
      if(mem->size > 32)
        report_progress(0, 1, rcap);
      rc = avr_read(pgm, p, umstr, 0);
      report_progress(1, 1, NULL);
      if(rc < 0) {
        pmsg_error("unable to read all of %s (rc = %d)\n", mem_desc, rc);
        goto error;
      }
      if(rc == 0)
        pmsg_notice("empty memory, resulting file has no contents\n");
      pmsg_info("writing %d byte%s to output file %s\n", rc, str_plural(rc), str_outfilename(upd->filename));
      rc = fileio_mem(FIO_WRITE, upd->filename, upd->format, p, mem, rc);
    }

    if(rc < 0) {
      pmsg_error("write to file %s failed\n", str_outfilename(upd->filename));
      goto error;
    }
    break;

  case DEVICE_WRITE:
    // Write the selected device memory/ies using data from a file
    if((allsize = update_all_from_file(upd, pgm, p, mem, mem_desc, &fs)) < 0)
      goto error;
    if(umemlist) {
      for(int i = 0; i < ns; i++) {
        m = umemlist[i];
        // Silently skip readonly memories and fuses/lock in bootloaders
        if(mem_is_readonly(m) || (is_spm(pgm) && (mem_is_in_fuses(m) || mem_is_lock(m))))
          continue;

        int ret, size = update_mem_from_all(upd, p, m, mem, allsize);

        switch(size) {
        case LIBAVRDUDE_GENERAL_FAILURE:
          rwvproblem = 1;
          break;
        case 0:
          rwvsoftfail = 1;
          break;
        default:
          if((ret = update_avr_write(pgm, p, m, upd, flags, size, 1)) < 0) {
            pmsg_warning("unable to write %s (ret = %d), skipping...\n", avr_mem_name(p, m), ret);
            rwvproblem = 1;
            continue;
          }
          // @@@ verify size could be too small if file was not a multi-file and had trailing 0xff
          if((flags & UF_VERIFY) && update_avr_verify(pgm, p, m, upd, size, rcap) < 0) {
            rwvproblem = 1;
            continue;
          }
        }
      }
    } else {
      if((rc = update_avr_write(pgm, p, mem, upd, flags, allsize, 0)) < 0) {
        pmsg_error("unable to write %s (rc = %d)\n", mem_desc, rc);
        goto error;
      }
      if((flags & UF_VERIFY) && update_avr_verify(pgm, p, mem, upd, fs.lastaddr + 1, rcap) < 0)
        goto error;
    }
    break;

  case DEVICE_VERIFY:
    // Verify that the in memory file is the same as what is on the chip
    if((allsize = update_all_from_file(upd, pgm, p, mem, mem_desc, &fs)) < 0)
      goto error;
    if(umemlist) {
      for(int i = 0; i < ns; i++) {
        m = umemlist[i];

        int size = update_mem_from_all(upd, p, m, mem, allsize);

        switch(size) {
        case LIBAVRDUDE_GENERAL_FAILURE:
          rwvproblem = 1;
          break;
        case 0:
          rwvsoftfail = 1;
          break;
        default:
          if(update_avr_verify(pgm, p, m, upd, size, rcap) < 0) {
            rwvproblem = 1;
            continue;
          }
        }
      }
    } else {
      if(update_avr_verify(pgm, p, mem, upd, allsize, rcap) < 0)
        goto error;
    }
    break;

  default:
    pmsg_error("invalid update operation (%d) requested\n", upd->op);
    goto error;
  }

  retval = rwvproblem? LIBAVRDUDE_GENERAL_FAILURE: rwvsoftfail? LIBAVRDUDE_SOFTFAIL: LIBAVRDUDE_SUCCESS;

error:
  if(umemlist) {
    avr_free_mem(mem);
    mmt_free(umemlist);
    mmt_free(seglist);
  }
  return retval;
}
