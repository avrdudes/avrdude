/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005  Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2007 Joerg Wunsch
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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "ac_cfg.h"
#include "avrdude.h"
#include "libavrdude.h"


/*
 * Parsing of [<memory>:<op>:<file>[:<fmt>] | <file>[:<fmt>]]
 *
 * As memory names don't contain colons and the r/w/v operation <op> is
 * a single character, check whether the first two colons sandwich one
 * character. If not, treat the argument as a filename (defaulting to
 * flash write). This allows colons in filenames other than those for
 * enclosing <op> and separating <fmt>, eg, C:/some/file.hex
 */
UPDATE *parse_op(const char *s) {
  // Assume -U <file>[:<fmt>] first
  UPDATE *upd = (UPDATE *) cfg_malloc(__func__, sizeof *upd);
  upd->memstr = NULL;           // Defaults to flash or application
  upd->op = DEVICE_WRITE;
  const char *fn = s;

  // Check for <memory>:c: start in which case override defaults
  const char *fc = strchr(s, ':');
  if(fc && fc[1] && fc[2] == ':') {
    if(!strchr("rwv", fc[1])) {
      pmsg_error("invalid I/O mode :%c: in -U %s\n", fc[1], s);
      imsg_error("I/O mode can be r, w or v for read, write or verify device\n");
      free(upd->memstr);
      free(upd);
      return NULL;
    }

    upd->memstr = memcpy(cfg_malloc(__func__, fc-s+1), s, fc-s);
    upd->op =
      fc[1]=='r'? DEVICE_READ:
      fc[1]=='w'? DEVICE_WRITE: DEVICE_VERIFY;
    fn = fc+3;
  }

  // Default to AUTO for write and verify, and to raw binary for read
  upd->format = upd->op == DEVICE_READ? FMT_RBIN: FMT_AUTO;

  // Filename: last char is format if the penultimate char is a colon
  size_t len = strlen(fn);
  if(len > 2 && fn[len-2] == ':') { // Assume format specified
    upd->format = fileio_format(fn[len-1]);
    if(upd->format == FMT_ERROR) {
      pmsg_error("invalid file format :%c in -U %s; known formats are\n", fn[len-1], s);
      for(int f, c, i=0; i<62; i++) {
        c = i<10? '0'+i: (i&1? 'A': 'a') + (i-10)/2;
        f = fileio_format(c);
        if(f != FMT_ERROR)
          imsg_error("  :%c %s\n", c, fileio_fmtstr(f));
      }
      free(upd->memstr);
      free(upd);
      return NULL;
    }
    len -= 2;
  }

  upd->filename = memcpy(cfg_malloc(__func__, len+1), fn, len);

  return upd;
}


UPDATE *dup_update(const UPDATE *upd) {
  UPDATE *u = (UPDATE *) cfg_malloc(__func__, sizeof *u);
  memcpy(u, upd, sizeof*u);
  u->memstr = upd->memstr? cfg_strdup(__func__, upd->memstr): NULL;
  u->filename = cfg_strdup(__func__, upd->filename);

  return u;
}

UPDATE *new_update(int op, const char *memstr, int filefmt, const char *fname) {
  UPDATE *u = (UPDATE *) cfg_malloc(__func__, sizeof *u);
  u->memstr = cfg_strdup(__func__, memstr);
  u->filename = cfg_strdup(__func__, fname);
  u->op = op;
  u->format = filefmt;

  return u;
}

UPDATE *cmd_update(const char *cmd) {
  UPDATE *u = (UPDATE *) cfg_malloc(__func__, sizeof *u);
  u->cmdline = cmd;

  return u;
}

void free_update(UPDATE *u) {
  if(u) {
    free(u->memstr);
    free(u->filename);
    memset(u, 0, sizeof *u);
    free(u);
  }
}

char *update_str(const UPDATE *upd) {
  if(upd->cmdline)
    return str_sprintf("-%c %s",
      str_eq("interactive terminal", upd->cmdline)? 't': 'T', upd->cmdline);
  return str_sprintf("-U %s:%c:%s:%c",
    upd->memstr,
    upd->op == DEVICE_READ? 'r': upd->op == DEVICE_WRITE? 'w': 'v',
    upd->filename,
    fileio_fmtchr(upd->format));
}

// Memory statistics considering holes after a file read returned size bytes
int memstats(const AVRPART *p, const char *memstr, int size, Filestats *fsp) {
  Filestats ret = { 0 };
  AVRMEM *mem = avr_locate_mem(p, memstr);

  if(!mem) {
    pmsg_error("%s %s undefined\n", p->desc, memstr);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(!mem->buf || !mem->tags) {
    pmsg_error("%s %s is not set\n", p->desc, memstr);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  int pgsize = mem->page_size;
  if(pgsize < 1)
    pgsize = 1;

  if(size < 0 || size > mem->size) {
    pmsg_error("size %d at odds with %s %s size %d\n", size, p->desc, memstr, mem->size);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  ret.lastaddr = -1;
  int firstset = 0, insection = 0;
  // Scan all memory
  for(int addr = 0; addr < mem->size; ) {
    int pageset = 0;
    // Go page by page
    for(int pgi = 0; pgi < pgsize; pgi++, addr++) {
      if(mem->tags[addr] & TAG_ALLOCATED) {
        if(!firstset) {
          firstset = 1;
          ret.firstaddr = addr;
        }
        ret.lastaddr = addr;
        // size can be smaller than tags suggest owing to flash trailing-0xff
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

  pmsg_ext_error("file %s is not %s: ", str_outname(upd->filename), iotype);
  if(errnocp)
    msg_ext_error("%s", strerror(errnocp));
  else if(upd->filename && *upd->filename)
    msg_ext_error("(not a regular or character file?)");
  msg_ext_error("\n");
}

// Basic checks to reveal serious failure before programming (and on autodetect set format)
int update_dryrun(const AVRPART *p, UPDATE *upd) {
  static const char **wrote, **termcmds;
  static int nfwritten, nterms;

  int known, format_detect, ret = LIBAVRDUDE_SUCCESS;

  if(upd->cmdline) {            // Todo: parse terminal command line?
    termcmds = realloc(termcmds, sizeof(*termcmds) * (nterms+1));
    termcmds[nterms++] = upd->cmdline;
    return 0;
  }

  /*
   * Reject an update if memory name is not known amongst any part (suspect a typo)
   * but accept when the specific part does not have it (allow unifying i/faces)
   */
  if(!avr_mem_might_be_known(upd->memstr)) {
    pmsg_error("unknown memory %s\n", upd->memstr);
    ret = LIBAVRDUDE_GENERAL_FAILURE;
  } else if(p && !avr_locate_mem(p, upd->memstr))
    ret = LIBAVRDUDE_SOFTFAIL;

  known = 0;
  // Necessary to check whether the file is readable?
  if(upd->op == DEVICE_VERIFY || upd->op == DEVICE_WRITE || upd->format == FMT_AUTO) {
    if(upd->format != FMT_IMM) {
      // Need to read the file: was it written before, so will be known?
      for(int i = 0; i < nfwritten; i++)
        if(!wrote || (upd->filename && str_eq(wrote[i], upd->filename)))
          known = 1;
      // Could a -T terminal command have created the file?
      for(int i = 0; i < nterms; i++)
        if(!termcmds || (upd->filename && str_contains(termcmds[i], upd->filename)))
          known = 1;
      // Any -t interactive terminal could have created it
      for(int i = 0; i < nterms; i++)
        if(!termcmds || str_eq(termcmds[i], "interactive terminal"))
          known = 1;

      errno = 0;
      if(!known && !update_is_readable(upd->filename)) {
        ioerror("readable", upd);
        ret = LIBAVRDUDE_SOFTFAIL; // Even so it might still be there later on
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
        pmsg_notice("%s file %s auto detected as %s\n",
          upd->op == DEVICE_READ? "output": "input", upd->filename,
          fileio_fmtstr(upd->format));
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
      } else if(upd->filename) { // Record filename (other than stdout) is available for future reads
        if(!str_eq(upd->filename, "-") && (wrote = realloc(wrote, sizeof(*wrote) * (nfwritten+1))))
          wrote[nfwritten++] = upd->filename;
      }
    }
    break;

  case DEVICE_VERIFY:           // Already checked that file is readable
  case DEVICE_WRITE:
    break;

  default:
    pmsg_error("invalid update operation (%d) requested\n", upd->op);
    ret = LIBAVRDUDE_GENERAL_FAILURE;
  }

  return ret;
}


int do_op(const PROGRAMMER *pgm, const AVRPART *p, const UPDATE *upd, enum updateflags flags) {
  AVRPART *v;
  AVRMEM *mem;
  int size;
  int rc;
  Filestats fs, fs_patched;
  char *tofree;

  msg_info("\v\n");
  pmsg_info("processing %s\n", tofree = update_str(upd));
  free(tofree);

  if(upd->cmdline) {
    if(!str_eq(upd->cmdline, "interactive terminal"))
      return terminal_line(pgm, p, upd->cmdline);
    // Interactive terminal shell
    clearerr(stdin);
    return terminal_mode(pgm, p);
  }

  mem = avr_locate_mem(p, upd->memstr);
  if (mem == NULL) {
    pmsg_warning("skipping -U %s:... as memory not defined for part %s\n", upd->memstr, p->desc);
    return LIBAVRDUDE_SOFTFAIL;
  }

  AVRMEM_ALIAS *alias_mem = avr_find_memalias(p, mem);
  char *alias_mem_desc = cfg_malloc("do_op()", 2 + (alias_mem && alias_mem->desc? strlen(alias_mem->desc): 0));
  if(alias_mem && alias_mem->desc && *alias_mem->desc) {
    *alias_mem_desc = '/';
    strcpy(alias_mem_desc+1, alias_mem->desc);
  }
  
  switch (upd->op) {
  case DEVICE_READ:
    // Read out the specified device memory and write it to a file
    if (upd->format == FMT_IMM) {
      pmsg_error("invalid file format 'immediate' for output\n");
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    pmsg_info("reading %s%s memory ...\n", mem->desc, alias_mem_desc);

    if(mem->size > 32 || verbose > 1)
      report_progress(0, 1, "Reading");
    
    rc = avr_read(pgm, p, upd->memstr, 0);
    report_progress(1, 1, NULL);
    if (rc < 0) {
      pmsg_error("unable to read all of %s%s memory, rc=%d\n", mem->desc, alias_mem_desc, rc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    size = rc;

    if (rc == 0)
      pmsg_notice("flash is empty, resulting file has no contents\n");
    pmsg_info("writing output file %s\n", str_outname(upd->filename));

    rc = fileio(FIO_WRITE, upd->filename, upd->format, p, upd->memstr, size);
    if (rc < 0) {
      pmsg_error("write to file %s failed\n", str_outname(upd->filename));
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    break;

  case DEVICE_WRITE:
    // Write the selected device memory using data from a file

    rc = fileio(FIO_READ, upd->filename, upd->format, p, upd->memstr, -1);
    if (rc < 0) {
      pmsg_error("read from file %s failed\n", str_inname(upd->filename));
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    pmsg_info("reading input file %s for %s%s\n",
      str_inname(upd->filename), mem->desc, alias_mem_desc);

    if(memstats(p, upd->memstr, rc, &fs) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

    imsg_info("with %d byte%s in %d section%s within %s\n",
      fs.nbytes, str_plural(fs.nbytes),
      fs.nsections, str_plural(fs.nsections),
      str_interval(fs.firstaddr, fs.lastaddr));
    if(mem->page_size > 1) {
      imsg_info("using %d page%s and %d pad byte%s",
        fs.npages, str_plural(fs.npages),
        fs.nfill, str_plural(fs.nfill));
      if(fs.ntrailing)
        msg_info(", cutting off %d trailing 0xff byte%s",
          fs.ntrailing, str_plural(fs.ntrailing));
      msg_info("\n");
    }

    // Patch flash input, eg, for vector bootloaders
    if(pgm->flash_readhook) {
      AVRMEM *mem = avr_locate_mem(p, upd->memstr);
      if(mem && mem_is_flash(mem)) {
        rc = pgm->flash_readhook(pgm, p, mem, upd->filename, rc);
        if (rc < 0) {
          pmsg_notice("readhook for file %s failed\n", str_inname(upd->filename));
          return LIBAVRDUDE_GENERAL_FAILURE;
        }
        if(memstats(p, upd->memstr, rc, &fs_patched) < 0)
          return LIBAVRDUDE_GENERAL_FAILURE;
        if(memcmp(&fs_patched, &fs, sizeof fs)) {
          pmsg_info("preparing flash input for device%s\n",
            pgm->prog_modes & PM_SPM? " bootloader": "");
            imsg_notice2("with %d byte%s in %d section%s within %s\n",
              fs_patched.nbytes, str_plural(fs_patched.nbytes),
              fs_patched.nsections, str_plural(fs_patched.nsections),
              str_interval(fs_patched.firstaddr, fs_patched.lastaddr));
            if(mem->page_size > 1) {
              imsg_notice2("using %d page%s and %d pad byte%s",
                fs_patched.npages, str_plural(fs_patched.npages),
                fs_patched.nfill, str_plural(fs_patched.nfill));
              if(fs_patched.ntrailing)
                msg_notice2(", and %d trailing 0xff byte%s",
                  fs_patched.ntrailing, str_plural(fs_patched.ntrailing));
              msg_notice2("\n");
            }
        }
      }
    }
    size = rc;

    // Write the buffer contents to the selected memory
    pmsg_info("writing %d byte%s %s%s ...\n", fs.nbytes,
      str_plural(fs.nbytes), mem->desc, alias_mem_desc);

    if (!(flags & UF_NOWRITE)) {
      if(mem->size > 32 || verbose > 1)
        report_progress(0, 1, "Writing");
      rc = avr_write(pgm, p, upd->memstr, size, (flags & UF_AUTO_ERASE) != 0);
      report_progress(1, 1, NULL);
    } else {
      // Test mode: write to stdout in intel hex rather than to the chip
      rc = fileio(FIO_WRITE, "-", FMT_IHEX, p, upd->memstr, size);
    }

    if (rc < 0) {
      pmsg_error("unable to write %s%s memory, rc=%d\n", mem->desc, alias_mem_desc, rc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    pmsg_info("%d byte%s of %s%s written\n", fs.nbytes,
      str_plural(fs.nbytes), mem->desc, alias_mem_desc);

    if (!(flags & UF_VERIFY))   // Fall through for auto verify unless
      break;
    // Fall through

  case DEVICE_VERIFY:
    // Verify that the in memory file is the same as what is on the chip
    led_set(pgm, LED_VFY);

    int userverify = upd->op == DEVICE_VERIFY; // Explicit -U :v by user

    pmsg_info("verifying %s%s memory against %s\n", mem->desc,
      alias_mem_desc, str_inname(upd->filename));

    // No need to read file when fallen through from DEVICE_WRITE
    if (userverify) {
      pmsg_notice("load %s%s data from input file %s\n", mem->desc,
        alias_mem_desc, str_inname(upd->filename));

      rc = fileio(FIO_READ_FOR_VERIFY, upd->filename, upd->format, p, upd->memstr, -1);

      if (rc < 0) {
        pmsg_error("read from file %s failed\n", str_inname(upd->filename));
        led_set(pgm, LED_ERR);
        led_clr(pgm, LED_VFY);
        return LIBAVRDUDE_GENERAL_FAILURE;
      }
      size = rc;

      if(memstats(p, upd->memstr, size, &fs) < 0) {
        led_set(pgm, LED_ERR);
        led_clr(pgm, LED_VFY);
        return LIBAVRDUDE_GENERAL_FAILURE;
      }
    } else {
      // Correct size of last read to include potentially cut off, trailing 0xff (flash)
      size = fs.lastaddr+1;
    }

    v = avr_dup_part(p);

    if (quell_progress < 2) {
      if (userverify)
        pmsg_notice("input file %s contains %d byte%s\n",
          str_inname(upd->filename), fs.nbytes, str_plural(fs.nbytes));
      pmsg_notice2("reading on-chip %s%s data ...\n", mem->desc, alias_mem_desc);
    }

    if(mem->size > 32 || verbose > 1)
      report_progress (0,1,"Reading");
    rc = avr_read(pgm, p, upd->memstr, v);
    report_progress (1,1,NULL);
    if (rc < 0) {
      pmsg_error("unable to read all of %s%s memory, rc=%d\n", mem->desc, alias_mem_desc, rc);
      led_set(pgm, LED_ERR);
      led_clr(pgm, LED_VFY);
      avr_free_part(v);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    if (quell_progress < 2)
      pmsg_notice2("verifying ...\n");

    rc = avr_verify(pgm, p, v, upd->memstr, size);
    if (rc < 0) {
      pmsg_error("verification mismatch\n");
      led_set(pgm, LED_ERR);
      led_clr(pgm, LED_VFY);
      avr_free_part(v);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    int verified = fs.nbytes+fs.ntrailing;
    pmsg_info("%d byte%s of %s%s verified\n", verified, str_plural(verified), mem->desc, alias_mem_desc);

    led_clr(pgm, LED_VFY);
    avr_free_part(v);
    break;

  default:
    pmsg_error("invalid update operation (%d) requested\n", upd->op);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return LIBAVRDUDE_SUCCESS;
}
