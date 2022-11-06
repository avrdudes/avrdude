/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2005  Brian S. Dean <bsd@bsdhome.com>
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

UPDATE * parse_op(char * s)
{
  char buf[1024];
  char * p, * cp, c;
  UPDATE * upd;
  int i;
  size_t fnlen;

  upd = (UPDATE *) cfg_malloc("parse_op()", sizeof(UPDATE));

  i = 0;
  p = s;
  while (i < (int) sizeof(buf)-1 && *p && *p != ':')
    buf[i++] = *p++;
  buf[i] = 0;

  if (*p != ':') {
    upd->memtype = NULL;        /* default memtype, "flash", or "application" */
    upd->op = DEVICE_WRITE;
    upd->filename = cfg_strdup("parse_op()", buf);
    upd->format = FMT_AUTO;
    return upd;
  }

  upd->memtype = cfg_strdup("parse_op()", buf);

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
    pmsg_error("invalid I/O mode '%c' in update specification\n", *p);
    msg_error("  allowed values are:\n"
              "    r = read device\n"
              "    w = write device\n"
              "    v = verify device\n");
    free(upd->memtype);
    free(upd);
    return NULL;
  }

  p++;

  if (*p != ':') {
    pmsg_error("invalid update specification\n");
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
    // missing format, default to "AUTO" for write and verify,
    // and to binary for read operations:
    upd->format = upd->op == DEVICE_READ? FMT_RBIN: FMT_AUTO;
    fnlen = strlen(cp);
    upd->filename = (char *) cfg_malloc("parse_op()", fnlen + 1);
  } else {
    fnlen = p - cp;
    upd->filename = (char *) cfg_malloc("parse_op()", fnlen +1);
    c = *++p;
    if (c && p[1])
      /* More than one char - force failure below. */
      c = '?';
    switch (c) {
      case 'a': upd->format = FMT_AUTO; break;
      case 's': upd->format = FMT_SREC; break;
      case 'i': upd->format = FMT_IHEX; break;
      case 'I': upd->format = FMT_IHXC; break;
      case 'r': upd->format = FMT_RBIN; break;
      case 'e': upd->format = FMT_ELF; break;
      case 'm': upd->format = FMT_IMM; break;
      case 'b': upd->format = FMT_BIN; break;
      case 'd': upd->format = FMT_DEC; break;
      case 'h': upd->format = FMT_HEX; break;
      case 'o': upd->format = FMT_OCT; break;
      default:
        pmsg_error("invalid file format '%s' in update specifier\n", p);
        free(upd->memtype);
        free(upd);
        return NULL;
    }
  }

  memcpy(upd->filename, cp, fnlen);
  upd->filename[fnlen] = 0;

  return upd;
}

UPDATE * dup_update(UPDATE * upd)
{
  UPDATE * u;

  u = (UPDATE *) cfg_malloc("dup_update()", sizeof(UPDATE));

  memcpy(u, upd, sizeof(UPDATE));

  if (upd->memtype != NULL)
    u->memtype = cfg_strdup("dup_update()", upd->memtype);
  else
    u->memtype = NULL;
  u->filename = cfg_strdup("dup_update()", upd->filename);

  return u;
}

UPDATE * new_update(int op, char * memtype, int filefmt, char * filename)
{
  UPDATE * u;

  u = (UPDATE *) cfg_malloc("new_update()", sizeof(UPDATE));

  u->memtype = cfg_strdup("new_update()", memtype);
  u->filename = cfg_strdup("new_update()", filename);
  u->op = op;
  u->format = filefmt;

  return u;
}

void free_update(UPDATE * u)
{
    if (u != NULL) {
	if(u->memtype != NULL) {
	    free(u->memtype);
	    u->memtype = NULL;
	}
	if(u->filename != NULL) {
	    free(u->filename);
	    u->filename = NULL;
	}
	free(u);
    }
}


// Memory statistics considering holes after a file read returned size bytes
int memstats(const AVRPART *p, const char *memtype, int size, Filestats *fsp) {
  Filestats ret = { 0 };
  AVRMEM *mem = avr_locate_mem(p, memtype);

  if(!mem) {
    pmsg_error("%s %s undefined\n", p->desc, memtype);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(!mem->buf || !mem->tags) {
    pmsg_error("%s %s is not set\n", p->desc, memtype);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  int pgsize = mem->page_size;
  if(pgsize < 1)
    pgsize = 1;

  if(size < 0 || size > mem->size) {
    pmsg_error("size %d at odds with %s %s size %d\n", size, p->desc, memtype, mem->size);
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


// Convenience functions for printing
const char *update_plural(int x) {
  return x==1? "": "s";
}

const char *update_inname(const char *fn) {
  return !fn? "???": strcmp(fn, "-")? fn: "<stdin>";
}

const char *update_outname(const char *fn) {
  return !fn? "???": strcmp(fn, "-")? fn: "<stdout>";
}

// Return sth like "[0, 0x1ff]"
const char *update_interval(int a, int b) {
  // Cyclic buffer for 20+ temporary interval strings each max 41 bytes at 64-bit int
  static char space[20*41 + 80], *sp;
  if(!sp || sp-space > (int) sizeof space - 80)
    sp = space;

  char *ret = sp;

  sprintf(sp, a<16? "[%d": "[0x%x", a);
  sp += strlen(sp);
  sprintf(sp, b<16? ", %d]": ", 0x%x]", b);

  // Advance beyond return string in temporary ring buffer
  sp += strlen(sp)+1;

  return ret;
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
  if(!strcmp(fn, "-"))
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
  if(!strcmp(fn, "-"))
    return 1;

  // File exists, is readable by the process and an OK file type?
  return access(fn, R_OK) == 0 && update_is_okfile(fn);
}


static void ioerror(const char *iotype, const UPDATE *upd) {
  int errnocp = errno;

  pmsg_ext_error("file %s is not %s: ", update_outname(upd->filename), iotype);
  if(errnocp)
    msg_ext_error("%s", strerror(errnocp));
  else if(upd->filename && *upd->filename)
    msg_ext_error("(not a regular or character file?)");
  msg_ext_error("\n");
}

// Basic checks to reveal serious failure before programming
int update_dryrun(const AVRPART *p, UPDATE *upd) {
  static char **wrote;
  static int nfwritten;

  int known, format_detect, ret = LIBAVRDUDE_SUCCESS;

  /*
   * Reject an update if memory name is not known amongst any part (suspect a typo)
   * but accept when the specific part does not have it (allow unifying i/faces)
   */
  if(!avr_mem_might_be_known(upd->memtype)) {
    pmsg_error("unknown memory type %s\n", upd->memtype);
    ret = LIBAVRDUDE_GENERAL_FAILURE;
  } else if(p && !avr_locate_mem(p, upd->memtype))
    ret = LIBAVRDUDE_SOFTFAIL;

  known = 0;
  // Necessary to check whether the file is readable?
  if(upd->op == DEVICE_VERIFY || upd->op == DEVICE_WRITE || upd->format == FMT_AUTO) {
    if(upd->format != FMT_IMM) {
      // Need to read the file: was it written before, so will be known?
      for(int i = 0; i < nfwritten; i++)
        if(!wrote || (upd->filename && !strcmp(wrote[i], upd->filename)))
          known = 1;

      errno = 0;
      if(!known && !update_is_readable(upd->filename)) {
        ioerror("readable", upd);
        ret = LIBAVRDUDE_GENERAL_FAILURE;
        known = 1;                // Pretend we know it, so no auto detect needed
      }
    }
  }

  if(!known && upd->format == FMT_AUTO) {
    if(!strcmp(upd->filename, "-")) {
      pmsg_error("cannot auto detect file format for stdin/out, "
        "specify explicitly\n");
      ret = LIBAVRDUDE_GENERAL_FAILURE;
    } else if((format_detect = fileio_fmt_autodetect(upd->filename)) < 0) {
      pmsg_error("cannot determine file format for %s, specify explicitly\n", upd->filename);
      ret = LIBAVRDUDE_GENERAL_FAILURE;
    } else {
      // Set format now, no need to repeat auto detection later
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
        ret = LIBAVRDUDE_GENERAL_FAILURE;
      } else if(upd->filename) { // Record filename (other than stdout) is available for future reads
        if(strcmp(upd->filename, "-") && (wrote = realloc(wrote, sizeof(*wrote) * (nfwritten+1))))
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


int do_op(const PROGRAMMER *pgm, const AVRPART *p, UPDATE *upd, enum updateflags flags) {
  AVRPART *v;
  AVRMEM *mem;
  int size;
  int rc;
  Filestats fs;

  mem = avr_locate_mem(p, upd->memtype);
  if (mem == NULL) {
    pmsg_warning("skipping -U %s:... as memory not defined for part %s\n", upd->memtype, p->desc);
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
    
    rc = avr_read(pgm, p, upd->memtype, 0);
    report_progress(1, 1, NULL);
    if (rc < 0) {
      pmsg_error("unable to read all of %s%s memory, rc=%d\n", mem->desc, alias_mem_desc, rc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    size = rc;

    if (rc == 0)
      pmsg_notice("flash is empty, resulting file has no contents\n");
    pmsg_info("writing output file %s\n", update_outname(upd->filename));

    rc = fileio(FIO_WRITE, upd->filename, upd->format, p, upd->memtype, size);
    if (rc < 0) {
      pmsg_error("write to file %s failed\n", update_outname(upd->filename));
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    break;

  case DEVICE_WRITE:
    // Write the selected device memory using data from a file

    rc = fileio(FIO_READ, upd->filename, upd->format, p, upd->memtype, -1);
    if (rc < 0) {
      pmsg_error("read from file %s failed\n", update_inname(upd->filename));
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    // Patch input if for flash, eg, for vector bootloaders?
    if(pgm->flash_readhook) {
      AVRMEM *mem = avr_locate_mem(p, upd->memtype);
      if(mem && !strcmp(mem->desc, "flash")) {
        rc = pgm->flash_readhook(pgm, p, mem, upd->filename, rc);
        if (rc < 0) {
          pmsg_notice("readhook for file %s failed\n", update_inname(upd->filename));
          return LIBAVRDUDE_GENERAL_FAILURE;
        }
      }
    }

    size = rc;
    pmsg_info("reading input file %s for %s%s\n",
      update_inname(upd->filename), mem->desc, alias_mem_desc);

    if(memstats(p, upd->memtype, size, &fs) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

    imsg_info("with %d byte%s in %d section%s within %s\n",
      fs.nbytes, update_plural(fs.nbytes),
      fs.nsections, update_plural(fs.nsections),
      update_interval(fs.firstaddr, fs.lastaddr));
    if(mem->page_size > 1) {
      imsg_info("using %d page%s and %d pad byte%s",
        fs.npages, update_plural(fs.npages),
        fs.nfill, update_plural(fs.nfill));
      if(fs.ntrailing)
        msg_info(", cutting off %d trailing 0xff byte%s",
          fs.ntrailing, update_plural(fs.ntrailing));
      msg_info("\n");
    }

    // Write the buffer contents to the selected memory type
    pmsg_info("writing %d byte%s %s%s ...\n", fs.nbytes,
      update_plural(fs.nbytes), mem->desc, alias_mem_desc);

    if (!(flags & UF_NOWRITE)) {
      if(mem->size > 32 || verbose > 1)
        report_progress(0, 1, "Writing");
      rc = avr_write(pgm, p, upd->memtype, size, (flags & UF_AUTO_ERASE) != 0);
      report_progress(1, 1, NULL);
    } else {
      // Test mode: write to stdout in intel hex rather than to the chip
      rc = fileio(FIO_WRITE, "-", FMT_IHEX, p, upd->memtype, size);
    }

    if (rc < 0) {
      pmsg_error("unable to write %s%s memory, rc=%d\n", mem->desc, alias_mem_desc, rc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    pmsg_info("%d byte%s of %s%s written\n", fs.nbytes,
      update_plural(fs.nbytes), mem->desc, alias_mem_desc);

    // Fall through for (default) auto verify, ie, unless -V was specified
    if (!(flags & UF_VERIFY))
      break;

  case DEVICE_VERIFY:
    // Verify that the in memory file is the same as what is on the chip
    pgm->vfy_led(pgm, ON);

    int userverify = upd->op == DEVICE_VERIFY; // Explicit -U :v by user

    pmsg_info("verifying %s%s memory against %s\n", mem->desc,
      alias_mem_desc, update_inname(upd->filename));

    // No need to read file when fallen through from DEVICE_WRITE
    if (userverify) {
      pmsg_notice("load %s%s data from input file %s\n", mem->desc,
        alias_mem_desc, update_inname(upd->filename));

      rc = fileio(FIO_READ_FOR_VERIFY, upd->filename, upd->format, p, upd->memtype, -1);

      if (rc < 0) {
        pmsg_error("read from file %s failed\n", update_inname(upd->filename));
        return LIBAVRDUDE_GENERAL_FAILURE;
      }
      size = rc;

      if(memstats(p, upd->memtype, size, &fs) < 0)
        return LIBAVRDUDE_GENERAL_FAILURE;
    } else {
      // Correct size of last read to include potentially cut off, trailing 0xff (flash)
      size = fs.lastaddr+1;
    }

    v = avr_dup_part(p);

    if (quell_progress < 2) {
      if (userverify)
        pmsg_notice("input file %s contains %d byte%s\n",
          update_inname(upd->filename), fs.nbytes, update_plural(fs.nbytes));
      pmsg_notice2("reading on-chip %s%s data ...\n", mem->desc, alias_mem_desc);
    }

    if(mem->size > 32 || verbose > 1)
      report_progress (0,1,"Reading");
    rc = avr_read(pgm, p, upd->memtype, v);
    report_progress (1,1,NULL);
    if (rc < 0) {
      pmsg_error("unable to read all of %s%s memory, rc=%d\n", mem->desc, alias_mem_desc, rc);
      pgm->err_led(pgm, ON);
      avr_free_part(v);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    if (quell_progress < 2)
      pmsg_notice2("verifying ...\n");

    rc = avr_verify(p, v, upd->memtype, size);
    if (rc < 0) {
      pmsg_error("verification mismatch\n");
      pgm->err_led(pgm, ON);
      avr_free_part(v);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    int verified = fs.nbytes+fs.ntrailing;
    pmsg_info("%d byte%s of %s%s verified\n", verified, update_plural(verified), mem->desc, alias_mem_desc);

    pgm->vfy_led(pgm, OFF);
    avr_free_part(v);
    break;

  default:
    pmsg_error("invalid update operation (%d) requested\n", upd->op);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return LIBAVRDUDE_SUCCESS;
}
