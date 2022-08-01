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

  upd = (UPDATE *)malloc(sizeof(UPDATE));
  if (upd == NULL) {
    avrdude_message(MSG_INFO, "%s: out of memory\n", progname);
    exit(1);
  }

  i = 0;
  p = s;
  while ((i < (sizeof(buf)-1) && *p && (*p != ':')))
    buf[i++] = *p++;
  buf[i] = 0;

  if (*p != ':') {
    upd->memtype = NULL;        /* default memtype, "flash", or "application" */
    upd->op = DEVICE_WRITE;
    upd->filename = (char *)malloc(strlen(buf) + 1);
    if (upd->filename == NULL) {
        avrdude_message(MSG_INFO, "%s: out of memory\n", progname);
        exit(1);
    }
    strcpy(upd->filename, buf);
    upd->format = FMT_AUTO;
    return upd;
  }

  upd->memtype = (char *)malloc(strlen(buf)+1);
  if (upd->memtype == NULL) {
    avrdude_message(MSG_INFO, "%s: out of memory\n", progname);
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
    avrdude_message(MSG_INFO, "%s: invalid I/O mode '%c' in update specification\n",
            progname, *p);
    avrdude_message(MSG_INFO, "  allowed values are:\n"
                    "    r = read device\n"
                    "    w = write device\n"
                    "    v = verify device\n");
    free(upd->memtype);
    free(upd);
    return NULL;
  }

  p++;

  if (*p != ':') {
    avrdude_message(MSG_INFO, "%s: invalid update specification\n", progname);
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
      case 'e': upd->format = FMT_ELF; break;
      case 'm': upd->format = FMT_IMM; break;
      case 'b': upd->format = FMT_BIN; break;
      case 'd': upd->format = FMT_DEC; break;
      case 'h': upd->format = FMT_HEX; break;
      case 'o': upd->format = FMT_OCT; break;
      default:
        avrdude_message(MSG_INFO, "%s: invalid file format '%s' in update specifier\n",
                progname, p);
        free(upd->memtype);
        free(upd);
        return NULL;
    }
  }

  if (upd->filename == NULL) {
    avrdude_message(MSG_INFO, "%s: out of memory\n", progname);
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
    avrdude_message(MSG_INFO, "%s: out of memory\n", progname);
    exit(1);
  }

  memcpy(u, upd, sizeof(UPDATE));

  if (upd->memtype != NULL)
    u->memtype = strdup(upd->memtype);
  else
    u->memtype = NULL;
  u->filename = strdup(upd->filename);

  return u;
}

UPDATE * new_update(int op, char * memtype, int filefmt, char * filename)
{
  UPDATE * u;

  u = (UPDATE *)malloc(sizeof(UPDATE));
  if (u == NULL) {
    avrdude_message(MSG_INFO, "%s: out of memory\n", progname);
    exit(1);
  }

  u->memtype = strdup(memtype);
  u->filename = strdup(filename);
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
int memstats(struct avrpart *p, char *memtype, int size, Filestats *fsp) {
  Filestats ret = { 0 };
  AVRMEM *mem = avr_locate_mem(p, memtype);

  if(!mem) {
    avrdude_message(MSG_INFO, "%s: %s %s undefined\n",
      progname, p->desc, memtype);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(!mem->buf || !mem->tags) {
    avrdude_message(MSG_INFO, "%s: %s %s is not set\n",
      progname, p->desc, memtype);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(mem->page_size < 1 || mem->size < 1 || mem->size % mem->page_size) {
    avrdude_message(MSG_INFO, "%s: %s %s size %d or page size %d not set\n",
      progname, p->desc, memtype, mem->size, mem->page_size);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(size < 0 || size > mem->size) {
    avrdude_message(MSG_INFO, "%s: memstats() size %d at odds with %s %s size %d\n",
      progname, size, p->desc, memtype, mem->size);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  int firstset = 0, insection = 0;
  // Scan all memory
  for(int addr = 0; addr < mem->size; ) {
    int pageset = 0;
    // Go page by page
    for(int pgi = 0; pgi < mem->page_size; pgi++, addr++) {
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
const char *plural(int x) {
  return x==1? "": "s";
}

const char *inname(const char *fn) {
  return !fn? "???": strcmp(fn, "-")? fn: "<stdin>";
}

const char *outname(const char *fn) {
  return !fn? "???": strcmp(fn, "-")? fn: "<stdout>";
}

// Return sth like "[0, 0x1ff]"
const char *interval(int a, int b) {
  // Cyclic buffer for 20+ temporary interval strings each max 41 bytes at 64-bit int
  static char space[20*41 + 80], *sp;
  if(!sp || sp-space > sizeof space - 80)
    sp = space;

  char *ret = sp;

  sprintf(sp, a? "[0x%x": "[0", a);
  sp += strlen(sp);
  sprintf(sp, b? ", 0x%x]": ", 0]", b);

  // Advance beyond return string in temporary ring buffer
  sp += strlen(sp)+1;

  return ret;
}


int do_op(PROGRAMMER * pgm, struct avrpart * p, UPDATE * upd, enum updateflags flags)
{
  struct avrpart * v;
  AVRMEM * mem;
  int size;
  int rc;
  Filestats fs;

  mem = avr_locate_mem(p, upd->memtype);
  if (mem == NULL) {
    avrdude_message(MSG_INFO, "%s memory type not defined for part %s\n",
      upd->memtype, p->desc);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  AVRMEM_ALIAS * alias_mem = avr_find_memalias(p, mem);
  char alias_mem_desc[AVR_DESCLEN + 1] = "";
  if(alias_mem) {
    strcat(alias_mem_desc, "/");
    strcat(alias_mem_desc, alias_mem->desc);
  }
  
  switch (upd->op) {
  case DEVICE_READ:
    // Read out the specified device memory and write it to a file
    if (upd->format == FMT_IMM) {
      avrdude_message(MSG_INFO,
        "%s: Invalid file format 'immediate' for output\n", progname);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    if (quell_progress < 2)
      avrdude_message(MSG_INFO, "%s: reading %s%s memory ...\n",
        progname, mem->desc, alias_mem_desc);

    report_progress(0, 1, "Reading");
    rc = avr_read(pgm, p, upd->memtype, 0);
    report_progress(1, 1, NULL);
    if (rc < 0) {
      avrdude_message(MSG_INFO, "%s: failed to read all of %s%s memory, rc=%d\n",
        progname, mem->desc, alias_mem_desc, rc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    size = rc;

    if (quell_progress < 2) {
      if (rc == 0)
        avrdude_message(MSG_INFO, "%s: flash is empty, resulting file has no contents\n",
          progname);
      avrdude_message(MSG_INFO, "%s: writing output file %s\n",
        progname, outname(upd->filename));
    }
    rc = fileio(FIO_WRITE, upd->filename, upd->format, p, upd->memtype, size);
    if (rc < 0) {
      avrdude_message(MSG_INFO, "%s: write to file %s failed\n",
        progname, outname(upd->filename));
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    break;

  case DEVICE_WRITE:
    // Write the selected device memory using data from a file

    rc = fileio(FIO_READ, upd->filename, upd->format, p, upd->memtype, -1);
    if (quell_progress < 2)
      avrdude_message(MSG_INFO, "%s: reading input file %s\n",
        progname, inname(upd->filename));
    if (rc < 0) {
      avrdude_message(MSG_INFO, "%s: read from file %s failed\n",
        progname, inname(upd->filename));
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    size = rc;

    if(memstats(p, upd->memtype, size, &fs) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

    if(quell_progress < 2) {
      avrdude_message(MSG_INFO, "%*s with %d byte%s in %d section%s within %s\n",
        (int) strlen(progname)+1, "",
        fs.nbytes, plural(fs.nbytes),
        fs.nsections, plural(fs.nsections),
        interval(fs.firstaddr, fs.lastaddr));
      if(mem->page_size > 1) {
        avrdude_message(MSG_INFO, "%*s using %d page%s and %d pad byte%s",
          (int) strlen(progname)+1, "",
          fs.npages, plural(fs.npages),
          fs.nfill, plural(fs.nfill));
        if(fs.ntrailing)
          avrdude_message(MSG_INFO, ", cutting off %d trailing 0xff byte%s",
            fs.ntrailing, plural(fs.ntrailing));
        avrdude_message(MSG_INFO, "\n");
      }
    }

    // Write the buffer contents to the selected memory type
    if (quell_progress < 2)
      avrdude_message(MSG_INFO, "%s: writing %d byte%s %s%s ...\n",
        progname, fs.nbytes, plural(fs.nbytes), mem->desc, alias_mem_desc);

    if (!(flags & UF_NOWRITE)) {
      report_progress(0, 1, "Writing");
      rc = avr_write(pgm, p, upd->memtype, size, (flags & UF_AUTO_ERASE) != 0);
      report_progress(1, 1, NULL);
    } else {
      // Test mode: write to stdout in intel hex rather than to the chip
      rc = fileio(FIO_WRITE, "-", FMT_IHEX, p, upd->memtype, size);
    }

    if (rc < 0) {
      avrdude_message(MSG_INFO, "%s: failed to write %s%s memory, rc=%d\n",
        progname, mem->desc, alias_mem_desc, rc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    if (quell_progress < 2)
      avrdude_message(MSG_INFO, "%s: %d byte%s of %s%s written\n",
        progname, fs.nbytes, plural(fs.nbytes), mem->desc, alias_mem_desc);

    // Fall through for (default) verify, ie, unless -V was specified
    if (!(flags & UF_VERIFY))
      break;

  case DEVICE_VERIFY:
    // Verify that the in memory file is the same as what is on the chip
    pgm->vfy_led(pgm, ON);

    if (quell_progress < 2) {
      avrdude_message(MSG_INFO, "%s: verifying %s%s memory against %s\n",
        progname, mem->desc, alias_mem_desc, inname(upd->filename));

      if (upd->op == DEVICE_VERIFY)
        avrdude_message(MSG_NOTICE2, "%s: load %s%s data from input file %s\n",
          progname, mem->desc, alias_mem_desc, inname(upd->filename));
    }

    // No need to read file when fallen through from DEVICE_WRITE
    if (upd->op == DEVICE_VERIFY) {
      rc = fileio(FIO_READ_FOR_VERIFY, upd->filename, upd->format, p, upd->memtype, -1);

      if (rc < 0) {
        avrdude_message(MSG_INFO, "%s: read from file %s failed\n",
          progname, inname(upd->filename));
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
      if (upd->op == DEVICE_VERIFY)
        avrdude_message(MSG_NOTICE2, "%s: input file %s contains %d byte%s\n",
          progname, inname(upd->filename), fs.nbytes, plural(fs.nbytes));
      avrdude_message(MSG_NOTICE2, "%s: reading on-chip %s%s data ...\n",
        progname, mem->desc, alias_mem_desc);
    }

    report_progress (0,1,"Reading");
    rc = avr_read(pgm, p, upd->memtype, v);
    report_progress (1,1,NULL);
    if (rc < 0) {
      avrdude_message(MSG_INFO, "%s: failed to read all of %s%s memory, rc=%d\n",
        progname, mem->desc, alias_mem_desc, rc);
      pgm->err_led(pgm, ON);
      avr_free_part(v);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    if (quell_progress < 2)
      avrdude_message(MSG_NOTICE2, "%s: verifying ...\n", progname);

    rc = avr_verify(p, v, upd->memtype, size);
    if (rc < 0) {
      avrdude_message(MSG_INFO, "%s: verification error; content mismatch\n",
        progname);
      pgm->err_led(pgm, ON);
      avr_free_part(v);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    if (quell_progress < 2) {
      int verified = fs.nbytes+fs.ntrailing;
      avrdude_message(MSG_INFO, "%s: %d byte%s of %s%s verified\n",
        progname, verified, plural(verified), mem->desc, alias_mem_desc);
    }

    pgm->vfy_led(pgm, OFF);
    avr_free_part(v);
    break;

  default:
    avrdude_message(MSG_INFO, "%s: invalid update operation (%d) requested\n",
      progname, upd->op);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return LIBAVRDUDE_SUCCESS;
}
