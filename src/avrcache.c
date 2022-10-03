/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022 Stefan Rueger <stefan.rueger@urclocks.c>
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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "avrdude.h"
#include "libavrdude.h"

// EEPROM and flash cache for byte-wise access (used in terminal, eg)

/*
 * Paged access?
 *  - Programmer must have paged routines
 *  - Memory has positive page size, which is a power of two
 *  - Memory has positive size, which is a multiple of the page size
 *  - Memory is flash type or eeprom type
 */
int avr_has_paged_access(const PROGRAMMER *pgm, const AVRMEM *mem) {
  return pgm->paged_load && pgm->paged_write &&
         mem->page_size > 0 && (mem->page_size & (mem->page_size-1)) == 0 &&
         mem->size > 0 && mem->size % mem->page_size == 0 &&
         (avr_mem_is_flash_type(mem) || avr_mem_is_eeprom_type(mem));
}


/*
 * Read the page containing addr from the device into buf
 *   - Caller to ensure buf has mem->page_size bytes
 *   - Part memory buffer mem is unaffected by this (though temporarily changed)
 */
int avr_read_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *buf) {
  if(!avr_has_paged_access(pgm, mem) || addr < 0 || addr >= mem->size)
    return LIBAVRDUDE_GENERAL_FAILURE;

  int rc, pgsize = mem->page_size, off = addr & ~(pgsize-1);
  unsigned char *pagecopy = cfg_malloc("avr_read_page_default()", pgsize);

  memcpy(pagecopy, mem->buf + off, pgsize);
  if((rc = pgm->paged_load(pgm, p, mem, pgsize, off, pgsize)) >= 0)
    memcpy(buf, mem->buf + off, pgsize);
  memcpy(mem->buf + off, pagecopy, pgsize);
  free(pagecopy);

  return rc;
}


/*
 * Write the data page to the device into the page containing addr
 *   - Caller to provide all mem->page_size bytes incl padding if any
 *   - Part memory buffer mem is unaffected by this (though temporarily changed)
 */
int avr_write_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *data) {
  if(!avr_has_paged_access(pgm, mem) || addr < 0 || addr >= mem->size)
    return LIBAVRDUDE_GENERAL_FAILURE;

  int rc, pgsize = mem->page_size, off = addr & ~(pgsize-1);
  unsigned char *pagecopy = cfg_malloc("avr_write_page_default()", pgsize);

  memcpy(pagecopy, mem->buf + off, pgsize);
  memcpy(mem->buf + off, data, pgsize);
  rc = pgm->paged_write(pgm, p, mem, pgsize, off, pgsize);
  memcpy(mem->buf + off, pagecopy, pgsize);
  free(pagecopy);

  return rc;
}


// Could memory region s1 be the result of a NOR-memory copy of s3 onto s2?
int avr_is_and(const unsigned char *s1, const unsigned char *s2, const unsigned char *s3, size_t n) {
  while(n--)
    if(*s1++ != (*s2++ & *s3++))
      return 0;

  return 1;
}


static int loadCachePage(AVR_Cache *cp, const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, int level) {
  // Init cache if needed
  if(!cp->cont) {
    cp->cont = cfg_malloc("loadCachePage()", mem->size);
    cp->copy = cfg_malloc("loadCachePage()", mem->size);
    cp->iscached = cfg_malloc("loadCachePage()", mem->size/mem->page_size);
  }

  int pgno = addr/mem->page_size;
  if(!cp->iscached[pgno]) {
    // Read cached section from device
    int base = addr & ~(mem->page_size-1);
    if(avr_read_page_default(pgm, p, mem, base, cp->cont + base) < 0) {
      if(level != MSG_INFO)
        avrdude_message(level, "%s: ", progname);
      avrdude_message(level, "loadCachePage() %s read failed at addr %x\n", mem->desc, addr);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    // Copy so we can later check for changes
    memcpy(cp->copy + base, cp->cont + base, mem->page_size);
    cp->iscached[pgno] = 1;
  }

  return LIBAVRDUDE_SUCCESS;
}


static int writeCachePage(AVR_Cache *cp, const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int base, int level) {
  // Write modified page cont to device
  if(avr_write_page_default(pgm, p, mem, base, cp->cont + base) < 0) {
    if(level != MSG_INFO)
      avrdude_message(level, "%s: ", progname);
    avrdude_message(level, "writeCachePage() %s write error at addr %x\n", mem->desc, base);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }
  // Read page back from device and update copy to what is on device
  if(avr_read_page_default(pgm, p, mem, base, cp->copy + base) < 0) {
    if(level != MSG_INFO)
      avrdude_message(level, "%s: ", progname);
    avrdude_message(level, "writeCachePage() %s read error at addr %x\n", mem->desc, base);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return LIBAVRDUDE_SUCCESS;
}


// Does the memory region only haxe 0xff?
static int _is_all_0xff(const void *p, size_t n) {
  const unsigned char *q = (const unsigned char *) p;
  return n <= 0 || (*q == 0xff && memcmp(q, q+1, n-1) == 0);
}

// Write both EEPROM and flash caches to device and free them
int avr_flush_cache(const PROGRAMMER *pgm, const AVRPART *p) {
  int chpages = 0, wrpages = 0;
  bool chiperase = 0;
  const char *mems[2] = { "flash", "eeprom" };

  // First count pages that were changed
  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = avr_locate_mem(p, mems[i]);
    AVR_Cache *cp = strcmp(mems[i], "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;
    if(!mem || !cp->cont)
      continue;

    for(int pgno = 0, n = 0; n < mem->size; pgno++, n += mem->page_size) {
      if(cp->iscached[pgno])
        if(memcmp(cp->copy + n, cp->cont + n, mem->page_size))
          chpages++;
    }
  }

  if(!chpages)
    return LIBAVRDUDE_SUCCESS;

  avrdude_message(MSG_INFO, "%s: writing cache to device... ", progname);
  fflush(stderr);

  // Then scan for pages that want to set a zero bit (problematic in NOR memory)
  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = avr_locate_mem(p, mems[i]);
    AVR_Cache *cp = strcmp(mems[i], "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;
    if(chiperase || !mem || !cp->cont)
      continue;
    for(int pgno = 0, n = 0; n < mem->size; pgno++, n += mem->page_size) {
      if(cp->iscached[pgno]) {
        if(!avr_is_and(cp->cont + n, cp->copy + n, cp->cont + n, mem->page_size)) {
          // Erase page if possible
          if(pgm->page_erase && pgm->page_erase(pgm, p, mem, n) >= 0)
            memset(cp->copy + n, 0xff, mem->page_size);
          // Still need to update the device page after it's been erased?
          if(memcmp(cp->copy + n, cp->cont + n, mem->page_size)) {
            // Can page erase or the programmer write w/o chip erase?
            if(writeCachePage(cp, pgm, p, mem, n, MSG_INFO) < 0)
              return LIBAVRDUDE_GENERAL_FAILURE;
            // Still different? Needs a chip erase then
            if(memcmp(cp->copy + n, cp->cont + n, mem->page_size)) {
              chiperase = 1;
              break;
            }
            wrpages++;
            if(chpages > 42 && wrpages == 1) {
              avrdude_message(MSG_INFO, "this may take some time ... ");
              fflush(stderr);
            }
          }
        }
      }
    }
  }

  if(!chiperase && chpages > 42 && wrpages == 0) {
    avrdude_message(MSG_INFO, "this may take some time ... ");
    fflush(stderr);
  }

  if(chiperase) {
    avrdude_message(MSG_INFO, "chip erase needed; this will take some time ... ");
    fflush(stderr);

    // Read full flash and EEPROM
    for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
      AVRMEM *mem = avr_locate_mem(p, mems[i]);
      AVR_Cache *cp = strcmp(mems[i], "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;
      if(mem)
        for(int addr = 0; addr < mem->size; addr += mem->page_size)
          if(loadCachePage(cp, pgm, p, mem, addr, MSG_INFO) < 0)
            return LIBAVRDUDE_GENERAL_FAILURE;
    }

    // Erase chip
    if(avr_chip_erase(pgm, p) < 0) {
      avrdude_message(MSG_INFO, "avr_flush_cache() chip erase failed\n");
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    // Update cache copies
    AVRMEM *flm = avr_locate_mem(p, "flash");
    if(flm)
      memset(pgm->cp_flash->copy, 0xff, flm->size);
    // Don't know whether chip erase zaps EEPROM: read from device until hitting data
    AVRMEM *eem = avr_locate_mem(p, "eeprom");
    if(eem) {
      for(int addr = 0; addr < eem->size; addr += eem->page_size) {
        if(!_is_all_0xff(pgm->cp_eeprom->copy + addr, eem->page_size)) { // Did this page have EEPROM data?
          if(avr_read_page_default(pgm, p, eem, addr, pgm->cp_eeprom->copy + addr) < 0) {
            avrdude_message(MSG_INFO, "EEPROM read failed at addr %x\n", addr);
            return LIBAVRDUDE_GENERAL_FAILURE;
          }
          // EEPROM unaffected by chip erase? Stop reading
          if(!_is_all_0xff(pgm->cp_eeprom->copy + addr, eem->page_size))
            break;
        }
      }
    }
  }

  // OK, now should be able to write all modified pages to the device w/o page erase
  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = avr_locate_mem(p, mems[i]);
    AVR_Cache *cp = strcmp(mems[i], "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;
    if(!mem || !cp->cont)
      continue;
    for(int pgno = 0, n = 0; n < mem->size; pgno++, n += mem->page_size) {
      if(cp->iscached[pgno] && memcmp(cp->copy + n, cp->cont + n, mem->page_size)) {
        if(writeCachePage(cp, pgm, p, mem, n, MSG_INFO) < 0)
          return LIBAVRDUDE_GENERAL_FAILURE;
        if(memcmp(cp->copy + n, cp->cont + n, mem->page_size)) {
          avrdude_message(MSG_INFO, "%s verification error at addr %x\n", mem->desc, n);
          return LIBAVRDUDE_GENERAL_FAILURE;
        }
      }
    }
  }

  avrdude_message(MSG_INFO, "done\n");
  return LIBAVRDUDE_SUCCESS;
}


/*
 * Read byte via a read/write cache
 *  - Used if paged routines available and if memory is EEPROM or flash
 *  - Otherwise fall back to pgm->read_byte()
 *  - Out of memory addr: synchronise cache with device and return whether successful
 *  - Cache is automagically created and initialised if needed
 */
int avr_read_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char *value) {

  // Use pgm->read_byte() if not EEPROM/flash or no paged access
  if(!avr_has_paged_access(pgm, mem))
    return pgm->read_byte(pgm, p, mem, addr, value);

  // If address is out of range synchronise caches, and if successful pretend reading a zero
  if(addr >= (unsigned long) mem->size) {
    if(avr_flush_cache(pgm, p) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;
    *value = 0;
    return LIBAVRDUDE_SUCCESS;
  }

  AVR_Cache *cp = avr_mem_is_eeprom_type(mem)? pgm->cp_eeprom: pgm->cp_flash;

  // Ensure cache page is there
  if(loadCachePage(cp, pgm, p, mem, addr, MSG_NOTICE) < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  *value = cp->cont[(int)addr];

  return LIBAVRDUDE_SUCCESS;
}


/*
 * Write byte via a read/write cache
 *  - Used if paged routines available and if memory is EEPROM or flash
 *  - Otherwise fall back to pgm->write_byte()
 *  - Out of memory addr: synchronise cache with device and return whether successful
 *  - Cache is automagically created and initialised if needed
 */
int avr_write_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char data) {

  // Use pgm->read_byte() if not EEPROM/flash or no paged access
  if(!avr_has_paged_access(pgm, mem))
    return pgm->write_byte(pgm, p, mem, addr, data);

  // If address is out of range synchronise caches with device and return whether successful
  if(addr >= (unsigned long) mem->size)
    return avr_flush_cache(pgm, p);

  AVR_Cache *cp = avr_mem_is_eeprom_type(mem)? pgm->cp_eeprom: pgm->cp_flash;

  // Ensure cache page is there
  if(loadCachePage(cp, pgm, p, mem, addr, MSG_NOTICE) < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  cp->cont[(int)addr] = data;

  return LIBAVRDUDE_SUCCESS;
}


// Erase the chip and set the cache accordingly
int avr_chip_erase_cached(const PROGRAMMER *pgm, const AVRPART *p) {
  const char *mems[2] = { "flash", "eeprom" };

  if(pgm->chip_erase(pgm, p) < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = avr_locate_mem(p, mems[i]);
    AVR_Cache *cp = strcmp(mems[i], "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;

    if(!mem || !avr_has_paged_access(pgm, mem))
      continue;

    if(!cp->cont) {
      cp->cont = cfg_malloc("avr_chip_erase_cache()", mem->size);
      cp->copy = cfg_malloc("avr_chip_erase_cache()", mem->size);
      cp->iscached = cfg_malloc("avr_chip_erase_cache()", mem->size/mem->page_size);
    }

    if(i == 0) { // flash
      // set all to erased
      memset(cp->copy, 0xff, mem->size);
      memset(cp->cont, 0xff, mem->size);
      memset(cp->iscached, 1, mem->size/mem->page_size);
    } else { // EEPROM: Don't know whether chip erase zaps it, so test cached pages
      bool erasedee = 0;
      for(int pgno = 0, n = 0; n < mem->size; pgno++, n += mem->page_size) {
        if(cp->iscached[pgno]) {
          if(!_is_all_0xff(cp->copy + n, mem->page_size)) { // Did this page have EEPROM data?
            if(avr_read_page_default(pgm, p, mem, n, cp->copy + n) < 0)
              return LIBAVRDUDE_GENERAL_FAILURE;
            erasedee = _is_all_0xff(cp->copy + n, mem->page_size);
            break;
          }
        }
      }
      if(erasedee) { // Have evidence that chip erase also erased EEPROM, set cache correspondingly
        memset(cp->copy, 0xff, mem->size);
        memset(cp->cont, 0xff, mem->size);
        memset(cp->iscached, 1, mem->size/mem->page_size);
      } // else? no evidence for EEPROM was erased, so leave data in cont
    }
  }

  return LIBAVRDUDE_SUCCESS;
}


// Free cache(s)
int avr_reset_cache(const PROGRAMMER *pgm, const AVRPART *p) {
  const char *mems[2] = { "flash", "eeprom" };

  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVR_Cache *cp = strcmp(mems[i], "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;
    if(cp->cont)
      free(cp->cont);
    if(cp->copy)
      free(cp->copy);
    if(cp->iscached)
      free(cp->iscached);
    memset(cp, 0, sizeof*cp);
  }

  return LIBAVRDUDE_SUCCESS;
}
