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

// FIXME: check out n_page_erase (some parts have larger page size for erase than for write)

static void setCacheSize(AVR_Cache *cp, const AVRMEM *mem) {
  cp->size = 256;
  if(cp->size < mem->page_size)
    cp->size = mem->page_size;
}


// Return the address of the page that contains addr
static int baseaddr(AVR_Cache *cp, const AVRMEM *mem, int addr) {
  if(cp->base < 0 || !cp->page || !cp->copy || cp->size < 1)
    setCacheSize(cp, mem);

  return addr - addr%cp->size;
}


/*
 * Paged access?
 *  - Programmer must have paged routines and
 *  - Memory a positive page size, which is a power of two
 */
int avr_has_paged_access(const PROGRAMMER *pgm, const AVRMEM *m) {
  return pgm->paged_load && pgm->paged_write && m->page_size > 0 &&
         (m->page_size & (m->page_size-1)) == 0;
}


/*
 * Read the page containing addr from the device into buf
 *   - Caller to ensure buf has mem->size bytes
 *   - Part memory buffer mem is unaffected by this (though temporarily changed)
 */
int avr_read_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *buf) {
  if(!avr_has_paged_access(pgm, mem) || addr < 0 || addr >= mem->size)
    return LIBAVRDUDE_GENERAL_FAILURE;

  int rc, pgsize = mem->page_size, off = addr & ~(pgsize-1);
  unsigned char *pagecopy = cfg_malloc("avr_read_page_default()", pgsize);

  memcpy(pagecopy, mem->buf+off, pgsize);
  if((rc = pgm->paged_load(pgm, p, mem, pgsize, off, pgsize)) >= 0)
    memcpy(buf, mem->buf+off, pgsize);
  memcpy(mem->buf+off, pagecopy, pgsize);
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

  memcpy(pagecopy, mem->buf+off, pgsize);
  memcpy(mem->buf+off, data, pgsize);
  rc = pgm->paged_write(pgm, p, mem, pgsize, off, pgsize);
  memcpy(mem->buf+off, pagecopy, pgsize);
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


// Expand the given cache to cover the full memory region of memtype == "flash" or "eeprom"
int avr_expand_cache(const PROGRAMMER *pgm, const AVRPART *p, const char *memtype) {
  AVRMEM *m, *mem;

  if(!(m = avr_locate_mem(p, memtype))) {
    avrdude_message(MSG_NOTICE, "%s: avr_expand_cache() cannot locate memory %s\n", progname, memtype);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  AVR_Cache *cp = strcmp(memtype, "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;

  mem = avr_dup_mem(m);
  if(avr_read_mem(pgm, p, mem, NULL) < 0) {
    avr_free_mem(mem);
    avrdude_message(MSG_NOTICE, "%s: avr_expand_cache() cannot read memory %s\n", progname, memtype);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }
  unsigned char *newpage, *newcopy;
  newpage = cfg_malloc("avr_expand_cache()", mem->size);
  newcopy = cfg_malloc("avr_expand_cache()", mem->size);
  memcpy(newpage, mem->buf, mem->size);
  memcpy(newcopy, newpage, mem->size);
  avr_free_mem(mem);

  // If cache existed, copy over the existing cache, then free it
  if(cp->base >= 0 && cp->page && cp->copy && cp->size > 0) {
    if(cp->base+cp->size <= mem->size)
      memcpy(newpage+cp->base, cp->page, cp->size);
    free(cp->page);
    free(cp->copy);
  }

  cp->base = 0;
  cp->size = mem->size;
  cp->flags &= ~CACHE_USE_PAGE_ERASE;
  cp->flags |= CACHE_FULL_DEVICE;
  cp->page = newpage;
  cp->copy = newcopy;

  return LIBAVRDUDE_SUCCESS;
}


// Sync the given full-memory cache to the device, memtype == "flash" or "eeprom"
int avr_sync_cache(const PROGRAMMER *pgm, const AVRPART *p, const char *memtype) {
  AVRMEM *m, *mem;

  if(!(m = avr_locate_mem(p, memtype))) {
    avrdude_message(MSG_NOTICE, "%s: avr_sync_cache() cannot locate memory %s\n", progname, memtype);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  AVR_Cache *cp = strcmp(memtype, "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;

  mem = avr_dup_mem(m);
  // Set buf and tags of the memory to reflect the cache
  memcpy(mem->buf, cp->page, cp->size);
  for(int i=0; i < cp->size; i++)
    mem->tags[i] = cp->page[i] == cp->copy[i]? 0: TAG_ALLOCATED;

  // Write memory to the device
  if(avr_write_mem(pgm, p, mem, cp->size, 0) < 0) {
    avrdude_message(MSG_NOTICE, "%s: avr_sync_cache() cannot write memory %s\n", progname, memtype);
    avr_free_mem(mem);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  // Read memory back from device and update copy of cache contents
  if(avr_read_mem(pgm, p, mem, NULL) < 0) {
    avrdude_message(MSG_NOTICE, "%s: avr_sync_cache() cannot read memory %s\n", progname, memtype);
    avr_free_mem(mem);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }
  memcpy(cp->copy, mem->buf, cp->size);

  // Verify cache against device readback
  for(int i=0; i < cp->size; i++)
    if(mem->buf[i] != cp->page[i]) {
      avrdude_message(MSG_NOTICE, "%s: avr_sync_cache() verification error for %s\n",
        progname, memtype);
      avrdude_message(MSG_NOTICE, "%*s first mismatch at 0x04x, device = %02x, cache = %02x\n",
        strlen(progname)+1, "", i, mem->buf[i], cp->page[i]);
      avr_free_mem(mem);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

  avr_free_mem(mem);
  return LIBAVRDUDE_SUCCESS;
}



/*
 * writeCache(cp, pgm, p, mem)
 *
 * Writes the normally small cache to the device using paged write for those
 * pages that were changed. Some memories look like NOR memories, ie, they
 * can only write 0 bits, not 1 bits, so in this case either page erase needs
 * to be deployed or, if that is not available, both EEPROM and flash caches
 * are expanded to cover the full device, no matter which cache writeCache()
 * was called with. In this case writeCache() does not try to immediately
 * erase the chip and write back the contents, as it could well be that the
 * user intended more modifications in other chip areas; writeCache() returns
 * LIBAVRDUDE_SOFTFAIL instead and marks in both EEPROM and flash caches that
 * now both cached are considered jointly on the next call to writeCache()
 * when a chip erase will be issued and both flash and EEPROM are
 * synchronised with the device.
 *
 * Possible return values
 *
 *   - LIBAVRDUDE_SUCCESS: All good
 *
 *   - LIBAVRDUDE_GENERAL_FAILURE: Failed, the device has the wrong contents
 *
 *   - LIBAVRDUDE_SOFTFAIL: Expanded both caches to the full device;
 *       writeCache() needs calling again to actually sync with device
 */

static int writeCache(AVR_Cache *cp, const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem) {
  // Cache not in use? All good
  if(cp->base < 0 || !cp->page || !cp->copy || cp->size < 1)
    return LIBAVRDUDE_SUCCESS;

  if(!avr_has_paged_access(pgm, mem)) {
    avrdude_message(MSG_NOTICE, "%s: writeCache() called for memory without paged access\n", progname);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(cp->flags & CACHE_FULL_DEVICE) { // Need to treat both EEPROM and flash
    // Has the cache been modified? If not all is good
    if(!memcmp(pgm->cp_flash->copy, pgm->cp_flash->page, pgm->cp_flash->size) &&
       !memcmp(pgm->cp_eeprom->copy, pgm->cp_eeprom->page, pgm->cp_eeprom->size))
      return LIBAVRDUDE_SUCCESS;

    avrdude_message(MSG_INFO, "%s: erasing chip and writing caches to all flash and EEPROM (takes time)...\n", progname);

    // Erase chip
    if(avr_chip_erase(pgm, p) < 0) {
      avrdude_message(MSG_NOTICE, "%s: writeCache() chip erase failed\n", progname);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    memset(pgm->cp_flash->copy, 0xff, pgm->cp_flash->size);
    // Don't know whether chip erase will have erased EEPROM: read from device
    if(avr_expand_cache(pgm, p, "eeprom") < 0) {
      avrdude_message(MSG_NOTICE, "%s: writeCache() EEPROM read failed\n", progname);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    int rc = LIBAVRDUDE_SUCCESS;
    if(avr_sync_cache(pgm, p, "flash") < 0) {
      avrdude_message(MSG_NOTICE, "%s: writeCache() cannot synchronise flash to device\n", progname);
      rc = LIBAVRDUDE_GENERAL_FAILURE;
    }
    if(avr_sync_cache(pgm, p, "eeprom") < 0) {
      avrdude_message(MSG_NOTICE, "%s: writeCache() cannot synchronise EEPROM to device\n", progname);
      rc = LIBAVRDUDE_GENERAL_FAILURE;
    }

    return rc;
  }

  for(int pgsize = mem->page_size, n = 0; n < cp->size; n += pgsize) {
    if(memcmp(cp->copy+n, cp->page+n, pgsize)) { // Page needs copying
      int page_erased = 0;

      // Erase the page if can do and needed
      if((cp->flags & CACHE_USE_PAGE_ERASE) && pgm->page_erase)
        if(!avr_is_and(cp->page+n, cp->copy+n, cp->page+n, pgsize)) {
          if(pgm->page_erase(pgm, p, mem, cp->base+n) < 0) {
            avrdude_message(MSG_NOTICE, "%s: writeCache() page erase error (b)\n", progname);
            return LIBAVRDUDE_GENERAL_FAILURE;
          }
          page_erased = 1;
        }

      // Write page to device
      if(avr_write_page_default(pgm, p, mem, cp->base+n, cp->page+n) < 0) {
        avrdude_message(MSG_NOTICE, "%s: writeCache() page write error\n", progname);
        return LIBAVRDUDE_GENERAL_FAILURE;
      }

      // Read page back from device
      unsigned char *devmem = cfg_malloc("writeCache()", pgsize);
      if(avr_read_page_default(pgm, p, mem, cp->base+n, devmem) < 0) {
        avrdude_message(MSG_NOTICE, "%s: writeCache() page read error (b)\n", progname);
        free(devmem);
        return LIBAVRDUDE_GENERAL_FAILURE;
      }

      if(memcmp(devmem, cp->page+n, pgsize)) { // Written page is different
        if(avr_is_and(devmem, cp->copy+n, cp->page+n, pgsize)) { // OK, prob NOR-memory
          if(!page_erased && pgm->page_erase) {
            if(pgm->page_erase(pgm, p, mem, cp->base+n) < 0)
              avrdude_message(MSG_NOTICE, "%s: writeCache() page erase error (a)\n", progname);
            // Write and verify page again
            if(avr_write_page_default(pgm, p, mem, cp->base+n, cp->page+n) < 0 ||
               avr_read_page_default(pgm, p, mem, cp->base+n, devmem) < 0) {
              avrdude_message(MSG_NOTICE, "%s: writeCache() page write/read error\n", progname);
              free(devmem);
              return LIBAVRDUDE_GENERAL_FAILURE;
            }
            if(memcmp(devmem, cp->page+n, pgsize)) { // Still different?
              avrdude_message(MSG_NOTICE, "%s: writeCache() verification error after page erase and write\n", progname);
              free(devmem);
              return LIBAVRDUDE_GENERAL_FAILURE;
            }
            // Page erase did the trick, do that in future straight away
            cp->flags |= CACHE_USE_PAGE_ERASE;
          } else { // Page erase not available or failed, so cache all flash and EEPROM
            avrdude_message(MSG_INFO, "%s: expanding cache to all flash and EEPROM (takes time)...\n", progname);
            if(avr_expand_cache(pgm, p, "flash") < 0)
              return LIBAVRDUDE_GENERAL_FAILURE;
            if(avr_expand_cache(pgm, p, "eeprom") < 0)
              return LIBAVRDUDE_GENERAL_FAILURE;
            // Needs a second call to writeCache() to write back the chip contents
            free(devmem);
            return LIBAVRDUDE_SOFTFAIL;
          }
        } else {
          avrdude_message(MSG_NOTICE, "%s: writeCache() verification error\n", progname);
          free(devmem);
          return LIBAVRDUDE_GENERAL_FAILURE;
        }
      }

      free(devmem);
      // All good, update copy of cache contents
      memcpy(cp->copy+n, cp->page+n, pgsize);
    }
  }

  return LIBAVRDUDE_SUCCESS;
}


static int readCache(AVR_Cache *cp, const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int base) {
  if(!avr_has_paged_access(pgm, mem)) {
    avrdude_message(MSG_NOTICE, "%s: readCache() called for memory without paged access\n", progname);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  // First save what's cached to the device if needed
  if(writeCache(cp, pgm, p, mem) == LIBAVRDUDE_GENERAL_FAILURE)
    return LIBAVRDUDE_GENERAL_FAILURE;

  // Init cache if needed
  if(!cp->page || !cp->copy || cp->size < 1) {
    cp->base = -1;
    setCacheSize(cp, mem);
    cp->flags = 0;
    cp->page = cfg_malloc("readCache()", cp->size);
    cp->copy = cfg_malloc("readCache()", cp->size);
  }

  // Read cached section from device
  for(int n = 0; n < cp->size; n += mem->page_size)
    if(avr_read_page_default(pgm, p, mem, base + n, cp->page + n) < 0) {
      avrdude_message(MSG_NOTICE, "%s: readCache() %s read failed\n", progname, mem->desc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

  // Copy so we can later check for changes
  memcpy(cp->copy, cp->page, cp->size);

  // And update the base of the freshly cached data
  cp->base = base;

  return LIBAVRDUDE_SUCCESS;
}


// Write both EEPROM and flash caches to device and free them
int avr_flush_cache(const PROGRAMMER *pgm, const AVRPART *p) {
  int rc = LIBAVRDUDE_SUCCESS;
  const char *mems[2] = { "flash", "eeprom" };

  for(int i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = avr_locate_mem(p, mems[i]);
    AVR_Cache *cp = strcmp(mems[i], "flash") == 0? pgm->cp_flash: pgm->cp_eeprom;

    if(mem) {
      int rr = writeCache(cp, pgm, p, mem);
      if(rr == LIBAVRDUDE_SOFTFAIL)
        rr =  writeCache(cp, pgm, p, mem);
      if(rr < 0)
        rc = LIBAVRDUDE_GENERAL_FAILURE;
    } else
      rc = LIBAVRDUDE_GENERAL_FAILURE;

    // Free cache
    if(cp->page)
     free(cp->page);
    if(cp->copy)
     free(cp->copy);
    memset(cp, 0, sizeof*cp);
  }

  return rc;
}


/*
 * Read byte via a read/write cache
 *  - Used if paged routines available and if memory is EEPROM or flash
 *  - Otherwise fall back to pgm->read_byte()
 *  - Out of memory addr: call writeCache() and pretend reading a 0 correctly
 *  - Out of cache addr: call writeCache(), then readCache() for addr, return byte
 *  - Cache is automagically created and initialised if needed
 *  - User should call avr_flush_cache() to write cache to device and free it
 */
int avr_read_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char *value) {

  int isflash  = avr_mem_is_flash_type(mem);
  int iseeprom = avr_mem_is_eeprom_type(mem);

  // Use pgm->read_byte() if not EEPROM/flash or no paged access
  if((!isflash && !iseeprom) || !avr_has_paged_access(pgm, mem))
    return pgm->read_byte(pgm, p, mem, addr, value);

  AVR_Cache *cp = isflash? pgm->cp_flash: pgm->cp_eeprom;

  // If address is out of range write cache (if any) and pretend reading a zero
  if(addr >= (unsigned long) mem->size) {
    writeCache(cp, pgm, p, mem);
    *value = 0;
    return LIBAVRDUDE_SUCCESS;
  }

  int base = baseaddr(cp, mem, addr);
  // If no cache set up or address not in cached range, initialise cache
  if(cp->base < 0 || !cp->page || !cp->copy || cp->base != base)
    if(readCache(cp, pgm, p, mem, base) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

  *value = cp->page[(int)addr-base];

  return LIBAVRDUDE_SUCCESS;
}


/*
 * Write byte via a read/write cache
 *  - Used if paged routines available and if memory is EEPROM or flash
 *  - Otherwise fall back to pgm->write_byte()
 *  - Out of memory addr: call writeCache() and pretend writing correctly
 *  - Out of cache addr: call writeCache(), then readCache() to write to addr
 *  - Cache is automagically created and initialised if needed
 *  - User should call avr_flush_cache() to write cache to device and free it
 */
int avr_write_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char data) {

  int isflash  = avr_mem_is_flash_type(mem);
  int iseeprom = avr_mem_is_eeprom_type(mem);

  // Use pgm->write_byte() if not EEPROM/flash or no paged access
  if((!isflash && !iseeprom) || !avr_has_paged_access(pgm, mem))
    return pgm->write_byte(pgm, p, mem, addr, data);

  AVR_Cache *cp = isflash? pgm->cp_flash: pgm->cp_eeprom;

  // If address is out of range write cache (if any) and pretend successfully written
  if(addr >= (unsigned long) mem->size) {
    writeCache(cp, pgm, p, mem);
    return LIBAVRDUDE_SUCCESS;
  }

  int base = baseaddr(cp, mem, addr);
  // If no cache set up or address not in cached range, initialise cache
  if(cp->base < 0 || !cp->page || !cp->copy || cp->base != base)
    if(readCache(cp, pgm, p, mem, base) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

  cp->page[(int)addr-base] = data;

  return LIBAVRDUDE_SUCCESS;
}
