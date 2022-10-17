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
#include "avrintel.h"

/*
 * Provides an API for cached bytewise access
 *
 * int avr_read_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const
 *   AVRMEM *mem, unsigned long addr, unsigned char *value);
 *
 * int avr_write_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const
 *  AVRMEM *mem, unsigned long addr, unsigned char data);
 *
 * int avr_flush_cache(const PROGRAMMER *pgm, const AVRPART *p);
 *
 * int avr_chip_erase_cached(const PROGRAMMER *pgm, const AVRPART *p);
 *
 * int avr_page_erase_cached(const PROGRAMMER *pgm, const AVRPART *p, const
 *  AVRMEM *mem, unsigned int baseaddr);
 *
 * int avr_reset_cache(const PROGRAMMER *pgm, const AVRPART *p);
 *
 * avr_read_byte_cached() and avr_write_byte_cached() use a cache if paged
 * routines are available and if the device memory is EEPROM or flash,
 * otherwise they fall back to pgm->read_byte() and pgm->write_byte(),
 * respectively. Bytewise cached read always gets its data from the cache,
 * possibly after reading a page from the device memory. Bytewise cached
 * write with an address in memory range only ever modifies the cache. Any
 * modifications are written to the device after calling avr_flush_cache() or
 * when attempting to read or write from a location outside the address range
 * of the device memory.
 *
 * avr_flush_cache() synchronises pending writes to EEPROM and flash with the
 * device. With some programmer and part combinations, flash (and sometimes
 * EEPROM, too) looks like a NOR memory, ie, one can only write 0 bits, not 1
 * bits. When this is detected, either page erase is deployed (eg, with parts
 * that have PDI/UPDI interfaces), or if that is not available, both EEPROM
 * and flash caches are fully read in, a pgm->chip_erase() command is issued
 * and both EEPROM and flash are written back to the device. Hence, it can
 * take minutes to ensure that a single previously cleared bit is set and,
 * therefore, this routine should be called sparingly.
 *
 * avr_chip_erase_cached() erases the chip and discards pending writes() to
 * flash or EEPROM. It presets the flash cache to all 0xff alleviating the
 * need to read from the device flash. However, if the programmer serves
 * bootloaders (pgm->prog_modes & PM_SPM) then the flash cache is reset
 * instead, necessitating flash memory be fetched from the device on first
 * read; the reason for this is that bootloaders emulate chip erase and they
 * won't overwrite themselves (some bootloaders, eg, optiboot ignore chip
 * erase commands) making it truly unknowable what the flash contents on
 * device is after a chip erase.
 *
 * For EEPROM avr_chip_erase_cached() concludes that it has been deleted if a
 * previously cached EEPROM page that contained cleared bits now no longer
 * has these clear bits on the device. Only with this evidence is the EEPROM
 * cache preset to all 0xff otherwise the cache discards all pending writes
 * to EEPROM and is left unchanged otherwise.
 *
 * The avr_page_erase_cached() function erases a page and synchronises it
 * with the cache.
 *
 * Finally, avr_reset_cache() resets the cache without synchronising pending
 * writes() to the device.
 *
 * This file also holds the following utility functions
 *
 * // Does the programmer/memory combo have paged memory access?
 * int avr_has_paged_access(const PROGRAMMER *pgm, const AVRMEM *mem);
 *
 * // Read the page containing addr from the device into buf
 * int avr_read_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *buf);
 *
 * // Write the data page to the device into the page containing addr
 * int avr_write_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *data);
 *
 * // Could memory region s1 be the result of a NOR-memory copy of s3 onto s2?
 * int avr_is_and(const unsigned char *s1, const unsigned char *s2, const unsigned char *s3, size_t n);
 *
 */


/*
 * Paged access?
 *  - Programmer must have paged routines
 *  - Memory has positive page size, which is a power of two
 *  - Memory has positive size, which is a multiple of the page size
 *  - Memory is flash type or eeprom type
 *
 * Note that in this definition the page size can be 1
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
 *   - Uses read_byte() if memory page size is one, otherwise paged_load()
 *   - Fall back to bytewise read if paged_load() returned an error
 */
int avr_read_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *buf) {
  if(!avr_has_paged_access(pgm, mem) || addr < 0 || addr >= mem->size)
    return LIBAVRDUDE_GENERAL_FAILURE;

  int rc, pgsize = mem->page_size, base = addr & ~(pgsize-1);
  unsigned char *pagecopy = cfg_malloc("avr_read_page_default()", pgsize);

  if(pgsize == 1)
    return pgm->read_byte(pgm, p, mem, addr, buf);

  memcpy(pagecopy, mem->buf + base, pgsize);
  if((rc = pgm->paged_load(pgm, p, mem, pgsize, base, pgsize)) >= 0)
    memcpy(buf, mem->buf + base, pgsize);
  memcpy(mem->buf + base, pagecopy, pgsize);

  if(rc < 0) {
    rc = LIBAVRDUDE_SUCCESS;
    for(int i=0; i<pgsize; i++) {
      if(pgm->read_byte(pgm, p, mem, base+i, pagecopy+i) < 0) {
        rc = LIBAVRDUDE_GENERAL_FAILURE;
        break;
      }
    }
    if(rc == LIBAVRDUDE_SUCCESS)
      memcpy(buf, pagecopy, pgsize);
  }
  free(pagecopy);

  return rc;
}


/*
 * Write the data page to the device into the page containing addr
 *   - Caller to provide all mem->page_size bytes incl padding if any
 *   - Part memory buffer mem is unaffected by this (though temporarily changed)
 *   - Uses write_byte() if memory page size is one, otherwise paged_write()
 */
int avr_write_page_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, unsigned char *data) {
  if(!avr_has_paged_access(pgm, mem) || addr < 0 || addr >= mem->size)
    return LIBAVRDUDE_GENERAL_FAILURE;

  int rc, pgsize = mem->page_size, base = addr & ~(pgsize-1);
  unsigned char *pagecopy = cfg_malloc("avr_write_page_default()", pgsize);

  if(pgsize == 1)
    return pgm->write_byte(pgm, p, mem, addr, *data);

  memcpy(pagecopy, mem->buf + base, pgsize);
  memcpy(mem->buf + base, data, pgsize);
  rc = pgm->paged_write(pgm, p, mem, pgsize, base, pgsize);
  memcpy(mem->buf + base, pagecopy, pgsize);
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


static int initCache(AVR_Cache *cp, const PROGRAMMER *pgm, const AVRPART *p) {
  AVRMEM *basemem = avr_locate_mem(p, cp == pgm->cp_flash? "flash": "eeprom");

  if(!basemem || !avr_has_paged_access(pgm, basemem))
    return LIBAVRDUDE_GENERAL_FAILURE;

  cp->size = basemem->size;
  cp->page_size = basemem->page_size;
  cp->offset = basemem->offset;
  cp->cont = cfg_malloc("initCache()", cp->size);
  cp->copy = cfg_malloc("initCache()", cp->size);
  cp->iscached = cfg_malloc("initCache()", cp->size/cp->page_size);

  return LIBAVRDUDE_SUCCESS;
}


static int cacheAddress(int addr, const AVR_Cache *cp, const AVRMEM *mem) {
  int cacheaddr = addr + (int) (mem->offset - cp->offset);

  if(cacheaddr < 0 || cacheaddr >= cp->size) { // Should never happen (unless offsets wrong in avrdude.conf)
    pmsg_error("%s cache address 0x%04x out of range [0, 0x%04x]\n", mem->desc, cacheaddr, cp->size);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  if(mem->page_size != cp->page_size) { // Should never happen (unless incompatible page sizes in avrdude.conf)
    pmsg_error("%s page size %d incompatible with cache page size %d\n", mem->desc, mem->page_size, cp->page_size);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return cacheaddr;
}


static int loadCachePage(AVR_Cache *cp, const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int addr, int cacheaddr, int nlOnErr) {
  int pgno = cacheaddr/cp->page_size;

  if(!cp->iscached[pgno]) {
    // Read cached section from device
    int cachebase = cacheaddr & ~(cp->page_size-1);
    if(avr_read_page_default(pgm, p, mem, addr & ~(cp->page_size-1), cp->cont + cachebase) < 0) {
      report_progress(1, -1, NULL);
      if(nlOnErr && quell_progress)
        msg_info("\n");
      pmsg_error("unable to read %s page at addr 0x%04x\n", mem->desc, addr);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    // Copy last read device page, so we can later check for changes
    memcpy(cp->copy + cachebase, cp->cont + cachebase, cp->page_size);
    cp->iscached[pgno] = 1;
  }

  return LIBAVRDUDE_SUCCESS;
}


static int writeCachePage(AVR_Cache *cp, const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, int base, int nlOnErr) {
  // Write modified page cont to device; if unsuccessful try bytewise access
  if(avr_write_page_default(pgm, p, mem, base, cp->cont + base) < 0) {
    for(int i=0; i < cp->page_size; i++)
      if(cp->cont[base+i] != cp->copy[base+i])
        if(pgm->write_byte(pgm, p, mem, base+i, cp->cont[base+i]) < 0 ||
           pgm->read_byte(pgm, p, mem, base+i, cp->copy+base+i) < 0) {
          report_progress(1, -1, NULL);
          if(nlOnErr && quell_progress)
            avrdude_message(MSG_INFO, "\n");
          avrdude_message(MSG_INFO, "%s: writeCachePage() %s access error at addr 0x%04x\n", progname, mem->desc, base+i);
          return LIBAVRDUDE_GENERAL_FAILURE;
        }

    return LIBAVRDUDE_SUCCESS;  // Bytewise writes & reads successful
  }
  // Read page back from device and update copy to what is on device
  if(avr_read_page_default(pgm, p, mem, base, cp->copy + base) < 0) {
    report_progress(1, -1, NULL);
    if(nlOnErr && quell_progress)
      msg_info("\n");
    pmsg_error("unable to read %s page at addr 0x%04x\n", mem->desc, base);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return LIBAVRDUDE_SUCCESS;
}


// Does the memory region only haxe 0xff?
static int _is_all_0xff(const void *p, size_t n) {
  const unsigned char *q = (const unsigned char *) p;
  return n <= 0 || (*q == 0xff && memcmp(q, q+1, n-1) == 0);
}


// A coarse guess where any bootloader might start (prob underestimates the start)
static int guessBootStart(const PROGRAMMER *pgm, const AVRPART *p) {
  int bootstart = 0;
  const AVR_Cache *cp = pgm->cp_flash;

  for(size_t i = 0; i < sizeof uP_table/sizeof*uP_table; i++)
    if(p->mcuid == uP_table[i].mcuid) {
      if(uP_table[i].nboots > 0 && uP_table[i].bootsize > 0 && uP_table[i].flashsize == cp->size)
        bootstart = cp->size - uP_table[i].nboots * uP_table[i].bootsize;
      break;
    }

  if(bootstart <= cp->size/2 || bootstart >= cp->size)
    bootstart = cp->size > 32768? cp->size - 16384: cp->size*3/4;

  return bootstart & ~(cp->page_size-1);
}


typedef struct {
  AVRMEM *mem;
  AVR_Cache *cp;
  int isflash, zopaddr, pgerase;
} CacheDesc_t;


// Write both EEPROM and flash caches to device and free them
int avr_flush_cache(const PROGRAMMER *pgm, const AVRPART *p) {
  CacheDesc_t mems[2] = {
    { avr_locate_mem(p, "flash"), pgm->cp_flash, 1, -1, 0 },
    { avr_locate_mem(p, "eeprom"), pgm->cp_eeprom, 0, -1, 0 },
  };

  int chpages = 0;
  bool chiperase = 0;
  // Count page changes and find a page that needs a clear bit set
  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = mems[i].mem;
    AVR_Cache *cp = mems[i].cp;
    if(!mem || !cp->cont)
      continue;

    for(int pgno = 0, n = 0; n < cp->size; pgno++, n += cp->page_size) {
      if(cp->iscached[pgno])
        if(memcmp(cp->copy + n, cp->cont + n, cp->page_size)) {
          chpages++;
          if(mems[i].zopaddr == -1 && !avr_is_and(cp->cont + n, cp->copy + n, cp->cont + n, cp->page_size))
            mems[i].zopaddr = n;
        }
    }
  }

  if(!chpages)
    return LIBAVRDUDE_SUCCESS;

  pmsg_info("synching cache to device ... ");
  fflush(stderr);

  // Check whether page erase needed and working and whether chip erase needed
  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = mems[i].mem;
    AVR_Cache *cp = mems[i].cp;

    if(!cp->cont)           // Ensure cache is initialised from now on
      if(initCache(cp, pgm, p) < 0) {
        if(quell_progress)
          msg_info("\n");
        pmsg_error("unable to initialise the cache\n");
        return LIBAVRDUDE_GENERAL_FAILURE;
      }

    if(chiperase || !mem || mems[i].zopaddr < 0)
      continue;

    int n=mems[i].zopaddr;

    if(writeCachePage(cp, pgm, p, mem, n, 1) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;
    // Same? OK, can set cleared bit to one, "normal" memory
    if(!memcmp(cp->copy + n, cp->cont + n, cp->page_size)) {
      chpages--;
      continue;
    }

    // Probably NOR memory, check out page erase
    if(pgm->page_erase && pgm->page_erase(pgm, p, mem, n) >= 0) {
      if(writeCachePage(cp, pgm, p, mem, n, 1) < 0)
        return LIBAVRDUDE_GENERAL_FAILURE;
      // Worked OK? Can use page erase on this memory
      if(!memcmp(cp->copy + n, cp->cont + n, cp->page_size)) {
        mems[i].pgerase = 1;
        chpages--;
        continue;
      }
    }

    chiperase = 1;
  }

  if(!chpages) {
    msg_info("done\n");
    return LIBAVRDUDE_SUCCESS;
  }

  if(chiperase) {
    if(quell_progress) {
      msg_info("reading/chip erase/writing cycle needed ... ");
      fflush(stderr);
    }

    int nrd = 0;
    // Count read operations needed
    for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
      AVRMEM *mem = mems[i].mem;
      AVR_Cache *cp = mems[i].cp;
      if(!mem)
        continue;

      for(int pgno = 0, n = 0; n < cp->size; pgno++, n += cp->page_size)
        if(!cp->iscached[pgno])
          nrd++;
    }

    report_progress(0, 1, "Reading");
    if(nrd) {
      // Read full flash and EEPROM
      for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
        AVRMEM *mem = mems[i].mem;
        AVR_Cache *cp = mems[i].cp;
        if(!mem)
          continue;

        for(int ird = 0, pgno = 0, n = 0; n < cp->size; pgno++, n += cp->page_size) {
          if(!cp->iscached[pgno]) {
            report_progress(ird++, nrd, NULL);
            if(loadCachePage(cp, pgm, p, mem, n, n, 1) < 0)
              return LIBAVRDUDE_GENERAL_FAILURE;
          }
        }
      }
    }
    report_progress(1, 0, NULL);

    report_progress(0, 1, "Erasing");
    if(avr_chip_erase(pgm, p) < 0) {
      report_progress(1, -1, NULL);
      if(quell_progress)
        msg_info("\n");
      pmsg_error("chip erase failed\n");
      return LIBAVRDUDE_GENERAL_FAILURE;
    }

    // Update cache copies after chip erase so that writing back is efficient
    for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
      AVRMEM *mem = mems[i].mem;
      AVR_Cache *cp = mems[i].cp;
      if(!mem)
        continue;

      if(mems[i].isflash) {     // flash
        memset(cp->copy, 0xff, cp->size); // record device memory as erased
        if(pgm->prog_modes & PM_SPM) { // Bootloaders will not overwrite themselves
          // Read back generously estimated bootloader section to avoid verification errors
          int bootstart = guessBootStart(pgm, p);
          int nbo = (cp->size - bootstart)/cp->page_size;

          for(int ibo = 0, n = bootstart; n < cp->size; n += cp->page_size) {
            report_progress(1+ibo++, nbo+2, NULL);
            if(avr_read_page_default(pgm, p, mem, n, cp->copy + n) < 0) {
              report_progress(1, -1, NULL);
              if(quell_progress)
                msg_info("\n");
              pmsg_error("flash read failed at addr 0x%04x\n", n);
              return LIBAVRDUDE_GENERAL_FAILURE;
            }
          }
        }
      } else {                  // EEPROM
        // Don't know whether chip erase has zapped EEPROM
        for(int n = 0; n < cp->size; n += cp->page_size) {
          if(!_is_all_0xff(cp->copy + n, cp->page_size)) { // First page that had EEPROM data
            if(avr_read_page_default(pgm, p, mem, n, cp->copy + n) < 0) {
              report_progress(1, -1, NULL);
              if(quell_progress)
                msg_info("\n");
              pmsg_error("EEPROM read failed at addr 0x%04x\n", n);
              return LIBAVRDUDE_GENERAL_FAILURE;
            }
            // EEPROM zapped by chip erase? Set all copy to 0xff
            if(_is_all_0xff(cp->copy + n, cp->page_size))
              memset(cp->copy, 0xff, cp->size);
            break;
          }
        }
      }
    }
    report_progress(1, 0, NULL);
  }

  int nwr = 0;
  // Count number of writes
  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = mems[i].mem;
    AVR_Cache *cp = mems[i].cp;
    if(!mem)
      continue;

    for(int pgno = 0, n = 0; n < cp->size; pgno++, n += cp->page_size)
      if(cp->iscached[pgno] && memcmp(cp->copy + n, cp->cont + n, cp->page_size))
        nwr++;
  }

  report_progress(0, 1, "Writing");
  if(nwr) {
    // Write all modified pages to the device
    for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
      AVRMEM *mem = mems[i].mem;
      AVR_Cache *cp = mems[i].cp;
      if(!mem || !cp->cont)
        continue;

      for(int iwr = 0, pgno = 0, n = 0; n < cp->size; pgno++, n += cp->page_size) {
        if(cp->iscached[pgno] && memcmp(cp->copy + n, cp->cont + n, cp->page_size)) {
          if(!chiperase && mems[i].pgerase)
            pgm->page_erase(pgm, p, mem, n);
          if(writeCachePage(cp, pgm, p, mem, n, 1) < 0)
            return LIBAVRDUDE_GENERAL_FAILURE;
          if(memcmp(cp->copy + n, cp->cont + n, cp->page_size)) {
            report_progress(1, -1, NULL);
            if(quell_progress)
              msg_info("\n");
            pmsg_error("verification mismatch at %s page addr 0x%04x\n", mem->desc, n);
            return LIBAVRDUDE_GENERAL_FAILURE;
          }
          report_progress(iwr++, nwr, NULL);
        }
      }
    }
  }
  report_progress(1, 0, NULL);

  msg_info(quell_progress? "done\n": "\n");
  return LIBAVRDUDE_SUCCESS;
}


/*
 * Read byte via a read/write cache
 *  - Used if paged routines available and if memory is EEPROM or flash
 *  - Otherwise fall back to pgm->read_byte()
 *  - Out of memory addr: synchronise cache and, if successful, pretend reading a zero
 *  - Cache is automagically created and initialised if needed
 */
int avr_read_byte_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char *value) {

  // Use pgm->read_byte() if not EEPROM/flash or no paged access
  if(!avr_has_paged_access(pgm, mem))
    return pgm->read_byte(pgm, p, mem, addr, value);

  // If address is out of range synchronise cache and, if successful, pretend reading a zero
  if(addr >= (unsigned long) mem->size) {
    if(avr_flush_cache(pgm, p) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;
    *value = 0;
    return LIBAVRDUDE_SUCCESS;
  }

  AVR_Cache *cp = avr_mem_is_eeprom_type(mem)? pgm->cp_eeprom: pgm->cp_flash;

  if(!cp->cont)                 // Init cache if needed
    if(initCache(cp, pgm, p) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

  int cacheaddr = cacheAddress((int) addr, cp, mem);
  if(cacheaddr < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  // Ensure cache page is there
  if(loadCachePage(cp, pgm, p, mem, addr, cacheaddr, 0) < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  *value = cp->cont[cacheaddr];

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

  if(!cp->cont)                 // Init cache if needed
    if(initCache(cp, pgm, p) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

  int cacheaddr = cacheAddress((int) addr, cp, mem);
  if(cacheaddr < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  // Ensure cache page is there
  if(loadCachePage(cp, pgm, p, mem, addr, cacheaddr, 0) < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  cp->cont[cacheaddr] = data;

  return LIBAVRDUDE_SUCCESS;
}


// Erase the chip and set the cache accordingly
int avr_chip_erase_cached(const PROGRAMMER *pgm, const AVRPART *p) {
  CacheDesc_t mems[2] = {
    { avr_locate_mem(p, "flash"), pgm->cp_flash, 1 },
    { avr_locate_mem(p, "eeprom"), pgm->cp_eeprom, 0 },
  };

  if(pgm->chip_erase(pgm, p) < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVRMEM *mem = mems[i].mem;
    AVR_Cache *cp = mems[i].cp;

    if(!mem || !avr_has_paged_access(pgm, mem))
      continue;

    if(!cp->cont)               // Init cache if needed
      if(initCache(cp, pgm, p) < 0)
        return LIBAVRDUDE_GENERAL_FAILURE;

    if(mems[i].isflash) {       // flash
      if(pgm->prog_modes & PM_SPM) { // reset cache to unknown
        memset(cp->iscached, 0, cp->size/cp->page_size);
      } else {                  // preset all pages as erased
        memset(cp->copy, 0xff, cp->size);
        memset(cp->cont, 0xff, cp->size);
        memset(cp->iscached, 1, cp->size/cp->page_size);
      }
    } else {                    // EEPROM: test whether cached pages were zapped
      bool erasedee = 0;
      for(int pgno = 0, n = 0; n < cp->size; pgno++, n += cp->page_size) {
        if(cp->iscached[pgno]) {
          if(!_is_all_0xff(cp->copy + n, cp->page_size)) { // Page has EEPROM data?
            if(avr_read_page_default(pgm, p, mem, n, cp->copy + n) < 0)
              return LIBAVRDUDE_GENERAL_FAILURE;
            erasedee = _is_all_0xff(cp->copy + n, cp->page_size);
            break;
          }
        }
      }
      if(erasedee) {            // EEPROM was erased, set cache correspondingly
        memset(cp->copy, 0xff, cp->size);
        memset(cp->cont, 0xff, cp->size);
        memset(cp->iscached, 1, cp->size/cp->page_size);
      } else {                  // discard previous writes, but leave cache
        for(int pgno = 0, n = 0; n < cp->size; pgno++, n += cp->page_size)
          if(cp->iscached[pgno])
            memcpy(cp->cont + n, cp->copy + n, cp->page_size);
      }
    }
  }

  return LIBAVRDUDE_SUCCESS;
}


// Erase a page and synchronise it with the cache
int avr_page_erase_cached(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int uaddr) {

  int addr = uaddr;

  if(!avr_has_paged_access(pgm, mem) || addr < 0 || addr >= mem->size)
    return LIBAVRDUDE_GENERAL_FAILURE;

  if(mem->page_size == 1) {
    if(pgm->write_byte(pgm, p, mem, uaddr, 0xff) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;
  } else {
    if(!pgm->page_erase || pgm->page_erase(pgm, p, mem, uaddr) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;
  }

  AVR_Cache *cp = avr_mem_is_eeprom_type(mem)? pgm->cp_eeprom: pgm->cp_flash;

  if(!cp->cont)                 // Init cache if needed
    if(initCache(cp, pgm, p) < 0)
      return LIBAVRDUDE_GENERAL_FAILURE;

  int cacheaddr = cacheAddress(addr, cp, mem);
  if(cacheaddr < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  // Invalidate this cache page and read back, ie, we don't trust the page_erase() routine
  cp->iscached[cacheaddr/cp->page_size] = 0;

  // Reload cache page
  if(loadCachePage(cp, pgm, p, mem, (int) addr, cacheaddr, 0) < 0)
    return LIBAVRDUDE_GENERAL_FAILURE;

  if(!_is_all_0xff(cp->cont + (cacheaddr & ~(cp->page_size-1)), cp->page_size))
    return LIBAVRDUDE_GENERAL_FAILURE;

  return LIBAVRDUDE_SUCCESS;
}


// Free cache(s) discarding any pending writes
int avr_reset_cache(const PROGRAMMER *pgm, const AVRPART *p) {
  AVR_Cache *mems[2] = { pgm->cp_flash, pgm->cp_eeprom, };

  for(size_t i = 0; i < sizeof mems/sizeof*mems; i++) {
    AVR_Cache *cp = mems[i];
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
