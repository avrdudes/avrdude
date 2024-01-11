/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2004  Brian S. Dean <bsd@bdmicro.com>
 * Copyright 2007 Joerg Wunsch <j@uriah.heep.sax.de>
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
#include <string.h>

#include "avrdude.h"
#include "libavrdude.h"

static void pgm_default(void);
static int  pgm_default_2(const PROGRAMMER *, const AVRPART *);
static int  pgm_default_3(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char * value);
static void pgm_default_4(const PROGRAMMER *);
static int  pgm_default_5(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char data);
static void pgm_default_6(const PROGRAMMER *, const char *);


static int pgm_default_open(PROGRAMMER *pgm, const char *name) {
  pmsg_error("programmer does not support open()");
  return -1;
}

static void pgm_default_close(PROGRAMMER *pgm) {
  pgm_default();
}

static void pgm_default_enable(PROGRAMMER *pgm, const AVRPART *p) {
  pgm_default();
}

static int pgm_default_led(const PROGRAMMER *pgm, int value) {
   // If programmer has no LEDs, just do nothing
  return 0;
}


static void pgm_default_powerup_powerdown(const PROGRAMMER *pgm) {
   // If programmer does not support powerup/down, just do nothing
}


PROGRAMMER *pgm_new(void) {
  PROGRAMMER *pgm = (PROGRAMMER *) cfg_malloc("pgm_new()", sizeof(*pgm));
  const char *nulp = cache_string("");

  // Initialise const char * and LISTID entities
  pgm->id = lcreat(NULL, 0);
  pgm->usbpid = lcreat(NULL, 0);
  pgm->hvupdi_support = lcreat(NULL, 0);
  pgm->desc = nulp;
  pgm->parent_id = nulp;
  pgm->usbdev = nulp;
  pgm->usbsn = nulp;
  pgm->usbvendor = nulp;
  pgm->usbproduct = nulp;
  pgm->config_file = nulp;

  // Allocate cache structures for flash and EEPROM, *do not* free in pgm_free()
  pgm->cp_flash = cfg_malloc("pgm_new()", sizeof(AVR_Cache));
  pgm->cp_eeprom = cfg_malloc("pgm_new()", sizeof(AVR_Cache));
  pgm->cp_bootrow = cfg_malloc("pgm_new()", sizeof(AVR_Cache));
  pgm->cp_usersig = cfg_malloc("pgm_new()", sizeof(AVR_Cache));

  // Default values
  pgm->initpgm = NULL;
  pgm->lineno = 0;
  pgm->baudrate = 0;

  // Clear pin array
  for(int i=0; i<N_PINS; i++) {
    pgm->pinno[i] = NO_PIN;
    pin_clear_all(&(pgm->pin[i]));
  }

  pgm->leds = cfg_malloc(__func__, sizeof(leds_t));
  /*
   * mandatory functions - these are called without checking to see
   * whether they are assigned or not
   */
  pgm->initialize     = pgm_default_2;
  pgm->display        = pgm_default_6;
  pgm->enable         = pgm_default_enable;
  pgm->disable        = pgm_default_4;
  pgm->powerup        = pgm_default_powerup_powerdown;
  pgm->powerdown      = pgm_default_powerup_powerdown;
  pgm->program_enable = pgm_default_2;
  pgm->chip_erase     = pgm_default_2;
  pgm->open           = pgm_default_open;
  pgm->close          = pgm_default_close;
  pgm->read_byte      = pgm_default_3;
  pgm->write_byte     = pgm_default_5;

  /*
   * predefined functions - these functions have a valid default
   * implementation. Hence, they don't need to be defined in
   * the programmer.
   */
  pgm->rdy_led        = pgm_default_led;
  pgm->err_led        = pgm_default_led;
  pgm->pgm_led        = pgm_default_led;
  pgm->vfy_led        = pgm_default_led;
  pgm->read_byte_cached  = avr_read_byte_cached;
  pgm->write_byte_cached = avr_write_byte_cached;
  pgm->chip_erase_cached = avr_chip_erase_cached;
  pgm->page_erase_cached = avr_page_erase_cached;
  pgm->flush_cache    = avr_flush_cache;
  pgm->reset_cache    = avr_reset_cache;

  /*
   * optional functions - these are checked to make sure they are
   * assigned before they are called
   */
  pgm->unlock         = NULL;
  pgm->cmd            = NULL;
  pgm->cmd_tpi        = NULL;
  pgm->spi            = NULL;
  pgm->paged_write    = NULL;
  pgm->paged_load     = NULL;
  pgm->page_erase     = NULL;
  pgm->write_setup    = NULL;
  pgm->read_sig_bytes = NULL;
  pgm->read_sib       = NULL;
  pgm->term_keep_alive= NULL;
  pgm->print_parms    = NULL;
  pgm->set_vtarget    = NULL;
  pgm->get_vtarget    = NULL;
  pgm->set_varef      = NULL;
  pgm->get_varef      = NULL;
  pgm->set_fosc       = NULL;
  pgm->get_fosc       = NULL;
  pgm->set_sck_period = NULL;
  pgm->get_sck_period = NULL;
  pgm->setpin         = NULL;
  pgm->getpin         = NULL;
  pgm->highpulsepin   = NULL;
  pgm->parseexitspecs = NULL;
  pgm->perform_osccal = NULL;
  pgm->parseextparams = NULL;
  pgm->setup          = NULL;
  pgm->teardown       = NULL;
  pgm->readonly       = NULL;
  pgm->flash_readhook = NULL;

  // For allocating "global" memory by the programmer
  pgm->cookie          = NULL;

  return pgm;
}

void pgm_free(PROGRAMMER *p) {
  if(p) {
    if(p->id) {
      ldestroy_cb(p->id, free);
      p->id = NULL;
    }
    if(p->usbpid) {
      ldestroy_cb(p->usbpid, free);
      p->usbpid = NULL;
    }
    if(p->hvupdi_support) {
      ldestroy_cb(p->hvupdi_support, free);
      p->hvupdi_support = NULL;
    }
    free(p->leds); p->leds = NULL;
    // Never free const char *, eg, p->desc, which are set by cache_string()
    // p->cookie was freed by pgm_teardown
    // Never free cp_flash, cp_eeprom, cp_bootrow or cp_usersig cache structures
    free(p);
  }
}

PROGRAMMER *pgm_dup(const PROGRAMMER *src) {
  PROGRAMMER *pgm = pgm_new();

  if(src) {
    ldestroy_cb(pgm->id, free);
    ldestroy_cb(pgm->usbpid, free);
    ldestroy_cb(pgm->hvupdi_support, free);
    // There must be only one cache, even though the part is duplicated
    if(pgm->cp_flash)
      free(pgm->cp_flash);
    if(pgm->cp_eeprom)
      free(pgm->cp_eeprom);
    if(pgm->cp_bootrow)
      free(pgm->cp_bootrow);
    if(pgm->cp_usersig)
      free(pgm->cp_usersig);

    leds_t *ls = pgm->leds;
    memcpy(pgm, src, sizeof(*pgm));
    if(ls && src->leds)
      memcpy(ls, src->leds, sizeof *ls);
    pgm->leds = ls;

    pgm->id = lcreat(NULL, 0);
    pgm->usbpid = lcreat(NULL, 0);
    pgm->hvupdi_support = lcreat(NULL, 0);

    // Leave id list empty but copy usbpid and hvupdi_support over
    if(src->hvupdi_support)
      for(LNODEID ln = lfirst(src->hvupdi_support); ln; ln = lnext(ln)) {
        int *ip = cfg_malloc("pgm_dup()", sizeof(int));
        *ip = *(int *) ldata(ln);
        ladd(pgm->hvupdi_support, ip);
      }
    if(src->usbpid)
      for(LNODEID ln = lfirst(src->usbpid); ln; ln = lnext(ln)) {
        int *ip = cfg_malloc("pgm_dup()", sizeof(int));
        *ip = *(int *) ldata(ln);
        ladd(pgm->usbpid, ip);
      }
  }

  return pgm;
}


static void pgm_default(void) {
  pmsg_error("programmer operation not supported\n");
}


static int  pgm_default_2 (const PROGRAMMER *pgm, const AVRPART *p) {
  pgm_default();
  return -1;
}

static int  pgm_default_3 (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
			   unsigned long addr, unsigned char * value) {
  pgm_default();
  return -1;
}

static void pgm_default_4 (const PROGRAMMER *pgm) {
  pgm_default();
}

static int  pgm_default_5 (const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
			   unsigned long addr, unsigned char data) {
  pgm_default();
  return -1;
}

static void pgm_default_6 (const PROGRAMMER *pgm, const char *p) {
  pgm_default();
}


void programmer_display(PROGRAMMER *pgm, const char * p) {
  msg_info("%sProgrammer Type       : %s\n", p, pgm->type);
  msg_info("%sDescription           : %s\n", p, pgm->desc);

  pgm->display(pgm, p);
}


void pgm_display_generic_mask(const PROGRAMMER *pgm, const char *p, unsigned int show) {
  if(show & (1<<PPI_AVR_VCC)) 
    msg_info("%s  VCC     = %s\n", p, pins_to_str(&pgm->pin[PPI_AVR_VCC]));
  if(show & (1<<PPI_AVR_BUFF))
    msg_info("%s  BUFF    = %s\n", p, pins_to_str(&pgm->pin[PPI_AVR_BUFF]));
  if(show & (1<<PIN_AVR_RESET))
    msg_info("%s  RESET   = %s\n", p, pins_to_str(&pgm->pin[PIN_AVR_RESET]));
  if(show & (1<<PIN_AVR_SCK))
    msg_info("%s  SCK     = %s\n", p, pins_to_str(&pgm->pin[PIN_AVR_SCK]));
  if(show & (1<<PIN_AVR_SDO))
    msg_info("%s  SDO     = %s\n", p, pins_to_str(&pgm->pin[PIN_AVR_SDO]));
  if(show & (1<<PIN_AVR_SDI))
    msg_info("%s  SDI     = %s\n", p, pins_to_str(&pgm->pin[PIN_AVR_SDI]));
  if(show & (1<<PIN_JTAG_TCK))
    msg_info("%s  TCK     = %s\n", p, pins_to_str(&pgm->pin[PIN_JTAG_TCK]));
  if(show & (1<<PIN_JTAG_TDI))
    msg_info("%s  TDI     = %s\n", p, pins_to_str(&pgm->pin[PIN_JTAG_TDI]));
  if(show & (1<<PIN_JTAG_TDO))
    msg_info("%s  TDO     = %s\n", p, pins_to_str(&pgm->pin[PIN_JTAG_TDO]));
  if(show & (1<<PIN_JTAG_TMS))
    msg_info("%s  TMS     = %s\n", p, pins_to_str(&pgm->pin[PIN_JTAG_TMS]));
  if(show & (1<<PIN_LED_ERR))
    msg_info("%s  ERR LED = %s\n", p, pins_to_str(&pgm->pin[PIN_LED_ERR]));
  if(show & (1<<PIN_LED_RDY))
    msg_info("%s  RDY LED = %s\n", p, pins_to_str(&pgm->pin[PIN_LED_RDY]));
  if(show & (1<<PIN_LED_PGM))
    msg_info("%s  PGM LED = %s\n", p, pins_to_str(&pgm->pin[PIN_LED_PGM]));
  if(show & (1<<PIN_LED_VFY))
    msg_info("%s  VFY LED = %s\n", p, pins_to_str(&pgm->pin[PIN_LED_VFY]));
}

void pgm_display_generic(const PROGRAMMER *pgm, const char *p) {
  pgm_display_generic_mask(pgm, p, SHOW_ALL_PINS);
}

// Locate a real programmer entry by partial initial id and set the matching id
PROGRAMMER *locate_programmer_starts_set(const LISTID programmers, const char *pgid, const char **setid, AVRPART *prt) {
  PROGRAMMER *pgm, *matchp;
  int matches, p1, pmode = prt? prt->prog_modes: -1;
  const char *matchid;
  size_t l;

  if(!pgid || !(p1 = *pgid))
    return NULL;

  l = strlen(pgid);
  matches = 0;
  matchp = NULL;
  for(LNODEID ln1=lfirst(programmers); ln1; ln1=lnext(ln1)) {
    pgm = ldata(ln1);
    if(is_programmer(pgm) && (pgm->prog_modes & pmode))
      for(LNODEID ln2=lfirst(pgm->id); ln2; ln2=lnext(ln2)) {
        const char *id = (const char *) ldata(ln2);
        if(p1 == *id && !strncasecmp(id, pgid, l)) { // Partial initial match
          matchp = pgm;
          matchid = id;
          matches++;
          if(id[l] == 0) {      // Exact match; return straight away
            matches = 1;
            goto done;
          }
          break;
        }
      }
    }

done:
  if(matches == 1) {
    if(setid)
      *setid = matchid;
    return matchp;
  }

  return NULL;
}

// Locate a programmer (or serial adapter) by full name and set the matching id
PROGRAMMER *locate_programmer_set(const LISTID programmers, const char *configid, const char **setid) {
  for(LNODEID ln1=lfirst(programmers); ln1; ln1=lnext(ln1)) {
    PROGRAMMER *p = ldata(ln1);
    for(LNODEID ln2=lfirst(p->id); ln2; ln2=lnext(ln2)) {
      const char *id = (const char *) ldata(ln2);
      if(str_caseeq(configid, id)) {
        if(setid)
          *setid = id;
        return p;
      }
    }
  }

  return NULL;
}

PROGRAMMER *locate_programmer(const LISTID programmers, const char *configid) {
  return locate_programmer_set(programmers, configid, NULL);
}

/*
 * Iterate over the list of programmers given as "programmers", and
 * call the callback function cb for each entry found.  cb is being
 * passed the following arguments:
 * . the name of the programmer (for -c)
 * . the descriptive text given in the config file
 * . the name of the config file this programmer has been defined in
 * . the line number of the config file this programmer has been defined at
 * . the "cookie" passed into walk_programmers() (opaque client data)
 */
void walk_programmers(LISTID programmers, walk_programmers_cb cb, void *cookie)
{
  LNODEID ln1;
  LNODEID ln2;
  PROGRAMMER * p;

  for (ln1 = lfirst(programmers); ln1; ln1 = lnext(ln1)) {
    p = ldata(ln1);
    for (ln2=lfirst(p->id); ln2; ln2=lnext(ln2)) {
      cb(ldata(ln2), p->desc, p->config_file, p->lineno, cookie);
    }
  }
}

/*
 * Compare function to sort the list of programmers
 */
static int sort_programmer_compare(const PROGRAMMER *p1, const PROGRAMMER *p2) {
  if(p1 == NULL || p1->id == NULL || p2 == NULL || p2->id == NULL)
    return 0;

  return strcasecmp(ldata(lfirst(p1->id)), ldata(lfirst(p2->id)));
}

/*
 * Sort the list of programmers given as "programmers"
 */
void sort_programmers(LISTID programmers)
{
  lsort(programmers,(int (*)(void*, void*)) sort_programmer_compare);
}


// Soft assignment: some struct programmer_t entries can be both programmers and serial adapters
int is_programmer(const PROGRAMMER *p) {
 return p && p->id && lsize(p->id) && p->prog_modes && p->initpgm;
}

int is_serialadapter(const SERIALADAPTER *p) {
  return p && p->id && lsize(p->id) && p->usbpid && lsize(p->usbpid) && (!p->prog_modes || p->is_serialadapter);
}
