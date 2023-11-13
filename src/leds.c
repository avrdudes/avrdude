/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2023 Stefan Rueger <stefan.rueger@urclocks.com>
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

#include "ac_cfg.h"
#include <string.h>
#include "avrdude.h"
#include "libavrdude.h"

/*
 * Handle LEDs for some programmers
 *
 * Some hardware programmers have LEDs, and the firmware controls them
 * fully without AVRDUDE having a way to influence the LED states. Other
 * programmers have LEDs and expect the host downloader/uploader to handle
 * them. For the latter type of programmers AVRDUDE provides support of
 * four LEDs (RDY, ERR, PGM and VFY) which can be set via corresponding
 * pgm->xxx_led(pgm, on_off) calls.
 *
 * The RDY LED is set once the programmer is initialised and switched
 * off when AVRDUDE exits. During reading, writing or erasing the target
 * the PGM LED flashes with around 2.5 Hz, whilst the VFY LED comes on
 * during -U verification of the uploaded contents. Errors are indicated
 * with the ERR LED.
 *
 * Assuming AVRDUDE got to the point where LEDs are accessible and the RDY
 * LED was switched on then, on exit, AVRDUDE will leave the LEDs in the
 * following states:
 *
 * | PGM | VFY | ERR | Semantics                                        |
 * | --- | --- | --- | ------------------------------------------------ |
 * | off | off | off | OK: all tasks done without errors                |
 * | off | off | on  | Some error not related to read/write/erase       |
 * | on  | off | on  | Read/write/erase error                           |
 * | off | on  | on  | Verification error but no read/write/erase error |
 * | on  | on  | on  | Read/write/erase error and verification error    |
 *
 * Other combinations should not show after exit.
 *
 */

#define TOFF                 2 // Toggle LED into off state
#define TON                  3 // Toggle LED into on state
#define CHECK               15 // Check LED needs changing

// Keep track of LED status and set LED 0 .. LED_N-1 physically on or off
static void led_direct(const PROGRAMMER *pgm, leds_t *ls, int led, int what) {
  if(what ^ !!(ls->phy & (1<<led))) {
    switch(led) {
    case LED_RDY:
      pgm->rdy_led(pgm, what);
      break;
    case LED_ERR:
      pgm->err_led(pgm, what);
      break;
    case LED_PGM:
      pgm->pgm_led(pgm, what);
      break;
    case LED_VFY:
      pgm->vfy_led(pgm, what);
      break;
    default:
      pmsg_error("unknown LED %d in %s()\n", led, __func__);
    }
    ls->phy ^= 1<<led;
  }
}

// Physical level of LED setting, deal with max blinking frequency LED_FMAX
static void led_physical(const PROGRAMMER *pgm, leds_t *ls, int led, int what) {
  if(led < 0 || led >= LED_N) // Sanity
    return;

  unsigned long now = avr_mstimestamp();

  if(what == ON || what == OFF) {
    if(what)                    // Force on or off
      ls->phy &= ~(1<<led);
    else
      ls->phy |= 1<<led;
    led_direct(pgm, ls, led, what);
    ls->chg &= ~(1<<led);
    ls->ms[led] = now;
    return;
  }

  if(what == TON && !(ls->set & (1<<led))) {
    // Never before set? Set immediately
    led_direct(pgm, ls, led, ON);
    ls->set |= 1<<led;
    ls->chg &= ~(1<<led);
    ls->ms[led] = now;
  } else if(what == TON || what == TOFF) {
    // Toggle led into on or off state once enough time has gone by
    ls->chg |= 1<<led;
  }

  // Check all LEDs whether they need toggling or setting
  for(int l = 0; l < LED_N; l++) {
    unsigned long diff = now - ls->ms[l];
    if(diff && diff >= (unsigned long) (1000.0/LED_FMAX/2)) {
      ls->ms[l] = now; // Toggle a fast signal or set to current value
      what = ls->chg & (1<<l)? !(ls->phy & (1<<l)): !!(ls->now & (1<<l));
      led_direct(pgm, ls, l, what);
      ls->chg &= ~(1<<l);
    }
  }
}

// Logical level of setting LEDs, passes on to physical level
int led_set(const PROGRAMMER *pgm, int led) {
  // leds should always be allocated, but if not use dummy
  leds_t sanity = { 0, 0, 0, 0, 0, {0, } }, *ls = pgm->leds? pgm->leds: &sanity;
  int what = led >= 0 && led < LED_N && !(ls->now & (1<<led))? TON: CHECK;

  switch(led) {
  case LED_BEG:
    memset(ls, 0, sizeof *ls);
    led_physical(pgm, ls, LED_RDY, OFF);
    led_physical(pgm, ls, LED_ERR, OFF);
    led_physical(pgm, ls, LED_PGM, OFF);
    led_physical(pgm, ls, LED_VFY, OFF);
    break;
  case LED_END:
    led_physical(pgm, ls, LED_RDY, OFF);
    led_physical(pgm, ls, LED_ERR, ls->end & (1<<LED_ERR)? ON: OFF);
    led_physical(pgm, ls, LED_PGM, ls->end & (1<<LED_PGM)? ON: OFF);
    led_physical(pgm, ls, LED_VFY, ls->end & (1<<LED_VFY)? ON: OFF);
    break;
  case LED_NOP:
    led_physical(pgm, ls, LED_RDY, CHECK); // All others will be checked, too
    break;
  case LED_ERR:                 // Record that error happened and in which mode
    ls->end |= 1<<LED_ERR;
    if(ls->now & (1<<LED_PGM))
      ls->end |= 1<<LED_PGM;
    if(ls->now & (1<<LED_VFY))
      ls->end |= 1<<LED_VFY;
  // Fall through
  case LED_RDY:
  case LED_PGM:
  case LED_VFY:
    ls->now |= 1<<led;
    led_physical(pgm, ls, led, what);
    break;
  default:
    pmsg_warning("unknown led %d in %s()\n", led, __func__);
    return -1;
  }

  return ls->now;
}

// Logical level of clearing LEDs, passes on to physical level
int led_clr(const PROGRAMMER *pgm, int led) {
  if(led < 0 || led >= LED_N) {
    pmsg_warning("unknown led %d in %s()\n", led, __func__);
    return -1;
  }

  // pgm->leds should always be allocated, but if not use dummy
  leds_t sanity = { 0, 0, 0, 0, 0, {0, } }, *ls = pgm->leds? pgm->leds: &sanity;
  int what = ls->now & (1<<led)? TOFF: CHECK;

  // Record logical level
  if(led >= 0 && led < LED_N)
    ls->now &= ~(1<<led);

  led_physical(pgm, ls, led, what);

  return ls->now;
}

// Programmer specific chip erase function with ERR/PGM LED info
int led_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  int rc = pgm->chip_erase(pgm, p);

  return rc;
}

// Programmer specific write byte function with ERR/PGM LED info
int led_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char value) {

  if(mem_is_readonly(m))
    return pgm->write_byte(pgm, p, m, addr, value);

  led_clr(pgm, LED_ERR);
  led_set(pgm, LED_PGM);

  int rc = pgm->write_byte(pgm, p, m, addr, value);

  if(rc < 0)
    led_set(pgm, LED_ERR);
  led_clr(pgm, LED_PGM);

  return rc;
}

// Programmer specific read byte function with ERR/PGM LED info
int led_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned long addr, unsigned char *valuep) {

  led_clr(pgm, LED_ERR);
  led_set(pgm, LED_PGM);

  int rc = pgm->read_byte(pgm, p, m, addr, valuep);

  if(rc<0)
    led_set(pgm, LED_ERR);
  led_clr(pgm, LED_PGM);

  return rc;
}

// Programmer-specific paged write function with ERR/PGM LED info
int led_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int baseaddr, unsigned int n_bytes) {

  led_clr(pgm, LED_ERR);

  int rc = pgm->paged_write? led_set(pgm, LED_PGM), pgm->paged_write(pgm, p, m, page_size, baseaddr, n_bytes): -1;

  if(rc<0)
    led_set(pgm, LED_ERR);
  led_clr(pgm, LED_PGM);

  return rc;
}

// Programmer-specific paged load function with ERR/PGM LED info
int led_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int baseaddr, unsigned int n_bytes) {

  led_clr(pgm, LED_ERR);

  int rc = pgm->paged_load? led_set(pgm, LED_PGM), pgm->paged_load(pgm, p, m, page_size, baseaddr, n_bytes): -1;

  if(rc<0)
    led_set(pgm, LED_ERR);
  led_clr(pgm, LED_PGM);

  return rc;
}

// Programmer-specific page erase function with ERR/PGM LED info
int led_page_erase(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int baseaddr) {

  led_clr(pgm, LED_ERR);

  int rc = pgm->page_erase? led_set(pgm, LED_PGM), pgm->page_erase(pgm, p, m, baseaddr): -1;

  if(rc<0)
    led_set(pgm, LED_ERR);
  led_clr(pgm, LED_PGM);

  return rc;
}
