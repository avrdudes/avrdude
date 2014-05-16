/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef __pindefs_h__
#define __pindefs_h__

#include <limits.h>

#include "ac_cfg.h"

/* lets try to select at least 32 bits */
#ifdef HAVE_STDINT_H
#include <stdint.h>
typedef uint32_t pinmask_t;
#else
#if UINT_MAX >= 0xFFFFFFFF
typedef unsigned int pinmask_t;
#else
typedef unsigned long pinmask_t;
#endif
#endif

#include <stdbool.h>


enum {
  PPI_AVR_VCC = 1,
  PPI_AVR_BUFF,
  PIN_AVR_RESET,
  PIN_AVR_SCK,
  PIN_AVR_MOSI,
  PIN_AVR_MISO,
  PIN_LED_ERR,
  PIN_LED_RDY,
  PIN_LED_PGM,
  PIN_LED_VFY,
  N_PINS
};

#define PIN_MASK    (UINT_MAX>>1)
#define PIN_INVERSE (~(PIN_MASK))	/* flag for inverted pin in serbb */
#define PIN_MIN     0   /* smallest allowed pin number */
#define PIN_MAX     31  /* largest allowed pin number */

#ifdef HAVE_LINUXGPIO
/* Embedded systems might have a lot more gpio than only 0-31 */
#undef PIN_MAX
#define PIN_MAX     255 /* largest allowed pin number */
#endif

/** Number of pins in each element of the bitfield */
#define PIN_FIELD_ELEMENT_SIZE (sizeof(pinmask_t) * 8)
/** Numer of elements to store the complete bitfield of all pins */
#define PIN_FIELD_SIZE ((PIN_MAX + PIN_FIELD_ELEMENT_SIZE)/PIN_FIELD_ELEMENT_SIZE)

/**
 * This sets the corresponding bits to 1 or 0, the inverse mask is used to invert the value in necessary.
 * It uses only the lowest element (index=0) of the bitfield, which should be enough for most
 * programmers.
 *
 * @param[in] x       input value
 * @param[in] pgm     the programmer whose pin definitions to use
 * @param[in] pinname the logical name of the pin (PIN_AVR_*, ...)
 * @param[in] level   the logical level (level != 0 => 1, level == 0 => 0),
 *                      if the pin is defined as inverted the resulting bit is also inverted
 * @returns           the input value with the relevant bits modified
 */
#define SET_BITS_0(x,pgm,pinname,level) (((x) & ~(pgm)->pin[pinname].mask[0]) \
    | (\
        (pgm)->pin[pinname].mask[0] & ( \
             (level) \
             ?~((pgm)->pin[pinname].inverse[0]) \
             : ((pgm)->pin[pinname].inverse[0]) \
        ) \
    ) \
)

/**
 * Check if the corresponding bit is set (returns != 0) or cleared.
 * The inverse mask is used, to invert the relevant bits.
 * If the pin definition contains multiple pins, then a single set pin leads to return value != 0.
 * Then you have to check the relevant bits of the returned value, if you need more information.
 * It uses only the lowest element (index=0) of the bitfield, which should be enough for most
 * programmers.
 *
 * @param[in] x       input value
 * @param[in] pgm     the programmer whose pin definitions to use
 * @param[in] pinname the logical name of the pin (PIN_AVR_*, ...)
 * @returns           the input value with only the relevant bits (which are already inverted,
 *                      so you get always the logical level)
 */
#define GET_BITS_0(x,pgm,pinname)       (((x) ^ (pgm)->pin[pinname].inverse[0]) & (pgm)->pin[pinname].mask[0])

/**
 * Data structure to hold used pins by logical function (PIN_AVR_*, ...)
 */
struct pindef_t {
  pinmask_t mask[PIN_FIELD_SIZE]; ///< bitfield of used pins
  pinmask_t inverse[PIN_FIELD_SIZE]; ///< bitfield of inverse/normal usage of used pins
};

/**
 * Data structure to define a checklist of valid pins for each function.
 */
struct pin_checklist_t {
  int pinname; ///< logical pinname eg. PIN_AVR_SCK
  int mandatory; ///< is this a mandatory pin
  const struct pindef_t* valid_pins; ///< mask defines allowed pins, inverse define is they might be used inverted
};

/**
 * Adds a pin in the pin definition as normal or inverse pin.
 *
 * @param[out] pindef pin definition to update
 * @param[in] pin number of pin [0..PIN_MAX]
 * @param[in] inverse inverse (true) or normal (false) pin
 */
void pin_set_value(struct pindef_t * const pindef, const int pin, const bool inverse);

/**
 * Clear all defined pins in pindef.
 *
 * @param[out] pindef pin definition to clear
 */
void pin_clear_all(struct pindef_t * const pindef);

struct programmer_t; /* forward declaration */

/**
 * Convert for given programmer new pin definitions to old pin definitions.
 *
 * @param[inout] pgm programmer whose pins shall be converted.
 */
int pgm_fill_old_pins(struct programmer_t * const pgm);

/**
 * This function checks all pin of pgm against the constraints given in the checklist.
 * It checks if 
 * @li any invalid pins are used
 * @li valid pins are used inverted when not allowed
 * @li any pins are used by more than one function
 * @li any mandatory pin is not set all.
 *
 * In case of any error it report the wrong function and the pin numbers.
 * For verbose >= 2 it also reports the possible correct values.
 * For verbose >=3 it shows also which pins were ok.
 *
 * @param[in] pgm the programmer to check
 * @param[in] checklist the constraint for the pins
 * @param[in] size the number of entries in checklist
 * @param[in] output false suppresses error messages to the user
 * @returns 0 if all pin definitions are valid, -1 otherwise
 */
int pins_check(const struct programmer_t * const pgm, const struct pin_checklist_t * const checklist, const int size, const bool output);

/**
 * Returns the name of the pin as string.
 * 
 * @param pinname the pinname which we want as string.
 * @returns a string with the pinname, or <unknown> if pinname is invalid.
 */
const char * avr_pin_name(int pinname);

/**
 * This function returns a string representation of defined pins eg. ~1,2,~4,~5,7
 * Another execution of this function will overwrite the previous result in the static buffer.
 *
 * @param[in] pindef the pin definition for which we want the string representation
 * @returns pointer to a static string.
 */
const char * pins_to_str(const struct pindef_t * const pindef);

/**
 * This function returns a string representation of pins in the mask eg. 1,3,5-7,9,12
 * Another execution of this function will overwrite the previous result in the static buffer.
 * Consecutive pin number are representated as start-end.
 *
 * @param[in] pinmask the pin mask for which we want the string representation
 * @returns pointer to a static string.
 */
const char * pinmask_to_str(const pinmask_t * const pinmask);


#endif

