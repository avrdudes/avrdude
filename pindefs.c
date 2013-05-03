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

/* $Id: pindefs.h 1132 2013-01-09 19:23:30Z rliebscher $ */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pindefs.h"
#include "pgm.h"

/**
 * Adds a pin in the pin definition as normal or inverse pin.
 *
 * @param[out] pindef pin definition to update
 * @param[in] pin number of pin [0..PIN_MAX]
 * @param[in] inverse inverse (true) or normal (false) pin
 */
void pin_set_value(struct pindef_t * const pindef, const int pin, const bool inverse){

  pindef->mask[pin/PIN_FIELD_ELEMENT_SIZE] |= 1 << (pin % PIN_FIELD_ELEMENT_SIZE);
  if (inverse)
    pindef->inverse[pin/PIN_FIELD_ELEMENT_SIZE] |=  (1 << (pin % PIN_FIELD_ELEMENT_SIZE));
  else
    pindef->inverse[pin/PIN_FIELD_ELEMENT_SIZE] &= ~(1 << (pin % PIN_FIELD_ELEMENT_SIZE));
}

/**
 * Clear all defined pins in pindef.
 *
 * @param[out] pindef pin definition to clear
 */
void pin_clear_all(struct pindef_t * const pindef){
    memset(pindef,0,sizeof(struct pindef_t));
}

/**
 * Convert new pin definition to old pin number
 *
 * @param[in] pindef new pin definition structure
 * @param[out] pinno old pin definition integer
 */
static void pin_fill_old_pinno(const struct pindef_t * const pindef, unsigned int * const pinno){
    bool found = false;
    int i;
    for (i=0;i<PIN_MAX;i++){
      if (pindef->mask[i/PIN_FIELD_ELEMENT_SIZE] & (1 << (i % PIN_FIELD_ELEMENT_SIZE))){
        if(found){
	    fprintf(stderr,"Multiple pins found\n"); //TODO
	    exit(1);
	}
	found = true;
	*pinno = i;
	if (pindef->inverse[i/PIN_FIELD_ELEMENT_SIZE] & (1 << (i % PIN_FIELD_ELEMENT_SIZE))){
	    *pinno |= PIN_INVERSE;
	}
      }
    }
}

/**
 * Convert new pin definition to old pinlist, does not support mixed inverted/non-inverted pin
 *
 * @param[in] pindef new pin definition structure
 * @param[out] pinno old pin definition integer
 */
static void pin_fill_old_pinlist(const struct pindef_t * const pindef, unsigned int * const pinno){
    int i;

    for (i=0;i<PIN_FIELD_SIZE;i++){
      if(i == 0) {
	if ((pindef->mask[i] & ~PIN_MASK) != 0){
	    fprintf(stderr,"Pins of higher index than max field size for old pinno found\n");
	    exit(1);
	}
	if (pindef->mask[i] == pindef->inverse[i]) { /* all set bits in mask are set in inverse */
	    *pinno = pindef->mask[i];
	    *pinno |= PIN_INVERSE;
	} else if (pindef->mask[i] == ((~pindef->inverse[i]) & pindef->mask[i])) { /* all set bits in mask are cleared in inverse */
	    *pinno = pindef->mask[i];
	} else {
	    fprintf(stderr,"pins have different polarity set\n");
	    exit(1);
        }
      } else if (pindef->mask[i] != 0){
	    fprintf(stderr,"Pins have higher number than fit in old format\n");
	    exit(1);
      }
    }
}


/**
 * Convert for given programmer new pin definitions to old pin definitions.
 *
 * @param[inout] pgm programmer whose pins shall be converted.
 */
void pgm_fill_old_pins(struct programmer_t * const pgm) {

  pin_fill_old_pinlist(&(pgm->pin[PPI_AVR_VCC]),  &(pgm->pinno[PPI_AVR_VCC]));
  pin_fill_old_pinlist(&(pgm->pin[PPI_AVR_BUFF]), &(pgm->pinno[PPI_AVR_BUFF]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_AVR_RESET]),&(pgm->pinno[PIN_AVR_RESET]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_AVR_SCK]),  &(pgm->pinno[PIN_AVR_SCK]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_AVR_MOSI]), &(pgm->pinno[PIN_AVR_MOSI]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_AVR_MISO]), &(pgm->pinno[PIN_AVR_MISO]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_LED_ERR]),  &(pgm->pinno[PIN_LED_ERR]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_LED_RDY]),  &(pgm->pinno[PIN_LED_RDY]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_LED_PGM]),  &(pgm->pinno[PIN_LED_PGM]));
  pin_fill_old_pinno(  &(pgm->pin[PIN_LED_VFY]),  &(pgm->pinno[PIN_LED_VFY]));

}