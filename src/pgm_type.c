/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2007 Joerg Wunsch <j@uriah.heep.sax.de>
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

#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "arduino.h"
#include "avr910.h"
#include "avrftdi.h"
#include "buspirate.h"
#include "butterfly.h"
#include "ch341a.h"
#include "dryrun.h"
#include "flip1.h"
#include "flip2.h"
#include "ft245r.h"
#include "jtagmkI.h"
#include "jtagmkII.h"
#include "jtag3.h"
#include "linuxgpio.h"
#include "linuxspi.h"
#include "micronucleus.h"
#include "par.h"
#include "pickit2.h"
#include "pickit5.h"
#include "ppi.h"
#include "serbb.h"
#include "serialupdi.h"
#include "serprog.h"
#include "stk500.h"
#include "stk500generic.h"
#include "stk500v2.h"
#include "teensy.h"
#include "urclock.h"
#include "usbasp.h"
#include "usbtiny.h"
#include "wiring.h"
#include "xbee.h"

const PROGRAMMER_TYPE programmers_types[] = {   // Name(s) the programmers call themselves
  {"arduino", arduino_initpgm, arduino_desc},   // "Arduino"
  {"avr910", avr910_initpgm, avr910_desc},      // "avr910"
  {"avrftdi", avrftdi_initpgm, avrftdi_desc},   // "avrftdi"
  {"avrftdi_jtag", avrftdi_jtag_initpgm, avrftdi_jtag_desc},    // "avrftdi_jtag"
  {"buspirate", buspirate_initpgm, buspirate_desc},     // "BusPirate"
  {"buspirate_bb", buspirate_bb_initpgm, buspirate_bb_desc},    // "BusPirate_BB"
  {"butterfly", butterfly_initpgm, butterfly_desc},     // "butterfly"
  {"butterfly_mk", butterfly_mk_initpgm, butterfly_mk_desc},    // "butterfly_mk"
  {"ch341a", ch341a_initpgm, ch341a_desc},      // "ch341a"
  {"dryrun", dryrun_initpgm, dryrun_desc},      // "Dryrun"
  {"dragon_dw", jtagmkII_dragon_dw_initpgm, jtagmkII_dragon_dw_desc},   // "DRAGON_DW"
  {"dragon_hvsp", stk500v2_dragon_hvsp_initpgm, stk500v2_dragon_hvsp_desc},     // "DRAGON_HVSP"
  {"dragon_isp", stk500v2_dragon_isp_initpgm, stk500v2_dragon_isp_desc},        // "DRAGON_ISP"
  {"dragon_jtag", jtagmkII_dragon_initpgm, jtagmkII_dragon_desc},       // "DRAGON_JTAG"
  {"dragon_pdi", jtagmkII_dragon_pdi_initpgm, jtagmkII_dragon_pdi_desc},        // "DRAGON_PDI"
  {"dragon_pp", stk500v2_dragon_pp_initpgm, stk500v2_dragon_pp_desc},   // "DRAGON_PP"
  {"flip1", flip1_initpgm, flip1_desc}, // "flip1"
  {"flip2", flip2_initpgm, flip2_desc}, // "flip2"
  {"ftdi_syncbb", ft245r_initpgm, ft245r_desc}, // "ftdi_syncbb"
  {"jtagmki", jtagmkI_initpgm, jtagmkI_desc},   // "JTAGMKI"
  {"jtagmkii", jtagmkII_initpgm, jtagmkII_desc},        // "JTAGMKII"
  {"jtagmkii_avr32", jtagmkII_avr32_initpgm, jtagmkII_avr32_desc},      // "JTAGMKII_AVR32"
  {"jtagmkii_dw", jtagmkII_dw_initpgm, jtagmkII_dw_desc},       // "JTAGMKII_DW"
  {"jtagmkii_isp", stk500v2_jtagmkII_initpgm, stk500v2_jtagmkII_desc},  // "JTAGMKII_ISP"
  {"jtagmkii_pdi", jtagmkII_pdi_initpgm, jtagmkII_pdi_desc},    // "JTAGMKII_PDI"
  {"jtagmkii_updi", jtagmkII_updi_initpgm, jtagmkII_updi_desc}, // "JTAGMKII_UPDI"
  {"jtagice3", jtag3_initpgm, jtag3_desc},      // "JTAGICE3"
  {"jtagice3_pdi", jtag3_pdi_initpgm, jtag3_pdi_desc},  // "JTAGICE3_PDI"
  {"jtagice3_updi", jtag3_updi_initpgm, jtag3_updi_desc},       // "JTAGICE3_UPDI"
  {"jtagice3_dw", jtag3_dw_initpgm, jtag3_dw_desc},     // "JTAGICE3_DW"
  {"jtagice3_isp", stk500v2_jtag3_initpgm, stk500v2_jtag3_desc},        // "JTAG3_ISP"
  {"jtagice3_tpi", jtag3_tpi_initpgm, jtag3_tpi_desc},  // "JTAGICE3_TPI"
  {"linuxgpio", linuxgpio_initpgm, linuxgpio_desc},     // "linuxgpio"
  {"linuxspi", linuxspi_initpgm, linuxspi_desc},        // LINUXSPI
  {"micronucleus", micronucleus_initpgm, micronucleus_desc},    // "micronucleus" or "Micronucleus V2.0"
  {"par", par_initpgm, par_desc},       // "PPI"
  {"pickit2", pickit2_initpgm, pickit2_desc},   // "pickit2"
  {"pickit5_dw", pickit5_initpgm, pickit5_desc},      // "pickit5"
  {"pickit5_isp", pickit5_initpgm, pickit5_desc},      // "pickit5"
  {"pickit5_jtag", pickit5_initpgm, pickit5_desc},      // "pickit5"
  {"pickit5_pdi", pickit5_initpgm, pickit5_desc},      // "pickit5"
  {"pickit5_tpi", pickit5_initpgm, pickit5_desc},      // "pickit5"
  {"pickit5_updi", pickit5_initpgm, pickit5_desc},      // "pickit5"
  {"serbb", serbb_initpgm, serbb_desc}, // "SERBB"
  {"serialupdi", serialupdi_initpgm, serialupdi_desc},  // "serialupdi"
  {"serprog", serprog_initpgm, serprog_desc},   // "serprog"
  {"stk500", stk500_initpgm, stk500_desc},      // "STK500"
  {"stk500generic", stk500generic_initpgm, stk500generic_desc}, // "STK500GENERIC"
  {"stk500v2", stk500v2_initpgm, stk500v2_desc},        // "STK500V2"
  {"stk500hvsp", stk500hvsp_initpgm, stk500hvsp_desc},  // "STK500HVSP"
  {"stk500pp", stk500pp_initpgm, stk500pp_desc},        // "STK500PP"
  {"stk600", stk600_initpgm, stk600_desc},      // "STK600"
  {"stk600hvsp", stk600hvsp_initpgm, stk600hvsp_desc},  // "STK600HVSP"
  {"stk600pp", stk600pp_initpgm, stk600pp_desc},        // "STK600PP"
  {"teensy", teensy_initpgm, teensy_desc},      // "teensy"
  {"urclock", urclock_initpgm, urclock_desc},   // "Urclock"
  {"usbasp", usbasp_initpgm, usbasp_desc},      // "usbasp"
  {"usbtiny", usbtiny_initpgm, usbtiny_desc},   // "USBtiny" or "usbtiny"
  {"wiring", wiring_initpgm, wiring_desc},      // "Wiring"
  {"xbee", xbee_initpgm, xbee_desc},    // "XBee"
};

const PROGRAMMER_TYPE *locate_programmer_type(const char *id) {
  for(size_t i = 0; i < sizeof programmers_types/sizeof *programmers_types; i++)
    if(str_caseeq(id, programmers_types[i].id))
      return programmers_types + i;

  return NULL;
}

// Return type id given the init function or "" if not found
const char *locate_programmer_type_id(void (*initpgm)(PROGRAMMER *pgm)) {
  for(size_t i = 0; i < sizeof programmers_types/sizeof *programmers_types; i++)
    if(programmers_types[i].initpgm == initpgm)
      return programmers_types[i].id;

  return "";
}

/*
 * Iterate over the list of programmer types given in the table above and
 * call the callback function cb for each entry found. cb is being
 * passed the following arguments:
 *  - Name of the programmer
 *  - Descriptive text
 *  - "Cookie" passed into walk_programmer_types() for opaque client data
 */

void walk_programmer_types(walk_programmer_types_cb cb, void *cookie) {
  for(size_t i = 0; i < sizeof(programmers_types)/sizeof(programmers_types[0]); i++)
    cb(programmers_types[i].id, programmers_types[i].desc, cookie);
}
