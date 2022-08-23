/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2021  Dawid Buchwald
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */

/*
 * Based on pymcuprog
 * See https://github.com/microchip-pic-avr-tools/pymcuprog
 */

#include "ac_cfg.h"

#include "libavrdude.h"
#include "updi_state.h"

updi_sib_info* updi_get_sib_info(const PROGRAMMER *pgm) {
  return &((updi_state *)(pgm->cookie))->sib_info;
}

updi_datalink_mode updi_get_datalink_mode(const PROGRAMMER *pgm) {
  return ((updi_state *)(pgm->cookie))->datalink_mode;
}

void updi_set_datalink_mode(const PROGRAMMER *pgm, updi_datalink_mode mode) {
  ((updi_state *)(pgm->cookie))->datalink_mode = mode;
}

updi_nvm_mode updi_get_nvm_mode(const PROGRAMMER *pgm) {
  return ((updi_state *)(pgm->cookie))->nvm_mode;
}

void updi_set_nvm_mode(const PROGRAMMER *pgm, updi_nvm_mode mode) {
  ((updi_state *)(pgm->cookie))->nvm_mode = mode;
}

updi_rts_mode updi_get_rts_mode(const PROGRAMMER *pgm) {
  return ((updi_state *)(pgm->cookie))->rts_mode;
}

void updi_set_rts_mode(const PROGRAMMER *pgm, updi_rts_mode mode) {
  ((updi_state *)(pgm->cookie))->rts_mode = mode;
}
