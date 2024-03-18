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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "updi_nvm.h"
#include "updi_nvm_v0.h"
#include "updi_nvm_v2.h"
#include "updi_nvm_v3.h"
#include "updi_nvm_v4.h"
#include "updi_nvm_v5.h"
#include "updi_state.h"

int updi_nvm_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_chip_erase_V0(pgm, p);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_chip_erase_V2(pgm, p);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_chip_erase_V3(pgm, p);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_chip_erase_V4(pgm, p);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_chip_erase_V5(pgm, p);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_erase_flash_page(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_erase_flash_page_V0(pgm, p, address);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_erase_flash_page_V2(pgm, p, address);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_erase_flash_page_V3(pgm, p, address);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_erase_flash_page_V4(pgm, p, address);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_erase_flash_page_V5(pgm, p, address);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_erase_eeprom(const PROGRAMMER *pgm, const AVRPART *p) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_erase_eeprom_V0(pgm, p);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_erase_eeprom_V2(pgm, p);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_erase_eeprom_V3(pgm, p);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_erase_eeprom_V4(pgm, p);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_erase_eeprom_V5(pgm, p);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_erase_user_row(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_erase_user_row_V0(pgm, p, address, size);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_erase_user_row_V2(pgm, p, address, size);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_erase_user_row_V3(pgm, p, address, size);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_erase_user_row_V4(pgm, p, address, size);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_erase_user_row_V5(pgm, p, address, size);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_flash(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_write_flash_V0(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_write_flash_V2(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_write_flash_V3(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_write_flash_V4(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_write_flash_V5(pgm, p, address, buffer, size);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_user_row(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_write_user_row_V0(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_write_user_row_V2(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_write_user_row_V3(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_write_user_row_V4(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_write_user_row_V5(pgm, p, address, buffer, size);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_eeprom(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_write_eeprom_V0(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_write_eeprom_V2(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_write_eeprom_V3(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_write_eeprom_V4(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_write_eeprom_V5(pgm, p, address, buffer, size);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_write_fuse_V0(pgm, p, address, value);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_write_fuse_V2(pgm, p, address, value);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_write_fuse_V3(pgm, p, address, value);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_write_fuse_V4(pgm, p, address, value);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_write_fuse_V5(pgm, p, address, value);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_wait_ready(const PROGRAMMER *pgm, const AVRPART *p) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_wait_ready_V0(pgm, p);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_wait_ready_V2(pgm, p);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_wait_ready_V3(pgm, p);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_wait_ready_V4(pgm, p);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_wait_ready_V5(pgm, p);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_command(const PROGRAMMER *pgm, const AVRPART *p, uint8_t command) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return updi_nvm_command_V0(pgm, p, command);
    case UPDI_NVM_MODE_V2:
      return updi_nvm_command_V2(pgm, p, command);
    case UPDI_NVM_MODE_V3:
      return updi_nvm_command_V3(pgm, p, command);
    case UPDI_NVM_MODE_V4:
      return updi_nvm_command_V4(pgm, p, command);
    case UPDI_NVM_MODE_V5:
      return updi_nvm_command_V5(pgm, p, command);
    default:
      pmsg_error("invalid NVM Mode %d\n", updi_get_nvm_mode(pgm));
      return -1;
  }
}
