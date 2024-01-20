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

#ifndef updi_state_h
#define updi_state_h

#include "libavrdude.h"

typedef enum
{
  UPDI_LINK_MODE_16BIT,
  UPDI_LINK_MODE_24BIT
} updi_datalink_mode;

typedef enum
{
  UPDI_NVM_MODE_V0,
  UPDI_NVM_MODE_V2,
  UPDI_NVM_MODE_V3,
  UPDI_NVM_MODE_V4,
  UPDI_NVM_MODE_V5
} updi_nvm_mode;

#define SIB_INFO_STRING_LENGTH 32
#define SIB_INFO_FAMILY_LENGTH 8
#define SIB_INFO_NVM_LENGTH    3
#define SIB_INFO_DEBUG_LENGTH  3
#define SIB_INFO_PDI_LENGTH    4
#define SIB_INFO_EXTRA_LENGTH  20

typedef struct 
{
  unsigned char sib_string[SIB_INFO_STRING_LENGTH+1];
  char family_string[SIB_INFO_FAMILY_LENGTH+1];
  char nvm_string[SIB_INFO_NVM_LENGTH+1];
  char debug_string[SIB_INFO_DEBUG_LENGTH+1];
  char pdi_string[SIB_INFO_PDI_LENGTH+1];
  char extra_string[SIB_INFO_EXTRA_LENGTH+1];
  char nvm_version;
  char debug_version;
} updi_sib_info;

typedef enum
{
  RTS_MODE_DEFAULT,
  RTS_MODE_LOW,
  RTS_MODE_HIGH
} updi_rts_mode;

typedef struct
{
  updi_sib_info sib_info;
  updi_datalink_mode datalink_mode;
  updi_nvm_mode nvm_mode;
  updi_rts_mode rts_mode;
} updi_state;

#ifdef __cplusplus
extern "C" {
#endif

updi_sib_info* updi_get_sib_info(const PROGRAMMER *pgm);
updi_datalink_mode updi_get_datalink_mode(const PROGRAMMER *pgm);
void updi_set_datalink_mode(const PROGRAMMER *pgm, updi_datalink_mode mode);
updi_nvm_mode updi_get_nvm_mode(const PROGRAMMER *pgm);
void updi_set_nvm_mode(const PROGRAMMER *pgm, updi_nvm_mode mode);
updi_rts_mode updi_get_rts_mode(const PROGRAMMER *pgm);
void updi_set_rts_mode(const PROGRAMMER *pgm, updi_rts_mode mode);

#ifdef __cplusplus
}
#endif

#endif /* updi_state_h */
