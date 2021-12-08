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
 * Interface to the SerialUPDI programmer.
 *
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
#include "serialupdi.h"
#include "updi_link.h"
#include "updi_state.h"
#include "updi_readwrite.h"

static void serialupdi_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(updi_state))) == 0) {
    avrdude_message(MSG_INFO,
	    "%s: serialupdi_setup(): Out of memory allocating private data\n",
	    progname);
    exit(1);
  }
  memset(pgm->cookie, 0, sizeof(updi_state));
  updi_set_datalink_mode(pgm, UPDI_LINK_MODE_16BIT);
}

static void serialupdi_teardown(PROGRAMMER * pgm)
{
  free(pgm->cookie);
}

static int serialupdi_open(PROGRAMMER * pgm, char * port)
{
  strcpy(pgm->port, port);
  return updi_link_open(pgm);
}

static int serialupdi_decode_sib(updi_sib_info * sib_info)
{
  char * str_ptr;

  sib_info->sib_string[SIB_INFO_STRING_LENGTH]=0;
  avrdude_message(MSG_DEBUG, "%s: Received SIB: [%s]\n", progname, sib_info->sib_string);
  memset(sib_info->family_string, 0, SIB_INFO_FAMILY_LENGTH+1);
  memset(sib_info->nvm_string, 0, SIB_INFO_NVM_LENGTH+1);
  memset(sib_info->debug_string, 0, SIB_INFO_DEBUG_LENGTH+1);
  memset(sib_info->pdi_string, 0, SIB_INFO_PDI_LENGTH+1);
  memset(sib_info->pdi_string, 0, SIB_INFO_PDI_LENGTH+1);
  memset(sib_info->extra_string, 0, SIB_INFO_EXTRA_LENGTH+1);

  memcpy(sib_info->family_string, sib_info->sib_string, SIB_INFO_FAMILY_LENGTH);
  memcpy(sib_info->nvm_string, sib_info->sib_string + 8, SIB_INFO_NVM_LENGTH);
  memcpy(sib_info->debug_string, sib_info->sib_string + 11, SIB_INFO_DEBUG_LENGTH);
  memcpy(sib_info->pdi_string, sib_info->sib_string + 15, SIB_INFO_PDI_LENGTH);
  strcpy(sib_info->extra_string, (char *)sib_info->sib_string + 19);

  str_ptr = strstr(sib_info->nvm_string, ":");
  if (!str_ptr) {
    avrdude_message(MSG_INFO, "%s: Incorrect format of NVM string\n", progname);
    return -1;
  }
  sib_info->nvm_version = *(str_ptr+1);

  str_ptr = strstr(sib_info->debug_string, ":");
  if (!str_ptr) {
    avrdude_message(MSG_INFO, "%s: Incorrect format of DEBUG string\n", progname);
    return -1;
  }
  sib_info->debug_version = *(str_ptr+1);

  avrdude_message(MSG_DEBUG, "%s: Device family ID: %s\n", progname, sib_info->family_string);
  avrdude_message(MSG_DEBUG, "%s: NVM interface: %s\n", progname, sib_info->nvm_string);
  avrdude_message(MSG_DEBUG, "%s: Debug interface: %s\n", progname, sib_info->debug_string);
  avrdude_message(MSG_DEBUG, "%s: PDI oscillator: %s\n", progname, sib_info->pdi_string);
  avrdude_message(MSG_DEBUG, "%s: Extra information: %s\n", progname, sib_info->extra_string);
  switch (sib_info->nvm_version) {
    case '0':
      avrdude_message(MSG_INFO, "%s: NVM type 0: 16-bit, page oriented write\n", progname);
      break;
    case '2':
      avrdude_message(MSG_INFO, "%s: NVM type 2: 24-bit, word oriented write\n", progname);
      break;
    case '3':
      avrdude_message(MSG_INFO, "%s: NVM type 3: 16-bit, page oriented\n", progname);
      break;
    default:
      avrdude_message(MSG_INFO, "%s: Unsupported NVM type: %c, please update software\n", progname, sib_info->nvm_version);
      return -1;
  }
  return 0;
}

static void serialupdi_close(PROGRAMMER * pgm)
{
  updi_link_close(pgm);
}

static int serialupdi_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  updi_sib_info * sib_info = updi_get_sib_info(pgm);

  if (updi_link_init(pgm) < 0) {
    avrdude_message(MSG_INFO, "%s: UPDI link initialization failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_INFO, "%s: UPDI link initialization OK\n", progname);
  if (updi_read_sib(pgm, sib_info->sib_string, 32) < 0) {
    avrdude_message(MSG_INFO, "%s: Read SIB operation failed\n", progname);
    return -1;
  }
  if (serialupdi_decode_sib(sib_info) < 0) {
    avrdude_message(MSG_INFO, "%s: Decode SIB_INFO failed\n", progname);
    return -1;
  }
  updi_set_nvm_mode(pgm, sib_info->nvm_version);
  if (sib_info->nvm_version == '2') {
    updi_set_datalink_mode(pgm, UPDI_LINK_MODE_24BIT);
  } else {
    updi_set_datalink_mode(pgm, UPDI_LINK_MODE_16BIT);
  }
  
  return 0;
}

static void serialupdi_disable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}

static void serialupdi_enable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}

static void serialupdi_display(PROGRAMMER * pgm, const char * p)
{
  return;
}

static int serialupdi_cmd(PROGRAMMER * pgm, const unsigned char * cmd,
                          unsigned char * res)
{
  avrdude_message(MSG_INFO, "%s: error: cmd %s[%s] not implemented yet\n",
    	    progname, cmd, res);
  return -1;
}

static int serialupdi_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  avrdude_message(MSG_INFO, "%s: error: program enable not implemented yet\n",
    	    progname);
  return -1;
}

static int serialupdi_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  avrdude_message(MSG_INFO, "%s: error: chip erase not implemented yet\n",
    	    progname);
  return -1;
}

static int serialupdi_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem, 
                                unsigned long addr, unsigned char * value)
{
//  avrdude_message(MSG_INFO, "%s: error: read byte not implemented yet\n",
//    	            progname);
  return updi_read_byte(pgm, mem->offset + addr, value);
//  return -1;
}

static int serialupdi_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                 unsigned int page_size,
                                 unsigned int addr, unsigned int n_bytes)
{
  avrdude_message(MSG_INFO, "%s: error: paged load not implemented yet\n",
    	    progname);
  return -1;
}

static int serialupdi_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                  unsigned int page_size,
                                  unsigned int addr, unsigned int n_bytes)
{
  avrdude_message(MSG_INFO, "%s: error: paged write not implemented yet\n",
    	    progname);
  return -1;
}

void serialupdi_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "serialupdi");

  /*
   * mandatory functions
   */

  pgm->initialize     = serialupdi_initialize;
  pgm->display        = serialupdi_display;
  pgm->enable         = serialupdi_enable;
  pgm->disable        = serialupdi_disable;
  pgm->program_enable = serialupdi_program_enable;
  pgm->chip_erase     = serialupdi_chip_erase;
  pgm->cmd            = serialupdi_cmd;
  pgm->open           = serialupdi_open;
  pgm->close          = serialupdi_close;
  pgm->read_byte      = serialupdi_read_byte;
  pgm->write_byte     = avr_write_byte_default;

  /*
   * optional functions
   */

  pgm->paged_write    = serialupdi_paged_write;
  pgm->paged_load     = serialupdi_paged_load;
  pgm->setup          = serialupdi_setup;
  pgm->teardown       = serialupdi_teardown;

}

const char serialupdi_desc[] = "Driver for SerialUPDI programmers";
