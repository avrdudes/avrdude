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
#include "updi_nvm.h"
#include "updi_constants.h"

static int serialupdi_enter_progmode(PROGRAMMER * pgm);
static int serialupdi_leave_progmode(PROGRAMMER * pgm);

static void serialupdi_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(updi_state))) == 0) {
    avrdude_message(MSG_INFO,
	    "%s: serialupdi_setup(): Out of memory allocating private data\n",
	    progname);
    exit(1);
  }
  memset(pgm->cookie, 0, sizeof(updi_state));
  updi_set_rts_mode(pgm, RTS_MODE_DEFAULT);
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

typedef enum {
  APPLY_RESET,
  RELEASE_RESET
} reset_mode;

static int serialupdi_reset(PROGRAMMER * pgm, reset_mode mode)
{
/*
    def reset(self, apply_reset):
        """
        Applies or releases an UPDI reset condition

        :param apply_reset: True to apply, False to release
        """
        if apply_reset:
            self.logger.info("Apply reset")
            self.readwrite.write_cs(constants.UPDI_ASI_RESET_REQ, constants.UPDI_RESET_REQ_VALUE)
        else:
            self.logger.info("Release reset")
            self.readwrite.write_cs(constants.UPDI_ASI_RESET_REQ, 0x00)
*/
  switch (mode) {
    case APPLY_RESET:
      avrdude_message(MSG_DEBUG, "%s: Sending reset request\n", progname);
      return updi_write_cs(pgm, UPDI_ASI_RESET_REQ, UPDI_RESET_REQ_VALUE);
    case RELEASE_RESET:
      avrdude_message(MSG_DEBUG, "%s: Sending release reset request\n", progname);
      return updi_write_cs(pgm, UPDI_ASI_RESET_REQ, 0x00);
  }
  return -1;
}

static int serialupdi_reset_connection(PROGRAMMER * pgm)
{
  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Apply reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Release reset operation failed\n", progname);
    return -1;
  }

  return updi_link_init(pgm);
}

static int serialupdi_decode_sib(PROGRAMMER * pgm, updi_sib_info * sib_info)
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
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V0);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_16BIT);
      break;
    case '2':
      avrdude_message(MSG_INFO, "%s: NVM type 2: 24-bit, word oriented write\n", progname);
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V2);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_24BIT);
      break;
    case '3':
      avrdude_message(MSG_INFO, "%s: NVM type 3: 16-bit, page oriented\n", progname);
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V3);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_16BIT);
      break;
    default:
      avrdude_message(MSG_INFO, "%s: Unsupported NVM type: %c, please update software\n", progname, sib_info->nvm_version);
      return -1;
  }
  return 0;
}

static void serialupdi_close(PROGRAMMER * pgm)
{
  avrdude_message(MSG_INFO, "%s: Leaving NVM programming mode\n", progname);

  if (serialupdi_leave_progmode(pgm) < 0) {
    avrdude_message(MSG_INFO, "%s: Unable to leave NVM programming mode\n", progname);
  }
  if (updi_get_rts_mode(pgm) != RTS_MODE_DEFAULT) {
    avrdude_message(MSG_INFO, "%s: Releasing DTR/RTS handshake lines\n", progname);
  }

  updi_link_close(pgm);
}

static int serialupdi_wait_for_unlock(PROGRAMMER * pgm, unsigned int ms) {
/*
    def wait_unlocked(self, timeout_ms):
        """
        Waits for the device to be unlocked.
        All devices boot up as locked until proven otherwise

        :param timeout_ms: number of milliseconds to wait
        """
        timeout = Timeout(timeout_ms)

        while not timeout.expired():
            if not self.readwrite.read_cs(constants.UPDI_ASI_SYS_STATUS) & (
                    1 << constants.UPDI_ASI_SYS_STATUS_LOCKSTATUS):
                return True

        self.logger.error("Timeout waiting for device to unlock")
        return False
*/  
  unsigned long start_time;
  unsigned long current_time;
  struct timeval tv;
  uint8_t status;
  gettimeofday (&tv, NULL);
  start_time = (tv.tv_sec * 1000000) + tv.tv_usec;
  do {
    if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &status) >= 0) {
      if (!(status & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS))) {
        return 0;
      }
    }
    gettimeofday (&tv, NULL);
    current_time = (tv.tv_sec * 1000000) + tv.tv_usec;
  } while ((current_time - start_time) < (ms * 1000));

  avrdude_message(MSG_INFO, "%s: Timeout waiting for device to unlock\n", progname);
  return -1;
}

typedef enum {
  WAIT_FOR_UROW_LOW,
  WAIT_FOR_UROW_HIGH
} urow_wait_mode;

static int serialupdi_wait_for_urow(PROGRAMMER * pgm, unsigned int ms, urow_wait_mode mode) {
/*
    def wait_urow_prog(self, timeout_ms, wait_for_high):
        """
        Waits for the device to be in user row write mode
        User row is writeable on a locked device using this mechanism

        :param timeout_ms: number of milliseconds to wait
        :param wait_for_high: set True to wait for bit to go high; False to wait for low
        """
        timeout = Timeout(timeout_ms)

        while not timeout.expired():
            status = self.readwrite.read_cs(constants.UPDI_ASI_SYS_STATUS)
            if wait_for_high:
                if status & (1 << constants.UPDI_ASI_SYS_STATUS_UROWPROG):
                    return True
            else:
                if not status & (1 << constants.UPDI_ASI_SYS_STATUS_UROWPROG):
                    return True

        self.logger.error("Timeout waiting for device to enter UROW WRITE mode")
        return False
*/  
  unsigned long start_time;
  unsigned long current_time;
  struct timeval tv;
  uint8_t status;
  gettimeofday (&tv, NULL);
  start_time = (tv.tv_sec * 1000000) + tv.tv_usec;
  do {
    if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &status) >= 0) {
      if (mode == WAIT_FOR_UROW_HIGH) {
        if (status & (1 << UPDI_ASI_SYS_STATUS_UROWPROG)) {
          return 0;
        }
      } else {
        if (!(status & (1 << UPDI_ASI_SYS_STATUS_UROWPROG))) {
          return 0;
        }
      }
    }
    gettimeofday (&tv, NULL);
    current_time = (tv.tv_sec * 1000000) + tv.tv_usec;
  } while ((current_time - start_time) < (ms * 1000));

  avrdude_message(MSG_INFO, "%s: Timeout waiting for device to complete UROW WRITE\n", progname);
  return -1;
}

static int serialupdi_in_prog_mode(PROGRAMMER * pgm, uint8_t * in_prog_mode)
{
/*
    def in_prog_mode(self):
        """
        Checks whether the NVM PROG flag is up
        """
        if self.readwrite.read_cs(constants.UPDI_ASI_SYS_STATUS) & (1 << constants.UPDI_ASI_SYS_STATUS_NVMPROG):
            return True
        return False
*/
  uint8_t value;
  int rc;
  
  rc = updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value);
  
  if (rc < 0) {
    avrdude_message(MSG_INFO, "%s: Read CS operation failed\n", progname);
    return rc;
  }

  if (value & (1 << UPDI_ASI_SYS_STATUS_NVMPROG)) {
    *in_prog_mode = 1;
  } else {
    *in_prog_mode = 0;
  }
  return 0;
}

static int serialupdi_enter_progmode(PROGRAMMER * pgm)
{
/*
def enter_progmode(self):
        """
        Enters into NVM programming mode
        """
        # First check if NVM is already enabled
        if self.in_prog_mode():
            self.logger.info("Already in NVM programming mode")
            return True

        self.logger.info("Entering NVM programming mode")

        # Put in the key
        self.readwrite.write_key(constants.UPDI_KEY_64, constants.UPDI_KEY_NVM)

        # Check key status
        key_status = self.readwrite.read_cs(constants.UPDI_ASI_KEY_STATUS)
        self.logger.debug("Key status = 0x%02X", key_status)

        if not key_status & (1 << constants.UPDI_ASI_KEY_STATUS_NVMPROG):
            self.logger.error("Key status = 0x%02X", key_status)
            raise IOError("Key not accepted")

        # Toggle reset
        self.reset(apply_reset=True)
        self.reset(apply_reset=False)

        # And wait for unlock
        if not self.wait_unlocked(100):
            raise IOError("Failed to enter NVM programming mode: device is locked")

        # Check for NVMPROG flag
        if not self.in_prog_mode():
            raise IOError("Failed to enter NVM programming mode")

        self.logger.debug("Now in NVM programming mode")
        return True
*/
  uint8_t in_prog_mode;
  unsigned char buffer[8];
  uint8_t key_status;

  if (serialupdi_in_prog_mode(pgm, &in_prog_mode) < 0) {
    avrdude_message(MSG_INFO, "%s: Checking UPDI NVM prog mode failed\n", progname);
    return -1;
  }
  if (in_prog_mode) {
    avrdude_message(MSG_DEBUG, "%s: Already in prog mode\n", progname);
    return 0;
  }

  memcpy(buffer, UPDI_KEY_NVM, sizeof(buffer));
  if (updi_write_key(pgm, buffer, UPDI_KEY_64, sizeof(buffer)) < 0) {
    avrdude_message(MSG_INFO, "%s: Writing NVM KEY failed\n", progname);
    return -1;
  }

  if (updi_read_cs(pgm, UPDI_ASI_KEY_STATUS, &key_status) < 0) {
    avrdude_message(MSG_INFO, "%s: Checking KEY status failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Key status: 0x%02X\n", progname, key_status);

  if (!(key_status & (1 << UPDI_ASI_KEY_STATUS_NVMPROG))) {
    avrdude_message(MSG_INFO, "%s: Key was not accepted\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Apply reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Release reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_wait_for_unlock(pgm, 100) < 0) {
    avrdude_message(MSG_INFO, "%s: Failed to enter NVM programming mode: device is locked\n", progname);
    return -1;
  }

  if (serialupdi_in_prog_mode(pgm, &in_prog_mode) < 0) {
    avrdude_message(MSG_INFO, "%s: Checking UPDI NVM prog mode failed\n", progname);
    return -1;
  }

  if (!in_prog_mode) {
    avrdude_message(MSG_INFO, "%s: Failed to enter NVM programming mode\n", progname);
    return -1;
  }

  avrdude_message(MSG_DEBUG, "%s: Entered NVM programming mode\n", progname);

  return 0;
}

static int serialupdi_leave_progmode(PROGRAMMER * pgm)
{
/*
    def leave_progmode(self):
        """
        Disables UPDI which releases any keys enabled
        """
        self.logger.info("Leaving NVM programming mode")
        self.reset(apply_reset=True)
        self.reset(apply_reset=False)
        self.readwrite.write_cs(constants.UPDI_CS_CTRLB,
                                (1 << constants.UPDI_CTRLB_UPDIDIS_BIT) | (1 << constants.UPDI_CTRLB_CCDETDIS_BIT))
*/
  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Apply reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Release reset operation failed\n", progname);
    return -1;
  }

  return updi_write_cs(pgm, UPDI_CS_CTRLB, (1 << UPDI_CTRLB_UPDIDIS_BIT) | (1 << UPDI_CTRLB_CCDETDIS_BIT));
}

static int serialupdi_write_userrow(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                    unsigned int page_size,
                                    unsigned int addr, unsigned int n_bytes)
{
  /*
    def write_user_row_locked_device(self, address, data):
        """
        Writes data to the user row when the device is locked, using a key.
        """
        # Put in the key
        self.readwrite.write_key(constants.UPDI_KEY_64, constants.UPDI_KEY_UROW)

        # Check key status
        key_status = self.readwrite.read_cs(constants.UPDI_ASI_KEY_STATUS)
        self.logger.debug("Key status = 0x%02X", key_status)

        if not key_status & (1 << constants.UPDI_ASI_KEY_STATUS_UROWWRITE):
            raise PymcuprogError("Key not accepted")

        # Toggle reset
        self.reset(apply_reset=True)
        self.reset(apply_reset=False)

        # Wait for mode to be entered
        if not self.wait_urow_prog(500, wait_for_high=True):
            raise PymcuprogError("Failed to enter urow write mode using key")

        # At this point we can write one 'page' to the device, and have it transfered into the user row
        # Transfer data
        self.readwrite.write_data(address, data)

        # Finalize
        self.readwrite.write_cs(constants.UPDI_ASI_SYS_CTRLA,
                                (1 << constants.UPDI_ASI_SYS_CTRLA_UROW_FINAL) |
                                (1 << constants.UPDI_CTRLB_CCDETDIS_BIT))

        # Wait for mode to be exited
        if not self.wait_urow_prog(500, wait_for_high=False):
            # Toggle reset
            self.reset(apply_reset=True)
            self.reset(apply_reset=False)
            raise PymcuprogError("Failed to exit urow write mode")

        # Clear status
        self.readwrite.write_cs(constants.UPDI_ASI_KEY_STATUS,
                                (1 << constants.UPDI_ASI_KEY_STATUS_UROWWRITE) |
                                (1 << constants.UPDI_CTRLB_CCDETDIS_BIT))

        # Toggle reset
        self.reset(apply_reset=True)
        self.reset(apply_reset=False)
  */
  unsigned char buffer[8];
  uint8_t key_status;

  memcpy(buffer, UPDI_KEY_UROW, sizeof(buffer));
  if (updi_write_key(pgm, buffer, UPDI_KEY_64, sizeof(buffer)) < 0) {
    avrdude_message(MSG_INFO, "%s: Writing USERROW KEY failed\n", progname);
    return -1;
  }

  if (updi_read_cs(pgm, UPDI_ASI_KEY_STATUS, &key_status) < 0) {
    avrdude_message(MSG_INFO, "%s: Checking KEY status failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Key status: 0x%02X\n", progname, key_status);

  if (!(key_status & (1 << UPDI_ASI_KEY_STATUS_UROWWRITE))) {
    avrdude_message(MSG_INFO, "%s: Key was not accepted\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Apply reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Release reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_wait_for_urow(pgm, 500, WAIT_FOR_UROW_HIGH) < 0) {
    avrdude_message(MSG_INFO, "%s: Failed to enter USERROW programming mode\n", progname);
    return -1;
  }

  if (updi_write_data(pgm, m->offset+addr, m->buf + addr, n_bytes) < 0) {
    avrdude_message(MSG_INFO, "%s: Writing USER ROW failed\n", progname);
    return -1;
  }

  if (updi_write_cs(pgm, UPDI_ASI_SYS_CTRLA, (1 << UPDI_ASI_SYS_CTRLA_UROW_FINAL) |
                                             (1 << UPDI_CTRLB_CCDETDIS_BIT)) < 0) {
    avrdude_message(MSG_INFO, "%s: Failed trying to commit user row write\n", progname);
    return -1;
  }

  if (serialupdi_wait_for_urow(pgm, 500, WAIT_FOR_UROW_LOW) < 0) {
    avrdude_message(MSG_DEBUG, "%s: Failed to exit USERROW programming mode\n", progname);

    if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
      avrdude_message(MSG_INFO, "%s: Apply reset operation failed\n", progname);
      return -1;
    }

    if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
      avrdude_message(MSG_INFO, "%s: Release reset operation failed\n", progname);
      return -1;
    }
  }

  if (updi_write_cs(pgm, UPDI_ASI_KEY_STATUS, (1 << UPDI_ASI_KEY_STATUS_UROWWRITE) |
                                              (1 << UPDI_CTRLB_CCDETDIS_BIT)) < 0) {
    avrdude_message(MSG_INFO, "%s: Failed trying to complete user row write\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Apply reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Release reset operation failed\n", progname);
    return -1;
  }

  serialupdi_reset_connection(pgm);

  serialupdi_enter_progmode(pgm);

  return 0;
}

static int serialupdi_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  uint8_t value;
  uint8_t reset_link_required=0;
  
  if (updi_link_init(pgm) < 0) {
    avrdude_message(MSG_INFO, "%s: UPDI link initialization failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_INFO, "%s: UPDI link initialization OK\n", progname);

  if (updi_get_rts_mode(pgm) != RTS_MODE_DEFAULT) {
    avrdude_message(MSG_INFO, "%s: Forcing serial DTR/RTS handshake lines %s\n", progname, updi_get_rts_mode(pgm) == RTS_MODE_LOW ? "LOW" : "HIGH");
  }

  if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {

    /* let's try reset the connection */
    if (!serialupdi_reset_connection(pgm)) {
      return -1;
    }

    if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {
      avrdude_message(MSG_INFO, "%s: Read CS operation during initialization failed\n", progname);
      return -1;
    }
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS)) {
    avrdude_message(MSG_INFO, "%s: Device is locked\n", progname);
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_UROWPROG)) {
    avrdude_message(MSG_INFO, "%s: Device in USER ROW programming state, leaving programming mode\n", progname);
    reset_link_required = 1;
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_NVMPROG)) {
    avrdude_message(MSG_INFO, "%s: Device in NVM programming state, leaving programming mode\n", progname);
    reset_link_required = 1;
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_INSLEEP)) {
    avrdude_message(MSG_INFO, "%s: Device is in SLEEP mode\n", progname);
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_RSTSYS)) {
    avrdude_message(MSG_INFO, "%s: Device in reset status, trying to release it\n", progname);
    if (serialupdi_reset(pgm, RELEASE_RESET)<0) {
      return -1;
    }
  }
  if (reset_link_required) {
    if (serialupdi_reset_connection(pgm) < 0) {
      avrdude_message(MSG_INFO, "%s: UPDI link reset failed\n", progname);
      return -1;
    }
  }

  updi_sib_info * sib_info = updi_get_sib_info(pgm);

  if (updi_read_sib(pgm, sib_info->sib_string, 32) < 0) {
    /* this should never happen, let's try to reset connection and try again */
    if (serialupdi_reset_connection(pgm) < 0) {
      avrdude_message(MSG_INFO, "%s: SerialUPDI reset connection failed\n", progname);  
      return -1;
    }
    if (updi_read_sib(pgm, sib_info->sib_string, 32) < 0) {
      avrdude_message(MSG_INFO, "%s: Read SIB operation failed\n", progname);  
      return -1;
    }
  }
  if (serialupdi_decode_sib(pgm, sib_info) < 0) {
    avrdude_message(MSG_INFO, "%s: Decode SIB_INFO failed\n", progname);
    return -1;
  }

  if (updi_link_init(pgm) < 0) {
    avrdude_message(MSG_INFO, "%s: UPDI link initialization failed\n", progname);
    return -1;
  }

  avrdude_message(MSG_INFO, "%s: Entering NVM programming mode\n", progname);
    /* try, but ignore failure */
  serialupdi_enter_progmode(pgm);

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

static int serialupdi_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem, 
                                unsigned long addr, unsigned char * value)
{
  return updi_read_byte(pgm, mem->offset + addr, value);
}

static int serialupdi_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                                 unsigned long addr, unsigned char value)
{
  if (strstr(mem->desc, "fuse") != 0) {
    return updi_nvm_write_fuse(pgm, p, mem->offset + addr, value);
  }
  if (strcmp(mem->desc, "lock") == 0) {
    return updi_nvm_write_fuse(pgm, p, mem->offset + addr, value);
  }
  if (strcmp(mem->desc, "eeprom") == 0) {
    unsigned char buffer[1];
    buffer[0]=value;
    return updi_nvm_write_eeprom(pgm, p, mem->offset + addr, buffer, 1);
  }
  if (strcmp(mem->desc, "flash") == 0) {
    unsigned char buffer[1];
    buffer[0]=value;
    return updi_nvm_write_flash(pgm, p, mem->offset + addr, buffer, 1);
  }
  return updi_write_byte(pgm, mem->offset + addr, value);
}


static int serialupdi_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                 unsigned int page_size,
                                 unsigned int addr, unsigned int n_bytes)
{
  if (n_bytes > m->readsize) {
    unsigned int read_offset = addr;
    unsigned int remaining_bytes = n_bytes;
    int read_bytes = 0;
    int rc;
    while (remaining_bytes > 0) {
      rc = updi_read_data(pgm, m->offset + read_offset, m->buf + read_offset, 
                          remaining_bytes > m->readsize ? m->readsize : remaining_bytes);
      if (rc < 0) {
        avrdude_message(MSG_INFO, "%s: Paged load operation failed\n", progname);
        return rc;
      } else {
        read_bytes+=rc;
        read_offset+=m->readsize;
        remaining_bytes-=m->readsize;
      }
    }
    return read_bytes;
  } else {
    return updi_read_data(pgm, m->offset + addr, m->buf + addr, n_bytes);
  }
}

static int serialupdi_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                  unsigned int page_size,
                                  unsigned int addr, unsigned int n_bytes)
{
  int rc;
  if (n_bytes > m->page_size) {
    unsigned int write_offset = addr;
    unsigned int remaining_bytes = n_bytes;
    int write_bytes = 0;
    while (remaining_bytes > 0) {

      if (strcmp(m->desc, "eeprom")==0) {
        rc = updi_nvm_write_eeprom(pgm, p, m->offset + write_offset, m->buf + write_offset, 
                                   remaining_bytes > m->page_size ? m->page_size : remaining_bytes);
      } else if (strcmp(m->desc, "flash")==0) {
        rc = updi_nvm_write_flash(pgm, p, m->offset + write_offset, m->buf + write_offset, 
                                  remaining_bytes > m->page_size ? m->page_size : remaining_bytes);
      } else if (strcmp(m->desc, "userrow")==0) {
        rc = serialupdi_write_userrow(pgm, p, m, page_size, write_offset, 
                                      remaining_bytes > m->page_size ? m->page_size : remaining_bytes);
      } else if (strcmp(m->desc, "fuses")==0) {
        avrdude_message(MSG_DEBUG, "%s: Page write operation requested for fuses, falling back to byte-level write\n", progname);
        return -1;
      } else {
        avrdude_message(MSG_INFO, "%s: Invalid memory type: <%s:%d>, 0x%06X, %d (0x%04X)\n", progname, m->desc, page_size, addr, n_bytes, n_bytes);
        rc = -1;
      }

      if (rc < 0) {
        avrdude_message(MSG_INFO, "%s: Paged write operation failed\n", progname);
        return rc;
      } else {
        write_bytes+=rc;
        write_offset+=m->page_size;
        remaining_bytes-=m->page_size;
      }
    }
    return write_bytes;
  } else {
    if (strcmp(m->desc, "eeprom")==0) {
      rc = updi_nvm_write_eeprom(pgm, p, m->offset+addr, m->buf+addr, n_bytes);
    } else if (strcmp(m->desc, "flash")==0) {
      rc = updi_nvm_write_flash(pgm, p, m->offset+addr, m->buf+addr, n_bytes);
    } else if (strcmp(m->desc, "userrow")==0) {
      rc = serialupdi_write_userrow(pgm, p, m, page_size, addr, n_bytes);
    } else if (strcmp(m->desc, "fuses")==0) {
        avrdude_message(MSG_DEBUG, "%s: Page write operation requested for fuses, falling back to byte-level write\n", progname);
        rc = -1;
    } else {
      avrdude_message(MSG_INFO, "%s: Invalid memory type: <%s:%d>, 0x%06X, %d (0x%04X)\n", progname, m->desc, page_size, addr, n_bytes, n_bytes);
      rc = -1;
    }
    return rc;
  }
}

static int serialupdi_unlock(PROGRAMMER * pgm, AVRPART * p)
{
/*
    def unlock(self):
        """
        Unlock by chip erase
        """
        # Put in the key
        self.readwrite.write_key(constants.UPDI_KEY_64, constants.UPDI_KEY_CHIPERASE)

        # Check key status
        key_status = self.readwrite.read_cs(constants.UPDI_ASI_KEY_STATUS)
        self.logger.debug("Key status = 0x%02X", key_status)

        if not key_status & (1 << constants.UPDI_ASI_KEY_STATUS_CHIPERASE):
            raise PymcuprogError("Key not accepted")

        # Toggle reset
        self.reset(apply_reset=True)
        self.reset(apply_reset=False)

        # And wait for unlock
        if not self.wait_unlocked(500):
            raise PymcuprogError("Failed to chip erase using key")
*/
  unsigned char buffer[8];
  uint8_t key_status;
  
  memcpy(buffer, UPDI_KEY_CHIPERASE, sizeof(buffer));

  if (updi_write_key(pgm, buffer, UPDI_KEY_64, sizeof(buffer)) < 0) {
    avrdude_message(MSG_INFO, "%s: Writing NVM KEY failed\n", progname);
    return -1;
  }

  if (updi_read_cs(pgm, UPDI_ASI_KEY_STATUS, &key_status) < 0) {
    avrdude_message(MSG_INFO, "%s: Checking KEY status failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Key status: 0x%02X\n", progname, key_status);

  if (!(key_status & (1 << UPDI_ASI_KEY_STATUS_CHIPERASE))) {
    avrdude_message(MSG_INFO, "%s: Key not accepted\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Apply reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    avrdude_message(MSG_INFO, "%s: Release reset operation failed\n", progname);
    return -1;
  }

  if (serialupdi_wait_for_unlock(pgm, 500) < 0) {
    avrdude_message(MSG_INFO, "%s: Waiting for unlock failed\n", progname);
    return -1;
  }

  if (updi_link_init(pgm) < 0) {
    avrdude_message(MSG_INFO, "%s: UPDI link reinitialization failed\n", progname);
    return -1;
  }

  return serialupdi_enter_progmode(pgm);
}

static int serialupdi_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  uint8_t value;

  if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {
    avrdude_message(MSG_INFO, "%s: Read CS operation during chip erase failed\n", progname);
    return -1;
  }
  
  if (value & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS)) {
    avrdude_message(MSG_INFO, "%s: Device is locked\n", progname);
    if (ovsigck) {
      avrdude_message(MSG_INFO, "%s: Attempting device erase\n", progname);
      return serialupdi_unlock(pgm, p);
    }
  } else {
    return updi_nvm_chip_erase(pgm, p);
  }
  return -1;
}

static int serialupdi_page_erase(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                 unsigned int baseaddr)
{
  avrdude_message(MSG_INFO, "%s: error: page erase not implemented yet\n",
    	    progname);
  return -1;
}

static int serialupdi_read_signature(PROGRAMMER * pgm, AVRPART *p, AVRMEM *m) {

  uint8_t value;

  if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {
    avrdude_message(MSG_INFO, "%s: Read CS operation during signature read failed\n", progname);
    return -1;
  }

  if (value & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS)) {
    m->buf[0]=0x00;
    m->buf[1]=0x00;
    m->buf[2]=0x00;
  } else {
    updi_read_byte(pgm, m->offset + 0, m->buf);
    updi_read_byte(pgm, m->offset + 1, m->buf+1);
    updi_read_byte(pgm, m->offset + 2, m->buf+2);
  }

  return 3;
}

static int serialupdi_read_sib(PROGRAMMER * pgm, AVRPART *p, char *sib) {

  updi_sib_info * sib_info = updi_get_sib_info(pgm);

  memcpy(sib, sib_info->sib_string, 32);
  
  return 0;
}

static int serialupdi_parseextparms(PROGRAMMER * pgm, LISTID extparms)
{
  LNODEID ln;
  const char *extended_param;
  char rts_mode[5];
  int rv = 0;

  for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

		if (sscanf(extended_param, "rtsdtr=%4s", rts_mode) == 1) {
      if (strcasecmp(rts_mode, "low") == 0) {
        updi_set_rts_mode(pgm, RTS_MODE_LOW);
      } else if (strcasecmp(rts_mode, "high") == 0) {
        updi_set_rts_mode(pgm, RTS_MODE_HIGH);
      } else {
        avrdude_message(MSG_INFO, "%s: RTS/DTR mode must be LOW or HIGH\n", progname);
        return -1;
      }
			continue;
		}

    avrdude_message(MSG_INFO, "%s: serialupdi_parseextparms(): invalid extended parameter '%s'\n",
                    progname, extended_param);
    rv = -1;
  }

  return rv;
}

void serialupdi_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "serialupdi");

  /*
   * mandatory functions
   */

  pgm->initialize     = serialupdi_initialize;
  pgm->parseextparams = serialupdi_parseextparms;
  pgm->display        = serialupdi_display;
  pgm->enable         = serialupdi_enable;
  pgm->disable        = serialupdi_disable;
  pgm->program_enable = serialupdi_program_enable;
  pgm->chip_erase     = serialupdi_chip_erase;
  pgm->cmd            = serialupdi_cmd;
  pgm->open           = serialupdi_open;
  pgm->close          = serialupdi_close;
  pgm->read_byte      = serialupdi_read_byte;
  pgm->write_byte     = serialupdi_write_byte;

  /*
   * optional functions
   */

  pgm->unlock         = serialupdi_unlock;
  pgm->paged_write    = serialupdi_paged_write;
  pgm->read_sig_bytes = serialupdi_read_signature;
  pgm->read_sib       = serialupdi_read_sib;
  pgm->paged_load     = serialupdi_paged_load;
  pgm->page_erase     = serialupdi_page_erase;
  pgm->setup          = serialupdi_setup;
  pgm->teardown       = serialupdi_teardown;

}

const char serialupdi_desc[] = "Driver for SerialUPDI programmers";
