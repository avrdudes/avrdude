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

static int serialupdi_enter_progmode(const PROGRAMMER *pgm);
static int serialupdi_leave_progmode(const PROGRAMMER *pgm);

static void serialupdi_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(updi_state))) == 0) {
    pmsg_error("out of memory allocating private data\n");
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

static int serialupdi_open(PROGRAMMER *pgm, const char *port) {
  strcpy(pgm->port, port);
  return updi_link_open(pgm);
}

typedef enum {
  APPLY_RESET,
  RELEASE_RESET
} reset_mode;

static int serialupdi_reset(const PROGRAMMER *pgm, reset_mode mode) {
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
      pmsg_debug("sending reset request\n");
      return updi_write_cs(pgm, UPDI_ASI_RESET_REQ, UPDI_RESET_REQ_VALUE);
    case RELEASE_RESET:
      pmsg_debug("sending release reset request\n");
      return updi_write_cs(pgm, UPDI_ASI_RESET_REQ, 0x00);
  }
  return -1;
}

static int serialupdi_reset_connection(const PROGRAMMER *pgm) {
  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    pmsg_error("apply reset operation failed\n");
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    pmsg_error("release reset operation failed\n");
    return -1;
  }

  return updi_link_init(pgm);
}

static int serialupdi_decode_sib(const PROGRAMMER *pgm, updi_sib_info *sib_info) {
  char * str_ptr;

  sib_info->sib_string[SIB_INFO_STRING_LENGTH]=0;
  pmsg_debug("received SIB: [%s]\n", sib_info->sib_string);
  memset(sib_info->family_string, 0, SIB_INFO_FAMILY_LENGTH+1);
  memset(sib_info->nvm_string, 0, SIB_INFO_NVM_LENGTH+1);
  memset(sib_info->debug_string, 0, SIB_INFO_DEBUG_LENGTH+1);
  memset(sib_info->pdi_string, 0, SIB_INFO_PDI_LENGTH+1);
  memset(sib_info->extra_string, 0, SIB_INFO_EXTRA_LENGTH+1);

  memcpy(sib_info->family_string, sib_info->sib_string, SIB_INFO_FAMILY_LENGTH);
  memcpy(sib_info->nvm_string, sib_info->sib_string + 8, SIB_INFO_NVM_LENGTH);
  memcpy(sib_info->debug_string, sib_info->sib_string + 11, SIB_INFO_DEBUG_LENGTH);
  memcpy(sib_info->pdi_string, sib_info->sib_string + 15, SIB_INFO_PDI_LENGTH);
  strcpy(sib_info->extra_string, (char *)sib_info->sib_string + 19);

  str_ptr = strstr(sib_info->nvm_string, ":");
  if (!str_ptr) {
    pmsg_error("incorrect format of NVM string\n");
    return -1;
  }
  sib_info->nvm_version = *(str_ptr+1);

  str_ptr = strstr(sib_info->debug_string, ":");
  if (!str_ptr) {
    pmsg_error("incorrect format of DEBUG string\n");
    return -1;
  }
  sib_info->debug_version = *(str_ptr+1);

  pmsg_debug("Device family ID: %s\n", sib_info->family_string);
  pmsg_debug("NVM interface: %s\n", sib_info->nvm_string);
  pmsg_debug("Debug interface: %s\n", sib_info->debug_string);
  pmsg_debug("PDI oscillator: %s\n", sib_info->pdi_string);
  pmsg_debug("Extra information: %s\n", sib_info->extra_string);
  switch (sib_info->nvm_version) {
    case '0':
      pmsg_notice("NVM type 0: 16-bit, page oriented write\n");
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V0);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_16BIT);
      break;
    case '2':
      pmsg_notice("NVM type 2: 24-bit, word oriented write\n");
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V2);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_24BIT);
      break;
    case '3':
      pmsg_notice("NVM type 3: 24-bit, page oriented\n");
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V3);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_24BIT);
      break;
    case '4':
      pmsg_notice("NVM type 4: 24-bit, word oriented\n");
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V4);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_24BIT);
      break;
    case '5':
      pmsg_notice("NVM type 5: 24-bit, page oriented\n");
      updi_set_nvm_mode(pgm, UPDI_NVM_MODE_V5);
      updi_set_datalink_mode(pgm, UPDI_LINK_MODE_24BIT);
      break;
    default:
      pmsg_warning("unsupported NVM type: %c, please update software\n", sib_info->nvm_version);
      return -1;
  }
  return 0;
}

static void serialupdi_close(PROGRAMMER * pgm)
{
  pmsg_notice("leaving NVM programming mode\n");

  if (serialupdi_leave_progmode(pgm) < 0) {
    pmsg_error("unable to leave NVM programming mode\n");
  }
  if (updi_get_rts_mode(pgm) != RTS_MODE_DEFAULT) {
    pmsg_info("releasing DTR/RTS handshake lines\n");
  }

  updi_link_close(pgm);
}

static int serialupdi_wait_for_unlock(const PROGRAMMER *pgm, unsigned int ms) {
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
  uint8_t status;
  start_time = avr_ustimestamp();
  do {
    if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &status) >= 0) {
      if (!(status & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS))) {
        return 0;
      }
    }
    current_time = avr_ustimestamp();
  } while ((current_time - start_time) < (ms * 1000));

  pmsg_error("timeout waiting for device to unlock\n");
  return -1;
}

typedef enum {
  WAIT_FOR_UROW_LOW,
  WAIT_FOR_UROW_HIGH
} urow_wait_mode;

static int serialupdi_wait_for_urow(const PROGRAMMER *pgm, unsigned int ms, urow_wait_mode mode) {
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
  uint8_t status;
  start_time = avr_ustimestamp();
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
    current_time = avr_ustimestamp();
  } while ((current_time - start_time) < (ms * 1000));

  pmsg_error("timeout waiting for device to complete UROW WRITE\n");
  return -1;
}

static int serialupdi_wait_for_nvmprog(const PROGRAMMER *pgm, unsigned int ms) {

  unsigned long start_time;
  unsigned long current_time;
  uint8_t status;
  start_time = avr_ustimestamp();
  do {
    if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &status) >= 0) {
      if (status & (1 << UPDI_ASI_SYS_STATUS_NVMPROG)) {
        return 0;
      }
    }
    current_time = avr_ustimestamp();
  } while ((current_time - start_time) < (ms * 1000));

  pmsg_error("timeout waiting for device to enter NVMPROG mode\n");
  return -1;
}


static int serialupdi_in_prog_mode(const PROGRAMMER *pgm, uint8_t *in_prog_mode) {
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
    pmsg_error("read CS operation failed\n");
    return rc;
  }

  if (value & (1 << UPDI_ASI_SYS_STATUS_NVMPROG)) {
    *in_prog_mode = 1;
  } else {
    *in_prog_mode = 0;
  }
  return 0;
}

static int serialupdi_enter_progmode(const PROGRAMMER *pgm) {
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
    pmsg_error("checking UPDI NVM prog mode failed\n");
    return -1;
  }
  if (in_prog_mode) {
    pmsg_debug("already in prog mode\n");
    return 0;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    pmsg_error("apply reset operation failed\n");
    return -1;
  }

  memcpy(buffer, UPDI_KEY_NVM, sizeof(buffer));
  if (updi_write_key(pgm, buffer, UPDI_KEY_64, sizeof(buffer)) < 0) {
    pmsg_error("writing NVM KEY failed\n");
    return -1;
  }

  if (updi_read_cs(pgm, UPDI_ASI_KEY_STATUS, &key_status) < 0) {
    pmsg_error("checking KEY status failed\n");
    return -1;
  }
  pmsg_debug("key status: 0x%02X\n", key_status);

  if (!(key_status & (1 << UPDI_ASI_KEY_STATUS_NVMPROG))) {
    pmsg_warning("key was not accepted\n");
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    pmsg_error("apply reset operation failed\n");
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    pmsg_error("release reset operation failed\n");
    return -1;
  }

  if (serialupdi_wait_for_unlock(pgm, 100) < 0) {
    pmsg_error("unable to enter NVM programming mode: device is locked\n");
    return -1;
  }

  if (serialupdi_wait_for_nvmprog(pgm, 500) < 0) {
    pmsg_error("unable to enter NVM programming mode\n");
    return -1;
  }

  pmsg_debug("entered NVM programming mode\n");

  return 0;
}

static int serialupdi_leave_progmode(const PROGRAMMER *pgm) {
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
    pmsg_error("apply reset operation failed\n");
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    pmsg_error("release reset operation failed\n");
    return -1;
  }

  return updi_write_cs(pgm, UPDI_CS_CTRLB, (1 << UPDI_CTRLB_UPDIDIS_BIT) | (1 << UPDI_CTRLB_CCDETDIS_BIT));
}

static int serialupdi_write_userrow(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
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
    pmsg_error("writing USERROW KEY failed\n");
    return -1;
  }

  if (updi_read_cs(pgm, UPDI_ASI_KEY_STATUS, &key_status) < 0) {
    pmsg_error("checking KEY status failed\n");
    return -1;
  }
  pmsg_debug("key status: 0x%02X\n", key_status);

  if (!(key_status & (1 << UPDI_ASI_KEY_STATUS_UROWWRITE))) {
    pmsg_error("key was not accepted\n");
    return -1;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    pmsg_error("apply reset operation failed\n");
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    pmsg_error("release reset operation failed\n");
    return -1;
  }

  if (serialupdi_wait_for_urow(pgm, 500, WAIT_FOR_UROW_HIGH) < 0) {
    pmsg_error("unable to enter USERROW programming mode\n");
    return -1;
  }

  if (n_bytes <= UPDI_MAX_REPEAT_SIZE) {
    if (updi_write_data(pgm, m->offset+addr, m->buf + addr, n_bytes) < 0) {
      pmsg_error("writing USER ROW failed\n");
      return -1;
    }
  } else {
    if (updi_write_data_words(pgm, m->offset+addr, m->buf + addr, n_bytes) < 0) {
      pmsg_error("writing USER ROW failed\n");
      return -1;
    }
  }

  if (updi_write_cs(pgm, UPDI_ASI_SYS_CTRLA, (1 << UPDI_ASI_SYS_CTRLA_UROW_FINAL) |
                                             (1 << UPDI_CTRLB_CCDETDIS_BIT)) < 0) {
    pmsg_error("unable to commit user row write\n");
    return -1;
  }

  if (serialupdi_wait_for_urow(pgm, 500, WAIT_FOR_UROW_LOW) < 0) {
    pmsg_debug("unable to exit USERROW programming mode\n");

    if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
      pmsg_error("apply reset operation failed\n");
      return -1;
    }

    if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
      pmsg_error("release reset operation failed\n");
      return -1;
    }
  }

  if (updi_write_cs(pgm, UPDI_ASI_KEY_STATUS, (1 << UPDI_ASI_KEY_STATUS_UROWWRITE) |
                                              (1 << UPDI_CTRLB_CCDETDIS_BIT)) < 0) {
    pmsg_error("unable to complete user row write\n");
    return -1;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    pmsg_error("apply reset operation failed\n");
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    pmsg_error("release reset operation failed\n");
    return -1;
  }

  serialupdi_reset_connection(pgm);

  serialupdi_enter_progmode(pgm);

  return 0;
}

static int serialupdi_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  uint8_t value;
  uint8_t reset_link_required=0;
  
  if (updi_link_init(pgm) < 0) {
    pmsg_error("UPDI link initialization failed\n");
    return -1;
  }
  pmsg_notice2("UPDI link initialization OK\n");

  if (updi_get_rts_mode(pgm) != RTS_MODE_DEFAULT) {
    pmsg_info("forcing serial DTR/RTS handshake lines %s\n", updi_get_rts_mode(pgm) == RTS_MODE_LOW ? "LOW" : "HIGH");
  }

  if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {

    /* let's try reset the connection */
    if (!serialupdi_reset_connection(pgm)) {
      return -1;
    }

    if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {
      pmsg_error("read CS operation during initialization failed\n");
      return -1;
    }
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS)) {
    pmsg_notice("device is locked\n");
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_UROWPROG)) {
    pmsg_notice("device in USER ROW programming state, leaving programming mode\n");
    reset_link_required = 1;
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_NVMPROG)) {
    pmsg_notice("device in NVM programming state, leaving programming mode\n");
    reset_link_required = 1;
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_INSLEEP)) {
    pmsg_notice("device is in SLEEP mode\n");
  }
  if (value & (1 << UPDI_ASI_SYS_STATUS_RSTSYS)) {
    pmsg_notice("device in reset status, trying to release it\n");
    if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
      return -1;
    }
  }
  if (reset_link_required) {
    if (serialupdi_reset_connection(pgm) < 0) {
      pmsg_error("UPDI link reset failed\n");
      return -1;
    }
  }

  updi_sib_info * sib_info = updi_get_sib_info(pgm);

  if (updi_read_sib(pgm, sib_info->sib_string, 32) < 0) {
    /* this should never happen, let's try to reset connection and try again */
    if (serialupdi_reset_connection(pgm) < 0) {
      pmsg_error("SerialUPDI reset connection failed\n");
      return -1;
    }
    if (updi_read_sib(pgm, sib_info->sib_string, 32) < 0) {
      pmsg_error("read SIB operation failed\n");
      return -1;
    }
  }
  if (serialupdi_decode_sib(pgm, sib_info) < 0) {
    pmsg_error("decode SIB_INFO failed\n");
    return -1;
  }

  if (updi_link_init(pgm) < 0) {
    pmsg_error("UPDI link initialization failed\n");
    return -1;
  }

  pmsg_notice("entering NVM programming mode\n");

  /* try, but ignore failure */
  /* It will always fail if the device is locked */
  /* The device will be unlocked by erasing the chip after this. */
  if (serialupdi_enter_progmode(pgm) == 0) {
    /* If successful, you can run silicon check */
    if (updi_read_data(pgm, p->syscfg_base+1, &value, 1) < 0) {
      pmsg_error("Reading chip silicon revision failed\n");
      return -1;
    } else {
      pmsg_debug("Received chip silicon revision 0x%02x\n", value);
      pmsg_notice("Chip silicon revision: %x.%x\n", value >> 4, value & 0x0f);
    }
  }

  return 0;
}

static void serialupdi_disable(const PROGRAMMER *pgm) {
  /* Do nothing. */

  return;
}

static void serialupdi_enable(PROGRAMMER * pgm, const AVRPART *p) {
  /* Do nothing. */

  return;
}

static void serialupdi_display(const PROGRAMMER *pgm, const char *p) {
  return;
}

static int serialupdi_cmd(const PROGRAMMER *pgm, const unsigned char *cmd,
                          unsigned char * res)
{
  pmsg_error("cmd %s[%s] not implemented yet\n", cmd, res);
  return -1;
}

static int serialupdi_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_error("program enable not implemented yet\n");
  return -1;
}

#define Return(...) do { pmsg_error(__VA_ARGS__); msg_error("\n"); return -1; } while (0)

static int serialupdi_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                                unsigned long addr, unsigned char * value)
{
  pmsg_debug("%s(%s, 0x%04lx)\n", __func__, mem->desc, addr);
  if(mem->size < 1)
    Return("cannot read byte from %s %s owing to its size %d", p->desc, mem->desc, mem->size);

  if(addr >= (unsigned long) mem->size)
    Return("cannot read byte from %s %s as address 0x%04lx outside range [0, 0x%04x]",
      p->desc, mem->desc, addr, mem->size-1);

  if(mem_is_sib(mem)) {
    if(addr >= SIB_INFO_STRING_LENGTH)
      Return("cannot read byte from %s sib as address 0x%04lx outside range [0, 0x%04x]",
        p->desc, addr, SIB_INFO_STRING_LENGTH-1);
    if(!*updi_get_sib_info(pgm)->sib_string) // This should never happen
      Return("cannot read byte from %s sib as memory not initialised", p->desc);
    *value = updi_get_sib_info(pgm)->sib_string[addr];
    return 0;
  }

  return updi_read_byte(pgm, mem->offset + addr, value);
}

static int serialupdi_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                                 unsigned long addr, unsigned char value)
{
  pmsg_debug("%s(%s, 0x%04lx, 0x%02x)\n", __func__, mem->desc, addr, value);
  if(mem->size < 1)
    Return("cannot write byte to %s %s owing to its size %d", p->desc, mem->desc, mem->size);

  if(addr >= (unsigned long) mem->size)
    Return("cannot write byte to %s %s as address 0x%04lx outside range [0, 0x%04x]",
      p->desc, mem->desc, addr, mem->size-1);

  if (mem_is_a_fuse(mem) || mem_is_fuses(mem)) {
    return updi_nvm_write_fuse(pgm, p, mem->offset + addr, value);
  }
  if (mem_is_lock(mem)) {
    return updi_nvm_write_fuse(pgm, p, mem->offset + addr, value);
  }
  if (mem_is_eeprom(mem)) {
    unsigned char buffer[1];
    buffer[0]=value;
    return updi_nvm_write_eeprom(pgm, p, mem->offset + addr, buffer, 1);
  }
  if (mem_is_flash(mem)) {
    unsigned char buffer[1];
    buffer[0]=value;
    return updi_nvm_write_flash(pgm, p, mem->offset + addr, buffer, 1);
  }
  // Read-only memories
  if(mem_is_readonly(mem)) {
    unsigned char is;
    if(serialupdi_read_byte(pgm, p, mem, addr, &is) >= 0 && is == value)
      return 0;

    Return("cannot write to read-only memory %s of %s", mem->desc, p->desc);
  }

  return updi_write_byte(pgm, mem->offset + addr, value);
}


static int serialupdi_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                 unsigned int page_size,
                                 unsigned int addr, unsigned int n_bytes)
{
  if(n_bytes > 65535) {
    pmsg_error("%s() called with implausibly high n_bytes = %u\n", __func__, n_bytes);
    return -1;
  }

  if ((int) n_bytes > m->readsize) {
    unsigned int read_offset = addr;
    int remaining_bytes = n_bytes;
    int read_bytes = 0;
    int rc;

    while (remaining_bytes > 0) {
      rc = updi_read_data(pgm, m->offset + read_offset, m->buf + read_offset, 
                          remaining_bytes > m->readsize ? m->readsize : remaining_bytes);
      if (rc < 0) {
        pmsg_error("paged load operation failed\n");
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

static int serialupdi_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                  unsigned int page_size,
                                  unsigned int addr, unsigned int n_bytes)
{
  int rc;

  if(n_bytes > 65535) {
    pmsg_error("%s() called with implausibly high n_bytes = %u\n", __func__, n_bytes);
    return -1;
  }
  if ((int) n_bytes > m->page_size) {
    unsigned int write_offset = addr;
    int remaining_bytes = n_bytes;
    int write_bytes = 0;

    while (remaining_bytes > 0) {

      if (mem_is_eeprom(m)) {
        rc = updi_nvm_write_eeprom(pgm, p, m->offset + write_offset, m->buf + write_offset, 
                                   remaining_bytes > m->page_size ? m->page_size : remaining_bytes);
      } else if (mem_is_flash(m)) {
        rc = updi_nvm_write_flash(pgm, p, m->offset + write_offset, m->buf + write_offset, 
                                  remaining_bytes > m->page_size ? m->page_size : remaining_bytes);
      } else if (mem_is_userrow(m)) {
        rc = serialupdi_write_userrow(pgm, p, m, page_size, write_offset, 
                                      remaining_bytes > m->page_size ? m->page_size : remaining_bytes);
      } else if (mem_is_fuses(m)) {
        pmsg_debug("page write operation requested for fuses, falling back to byte-level write\n");
        return -1;
      } else {
        pmsg_error("invalid memory <%s:%d>, 0x%06X, %d (0x%04X)\n", m->desc, page_size, addr, n_bytes, n_bytes);
        rc = -1;
      }

      if (rc < 0) {
        pmsg_error("paged write operation failed\n");
        return rc;
      } else {
        write_bytes+=rc;
        write_offset+=m->page_size;
        remaining_bytes-=m->page_size;
      }
    }
    return write_bytes;
  } else {
    if (mem_is_eeprom(m)) {
      rc = updi_nvm_write_eeprom(pgm, p, m->offset+addr, m->buf+addr, n_bytes);
    } else if (mem_is_flash(m)) {
      rc = updi_nvm_write_flash(pgm, p, m->offset+addr, m->buf+addr, n_bytes);
    } else if (mem_is_userrow(m)) {
      rc = serialupdi_write_userrow(pgm, p, m, page_size, addr, n_bytes);
    } else if (mem_is_fuses(m)) {
        pmsg_debug("page write operation requested for fuses, falling back to byte-level write\n");
        rc = -1;
    } else {
      pmsg_error("invalid memory: <%s:%d>, 0x%06X, %d (0x%04X)\n", m->desc, page_size, addr, n_bytes, n_bytes);
      rc = -1;
    }
    return rc;
  }
}

static int serialupdi_unlock(const PROGRAMMER *pgm, const AVRPART *p) {
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
    pmsg_error("writing NVM KEY failed\n");
    return -1;
  }

  if (updi_read_cs(pgm, UPDI_ASI_KEY_STATUS, &key_status) < 0) {
    pmsg_error("checking KEY status failed\n");
    return -1;
  }
  pmsg_debug("key status: 0x%02X\n", key_status);

  if (!(key_status & (1 << UPDI_ASI_KEY_STATUS_CHIPERASE))) {
    pmsg_error("key not accepted\n");
    return -1;
  }

  if (serialupdi_reset(pgm, APPLY_RESET) < 0) {
    pmsg_error("apply reset operation failed\n");
    return -1;
  }

  if (serialupdi_reset(pgm, RELEASE_RESET) < 0) {
    pmsg_error("release reset operation failed\n");
    return -1;
  }

  if (serialupdi_wait_for_unlock(pgm, 500) < 0) {
    pmsg_error("waiting for unlock failed\n");
    return -1;
  }

  if (updi_link_init(pgm) < 0) {
    pmsg_error("UPDI link reinitialization failed\n");
    return -1;
  }

  return serialupdi_enter_progmode(pgm);
}

static int serialupdi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  uint8_t value;

  if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {
    pmsg_error("read CS operation during chip erase failed\n");
    return -1;
  }
  
  if (value & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS)) {
    pmsg_warning("device is locked\n");
    if (ovsigck) {
      pmsg_warning("attempting device erase\n");
      return serialupdi_unlock(pgm, p);
    }
  } else {
    return updi_nvm_chip_erase(pgm, p);
  }
  return -1;
}

static int serialupdi_page_erase(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
                                 unsigned int baseaddr)
{
  return updi_nvm_erase_flash_page(pgm, p, m->offset + baseaddr);
}

static int serialupdi_read_signature(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m) {

  uint8_t value;

  if (updi_read_cs(pgm, UPDI_ASI_SYS_STATUS, &value)<0) {
    pmsg_error("read CS operation during signature read failed\n");
    return -1;
  }

  if (value & (1 << UPDI_ASI_SYS_STATUS_LOCKSTATUS)) {
    m->buf[0]=0x00;
    m->buf[1]=0x00;
    m->buf[2]=0x00;
    return LIBAVRDUDE_SOFTFAIL;
  } else {
    updi_read_byte(pgm, m->offset + 0, m->buf);
    updi_read_byte(pgm, m->offset + 1, m->buf+1);
    updi_read_byte(pgm, m->offset + 2, m->buf+2);
  }

  return 3;
}

static int serialupdi_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib) {

  updi_sib_info * sib_info = updi_get_sib_info(pgm);

  memcpy(sib, sib_info->sib_string, 32);
  
  return 0;
}

static int serialupdi_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  char rts_mode[5];
  int rv = 0;

  for (ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if (sscanf(extended_param, "rtsdtr=%4s", rts_mode) == 1) {
      if (str_caseeq(rts_mode, "low")) {
        updi_set_rts_mode(pgm, RTS_MODE_LOW);
      } else if (str_caseeq(rts_mode, "high")) {
        updi_set_rts_mode(pgm, RTS_MODE_HIGH);
      } else {
        pmsg_error("RTS/DTR mode must be LOW or HIGH\n");
        return -1;
      }
      continue;
    }
    if (str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -xrtsdtr=low,high Force RTS/DTR lines low or high state during programming\n");
      msg_error("  -xhelp            Show this help menu and exit\n");
      exit(0);
    }

    pmsg_error("invalid extended parameter '%s'\n", extended_param);
    rv = -1;
  }

  return rv;
}

void serialupdi_initpgm(PROGRAMMER *pgm) {
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
