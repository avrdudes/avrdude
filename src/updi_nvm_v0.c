/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2024  Dawid Buchwald
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
#include "updi_nvm_v0.h"
#include "updi_state.h"
#include "updi_constants.h"
#include "updi_readwrite.h"

// NVMCTRL v0 REGISTERS
#define UPDI_V0_NVMCTRL_CTRLA                  0x00
#define UPDI_V0_NVMCTRL_CTRLB                  0x01
#define UPDI_V0_NVMCTRL_STATUS                 0x02
#define UPDI_V0_NVMCTRL_INTCTRL                0x03
#define UPDI_V0_NVMCTRL_INTFLAGS               0x04
#define UPDI_V0_NVMCTRL_DATAL                  0x06
#define UPDI_V0_NVMCTRL_DATAH                  0x07
#define UPDI_V0_NVMCTRL_ADDRL                  0x08
#define UPDI_V0_NVMCTRL_ADDRH                  0x09

// NVMCTRL v0 CTRLA
#define UPDI_V0_NVMCTRL_CTRLA_NOP              0x00
#define UPDI_V0_NVMCTRL_CTRLA_WRITE_PAGE       0x01
#define UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE       0x02
#define UPDI_V0_NVMCTRL_CTRLA_ERASE_WRITE_PAGE 0x03
#define UPDI_V0_NVMCTRL_CTRLA_PAGE_BUFFER_CLR  0x04
#define UPDI_V0_NVMCTRL_CTRLA_CHIP_ERASE       0x05
#define UPDI_V0_NVMCTRL_CTRLA_ERASE_EEPROM     0x06
#define UPDI_V0_NVMCTRL_CTRLA_WRITE_FUSE       0x07

// NVMCTRL STATUS
#define UPDI_V0_NVM_STATUS_WRITE_ERROR_BIT 2
#define UPDI_V0_NVM_STATUS_EEPROM_BUSY_BIT 1
#define UPDI_V0_NVM_STATUS_FLASH_BUSY_BIT  0

#define USE_DEFAULT_COMMAND 0xFF

typedef enum 
{
  DONT_USE_WORD_ACCESS,
  USE_WORD_ACCESS
} access_mode;


int updi_nvm_chip_erase_V0(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def chip_erase(self):
        """
        Does a chip erase using the NVM controller

        Note that on locked devices this is not possible
        and the ERASE KEY has to be used instead, see the unlock method
        """
        self.logger.info("Chip erase using NVM CTRL")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready before chip erase")

        # Erase
        self.execute_nvm_command(constants.UPDI_V0_NVMCTRL_CTRLA_CHIP_ERASE)

        # And wait for it
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready after chip erase")

        return True
*/
  pmsg_debug("Chip erase using NVM CTRL\n");
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  if (updi_nvm_command_V0(pgm, p, UPDI_V0_NVMCTRL_CTRLA_CHIP_ERASE) < 0) {
    pmsg_error("UPDI chip erase command failed\n");
    return -1;
  }
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_erase_flash_page_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address) {
/*
    def erase_flash_page(self, address):
        """
        Erasing single flash page using the NVM controller (v0)

        :param address: Start address of page to erase
        :type address: int
        """
        self.logger.info("Erase flash page at address 0x%08X", address)

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready before flash page erase")

        # Dummy write
        self.readwrite.write_data(address, [0xFF])

        # Erase
        self.execute_nvm_command(constants.UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE)

        # And wait for it
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready after flash page erase")
*/
  unsigned char data[1];
  pmsg_debug("erase flash page at address 0x%06X\n", address);
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  data[0] = 0xFF;
  if (updi_write_data(pgm, address, data, 1) < 0) {
    pmsg_error("dummy write operation failed\n");
    return -1;
  }
  if (updi_nvm_command_V0(pgm, p, UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE) < 0) {
    pmsg_error("UPDI flash page erase command failed\n");
    return -1;
  }
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_erase_eeprom_V0(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def erase_eeprom(self):
        """
        Erase EEPROM memory only (v0)
        """
        self.logger.info("Erase EEPROM")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready before EEPROM erase")

        # Erase
        self.execute_nvm_command(constants.UPDI_V0_NVMCTRL_CTRLA_ERASE_EEPROM)

        # And wait for it
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready after EEPROM erase")
*/  
  pmsg_debug("erase EEPROM\n");
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  if (updi_nvm_command_V0(pgm, p, UPDI_V0_NVMCTRL_CTRLA_ERASE_EEPROM) < 0) {
    pmsg_error("UPDI EEPROM erase command failed\n");
    return -1;
  }
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_erase_user_row_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size) {
/*
    def erase_user_row(self, address, size):
        """
        Erase User Row memory only (v0)

        :param address: Start address of user row
        :type address: int
        """
        self.logger.info("Erase user row")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready before user row erase")

        # On this NVM version user row is implemented as EEPROM
        # When erasing single EEPROM pages a dummy write is needed for each location to be erased
        for offset in range(size):
            self.readwrite.write_data(address+offset, [0xFF])

        # Erase
        self.execute_nvm_command(constants.UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE)

        # And wait for it
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready after user row erase")
*/
  uint16_t offset;
  unsigned char data[1];
  pmsg_debug("erase user row\n");
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  data[0]=0xFF;
  for (offset = 0; offset<size; offset++)
  {
    if (updi_write_data(pgm, address+offset, data, 1) < 0) {
      pmsg_error("write data operation failed at offset 0x%04x\n", offset);
      return -1;
    }
  }
  if (updi_nvm_command_V0(pgm, p, UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE) < 0) {
    pmsg_error("erase page operation failed\n");
    return -1;
  }
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  return 0;
}

static int nvm_write_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode, uint8_t nvm_command);

int updi_nvm_write_flash_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_flash(self, address, data):
        """
        Writes data to flash (v0)

        :param address: address to write to
        :param data: data to write
        """
        return self.write_nvm(address, data, use_word_access=True)
*/
  return nvm_write_V0(pgm, p, address, buffer, size, USE_WORD_ACCESS, USE_DEFAULT_COMMAND);
}

int updi_nvm_write_user_row_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_user_row(self, address, data):
        """
        Writes data to user row (v0)

        :param address: address to write to
        :param data: data to write
        """
        # On this NVM variant user row is implemented as EEPROM
        return self.write_eeprom(address, data)
*/
  return updi_nvm_write_eeprom_V0(pgm, p, address, buffer, size);
}

int updi_nvm_write_eeprom_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_eeprom(self, address, data):
        """
        Write data to EEPROM (v0)

        :param address: address to write to
        :param data: data to write
        """
        return self.write_nvm(address, data, use_word_access=False,
                              nvmcommand=constants.UPDI_V0_NVMCTRL_CTRLA_ERASE_WRITE_PAGE)
*/
  return nvm_write_V0(pgm, p, address, buffer, size, DONT_USE_WORD_ACCESS, UPDI_V0_NVMCTRL_CTRLA_ERASE_WRITE_PAGE);
}

int updi_nvm_write_fuse_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value) {
/*
    def write_fuse(self, address, data):
        """
        Writes one fuse value (v0)

        :param address: address to write to
        :param data: data to write
        """

        # Check that NVM controller is ready
        if not self.wait_nvm_ready():
            raise PymcuprogError("Timeout waiting for NVM controller to be ready before fuse write")

        # Write address to NVMCTRL ADDR
        self.logger.debug("Load NVM address")
        self.readwrite.write_byte(self.device.nvmctrl_address + constants.UPDI_NVMCTRL_ADDRL, address & 0xFF)
        self.readwrite.write_byte(self.device.nvmctrl_address + constants.UPDI_NVMCTRL_ADDRH, (address >> 8) & 0xFF)

        # Write data
        self.logger.debug("Load fuse data")
        self.readwrite.write_byte(self.device.nvmctrl_address + constants.UPDI_NVMCTRL_DATAL, data[0] & 0xFF)

        # Execute
        self.logger.debug("Execute fuse write")
        self.execute_nvm_command(constants.UPDI_V0_NVMCTRL_CTRLA_WRITE_FUSE)

        if not self.wait_nvm_ready():
            raise PymcuprogError("Timeout waiting for NVM controller to be ready after fuse write")
*/
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  pmsg_debug("load NVM address\n");
  if (updi_write_byte(pgm, p->nvm_base + UPDI_V0_NVMCTRL_ADDRL, address & 0xFF) < 0) {
    pmsg_error("UPDI write ADDRL operation failed\n");
    return -1;
  }
  if (updi_write_byte(pgm, p->nvm_base + UPDI_V0_NVMCTRL_ADDRH, (address >> 8) & 0xFF) < 0) {
    pmsg_error("write ADDRH operation failed\n");
    return -1;
  }
  pmsg_debug("load fuse data\n");
  if (updi_write_byte(pgm, p->nvm_base + UPDI_V0_NVMCTRL_DATAL, value & 0xFF) < 0) {
    pmsg_error("write DATAL operation failed\n");
    return -1;
  }
  pmsg_debug("execute fuse write\n");
  if (updi_nvm_command_V0(pgm, p, UPDI_V0_NVMCTRL_CTRLA_WRITE_FUSE) < 0) {
    pmsg_error("write fuse operation failed\n");
    return -1;
  }
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  return 0;
}

static int nvm_write_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode, uint8_t nvm_command)
{
/*
    def write_nvm(self, address, data, use_word_access, nvmcommand=constants.UPDI_V0_NVMCTRL_CTRLA_WRITE_PAGE):
        """
        Writes a page of data to NVM (v0)

        By default the PAGE_WRITE command is used, which
        requires that the page is already erased.
        By default word access is used (flash)

        :param address: address to write to
        :param data: data to write
        :param use_word_access: write whole words?
        :param nvmcommand: command to use for commit
        """

        # Check that NVM controller is ready
        if not self.wait_nvm_ready():
            raise PymcuprogError("Timeout waiting for NVM controller to be ready before page buffer clear")

        # Clear the page buffer
        self.logger.debug("Clear page buffer")
        self.execute_nvm_command(constants.UPDI_V0_NVMCTRL_CTRLA_PAGE_BUFFER_CLR)

        # Wait for NVM controller to be ready
        if not self.wait_nvm_ready():
            raise PymcuprogError("Timeout waiting for NVM controller to be ready after page buffer clear")

        # Load the page buffer by writing directly to location
        if use_word_access:
            self.readwrite.write_data_words(address, data)
        else:
            self.readwrite.write_data(address, data)

        # Write the page to NVM, maybe erase first
        self.logger.debug("Committing data")
        self.execute_nvm_command(nvmcommand)

        # Wait for NVM controller to be ready again
        if not self.wait_nvm_ready():
            raise PymcuprogError("Timeout waiting for NVM controller to be ready after page write")
*/
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  pmsg_debug("clear page buffer\n");
  if (updi_nvm_command_V0(pgm, p, UPDI_V0_NVMCTRL_CTRLA_PAGE_BUFFER_CLR) < 0) {
    pmsg_error("clear page operation failed\n");
    return -1;
  }
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  if (mode == USE_WORD_ACCESS) {
    if (updi_write_data_words(pgm, address, buffer, size) < 0) {
      pmsg_error("write data words operation failed\n");
      return -1;
    }
  } else {
    if (updi_write_data(pgm, address, buffer, size) < 0) {
      pmsg_error("write data operation failed\n");
      return -1;
    }
  }
  pmsg_debug("committing data\n");
  if (nvm_command == USE_DEFAULT_COMMAND) {
    nvm_command = UPDI_V0_NVMCTRL_CTRLA_WRITE_PAGE;
  }
  if (updi_nvm_command_V0(pgm, p, nvm_command) < 0) {
      pmsg_error("commit data command failed\n");
      return -1;
  }
  if (updi_nvm_wait_ready_V0(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V0() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_wait_ready_V0(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def wait_nvm_ready(self):
        """
        Waits for the NVM controller to be ready
        """
        timeout = Timeout(10000)  # 10 sec timeout, just to be sure

        self.logger.debug("Wait NVM ready")
        while not timeout.expired():
            status = self.readwrite.read_byte(self.device.nvmctrl_address + constants.UPDI_NVMCTRL_STATUS)
            if status & (1 << constants.UPDI_V0_NVM_STATUS_WRITE_ERROR):
                self.logger.error("NVM error")
                return False

            if not status & ((1 << constants.UPDI_NVM_STATUS_EEPROM_BUSY) |
                             (1 << constants.UPDI_NVM_STATUS_FLASH_BUSY)):
                return True

        self.logger.error("Wait NVM ready timed out")
        return False
*/
  unsigned long start_time;
  unsigned long current_time;
  uint8_t status;
  start_time = avr_ustimestamp();
  do {
    if (updi_read_byte(pgm, p->nvm_base + UPDI_V0_NVMCTRL_STATUS, &status) >= 0) {
      if (status & (1 << UPDI_V0_NVM_STATUS_WRITE_ERROR_BIT)) {
        pmsg_error("unable to write NVM status\n");
        return -1;
      }
      if (!(status & ((1 << UPDI_V0_NVM_STATUS_EEPROM_BUSY_BIT) | 
                      (1 << UPDI_V0_NVM_STATUS_FLASH_BUSY_BIT)))) {
        return 0;
      }
    }
    current_time = avr_ustimestamp();
  } while ((current_time - start_time) < 10000000);

  pmsg_error("wait NVM ready timed out\n");
  return -1;
}

int updi_nvm_command_V0(const PROGRAMMER *pgm, const AVRPART *p, uint8_t command) {
/*
    def execute_nvm_command(self, command):
        """
        Executes an NVM COMMAND on the NVM CTRL

        :param command: command to execute
        """
        self.logger.debug("NVMCMD %d executing", command)
        return self.readwrite.write_byte(self.device.nvmctrl_address + constants.UPDI_NVMCTRL_CTRLA, command)
*/
  pmsg_debug("NVMCMD %d executing\n", command);

  return updi_write_byte(pgm, p->nvm_base + UPDI_V0_NVMCTRL_CTRLA, command);
}
