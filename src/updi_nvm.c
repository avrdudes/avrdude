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
#include "updi_state.h"
#include "updi_constants.h"
#include "updi_readwrite.h"

typedef enum 
{
  DONT_USE_WORD_ACCESS,
  USE_WORD_ACCESS
} access_mode;

#define USE_DEFAULT_COMMAND 0xFF

static int nvm_chip_erase_V0(const PROGRAMMER *pgm, const AVRPART *p) {
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
  avrdude_message(MSG_DEBUG, "%s: Chip erase using NVM CTRL\n", progname);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V0_NVMCTRL_CTRLA_CHIP_ERASE) < 0) {
    avrdude_message(MSG_INFO, "%s: Chip erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_flash_page_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address) {
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
  avrdude_message(MSG_DEBUG, "%s: Erase flash page at address 0x%06X\n", progname, address);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  data[0] = 0xFF;
  if (updi_write_data(pgm, address, data, 1) < 0) {
    avrdude_message(MSG_INFO, "%s: Dummy write operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE) < 0) {
    avrdude_message(MSG_INFO, "%s: Flash page erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_eeprom_V0(const PROGRAMMER *pgm, const AVRPART *p) {
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
  avrdude_message(MSG_DEBUG, "%s: Erase EEPROM\n", progname);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V0_NVMCTRL_CTRLA_ERASE_EEPROM) < 0) {
    avrdude_message(MSG_INFO, "%s: EEPROM erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_user_row_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size) {
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
  avrdude_message(MSG_DEBUG, "%s: Erase user row\n", progname);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  data[0]=0xFF;
  for (offset = 0; offset<size; offset++)
  {
    if (updi_write_data(pgm, address+offset, data, 1) < 0) {
      avrdude_message(MSG_INFO, "%s: Write data operation failed at offset 0x%04x\n", progname, offset);
      return -1;
    }
  }
  if (updi_nvm_command(pgm, p, UPDI_V0_NVMCTRL_CTRLA_ERASE_PAGE) < 0) {
    avrdude_message(MSG_INFO, "%s: Erase page operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_write_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode, uint8_t nvm_command);

static int nvm_write_eeprom_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size);


static int nvm_write_flash_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
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

static int nvm_write_user_row_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
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
  return nvm_write_eeprom_V0(pgm, p, address, buffer, size);
}

static int nvm_write_eeprom_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
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

static int nvm_write_fuse_V0(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value) {
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
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Load NVM address\n", progname);
  if (updi_write_byte(pgm, p->nvm_base + UPDI_NVMCTRL_ADDRL, address & 0xFF) < 0) {
    avrdude_message(MSG_INFO, "%s: Write ADDRL operation failed\n", progname);
    return -1;
  }
  if (updi_write_byte(pgm, p->nvm_base + UPDI_NVMCTRL_ADDRH, (address >> 8) & 0xFF) < 0) {
    avrdude_message(MSG_INFO, "%s: Write ADDRH operation failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Load fuse data\n", progname);
  if (updi_write_byte(pgm, p->nvm_base + UPDI_NVMCTRL_DATAL, value & 0xFF) < 0) {
    avrdude_message(MSG_INFO, "%s: Write DATAL operation failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Execute fuse write\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V0_NVMCTRL_CTRLA_WRITE_FUSE) < 0) {
    avrdude_message(MSG_INFO, "%s: Write fuse operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
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
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Clear page buffer\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V0_NVMCTRL_CTRLA_PAGE_BUFFER_CLR) < 0) {
    avrdude_message(MSG_INFO, "%s: Clear page operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (mode == USE_WORD_ACCESS) {
    if (updi_write_data_words(pgm, address, buffer, size) < 0) {
      avrdude_message(MSG_INFO, "%s: Write data words operation failed\n", progname);
      return -1;
    }
  } else {
    if (updi_write_data(pgm, address, buffer, size) < 0) {
      avrdude_message(MSG_INFO, "%s: Write data operation failed\n", progname);
      return -1;
    }
  }
  avrdude_message(MSG_DEBUG, "%s: Committing data\n", progname);
  if (nvm_command == USE_DEFAULT_COMMAND) {
    nvm_command = UPDI_V0_NVMCTRL_CTRLA_WRITE_PAGE;
  }
  if (updi_nvm_command(pgm, p, nvm_command) < 0) {
      avrdude_message(MSG_INFO, "%s: Commit data command failed\n", progname);
      return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_chip_erase_V2(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def chip_erase(self):
        """
        Does a chip erase using the NVM controller
        Note that on locked devices this it not possible
        and the ERASE KEY has to be used instead
        """
        self.logger.info("Chip erase using NVM CTRL")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise Exception("Timeout waiting for NVM controller to be ready before chip erase")

        # Erase
        self.execute_nvm_command(constants.UPDI_V2_NVMCTRL_CTRLA_CHIP_ERASE)

        # And wait for it
        if not self.wait_nvm_ready():
            raise Exception("Timeout waiting for NVM controller to be ready after chip erase")

        return True
*/
  avrdude_message(MSG_DEBUG, "%s: Chip erase using NVM CTRL\n", progname);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_CHIP_ERASE) < 0) {
    avrdude_message(MSG_INFO, "%s: Chip erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_flash_page_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address) {
/*
    def erase_flash_page(self, address):
        """
        Erasing single flash page using the NVM controller (v1)

        :param address: Start address of page to erase
        :type address: int
        """
        self.logger.info("Erase flash page at address 0x%08X", address)

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready before flash page erase")

        # Erase command
        self.execute_nvm_command(constants.UPDI_V2_NVMCTRL_CTRLA_FLASH_PAGE_ERASE)

        # Dummy write
        self.readwrite.write_data(address, [0xFF])

        # And wait for it
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready after flash page erase")

        # Remove command from NVM controller
        self.logger.debug("Clear NVM command")
        self.execute_nvm_command(constants.UPDI_V2_NVMCTRL_CTRLA_NOCMD)
*/
  unsigned char data[1];
  avrdude_message(MSG_DEBUG, "%s: Erase flash page at address 0x%06X\n", progname, address);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  data[0] = 0xFF;
  if (updi_write_data(pgm, address, data, 1) < 0) {
    avrdude_message(MSG_INFO, "%s: Dummy write operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_FLASH_PAGE_ERASE) < 0) {
    avrdude_message(MSG_INFO, "%s: Flash page erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_eeprom_V2(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def erase_eeprom(self):
        """
        Erase EEPROM memory only (v1)
        """
        self.logger.info("Erase EEPROM")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready before EEPROM erase")

        # Erase
        self.execute_nvm_command(constants.UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE)

        # And wait for it
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready after EEPROM erase")

        # Remove command from NVM controller
        self.logger.debug("Clear NVM command")
        self.execute_nvm_command(constants.UPDI_V2_NVMCTRL_CTRLA_NOCMD)
*/
  avrdude_message(MSG_DEBUG, "%s: Erase EEPROM\n", progname);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE) < 0) {
    avrdude_message(MSG_INFO, "%s: EEPROM erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Clear NVM command\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    avrdude_message(MSG_INFO, "%s: Sending empty command failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_user_row_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size) {
/*
    def erase_user_row(self, address, size):
        """
        Erase User Row memory only (v1)

        :param address: Start address of user row
        :type address: int
        """
        # size is not used for this NVM version
        _dummy = size
        # On this NVM version user row is implemented as flash
        return self.erase_flash_page(address)
*/
  return nvm_erase_flash_page_V2(pgm, p, address);
}

static int nvm_write_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode);

static int nvm_write_flash_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_flash(self, address, data):
        """
        Writes data to flash (v1)

        :param address: address to write to
        :param data: data to write
        """
        return self.write_nvm(address, data, use_word_access=True)
*/
  return nvm_write_V2(pgm, p, address, buffer, size, USE_WORD_ACCESS);
}

static int nvm_write_user_row_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_user_row(self, address, data):
        """
        Writes data to user row (v1)

        :param address: address to write to
        :param data: data to write
        """
        # On this NVM variant user row is implemented as Flash
        return self.write_nvm(address, data, use_word_access=False)
*/
  return nvm_write_V2(pgm, p, address, buffer, size, DONT_USE_WORD_ACCESS);
}

static int nvm_write_eeprom_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_eeprom(self, address, data):
        """
        Writes data to NVM (EEPROM)

        :param address: address to write to
        :param data: data to write
        """
        nvm_command = constants.UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE_WRITE

        # Check that NVM controller is ready
        if not self.wait_nvm_ready():
            raise Exception("Timeout waiting for NVM ready before command write")

        # Write the command to the NVM controller
        self.logger.info("NVM EEPROM erase/write command")
        self.execute_nvm_command(nvm_command)

        # Write the data
        self.readwrite.write_data(address, data)

        # Wait for NVM controller to be ready again
        if not self.wait_nvm_ready():
            raise Exception("Timeout waiting for NVM ready after data write")

        # Remove command from NVM controller
        self.logger.info("Clear NVM command")
        self.execute_nvm_command(constants.UPDI_V2_NVMCTRL_CTRLA_NOCMD)
*/
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: NVM EEPROM erase/write command\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE_WRITE) < 0) {
    avrdude_message(MSG_INFO, "%s: EEPROM erase command failed\n", progname);
    return -1;
  }
  if (updi_write_data(pgm, address, buffer, size) < 0) {
    avrdude_message(MSG_INFO, "%s: Write data operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Clear NVM command\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    avrdude_message(MSG_INFO, "%s: Clear NVM command failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_write_fuse_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value) {
/*
    def write_fuse(self, address, data):
        """
        Writes one fuse value
        V1 fuses are EEPROM-based

        :param address: address to write to
        :param data: data to write
        """
        return self.write_eeprom(address, data)
*/
  unsigned char buffer[1];
  buffer[0]=value;
  return nvm_write_eeprom_V2(pgm, p, address, buffer, 1);
}

static int nvm_write_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode)
{
/*
    def write_nvm(self, address, data, use_word_access):
        """
        Writes data to NVM (version 1)
        This version of the NVM block has no page buffer, so words are written directly.

        :param address: address to write to
        :param data: data to write
        :param use_word_access: write in whole words?
        """
        nvm_command = constants.UPDI_V2_NVMCTRL_CTRLA_FLASH_WRITE

        # Check that NVM controller is ready
        if not self.wait_nvm_ready():
            raise Exception("Timeout waiting for NVM controller to be ready before page buffer clear")

        # Write the command to the NVM controller
        self.logger.info("NVM write command")
        self.execute_nvm_command(nvm_command)

        # Write the data
        if use_word_access:
            self.readwrite.write_data_words(address, data)
        else:
            self.readwrite.write_data(address, data)

        # Wait for NVM controller to be ready again
        if not self.wait_nvm_ready():
            raise Exception("Timeout waiting for NVM controller to be ready after data write")

        # Remove command from NVM controller
        self.logger.info("Clear NVM command")
        self.execute_nvm_command(constants.UPDI_V2_NVMCTRL_CTRLA_NOCMD)
*/
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: NVM write command\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_FLASH_WRITE) < 0) {
    avrdude_message(MSG_INFO, "%s: Clear page operation failed\n", progname);
    return -1;
  }
  if (mode == USE_WORD_ACCESS) {
    if (updi_write_data_words(pgm, address, buffer, size) < 0) {
      avrdude_message(MSG_INFO, "%s: Write data words operation failed\n", progname);
      return -1;
    }
  } else {
    if (updi_write_data(pgm, address, buffer, size) < 0) {
      avrdude_message(MSG_INFO, "%s: Write data operation failed\n", progname);
      return -1;
    }
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Clear NVM command\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    avrdude_message(MSG_INFO, "%s: Clear NVM command failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_chip_erase_V3(const PROGRAMMER *pgm, const AVRPART *p) {
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
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_CHIP_ERASE)

        # And wait for it
        status = self.wait_nvm_ready()

        # Remove command
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_NOCMD)

        if not status:
            raise IOError("Timeout waiting for NVM controller to be ready after chip erase")

        return True
*/
  avrdude_message(MSG_DEBUG, "%s: Chip erase using NVM CTRL\n", progname);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V3_NVMCTRL_CTRLA_CHIP_ERASE) < 0) {
    avrdude_message(MSG_INFO, "%s: Chip erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V3_NVMCTRL_CTRLA_NOCMD) < 0) {
    avrdude_message(MSG_INFO, "%s: Sending empty command failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_flash_page_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address) {
/*
    def erase_flash_page(self, address):
        """
        Erasing single flash page using the NVM controller (v3)

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
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_ERASE)

        # And wait for it
        status = self.wait_nvm_ready()

        # Remove command
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_NOCMD)

        if not status:
            raise IOError("Timeout waiting for NVM controller to be ready after flash page erase")
*/  
  unsigned char data[1];
  avrdude_message(MSG_DEBUG, "%s: Erase flash page at address 0x%06X\n", progname, address);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  data[0] = 0xFF;
  if (updi_write_data(pgm, address, data, 1) < 0) {
    avrdude_message(MSG_INFO, "%s: Dummy write operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_ERASE) < 0) {
    avrdude_message(MSG_INFO, "%s: Flash page erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_eeprom_V3(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def erase_eeprom(self):
        """
        Erase EEPROM memory only
        """
        self.logger.info("Erase EEPROM")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise IOError("Timeout waiting for NVM controller to be ready before EEPROM erase")

        # Erase
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_EEPROM_ERASE)

        # And wait for it
        status = self.wait_nvm_ready()

        # Remove command
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_NOCMD)

        if not status:
            raise IOError("Timeout waiting for NVM controller to be ready after EEPROM erase")
*/
  avrdude_message(MSG_DEBUG, "%s: Erase EEPROM\n", progname);
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V3_NVMCTRL_CTRLA_EEPROM_ERASE) < 0) {
    avrdude_message(MSG_INFO, "%s: EEPROM erase command failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V3_NVMCTRL_CTRLA_NOCMD) < 0) {
    avrdude_message(MSG_INFO, "%s: Sending empty command failed\n", progname);
    return -1;
  }
  return 0;
}

static int nvm_erase_user_row_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size) {
/*
    def erase_user_row(self, address, size):
        """
        Erase User Row memory only

        :param address: Start address of user row
        :type address: int
        """
        self.logger.info("Erase user row")

        # On this NVM version user row is implemented as FLASH
        return self.erase_flash_page(self, address)
*/
  avrdude_message(MSG_DEBUG, "%s: Erase user row at address 0x%06X\n", progname, address);
  
  return nvm_erase_flash_page_V3(pgm, p, address);
}

static int nvm_write_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode, uint8_t nvm_command);

static int nvm_write_flash_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_flash(self, address, data):
        """
        Writes data to flash (v3)

        :param address: address to write to
        :param data: data to write
        """
        return self.write_nvm(address, data, use_word_access=True)
*/
  return nvm_write_V3(pgm, p, address, buffer, size, USE_WORD_ACCESS, USE_DEFAULT_COMMAND);
}

static int nvm_write_user_row_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_user_row(self, address, data):
        """
        Writes data to user row (v3)

        :param address: address to write to
        :param data: data to write
        """
        # On this NVM variant user row is implemented as FLASH
        return self.write_nvm(address, data, use_word_access=True)
*/
  return nvm_write_V3(pgm, p, address, buffer, size, USE_WORD_ACCESS, USE_DEFAULT_COMMAND);
}

static int nvm_write_eeprom_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_eeprom(self, address, data):
        """
        Write data to EEPROM (v3)

        :param address: address to write to
        :param data: data to write
        """
        return self.write_nvm(address, data, use_word_access=False,
                              nvmcommand=constants.UPDI_V3_NVMCTRL_CTRLA_EEPROM_PAGE_ERASE_WRITE)
*/
  return nvm_write_V3(pgm, p, address, buffer, size, DONT_USE_WORD_ACCESS, UPDI_V3_NVMCTRL_CTRLA_EEPROM_PAGE_ERASE_WRITE);
}

static int nvm_write_fuse_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value) {
/*
    def write_fuse(self, address, data):
        """
        Writes one fuse value (v3)

        :param address: address to write to
        :param data: data to write
        """
        return self.write_eeprom(address, data)
*/
  unsigned char buffer[1];
  buffer[0] = value;
  return nvm_write_eeprom_V3(pgm, p, address, buffer, 1);
}

static int nvm_write_V3(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode, uint8_t nvm_command)
{
/*
    def write_nvm(self, address, data, use_word_access, nvmcommand=constants.UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_WRITE):
        """
        Writes a page of data to NVM (v3)

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
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_BUFFER_CLEAR)

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

        # Remove command
        self.execute_nvm_command(constants.UPDI_V3_NVMCTRL_CTRLA_NOCMD)
*/
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  avrdude_message(MSG_DEBUG, "%s: Clear page buffer\n", progname);
  if (updi_nvm_command(pgm, p, UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_BUFFER_CLEAR) < 0) {
    avrdude_message(MSG_INFO, "%s: Clear page operation failed\n", progname);
    return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (mode == USE_WORD_ACCESS) {
    if (updi_write_data_words(pgm, address, buffer, size) < 0) {
      avrdude_message(MSG_INFO, "%s: Write data words operation failed\n", progname);
      return -1;
    }
  } else {
    if (updi_write_data(pgm, address, buffer, size) < 0) {
      avrdude_message(MSG_INFO, "%s: Write data operation failed\n", progname);
      return -1;
    }
  }
  avrdude_message(MSG_DEBUG, "%s: Committing data\n", progname);
  if (nvm_command == USE_DEFAULT_COMMAND) {
    nvm_command = UPDI_V3_NVMCTRL_CTRLA_FLASH_PAGE_WRITE;
  }
  if (updi_nvm_command(pgm, p, nvm_command) < 0) {
      avrdude_message(MSG_INFO, "%s: Commit data command failed\n", progname);
      return -1;
  }
  if (updi_nvm_wait_ready(pgm, p) < 0) {
    avrdude_message(MSG_INFO, "%s: Wait for ready chip failed\n", progname);
    return -1;
  }
  if (updi_nvm_command(pgm, p, UPDI_V3_NVMCTRL_CTRLA_NOCMD) < 0) {
    avrdude_message(MSG_INFO, "%s: Sending empty command failed\n", progname);
    return -1;
  }
  return 0;
}


int updi_nvm_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_chip_erase_V0(pgm, p);
    case UPDI_NVM_MODE_V2:
      return nvm_chip_erase_V2(pgm, p);
    case UPDI_NVM_MODE_V3:
      return nvm_chip_erase_V3(pgm, p);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_erase_flash_page(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_erase_flash_page_V0(pgm, p, address);
    case UPDI_NVM_MODE_V2:
      return nvm_erase_flash_page_V2(pgm, p, address);
    case UPDI_NVM_MODE_V3:
      return nvm_erase_flash_page_V3(pgm, p, address);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_erase_eeprom(const PROGRAMMER *pgm, const AVRPART *p) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_erase_eeprom_V0(pgm, p);
    case UPDI_NVM_MODE_V2:
      return nvm_erase_eeprom_V2(pgm, p);
    case UPDI_NVM_MODE_V3:
      return nvm_erase_eeprom_V3(pgm, p);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_erase_user_row(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_erase_user_row_V0(pgm, p, address, size);
    case UPDI_NVM_MODE_V2:
      return nvm_erase_user_row_V2(pgm, p, address, size);
    case UPDI_NVM_MODE_V3:
      return nvm_erase_user_row_V3(pgm, p, address, size);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_flash(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_write_flash_V0(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V2:
      return nvm_write_flash_V2(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V3:
      return nvm_write_flash_V3(pgm, p, address, buffer, size);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_user_row(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_write_user_row_V0(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V2:
      return nvm_write_user_row_V2(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V3:
      return nvm_write_user_row_V3(pgm, p, address, buffer, size);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_eeprom(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_write_eeprom_V0(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V2:
      return nvm_write_eeprom_V2(pgm, p, address, buffer, size);
    case UPDI_NVM_MODE_V3:
      return nvm_write_eeprom_V3(pgm, p, address, buffer, size);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value) {
  switch(updi_get_nvm_mode(pgm))
  {
    case UPDI_NVM_MODE_V0:
      return nvm_write_fuse_V0(pgm, p, address, value);
    case UPDI_NVM_MODE_V2:
      return nvm_write_fuse_V2(pgm, p, address, value);
    case UPDI_NVM_MODE_V3:
      return nvm_write_fuse_V3(pgm, p, address, value);
    default:
      avrdude_message(MSG_INFO, "%s: Invalid NVM Mode %d\n", progname, updi_get_nvm_mode(pgm));
      return -1;
  }
}

int updi_nvm_wait_ready(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def wait_nvm_ready(self):
        """
        Waits for the NVM controller to be ready
        """
        timeout = Timeout(10000)  # 10 sec timeout, just to be sure

        self.logger.debug("Wait NVM ready")
        while not timeout.expired():
            status = self.readwrite.read_byte(self.device.nvmctrl_address + constants.UPDI_NVMCTRL_STATUS)
            if status & (1 << constants.UPDI_NVM_STATUS_WRITE_ERROR):
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
  struct timeval tv;
  uint8_t status;
  gettimeofday (&tv, NULL);
  start_time = (tv.tv_sec * 1000000) + tv.tv_usec;
  do {
    if (updi_read_byte(pgm, p->nvm_base + UPDI_NVMCTRL_STATUS, &status) >= 0) {
      if (status & (1 << UPDI_NVM_STATUS_WRITE_ERROR)) {
        avrdude_message(MSG_INFO, "%s: NVM error\n", progname);
        return -1;
      }
      if (!(status & ((1 << UPDI_NVM_STATUS_EEPROM_BUSY) | 
                      (1 << UPDI_NVM_STATUS_FLASH_BUSY)))) {
        return 0;
      }
    }
    gettimeofday (&tv, NULL);
    current_time = (tv.tv_sec * 1000000) + tv.tv_usec;
  } while ((current_time - start_time) < 10000000);

  avrdude_message(MSG_INFO, "%s: Wait NVM ready timed out\n", progname);
  return -1;
}

int updi_nvm_command(const PROGRAMMER *pgm, const AVRPART *p, uint8_t command) {
/*
    def execute_nvm_command(self, command):
        """
        Executes an NVM COMMAND on the NVM CTRL

        :param command: command to execute
        """
        self.logger.debug("NVMCMD %d executing", command)
        return self.readwrite.write_byte(self.device.nvmctrl_address + constants.UPDI_NVMCTRL_CTRLA, command)
*/
  avrdude_message(MSG_DEBUG, "%s: NVMCMD %d executing\n", progname, command);

  return updi_write_byte(pgm, p->nvm_base + UPDI_NVMCTRL_CTRLA, command);
}
