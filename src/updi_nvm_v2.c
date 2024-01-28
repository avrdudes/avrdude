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
#include "updi_nvm_v2.h"
#include "updi_state.h"
#include "updi_constants.h"
#include "updi_readwrite.h"

// NVMCTRL v2 REGISTERS
#define UPDI_V2_NVMCTRL_CTRLA                  0x00
#define UPDI_V2_NVMCTRL_CTRLB                  0x01
#define UPDI_V2_NVMCTRL_STATUS                 0x02
#define UPDI_V2_NVMCTRL_INTCTRL                0x03
#define UPDI_V2_NVMCTRL_INTFLAGS               0x04
#define UPDI_V2_NVMCTRL_DATAL                  0x06
#define UPDI_V2_NVMCTRL_DATAH                  0x07
#define UPDI_V2_NVMCTRL_ADDR0                  0x08
#define UPDI_V2_NVMCTRL_ADDR1                  0x09
#define UPDI_V2_NVMCTRL_ADDR2                  0x0a
#define UPDI_V2_NVMCTRL_ADDR3                  0x0b

// NVMCTRL v2 CTRLA
#define UPDI_V2_NVMCTRL_CTRLA_NOCMD              0x00
#define UPDI_V2_NVMCTRL_CTRLA_NOOP               0x01
#define UPDI_V2_NVMCTRL_CTRLA_FLASH_WRITE        0x02
#define UPDI_V2_NVMCTRL_CTRLA_FLASH_PAGE_ERASE   0x08
#define UPDI_V2_NVMCTRL_CTRLA_EEPROM_WRITE       0x12
#define UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE_WRITE 0x13
#define UPDI_V2_NVMCTRL_CTRLA_EEPROM_BYTE_ERASE  0x18
#define UPDI_V2_NVMCTRL_CTRLA_CHIP_ERASE         0x20
#define UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE       0x30

// NVMCTRL STATUS
#define UPDI_V2_NVM_STATUS_WRITE_ERROR_MASK 0x30
#define UPDI_V2_NVM_STATUS_WRITE_ERROR_BIT     4
#define UPDI_V2_NVM_STATUS_EEPROM_BUSY_BIT     1
#define UPDI_V2_NVM_STATUS_FLASH_BUSY_BIT      0

#define USE_DEFAULT_COMMAND 0xFF

typedef enum 
{
  DONT_USE_WORD_ACCESS,
  USE_WORD_ACCESS
} access_mode;

int updi_nvm_chip_erase_V2(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def chip_erase(self):
        """
        Does a chip erase using the NVM controller

        Note that on locked devices this it not possible and the ERASE KEY has to be used instead
        """
        self.logger.debug("Chip erase using NVM CTRL")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready before chip erase")

        # Erase
        self.execute_nvm_command(self.NVMCMD_CHIP_ERASE)

        # And wait for it
        status = self.wait_nvm_ready()

        # Remove command from NVM controller
        self.logger.debug("Clear NVM command")
        self.execute_nvm_command(self.NVMCMD_NOCMD)
        if not status:
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready after chip erase")
*/
  int status;
  pmsg_debug("chip erase using NVM CTRL\n");
  if (updi_nvm_wait_ready_V2(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_CHIP_ERASE) < 0) {
    pmsg_error("chip erase command failed\n");
    return -1;
  }
  status = updi_nvm_wait_ready_V2(pgm, p); 
  pmsg_debug("clear NVM command\n");
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    pmsg_error("command buffer erase failed\n");
    return -1;
  }
  if (status < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_erase_flash_page_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address) {
/*
    def erase_flash_page(self, address):
        """
        Erasing single flash page using the NVM controller

        :param address: Start address of page to erase
        :type address: int
        """
        self.logger.debug("Erase flash page at address 0x%08X", address)

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready before flash page erase")

        # Erase command
        self.execute_nvm_command(self.NVMCMD_FLASH_PAGE_ERASE)

        # Dummy write
        self.readwrite.write_data(address, [0xFF])

        # And wait for it
        status = self.wait_nvm_ready()

        # Remove command from NVM controller
        self.logger.debug("Clear NVM command")
        self.execute_nvm_command(self.NVMCMD_NOCMD)
        if not status:
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready after flash page erase")
*/
  unsigned char data[1];
  int status;
  pmsg_debug("erase flash page at address 0x%08X\n", address);
  if (updi_nvm_wait_ready_V2(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_FLASH_PAGE_ERASE) < 0) {
    pmsg_error("flash page erase command failed\n");
    return -1;
  }
  data[0] = 0xFF;
  if (updi_write_data(pgm, address, data, 1) < 0) {
    pmsg_error("dummy write operation failed\n");
    return -1;
  }
  status = updi_nvm_wait_ready_V2(pgm, p); 
  pmsg_debug("clear NVM command\n");
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    pmsg_error("command buffer erase failed\n");
    return -1;
  }
  if (status < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_erase_eeprom_V2(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def erase_eeprom(self):
        """
        Erase EEPROM memory only
        """
        self.logger.debug("Erase EEPROM")

        # Wait until NVM CTRL is ready to erase
        if not self.wait_nvm_ready():
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready before EEPROM erase")

        # Erase
        self.execute_nvm_command(self.NVMCMD_EEPROM_ERASE)

        # And wait for it
        status = self.wait_nvm_ready()

        # Remove command from NVM controller
        self.logger.debug("Clear NVM command")
        self.execute_nvm_command(self.NVMCMD_NOCMD)
        if not status:
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready after EEPROM erase")
*/
  int status;
  pmsg_debug("erase EEPROM\n");
  if (updi_nvm_wait_ready_V2(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE) < 0) {
    pmsg_error("EEPROM erase command failed\n");
    return -1;
  }
  status = updi_nvm_wait_ready_V2(pgm, p); 
  pmsg_debug("clear NVM command\n");
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    pmsg_error("command buffer erase failed\n");
    return -1;
  }
  if (status < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_erase_user_row_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint16_t size) {
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
  return updi_nvm_erase_flash_page_V2(pgm, p, address);
}

static int nvm_write_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode);

int updi_nvm_write_flash_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
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

int updi_nvm_write_user_row_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
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

int updi_nvm_write_eeprom_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer, uint16_t size) {
/*
    def write_eeprom(self, address, data):
        """
        Writes data to NVM (EEPROM)

        :param address: address to write to
        :type address: int
        :param data: data to write
        :type data: list of bytes
        """
        nvm_command = self.NVMCMD_EEPROM_ERASE_WRITE

        # Check that NVM controller is ready
        if not self.wait_nvm_ready():
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM ready before command write")

        # Write the command to the NVM controller
        self.logger.debug("NVM EEPROM erase/write command")
        self.execute_nvm_command(nvm_command)

        # Write the data
        self.readwrite.write_data(address, data)

        # Wait for NVM controller to be ready again
        status = self.wait_nvm_ready()

        # Remove command from NVM controller
        self.logger.debug("Clear NVM command")
        self.execute_nvm_command(self.NVMCMD_NOCMD)
        if not status:
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM ready after data write")
*/
  int status;
  if (updi_nvm_wait_ready_V2(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  pmsg_debug("NVM EEPROM erase/write command\n");
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_EEPROM_ERASE_WRITE) < 0) {
    pmsg_error("EEPROM erase command failed\n");
    return -1;
  }
  if (updi_write_data(pgm, address, buffer, size) < 0) {
    pmsg_error("write data operation failed\n");
    return -1;
  }
  status = updi_nvm_wait_ready_V2(pgm, p); 
  pmsg_debug("clear NVM command\n");
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    pmsg_error("command buffer erase failed\n");
    return -1;
  }
  if (status < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_write_fuse_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, uint8_t value) {
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
  return updi_nvm_write_eeprom_V2(pgm, p, address, buffer, 1);
}

static int nvm_write_V2(const PROGRAMMER *pgm, const AVRPART *p, uint32_t address, unsigned char *buffer,
                        uint16_t size, access_mode mode)
{
/*
    def write_nvm(self, address, data, use_word_access=True):
        """
        Writes data to NVM.

        This version of the NVM block has no page buffer, so words are written directly.

        :param address: address to write to
        :type address: int
        :param data: data to write
        :type data: list of bytes
        :param use_word_access: True for 16-bit writes (eg: flash)
        :type use_word_access: bool, defaults to True
        :raises: PymcuprogSerialUpdiNvmTimeout if a timeout occurred
        :raises: PymcuprogSerialUpdiNvmError if an error condition is encountered
        """
        nvm_command = self.NVMCMD_FLASH_WRITE

        # Check that NVM controller is ready
        if not self.wait_nvm_ready():
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready before page buffer clear")

        # Write the command to the NVM controller
        self.logger.debug("NVM write command")
        self.execute_nvm_command(nvm_command)

        # Write the data
        if use_word_access:
            self.readwrite.write_data_words(address, data)
        else:
            self.readwrite.write_data(address, data)

        # Wait for NVM controller to be ready again
        status = self.wait_nvm_ready()

        # Remove command from NVM controller
        self.logger.debug("Clear NVM command")
        self.execute_nvm_command(self.NVMCMD_NOCMD)
        if not status:
            raise PymcuprogSerialUpdiNvmTimeout("Timeout waiting for NVM controller to be ready after data write")
*/
  int status;
  if (updi_nvm_wait_ready_V2(pgm, p) < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  pmsg_debug("NVM write command\n");
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_FLASH_WRITE) < 0) {
    pmsg_error("clear page operation failed\n");
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
  status = updi_nvm_wait_ready_V2(pgm, p); 
  pmsg_debug("clear NVM command\n");
  if (updi_nvm_command_V2(pgm, p, UPDI_V2_NVMCTRL_CTRLA_NOCMD) < 0) {
    pmsg_error("command buffer erase failed\n");
    return -1;
  }
  if (status < 0) {
    pmsg_error("updi_nvm_wait_ready_V2() failed\n");
    return -1;
  }
  return 0;
}

int updi_nvm_wait_ready_V2(const PROGRAMMER *pgm, const AVRPART *p) {
/*
    def wait_nvm_ready(self, timeout_ms=100):
        """
        Waits for the NVM controller to be ready

        :param timeout_ms: Timeout period in milliseconds
        :type timeout_ms: int, defaults to 100
        :returns: True if 'ready', False if timeout occurred before ready
        :rtype: bool
        :raises: PymcuprogSerialUpdiNvmError if an error condition is encountered
        """
        timeout = Timeout(timeout_ms)

        self.logger.debug("Wait NVM ready")
        while not timeout.expired():
            status = self.readwrite.read_byte(self.device.nvmctrl_address + self.NVMCTRL_STATUS)
            if status & self.STATUS_WRITE_ERROR_bm:
                self.logger.error("NVM error (%d)", status >> self.STATUS_WRITE_ERROR_bp)
                raise PymcuprogSerialUpdiNvmError(msg="NVM error", code=(status >> self.STATUS_WRITE_ERROR_bp))

            if not status & ((1 << self.STATUS_EEPROM_BUSY_bp) | (1 << self.STATUS_FLASH_BUSY_bp)):
                return True

        self.logger.error("Wait NVM ready timed out")
        return False
*/
  unsigned long start_time;
  unsigned long current_time;
  uint8_t status;
  start_time = avr_ustimestamp();
  do {
    if (updi_read_byte(pgm, p->nvm_base + UPDI_V2_NVMCTRL_STATUS, &status) >= 0) {
      if (status & UPDI_V2_NVM_STATUS_WRITE_ERROR_MASK) {
        pmsg_error("unable to write NVM status, error %d\n", status >> UPDI_V2_NVM_STATUS_WRITE_ERROR_BIT);
        return -1;
      }
      if (!(status & ((1 << UPDI_V2_NVM_STATUS_EEPROM_BUSY_BIT) | 
                      (1 << UPDI_V2_NVM_STATUS_FLASH_BUSY_BIT)))) {
        return 0;
      }
    }
    current_time = avr_ustimestamp();
  } while ((current_time - start_time) < 10000000);

  pmsg_error("wait NVM ready timed out\n");
  return -1;
}

int updi_nvm_command_V2(const PROGRAMMER *pgm, const AVRPART *p, uint8_t command) {
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

  return updi_write_byte(pgm, p->nvm_base + UPDI_V2_NVMCTRL_CTRLA, command);
}
