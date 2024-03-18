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
#include "updi_constants.h"
#include "updi_link.h"
#include "updi_readwrite.h"

int updi_read_cs(const PROGRAMMER *pgm, uint8_t address, uint8_t *value) {
/*
    def read_cs(self, address):
        """
        Read from Control/Status space

        :param address: address (index) to read
        :return: value read
        """
        return self.datalink.ldcs(address)
*/
  return updi_link_ldcs(pgm, address, value);
}

int updi_write_cs(const PROGRAMMER *pgm, uint8_t address, uint8_t value) {
/*
    def write_cs(self, address, value):
        """
        Write to Control/Status space

        :param address: address (index) to write
        :param value: 8-bit value to write
        """
        return self.datalink.stcs(address, value)
*/
  return updi_link_stcs(pgm, address, value);
}

int updi_write_key(const PROGRAMMER *pgm, unsigned char *buffer, uint8_t size_type, uint16_t size) {
/*
    def write_key(self, size, key):
        """
        Write a KEY into UPDI

        :param size: size of key to send
        :param key: key value
        """
        return self.datalink.key(size, key)
*/
  return updi_link_key(pgm, buffer, size_type, size);
}

int updi_read_sib(const PROGRAMMER *pgm, unsigned char *buffer, uint16_t size) {
/*
    def read_sib(self):
        """
        Read the SIB from UPDI

        :return: SIB string (bytearray) read
        """
        return self.datalink.read_sib()
*/
  return updi_link_read_sib(pgm, buffer, size);
}

int updi_read_byte(const PROGRAMMER *pgm, uint32_t address, uint8_t *value) {
/*
    def read_byte(self, address):
        """
        Read a single byte from UPDI

        :param address: address to read from
        :return: value read
        """
        return self.datalink.ld(address)
*/
  return updi_link_ld(pgm, address, value);
}

int updi_write_byte(const PROGRAMMER *pgm, uint32_t address, uint8_t value) {
/*
    def write_byte(self, address, value):
        """
        Writes a single byte to UPDI

        :param address: address to write to
        :param value: value to write
        """
        return self.datalink.st(address, value)
*/
  return updi_link_st(pgm, address, value);
}

int updi_read_data(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size) {
/*
    def read_data(self, address, size):
        """
        Reads a number of bytes of data from UPDI

        :param address: address to write to
        :param size: number of bytes to read
        """
        self.logger.debug("Reading %d bytes from 0x%04X", size, address)
        # Range check
        if size > constants.UPDI_MAX_REPEAT_SIZE:
            raise PymcuprogError("Can't read that many bytes in one go")

        # Store the address
        self.datalink.st_ptr(address)

        # Fire up the repeat
        if size > 1:
            self.datalink.repeat(size)

        # Do the read(s)
        return self.datalink.ld_ptr_inc(size)
*/
  pmsg_debug("reading %d bytes from 0x%06X\n", size, address);

  if (size > UPDI_MAX_REPEAT_SIZE) {
    pmsg_debug("cannot read that many bytes in one go\n");
    return -1;
  }

  if (updi_link_st_ptr(pgm, address) < 0) {
    pmsg_debug("ST_PTR operation failed\n");
    return -1;
  }

  if (size > 1) {
    if (updi_link_repeat(pgm, size) < 0) {
      pmsg_debug("repeat operation failed\n");
      return -1;
    }
  }
  return updi_link_ld_ptr_inc(pgm, buffer, size);
}

int updi_write_data(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size) {
/*
    def write_data(self, address, data):
        """
        Writes a number of bytes to memory

        :param address: address to write to
        :param data: data to write
        """
        # Special case of 1 byte
        if len(data) == 1:
            return self.datalink.st(address, data[0])
        # Special case of 2 byte
        if len(data) == 2:
            self.datalink.st(address, data[0])
            return self.datalink.st(address + 1, data[1])

        # Range check
        if len(data) > constants.UPDI_MAX_REPEAT_SIZE:
            raise PymcuprogError("Invalid length")

        # Store the address
        self.datalink.st_ptr(address)

        # Fire up the repeat
        self.datalink.repeat(len(data))
        return self.datalink.st_ptr_inc(data)
*/
  if (size == 1) {
    return updi_link_st(pgm, address, buffer[0]);
  }
  if (size == 2) {
    if (updi_link_st(pgm, address, buffer[0]) < 0) {
      pmsg_debug("ST operation failed\n");
      return -1;
    }
    return updi_link_st(pgm, address+1, buffer[1]);
  }
  if (size > UPDI_MAX_REPEAT_SIZE) {
    pmsg_debug("invalid length\n");
    return -1;
  }
  if (updi_link_st_ptr(pgm, address) < 0) {
    pmsg_debug("ST_PTR operation failed\n");
    return -1;
  }
  if (updi_link_repeat(pgm, size) < 0) {
    pmsg_debug("repeat operation failed\n");
    return -1;
  }
  return updi_link_st_ptr_inc(pgm, buffer, size);
}

int updi_read_data_words(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size) {
/*
    def read_data_words(self, address, words):
        """
        Reads a number of words of data from UPDI

        :param address: address to write to
        :param words: number of words to read
        """
        self.logger.debug("Reading %d words from 0x%04X", words, address)

        # Range check
        if words > constants.UPDI_MAX_REPEAT_SIZE >> 1:
            raise PymcuprogError("Can't read that many words in one go")

        # Store the address
        self.datalink.st_ptr(address)

        # Fire up the repeat
        if words > 1:
            self.datalink.repeat(words)

        # Do the read
        return self.datalink.ld_ptr_inc16(words)
*/
  pmsg_debug("reading %d words from 0x%06X", size, address);

  if (size > (UPDI_MAX_REPEAT_SIZE >> 1)) {
    pmsg_debug("cannot read that many words in one go\n");
    return -1;
  }

  if (updi_link_st_ptr(pgm, address) < 0) {
    pmsg_debug("ST_PTR operation failed\n");
    return -1;
  }

  if (size > 1) {
    if (updi_link_repeat(pgm, size) < 0) {
      pmsg_debug("repeat operation failed\n");
      return -1;
    }
  }
  return updi_link_ld_ptr_inc16(pgm, buffer, size);
}

int updi_write_data_words(const PROGRAMMER *pgm, uint32_t address, uint8_t *buffer, uint16_t size) {
/*
    def write_data_words(self, address, data):
        """
        Writes a number of words to memory

        :param address: address to write to
        :param data: data to write
        """
        # Special-case of 1 word
        if len(data) == 2:
            value = data[0] + (data[1] << 8)
            return self.datalink.st16(address, value)

        # Range check
        if len(data) > constants.UPDI_MAX_REPEAT_SIZE << 1:
            raise PymcuprogError("Invalid length")

        # Store the address
        self.datalink.st_ptr(address)

        # Fire up the repeat
        self.datalink.repeat(len(data) >> 1)
        return self.datalink.st_ptr_inc16(data)
*/
  if (size == 2) {
    return updi_link_st16(pgm, address, buffer[0] + (buffer[1] << 8));
  }
  if (size > UPDI_MAX_REPEAT_SIZE << 1) {
    pmsg_debug("invalid length\n");
    return -1;
  }
  if (updi_link_st_ptr(pgm, address) < 0) {
    pmsg_debug("ST_PTR operation failed\n");
    return -1;
  }
  return updi_link_st_ptr_inc16_RSD(pgm, buffer, size >> 1, -1);
}
