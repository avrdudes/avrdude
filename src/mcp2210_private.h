/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2026 Stefan Rueger
 *
 * Symbolic names mostly taken from
 * https://github.com/daniel-santos/mcp2210-linux/blob/master/mcp2210.h
 * Copyright (C) 2013-2017 Daniel Santos <daniel.santos@pobox.com>
 *               2013 Mathew King <mking@trilithic.com> for Trilithic, Inc
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This code is not affiliated in any way with Microchip®
 */

#ifndef mcp2210_private_h
#define mcp2210_private_h

/*
 * Valid command codes for the MCP2210 listed datas heet section numbers
 * in comments. See https://www.microchip.com/en-us/product/mcp2210
 */
enum mcp2210_cmd_code {
  MCP2210_CMD_SET_NVRAM           = 0x60, // 3.1.1 - 3.1.5
  MCP2210_CMD_GET_NVRAM           = 0x61, // 3.1.6 - 3.1.10
  MCP2210_CMD_SEND_PASSWD         = 0x70, // 3.1.11
  MCP2210_CMD_GET_SPI_CONFIG      = 0x41, // 3.2.1
  MCP2210_CMD_SET_SPI_CONFIG      = 0x40, // 3.2.2
  MCP2210_CMD_GET_CHIP_CONFIG     = 0x20, // 3.2.3
  MCP2210_CMD_SET_CHIP_CONFIG     = 0x21, // 3.2.4
  MCP2210_CMD_GET_PIN_DIR         = 0x33, // 3.2.5
  MCP2210_CMD_SET_PIN_DIR         = 0x32, // 3.2.6
  MCP2210_CMD_GET_PIN_VALUE       = 0x31, // 3.2.7
  MCP2210_CMD_SET_PIN_VALUE       = 0x30, // 3.2.8
  MCP2210_CMD_READ_EEPROM         = 0x50, // 3.3.1
  MCP2210_CMD_WRITE_EEPROM        = 0x51, // 3.3.2
  MCP2210_CMD_GET_INTERRUPTS      = 0x12, // 3.4.1
  MCP2210_CMD_SPI_TRANSFER        = 0x42, // 3.5.1
  MCP2210_CMD_SPI_CANCEL          = 0x11, // 3.5.2
  MCP2210_CMD_SPI_RELEASE         = 0x80, // 3.5.3
  MCP2210_CMD_GET_STATUS          = 0x10  // 3.6.1
};

// Valid subcommand codes for MCP2210_CMD_SET_NVRAM and MCP2210_CMD_GET_NVRAM
enum mcp2210_nvram_subcmd_code {
  MCP2210_NVRAM_SPI               = 0x10, // SPI power-up transfer settings
  MCP2210_NVRAM_CHIP              = 0x20, // Chip settings power-up default
  MCP2210_NVRAM_KEY_PARAMS        = 0x30, // USB power-up key parameters
  MCP2210_NVRAM_MFG               = 0x50, // USB manufacturer name
  MCP2210_NVRAM_PROD              = 0x40  // USB product name
};

// Status codes
enum mcp2210_status {
  MCP2210_STATUS_SUCCESS          = 0x00, // No error
  MCP2210_STATUS_XFER_DONE        = 0x10, // SPI transfer finished: no more data to send
  MCP2210_STATUS_XFER_STARTED     = 0x20, // SPI transfer started: no data to receive
  MCP2210_STATUS_XFER_NOT_FINISHED= 0x30, // SPI transfer not finished: received data available
  MCP2210_STATUS_SPI_NOT_OWNED    = 0xf7, // SPI bus is not currently available
  MCP2210_STATUS_BUSY             = 0xf8, // Either the SPI bus or USB interface is busy
  MCP2210_STATUS_UNKNOWN_CMD      = 0xf9, // Invalid command code sent
  MCP2210_STATUS_WRITE_FAIL       = 0xfa, // Write to EEPROM failed
  MCP2210_STATUS_NO_ACCESS        = 0xfb, // Either the device is permanently locked
                                          // or is password-protected and wrong password supplied
                                          // or tried too many times wrong password (power cycle)
  MCP2210_STATUS_PERM_LOCKED      = 0xfc, // Device is permanently locked
  MCP2210_STATUS_BAD_PASSWD       = 0xfd, // Bad password, but can try again
};

enum mcp2210_pin_mode {
  MCP2210_PIN_GPIO        = 0,
  MCP2210_PIN_SPI         = 1,
  MCP2210_PIN_DEDICATED   = 2,
};

enum mcp2210_gpio_direction {
  MCP2210_GPIO_NO_CHANGE  = -1,
  MCP2210_GPIO_OUTPUT     = 0,
  MCP2210_GPIO_INPUT      = 1
};

/*
 * Other settings: byte 17 of chip settings message (Section 3.1.1, 3.1.2 etc)
 *
 * These values are generally ORed together except that only one
 * MCP2210_INTERRUPT_* value may be chosen (they do not OR together). To disable
 * any of these options, exclude them (zero is "disabled" for all options).
 * See Table 3-1 in the datasheet for more information.
 */
enum mcp2210_other_settings {
  MCP2210_SPI_BUS_RELEASE_DISABLED = 0x01,
  MCP2210_INTERRUPT_HIGH_PULSE     = 0x08,
  MCP2210_INTERRUPT_LOW_PULSE      = 0x06,
  MCP2210_INTERRUPT_RISING_EDGE    = 0x04,
  MCP2210_INTERRUPT_FALLING_EDGE   = 0x02,
  MCP2210_REMOTE_WAKEUP_ENABLED    = 0x80,
};

enum mcp2210_eeprom_status {
  MCP2210_EEPROM_UNREAD       = 0,
  MCP2210_EEPROM_READ_PENDING = 1,
  MCP2210_EEPROM_READ         = 2,
  MCP2210_EEPROM_DIRTY        = 3
};

// Default vendor and product IDs
#define MCP2210_DEFAULT_VID      0x04d8
#define MCP2210_DEFAULT_PID      0x00de

// Max SPI payload and bitrate
#define MCP2210_PAYLOAD              60
#define MCP2210_MAX_SPI_BITRATE 3000000

#endif // mcp2210_private_h
