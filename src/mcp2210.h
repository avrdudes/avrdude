/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2026 David Hubner
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

#ifndef mcp2210_h
#define mcp2210_h

/*
 * Constant naming taken from https://github.com/daniel-santos
 */

#define MCP2210_CMD_GET_STATUS 0x10
#define MCP2210_CMD_SPI_CANCEL 0x11
#define MCP2210_CMD_SET_SPI_CONFIG 0x40
#define MCP2210_CMD_GET_SPI_CONFIG 0x41
#define MCP2210_CMD_SPI_TRANSFER 0x42

#define MCP2210_STATUS_SUCCESS 0x00
#define MCP2210_STATUS_XFER_DONE 0x10
#define MCP2210_STATUS_XFER_STARTED 0x20
#define MCP2210_STATUS_XFER_NOT_FINISHED 0x30
#define MCP2210_STATUS_SPI_NOT_OWNED 0xF7
#define MCP2210_STATUS_BUSY	 0xF8

#define MCP2210_DEFAULT_VID 0x04d8
#define MCP2210_DEFAULT_PID 0x00DE
#define HID_REPORT_LENGTH 65
#define TRANSFER_RETRIES 30
#define SPI_MAX_CHUNK 60
#define MAXIMUM_TRANSFER 3000000

#ifdef __cplusplus
extern "C" {
#endif

extern const char mcp2210_desc[];
void mcp2210_initpgm (PROGRAMMER * pgm);

#ifdef __cplusplus
}
#endif

#endif // mcp2210_h
