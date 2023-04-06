/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 *
 * avrdude support for CH341A/B
 * Copyright (C) 2016  Alexey Sadkov
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
 */


#ifndef ch341a_h
#define ch341a_h

#define CH341A_VID             0x1A86
#define CH341A_PID             0x5512

#define CH341A_PACKET_LENGTH     0x20

#define CH341A_USB_BULK_ENDPOINT 0x02
#define CH341A_PACKET_LENGTH     0x20

#define CH341A_USB_TIMEOUT      15000

#define CH341A_CMD_SPI_STREAM    0xA8 // SPI command
#define CH341A_CMD_UIO_STREAM    0xAB // UIO command

#define CH341A_CMD_UIO_STM_IN    0x00 // UIO Interface In (D0~D7)
#define CH341A_CMD_UIO_STM_DIR   0x40 // UIO interface Dir (set dir of D0~D5)
#define CH341A_CMD_UIO_STM_OUT   0x80 // UIO Interface Output (D0~D5)
#define CH341A_CMD_UIO_STM_END   0x20 // UIO Interface End Command

#define  CH341A_CMD_I2C_STREAM   0xAA
#define  CH341A_CMD_I2C_STM_SET  0x60 // Bit 2: SPI with two data pairs D5,D4=out, D7,D6=in
#define  CH341A_CMD_I2C_STM_END  0x00


// USB error identifiers
#define USB_ERROR_NOTFOUND    1
#define USB_ERROR_ACCESS      2
#define USB_ERROR_IO          3


#ifdef __cplusplus
extern "C" {
#endif

extern const char ch341a_desc[];
void ch341a_initpgm (PROGRAMMER * pgm);

#ifdef __cplusplus
}
#endif

#endif // ch341a_h
