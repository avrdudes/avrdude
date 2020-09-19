/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2013 Kevin Cuzner <kevin@kevincuner.com>
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

#ifndef linuxspi_h
#define linuxspi_h

#ifdef __cplusplus
extern "C" {
#endif

extern const char linuxspi_desc[];
void linuxspi_initpgm        (PROGRAMMER * pgm);

#ifdef __cplusplus
}
#endif

#endif //linuxspi_h

