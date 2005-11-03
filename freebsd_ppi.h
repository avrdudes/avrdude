/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005 Joerg Wunsch
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

#ifndef freebsd_ppi_h
#define freebsd_ppi_h

#include <dev/ppbus/ppi.h>

#define ppi_claim(pgm) {}

#define ppi_release(pgm) {}

#define DO_PPI_READ(fd, reg, valp) \
	(void)ioctl(fd, \
		(reg) == PPIDATA? PPIGDATA: ((reg) == PPICTRL? PPIGCTRL: PPIGSTATUS), \
		    valp)
#define DO_PPI_WRITE(fd, reg, valp) \
	(void)ioctl(fd, \
		(reg) == PPIDATA? PPISDATA: ((reg) == PPICTRL? PPISCTRL: PPISSTATUS), \
		    valp)

#endif /* freebsd_ppi_h */
