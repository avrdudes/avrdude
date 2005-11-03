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

#ifndef solaris_ecpp_h
#define solaris_ecpp_h

#include <sys/ecppio.h>

#define ppi_claim(pgm) \
	do { \
		struct ecpp_transfer_parms p; \
		(void)ioctl(pgm->fd, ECPPIOC_GETPARMS, &p); \
		p.mode = ECPP_DIAG_MODE; \
		(void)ioctl(pgm->fd, ECPPIOC_SETPARMS, &p); \
	} while(0);

#define ppi_release(pgm)

#define DO_PPI_READ(fd, reg, valp) \
	do { struct ecpp_regs r; \
	if ((reg) == PPIDATA) { (void)ioctl(fd, ECPPIOC_GETDATA, valp); } \
	else { (void)ioctl(fd, ECPPIOC_GETREGS, &r); \
		*(valp) = ((reg) == PPICTRL)? r.dcr: r.dsr; } \
	} while(0)
#define DO_PPI_WRITE(fd, reg, valp) \
	do { struct ecpp_regs r; \
	if ((reg) == PPIDATA) { (void)ioctl(fd, ECPPIOC_SETDATA, valp); } \
	else { if ((reg) == PPICTRL) r.dcr = *(valp); else r.dsr = *(valp); \
		(void)ioctl(fd, ECPPIOC_SETREGS, &r); } \
	} while(0)


#endif /* solaris_ecpp_h */
