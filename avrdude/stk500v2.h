/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2005  Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
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

#ifndef stk500v2_h__
#define stk500v2_h__

void stk500v2_initpgm (PROGRAMMER * pgm);
void stk500hvsp_initpgm (PROGRAMMER * pgm);
void stk500pp_initpgm (PROGRAMMER * pgm);
void stk500v2_jtagmkII_initpgm(PROGRAMMER * pgm);
void stk500v2_dragon_isp_initpgm(PROGRAMMER * pgm);

#endif


