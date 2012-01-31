/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002-2004  Brian S. Dean <bsd@bsdhome.com>
 * Copyright 2007 Joerg Wunsch <j@uriah.heep.sax.de>
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

/* $Id: pgm.h 1007 2011-09-14 21:49:42Z joerg_wunsch $ */

#ifndef pgm_type_h
#define pgm_type_h

#include "lists.h"
#include "pgm.h"

/*LISTID programmer_types;*/

typedef struct programmer_type_t {
  const char * const id;
  void (*initpgm)(struct programmer_t * pgm);
  const char * const desc;
} PROGRAMMER_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

const PROGRAMMER_TYPE * locate_programmer_type(/*LISTID programmer_types, */const char * id);

typedef void (*walk_programmer_types_cb)(const char *id, const char *desc,
                                    void *cookie);
void walk_programmer_types(/*LISTID programmer_types,*/ walk_programmer_types_cb cb, void *cookie);

#ifdef __cplusplus
}
#endif

#endif
