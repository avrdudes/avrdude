/*
 * Copyright (c) 2000, 2001, 2002  Brian S. Dean <bsd@bsdhome.com>
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BRIAN S. DEAN ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BRIAN S. DEAN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * 
 */

/* $Id$ */

#ifndef __pgm_h__
#define __pgm_h__

#include <limits.h>

#include "avrpart.h"
#include "lists.h"
#include "pindefs.h"


#define ON  1
#define OFF 0

#define PGM_DESCLEN 80
#define PGM_PORTLEN PATH_MAX
#define PGM_TYPELEN 32
typedef struct programmer_t {
  LISTID id;
  char desc[PGM_DESCLEN];
  char type[PGM_TYPELEN];
  char port[PGM_PORTLEN];
  unsigned int pinno[N_PINS];
  int ppidata;
  int fd;
  int  (*rdy_led)        (struct programmer_t * pgm, int value);
  int  (*err_led)        (struct programmer_t * pgm, int value);
  int  (*pgm_led)        (struct programmer_t * pgm, int value);
  int  (*vfy_led)        (struct programmer_t * pgm, int value);
  int  (*initialize)     (struct programmer_t * pgm, AVRPART * p);
  void (*display)        (struct programmer_t * pgm, char * p);
  int  (*save)           (struct programmer_t * pgm);
  void (*restore)        (struct programmer_t * pgm);
  void (*enable)         (struct programmer_t * pgm);
  void (*disable)        (struct programmer_t * pgm);
  void (*powerup)        (struct programmer_t * pgm);
  void (*powerdown)      (struct programmer_t * pgm);
  int  (*program_enable) (struct programmer_t * pgm, AVRPART * p);
  int  (*chip_erase)     (struct programmer_t * pgm, AVRPART * p);
  int  (*cmd)            (struct programmer_t * pgm, unsigned char cmd[4], 
                          unsigned char res[4]);
  void (*open)           (struct programmer_t * pgm, char * port);
  void (*close)          (struct programmer_t * pgm);
  int  (*paged_write)    (struct programmer_t * pgm, AVRPART * p, AVRMEM * m, 
                          int n_bytes);
  int  (*paged_load)     (struct programmer_t * pgm, AVRPART * p, AVRMEM * m,
                          int n_bytes);
} PROGRAMMER;


PROGRAMMER * pgm_new(void);

#endif
