/*
 * Copyright 2002  Brian S. Dean <bsd@bsdhome.com>
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


#include <stdlib.h>
#include <stdio.h>

#include "pgm.h"

extern char * progname;

int  pgm_default_1 (struct programmer_t *, int);
int  pgm_default_2 (struct programmer_t *, AVRPART *);
int  pgm_default_3 (struct programmer_t *);
void pgm_default_4 (struct programmer_t *);
int  pgm_default_5 (struct programmer_t *, unsigned char cmd[4], 
                    unsigned char res[4]);
void pgm_default_6 (struct programmer_t *, char *);

char * pgm_version = "$Id$";


PROGRAMMER * pgm_new(void)
{
  int i;
  PROGRAMMER * pgm;

  pgm = (PROGRAMMER *)malloc(sizeof(*pgm));
  if (pgm == NULL) {
    fprintf(stderr, "%s: out of memory allocating programmer structure\n",
            progname);
    exit(1);
  }

  memset(pgm, 0, sizeof(*pgm));

  pgm->id = lcreat(NULL, 0);
  pgm->desc[0] = 0;
  pgm->type[0] = 0;

  for (i=0; i<N_PINS; i++)
    pgm->pinno[i] = 0;

  /*
   * mandatory functions - these are called without checking to see
   * whether they are assigned or not
   */
  pgm->rdy_led        = pgm_default_1;
  pgm->err_led        = pgm_default_1;
  pgm->pgm_led        = pgm_default_1;
  pgm->vfy_led        = pgm_default_1;
  pgm->initialize     = pgm_default_2;
  pgm->display        = pgm_default_6;
  pgm->save           = pgm_default_3;
  pgm->restore        = pgm_default_4;
  pgm->enable         = pgm_default_4;
  pgm->disable        = pgm_default_4;
  pgm->powerup        = pgm_default_4;
  pgm->powerdown      = pgm_default_4;
  pgm->program_enable = pgm_default_2;
  pgm->chip_erase     = pgm_default_2;
  pgm->cmd            = pgm_default_5;
  pgm->open           = pgm_default_6;
  pgm->close          = pgm_default_4;

  /*
   * optional functions - these are checked to make sure they are
   * assigned before they are called
   */
  pgm->paged_write    = NULL;
  pgm->paged_load     = NULL;

  return pgm;
}


void pgm_default(void)
{
  fprintf(stderr, "%s: programmer operation not supported\n", progname);
}


int  pgm_default_1 (struct programmer_t * pgm, int value)
{
  pgm_default();
  return -1;
}

int  pgm_default_2 (struct programmer_t * pgm, AVRPART * p)
{
  pgm_default();
  return -1;
}

int  pgm_default_3 (struct programmer_t * pgm)
{
  pgm_default();
  return -1;
}

void pgm_default_4 (struct programmer_t * pgm)
{
  pgm_default();
}

int  pgm_default_5 (struct programmer_t * pgm, unsigned char cmd[4], 
                    unsigned char res[4])
{
  pgm_default();
  return -1;
}

void pgm_default_6 (struct programmer_t * pgm, char * p)
{
  pgm_default();
}


