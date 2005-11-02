/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
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

#ifndef __config_h__
#define __config_h__

#include "lists.h"
#include "pindefs.h"
#include "avr.h"


#define MAX_STR_CONST 1024

enum { V_NONE, V_NUM, V_STR };
typedef struct value_t {
  int      type;
  double   number;
  char   * string;
} VALUE;


typedef struct token_t {
  int primary;
  VALUE value;
} TOKEN;
typedef struct token_t *token_p;


extern FILE       * yyin;
extern PROGRAMMER * current_prog;
extern AVRPART    * current_part;
extern AVRMEM     * current_mem;
extern int          lineno;
extern char       * infile;
extern LISTID       string_list;
extern LISTID       number_list;
extern char         default_programmer[];
extern char         default_parallel[];
extern char         default_serial[];



#if !defined(HAS_YYSTYPE)
#define YYSTYPE token_p
#endif
extern YYSTYPE yylval;

extern char string_buf[MAX_STR_CONST];
extern char *string_buf_ptr;

int yyparse(void);


int init_config(void);

TOKEN * new_token(int primary);

void free_token(TOKEN * tkn);

void free_tokens(int n, ...);

TOKEN * number(char * text);

TOKEN * hexnumber(char * text);

TOKEN * string(char * text);

TOKEN * id(char * text);

TOKEN * keyword(int primary);

void print_token(TOKEN * tkn);

PROGRAMMER * new_programmer(void);

AVRPART * new_part(void);

AVRPART * dup_part(AVRPART * d);

char * dup_string(char * str);

#endif
