/*
 * Copyright (c) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
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


extern FILE       * yyin;
extern PROGRAMMER * current_prog;
extern AVRPART    * current_part;
extern AVRMEM     * current_mem;
extern LISTID       programmers;
extern LISTID       part_list;
extern int          lineno;
extern char       * infile;
extern LISTID       string_list;
extern LISTID       number_list;

#if 0
#define YYSTYPE struct token_t *
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
