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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "avr.h"
#include "config.h"
#include "config_gram.h"

char default_programmer[MAX_STR_CONST];
char default_parallel[PATH_MAX];
char default_serial[PATH_MAX];

char string_buf[MAX_STR_CONST];
char *string_buf_ptr;

LISTID       string_list;
LISTID       number_list;
PROGRAMMER * current_prog;
AVRPART    * current_part;
AVRMEM     * current_mem;
LISTID       part_list;
LISTID       programmers;

int    lineno;
char * infile;

extern char * yytext;

#define DEBUG 0


int init_config(void)
{
  string_list  = lcreat(NULL, 0);
  number_list  = lcreat(NULL, 0);
  current_prog = NULL;
  current_part = NULL;
  current_mem  = 0;
  part_list    = lcreat(NULL, 0);
  programmers  = lcreat(NULL, 0);

  lineno       = 1;
  infile       = NULL;

  return 0;
}



int yywrap()
{
  return 1;
}


int yyerror(char * errmsg)
{
  fprintf(stderr, "%s at %s:%d\n", errmsg, infile, lineno);
  exit(1);
}


TOKEN * new_token(int primary)
{
  TOKEN * tkn;

  tkn = (TOKEN *)malloc(sizeof(TOKEN));
  if (tkn == NULL) {
    fprintf(stderr, "new_token(): out of memory\n");
    exit(1);
  }

  memset(tkn, 0, sizeof(TOKEN));

  tkn->primary = primary;

  return tkn;
}


void free_token(TOKEN * tkn)
{
  if (tkn) {
    switch (tkn->primary) {
      case TKN_STRING:
      case TKN_ID:
        if (tkn->value.string)
          free(tkn->value.string);
        tkn->value.string = NULL;
        break;
    }
    
    free(tkn);
  }
}


void free_tokens(int n, ...)
{
  TOKEN * t;
  va_list ap;

  va_start(ap, n);
  while (n--) {
    t = va_arg(ap, TOKEN *);
    free_token(t);
  }
  va_end(ap);
}



TOKEN * number(char * text)
{
  struct token_t * tkn;

  tkn = new_token(TKN_NUMBER);
  tkn->value.type   = V_NUM;
  tkn->value.number = atof(text);

#if DEBUG
  fprintf(stderr, "NUMBER(%g)\n", tkn->value.number);
#endif

  return tkn;
}


TOKEN * hexnumber(char * text)
{
  struct token_t * tkn;
  char * e;

  tkn = new_token(TKN_NUMBER);
  tkn->value.type   = V_NUM;
  tkn->value.number = strtoul(text, &e, 16);
  if ((e == text) || (*e != 0)) {
    fprintf(stderr, "error at %s:%d: can't scan hex number \"%s\"\n",
            infile, lineno, text);
    exit(1);
  }
  
#if DEBUG
  fprintf(stderr, "HEXNUMBER(%g)\n", tkn->value.number);
#endif

  return tkn;
}


TOKEN * string(char * text)
{
  struct token_t * tkn;
  int len;

  tkn = new_token(TKN_STRING);

  len = strlen(text);

  tkn->value.type   = V_STR;
  tkn->value.string = (char *) malloc(len+1);
  if (tkn->value.string == NULL) {
    fprintf(stderr, "id(): out of memory\n");
    exit(1);
  }
  strcpy(tkn->value.string, text);

#if DEBUG
  fprintf(stderr, "STRING(%s)\n", tkn->value.string);
#endif

  return tkn;
}


TOKEN * id(char * text)
{
  struct token_t * tkn;
  int len;

  tkn = new_token(TKN_ID);

  len = strlen(text);

  tkn->value.type   = V_STR;
  tkn->value.string = (char *) malloc(len+1);
  if (tkn->value.string == NULL) {
    fprintf(stderr, "id(): out of memory\n");
    exit(1);
  }
  strcpy(tkn->value.string, text);

#if DEBUG
  fprintf(stderr, "ID(%s)\n", tkn->value.string);
#endif

  return tkn;
}


TOKEN * keyword(int primary)
{
  struct token_t * tkn;

  tkn = new_token(primary);

  return tkn;
}


void print_token(TOKEN * tkn)
{
  if (!tkn)
    return;

  fprintf(stderr, "token = %d = ", tkn->primary);
  switch (tkn->primary) {
    case TKN_NUMBER: 
      fprintf(stderr, "NUMBER, value=%g", tkn->value.number); 
      break;

    case TKN_STRING: 
      fprintf(stderr, "STRING, value=%s", tkn->value.string); 
      break;

    case TKN_ID:  
      fprintf(stderr, "ID,     value=%s", tkn->value.string); 
      break;

    default:     
      fprintf(stderr, "<other>"); 
      break;
  }

  fprintf(stderr, "\n");
}


void pyytext(void)
{
#if DEBUG
  fprintf(stderr, "TOKEN: \"%s\"\n", yytext);
#endif
}


char * dup_string(char * str)
{
  char * s;

  s = strdup(str);
  if (s == NULL) {
    fprintf(stderr, "dup_string(): out of memory\n");
    exit(1);
  }

  return s;
}

