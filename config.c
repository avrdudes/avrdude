/*
 * Copyright 2001  Brian S. Dean <bsd@bsdhome.com>
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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "avr.h"
#include "config.h"
#include "y.tab.h"

char string_buf[MAX_STR_CONST];
char *string_buf_ptr;

LISTID       string_list;
LISTID       number_list;
PROGRAMMER * current_prog;
AVRPART    * current_part;
AVRMEM     * current_mem;
LISTID       part_list;
LISTID       programmers;

int    lineno = 0;
char * infile = NULL;

#define DEBUG 0

char * config_version = "$Id$";


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
  extern char * yytext;

  fprintf(stderr, "TOKEN: \"%s\"\n", yytext);
#endif
}


PROGRAMMER * new_programmer(void)
{
  PROGRAMMER * p;
  int i;

  p = (PROGRAMMER *)malloc(sizeof(PROGRAMMER));
  if (p == NULL) {
    fprintf(stderr, "new_programmer(): out of memory\n");
    exit(1);
  }

  memset(p, 0, sizeof(*p));

  p->id = lcreat(NULL, 0);
  p->desc[0] = 0;

  for (i=0; i<N_PINS; i++)
    p->pinno[i] = 0;

  return p;
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

