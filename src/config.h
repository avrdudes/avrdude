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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* $Id$ */

/* These are the internal definitions needed for config parsing */

#ifndef config_h
#define config_h

#include "libavrdude.h"

#if defined(WIN32) || defined(_MSC_VER) || defined(__MINGW32__)
#define realpath(N,R) _fullpath((R), (N), PATH_MAX)
#endif


enum { V_NONE, V_NUM, V_NUM_REAL, V_STR };
typedef struct value_t {
  int      type;
  union {
    int      number;
    double   number_real;
    char   * string;
  };
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
extern int          cfg_lineno;
extern char       * cfg_infile;
extern LISTID       string_list;
extern LISTID       number_list;
extern bool         is_alias; // current entry is alias


#if !defined(HAS_YYSTYPE)
#define YYSTYPE token_p
#endif
extern YYSTYPE yylval;

#ifdef __cplusplus
extern "C" {
#endif

int yyparse(void);

int yyerror(char *errmsg, ...);

int yywarning(char *errmsg, ...);

TOKEN *new_token(int primary);

void free_token(TOKEN *tkn);

void free_tokens(int n, ...);

TOKEN *number(const char *text);

TOKEN *number_real(const char *text);

TOKEN *hexnumber(const char *text);

TOKEN *string(const char *text);

TOKEN *keyword(int primary);

void print_token(TOKEN *tkn);

void pyytext(void);

int capture_comment_char(int c);

#ifdef __cplusplus
}
#endif

#endif
