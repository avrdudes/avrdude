/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bdmicro.com>
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


typedef struct {
  char *kw;                     // Keyword near the comments
  LISTID comms;                 // Chained list of comments
  int rhs;                      // Comments to print rhs of keyword line
} COMMENT;


enum {                          // Which structures a component can occur in
  COMP_CONFIG_MAIN,
  COMP_PROGRAMMER,
  COMP_AVRPART,
  COMP_AVRMEM,
};

enum {                          // Component types in structure
  COMP_INT,
  COMP_SHORT,
  COMP_CHAR,
  COMP_BOOL,
  COMP_STRING,
  COMP_CHAR_ARRAY,              // This and below are not yet implemented
  COMP_INT_LISTID,
  COMP_STRING_LISTID,
  COMP_OPCODE,
  COMP_PIN,                     // Pins may never be implemented
  COMP_PIN_LIST
};

typedef struct {                // Description of a component in a structure
  const char *name;             // Component name
  int strct;                    // Structure, eg, COMP_AVRPART
  int offset, size, type;       // Location, size and type within structure
} Component_t;


enum {                          // Value types for VALUE struct
  V_NONE,
  V_NUM,
  V_NUM_REAL,
  V_STR,
  V_COMPONENT,
};

typedef struct value_t {
  int      type;
  union {
    int      number;
    double   number_real;
    char   * string;
    Component_t *comp;
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
extern int          current_strct;
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

TOKEN *new_number(const char *text);

TOKEN *new_number_real(const char *text);

TOKEN *new_constant(const char *text);

TOKEN *new_string(const char *text);

TOKEN *new_keyword(int primary);

void print_token(TOKEN *tkn);

void pyytext(void);

COMMENT *locate_comment(const LISTID comments, const char *where, int rhs);

void cfg_capture_prologue(void);

LISTID cfg_get_prologue(void);

void capture_comment_str(const char *com, int lineno);

void capture_lvalue_kw(const char *kw, int lineno);

LISTID cfg_move_comments(void);

void cfg_pop_comms(void);

Component_t *cfg_comp_search(const char *name, int strct);

const char *cfg_v_type(int type);

const char *cfg_strct_name(int strct);

void cfg_assign(char *sp, int strct, Component_t *cp, VALUE *v);

void cfg_update_mcuid(AVRPART *part);

#ifdef __cplusplus
}
#endif

#endif
