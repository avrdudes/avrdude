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


#define PGM_DESCLEN 80
typedef struct programmer_t {
  LISTID id;
  char desc[PGM_DESCLEN];
  unsigned int pinno[N_PINS];
} PROGRAMMER;

extern FILE       * yyin;
extern PROGRAMMER * current_prog;
extern AVRPART    * current_part;
extern int          current_mem;
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
