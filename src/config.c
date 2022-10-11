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

#include "ac_cfg.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "config.h"
#include "avrintel.h"

#include "config_gram.h"

const char *default_programmer;
const char *default_parallel;
const char *default_serial;
const char *default_spi;
double default_bitclock;

LISTID       string_list;
LISTID       number_list;
PROGRAMMER * current_prog;
AVRPART    * current_part;
AVRMEM     * current_mem;
int          current_strct;
LISTID       part_list;
LISTID       programmers;
bool         is_alias;

int    cfg_lineno;
char * cfg_infile;

extern char * yytext;

#define pgm_comp_desc(x, type)  { #x, COMP_PROGRAMMER, offsetof(PROGRAMMER, x), sizeof(((PROGRAMMER *) NULL)->x), type }
#define part_comp_desc(x, type) { #x, COMP_AVRPART, offsetof(AVRPART, x), sizeof(((AVRPART *) NULL)->x), type }
#define mem_comp_desc(x, type)  { #x, COMP_AVRMEM, offsetof(AVRMEM, x), sizeof(((AVRMEM *) NULL)->x), type }

// Component description for config_gram.y, will be sorted appropriately on first use
Component_t avr_comp[] = {
  // PROGRAMMER
  pgm_comp_desc(prog_modes, COMP_INT),

  // AVRPART
  part_comp_desc(prog_modes, COMP_INT),
  part_comp_desc(mcuid, COMP_INT),
  part_comp_desc(n_interrupts, COMP_INT),
  part_comp_desc(n_page_erase, COMP_INT),

  // AVRMEM
  mem_comp_desc(n_word_writes, COMP_INT),
};

#define DEBUG 0

void cleanup_config(void)
{
  ldestroy_cb(part_list, (void(*)(void*))avr_free_part);
  ldestroy_cb(programmers, (void(*)(void*))pgm_free);
  ldestroy_cb(string_list, (void(*)(void*))free_token);
  ldestroy_cb(number_list, (void(*)(void*))free_token);
}

int init_config(void)
{
  string_list  = lcreat(NULL, 0);
  number_list  = lcreat(NULL, 0);
  current_prog = NULL;
  current_part = NULL;
  current_mem  = NULL;
  part_list    = lcreat(NULL, 0);
  programmers  = lcreat(NULL, 0);
  is_alias     = false;

  cfg_lineno   = 1;
  cfg_infile   = NULL;

  return 0;
}

void *cfg_malloc(const char *funcname, size_t n) {
  void *ret = malloc(n);
  if(!ret) {
    avrdude_message(MSG_INFO, "%s: out of memory in %s (needed %lu bytes)\n", progname, funcname, (unsigned long) n);
    exit(1);
  }
  memset(ret, 0, n);
  return ret;
}

void *cfg_realloc(const char *funcname, void *p, size_t n) {
  void *ret;

  if(!(ret = p? realloc(p, n): calloc(1, n))) {
    avrdude_message(MSG_INFO, "%s: out of memory in %s (needed %lu bytes)\n", progname, funcname, (unsigned long) n);
    exit(1);
  }

  return ret;
}


char *cfg_strdup(const char *funcname, const char *s) {
  char *ret = strdup(s);
  if(!ret) {
    avrdude_message(MSG_INFO, "%s: out of memory in %s\n", progname, funcname);
    exit(1);
  }
  return ret;
}


int yywrap()
{
  return 1;
}


int yyerror(char * errmsg, ...)
{
  va_list args;

  char message[512];

  va_start(args, errmsg);

  vsnprintf(message, sizeof(message), errmsg, args);
  avrdude_message(MSG_INFO, "%s: error at %s:%d: %s\n", progname, cfg_infile, cfg_lineno, message);

  va_end(args);

  return 0;
}


int yywarning(char * errmsg, ...)
{
  va_list args;

  char message[512];

  va_start(args, errmsg);

  vsnprintf(message, sizeof(message), errmsg, args);
  avrdude_message(MSG_INFO, "%s: warning at %s:%d: %s\n", progname, cfg_infile, cfg_lineno, message);

  va_end(args);

  return 0;
}


TOKEN * new_token(int primary) {
  TOKEN * tkn = (TOKEN *) cfg_malloc("new_token()", sizeof(TOKEN));
  tkn->primary = primary;
  return tkn;
}


void free_token(TOKEN * tkn)
{
  if (tkn) {
    switch (tkn->value.type) {
      case V_STR:
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



TOKEN *new_number(const char *text) {
  struct token_t *tkn = new_token(TKN_NUMBER);
  tkn->value.type   = V_NUM;
  tkn->value.number = atoi(text);

#if DEBUG
  avrdude_message(MSG_INFO, "NUMBER(%d)\n", tkn->value.number);
#endif

  return tkn;
}

TOKEN *new_number_real(const char *text) {
  struct token_t * tkn = new_token(TKN_NUMBER);
  tkn->value.type   = V_NUM_REAL;
  tkn->value.number_real = atof(text);

#if DEBUG
  avrdude_message(MSG_INFO, "NUMBER(%g)\n", tkn->value.number_real);
#endif

  return tkn;
}

TOKEN *new_hexnumber(const char *text) {
  struct token_t *tkn = new_token(TKN_NUMBER);
  char * e;

  tkn->value.type   = V_NUM;
  tkn->value.number = strtoul(text, &e, 16);
  if ((e == text) || (*e != 0)) {
    yyerror("can't scan hex number \"%s\"", text);
    free_token(tkn);
    return NULL;
  }
  
#if DEBUG
  avrdude_message(MSG_INFO, "HEXNUMBER(%g)\n", tkn->value.number);
#endif

  return tkn;
}

TOKEN *new_constant(const char *con) {
  struct token_t *tkn = new_token(TKN_NUMBER);
  int assigned = 1;

  tkn->value.type = V_NUM;
  tkn->value.number =
    !strcmp("PM_SPM", con)? PM_SPM:
    !strcmp("PM_TPI", con)? PM_TPI:
    !strcmp("PM_ISP", con)? PM_ISP:
    !strcmp("PM_PDI", con)? PM_PDI:
    !strcmp("PM_UPDI", con)? PM_UPDI:
    !strcmp("PM_HVSP", con)? PM_HVSP:
    !strcmp("PM_HVPP", con)? PM_HVPP:
    !strcmp("PM_debugWIRE", con)? PM_debugWIRE:
    !strcmp("PM_JTAG", con)? PM_JTAG:
    !strcmp("PM_JTAGmkI", con)? PM_JTAGmkI:
    !strcmp("PM_XMEGAJTAG", con)? PM_XMEGAJTAG:
    !strcmp("PM_AVR32JTAG", con)? PM_AVR32JTAG:
    !strcmp("PM_aWire", con)? PM_aWire:
    (assigned = 0);

  if(!assigned) {
    yyerror("can't identify constant %s", con);
    free_token(tkn);
    return NULL;
  }

#if DEBUG
  avrdude_message(MSG_INFO, "CONSTANT(%s=%d)\n", con, tkn->value.number);
#endif

  return tkn;
}

TOKEN *new_string(const char *text) {
  struct token_t *tkn = new_token(TKN_STRING);
  tkn->value.type   = V_STR;
  tkn->value.string = cfg_strdup("new_string()", text);

#if DEBUG
  avrdude_message(MSG_INFO, "STRING(%s)\n", tkn->value.string);
#endif

  return tkn;
}


TOKEN *new_keyword(int primary) {
  return new_token(primary);
}


void print_token(TOKEN * tkn)
{
  if (!tkn)
    return;

  avrdude_message(MSG_INFO, "token = %d = ", tkn->primary);
  switch (tkn->value.type) {
    case V_NUM:
      avrdude_message(MSG_INFO, "NUMBER, value=%d", tkn->value.number);
      break;

    case V_NUM_REAL:
      avrdude_message(MSG_INFO, "NUMBER, value=%g", tkn->value.number_real);
      break;

    case V_STR:
      avrdude_message(MSG_INFO, "STRING, value=%s", tkn->value.string);
      break;

    default:
      avrdude_message(MSG_INFO, "<other>");
      break;
  }

  avrdude_message(MSG_INFO, "\n");
}


void pyytext(void)
{
#if DEBUG
  avrdude_message(MSG_INFO, "TOKEN: \"%s\"\n", yytext);
#endif
}


#ifdef HAVE_YYLEX_DESTROY
/* reset lexer and free any allocated memory */
extern int yylex_destroy(void);
#endif

int read_config(const char * file)
{
  FILE * f;
  int r;

  if(!(cfg_infile = realpath(file, NULL))) {
    avrdude_message(MSG_INFO, "%s: can't determine realpath() of config file \"%s\": %s\n",
            progname, file, strerror(errno));
    return -1;
  }

  f = fopen(cfg_infile, "r");
  if (f == NULL) {
    avrdude_message(MSG_INFO, "%s: can't open config file \"%s\": %s\n",
            progname, cfg_infile, strerror(errno));
    free(cfg_infile);
    cfg_infile = NULL;
    return -1;
  }

  cfg_lineno = 1;
  yyin   = f;

  r = yyparse();

#ifdef HAVE_YYLEX_DESTROY
  /* reset lexer and free any allocated memory */
  yylex_destroy();
#endif

  fclose(f);

  if(cfg_infile) {
    free(cfg_infile);
    cfg_infile = NULL;
  }

  return r;
}


// Adapted version of a neat empirical hash function from comp.lang.c by Daniel Bernstein
unsigned strhash(const char *str) {
  unsigned c, hash = 5381, n = 0;

  while((c = (unsigned char) *str++) && n++ < 20)
    hash = 33*hash ^ c;

  return hash;
}


static char **hstrings[1<<12];

// Return a copy of the argument as hashed string
const char *cache_string(const char *p) {
  int h, k;
  char **hs;

  if(!p)
    p = "(NULL)";

  h = strhash(p) % (sizeof hstrings/sizeof*hstrings);
  if(!(hs=hstrings[h]))
    hs = hstrings[h] = (char **) cfg_realloc("cache_string()", NULL, (16+1)*sizeof**hstrings);

  for(k=0; hs[k]; k++)
    if(*p == *hs[k] && !strcmp(p, hs[k]))
      return hs[k];

  if(k && k%16 == 0)
    hstrings[h] = (char **) cfg_realloc("cache_string()", hstrings[h], (k+16+1)*sizeof**hstrings);

  hstrings[h][k+1]=NULL;

  return hstrings[h][k] = cfg_strdup("cache_string()", p);
}


static LISTID cfg_comms;        // A chain of comment lines
static LISTID cfg_prologue;     // Comment lines at start of avrdude.conf
static char *lkw;               // Last seen keyword
static int lkw_lineno;          // Line number of that

static LISTID cfg_strctcomms;   // Passed on to config_gram.y
static LISTID cfg_pushedcomms;  // Temporarily pushed main comments
static int cfg_pushed;          // ... for memory sections

COMMENT *locate_comment(const LISTID comments, const char *where, int rhs) {
  if(comments)
    for(LNODEID ln=lfirst(comments); ln; ln=lnext(ln)) {
      COMMENT *n = ldata(ln);
      if(n && rhs == n->rhs && n->kw && strcmp(where, n->kw) == 0)
        return n;
    }

  return NULL;
}

static void addcomment(int rhs) {
  if(lkw) {
    COMMENT *node = cfg_malloc("addcomment()", sizeof(*node));
    node->rhs = rhs;
    node->kw = cfg_strdup("addcomment()", lkw);
    node->comms = cfg_comms;
    cfg_comms = NULL;
    if(!cfg_strctcomms)
      cfg_strctcomms = lcreat(NULL, 0);
    ladd(cfg_strctcomms, node);
  }
}

// Capture prologue during parsing (triggered by lexer.l)
void cfg_capture_prologue(void) {
  cfg_prologue = cfg_comms;
  cfg_comms = NULL;
}

LISTID cfg_get_prologue(void) {
  return cfg_prologue;
}

// Captures comments during parsing
void capture_comment_str(const char *com, int lineno) {
  if(!cfg_comms)
    cfg_comms = lcreat(NULL, 0);
  ladd(cfg_comms, cfg_strdup("capture_comment_str()", com));

  // Last keyword lineno is the same as this comment's
  if(lkw && lkw_lineno == lineno)
    addcomment(1);              // Register comms to show right of lkw = ...;
}

// Capture assignments (keywords left of =) and associate comments to them
void capture_lvalue_kw(const char *kw, int lineno) {
  if(!strcmp(kw, "memory")) {   // Push part comments and start memory comments
    if(!cfg_pushed) {           // config_gram.y pops the part comments
      cfg_pushed = 1;
      cfg_pushedcomms = cfg_strctcomms;
      cfg_strctcomms = NULL;
    }
  }

  if(!strcmp(kw, "programmer") || !strcmp(kw, "part") || !strcmp(kw, "memory"))
    kw = "*";                   // Show comment before programmer/part/memory

  if(lkw)
    free(lkw);
  lkw = cfg_strdup("capture_lvalue_kw()", kw);
  lkw_lineno = lineno;
  if(cfg_comms)                 // Accrued list of # one-line comments
    addcomment(0);              // Register comment to appear before lkw assignment
}

// config_gram.y calls this once for each programmer/part/memory structure
LISTID cfg_move_comments(void) {
  capture_lvalue_kw(";", -1);

  LISTID ret = cfg_strctcomms;
  cfg_strctcomms = NULL;
  return ret;
}

// config_gram.y calls this after ingressing the memory structure
void cfg_pop_comms(void) {
  if(cfg_pushed) {
    cfg_pushed = 0;
    cfg_strctcomms = cfg_pushedcomms;
  }
}

// Convert the next n hex digits of s to a hex number
static unsigned int tohex(const unsigned char *s, unsigned int n) {
  int ret, c;

  ret = 0;
  while(n--) {
    ret *= 16;
    c = *s++;
    ret += c >= '0' && c <= '9'? c - '0': c >= 'a' && c <= 'f'? c - 'a' + 10: c - 'A' + 10;
  }

  return ret;
}

/*
 * Create a utf-8 character sequence from a single unicode character.
 * Permissive for some invalid unicode sequences but not for those with
 * high bit set). Returns numbers of characters written (0-6).
 */
static int wc_to_utf8str(unsigned int wc, unsigned char *str) {
  if(!(wc & ~0x7fu)) {
    *str = (char) wc;
    return 1;
  }
  if(!(wc & ~0x7ffu)) {
    *str++ = (char) ((wc >> 6) | 0xc0);
    *str++ = (char) ((wc & 0x3f) | 0x80);
    return 2;
  }
  if(!(wc & ~0xffffu)) {
    *str++ = (char) ((wc >> 12) | 0xe0);
    *str++ = (char) (((wc >> 6) & 0x3f) | 0x80);
    *str++ = (char) ((wc & 0x3f) | 0x80);
    return 3;
  }
  if(!(wc & ~0x1fffffu)) {
    *str++ = (char) ((wc >> 18) | 0xf0);
    *str++ = (char) (((wc >> 12) & 0x3f) | 0x80);
    *str++ = (char) (((wc >> 6) & 0x3f) | 0x80);
    *str++ = (char) ((wc & 0x3f) | 0x80);
    return 4;
  }
  if(!(wc & ~0x3ffffffu)) {
    *str++ = (char) ((wc >> 24) | 0xf8);
    *str++ = (char) (((wc >> 18) & 0x3f) | 0x80);
    *str++ = (char) (((wc >> 12) & 0x3f) | 0x80);
    *str++ = (char) (((wc >> 6) & 0x3f) | 0x80);
    *str++ = (char) ((wc & 0x3f) | 0x80);
    return 5;
  }
  if(!(wc & ~0x7fffffffu)) {
    *str++ = (char) ((wc >> 30) | 0xfc);
    *str++ = (char) (((wc >> 24) & 0x3f) | 0x80);
    *str++ = (char) (((wc >> 18) & 0x3f) | 0x80);
    *str++ = (char) (((wc >> 12) & 0x3f) | 0x80);
    *str++ = (char) (((wc >> 6) & 0x3f) | 0x80);
    *str++ = (char) ((wc & 0x3f) | 0x80);
    return 6;
  }
  return 0;
}

// Unescape C-style strings, destination d must hold enough space (and can be source s)
unsigned char *cfg_unescapeu(unsigned char *d, const unsigned char *s) {
  unsigned char *ret = d;
  int n, k;

  while(*s) {
    switch (*s) {
    case '\\':
      switch (*++s) {
      case '\n':                // String continuation over new line
#if '\n' != '\r'
      case '\r':
#endif
        --d;
        break;
      case 'n':
        *d = '\n';
        break;
      case 't':
        *d = '\t';
        break;
      case 'a':
        *d = '\a';
        break;
      case 'b':
        *d = '\b';
        break;
      case 'e':                 // Non-standard ESC
        *d = 27;
        break;
      case 'f':
        *d = '\f';
        break;
      case 'r':
        *d = '\r';
        break;
      case 'v':
        *d = '\v';
        break;
      case '?':
        *d = '?';
        break;
      case '`':
        *d = '`';
        break;
      case '"':
        *d = '"';
        break;
      case '\'':
        *d = '\'';
        break;
      case '\\':
        *d = '\\';
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':                 // 1-3 octal digits
        n = *s - '0';
        for(k = 0; k < 2 && s[1] >= '0' && s[1] <= '7'; k++)  // Max 2 more octal characters
          n *= 8, n += s[1] - '0', s++;
        *d = n;
        break;
      case 'x':                 // Unlimited hex digits
        for(k = 0; isxdigit(s[k + 1]); k++)
          continue;
        if(k > 0) {
          *d = tohex(s + 1, k);
          s += k;
        } else {                // No hex digits after \x? copy \x
          *d++ = '\\';
          *d = 'x';
        }
        break;
      case 'u':                 // Exactly 4 hex digits and valid unicode
        if(isxdigit(s[1]) && isxdigit(s[2]) && isxdigit(s[3]) && isxdigit(s[4]) &&
          (n = wc_to_utf8str(tohex(s+1, 4), d))) {
          d += n - 1;
          s += 4;
        } else {                // Invalid \u sequence? copy \u
          *d++ = '\\';
          *d = 'u';
        }
        break;
      case 'U':                 // Exactly 6 hex digits and valid unicode
        if(isxdigit(s[1]) && isxdigit(s[2]) && isxdigit(s[3]) && isxdigit(s[4]) && isxdigit(s[5]) && isxdigit(s[6]) &&
          (n = wc_to_utf8str(tohex(s+1, 6), d))) {
          d += n - 1;
          s += 6;
        } else {                // Invalid \U sequence? copy \U
          *d++ = '\\';
          *d = 'U';
        }
        break;
      default:                  // Keep the escape sequence (C would warn and remove \)
        *d++ = '\\';
        *d = *s;
      }
      break;

    default:                    // Not an escape sequence: just copy the character
      *d = *s;
    }
    d++;
    s++;
  }
  *d = *s;                      // Terminate

  return ret;
}

// Unescape C-style strings, destination d must hold enough space (and can be source s)
char *cfg_unescape(char *d, const char *s) {
  return (char *) cfg_unescapeu((unsigned char *) d, (const unsigned char *) s);
}

// Return an escaped string that looks like a C-style input string incl quotes, memory is malloc'd
char *cfg_escape(const char *s) {
  char buf[50*1024], *d = buf;

  *d++ = '"';
  for(; *s && d-buf < sizeof buf-7; s++) {
    switch(*s) {
    case '\n':
      *d++ = '\\'; *d++ = 'n';
      break;
    case '\t':
      *d++ = '\\'; *d++ = 't';
      break;
    case '\a':
      *d++ = '\\'; *d++ = 'a';
      break;
    case '\b':
      *d++ = '\\'; *d++ = 'b';
      break;
    case '\f':
      *d++ = '\\'; *d++ = 'f';
      break;
#if '\r' != '\n'
    case '\r':
      *d++ = '\\'; *d++ = 'r';
      break;
#endif
    case '\v':
      *d++ = '\\'; *d++ = 'v';
      break;
    case '\"':
      *d++ = '\\'; *d++ = '\"';
      break;
    default:
      if(*s == 0x7f || (*s >= 0 && *s < 32)) {
        sprintf(d, "\\%03o", *s);
        d += strlen(d);
      } else
        *d++ = *s;
    }
  }
  *d++ = '"';
  *d = 0;

  return cfg_strdup("cfg_escape()", buf);
}


static int cmp_comp(const void *v1, const void *v2) {
  const Component_t *c1 = v1, *c2 = v2;
  int ret = strcmp(c1->name, c2->name);

  return ret? ret: c1->strct - c2->strct;
}

Component_t *cfg_comp_search(const char *name, int strct) {
  static int init;
  Component_t key;

  if(!init++)
    qsort(avr_comp, sizeof avr_comp/sizeof*avr_comp, sizeof(Component_t), cmp_comp);


  key.name = name;
  key.strct = strct;
  return bsearch(&key, avr_comp, sizeof avr_comp/sizeof*avr_comp, sizeof(Component_t), cmp_comp);
}


const char *cfg_strct_name(int strct) {
  switch(strct) {
  case COMP_CONFIG_MAIN: return "avrdude.conf main";
  case COMP_AVRPART: return "AVRPART";
  case COMP_AVRMEM: return "AVRMEM";
  case COMP_PROGRAMMER: return "PROGRAMMER";
  }
  return "unknown struct";
}

const char *cfg_v_type(int type) {
  switch(type) {
  case V_NONE: return "void";
  case V_NUM: return "number";
  case V_NUM_REAL: return "real";
  case V_STR: return "string";
  case V_COMPONENT: return "component";
  }
  return "unknown v type";
}

const char *cfg_comp_type(int type) {
  switch(type) {
  case COMP_INT: return "number";
  case COMP_SHORT: return "short";
  case COMP_CHAR: return "char";
  case COMP_STRING: return "string";
  case COMP_CHAR_ARRAY: return "byte array";
  case COMP_INT_LISTID: return "number list";
  case COMP_STRING_LISTID: return "string list";
  case COMP_OPCODE: return "opcode";
  case COMP_PIN: return "pin";
  case COMP_PIN_LIST: return "pin list";
  }
  return "unknown comp type";
}


// Used by config_gram.y to assign a component in one of the relevant structures with a value
void cfg_assign(char *sp, int strct, Component_t *cp, VALUE *v) {
  const char *str;
  int num;

  switch(cp->type) {
  case COMP_CHAR:
  case COMP_SHORT:
  case COMP_INT:
    if(v->type != V_NUM) {
      yywarning("%s in %s expects a %s but is assigned a %s",
        cp->name, cfg_strct_name(strct), cfg_comp_type(cp->type), cfg_v_type(v->type));
      return;
    }
    // TODO: consider endianess (code currently assumes little endian)
    num = v->number;
    memcpy(sp+cp->offset, &num, cp->size);
    break;
  case COMP_STRING:
    if(v->type != V_STR) {
      yywarning("%s in %s expects a string but is assigned a %s",
        cp->name, cfg_strct_name(strct), cfg_v_type(v->type));
      return;
    }
    str = cache_string(v->string);
    memcpy(sp+cp->offset, &str, cp->size);
    break;
  // TODO: implement COMP_CHAR_ARRAY, COMP_INT_LISTID, COMP_STRING_LISTID, ...
  default:
    yywarning("%s in %s expects a %s but that is not implemented",
      cp->name, cfg_strct_name(strct), cfg_comp_type(cp->type));
  }
}

// Automatically assign an mcuid if known from avrintel.c table
void cfg_update_mcuid(AVRPART *part) {
  // Don't assign an mcuid for template parts that has a space in desc
  if(!part->desc || *part->desc == 0 || strchr(part->desc, ' '))
    return;

  // Don't assign an mcuid for template parts where id starts with "."
  if(!part->id || !*part->id || *part->id == '.')
    return;

  // Don't assign an mcuid for 32-bit AVR parts
  if(part->prog_modes & PM_aWire)
    return;

  // Find an entry that shares the same name, overwrite mcuid with known, existing mcuid
  for(int i=0; i < sizeof uP_table/sizeof *uP_table; i++) {
    if(strcasecmp(part->desc, uP_table[i].name) == 0) {
      if(part->mcuid != (int) uP_table[i].mcuid) {
        if(part->mcuid >= 0 && verbose >= MSG_DEBUG)
          yywarning("overwriting mcuid of part %s to be %d", part->desc, uP_table[i].mcuid);
        part->mcuid = uP_table[i].mcuid;
      }
      return;
    }
  }

  // None have the same name: an entry with part->mcuid might be an error
  for(int i=0; i < sizeof uP_table/sizeof *uP_table; i++)
    if(part->mcuid == (int) uP_table[i].mcuid) {
      // Complain unless it can be considered a variant, eg, ATmega32L and ATmega32
      AVRMEM *flash = avr_locate_mem(part, "flash");
      if(flash) {
        size_t l1 = strlen(part->desc), l2 = strlen(uP_table[i].name);
        if(strncasecmp(part->desc, uP_table[i].name, l1 < l2? l1: l2) ||
            flash->size != uP_table[i].flashsize ||
            flash->page_size != uP_table[i].pagesize ||
            part->n_interrupts != uP_table[i].ninterrupts)
          yywarning("mcuid %d is reserved for %s, use a free number >= %d",
            part->mcuid, uP_table[i].name, sizeof uP_table/sizeof *uP_table);
      }
      return;
    }

  // Range check
  if(part->mcuid < 0 || part->mcuid >= UB_N_MCU)
    yywarning("mcuid %d for %s is out of range [0..%d], use a free number >= %d",
      part->mcuid, part->desc, UB_N_MCU-1, sizeof uP_table/sizeof *uP_table);
}
