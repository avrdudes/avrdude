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

%token K_READ
%token K_WRITE
%token K_READ_LO
%token K_READ_HI
%token K_WRITE_LO
%token K_WRITE_HI
%token K_LOADPAGE_LO
%token K_LOADPAGE_HI
%token K_WRITEPAGE
%token K_CHIP_ERASE
%token K_PGM_ENABLE

%token K_MEMORY

%token K_PAGE_SIZE
%token K_PAGED
%token K_BUFF
%token K_CHIP_ERASE_DELAY
%token K_DESC
%token K_EEPROM
%token K_ERRLED
%token K_FLASH
%token K_ID
%token K_LOADPAGE
%token K_MAX_WRITE_DELAY
%token K_MIN_WRITE_DELAY
%token K_MISO
%token K_MOSI
%token K_NUM_PAGES
%token K_PART
%token K_PGMLED
%token K_PROGRAMMER
%token K_RDYLED
%token K_READBACK_P1
%token K_READBACK_P2
%token K_READMEM
%token K_RESET
%token K_SCK
%token K_SIZE
%token K_VCC
%token K_VFYLED
%token K_WRITEPAGE

%token K_NO
%token K_YES

%token TKN_COMMA
%token TKN_EQUAL
%token TKN_SEMI
%token TKN_NUMBER
%token TKN_STRING
%token TKN_ID

%start config

%%


config :
  def |
  config def
;


def :
  prog_def TKN_SEMI |
  part_def TKN_SEMI
;


prog_def :
  K_PROGRAMMER 
    { current_prog = new_programmer(); }
    prog_parms
    { 
      if (lsize(current_prog->id) == 0) {
        fprintf(stderr,
                "%s: error at %s:%d: required parameter id not specified\n",
                progname, infile, lineno);
        exit(1);
      }
      ladd(programmers, current_prog); 
      current_prog = NULL; 
    }
;


part_def :
  K_PART
    { current_part = avr_new_part(); }
    part_parms 
    { 
      LNODEID ln;
      AVRMEM * m;

      if (current_part->id[0] == 0) {
        fprintf(stderr,
                "%s: error at %s:%d: required parameter id not specified\n",
                progname, infile, lineno);
        exit(1);
      }

      /*
       * perform some sanity checking, and compute the number of bits
       * to shift a page for constructing the page address for
       * page-addressed memories.
       */
      for (ln=lfirst(current_part->mem); ln; ln=lnext(ln)) {
        m = ldata(ln);
        if (m->paged) {
          if (m->page_size == 0) {
            fprintf(stderr, 
                    "%s: error at %s:%d: must specify page_size for paged "
                    "memory\n",
                    progname, infile, lineno);
            exit(1);
          }
          if (m->num_pages == 0) {
            fprintf(stderr, 
                    "%s: error at %s:%d: must specify num_pages for paged "
                    "memory\n",
                    progname, infile, lineno);
            exit(1);
          }
          if (m->size != m->page_size * m->num_pages) {
            fprintf(stderr, 
                    "%s: error at %s:%d: page size (%u) * num_pages (%u) = "
                    "%u does not match memory size (%u)\n",
                    progname, infile, lineno,
                    m->page_size, 
                    m->num_pages, 
                    m->page_size * m->num_pages,
                    m->size);
            exit(1);
          }

        }
      }

      ladd(part_list, current_part); 
      current_part = NULL; 
    }
;


string_list :
  TKN_STRING { ladd(string_list, $1); } |
  string_list TKN_COMMA TKN_STRING { ladd(string_list, $3); }
;


num_list :
  TKN_NUMBER { ladd(number_list, $1); } |
  num_list TKN_COMMA TKN_NUMBER { ladd(number_list, $3); }
;


prog_parms :
  prog_parm TKN_SEMI |
  prog_parms prog_parm TKN_SEMI
;


prog_parm :
  K_ID TKN_EQUAL string_list {
    { 
      TOKEN * t;
      while (lsize(string_list)) {
        t = lrmv_n(string_list, 1);
        ladd(current_prog->id, dup_string(t->value.string));
        free_token(t);
      }
    }
  } |

  K_DESC TKN_EQUAL TKN_STRING {
    strncpy(current_prog->desc, $3->value.string, PGM_DESCLEN);
    current_prog->desc[PGM_DESCLEN-1] = 0;
    free_token($3);
  } |

  K_VCC TKN_EQUAL num_list {
    { 
      TOKEN * t;
      int pin;

      current_prog->pinno[PPI_AVR_VCC] = 0;

      while (lsize(number_list)) {
        t = lrmv_n(number_list, 1);
        pin = t->value.number;
        if ((pin < 2) || (pin > 9)) {
          fprintf(stderr, 
                  "%s: error at line %d of %s: VCC must be one or more "
                  "pins from the range 2-9\n",
                  progname, lineno, infile);
          exit(1);
        }

        current_prog->pinno[PPI_AVR_VCC] |= (1 << (pin-2));

        free_token(t);
      }
    }
  } |

  K_BUFF TKN_EQUAL num_list {
    { 
      TOKEN * t;
      int pin;

      current_prog->pinno[PPI_AVR_BUFF] = 0;

      while (lsize(number_list)) {
        t = lrmv_n(number_list, 1);
        pin = t->value.number;
        if ((pin < 2) || (pin > 9)) {
          fprintf(stderr, 
                  "%s: error at line %d of %s: BUFF must be one or more "
                  "pins from the range 2-9\n",
                  progname, lineno, infile);
          exit(1);
        }

        current_prog->pinno[PPI_AVR_BUFF] |= (1 << (pin-2));

        free_token(t);
      }
    }
  } |

  K_RESET  TKN_EQUAL TKN_NUMBER { assign_pin(PIN_AVR_RESET, $3); } |
  K_SCK    TKN_EQUAL TKN_NUMBER { assign_pin(PIN_AVR_SCK, $3); } |
  K_MOSI   TKN_EQUAL TKN_NUMBER { assign_pin(PIN_AVR_MOSI, $3); } |
  K_MISO   TKN_EQUAL TKN_NUMBER { assign_pin(PIN_AVR_MISO, $3); } |
  K_ERRLED TKN_EQUAL TKN_NUMBER { assign_pin(PIN_LED_ERR, $3); } |
  K_RDYLED TKN_EQUAL TKN_NUMBER { assign_pin(PIN_LED_RDY, $3); } |
  K_PGMLED TKN_EQUAL TKN_NUMBER { assign_pin(PIN_LED_PGM, $3); } |
  K_VFYLED TKN_EQUAL TKN_NUMBER { assign_pin(PIN_LED_VFY, $3); }
;


opcode :
  K_READ         |
  K_WRITE        |
  K_READ_LO      |
  K_READ_HI      |
  K_WRITE_LO     |
  K_WRITE_HI     |
  K_LOADPAGE_LO  |
  K_LOADPAGE_HI  |
  K_WRITEPAGE    |
  K_CHIP_ERASE   |
  K_PGM_ENABLE
;


part_parms :
  part_parm TKN_SEMI |
  part_parms part_parm TKN_SEMI
;


part_parm :
  K_ID TKN_EQUAL TKN_STRING 
    {
      strncpy(current_part->id, $3->value.string, AVR_IDLEN);
      current_part->id[AVR_IDLEN-1] = 0;
      free_token($3);
    } |

  K_DESC TKN_EQUAL TKN_STRING 
    {
      strncpy(current_part->desc, $3->value.string, AVR_DESCLEN);
      current_part->desc[AVR_DESCLEN-1] = 0;
      free_token($3);
    } |

  K_CHIP_ERASE_DELAY TKN_EQUAL TKN_NUMBER
    {
      current_part->chip_erase_delay = $3->value.number;
      free_token($3);
    } |

/*
  K_EEPROM { current_mem = AVR_M_EEPROM; }
    mem_specs |

  K_FLASH { current_mem = AVR_M_FLASH; }
    mem_specs | 
*/

  K_MEMORY TKN_STRING 
    { 
      current_mem = avr_new_memtype(); 
      strcpy(current_mem->desc, strdup($2->value.string)); 
      free_token($2); 
    } 
    mem_specs 
    { 
      ladd(current_part->mem, current_mem); 
      current_mem = NULL; 
    } |

  opcode TKN_EQUAL string_list {
    { 
      int opnum;
      OPCODE * op;

      opnum = which_opcode($1);
      op = avr_new_opcode();
      parse_cmdbits(op);
      current_part->op[opnum] = op;

      free_token($1);
    }
  }
;


yesno :
  K_YES | K_NO
;


mem_specs :
  mem_spec TKN_SEMI |
  mem_specs mem_spec TKN_SEMI
;


mem_spec :
  K_PAGED          TKN_EQUAL yesno
    {
      current_mem->paged = $3->primary == K_YES ? 1 : 0;
      free_token($3);
    } |

  K_SIZE            TKN_EQUAL TKN_NUMBER
    {
      current_mem->size = $3->value.number;
      free_token($3);
    } |


  K_PAGE_SIZE       TKN_EQUAL TKN_NUMBER
    {
      current_mem->page_size = $3->value.number;
      free_token($3);
    } |

  K_NUM_PAGES       TKN_EQUAL TKN_NUMBER
    {
      current_mem->num_pages = $3->value.number;
      free_token($3);
    } |

  K_MIN_WRITE_DELAY TKN_EQUAL TKN_NUMBER
    {
      current_mem->min_write_delay = $3->value.number;
      free_token($3);
    } |

  K_MAX_WRITE_DELAY TKN_EQUAL TKN_NUMBER
    {
      current_mem->max_write_delay = $3->value.number;
      free_token($3);
    } |

  K_READBACK_P1     TKN_EQUAL TKN_NUMBER
    {
      current_mem->readback[0] = $3->value.number;
      free_token($3);
    } |

  K_READBACK_P2     TKN_EQUAL TKN_NUMBER
    {
      current_mem->readback[1] = $3->value.number;
      free_token($3);
    } |

  opcode TKN_EQUAL string_list {
    { 
      int opnum;
      OPCODE * op;

      opnum = which_opcode($1);
      op = avr_new_opcode();
      parse_cmdbits(op);
      current_mem->op[opnum] = op;

      free_token($1);
    }
  }
;


%%

#include <string.h>
#include <math.h>

#include "config.h"
#include "lists.h"
#include "pindefs.h"
#include "avr.h"

extern char * progname;

int yylex(void);
int yyerror(char * errmsg);


#if 0
static char * vtypestr(int type)
{
  switch (type) {
    case V_NUM : return "NUMERIC";
    case V_STR : return "STRING";
    default:
      return "<UNKNOWN>";
  }
}
#endif


static int assign_pin(int pinno, TOKEN * v)
{
  int value;

  value = v->value.number;

  if ((value <= 0) || (value >= 18)) {
    fprintf(stderr, 
            "%s: error at line %d of %s: pin must be in the "
            "range 1-17\n",
            progname, lineno, infile);
    exit(1);
  }

  current_prog->pinno[pinno] = value;

  return 0;
}


static int which_opcode(TOKEN * opcode)
{
  switch (opcode->primary) {
    case K_READ        : return AVR_OP_READ; break;
    case K_WRITE       : return AVR_OP_WRITE; break;
    case K_READ_LO     : return AVR_OP_READ_LO; break;
    case K_READ_HI     : return AVR_OP_READ_HI; break;
    case K_WRITE_LO    : return AVR_OP_WRITE_LO; break;
    case K_WRITE_HI    : return AVR_OP_WRITE_HI; break;
    case K_LOADPAGE_LO : return AVR_OP_LOADPAGE_LO; break;
    case K_LOADPAGE_HI : return AVR_OP_LOADPAGE_HI; break;
    case K_WRITEPAGE   : return AVR_OP_WRITEPAGE; break;
    case K_CHIP_ERASE  : return AVR_OP_CHIP_ERASE; break;
    case K_PGM_ENABLE  : return AVR_OP_PGM_ENABLE; break;
    default :
      fprintf(stderr, 
              "%s: error at %s:%d: invalid opcode\n",
              progname, infile, lineno);
      exit(1);
      break;
  }
}


static int parse_cmdbits(OPCODE * op)
{
  TOKEN * t;
  int bitno;
  char ch;
  char * e;
  char * q;
  int len;
  char * s, *brkt;

  bitno = 32;
  while (lsize(string_list)) {

    t = lrmv_n(string_list, 1);

    s = strtok_r(t->value.string, " ", &brkt);
    while (s != NULL) {

      bitno--;
      if (bitno < 0) {
        fprintf(stderr, 
                "%s: error at %s:%d: too many opcode bits for instruction\n",
                progname, infile, lineno);
        exit(1);
      }

      len = strlen(s);

      if (len == 0) {
        fprintf(stderr, 
                "%s: error at %s:%d: invalid bit specifier \"\"\n",
                progname, infile, lineno);
        exit(1);
      }

      ch = s[0];

      if (len == 1) {
        switch (ch) {
          case '1':
            op->bit[bitno].type  = AVR_CMDBIT_VALUE;
            op->bit[bitno].value = 1;
            op->bit[bitno].bitno = bitno % 8;
            break;
          case '0':
            op->bit[bitno].type  = AVR_CMDBIT_VALUE;
            op->bit[bitno].value = 0;
            op->bit[bitno].bitno = bitno % 8;
            break;
          case 'x':
            op->bit[bitno].type  = AVR_CMDBIT_IGNORE;
            op->bit[bitno].value = 0;
            op->bit[bitno].bitno = bitno % 8;
            break;
          case 'a':
            op->bit[bitno].type  = AVR_CMDBIT_ADDRESS;
            op->bit[bitno].value = 0;
            op->bit[bitno].bitno = 8*(bitno/8) + bitno % 8;
            break;
          case 'i':
            op->bit[bitno].type  = AVR_CMDBIT_INPUT;
            op->bit[bitno].value = 0;
            op->bit[bitno].bitno = bitno % 8;
            break;
          case 'o':
            op->bit[bitno].type  = AVR_CMDBIT_OUTPUT;
            op->bit[bitno].value = 0;
            op->bit[bitno].bitno = bitno % 8;
            break;
          default :
            fprintf(stderr, 
                    "%s: error at %s:%d: invalid bit specifier '%c'\n",
                    progname, infile, lineno, ch);
            exit(1);
            break;
        }
      }
      else {
        if (ch == 'a') {
          q = &s[1];
          op->bit[bitno].bitno = strtol(q, &e, 0);
          if ((e == q)||(*e != 0)) {
            fprintf(stderr, 
                    "%s: error at %s:%d: can't parse bit number from \"%s\"\n",
                    progname, infile, lineno, q);
            exit(1);
          }
          op->bit[bitno].type = AVR_CMDBIT_ADDRESS;
          op->bit[bitno].value = 0;
        }
        else {
          fprintf(stderr, 
                  "%s: error at %s:%d: invalid bit specifier \"%s\"\n",
                  progname, infile, lineno, s);
          exit(1);
        }
      }

      s = strtok_r(NULL, " ", &brkt);
    }

    free_token(t);

  }  /* while */

  return 0;
}


