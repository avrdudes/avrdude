/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
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
%{

#include "ac_cfg.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "config.h"
#include "developer_opts.h"

#if defined(WIN32)
#define strtok_r( _s, _sep, _lasts ) \
    ( *(_lasts) = strtok( (_s), (_sep) ) )
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

int yylex(void);
int yyerror(char * errmsg, ...);
int yywarning(char * errmsg, ...);

static int clear_pin(int pinfunc);
static int assign_pin(int pinfunc, TOKEN *v, int invert);
static int assign_pin_list(int invert);
static int which_opcode(TOKEN * opcode);
static int parse_cmdbits(OPCODE * op, int opnum);

static int pin_name;
%}

%token K_NULL;

%token K_READ
%token K_WRITE
%token K_READ_LO
%token K_READ_HI
%token K_WRITE_LO
%token K_WRITE_HI
%token K_LOADPAGE_LO
%token K_LOADPAGE_HI
%token K_LOAD_EXT_ADDR
%token K_WRITEPAGE
%token K_CHIP_ERASE
%token K_PGM_ENABLE

%token K_MEMORY

%token K_PAGE_SIZE

%token K_ALIAS
%token K_ALLOW_SUBSHELLS
%token K_BUFF
%token K_CONNTYPE
%token K_DEDICATED
%token K_DEFAULT_BITCLOCK
%token K_DEFAULT_PARALLEL
%token K_DEFAULT_PROGRAMMER
%token K_DEFAULT_SERIAL
%token K_DEFAULT_SPI
%token K_DEFAULT_LINUXGPIO
%token K_HVUPDI_SUPPORT
%token K_DEVICECODE
%token K_EEPROM
%token K_ERRLED
%token K_FLASH
%token K_ID
%token K_IO
%token K_LINUXGPIO
%token K_LOADPAGE
%token K_SDI
%token K_SDO
%token K_PARALLEL
%token K_PARENT
%token K_PART
%token K_PGMLED
%token K_PROGRAMMER
%token K_RDYLED
%token K_READBACK
%token K_READMEM
%token K_RESET
%token K_RETRY_PULSE
%token K_SERIAL
%token K_SPI
%token K_SCK
%token K_SIGNATURE
%token K_SIZE
%token K_TCK
%token K_TDI
%token K_TDO
%token K_TMS
%token K_USB
%token K_USBPID
%token K_TYPE
%token K_VARIANTS
%token K_VCC
%token K_VFYLED

/* stk500 v2 xml file parameters */
/* ISP */

%token K_PP_CONTROLSTACK
%token K_HVSP_CONTROLSTACK

/* JTAG ICE mkII specific parameters */
%token K_ALLOWFULLPAGEBITSTREAM	/*
				 * Internal parameter for the JTAG
				 * ICE; describes the internal JTAG
				 * streaming behaviour inside the MCU.
				 * 1 for all older chips, 0 for newer
				 * MCUs.
				 */
%token K_ENABLEPAGEPROGRAMMING	/* ? yes for mega256*, mega406 */
%token K_HAS_JTAG		/* MCU has JTAG i/f. */
%token K_HAS_DW			/* MCU has debugWire i/f. */
%token K_HAS_PDI                /* MCU has PDI i/f rather than ISP (ATxmega). */
%token K_HAS_UPDI               /* MCU has UPDI i/f (AVR8X). */
%token K_HAS_TPI                /* MCU has TPI i/f rather than ISP (ATtiny4/5/9/10). */
%token K_IS_AT90S1200		/* chip is an AT90S1200 (needs special treatment) */
%token K_IS_AVR32               /* chip is in the avr32 family */
%token K_FLASH_INSTR		/* flash instructions */
%token K_EEPROM_INSTR		/* EEPROM instructions */

%token TKN_COMMA
%token TKN_EQUAL
%token TKN_SEMI
%token TKN_LEFT_PAREN
%token TKN_RIGHT_PAREN
%token TKN_NUMBER
%token TKN_NUMBER_REAL
%token TKN_STRING
%token TKN_COMPONENT

%left  OP_OR                    /* calculator operations */
%left  OP_XOR
%left  OP_AND
%left  OP_PLUS OP_MINUS
%left  OP_TIMES OP_DIVIDE OP_MODULO
%right OP_TILDE UNARY

%start configuration

%%

number_real : 
 numexpr {
    $$ = $1;
    /* convert value to real */
    $$->value.number_real = $$->value.number;
    $$->value.type = V_NUM_REAL;
  } |
  TKN_NUMBER_REAL {
    $$ = $1;
  }
;


expr: numexpr | TKN_STRING;

numexpr:
  TKN_NUMBER |
  numexpr OP_OR numexpr     { $$ = $1; $$->value.number |= $3->value.number; } |
  numexpr OP_XOR numexpr    { $$ = $1; $$->value.number ^= $3->value.number; } |
  numexpr OP_AND numexpr    { $$ = $1; $$->value.number &= $3->value.number; } |
  numexpr OP_PLUS numexpr   { $$ = $1; $$->value.number += $3->value.number; } |
  numexpr OP_MINUS numexpr  { $$ = $1; $$->value.number -= $3->value.number; } |
  numexpr OP_TIMES numexpr  { $$ = $1; $$->value.number *= $3->value.number; } |
  numexpr OP_DIVIDE numexpr { $$ = $1; $$->value.number /= $3->value.number; } |
  numexpr OP_MODULO numexpr { $$ = $1; $$->value.number %= $3->value.number; } |
  OP_PLUS numexpr %prec UNARY  { $$ = $2; } |
  OP_MINUS numexpr %prec UNARY { $$ = $2; $$->value.number = -$$->value.number; } |
  OP_TILDE numexpr %prec UNARY { $$ = $2; $$->value.number = ~$$->value.number; } |
  TKN_LEFT_PAREN numexpr TKN_RIGHT_PAREN { $$ = $2; }
;


configuration :
  /* empty */ | config
;

config :
  def |
  config def
;


def :
  prog_def TKN_SEMI |

  part_def TKN_SEMI |

  K_DEFAULT_PROGRAMMER TKN_EQUAL TKN_STRING TKN_SEMI {
    default_programmer = cache_string($3->value.string);
    free_token($3);
  } |

  K_DEFAULT_PARALLEL TKN_EQUAL TKN_STRING TKN_SEMI {
    default_parallel = cache_string($3->value.string);
    free_token($3);
  } |

  K_DEFAULT_SERIAL TKN_EQUAL TKN_STRING TKN_SEMI {
    default_serial = cache_string($3->value.string);
    free_token($3);
  } |

  K_DEFAULT_SPI TKN_EQUAL TKN_STRING TKN_SEMI {
    default_spi = cache_string($3->value.string);
    free_token($3);
  } |

  K_DEFAULT_BITCLOCK TKN_EQUAL number_real TKN_SEMI {
    default_bitclock = $3->value.number_real;
    free_token($3);
  } |

  K_ALLOW_SUBSHELLS TKN_EQUAL numexpr TKN_SEMI {
    allow_subshells = $3->value.number;
    free_token($3);
  } |

  K_DEFAULT_LINUXGPIO TKN_EQUAL TKN_STRING TKN_SEMI {
    default_linuxgpio = cache_string($3->value.string);
    free_token($3);
  }
;


prog_def :
  prog_decl prog_parms
    {
      PROGRAMMER * existing_prog;
      if (lsize(current_prog->id) == 0) {
        yyerror("required parameter id not specified");
        YYABORT;
      }
      if (current_prog->initpgm == NULL && current_prog->prog_modes) {
        yyerror("programmer type not specified");
        YYABORT;
      }
      for(LNODEID ln = lfirst(current_prog->id); ln; ln = lnext(ln)) {
        char *id = ldata(ln);
        if((existing_prog = locate_programmer(programmers, id))) {
          // Temporarily set lineno to lineno of programmer start
          int temp = cfg_lineno; cfg_lineno = current_prog->lineno;
          yywarning("programmer %s overwrites previous definition %s:%d.",
            id, existing_prog->config_file, existing_prog->lineno);
          cfg_lineno = temp;
          lrmv_d(programmers, existing_prog);
          pgm_free(existing_prog);
        }
      }
      current_prog->comments = cfg_move_comments();
      LISTADD(programmers, current_prog);
//      pgm_fill_old_pins(current_prog); // TODO to be removed if old pin data no longer needed
//      pgm_display_generic(current_prog, id);
      current_prog = NULL;
      current_strct = COMP_CONFIG_MAIN;
    }
;


prog_decl :
  K_PROGRAMMER
    { current_prog = pgm_new();
      current_prog->config_file = cache_string(cfg_infile);
      current_prog->lineno = cfg_lineno;
    }
    |
  K_PROGRAMMER K_PARENT TKN_STRING
    {
      PROGRAMMER * pgm = locate_programmer(programmers, $3->value.string);
      if (pgm == NULL) {
        yyerror("parent programmer %s not found", $3->value.string);
        free_token($3);
        YYABORT;
      }
      current_prog = pgm_dup(pgm);
      current_prog->parent_id = cache_string($3->value.string);
      current_prog->comments = NULL;
      current_prog->config_file = cache_string(cfg_infile);
      current_prog->lineno = cfg_lineno;
      free_token($3);
    }
;


part_def :
  part_decl part_parms 
    { 
      LNODEID ln;
      AVRMEM * m;
      AVRPART * existing_part;

      if (current_part->id[0] == 0) {
        yyerror("required parameter id not specified");
        YYABORT;
      }

      cfg_update_mcuid(current_part);

      // Sanity checks for memory sizes and compute/override num_pages entry
      for (ln=lfirst(current_part->mem); ln; ln=lnext(ln)) {
        m = ldata(ln);
        if (m->paged) {
          if (m->size <= 0) {
            yyerror("must specify a positive size for paged memory %s", m->desc);
            YYABORT;
          }
          if (m->page_size <= 0) {
            yyerror("must specify a positive page size for paged memory %s", m->desc);
            YYABORT;
          }
          // Code base relies on page_size being a power of 2 in some places
          if (m->page_size & (m->page_size - 1)) {
            yyerror("page size must be a power of 2 for paged memory %s", m->desc);
            YYABORT;
          }
          // Code base relies on size being a multiple of page_size
          if (m->size % m->page_size) {
            yyerror("size must be a multiple of page size for paged memory %s", m->desc);
            YYABORT;
          }
          // Warn if num_pages was specified but is inconsistent with size and page size
          if (m->num_pages && m->num_pages != m->size / m->page_size)
            yywarning("overriding num_page to be %d for memory %s", m->size/m->page_size, m->desc);

          m->num_pages = m->size / m->page_size;
        }
      }

      existing_part = locate_part(part_list, current_part->id);
      if (existing_part) {
        { /* temporarily set lineno to lineno of part start */
          int temp = cfg_lineno; cfg_lineno = current_part->lineno;
          yywarning("part %s overwrites previous definition %s:%d.",
                current_part->id,
                existing_part->config_file, existing_part->lineno);
          cfg_lineno = temp;
        }
        lrmv_d(part_list, existing_part);
        avr_free_part(existing_part);
      }

      current_part->comments = cfg_move_comments();
      LISTADD(part_list, current_part); 
      current_part = NULL; 
      current_strct = COMP_CONFIG_MAIN;
    }
;

part_decl :
  K_PART
    {
      current_part = avr_new_part();
      current_part->config_file = cache_string(cfg_infile);
      current_part->lineno = cfg_lineno;
    } |
  K_PART K_PARENT TKN_STRING 
    {
      AVRPART * parent_part = locate_part(part_list, $3->value.string);
      if (parent_part == NULL) {
        yyerror("can't find parent part");
        free_token($3);
        YYABORT;
      }

      current_part = avr_dup_part(parent_part);
      current_part->parent_id = cache_string($3->value.string);
      current_part->comments = NULL;
      current_part->config_file = cache_string(cfg_infile);
      current_part->lineno = cfg_lineno;

      free_token($3);
    }
;

string_list :
  TKN_STRING { ladd(string_list, $1); } |
  string_list TKN_COMMA TKN_STRING { ladd(string_list, $3); }
;


num_list :
  numexpr { ladd(number_list, $1); } |
  num_list TKN_COMMA numexpr { ladd(number_list, $3); }
;

prog_parms :
  prog_parm TKN_SEMI |
  prog_parms prog_parm TKN_SEMI
;

prog_parm :
  TKN_COMPONENT TKN_EQUAL expr {
    cfg_assign((char *) current_prog, COMP_PROGRAMMER, $1->value.comp, &$3->value);
    free_token($1);
  } |
  K_ID TKN_EQUAL string_list {
    {
      while (lsize(string_list)) {
        TOKEN *t = lrmv_n(string_list, 1);
        ladd(current_prog->id, cfg_strdup("config_gram.y", t->value.string));
        free_token(t);
      }
    }
  } |
  prog_parm_type
  |
  prog_parm_pins
  |
  K_USBPID TKN_EQUAL usb_pid_list
  |
  prog_parm_conntype
  |
  prog_parm_updi
;

prog_parm_type:
  K_TYPE TKN_EQUAL prog_parm_type_id
;

prog_parm_type_id:
  TKN_STRING        {
  const struct programmer_type_t * pgm_type = locate_programmer_type($1->value.string);
    if (pgm_type == NULL) {
        yyerror("programmer type %s not found", $1->value.string);
        free_token($1); 
        YYABORT;
    }
    current_prog->initpgm = pgm_type->initpgm;
    free_token($1); 
}
  | error
{
        yyerror("programmer type must be written as \"id_type\"");
        YYABORT;
}
;

prog_parm_conntype:
  K_CONNTYPE TKN_EQUAL prog_parm_conntype_id
;

prog_parm_conntype_id:
  K_LINUXGPIO       { current_prog->conntype = CONNTYPE_LINUXGPIO; } |
  K_PARALLEL        { current_prog->conntype = CONNTYPE_PARALLEL; } |
  K_SERIAL          { current_prog->conntype = CONNTYPE_SERIAL; } |
  K_USB             { current_prog->conntype = CONNTYPE_USB; } |
  K_SPI             { current_prog->conntype = CONNTYPE_SPI; }
;

usb_pid_list:
  numexpr {
    {
      /* overwrite pids, so clear the existing entries */
      if(current_prog->usbpid)
        ldestroy_cb(current_prog->usbpid, free);
      current_prog->usbpid = lcreat(NULL, 0);
    }
    {
      int *ip = cfg_malloc("usb_pid_list", sizeof(int));
      *ip = $1->value.number;
      ladd(current_prog->usbpid, ip);
      free_token($1);
    }
  } |
  usb_pid_list TKN_COMMA numexpr {
    {
      int *ip = cfg_malloc("usb_pid_list", sizeof(int));
      *ip = $3->value.number;
      ladd(current_prog->usbpid, ip);
      free_token($3);
    }
  }
;

prog_parm_updi:
  K_HVUPDI_SUPPORT TKN_EQUAL hvupdi_support_list
;

hvupdi_support_list:
  numexpr {
    {
      /* overwrite list entries, so clear the existing entries */
      if(current_prog->hvupdi_support)
        ldestroy_cb(current_prog->hvupdi_support, free);
      current_prog->hvupdi_support = lcreat(NULL, 0);
    }
    {
      int *ip = cfg_malloc("hvupdi_support_list", sizeof(int));
      *ip = $1->value.number;
      ladd(current_prog->hvupdi_support, ip);
      free_token($1);
    }
  } |
  hvupdi_support_list TKN_COMMA numexpr {
    {
      int *ip = cfg_malloc("hvupdi_support_list", sizeof(int));
      *ip = $3->value.number;
      ladd(current_prog->hvupdi_support, ip);
      free_token($3);
    }
  }
;

pin_number_non_empty:
  TKN_NUMBER { if(0 != assign_pin(pin_name, $1, 0)) YYABORT;  }
  |
  OP_TILDE TKN_NUMBER { if(0 != assign_pin(pin_name, $2, 1)) YYABORT; }
;

pin_number:
  pin_number_non_empty
  |
  /* empty */ { pin_clear_all(&(current_prog->pin[pin_name])); }
;

pin_list_element:
  pin_number_non_empty
  |
  OP_TILDE TKN_LEFT_PAREN num_list TKN_RIGHT_PAREN { if(0 != assign_pin_list(1)) YYABORT; }
;

pin_list_non_empty:
  pin_list_element
  |
  pin_list_non_empty TKN_COMMA pin_list_element
;


pin_list:
  pin_list_non_empty
  |
  /* empty */ { pin_clear_all(&(current_prog->pin[pin_name])); }
;

prog_parm_pins:
  K_VCC    TKN_EQUAL {pin_name = PPI_AVR_VCC; clear_pin(pin_name);  } pin_list |
  K_BUFF   TKN_EQUAL {pin_name = PPI_AVR_BUFF; clear_pin(pin_name); } pin_list |
  K_RESET  TKN_EQUAL {pin_name = PIN_AVR_RESET; clear_pin(pin_name);} pin_number { free_token($1); } |
  K_SCK    TKN_EQUAL {pin_name = PIN_AVR_SCK; clear_pin(pin_name);  } pin_number { free_token($1); } |
  K_SDO    TKN_EQUAL {pin_name = PIN_AVR_SDO; clear_pin(pin_name);  } pin_number |
  K_SDI    TKN_EQUAL {pin_name = PIN_AVR_SDI; clear_pin(pin_name);  } pin_number |
  K_TCK    TKN_EQUAL {pin_name = PIN_JTAG_TCK; clear_pin(pin_name); } pin_number |
  K_TDI    TKN_EQUAL {pin_name = PIN_JTAG_TDI; clear_pin(pin_name); } pin_number |
  K_TDO    TKN_EQUAL {pin_name = PIN_JTAG_TDO; clear_pin(pin_name); } pin_number |
  K_TMS    TKN_EQUAL {pin_name = PIN_JTAG_TMS; clear_pin(pin_name); } pin_number |
  K_ERRLED TKN_EQUAL {pin_name = PIN_LED_ERR; clear_pin(pin_name);  } pin_number |
  K_RDYLED TKN_EQUAL {pin_name = PIN_LED_RDY; clear_pin(pin_name);  } pin_number |
  K_PGMLED TKN_EQUAL {pin_name = PIN_LED_PGM; clear_pin(pin_name);  } pin_number |
  K_VFYLED TKN_EQUAL {pin_name = PIN_LED_VFY; clear_pin(pin_name);  } pin_number
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
  K_LOAD_EXT_ADDR |
  K_WRITEPAGE    |
  K_CHIP_ERASE   |
  K_PGM_ENABLE
;


part_parms :
  part_parm TKN_SEMI |
  part_parms part_parm TKN_SEMI
;


reset_disposition :
  K_DEDICATED | K_IO
;

retry_lines :
  K_RESET | K_SCK
;

part_parm :
  TKN_COMPONENT TKN_EQUAL expr {
    cfg_assign((char *) current_part, COMP_AVRPART, $1->value.comp, &$3->value);
    free_token($1);
  } |
  K_ID TKN_EQUAL TKN_STRING 
    {
      current_part->id = cache_string($3->value.string);
      free_token($3);
    } |

  K_VARIANTS TKN_EQUAL K_NULL {
    {
      ldestroy_cb(current_part->variants, free);
      current_part->variants = lcreat(NULL, 0);
    }
  } |

  K_VARIANTS TKN_EQUAL string_list {
    {
      while (lsize(string_list)) {
        TOKEN *t = lrmv_n(string_list, 1);
        int found = 0;
        for(LNODEID ln = lfirst(current_part->variants); ln; ln = lnext(ln)) {
          if(str_eq((char *) ldata(ln), t->value.string)) {
            found = 1;
            break;
          }
        }
        if(!found)
          ladd(current_part->variants, cfg_strdup("config_gram.y", t->value.string));
        free_token(t);
      }
    }
  } |

  K_DEVICECODE TKN_EQUAL numexpr {
    {
      yyerror("devicecode is deprecated, will be removed in v8.0, use stk500_devcode instead");
      YYABORT;
    }
  } |

  K_SIGNATURE TKN_EQUAL TKN_NUMBER TKN_NUMBER TKN_NUMBER {
    {
      current_part->signature[0] = $3->value.number;
      current_part->signature[1] = $4->value.number;
      current_part->signature[2] = $5->value.number;
      free_token($3);
      free_token($4);
      free_token($5);
    }
  } |

 K_USBPID TKN_EQUAL numexpr {
    {
      current_part->usbpid = $3->value.number;
      free_token($3);
    }
  } |

  K_PP_CONTROLSTACK TKN_EQUAL num_list {
    {
      TOKEN * t;
      unsigned nbytes;
      int ok;

      current_part->ctl_stack_type = CTL_STACK_PP;
      nbytes = 0;
      ok = 1;

      memset(current_part->controlstack, 0, CTL_STACK_SIZE);
      while (lsize(number_list)) {
        t = lrmv_n(number_list, 1);
	if (nbytes < CTL_STACK_SIZE)
	  {
	    current_part->controlstack[nbytes] = t->value.number;
	    nbytes++;
	  }
	else
	  {
	    ok = 0;
	  }
        free_token(t);
      }
      if (!ok)
	{
	  yywarning("too many bytes in control stack");
        }
    }
  } |

  K_PP_CONTROLSTACK TKN_EQUAL K_NULL {
    {
      current_part->ctl_stack_type = CTL_STACK_NONE;
      memset(current_part->controlstack, 0, CTL_STACK_SIZE);
    }
  } |

  K_HVSP_CONTROLSTACK TKN_EQUAL num_list {
    {
      TOKEN * t;
      unsigned nbytes;
      int ok;

      current_part->ctl_stack_type = CTL_STACK_HVSP;
      nbytes = 0;
      ok = 1;

      memset(current_part->controlstack, 0, CTL_STACK_SIZE);
      while (lsize(number_list)) {
        t = lrmv_n(number_list, 1);
	if (nbytes < CTL_STACK_SIZE)
	  {
	    current_part->controlstack[nbytes] = t->value.number;
	    nbytes++;
	  }
	else
	  {
	    ok = 0;
	  }
        free_token(t);
      }
      if (!ok)
	{
	  yywarning("too many bytes in control stack");
        }
    }
  } |

  K_HVSP_CONTROLSTACK TKN_EQUAL K_NULL {
    {
      current_part->ctl_stack_type = CTL_STACK_NONE;
      memset(current_part->controlstack, 0, CTL_STACK_SIZE);
    }
  } |

  K_FLASH_INSTR TKN_EQUAL num_list {
    {
      TOKEN * t;
      unsigned nbytes;
      int ok;

      nbytes = 0;
      ok = 1;

      memset(current_part->flash_instr, 0, FLASH_INSTR_SIZE);
      while (lsize(number_list)) {
        t = lrmv_n(number_list, 1);
	if (nbytes < FLASH_INSTR_SIZE)
	  {
	    current_part->flash_instr[nbytes] = t->value.number;
	    nbytes++;
	  }
	else
	  {
	    ok = 0;
	  }
        free_token(t);
      }
      if (!ok)
	{
	  yywarning("too many bytes in flash instructions");
        }
    }
  } |

  K_FLASH_INSTR TKN_EQUAL K_NULL {
    {
      memset(current_part->flash_instr, 0, FLASH_INSTR_SIZE);
    }
  } |

  K_EEPROM_INSTR TKN_EQUAL num_list {
    {
      TOKEN * t;
      unsigned nbytes;
      int ok;

      nbytes = 0;
      ok = 1;

      memset(current_part->eeprom_instr, 0, EEPROM_INSTR_SIZE);
      while (lsize(number_list)) {
        t = lrmv_n(number_list, 1);
	if (nbytes < EEPROM_INSTR_SIZE)
	  {
	    current_part->eeprom_instr[nbytes] = t->value.number;
	    nbytes++;
	  }
	else
	  {
	    ok = 0;
	  }
        free_token(t);
      }
      if (!ok)
	{
	  yywarning("too many bytes in EEPROM instructions");
        }
    }
  } |

  K_EEPROM_INSTR TKN_EQUAL K_NULL {
    {
      memset(current_part->eeprom_instr, 0, EEPROM_INSTR_SIZE);
    }
  } |

  K_RESET TKN_EQUAL reset_disposition
    {
      if ($3->primary == K_DEDICATED)
        current_part->reset_disposition = RESET_DEDICATED;
      else if ($3->primary == K_IO)
        current_part->reset_disposition = RESET_IO;

      free_tokens(2, $1, $3);
    } |

  K_HAS_JTAG TKN_EQUAL numexpr
    {
      yywarning("has_jtag is deprecated, will be removed in v8.0, use prog_modes");
      if ($3->value.number == 1)
        current_part->prog_modes |= PM_JTAG;
      else if ($3->value.number == 0)
        current_part->prog_modes &= ~(PM_JTAG | PM_JTAGmkI | PM_XMEGAJTAG | PM_AVR32JTAG);
      free_token($3);
    } |

  K_HAS_DW TKN_EQUAL numexpr
    {
      yywarning("has_debugwire is deprecated, will be removed in v8.0, use prog_modes");
      if ($3->value.number == 1)
        current_part->prog_modes |= PM_debugWIRE;
      else if ($3->value.number == 0)
        current_part->prog_modes &= ~PM_debugWIRE;
      free_token($3);
    } |

  K_HAS_PDI TKN_EQUAL numexpr
    {
      yywarning("has_pdi is deprecated, will be removed in v8.0, use prog_modes");
      if ($3->value.number == 1)
        current_part->prog_modes |= PM_PDI;
      else if ($3->value.number == 0)
        current_part->prog_modes &= ~PM_PDI;
      free_token($3);
    } |

  K_HAS_UPDI TKN_EQUAL numexpr
    {
      yywarning("has_updi is deprecated, will be removed in v8.0, use prog_modes");
      if ($3->value.number == 1)
        current_part->prog_modes |= PM_UPDI;
      else if ($3->value.number == 0)
        current_part->prog_modes &= ~PM_UPDI;
      free_token($3);
    } |

  K_HAS_TPI TKN_EQUAL numexpr
    {
      yywarning("has_tpi is deprecated, will be removed in v8.0, use prog_modes");
      if ($3->value.number == 1)
        current_part->prog_modes |= PM_TPI;
      else if ($3->value.number == 0)
        current_part->prog_modes &= ~PM_TPI;
      free_token($3);
    } |

  K_IS_AT90S1200 TKN_EQUAL numexpr
    {
      if ($3->value.number == 1)
        current_part->flags |= AVRPART_IS_AT90S1200;
      else if ($3->value.number == 0)
        current_part->flags &= ~AVRPART_IS_AT90S1200;
      else {
        yyerror("is_at90s1200 not a Boolean value");
        free_token($3);
        YYABORT;
      }

      free_token($3);
    } |

  K_IS_AVR32 TKN_EQUAL numexpr
    {
      yywarning("is_avr32 is deprecated, will be removed in v8.0, use prog_modes");
      if ($3->value.number == 1)
        current_part->prog_modes |= PM_aWire;
      else if ($3->value.number == 0)
        current_part->prog_modes &= ~PM_aWire;
      free_token($3);
    } |

  K_ALLOWFULLPAGEBITSTREAM TKN_EQUAL numexpr
    {
      if ($3->value.number == 1)
        current_part->flags |= AVRPART_ALLOWFULLPAGEBITSTREAM;
      else if ($3->value.number == 0)
        current_part->flags &= ~AVRPART_ALLOWFULLPAGEBITSTREAM;
      else {
        yyerror("allowfullpagebitstream not a Boolean value");
        free_token($3);
        YYABORT;
      }

      free_token($3);
    } |

  K_ENABLEPAGEPROGRAMMING TKN_EQUAL numexpr
    {
      if ($3->value.number == 1)
        current_part->flags |= AVRPART_ENABLEPAGEPROGRAMMING;
      else if ($3->value.number == 0)
        current_part->flags &= ~AVRPART_ENABLEPAGEPROGRAMMING;
      else {
        yyerror("enablepageprogramming not a Boolean value");
        free_token($3);
        YYABORT;
      }

      free_token($3);
    } |

  K_SERIAL TKN_EQUAL numexpr
    {
      if ($3->value.number == 1)
        current_part->flags |= AVRPART_SERIALOK;
      else if ($3->value.number == 0)
        current_part->flags &= ~AVRPART_SERIALOK;
      else {
        yyerror("serial not a Boolean value");
        free_token($3);
        YYABORT;
      }

      free_token($3);
    } |

  K_PARALLEL TKN_EQUAL numexpr
    {
      if ($3->value.number == 1) {
        current_part->flags |= AVRPART_PARALLELOK;
        current_part->flags &= ~AVRPART_PSEUDOPARALLEL;
      }
      else if ($3->value.number == 0) {
        current_part->flags &= ~AVRPART_PARALLELOK;
        current_part->flags &= ~AVRPART_PSEUDOPARALLEL;
      }
      else if ($3->value.number == 2) {
        current_part->flags |= AVRPART_PARALLELOK;
        current_part->flags |= AVRPART_PSEUDOPARALLEL;
      }
      else {
        yyerror("parallel outside [0, 2] (yes/no/pseudo)");
        free_token($3);
        YYABORT;
      }


      free_token($3);
    } |

  K_RETRY_PULSE TKN_EQUAL retry_lines
    {
      switch ($3->primary) {
        case K_RESET :
          current_part->retry_pulse = PIN_AVR_RESET;
          break;
        case K_SCK :
          current_part->retry_pulse = PIN_AVR_SCK;
          break;
      }

      free_token($1);
    } |


  K_MEMORY TKN_STRING 
    { /* select memory for extension or create if not there */
      AVRMEM *mem = avr_locate_mem_noalias(current_part, $2->value.string);
      if(!mem) {
        mem = avr_new_memtype();
        mem->desc = cache_string($2->value.string);
        ladd(current_part->mem, mem);
      }
      avr_add_mem_order($2->value.string);
      current_mem = mem;
      free_token($2);
    }
    mem_specs 
    {
      if (is_alias) {           // alias mem has been already entered
        lrmv_d(current_part->mem, current_mem);
        avr_free_mem(current_mem);
        is_alias = false;
      } else {                  // check all opcodes re necessary address bits
        unsigned char cmd[4] = { 0, 0, 0, 0, };
        int bn;

        for(int i=0; i<AVR_OP_MAX; i++)
          if(current_mem && current_mem->op[i]) {
            if((bn = avr_set_addr_mem(current_mem, i, cmd, 0UL)) > 0)
              yywarning("%s's %s %s misses a necessary address bit a%d",
                 current_part->desc, current_mem->desc, opcodename(i), bn-1);
            }
        current_mem->comments = cfg_move_comments();
      }
      cfg_pop_comms();
      current_mem = NULL; 
      current_strct = COMP_AVRPART;
    } |
  K_MEMORY TKN_STRING TKN_EQUAL K_NULL
   {
      AVRMEM *existing_mem = avr_locate_mem_noalias(current_part, $2->value.string);
      if (existing_mem != NULL) {
        lrmv_d(current_part->mem, existing_mem);
        avr_free_mem(existing_mem);
      }
      free_token($2);
      cfg_pop_comms();
      current_mem = NULL;
      current_strct = COMP_AVRPART;
    } |
  opcode TKN_EQUAL string_list {
    { 
      int opnum;
      OPCODE * op;

      opnum = which_opcode($1);
      if (opnum < 0) YYABORT;
      op = avr_new_opcode();
      if(0 != parse_cmdbits(op, opnum))
        YYABORT;
      if (current_part->op[opnum] != NULL) {
        /*yywarning("operation redefined");*/
        avr_free_opcode(current_part->op[opnum]);
      }
      current_part->op[opnum] = op;

      free_token($1);
    }
  } |

  opcode TKN_EQUAL K_NULL {
    {
      int opnum = which_opcode($1);
      if(opnum < 0)
         YYABORT;
      if(current_part->op[opnum] != NULL)
        avr_free_opcode(current_part->op[opnum]);
      current_part->op[opnum] = NULL;

      free_token($1);
    }
  }
;


mem_specs :
  mem_spec TKN_SEMI |
  mem_alias TKN_SEMI |
  mem_specs mem_spec TKN_SEMI
;


mem_spec :
  TKN_COMPONENT TKN_EQUAL expr {
    cfg_assign((char *) current_mem, COMP_AVRMEM, $1->value.comp, &$3->value);
    free_token($1);
  } |

  K_PAGE_SIZE       TKN_EQUAL numexpr
    {
      int ps = $3->value.number;
      if (ps <= 0)
        pmsg_warning("invalid page size %d, ignored [%s:%d]\n", ps, cfg_infile, cfg_lineno);
      else
        current_mem->page_size = ps;
      free_token($3);
    } |

  K_READBACK        TKN_EQUAL TKN_NUMBER TKN_NUMBER
    {
      current_mem->readback[0] = $3->value.number;
      current_mem->readback[1] = $4->value.number;
      free_token($3);
      free_token($4);
    } |

  opcode TKN_EQUAL string_list {
    { 
      int opnum;
      OPCODE * op;

      opnum = which_opcode($1);
      if (opnum < 0) YYABORT;
      op = avr_new_opcode();
      if(0 != parse_cmdbits(op, opnum))
        YYABORT;
      if (current_mem->op[opnum] != NULL) {
        /*yywarning("operation redefined");*/
        avr_free_opcode(current_mem->op[opnum]);
      }
      current_mem->op[opnum] = op;

      free_token($1);
    }
  } |

  opcode TKN_EQUAL K_NULL {
    {
      int opnum = which_opcode($1);
      if(opnum < 0)
         YYABORT;
      if(current_mem->op[opnum] != NULL)
        avr_free_opcode(current_mem->op[opnum]);
      current_mem->op[opnum] = NULL;

      free_token($1);
    }
  }
;

mem_alias :
  K_ALIAS TKN_STRING
  {
      AVRMEM * existing_mem;

      existing_mem = avr_locate_mem(current_part, $2->value.string);
      if (existing_mem == NULL) {
        yyerror("%s alias to non-existent memory %s",
                current_mem->desc, $2->value.string);
        free_token($2);
        YYABORT;
      }

      // if this alias does already exist, drop the old one
      AVRMEM_ALIAS * alias = avr_locate_memalias(current_part, current_mem->desc);
      if (alias) {
        lrmv_d(current_part->mem_alias, alias);
        avr_free_memalias(alias);
      }

      is_alias = true;
      alias = avr_new_memalias();
      alias->desc = current_mem->desc;
      alias->aliased_mem = existing_mem;
      ladd(current_part->mem_alias, alias);

      free_token($2);
  }
;

%%

#if 0
static char * vtypestr(int type)
{
  switch (type) {
    case V_NUM : return "INTEGER";
    case V_NUM_REAL: return "REAL";
    case V_STR : return "STRING";
    default:
      return "<UNKNOWN>";
  }
}
#endif


static int clear_pin(int pinfunc) {
  if(pinfunc < 0 || pinfunc >= N_PINS) {
    yyerror("pin function must be in the range [0, %d]", N_PINS-1);
    return -1;
  }

  pin_clear_all(&(current_prog->pin[pinfunc]));

  return 0;
}

static int assign_pin(int pinfunc, TOKEN *v, int invert) {
  if(pinfunc < 0 || pinfunc >= N_PINS) {
    yyerror("pin function must be in the range [0, %d]", N_PINS-1);
    return -1;
  }

  int value = v->value.number;
  free_token(v);

  if ((value < PIN_MIN) || (value > PIN_MAX)) {
    yyerror("pin must be in the range " TOSTRING(PIN_MIN) "-"  TOSTRING(PIN_MAX));
    return -1;
  }

  pin_set_value(&(current_prog->pin[pinfunc]), value, invert);

  return 0;
}

static int assign_pin_list(int invert)
{
  TOKEN * t;
  int pin;
  int rv = 0;

  if(pin_name < 0 || pin_name >= N_PINS) {
    yyerror("pin_name should be in the range [0, %d]", N_PINS-1);
    return -1;
  }

  current_prog->pinno[pin_name] = NO_PIN;
  while (lsize(number_list)) {
    t = lrmv_n(number_list, 1);
    if (rv == 0) {
      pin = t->value.number;
      if ((pin < PIN_MIN) || (pin > PIN_MAX)) {
        yyerror("pin must be in the range " TOSTRING(PIN_MIN) "-"  TOSTRING(PIN_MAX));
        rv = -1;
      /* loop clears list and frees tokens */
      }
      pin_set_value(&(current_prog->pin[pin_name]), pin, invert);
    }
    free_token(t);
  }
  return rv;
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
    case K_LOAD_EXT_ADDR : return AVR_OP_LOAD_EXT_ADDR; break;
    case K_WRITEPAGE   : return AVR_OP_WRITEPAGE; break;
    case K_CHIP_ERASE  : return AVR_OP_CHIP_ERASE; break;
    case K_PGM_ENABLE  : return AVR_OP_PGM_ENABLE; break;
    default :
      yyerror("invalid opcode");
      return -1;
      break;
  }
}


static int parse_cmdbits(OPCODE * op, int opnum)
{
  TOKEN *t;
  int bitno;
  int len;
  char *s, *brkt = NULL;
  int rv = 0;

  bitno = 32;
  while (lsize(string_list)) {

    t = lrmv_n(string_list, 1);

    char *str = t->value.string;
    // Compact alternative specification? (eg, "0100.0000--000x.xxxx--xxaa.aaaa--iiii.iiii")
    char bit[2] = {0, 0}, *cc = str;
    int compact = !strchr(str, ' ') && strlen(str) > 7;

    bit[0] = *cc++;
    s = !compact? strtok_r(str, " ", &brkt): *bit? bit: NULL;
    while (rv == 0 && s != NULL) {

      // Ignore visual grouping characters in compact mode
      if(*s != '.' && *s != '-' && *s != '_' && *s !='/')
        bitno--;
      if (bitno < 0) {
        yyerror("too many opcode bits for instruction");
        rv = -1;
        break;
      }

      len = strlen(s);

      if (len == 0) {
        yyerror("invalid bit specifier \"\"");
        rv = -1;
        break;
      }

      if (len == 1) {
        switch (*s) {
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
            op->bit[bitno].bitno = bitno < 8 || bitno > 23? 0:
              opnum == AVR_OP_LOAD_EXT_ADDR? bitno+8: bitno-8; /* correct bit number for lone 'a' */
            if(bitno < 8 || bitno > 23)
              yywarning("address bits don't normally appear in Bytes 0 or 3 of SPI commands");
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
          case '.':
          case '-':
          case '_':
          case '/':
            break;
          default :
            yyerror("invalid bit specifier '%c'", *s);
            rv = -1;
            break;
        }
      } else {
        const char *errstr;
        int sb, bn = str_int(s+1, STR_INT32, &errstr); // Bit number
        if(errstr) {
          yywarning("bit number from %s: %s", s, errstr);
          bn = 0;
        } else if(strchr("io", *s) && (bn < 0 || bn > 7)) {
          bn &= 7;
          yywarning("out-of-range bit number %s mapped to %d", s, bn);
        }

        switch(*s) {
        case 'a':
          sb = opnum == AVR_OP_LOAD_EXT_ADDR? bitno+8: bitno-8; // should be this number
          if(bitno < 8 || bitno > 23)
            yywarning("address bits don't normally appear in Bytes 0 or 3 of SPI commands");
          else if((bn & 31) != sb) {
            if(!str_casestarts(current_part->desc, "AT89S5")) // Exempt AT89S5x from warning
              yywarning("a%d would normally be expected to be a%d", bn, sb);
          } else if(bn < 0 || bn > 31)
            yywarning("invalid address bit a%d, using a%d", bn, bn & 31);

          op->bit[bitno].bitno = bn & 31;
          op->bit[bitno].type = AVR_CMDBIT_ADDRESS;
          op->bit[bitno].value = 0;
          break;
        case 'o':
          op->bit[bitno].type  = AVR_CMDBIT_OUTPUT;
          op->bit[bitno].value = 0;
          op->bit[bitno].bitno = bn;
          break;
        case 'i':
          op->bit[bitno].type  = AVR_CMDBIT_INPUT;
          op->bit[bitno].value = 0;
          op->bit[bitno].bitno = bn;
          break;
        default:
          yyerror("invalid bit specifier %s", s);
          rv = -1;
          break;
        }
      }

      bit[0] = *cc++;
      s = !compact? strtok_r(NULL, " ", &brkt): *bit? bit: NULL;
    } /* while */

    free_token(t);

  }  /* while */

  if(bitno > 0)
    yywarning("too few opcode bits in instruction");

  return rv;
}
