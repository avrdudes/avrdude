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
%{

#include "ac_cfg.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "config.h"
#include "lists.h"
#include "par.h"
#include "pindefs.h"
#include "ppi.h"
#include "pgm.h"
#include "stk500.h"
#include "stk500v2.h"
#include "avr910.h"
#include "butterfly.h"
#include "avr.h"
#include "jtagmkII.h"

#if defined(WIN32NATIVE)
#define strtok_r( _s, _sep, _lasts ) \
    ( *(_lasts) = strtok( (_s), (_sep) ) )
#endif

extern char * progname;

int yylex(void);
int yyerror(char * errmsg);

static int assign_pin(int pinno, TOKEN * v);
static int which_opcode(TOKEN * opcode);
static int parse_cmdbits(OPCODE * op);
 
%}

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

%token K_BAUDRATE
%token K_BS2
%token K_BUFF
%token K_CHIP_ERASE_DELAY
%token K_DEDICATED
%token K_DEFAULT_PARALLEL
%token K_DEFAULT_PROGRAMMER
%token K_DEFAULT_SERIAL
%token K_DESC
%token K_DEVICECODE
%token K_STK500_DEVCODE
%token K_AVR910_DEVCODE
%token K_EEPROM
%token K_ERRLED
%token K_FLASH
%token K_ID
%token K_IO
%token K_JTAG_MKII
%token K_LOADPAGE
%token K_MAX_WRITE_DELAY
%token K_MIN_WRITE_DELAY
%token K_MISO
%token K_MOSI
%token K_NUM_PAGES
%token K_PAGEL
%token K_PAR
%token K_PARALLEL
%token K_PART
%token K_PGMLED
%token K_PROGRAMMER
%token K_PSEUDO
%token K_PWROFF_AFTER_WRITE
%token K_RDYLED
%token K_READBACK_P1
%token K_READBACK_P2
%token K_READMEM
%token K_RESET
%token K_RETRY_PULSE
%token K_SERIAL
%token K_SCK
%token K_SIZE
%token K_STK500
%token K_STK500V2
%token K_AVR910
%token K_BUTTERFLY
%token K_TYPE
%token K_VCC
%token K_VFYLED
%token K_WRITEPAGE

%token K_NO
%token K_YES

/* stk500 v2 xml file parameters */
%token K_TIMEOUT
%token K_STABDELAY
%token K_CMDEXEDELAY
%token K_SYNCHLOOPS
%token K_BYTEDELAY
%token K_POLLVALUE
%token K_POLLINDEX
%token K_PREDELAY
%token K_POSTDELAY
%token K_POLLMETHOD
%token K_MODE
%token K_DELAY
%token K_BLOCKSIZE
%token K_READSIZE

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
%token K_IDR			/* address of OCD register in IO space */
%token K_RAMPZ			/* address of RAMPZ reg. in IO space */
%token K_SPMCR			/* address of SPMC[S]R in memory space */

%token TKN_COMMA
%token TKN_EQUAL
%token TKN_SEMI
%token TKN_NUMBER
%token TKN_STRING
%token TKN_ID

%start configuration

%%

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
    strncpy(default_programmer, $3->value.string, MAX_STR_CONST);
    default_programmer[MAX_STR_CONST-1] = 0;
    free_token($3);
  } |

  K_DEFAULT_PARALLEL TKN_EQUAL TKN_STRING TKN_SEMI {
    strncpy(default_parallel, $3->value.string, PATH_MAX);
    default_parallel[PATH_MAX-1] = 0;
    free_token($3);
  } |

  K_DEFAULT_SERIAL TKN_EQUAL TKN_STRING TKN_SEMI {
    strncpy(default_serial, $3->value.string, PATH_MAX);
    default_serial[PATH_MAX-1] = 0;
    free_token($3);
  }
;


prog_def :
  K_PROGRAMMER 
    { current_prog = pgm_new();
      strcpy(current_prog->config_file, infile);
      current_prog->lineno = lineno;
    }
    prog_parms
    { 
      if (lsize(current_prog->id) == 0) {
        fprintf(stderr,
                "%s: error at %s:%d: required parameter id not specified\n",
                progname, infile, lineno);
        exit(1);
      }
      if (current_prog->type[0] == 0) {
        fprintf(stderr, "%s: error at %s:%d: programmer type not specified\n",
                progname, infile, lineno);
        exit(1);
      }
      PUSH(programmers, current_prog); 
      current_prog = NULL; 
    }
;


part_def :
  K_PART
    {
      current_part = avr_new_part();
      strcpy(current_part->config_file, infile);
      current_part->lineno = lineno;
    }
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

      PUSH(part_list, current_part); 
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

  K_TYPE TKN_EQUAL K_PAR {
    { 
      par_initpgm(current_prog);
    }
  } |

  K_TYPE TKN_EQUAL K_STK500 {
    { 
      stk500_initpgm(current_prog);
    }
  } |

  K_TYPE TKN_EQUAL K_STK500V2 {
    {
      stk500v2_initpgm(current_prog);
    }
  } |

  K_TYPE TKN_EQUAL K_AVR910 {
    { 
      avr910_initpgm(current_prog);
    }
  } |

  K_TYPE TKN_EQUAL K_BUTTERFLY {
    { 
      butterfly_initpgm(current_prog);
    }
  } |

  K_TYPE TKN_EQUAL K_JTAG_MKII {
    {
      jtagmkII_initpgm(current_prog);
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

  K_BAUDRATE TKN_EQUAL TKN_NUMBER {
    {
      current_prog->baudrate = $3->value.number;
    }
  } |

  K_RESET  TKN_EQUAL TKN_NUMBER { free_token($1); 
                                  assign_pin(PIN_AVR_RESET, $3); } |
  K_SCK    TKN_EQUAL TKN_NUMBER { free_token($1); 
                                  assign_pin(PIN_AVR_SCK, $3); } |
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


reset_disposition :
  K_DEDICATED | K_IO
;

parallel_modes :
  yesno | K_PSEUDO
;

retry_lines :
  K_RESET | K_SCK
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

  K_DEVICECODE TKN_EQUAL TKN_NUMBER {
    {
      fprintf(stderr, 
              "%s: error at %s:%d: devicecode is deprecated, use "
              "stk500_devcode instead\n",
              progname, infile, lineno);
      exit(1);
    }
  } |

  K_STK500_DEVCODE TKN_EQUAL TKN_NUMBER {
    {
      current_part->stk500_devcode = $3->value.number;
      free_token($3);
    }
  } |

  K_AVR910_DEVCODE TKN_EQUAL TKN_NUMBER {
    {
      current_part->avr910_devcode = $3->value.number;
      free_token($3);
    }
  } |

  K_CHIP_ERASE_DELAY TKN_EQUAL TKN_NUMBER
    {
      current_part->chip_erase_delay = $3->value.number;
      free_token($3);
    } |

  K_PAGEL TKN_EQUAL TKN_NUMBER
    {
      current_part->pagel = $3->value.number;
      free_token($3);
    } |

  K_BS2 TKN_EQUAL TKN_NUMBER
    {
      current_part->bs2 = $3->value.number;
      free_token($3);
    } |

  K_RESET TKN_EQUAL reset_disposition
    {
      if ($3->primary == K_DEDICATED)
        current_part->reset_disposition = RESET_DEDICATED;
      else if ($3->primary == K_IO)
        current_part->reset_disposition = RESET_IO;

      free_tokens(2, $1, $3);
    } |

  K_TIMEOUT TKN_EQUAL TKN_NUMBER
    {
      current_part->timeout = $3->value.number;
      free_token($3);
    } |

  K_STABDELAY TKN_EQUAL TKN_NUMBER
    {
      current_part->stabdelay = $3->value.number;
      free_token($3);
    } |

  K_CMDEXEDELAY TKN_EQUAL TKN_NUMBER
    {
      current_part->cmdexedelay = $3->value.number;
      free_token($3);
    } |

  K_SYNCHLOOPS TKN_EQUAL TKN_NUMBER
    {
      current_part->synchloops = $3->value.number;
      free_token($3);
    } |

  K_BYTEDELAY TKN_EQUAL TKN_NUMBER
    {
      current_part->bytedelay = $3->value.number;
      free_token($3);
    } |

  K_POLLVALUE TKN_EQUAL TKN_NUMBER
    {
      current_part->pollvalue = $3->value.number;
      free_token($3);
    } |

  K_POLLINDEX TKN_EQUAL TKN_NUMBER
    {
      current_part->pollindex = $3->value.number;
      free_token($3);
    } |

  K_PREDELAY TKN_EQUAL TKN_NUMBER
    {
      current_part->predelay = $3->value.number;
      free_token($3);
    } |

  K_POSTDELAY TKN_EQUAL TKN_NUMBER
    {
      current_part->postdelay = $3->value.number;
      free_token($3);
    } |

  K_POLLMETHOD TKN_EQUAL TKN_NUMBER
    {
      current_part->pollmethod = $3->value.number;
      free_token($3);
    } |

  K_HAS_JTAG TKN_EQUAL yesno
    {
      if ($3->primary == K_YES)
        current_part->flags |= AVRPART_HAS_JTAG;
      else if ($3->primary == K_NO)
        current_part->flags &= ~AVRPART_HAS_JTAG;

      free_token($3);
    } |

  K_ALLOWFULLPAGEBITSTREAM TKN_EQUAL yesno
    {
      if ($3->primary == K_YES)
        current_part->flags |= AVRPART_ALLOWFULLPAGEBITSTREAM;
      else if ($3->primary == K_NO)
        current_part->flags &= ~AVRPART_ALLOWFULLPAGEBITSTREAM;

      free_token($3);
    } |

  K_ENABLEPAGEPROGRAMMING TKN_EQUAL yesno
    {
      if ($3->primary == K_YES)
        current_part->flags |= AVRPART_ENABLEPAGEPROGRAMMING;
      else if ($3->primary == K_NO)
        current_part->flags &= ~AVRPART_ENABLEPAGEPROGRAMMING;

      free_token($3);
    } |

  K_IDR TKN_EQUAL TKN_NUMBER
    {
      current_part->idr = $3->value.number;
      free_token($3);
    } |

  K_RAMPZ TKN_EQUAL TKN_NUMBER
    {
      current_part->rampz = $3->value.number;
      free_token($3);
    } |

  K_SPMCR TKN_EQUAL TKN_NUMBER
    {
      current_part->spmcr = $3->value.number;
      free_token($3);
    } |

  K_SERIAL TKN_EQUAL yesno
    {
      if ($3->primary == K_YES)
        current_part->flags |= AVRPART_SERIALOK;
      else if ($3->primary == K_NO)
        current_part->flags &= ~AVRPART_SERIALOK;

      free_token($3);
    } |

  K_PARALLEL TKN_EQUAL parallel_modes
    {
      if ($3->primary == K_YES) {
        current_part->flags |= AVRPART_PARALLELOK;
        current_part->flags &= ~AVRPART_PSEUDOPARALLEL;
      }
      else if ($3->primary == K_NO) {
        current_part->flags &= ~AVRPART_PARALLELOK;
        current_part->flags &= ~AVRPART_PSEUDOPARALLEL;
      }
      else if ($3->primary == K_PSEUDO) {
        current_part->flags |= AVRPART_PARALLELOK;
        current_part->flags |= AVRPART_PSEUDOPARALLEL;
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

  K_PWROFF_AFTER_WRITE TKN_EQUAL yesno
    {
      current_mem->pwroff_after_write = $3->primary == K_YES ? 1 : 0;
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


  K_MODE TKN_EQUAL TKN_NUMBER
    {
      current_mem->mode = $3->value.number;
      free_token($3);
    } |

  K_DELAY TKN_EQUAL TKN_NUMBER
    {
      current_mem->delay = $3->value.number;
      free_token($3);
    } |

  K_BLOCKSIZE TKN_EQUAL TKN_NUMBER
    {
      current_mem->blocksize = $3->value.number;
      free_token($3);
    } |

  K_READSIZE TKN_EQUAL TKN_NUMBER
    {
      current_mem->readsize = $3->value.number;
      free_token($3);
    } |

  K_POLLINDEX TKN_EQUAL TKN_NUMBER
    {
      current_mem->pollindex = $3->value.number;
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


