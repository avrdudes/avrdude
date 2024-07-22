/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 *
 * This file was ported from
 * avrdisas - A disassembler for AVR microcontroller units
 * Copyright (C) 2007 Johannes Bauer <JohannesBauer@gmx.de>
 *
 * Copyright (C) 2024 port by Stefan Rueger <stefan.rueger@urclocks.com>
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "disasm_private.h"

static void zap_symbols() {
  if(cx->dis_symbols) {
    for(int i = 0; i < cx->dis_symbolN; i++) {
      mmt_free(cx->dis_symbols[i].comment);
      mmt_free(cx->dis_symbols[i].name);
    }
    mmt_free(cx->dis_symbols);
    cx->dis_symbols = NULL;
  }
  cx->dis_symbolN = 0;
}

static int symbol_sort(const void *v1, const void *v2) {
  const Disasm_symbol *p1 = v1, *p2 = v2;
  int diff;

  if((diff = p1->type - p2->type))
    return diff;
  return p1->address - p2->address;
}

int disasm_find_symbol(int type, int address) {
  Disasm_symbol key, *found;

  key.type = type;
  key.address = address;
  found = bsearch(&key, cx->dis_symbols, cx->dis_symbolN, sizeof(Disasm_symbol), symbol_sort);

  return found? found - cx->dis_symbols: -1;
}

static void add_symbol(int address, int type, int subtype, int count, const char *name, const char *comment) {
  int N = cx->dis_symbolN++;

  cx->dis_symbols = (Disasm_symbol *) mmt_realloc(cx->dis_symbols, sizeof(Disasm_symbol) * (N+1));
  cx->dis_symbols[N].address = address;
  cx->dis_symbols[N].type = type;
  cx->dis_symbols[N].subtype = subtype;
  cx->dis_symbols[N].count = count;
  cx->dis_symbols[N].used = 0;
  cx->dis_symbols[N].name = name? mmt_strdup(name): NULL;
  cx->dis_symbols[N].comment = comment? mmt_strdup(comment): NULL;
}

static int LineError(const char *token, const char *message, int lineno) {
  if((token == NULL) || (strlen(token) == 0)) {
    pmsg_error("%s in tagfile, line %d\n", message, lineno);
    return 1;
  }
  return 0;
}

static int Tagfile_Readline(char *line, int lineno) {
  char *token, type, subtype, *name;
  int address, count;
  const char *errptr;

  if(line[0] == '#' || strlen(line) <= 1)
    return 0;

  token = strtok(line, " \t\n");
  if(LineError(token, "nonempty line", lineno))
    return -1;
  address = str_int(token, STR_INT32, &errptr);
  if(errptr) {
    pmsg_error("address %s: %s\n", token, errptr);
    return -1;
  }

  token = strtok(NULL, " \t\n");
  if(LineError(token, "no second argument", lineno))
    return -1;
  if(strlen(token) != 1) {
    LineError(NULL, "second argument should be a type (L, P or M)", lineno);
    return -1;
  }
  type = token[0];

  token = strtok(NULL, " \t\n");
  if(LineError(token, "no third argument", lineno))
    return -1;

  if(type == 'L') {
    name = token;               // Name, comment is optional
    add_symbol(address, 'L', 0, 0, name, strtok(NULL, "\t\n"));
    return 0;
  }

  if(LineError(token, "no fourth argument", lineno))
    return -1;
  if(strlen(token) != 1) {
    LineError(NULL, "fourth argument should be a subtype (B, W, A or S)", lineno);
    return -1;
  }
  subtype = token[0];

  // Either B(yte), W(ord), A(utoterminated string) or S(tring)
  switch(subtype) {
  case 'B':
    subtype = TYPE_BYTE;
    break;
  case 'W':
    subtype = TYPE_WORD;
    break;
  case 'A':
    subtype = TYPE_ASTRING;
    break;
  case 'S':
    subtype = TYPE_STRING;
    break;
  default:
    LineError(NULL, "invalid subtype (expected one of B, W, A or S)", lineno);
    return -1;
  }

  if((type == 'M') && ((subtype != TYPE_BYTE) && (subtype != TYPE_WORD))) {
    LineError(NULL, "memory labels can only be of type B or W", lineno);
    return -1;
  }

  token = strtok(NULL, " \t\n");
  count = str_int(token, STR_INT32, &errptr);
  if(errptr) {
    pmsg_error("count %s: %s\n", token, errptr);
    return -1;
  }
  if(count < 1) {
    LineError(NULL, str_ccprintf("invalid count %d given", count), lineno);
    return -1;
  }

  name = strtok(NULL, " \t\n");
  if(type != 'P' && type != 'M') {
    pmsg_error("invalid tag type %c (must be L, P or M)\n", type);
    return -1;
  }

  add_symbol(address, type, subtype, count, name, NULL);
  return 0;
}


int disasm_init_tagfile(const AVRPART *p, const char *fname) {
  FILE *inf = fopen(fname, "r");
  int lineno = 1;
  const char *errstr;

  if(!inf) {
    pmsg_ext_error("cannot open tagfile %s: %s\n", fname, strerror(errno));
    return -1;
  }

  zap_symbols();
  disasm_init_regfile(p);

  for(char *buffer; (buffer = str_fgets(inf, &errstr)); mmt_free(buffer))
    if(Tagfile_Readline(buffer, lineno++) < 0)
      goto error;

  if(errstr) {
    pmsg_error("read error in tag file %s: %s\n", fname, errstr);
    goto error;
  }

  fclose(inf);
  qsort(cx->dis_symbols, cx->dis_symbolN, sizeof(Disasm_symbol), symbol_sort);
  return 0;

error:
  fclose(inf);
  return -1;
}

char *Tagfile_GetLabel(int index) {
  return cx->dis_symbols[index].name;
}

char *Tagfile_GetLabelComment(int index) {
  return cx->dis_symbols[index].comment;
}

const char *Tagfile_Resolve_Mem_Address(int address) {
  for(int i = 0; i < cx->dis_symbolN; i++) {
    if(cx->dis_symbols[i].type != 'M')
      continue;
    if(cx->dis_symbols[i].address > address)
      break;

    int start = cx->dis_symbols[i].address;
    int size = cx->dis_symbols[i].subtype == TYPE_WORD? 2: 1;
    int end = cx->dis_symbols[i].address + cx->dis_symbols[i].count * size - 1;

    if(address >= start && address <= end) {
      if(cx->dis_symbols[i].count == 1) { // Single variable
        if(size == 1)
          return str_ccprintf("%s", cx->dis_symbols[i].name);
        return str_ccprintf("_%s8(%s)", address == start? "lo": "hi", cx->dis_symbols[i].name);
      }
      // Array
      if(size == 1)
        return str_ccprintf("%s[%d]", cx->dis_symbols[i].name, address - start);
      return str_ccprintf("_%s8(%s[%d])", (address - start)%2? "hi": "lo",
        cx->dis_symbols[i].name, (address - start)/2);
    }
  }

  return NULL;
}

static int Tagfile_Process_Byte(const char *Bitstream, int Position, int offset, int ArgumentNo, const char *Label) {
  term_out(".byte 0x%02x\n", Bitstream[Position] & 0xff);
  return 1;
}

static int Tagfile_Process_Word(const char *Bitstream, int Position, int offset, int ArgumentNo, const char *Label) {
  term_out(".word 0x%02x%02x\n", Bitstream[Position + 1] & 0xff, Bitstream[Position] & 0xff);
  return 2;
}

static int Tagfile_Process_String(const char *Bitstream, int Position, int offset, int ArgumentNo, const char *Label) {
  int i;
  unsigned char c;
  unsigned int InString = 0;

  term_out("String_0x%s_%d:    ; Address 0x%x (%d)\n", Label, ArgumentNo, Position, Position);
  i = 0;
  while((c = Bitstream[Position + i])) {
    if((c >= 32) && (c <= 127)) {
      if(!InString)
        term_out(".ascii \"");
      term_out("%c", c);
      InString = 1;
    } else {
      if(InString)
        term_out("\"\n");
      term_out(".byte 0x%02x\n", c);
      InString = 0;
    }
    i++;
  }
  if(InString)
    term_out("\\0\"\n");
  else
    term_out(".byte 0x00\n");

  term_out("\n");
  return i + 1;
}

static void Sanitize_String(char *String) {
  size_t i;
  size_t l = strlen(String);

  for(i = 0; i < l; i++) {
    if(!(((String[i] >= 'a') && (String[i] <= 'z')) ||
        (((String[i] >= 'A') && (String[i] <= 'Z'))) || (((String[i] >= '0') && (String[i] <= '9'))))) {
      String[i] = '_';
    }
  }
}

int Tagfile_Process_Data(const char *Bitstream, int Position, int offset) {
  int BytesAdvanced;
  int (*ProcessingFunction)(const char *, int, int, int, const char *) = NULL;
  char Buffer[32];

  int index = disasm_find_symbol('P', disasm_wrap(Position + offset));
  if(index < 0)
    return 0;

  switch(cx->dis_symbols[index].subtype) {
  case TYPE_BYTE:
    ProcessingFunction = Tagfile_Process_Byte;
    break;
  case TYPE_WORD:
    ProcessingFunction = Tagfile_Process_Word;
    break;
  case TYPE_ASTRING:
    ProcessingFunction = Tagfile_Process_String;
    break;
  case TYPE_STRING:
    ProcessingFunction = Tagfile_Process_String;
    break;
  }

  term_out("; Inline PGM data: %d ", cx->dis_symbols[index].count);
  switch(cx->dis_symbols[index].subtype) {
  case TYPE_BYTE:
    term_out("byte");
    break;
  case TYPE_WORD:
    term_out("word");
    break;
  case TYPE_ASTRING:
    term_out("autoaligned string");
    break;
  case TYPE_STRING:
    term_out("string");
    break;
  }
  if(cx->dis_symbols[index].count != 1)
    term_out("s");
  term_out(" starting at 0x%0*x", cx->dis_addrwidth, disasm_wrap(Position + offset));

  if(cx->dis_symbols[index].name)
    term_out(" (%s)", cx->dis_symbols[index].name);
  term_out("\n");

  if((cx->dis_symbols[index].subtype == TYPE_ASTRING) || (cx->dis_symbols[index].subtype == TYPE_STRING)) {
    if(cx->dis_symbols[index].name != NULL) {
      snprintf(Buffer, sizeof(Buffer), "%x_%s", disasm_wrap(Position + offset), cx->dis_symbols[index].name);
      Sanitize_String(Buffer);
    } else {
      snprintf(Buffer, sizeof(Buffer), "%x", disasm_wrap(Position + offset));
    }
  }

  BytesAdvanced = 0;
  for(int i = 0; i < cx->dis_symbols[index].count; i++)
    BytesAdvanced += ProcessingFunction(Bitstream, Position + BytesAdvanced, offset, i, Buffer);

  if(cx->dis_symbols[index].subtype == TYPE_ASTRING) {
    // Autoaligned string
    if((BytesAdvanced % 2) != 0) {
      // Not yet aligned correctly
      if(Bitstream[Position + BytesAdvanced] != 0x00)
        pmsg_warning("autoalignment expected zero but got 0x%02x padding; ignored\n",
          Bitstream[Position + BytesAdvanced] & 0xff);
      term_out(".byte 0x%02x        ; String Autoalignment\n", ((unsigned char *) Bitstream)[Position + BytesAdvanced]);
      BytesAdvanced++;
    }
  }

  term_out("\n");
  return BytesAdvanced;
}


// Allocate, copy, append a suffix (H, L, 0...8 or nothing), make upper case and return
static char *regname(const char *pre, const char *reg, int suf) {
  char *ret =
    suf <= -1? str_sprintf("%s%s", pre, reg):
    suf == 'h' || suf == 'l'? str_sprintf("%s%s%c", pre, reg, suf):
    str_sprintf("%s%s%d", pre, reg, suf);

  for(char *s = ret; *s; s++)
    *s = *s == '.'? '_': isascii(*s & 0xff)? toupper(*s & 0xff): *s;

  return ret;
}

// Initialise cx->dis_symbols from part register file
void disasm_init_regfile(const AVRPART *p) {
  int nr = 0, offset = 0;
  const Register_file *rf = avr_locate_register_file(p, &nr);

  if(rf) {
    AVRMEM *mem = avr_locate_io(p);
    if(mem)
      offset = mem->offset;
    const char *mpre = offset? "MEM_": "";
    const char *ipre = offset? "IO_": "";
    for(int i = 0; i< nr; i++) {
      int addr = offset + rf[i].addr;
      int sub = rf[i].size == 2? TYPE_WORD: TYPE_BYTE;
      int count = rf[i].size > 2? rf[i].size: 1;
      add_symbol(addr, 'M', sub, count, regname(mpre, rf[i].reg, -1), NULL);

      if(rf[i].addr < 0x40) {
        if(rf[i].size == 1)
          add_symbol(rf[i].addr, 'I', TYPE_BYTE, 1, regname(ipre, rf[i].reg, -1), NULL);
        else if(rf[i].size == 2) {
          add_symbol(rf[i].addr,   'I', TYPE_BYTE, 1, regname(ipre, rf[i].reg, 'l'), NULL);
          add_symbol(rf[i].addr+1, 'I', TYPE_BYTE, 1, regname(ipre, rf[i].reg, 'h'), NULL);
        } else if(rf[i].size > 2) {
          for(int k = 0; k < rf[i].size; k++)
            add_symbol(rf[i].addr+k, 'I', TYPE_BYTE, 1, regname(ipre, rf[i].reg, k), NULL);
        }
      }
    }
    qsort(cx->dis_symbols, cx->dis_symbolN, sizeof(Disasm_symbol), symbol_sort);
  }
}

const char *Resolve_IO_Register(int Number) {
  for(int i = 0; i < cx->dis_symbolN; i++) {
    if(cx->dis_symbols[i].type != 'I')
      continue;
    if(cx->dis_symbols[i].address == Number) {
      cx->dis_symbols[i].used = 1;
      return cx->dis_symbols[i].name;
    }
  }

  return NULL;
}

void Emit_Used_IO_Registers() {
  for(int i = 0; i < cx->dis_symbolN; i++)
    if(cx->dis_symbols[i].used)
      term_out(".equ %s, 0x%02x\n", cx->dis_symbols[i].name, cx->dis_symbols[i].address);
}
