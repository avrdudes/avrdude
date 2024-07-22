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

static int LineError(const char *Token, const char *Message, int LineNo) {
  if((Token == NULL) || (strlen(Token) == 0)) {
    pmsg_error("%s in tagfile, line %d\n", Message, LineNo);
    return 1;
  }
  return 0;
}

static void zap_IORegisters() {
  if(cx->dis_IORegisters) {
    for(int i = 0; i < cx->dis_IORegisterN; i++)
      mmt_free(cx->dis_IORegisters[i].Name);
    mmt_free(cx->dis_IORegisters);
    cx->dis_IORegisters = NULL;
  }
  cx->dis_IORegisterN = 0;
}

static void zap_CodeLabels() {
  if(cx->dis_CodeLabels) {
    for(int i = 0; i < cx->dis_CodeLabelN; i++) {
      mmt_free(cx->dis_CodeLabels[i].Comment);
      mmt_free(cx->dis_CodeLabels[i].Text);
    }
    mmt_free(cx->dis_CodeLabels);
    cx->dis_CodeLabels = NULL;
  }
  cx->dis_CodeLabelN = 0;
}

static void zap_PGMLabels() {
  if(cx->dis_PGMLabels) {
    for(int i = 0; i < cx->dis_PGMLabelN; i++)
      mmt_free(cx->dis_PGMLabels[i].Comment);
    mmt_free(cx->dis_PGMLabels);
    cx->dis_PGMLabels = NULL;
  }
  cx->dis_PGMLabelN = 0;
}

static void zap_MemLabels() {
  if(cx->dis_MemLabels) {
    for(int i = 0; i < cx->dis_MemLabelN; i++)
      mmt_free(cx->dis_MemLabels[i].Comment);
    mmt_free(cx->dis_MemLabels);
    cx->dis_MemLabels = NULL;
  }
  cx->dis_MemLabelN = 0;
}


static void Add_Code_Tag(int Address, const char *LabelText, const char *LabelComment) {
  cx->dis_CodeLabelN++;

  cx->dis_CodeLabels = (Disasm_CodeLabel *) mmt_realloc(cx->dis_CodeLabels, sizeof(Disasm_CodeLabel) * cx->dis_CodeLabelN);
  cx->dis_CodeLabels[cx->dis_CodeLabelN - 1].Address = Address;
  cx->dis_CodeLabels[cx->dis_CodeLabelN - 1].Text = LabelText? mmt_strdup(LabelText): NULL;
  cx->dis_CodeLabels[cx->dis_CodeLabelN - 1].Comment = LabelComment? mmt_strdup(LabelComment): NULL;
}

static void Add_PGM_Tag(int Address, char subtype, unsigned int Count, const char *Comment) {
  cx->dis_PGMLabelN++;

  cx->dis_PGMLabels = (Disasm_PGMLabel *) mmt_realloc(cx->dis_PGMLabels, sizeof(Disasm_PGMLabel) * cx->dis_PGMLabelN);
  cx->dis_PGMLabels[cx->dis_PGMLabelN - 1].Address = Address;
  cx->dis_PGMLabels[cx->dis_PGMLabelN - 1].subtype = subtype;
  cx->dis_PGMLabels[cx->dis_PGMLabelN - 1].Count = Count;
  cx->dis_PGMLabels[cx->dis_PGMLabelN - 1].Comment = Comment? mmt_strdup(Comment): NULL;
}

static void Add_Mem_Tag(int Address, char subtype, unsigned int Count, const char *Comment) {
  cx->dis_MemLabelN++;

  cx->dis_MemLabels = (Disasm_MemLabel *) mmt_realloc(cx->dis_MemLabels, sizeof(Disasm_MemLabel) * cx->dis_MemLabelN);
  cx->dis_MemLabels[cx->dis_MemLabelN - 1].Address = Address;
  cx->dis_MemLabels[cx->dis_MemLabelN - 1].subtype = subtype;
  cx->dis_MemLabels[cx->dis_MemLabelN - 1].Count = Count;
  cx->dis_MemLabels[cx->dis_MemLabelN - 1].Comment = Comment? mmt_strdup(Comment): NULL;
}

static int Tagfile_Readline(char *Line, int LineNo) {
  char *Token, Type, Subtype, *Name;
  int Address, Count;
  const char *errptr;

  if(Line[0] == '#' || strlen(Line) <= 1)
    return 0;

  Token = strtok(Line, " \t\n");
  if(LineError(Token, "nonempty line", LineNo))
    return -1;
  Address = str_int(Token, STR_INT32, &errptr);
  if(errptr) {
    pmsg_error("address %s: %s\n", Token, errptr);
    return -1;
  }

  Token = strtok(NULL, " \t\n");
  if(LineError(Token, "no second argument", LineNo))
    return -1;
  if(strlen(Token) != 1) {
    LineError(NULL, "second argument should be a type (L, P or M)", LineNo);
    return -1;
  }
  Type = Token[0];

  Token = strtok(NULL, " \t\n");
  if(LineError(Token, "no third argument", LineNo))
    return -1;

  if(Type == 'L') {
    Name = Token;               // Name, comment is optional
    Add_Code_Tag(Address, Name, strtok(NULL, "\t\n"));
    return 0;
  }

  if(LineError(Token, "no fourth argument", LineNo))
    return -1;
  if(strlen(Token) != 1) {
    LineError(NULL, "fourth argument should be a subtype (B, W, A or S)", LineNo);
    return -1;
  }
  Subtype = Token[0];

  // Either B(yte), W(ord), A(utoterminated string) or S(tring)
  switch (Subtype) {
  case 'B':
    Subtype = TYPE_BYTE;
    break;
  case 'W':
    Subtype = TYPE_WORD;
    break;
  case 'A':
    Subtype = TYPE_ASTRING;
    break;
  case 'S':
    Subtype = TYPE_STRING;
    break;
  default:
    LineError(NULL, "invalid subtype (expected one of B, W, A or S)", LineNo);
    return -1;
  }

  if((Type == 'M') && ((Subtype != TYPE_BYTE) && (Subtype != TYPE_WORD))) {
    LineError(NULL, "memory labels can only be of type B or W", LineNo);
    return -1;
  }

  Token = strtok(NULL, " \t\n");
  Count = str_int(Token, STR_INT32, &errptr);
  if(errptr) {
    pmsg_error("count %s: %s\n", Token, errptr);
    return -1;
  }
  if(Count < 1) {
    LineError(NULL, str_ccprintf("invalid count %d given", Count), LineNo);
    return -1;
  }

  Name = strtok(NULL, " \t\n");
  if(Type == 'P') {
    Add_PGM_Tag(Address, Subtype, Count, Name);
  } else if(Type == 'M') {
    Add_Mem_Tag(Address, Subtype, Count, Name);
  } else {
    pmsg_error("invalid tag type %c\n", Type);
    return -1;
  }

  return 0;
}

static int CodeLabelSort(const void *A, const void *B) {
  const Disasm_CodeLabel *X, *Y;

  X = (const Disasm_CodeLabel *) A;
  Y = (const Disasm_CodeLabel *) B;
  if(X->Address == Y->Address)
    return 0;
  if(X->Address < Y->Address)
    return -1;
  return 1;
}

static int PGMLabelSort(const void *A, const void *B) {
  const Disasm_PGMLabel *X, *Y;

  X = (const Disasm_PGMLabel *) A;
  Y = (const Disasm_PGMLabel *) B;
  if(X->Address == Y->Address)
    return 0;
  if(X->Address < Y->Address)
    return -1;
  return 1;
}

static int MemLabelSort(const void *A, const void *B) {
  const Disasm_MemLabel *X, *Y;

  X = (const Disasm_MemLabel *) A;
  Y = (const Disasm_MemLabel *) B;
  if(X->Address == Y->Address)
    return 0;
  if(X->Address < Y->Address)
    return -1;
  return 1;
}

static void Tagfile_SortLabels() {
  qsort(cx->dis_CodeLabels, cx->dis_CodeLabelN, sizeof(Disasm_CodeLabel), CodeLabelSort);
  qsort(cx->dis_PGMLabels, cx->dis_PGMLabelN, sizeof(Disasm_PGMLabel), PGMLabelSort);
  qsort(cx->dis_MemLabels, cx->dis_MemLabelN, sizeof(Disasm_MemLabel), MemLabelSort);
}

int disasm_init_tagfile(const AVRPART *p, const char *Filename) {
  FILE *inf = fopen(Filename, "r");
  int LineNo = 1;
  const char *errstr;

  if(!inf) {
    pmsg_ext_error("cannot open tagfile %s: %s\n", Filename, strerror(errno));
    return -1;
  }

  zap_CodeLabels();
  zap_PGMLabels();
  zap_MemLabels();
  disasm_init_regfile(p);

  for(char *buffer; (buffer = str_fgets(inf, &errstr)); mmt_free(buffer))
    if(Tagfile_Readline(buffer, LineNo++) < 0)
      goto error;

  if(errstr) {
    pmsg_error("read error in tag file %s: %s\n", Filename, errstr);
    goto error;
  }

  fclose(inf);
  Tagfile_SortLabels();
  return 0;

error:
  fclose(inf);
  return -1;
}

int Tagfile_FindLabelAddress(int Address) {
  Disasm_CodeLabel Goal;
  Disasm_CodeLabel *Result;

  Goal.Address = Address;
  Result = bsearch(&Goal, cx->dis_CodeLabels, cx->dis_CodeLabelN, sizeof(Disasm_CodeLabel), CodeLabelSort);
  if(Result == NULL)
    return -1;
  return Result - cx->dis_CodeLabels;
}

char *Tagfile_GetLabel(int TagIndex) {
  return cx->dis_CodeLabels[TagIndex].Text;
}

char *Tagfile_GetLabelComment(int TagIndex) {
  return cx->dis_CodeLabels[TagIndex].Comment;
}

int Tagfile_FindPGMAddress(int Address) {
  Disasm_PGMLabel Goal;
  Disasm_PGMLabel *Result;

  Goal.Address = Address;
  Result = bsearch(&Goal, cx->dis_PGMLabels, cx->dis_PGMLabelN, sizeof(Disasm_PGMLabel), PGMLabelSort);
  if(Result == NULL)
    return -1;
  return Result - cx->dis_PGMLabels;
}

const char *Tagfile_Resolve_Mem_Address(int Address) {
  for(int i = 0; i < cx->dis_MemLabelN && cx->dis_MemLabels[i].Address <= Address; i++) {
    int Start = cx->dis_MemLabels[i].Address;
    int Size = cx->dis_MemLabels[i].subtype == TYPE_WORD? 2: 1;
    int End = cx->dis_MemLabels[i].Address + cx->dis_MemLabels[i].Count * Size - 1;

    if(Address >= Start && Address <= End) {
      if(cx->dis_MemLabels[i].Count == 1) { // Single variable
        if(Size == 1)
          return str_ccprintf("%s", cx->dis_MemLabels[i].Comment);
        return str_ccprintf("_%s8(%s)", Address == Start? "lo": "hi", cx->dis_MemLabels[i].Comment);
      }
      // Array
      if(Size == 1)
        return str_ccprintf("%s[%d]", cx->dis_MemLabels[i].Comment, Address - Start);
      return str_ccprintf("_%s8(%s[%d])", (Address-Start)%2? "hi": "lo", cx->dis_MemLabels[i].Comment,
        (Address - Start)/2);
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
  int Index;
  int (*ProcessingFunction)(const char *, int, int, int, const char *) = NULL;
  char Buffer[32];

  Index = Tagfile_FindPGMAddress(disasm_wrap(Position + offset));
  if(Index == -1)
    return 0;

  switch (cx->dis_PGMLabels[Index].subtype) {
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

  term_out("; Inline PGM data: %d ", cx->dis_PGMLabels[Index].Count);
  switch (cx->dis_PGMLabels[Index].subtype) {
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
  if(cx->dis_PGMLabels[Index].Count != 1)
    term_out("s");
  term_out(" starting at 0x%0*x", cx->dis_addrwidth, disasm_wrap(Position + offset));

  if(cx->dis_PGMLabels[Index].Comment)
    term_out(" (%s)", cx->dis_PGMLabels[Index].Comment);
  term_out("\n");

  if((cx->dis_PGMLabels[Index].subtype == TYPE_ASTRING) || (cx->dis_PGMLabels[Index].subtype == TYPE_STRING)) {
    if(cx->dis_PGMLabels[Index].Comment != NULL) {
      snprintf(Buffer, sizeof(Buffer), "%x_%s", disasm_wrap(Position + offset), cx->dis_PGMLabels[Index].Comment);
      Sanitize_String(Buffer);
    } else {
      snprintf(Buffer, sizeof(Buffer), "%x", disasm_wrap(Position + offset));
    }
  }

  BytesAdvanced = 0;
  for(unsigned i = 0; i < cx->dis_PGMLabels[Index].Count; i++)
    BytesAdvanced += ProcessingFunction(Bitstream, Position + BytesAdvanced, offset, i, Buffer);

  if(cx->dis_PGMLabels[Index].subtype == TYPE_ASTRING) {
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
static char *regname(const char *reg, int suf) {
  char *ret =
    suf <= -1? mmt_strdup(reg):
    suf == 'h' || suf == 'l'? str_sprintf("%s%c", reg, suf):
    str_sprintf("%s%d", reg, suf);

  for(char *s = ret; *s; s++)
    *s = *s == '.'? '_': isascii(*s & 0xff)? toupper(*s & 0xff): *s;

  return ret;
}

// Initialise cx->dis_IORegisters and cx->dis_MemLabels from part register file
void disasm_init_regfile(const AVRPART *p) {
  int nr = 0, nio = 0, offset = 0;
  const Register_file *rf = avr_locate_register_file(p, &nr);

  if(rf) {
    zap_MemLabels();
    zap_IORegisters();

    // Count how many entries are needed
    for(int i = 0; i< nr; i++)
      if(rf[i].addr < 0x40 && rf[i].size > 0)
        nio += rf[i].size;
    cx->dis_IORegisters = mmt_malloc(nio*sizeof*cx->dis_IORegisters);
    cx->dis_MemLabels = mmt_malloc(nr*sizeof*cx->dis_MemLabels);

    AVRMEM *mem = avr_locate_io(p);
    if(mem)
      offset = mem->offset;
    nio = 0;
    for(int i = 0; i< nr; i++) {
      cx->dis_MemLabels[i].Address = offset + rf[i].addr;
      cx->dis_MemLabels[i].subtype = rf[i].size == 2? TYPE_WORD: TYPE_BYTE;
      cx->dis_MemLabels[i].Count = rf[i].size > 2? rf[i].size: 1;
      cx->dis_MemLabels[i].Comment = regname(rf[i].reg, -1);

      if(rf[i].addr < 0x40) {
        if(rf[i].size == 1) {
          cx->dis_IORegisters[nio].Name = regname(rf[i].reg, -1);
          cx->dis_IORegisters[nio].Address = rf[i].addr;
          nio++;
        } else if(rf[i].size == 2) {
          cx->dis_IORegisters[nio].Name = regname(rf[i].reg, 'l');
          cx->dis_IORegisters[nio].Address = rf[i].addr;
          nio++;
          cx->dis_IORegisters[nio].Name = regname(rf[i].reg, 'h');
          cx->dis_IORegisters[nio].Address = rf[i].addr+1;
          nio++;
        } else if(rf[i].size > 2) {
          for(int k = 0; k < rf[i].size; k++) {
            cx->dis_IORegisters[nio].Name = regname(rf[i].reg, k);
            cx->dis_IORegisters[nio].Address = rf[i].addr + k;
            nio++;
          }
        }
      }
    }
    cx->dis_IORegisterN = nio;
    cx->dis_MemLabelN = nr;
    qsort(cx->dis_MemLabels, cx->dis_MemLabelN, sizeof(Disasm_MemLabel), MemLabelSort);
  }
}

const char *Resolve_IO_Register(int Number) {
  for(int i = 0; i < cx->dis_IORegisterN; i++) {
    if(cx->dis_IORegisters[i].Address == Number) {
      cx->dis_IORegisters[i].Used = 1;
      return cx->dis_IORegisters[i].Name;
    }
  }

  return NULL;
}

void Emit_Used_IO_Registers() {
  for(int i = 0; i < cx->dis_IORegisterN; i++)
    if(cx->dis_IORegisters[i].Used)
      term_out(".equ %s, 0x%02x\n", cx->dis_IORegisters[i].Name, cx->dis_IORegisters[i].Address);
}
