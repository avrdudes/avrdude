
/*
    avrdisas - A disassembler for AVR microcontroller units
    Copyright (C) 2007 Johannes Bauer

    This file is part of avrdisas.

    avrdisas is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    avrdisas is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with avrdisas; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    Johannes Bauer
    Mussinanstr. 140
    92318 Neumarkt/Opf.
    JohannesBauer@gmx.de
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "disasm_private.h"
#include "disasm_tagfile.h"

struct CodeLabel {
  int Address;
  char *Text;
  char *Comment;
};

struct PGMLabel {
  int Address;
  char Type;
  unsigned int Count;
  char *Comment;
};

struct MemLabel {
  int Address;
  char Type;
  unsigned int Count;
  char *Comment;
};

struct IO_Register {
  int Address;
  char *Name;
  unsigned char Used;
};

static int CodeLabelCount = 0;
static struct CodeLabel *CodeLabels;

static int PGMLabelCount = 0;
static struct PGMLabel *PGMLabels = NULL;

static int MemLabelCount = 0;
static struct MemLabel *MemLabels = NULL;

static struct IO_Register *IORegisters = NULL;
static int IORegistersCount = 0;

/*
static void Display_Tagfile() {
  int i;
  printf("%d code labels:\n", CodeLabelCount);
  for(i = 0; i < CodeLabelCount; i++)
    printf("%d: 0x%x = %s\n", i, CodeLabels[i].Address, CodeLabels[i].Text);

  printf("%d PGM labels:\n", PGMLabelCount);
  for(i = 0; i < PGMLabelCount; i++)
     printf("%d: 0x%x = %d * %d\n", i, PGMLabels[i].Address, PGMLabels[i].Count, PGMLabels[i].Type);
}
*/

static int LineError(const char *Token, const char *Message, int LineNo) {
  if((Token == NULL) || (strlen(Token) == 0)) {
    fprintf(stderr, "Error: %s in tagfile, line %d.\n", Message, LineNo);
    return 1;
  }
  return 0;
}

static int ahtoi(const char *String) {
  int Value;
  size_t i, l;

  if(strlen(String) < 3)
    return atoi(String);
  if((String[0] != '0') || (String[1] != 'x'))
    return atoi(String);
  String += 2;
  Value = 0;
  l = strlen(String);
  for(i = 0; i < l; i++) {
    Value *= 16;
    if((String[i] >= '0') && (String[i] <= '9'))
      Value += String[i] - '0';
    else if((String[i] >= 'a') && (String[i] <= 'f'))
      Value += String[i] - 'a' + 10;
    else if((String[i] >= 'A') && (String[i] <= 'F'))
      Value += String[i] - 'A' + 10;
    else
      return 0;
  }
  return Value;
}

static void Add_LabelTag(int Address, const char *LabelText, const char *LabelComment) {
  CodeLabelCount++;

  CodeLabels = (struct CodeLabel *) mmt_realloc(CodeLabels, sizeof(struct CodeLabel) * CodeLabelCount);
  CodeLabels[CodeLabelCount - 1].Address = Address;
  CodeLabels[CodeLabelCount - 1].Text = LabelText? mmt_strdup(LabelText): NULL;
  CodeLabels[CodeLabelCount - 1].Comment = LabelComment? mmt_strdup(LabelComment): NULL;
}

static void Add_PGM_Tag(int Address, char Type, unsigned int Count, const char *Comment) {
  PGMLabelCount++;

  PGMLabels = (struct PGMLabel *) mmt_realloc(PGMLabels, sizeof(struct PGMLabel) * PGMLabelCount);
  PGMLabels[PGMLabelCount - 1].Address = Address;
  PGMLabels[PGMLabelCount - 1].Type = Type;
  PGMLabels[PGMLabelCount - 1].Count = Count;
  PGMLabels[PGMLabelCount - 1].Comment = Comment? mmt_strdup(Comment): NULL;
}

static void Add_Mem_Tag(int Address, char Type, unsigned int Count, const char *Comment) {
  MemLabelCount++;

  MemLabels = (struct MemLabel *) mmt_realloc(MemLabels, sizeof(struct MemLabel) * MemLabelCount);
  MemLabels[MemLabelCount - 1].Address = Address;
  MemLabels[MemLabelCount - 1].Type = Type;
  MemLabels[MemLabelCount - 1].Count = Count;
  MemLabels[MemLabelCount - 1].Comment = Comment? mmt_strdup(Comment): NULL;
}

static void Tagfile_Readline(char *Line, int LineNo) {
  char *Token;
  int Address;
  char Type, Subtype;
  int Count;

  if(Line[0] == '#')
    return;
  if(strlen(Line) <= 1)
    return;

  Token = strtok(Line, "\t\n");
  if(LineError(Token, "nonempty line", LineNo))
    return;

  // Token now holds an address, determine if hex or dec
  Address = ahtoi(Token);

  Token = strtok(NULL, "\t\n");
  if(LineError(Token, "no second argument", LineNo))
    return;
  if(strlen(Token) != 1) {
    LineError(NULL, "second argument too long", LineNo);
    return;
  }

  Type = Token[0];
  Token = strtok(NULL, "\t\n");
  if(LineError(Token, "no third argument", LineNo))
    return;

  if(Type == 'L') {
    char *LabelName = Token;

    Token = strtok(NULL, "\t\n");
    Add_LabelTag(Address, LabelName, Token);
    return;
  }

  if(LineError(Token, "no fourth argument", LineNo))
    return;
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
    Subtype = 0;
  }
  if(!Subtype) {
    LineError(NULL, "invalid type (expected one of L, B, W, A or S)", LineNo);
    return;
  }
  if((Type == 'M') && ((Subtype != TYPE_BYTE) && (Subtype != TYPE_WORD))) {
    LineError(NULL, "memory labels can only be of type B or W", LineNo);
    return;
  }

  Token = strtok(NULL, "\t\n");
  Count = ahtoi(Token);
  if(Count < 1) {
    LineError(NULL, "invalid count given", LineNo);
    return;
  }

  Token = strtok(NULL, "\t\n");
  if(Type == 'P') {
    Add_PGM_Tag(Address, Subtype, Count, Token);
  } else if(Type == 'M') {
    Add_Mem_Tag(Address, Subtype, Count, Token);
  } else {
    fprintf(stderr, "Invalid tag type '%c'.\n", Type);
  }
}

static int CodeLabelSort(const void *A, const void *B) {
  const struct CodeLabel *X, *Y;

  X = (const struct CodeLabel *) A;
  Y = (const struct CodeLabel *) B;
  if(X->Address == Y->Address)
    return 0;
  if(X->Address < Y->Address)
    return -1;
  return 1;
}

static int PGMLabelSort(const void *A, const void *B) {
  const struct PGMLabel *X, *Y;

  X = (const struct PGMLabel *) A;
  Y = (const struct PGMLabel *) B;
  if(X->Address == Y->Address)
    return 0;
  if(X->Address < Y->Address)
    return -1;
  return 1;
}

static int MemLabelSort(const void *A, const void *B) {
  const struct MemLabel *X, *Y;

  X = (const struct MemLabel *) A;
  Y = (const struct MemLabel *) B;
  if(X->Address == Y->Address)
    return 0;
  if(X->Address < Y->Address)
    return -1;
  return 1;
}

static void Tagfile_SortLabels() {
  qsort(CodeLabels, CodeLabelCount, sizeof(struct CodeLabel), CodeLabelSort);
  qsort(PGMLabels, PGMLabelCount, sizeof(struct PGMLabel), PGMLabelSort);
  qsort(MemLabels, MemLabelCount, sizeof(struct MemLabel), MemLabelSort);
}

int Read_Tagfile(const char *Filename) {
  FILE *f;

  f = fopen(Filename, "r");
  if(!f) {
    fprintf(stderr, "Error opening tagfile '%s': %s\n", Filename, strerror(errno));
    return 0;
  }

  {
    char Buffer[256];
    int LineNo = 1;

    while(fgets(Buffer, sizeof(Buffer), f) != NULL) {
      Tagfile_Readline(Buffer, LineNo++);
    }
  }

  fclose(f);

  Tagfile_SortLabels();
  return 1;
}

int Tagfile_FindLabelAddress(int Address) {
  struct CodeLabel Goal;
  struct CodeLabel *Result;

  Goal.Address = Address;
  Result = bsearch(&Goal, CodeLabels, CodeLabelCount, sizeof(struct CodeLabel), CodeLabelSort);
  if(Result == NULL)
    return -1;
  return Result - CodeLabels;
}

char *Tagfile_GetLabel(int TagIndex) {
  return CodeLabels[TagIndex].Text;
}

char *Tagfile_GetLabelComment(int TagIndex) {
  return CodeLabels[TagIndex].Comment;
}

int Tagfile_FindPGMAddress(int Address) {
  struct PGMLabel Goal;
  struct PGMLabel *Result;

  Goal.Address = Address;
  Result = bsearch(&Goal, PGMLabels, PGMLabelCount, sizeof(struct PGMLabel), PGMLabelSort);
  if(Result == NULL)
    return -1;
  return Result - PGMLabels;
}

const char *Tagfile_Resolve_Mem_Address(int Address) {
  for(int i = 0; i < MemLabelCount && MemLabels[i].Address <= Address; i++) {
    int Start = MemLabels[i].Address;
    int Size = MemLabels[i].Type == TYPE_WORD? 2: 1;
    int End = MemLabels[i].Address + MemLabels[i].Count * Size - 1;

    if(Address >= Start && Address <= End) {
      if(MemLabels[i].Count == 1) { // Single variable
        if(Size == 1)
          return str_ccprintf("%s", MemLabels[i].Comment);
        return str_ccprintf("_%s8(%s)", Address == Start? "lo": "hi", MemLabels[i].Comment);
      }
      // Array
      if(Size == 1)
        return str_ccprintf("%s[%d]", MemLabels[i].Comment, Address - Start);
      return str_ccprintf("_%s8(%s[%d])", (Address-Start)%2? "hi": "lo", MemLabels[i].Comment,
        (Address - Start)/2);
    }
  }

  return NULL;
}

static int Tagfile_Process_Byte(const char *Bitstream, int Position, int ArgumentNo, const char *Label) {
  printf(".byte 0x%02x\n", Bitstream[Position] & 0xff);
  return 1;
}

static int Tagfile_Process_Word(const char *Bitstream, int Position, int ArgumentNo, const char *Label) {
  printf(".word 0x%02x%02x\n", Bitstream[Position + 1] & 0xff, Bitstream[Position] & 0xff);
  return 2;
}

static int Tagfile_Process_String(const char *Bitstream, int Position, int ArgumentNo, const char *Label) {
  int i;
  unsigned char c;
  unsigned int InString = 0;

  printf("String_0x%s_%d:    ; Address 0x%x (%d)\n", Label, ArgumentNo, Position, Position);
  i = 0;
  while((c = Bitstream[Position + i])) {
    if((c >= 32) && (c <= 127)) {
      if(!InString)
        printf(".ascii \"");
      printf("%c", c);
      InString = 1;
    } else {
      if(InString)
        printf("\"\n");
      printf(".byte 0x%02x\n", c);
      InString = 0;
    }
    i++;
  }
  if(InString)
    printf("\\0\"\n");
  else
    printf(".byte 0x00\n");

  printf("\n");
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

int Tagfile_Process_Data(const char *Bitstream, int Position) {
  int i;
  int BytesAdvanced;
  int Index;
  int (*ProcessingFunction)(const char *, int, int, const char *) = NULL;
  char Buffer[32];

  Index = Tagfile_FindPGMAddress(Position);
  if(Index == -1)
    return 0;

  switch (PGMLabels[Index].Type) {
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

  printf("; Inline PGM data: %d ", PGMLabels[Index].Count);
  switch (PGMLabels[Index].Type) {
  case TYPE_BYTE:
    printf("byte");
    break;
  case TYPE_WORD:
    printf("word");
    break;
  case TYPE_ASTRING:
    printf("autoaligned string");
    break;
  case TYPE_STRING:
    printf("string");
    break;
  }
  if(PGMLabels[Index].Count != 1)
    printf("s");
  printf(" starting at 0x%x", Position);

  if(PGMLabels[Index].Comment != NULL) {
    printf(" (%s)", PGMLabels[Index].Comment);
  }
  printf("\n");

  if((PGMLabels[Index].Type == TYPE_ASTRING) || (PGMLabels[Index].Type == TYPE_STRING)) {
    if(PGMLabels[Index].Comment != NULL) {
      snprintf(Buffer, sizeof(Buffer), "%x_%s", Position, PGMLabels[Index].Comment);
      Sanitize_String(Buffer);
    } else {
      snprintf(Buffer, sizeof(Buffer), "%x", Position);
    }
  }

  BytesAdvanced = 0;
  for(i = 0; i < PGMLabels[Index].Count; i++) {
    BytesAdvanced += ProcessingFunction(Bitstream, Position + BytesAdvanced, i, Buffer);
  }

  if(PGMLabels[Index].Type == TYPE_ASTRING) {
    // Autoaligned string
    if((BytesAdvanced % 2) != 0) {
      // Not yet aligned correctly
      if(Bitstream[Position + BytesAdvanced] != 0x00)
        fprintf(stderr, "Warning in autoalignment: expected zero but got 0x%0x padding. Ignored.\n",
          ((unsigned char *) Bitstream)[Position + BytesAdvanced]);
      printf(".byte 0x%02x        ; String Autoalignment\n", ((unsigned char *) Bitstream)[Position + BytesAdvanced]);
      BytesAdvanced++;
    }
  }

  printf("\n");
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

// Initialise IORegisters and MemLabels from part register file
void initRegisters(const AVRPART *p) {
  int nr = 0, nio = 0, offset = 0;
  const Register_file *rf = avr_locate_register_file(p, &nr);

  if(rf) {
    if(IORegisters) {
      for(int i = 0; i < IORegistersCount; i++)
        mmt_free(IORegisters[i].Name);
      mmt_free(IORegisters);
    }
    if(MemLabels) {
      for(int i = 0; i < MemLabelCount; i++)
        mmt_free(MemLabels[i].Comment);
      mmt_free(MemLabels);
    }

    // Count how many entries are needed
    for(int i = 0; i< nr; i++)
      if(rf[i].addr < 0x40 && rf[i].size > 0)
        nio += rf[i].size;
    IORegisters = mmt_malloc(nio*sizeof*IORegisters);
    MemLabels = mmt_malloc(nr*sizeof*MemLabels);

    AVRMEM *mem = avr_locate_io(p);
    if(mem)
      offset = mem->offset;
    nio = 0;
    for(int i = 0; i< nr; i++) {
      MemLabels[i].Address = offset + rf[i].addr;
      MemLabels[i].Type = rf[i].size == 2? TYPE_WORD: TYPE_BYTE;
      MemLabels[i].Count = rf[i].size > 2? rf[i].size: 1;
      MemLabels[i].Comment = regname(rf[i].reg, -1);

      if(rf[i].addr < 0x40) {
        if(rf[i].size == 1) {
          IORegisters[nio].Name = regname(rf[i].reg, -1);
          IORegisters[nio].Address = rf[i].addr;
          nio++;
        } else if(rf[i].size == 2) {
          IORegisters[nio].Name = regname(rf[i].reg, 'l');
          IORegisters[nio].Address = rf[i].addr;
          nio++;
          IORegisters[nio].Name = regname(rf[i].reg, 'h');
          IORegisters[nio].Address = rf[i].addr+1;
          nio++;
        } else if(rf[i].size > 2) {
          for(int k = 0; k < rf[i].size; k++) {
            IORegisters[nio].Name = regname(rf[i].reg, k);
            IORegisters[nio].Address = rf[i].addr + k;
            nio++;
          }
        }
      }
    }
    IORegistersCount = nio;
    MemLabelCount = nr;
    qsort(MemLabels, MemLabelCount, sizeof(struct MemLabel), MemLabelSort);
  }
}

const char *Resolve_IO_Register(int Number) {
  for(int i = 0; i < IORegistersCount; i++) {
    if(IORegisters[i].Address == Number) {
      IORegisters[i].Used = 1;
      return IORegisters[i].Name;
    }
  }

  return NULL;
}

void Emit_Used_IO_Registers() {
  for(int i = 0; i < IORegistersCount; i++)
    if(IORegisters[i].Used)
      printf(".equ %s, 0x%x\n", IORegisters[i].Name, IORegisters[i].Address);
}
