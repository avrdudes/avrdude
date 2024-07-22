/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2007 Johannes Bauer <JohannesBauer@gmx.de>
 * Copyright (C) 2024 by Stefan Rueger <stefan.rueger@urclocks.com>
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

/*
 * The disassembly code originates from the avrdisas disassembler written in
 * 2007 by Johannes Bauer. The current code has been ported by Stefan Rueger to
 *   - Enable disassembly of small memory chunks in AVRDUDE's terminal
 *   - Drive disassembly from the avr_opcode[] table
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "avrdude.h"
#include "libavrdude.h"

#define TYPE_BYTE       1
#define TYPE_WORD       2
#define TYPE_ASTRING    3
#define TYPE_STRING     4


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

static int find_symbol(int type, int address) {
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
static void init_regfile(const AVRPART *p) {
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

int disasm_init_tagfile(const AVRPART *p, const char *fname) {
  FILE *inf = fopen(fname, "r");
  int lineno = 1;
  const char *errstr;

  if(!inf) {
    pmsg_ext_error("cannot open tagfile %s: %s\n", fname, strerror(errno));
    return -1;
  }

  zap_symbols();
  init_regfile(p);

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

static char *Tagfile_GetLabel(int index) {
  return cx->dis_symbols[index].name;
}

static char *Tagfile_GetLabelComment(int index) {
  return cx->dis_symbols[index].comment;
}

static const char *resolve_mem_address(int address) {
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

static int process_byte(const char *Bitstream, int Position, int offset, int ArgumentNo, const char *Label) {
  term_out(".byte 0x%02x\n", Bitstream[Position] & 0xff);
  return 1;
}

static int process_word(const char *Bitstream, int Position, int offset, int ArgumentNo, const char *Label) {
  term_out(".word 0x%02x%02x\n", Bitstream[Position + 1] & 0xff, Bitstream[Position] & 0xff);
  return 2;
}

static int process_string(const char *Bitstream, int Position, int offset, int ArgumentNo, const char *Label) {
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

// Wrap around flash
static int disasm_wrap(int addr) {
  if(cx->dis_flashsz2)
    addr &= cx->dis_flashsz2-1;

  return addr;
}

static int process_data(const char *Bitstream, int Position, int offset) {
  int BytesAdvanced;
  int (*ProcessingFunction)(const char *, int, int, int, const char *) = NULL;
  char Buffer[32];

  int index = find_symbol('P', disasm_wrap(Position + offset));
  if(index < 0)
    return 0;

  switch(cx->dis_symbols[index].subtype) {
  case TYPE_BYTE:
    ProcessingFunction = process_byte;
    break;
  case TYPE_WORD:
    ProcessingFunction = process_word;
    break;
  case TYPE_ASTRING:
    ProcessingFunction = process_string;
    break;
  case TYPE_STRING:
    ProcessingFunction = process_string;
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

static const char *resolve_io_register(int Number) {
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

void emit_used_io_registers() {
  for(int i = 0; i < cx->dis_symbolN; i++)
    if(cx->dis_symbols[i].used)
      term_out(".equ %s, 0x%02x\n", cx->dis_symbols[i].name, cx->dis_symbols[i].address);
}


void disasm_zap_jumpcalls() {
  if(cx->dis_JumpCalls) {
    mmt_free(cx->dis_JumpCalls);
    cx->dis_JumpCalls = NULL;
  }
  cx->dis_JumpCallN = 0;
}

static void register_jumpcall(int From, int To, int mnemo, unsigned char FunctionCall) {
  if(cx->dis_opts.process_labels) {
    Disasm_JumpCall *jc = cx->dis_JumpCalls;
    int N = cx->dis_JumpCallN;

    // Already entered this JC?
    for(int i = 0; i < N; i++)
      if(jc[i].From == From && jc[N].To == To && jc[N].mnemo == mnemo)
        return;

    jc = mmt_realloc(jc, sizeof(Disasm_JumpCall) * (N+1));
    jc[N].From = From;
    jc[N].To = To;
    jc[N].mnemo = mnemo;
    jc[N].LabelNumber = 0;
    jc[N].FunctionCall = FunctionCall;

    cx->dis_JumpCalls = jc;
    cx->dis_JumpCallN++;
  }
}

static int JC_Comparison(const void *Element1, const void *Element2) {
  Disasm_JumpCall *JC1, *JC2;

  JC1 = (Disasm_JumpCall *) Element1;
  JC2 = (Disasm_JumpCall *) Element2;
  if((JC1->To) > (JC2->To))
    return 1;
  else if((JC1->To) == (JC2->To))
    return 0;
  return -1;
}

static void Correct_Label_IsFunct(void) {
  int i, j;
  int LastIdx = 0;
  int LastDest = cx->dis_JumpCalls[0].To;
  char CurIsFunct = cx->dis_JumpCalls[0].FunctionCall;

  for(i = 1; i < cx->dis_JumpCallN; i++) {
    if(cx->dis_JumpCalls[i].To != LastDest) {
      for(j = LastIdx; j < i; j++)
        cx->dis_JumpCalls[j].FunctionCall = CurIsFunct;
      LastIdx = i;
      LastDest = cx->dis_JumpCalls[i].To;
      CurIsFunct = 0;
    }
    CurIsFunct = CurIsFunct || cx->dis_JumpCalls[i].FunctionCall;
  }
  for(j = LastIdx; j < cx->dis_JumpCallN; j++)
    cx->dis_JumpCalls[j].FunctionCall = CurIsFunct;
}

static void enumerate_labels(void) {
  int i;
  int CurrentLabelNumber = 0;
  int CurrentFunctionNumber = 0;
  int Destination;

  if(cx->dis_JumpCallN < 2)
    return;

  qsort(cx->dis_JumpCalls, cx->dis_JumpCallN, sizeof(Disasm_JumpCall), JC_Comparison);
  Correct_Label_IsFunct();

  Destination = cx->dis_JumpCalls[0].To;
  if(cx->dis_JumpCalls[0].FunctionCall)
    CurrentFunctionNumber++;
  else
    CurrentLabelNumber++;
  for(i = 0; i < cx->dis_JumpCallN; i++) {
    if(Destination != cx->dis_JumpCalls[i].To) {
      if(cx->dis_JumpCalls[i].FunctionCall)
        CurrentFunctionNumber++;
      else
        CurrentLabelNumber++;
      Destination = cx->dis_JumpCalls[i].To;
    }
    if(cx->dis_JumpCalls[i].FunctionCall)
      cx->dis_JumpCalls[i].LabelNumber = CurrentFunctionNumber;
    else
      cx->dis_JumpCalls[i].LabelNumber = CurrentLabelNumber;
  }
}

static const char *get_label_name(int destination, char **comment) {
  int index = find_symbol('L', destination);
  if(index >= 0) {
    if(comment)
      *comment = Tagfile_GetLabelComment(index);
    return str_ccprintf("%s", Tagfile_GetLabel(index));
  }

  for(int i = 0; i < cx->dis_JumpCallN; i++)
    if(cx->dis_JumpCalls[i].To == destination)
      return str_ccprintf("%s%d", cx->dis_JumpCalls[i].FunctionCall? "Function": "Label", cx->dis_JumpCalls[i].LabelNumber);

  return "unknown";
}

// Show all references which refer to "Position" as destination
static void print_jumpcalls(int Position) {
  int i;
  int Match = 0;

  for(i = 0; i < cx->dis_JumpCallN; i++) {
    if((cx->dis_JumpCalls[i].To) == Position) {
      if(Match == 0) {
        term_out("\n");
        Match = 1;
      }
      term_out("; Referenced from 0x%0*x by %s\n", cx->dis_addrwidth, 
        cx->dis_JumpCalls[i].From, avr_opcodes[cx->dis_JumpCalls[i].mnemo].opcode);
    }
  }
  if(Match == 1) {
    char *LabelComment = NULL;
    const char *LabelName = get_label_name(Position, &LabelComment);
    if(LabelComment == NULL) {
      term_out("%s:\n", LabelName);
    } else {
      term_out("%-23s ; %s\n", str_ccprintf("%s:", LabelName), LabelComment);
    }
  }
}

typedef struct {
  char code[256], comment[256];
} Disasm_line;

#define Ra (regs['a'])
#define Rd (regs['d'])
#define Rr (regs['r'])
#define Rk (regs['k'])
#define RK (regs['K'])
#define Rs (regs['s'])
#define RA (regs['A'])
#define Rb (regs['b'])
#define Rq (regs['q'])

#define Na (bits['a'])
#define Nd (bits['d'])
#define Nr (bits['r'])
#define Nk (bits['k'])
#define NK (bits['K'])
#define Ns (bits['s'])
#define NA (bits['A'])
#define Nb (bits['b'])
#define Nq (bits['q'])

static char *add_comment(Disasm_line *line, const char *comment) {
  int len = strlen(line->comment), rem = 256-len-1;
  char *p = line->comment + len;

  if(len && *comment && rem > 2)
    strcpy(p, ", "), p += 2, rem -= 2;
  strncpy(p, comment, rem);
  p[rem] = 0;

  return p + strlen(p);
}

static const char *regstyle(int n, int regword) {
  if(regword && !cx->dis_opts.avrgcc_style)
    return str_ccprintf("%d:%d", n+1, n);
  return str_ccprintf("%d", n);
}

// Return the number of bits set in Number
static unsigned bitcount(unsigned n) {
  unsigned ret;

  // A la Kernighan (and Richie): iteratively clear the least significant bit set
  for(ret = 0; n; ret++)
    n &= n-1;

  return ret;
}

static void disassemble(const char *buf, int addr, int opcode, AVR_opcode mnemo, Disasm_line *line, int pass) {
  memset(line, 0, sizeof*line);
  if(mnemo < 0) {
    add_comment(line, "Invalid opcode");
    snprintf(line->code, 256, ".word   0x%02x%02x", buf[1] & 0xff, buf[0] & 0xff);
    return;
  }

  const AVR_opcode_data *oc = avr_opcodes+mnemo;
  int regs[128] = {0}, bits[128] = {0};
  unsigned bmask = 0x8000;
  for(const char *p = oc->bits; *p && bmask; p++) {
    if(*p == ' ')
      continue;
    bits[*p & 0x7f]++;
    regs[*p & 0x7f] <<= 1;
    regs[*p & 0x7f] |= !!(opcode & bmask);
    bmask >>= 1;
  }

  // Treat 32 bit opcodes
  if(oc->nwords == 2) {
    bits['k'] += 16;
    regs['k'] <<= 16;
    regs['k'] |= (buf[2] & 0xff) | (buf[3] & 0xff)<<8;
  }

  // Some sanity checks for things the code relies on
  if(NA && NA != 5 && NA != 6)
    pmsg_warning("unexpected number of A bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Na && Na != 7)
    pmsg_warning("unexpected number of a bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nb && Nb != 3)
    pmsg_warning("unexpected number of b bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nk && Nk != 7 && Nk != 12 && Nk != 16 && Nk != 22)
    pmsg_warning("unexpected number of k bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(NK && NK != 4 && NK != 6 && NK != 8)
    pmsg_warning("unexpected number of  bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nq && Nq != 6)
    pmsg_warning("unexpected number of q bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nd && (Nd < 2 || Nd > 5))
    pmsg_warning("unexpected number of Rd bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nr && (Nr < 3 || Nr > 5))
    pmsg_warning("unexpected number of Rr bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Ns && Ns != 3)
    pmsg_warning("unexpected number of s bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);

  switch(mnemo) {               // Exceptions go here
  case OPCODE_andi:             // cbr r17, 0x06 is marginally easier to read than andi r17, 0xf9
    if(bitcount(RK) >= 4) {
      RK = ~RK & 0xff;
      mnemo = OPCODE_cbr;
      oc = avr_opcodes+mnemo;
    }
  default:
    break;
  }

  // Apply register formula
  int regword = 0;
  switch(oc->type & OTY_REG_MASK) {
  case OTY_REVN:                // Even registers r0, r2, ..., r30
    Rd *= 2, Rr *= 2;
    regword = 1;                // movw
    break;
  case OTY_RUPP:                // Upper registers only r16, ..., r31
    Rd += 16, Rr += 16;
    break;
  case OTY_RW24:                // r24, r26, r28, r30 only
    Rd = 2*Rd + 24;
    regword = 1;                // adiw, sbiw
    break;
  }

  if(Na) {
    /*
     * Address is limited to 0x40...0xbf for the reduced-core (TPI part)
     * ADDR[7:0] ← (/INST[8], INST[8], INST[10], INST[9], INST[3], INST[2], INST[1], INST[0])
     * ADDR[7:0] ← (/a[4], a[4], a[6], a[5], a[3], a[2], a[1], a[0])
     */
    Ra = (Ra & 0xf) | ((Ra >> 1) & 0x30) | ((Ra & 0x10) << 2) | (((Ra & 0x10) ^ 0x10) << 3);
  }

  int awd = cx->dis_addrwidth, swd = cx->dis_sramwidth;
  snprintf(line->code, 256, "%-7s ", oc->opcode);
  char *c = line->code + strlen(line->code);

  // Check for opcodes with undefined results
  switch(oc->type & OTY_WARN_MASK) {
  case OTY_XWRN:
    if(Rd == 26 || Rd == 27 || Rr == 26 || Rr == 27)
      add_comment(line, "Warning: the result of this operation is undefined");
    break;
  case OTY_YWRN:
    if(Rd == 28 || Rd == 29 || Rr == 28 || Rr == 29)
      add_comment(line, "Warning: the result of this operation is undefined");
    break;
  case OTY_ZWRN:
    if(Rd == 30 || Rd == 31 || Rr == 30 || Rr == 31)
      add_comment(line, "Warning: the result of this operation is undefined");
    break;
  }

  int target = 0, offset = 0, is_jumpcall = 0, is_relative = 0;
  int is_function = !!(oc->type & OTY_EXTERNAL); // call/rcall affects stack memory
  const char *kmemaddr = NULL, *memaddr, *regname;

  switch(Nk) {
  case 0:
    break;
  case 7:                       // Branches
    offset = (int8_t) (Rk<<1);  // Sign-extend and multiply by 2
    target = disasm_wrap(addr + offset + 2);
    if(pass == 1)
      register_jumpcall(addr, target, mnemo, 0);
    is_jumpcall = 1;
    is_relative = 1;
    break;                      // rjmp/rcall
  case 12:
    offset = (int16_t) (Rk<<4) >> 3; // Sign extend and multiply by 2
    target = disasm_wrap(addr + offset + 2);
    if(pass == 1)
      register_jumpcall(addr, target, mnemo, is_function);
    is_jumpcall = 1;
    is_relative = 1;
    break;
  case 16:                      // lds/sts
    kmemaddr = resolve_mem_address(Rk);
    break;
  case 22:
    if(cx->dis_flashsz && 2*Rk > cx->dis_flashsz)
      add_comment(line,
        str_ccprintf("Warning: destination outside flash [0, 0x%0*x]", awd, cx->dis_flashsz-1));
    target = 2*Rk;              // disasm_wrap(2*Rk);
    if(pass == 1)
      register_jumpcall(addr, target, mnemo, is_function);
    is_jumpcall = 1;
    break;
  }

  for(const char *o = oc->operands; *o && c-line->code < 255; o++) {
    switch(*o) {
    case 'R':
      *c++ = 'r', *c = 0;
      break;
    default:
      *c++ = *o, *c = 0;
      break;
    case 'A':
      if((regname = resolve_io_register(RA)))
        snprintf(c, 256-strlen(c), "%s", regname);
      else
        snprintf(c, 256-strlen(c), "0x%02x", RA);
      c += strlen(c);
      break;
    case 'a':
      snprintf(c, 255 - (c-line->code), "0x%02x", Ra);
      if((memaddr = resolve_mem_address(Ra)))
        add_comment(line, str_ccprintf("%s", memaddr));
      break;
    case 'k':
      if(is_jumpcall) {
        if(cx->dis_opts.process_labels) {
          snprintf(c, 255 - (c-line->code), "%s", get_label_name(target, NULL));
          add_comment(line, str_ccprintf("0x%0*x", awd, target));
        } else {
          if(is_relative) {
            snprintf(c, 255 - (c-line->code), ".%+d", offset);
            add_comment(line, str_ccprintf("0x%0*x", awd, target));
          } else
            snprintf(c, 255 - (c-line->code), "0x%0*x", awd, target);
        }
      } else {
        snprintf(c, 255 - (c-line->code), "0x%0*x", swd, Rk);
        if(kmemaddr)
          add_comment(line, str_ccprintf("%s", kmemaddr));
      }
      break;
    case 'b':
      snprintf(c, 255 - (c-line->code), "%d", Rb);
      add_comment(line, str_ccprintf("bit %d = 0x%02x", Rb, 1 << Rb));
      break;
    case 's':
      snprintf(c, 255 - (c-line->code), "%d", Rs);
      break;
    case 'd':
      snprintf(c, 255 - (c-line->code), "%s", regstyle(Rd, regword));
      break;
    case 'r':
      snprintf(c, 255 - (c-line->code), "%s", regstyle(Rr, regword));
      break;
    case 'K':
      if(NK == 4)
        snprintf(c, 255 - (c-line->code), "%d", RK);
      else {
        snprintf(c, 255 - (c-line->code), "0x%02x", RK);
        add_comment(line, str_ccprintf("%d", RK));
      }
      break;
    case 'q':        snprintf(c, 255 - (c-line->code), "%d", Rq);
      break;
    }
    c += strlen(c);
  }
  if(cx->dis_opts.show_name)
    add_comment(line, avr_opcodes[mnemo].description);
  if(cx->dis_opts.show_explanation)
    add_comment(line, avr_opcodes[mnemo].operation);
  // Trim trailing spaces
  while(--c >= line->code && *c == ' ')
    *c = 0;
}


// Increase cycle number by 1 if it's a 3 byte PC
static const char *cycles(int mnemo) {
  if(mnemo < 0)
    return "---";

  const char *ret = avr_opcodes[mnemo].clock[cx->dis_cycle_index];

  // A plus sign after the cycle number means add one for 3-byte PC
  if(*ret && ret[1] == '+')
    return str_ccprintf("%c", cx->dis_flashsz > 128*1024? *ret+1: *ret);

  return ret;
}

/*
 * Disassemble buflen bytes at buf which corresponds to address addr
 *
 * Before(!) the location buf there are leadin bytes available (0 - 2)
 * After the location buf+readlen there are leadout bytes available (0 -4)
 */
int disasm(const char *buf, int buflen, int addr, int leadin, int leadout) {
  int pos, opcode, mnemo, oplen;
  Disasm_line line;
  int awd = cx->dis_addrwidth;

  pos = 0;
  for(int i = 0; i < cx->dis_symbolN; i++)
    if(cx->dis_symbols[i].type == 'I')
      cx->dis_symbols[i].used = 0;

  if(cx->dis_opts.process_labels || cx->dis_opts.avrgcc_style) {
    // Preprocess to gather jump labels or to gain knowledge about registers which are being used
    while(pos < buflen) {
      opcode = (buf[pos] & 0xff) | (buf[pos+1] & 0xff)<<8;
      mnemo = opcode_mnemo(opcode, cx->dis_opts.avrlevel);
      disassemble(buf + pos, disasm_wrap(pos + addr), opcode, mnemo, &line, 1);
      pos += mnemo < 0? 2: 2*avr_opcodes[mnemo].nwords;
    }
    enumerate_labels();
    pos = 0;
  }

  if(cx->dis_opts.avrgcc_style)
    emit_used_io_registers();

  while(pos < buflen) {
    // Check if this is actually code or maybe only data from tagfile
    int added = process_data(buf, pos, addr);
    if(added) {
      pos += added;
      continue;
    }

    opcode = (buf[pos] & 0xff) | (buf[pos+1] & 0xff)<<8;
    mnemo = opcode_mnemo(opcode, cx->dis_opts.avrlevel);
    oplen = mnemo < 0? 2: 2*avr_opcodes[mnemo].nwords;

    disassemble(buf + pos, disasm_wrap(pos + addr), opcode, mnemo, &line, 2);

    if(cx->dis_opts.process_labels)
      print_jumpcalls(disasm_wrap(pos + addr));

    if(cx->dis_opts.show_addresses)
      term_out("%*x: ", awd, disasm_wrap(pos + addr));
    if(cx->dis_opts.show_cycles || cx->dis_opts.show_flags) {
      if(cx->dis_opts.show_flags)
        term_out("%s ", mnemo < 0? "--------": avr_opcodes[mnemo].flags);
      if(cx->dis_opts.show_cycles)
        term_out("%3s ", cycles(mnemo));
    }

    if(cx->dis_opts.show_opcodes) {
      for(int i = 0; i < 4; i++)
        term_out(i < oplen? "%02x ": "   ", buf[pos + i] & 0xff);
      term_out(" ");
    }

    if(!*line.comment || !cx->dis_opts.show_comments)
      term_out("%s\n", line.code);
    else
      term_out("%-27s ; %s\n", line.code, line.comment);
    if(mnemo == OPCODE_ret || mnemo == OPCODE_u_ret || mnemo == OPCODE_ret || mnemo == OPCODE_u_ret)
      term_out("\n");

    pos += oplen;
  }

  return 0;
}

int disasm_init(const AVRPART *p) {
  AVRMEM *mem;

  // Sanity check (problems only occur if avr_opcodes was changed)
  for(size_t i = 0; i < sizeof avr_opcodes/sizeof*avr_opcodes; i++)
    if(avr_opcodes[i].mnemo != (AVR_opcode) i) {
      msg_error("avr_opcodes[] table broken (this should never happen)\n");
      return -1;
    }

  cx->dis_flashsz = 0;        // Flash size
  cx->dis_flashsz2 = 0;       // Flash size rounded up to next power of two
  cx->dis_addrwidth = 4;      // Number of hex digits needed for flash addresses
  cx->dis_sramwidth = 3;      // Number of hex digits needed for sram addresses

  if((mem = avr_locate_flash(p)) && mem->size > 1) {
    int nbits = intlog2(mem->size - 1) + 1;
    cx->dis_flashsz = mem->size;
    cx->dis_flashsz2 = 1 << nbits;
    cx->dis_addrwidth = (nbits+3)/4;
  }

  if((mem = avr_locate_sram(p)) && mem->size > 1) {
    int size = mem->size;
    if(mem->offset > 0 && mem->offset <= 0x200)
      size += mem->offset;
    cx->dis_sramwidth = (intlog2(size - 1) + 1 + 3)/4;
  }

  cx->dis_cycle_index = avr_get_cycle_index(p);

  init_regfile(p);
  return 0;
}
