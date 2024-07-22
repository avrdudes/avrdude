/*
 * AVRDUDE - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2024 by Stefan Rueger <stefan.rueger@urclocks.com>
 *
 * Based on avrdisas (but heavily rewritten)
 * Copyright (C) 2007 Johannes Bauer <JohannesBauer@gmx.de>
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "disasm_private.h"

// Wrap around flash
int disasm_wrap(int addr) {
  if(cx->dis_flashsz)
    addr &= cx->dis_flashsz-1;

  return addr;
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

char *add_comment(Disasm_line *line, const char *comment) {
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

void disassemble(const char *buf, int addr, int opcode, AVR_opcode mnemo, Disasm_line *line, int pass) {
  const AVR_opcode_data *oc = avr_opcodes+mnemo;
  memset(line, 0, sizeof*line);

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
  *line->comment = 0;

  // Check for opcodes with undefined results
  switch(oc->type & OTY_WARN_MASK) {
  case OTY_XWRN:
    if(Rd == 26 || Rd == 27 || Rr == 26 || Rr == 27)
      add_comment(line, "Warning: the result of this operation is undefined\n");
    break;
  case OTY_YWRN:
    if(Rd == 28 || Rd == 29 || Rr == 28 || Rr == 29)
      add_comment(line, "Warning: the result of this operation is undefined\n");
    break;
  case OTY_ZWRN:
    if(Rd == 30 || Rd == 31 || Rr == 30 || Rr == 31)
      add_comment(line, "Warning: the result of this operation is undefined\n");
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
      Register_JumpCall(addr, target, mnemo, 0);
    is_jumpcall = 1;
    is_relative = 1;
    break;                      // rjmp/rcall
  case 12:
    offset = (int16_t) (Rk<<4) >> 3; // Sign extend and multiply by 2
    target = disasm_wrap(addr + offset + 2);
    if(pass == 1)
      Register_JumpCall(addr, target, mnemo, is_function);
    is_jumpcall = 1;
    is_relative = 1;
    break;
  case 16:                      // lds/sts
    kmemaddr = Tagfile_Resolve_Mem_Address(Rk);
    break;
  case 22:
    target = disasm_wrap(2*Rk);
    if(pass == 1)
      Register_JumpCall(addr, target, mnemo, is_function);
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
      if((regname = Resolve_IO_Register(RA)))
        snprintf(c, 256-strlen(c), "%s", regname);
      else
        snprintf(c, 256-strlen(c), "0x%02x", RA);
      c += strlen(c);
      break;
    case 'a':
      snprintf(c, 255 - (c-line->code), "0x%02x", Ra);
      if((memaddr = Tagfile_Resolve_Mem_Address(Ra)))
        add_comment(line, str_ccprintf("%s", memaddr));
      break;
    case 'k':
      if(is_jumpcall) {
        if(cx->dis_opts.Process_Labels) {
          snprintf(c, 255 - (c-line->code), "%s", Get_Label_Name(target, NULL));
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
  // Trim trailing spaces
  while(--c >= line->code && *c == ' ')
    *c = 0;
}

/*
 * Disassemble buflen bytes at buf which corresponds to address addr
 *
 * Before the location buf there are leadin bytes available (0 - 2)
 * After the location buf+readlen there are leadout bytes available (0 -4)
 */
int disasm(const char *buf, int buflen, int addr, int leadin, int leadout) {
  int Pos;
  int opcode, mnemo, oplen;
  int i;
  Disasm_line line;

  Pos = 0;

  for(int i = 0; i < cx->dis_IORegisterN; i++)
    cx->dis_IORegisters[i].Used = 0;

  if(cx->dis_opts.Process_Labels || cx->dis_opts.avrgcc_style) {
    // Preprocess to gather jump labels or to gain knowledge about registers which are being used
    while(Pos < buflen) {
      opcode = (buf[Pos] & 0xff) | (buf[Pos+1] & 0xff)<<8;
      mnemo = opcode_mnemo(opcode, cx->dis_opts.avrlevel);
      oplen = 2*avr_opcodes[mnemo].nwords;
      if(mnemo == -1) {
        Pos += 2;
      } else {
        disassemble(buf + Pos, disasm_wrap(Pos + addr), opcode, mnemo, &line, 1);
        Pos += oplen;
      }
    }
    Enumerate_Labels();
    Pos = 0;
  }

  if(cx->dis_opts.avrgcc_style)
    Emit_Used_IO_Registers();

  while(Pos < buflen) {
    // Check if this is actually code or maybe only data from tagfile
    int Added = Tagfile_Process_Data(buf, Pos, addr);
    if(Added) {
      Pos += Added;
      continue;
    }

    opcode = (buf[Pos] & 0xff) | (buf[Pos+1] & 0xff)<<8;
    mnemo = opcode_mnemo(opcode, cx->dis_opts.avrlevel);
    oplen = 2*avr_opcodes[mnemo].nwords;

    if(mnemo != -1) {
      disassemble(buf + Pos, disasm_wrap(Pos + addr), opcode, mnemo, &line, 2);

      if(cx->dis_opts.Process_Labels)
        Print_JumpCalls(disasm_wrap(Pos + addr));

      if(cx->dis_opts.Show_Addresses)
        term_out("%4x:   ", disasm_wrap(Pos + addr));
      if(cx->dis_opts.Show_Cycles)
        term_out("[%-3s] ", avr_opcodes[mnemo].clock[cx->dis_cycle_index]);

      if(cx->dis_opts.Show_Opcodes) {
        // Now display the Opcode
        for(i = 0; i < oplen; i++)
          term_out("%02x ", buf[Pos + i] & 0xff);
        term_out(" ");
        for(i = 0; i < 5 - oplen; i++)
          term_out("   ");
      }

      if(line.code[0] == 0) {
        // No code was generated?
        term_out("; opcode %s not implemented\n", avr_opcodes[mnemo].idname);
      } else {
        if(!line.comment[0] || !cx->dis_opts.Show_Comments) {
          term_out("%s\n", line.code);
        } else {
          term_out("%-23s ; %s\n", line.code, line.comment);
        }
      }
      if(mnemo == OPCODE_ret || mnemo == OPCODE_u_ret) // @@@
        term_out("\n");

      Pos += oplen;
    } else {
      term_out(".word 0x%02x%02x    ; Invalid opcode\n", buf[Pos+1] & 0xff, buf[Pos] & 0xff);
      Pos += 2;
    }
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

  cx->dis_flashsz = 0;        // Flash size rounded up to next power of two
  cx->dis_addrwidth = 4;      // Number of hex digits needed for flash addresses
  cx->dis_sramwidth = 3;      // Number of hex digits needed for sram addresses

  if((mem = avr_locate_flash(p)) && mem->size > 1) {
    int nbits = intlog2(mem->size - 1) + 1;
    cx->dis_flashsz = 1 << nbits;
    cx->dis_addrwidth = (nbits+3)/4;
  }

  if((mem = avr_locate_sram(p)) && mem->size > 1) {
    int size = mem->size;
    if(mem->offset > 0 && mem->offset <= 0x200)
      size += mem->offset;
    cx->dis_sramwidth = (intlog2(size - 1) + 1 + 3)/4;
  }

  cx->dis_cycle_index = avr_get_cycle_index(p);

  disasm_init_regfile(p);
  return 0;
}
