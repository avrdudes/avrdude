
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
#include <string.h>

#include "libavrdude.h"

#include "disasm_globals.h"
#include "disasm_callbacks_pseudocode.h"
#include "disasm_jumpcall.h"
#include "disasm_ioregisters.h"
#include "disasm_tagfile.h"

void PC_Operation_Simple(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%s", avr_opcodes[mnemo].opcode);
}

void PC_Operation_Rd(AVR_opcode mnemo) {
  switch (mnemo) {
  case OPCODE_lsl:
    snprintf(cx->dis_code, 255, "r%d <<= 1;", Rd);
    snprintf(cx->dis_comment, 255, "_BV(0) = 0 (logical shift)");
    break;
  case OPCODE_lsr:
    snprintf(cx->dis_code, 255, "r%d >>= 1;", Rd);
    snprintf(cx->dis_comment, 255, "_BV(7) = 0, Carry = _BV(0) (logical shift)");
    break;
  case OPCODE_ror:
    snprintf(cx->dis_code, 255, "r%d >>= 1;", Rd);
    snprintf(cx->dis_comment, 255, "_BV(7) = Carry, Carry = _BV(0) (rotate right)");
    break;
  case OPCODE_asr:

    // snprintf(cx->dis_code, 255, "r%d >>= 1;", Rd);

    // snprintf(cx->dis_comment, 255, "_BV(7) = Sign (Arithmetic Shift)");
    snprintf(cx->dis_code, 255, "r%d /= 2;", Rd);
    snprintf(cx->dis_comment, 255, "arithmetic shift right");
    break;
  case OPCODE_swap:
    snprintf(cx->dis_code, 255, "r%d = ((r%d & 0xf0) >> 4) | ((r%d & 0x0f) << 4);", Rd, Rd, Rd);
    snprintf(cx->dis_comment, 255, "swap nibbles");
    break;
  case OPCODE_clr:
    snprintf(cx->dis_code, 255, "r%d = 0x00;", Rd);
    snprintf(cx->dis_comment, 255, "0");
    break;
  default:
    snprintf(cx->dis_code, 255, "%-7s r%d", avr_opcodes[mnemo].opcode, Rd);
  }
}

void PC_Operation_Rd16(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d", avr_opcodes[mnemo].opcode, Rd + 16);
}

void PC_Operation_Rd_Rr(AVR_opcode mnemo) {
  switch (mnemo) {
  case OPCODE_add:
    if(Rd != Rr) {
      snprintf(cx->dis_code, 255, "r%d += r%d;", Rd, Rr);
      snprintf(cx->dis_comment, 255, "No carry");
    } else {
      snprintf(cx->dis_code, 255, "r%d *= 2;", Rd);
    }
    break;
  case OPCODE_adc:
    snprintf(cx->dis_code, 255, "r%d += r%d;", Rd, Rr);
    snprintf(cx->dis_comment, 255, "With carry");
    break;
  case OPCODE_sub:
    snprintf(cx->dis_code, 255, "r%d -= r%d;", Rd, Rr);
    snprintf(cx->dis_comment, 255, "No carry");
    break;
  case OPCODE_sbc:
    snprintf(cx->dis_code, 255, "r%d += r%d;", Rd, Rr);
    snprintf(cx->dis_comment, 255, "With carry");
    break;
  case OPCODE_mul:
    snprintf(cx->dis_code, 255, "r1:r0 = r%d * r%d;", Rd, Rr);
    snprintf(cx->dis_comment, 255, "Unsigned");
    break;
  case OPCODE_mov:
    snprintf(cx->dis_code, 255, "r%d = r%d;", Rd, Rr);
    break;
  case OPCODE_eor:
    snprintf(cx->dis_code, 255, "r%d ^= r%d;", Rd, Rr);
    break;
  case OPCODE_and:
    if(Rd != Rr) {
      snprintf(cx->dis_code, 255, "r%d &= r%d;", Rd, Rr);
    } else {
      snprintf(cx->dis_code, 255, "(r%d == 0) || (r%d < 0);", Rd, Rd);
      snprintf(cx->dis_comment, 255, "test r%d", Rd);
    }
    break;
  case OPCODE_or:
    snprintf(cx->dis_code, 255, "r%d |= r%d;", Rd, Rr);
    break;
  case OPCODE_cp:
    snprintf(cx->dis_code, 255, "cmp(r%d, r%d);", Rd, Rr);
    break;
  case OPCODE_cpc:
    snprintf(cx->dis_code, 255, "cmp(r%d, r%d);", Rd, Rr);
    snprintf(cx->dis_comment, 255, "with carry");
    break;
  case OPCODE_cpse:
    snprintf(cx->dis_code, 255, "skipif (r%d == r%d)", Rd, Rr);
    break;
  default:
    snprintf(cx->dis_code, 255, "%-7s r%d, r%d", avr_opcodes[mnemo].opcode, Rd, Rr);
  }
}

void PC_Operation_Rd16_Rr16(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d, r%d", avr_opcodes[mnemo].opcode, Rd + 16, Rr + 16);
}

void PC_Operation_Rd16_K(AVR_opcode mnemo) {
  switch (mnemo) {
  case OPCODE_andi:
    snprintf(cx->dis_code, 255, "r%d &= %d;", Rd + 16, RK);
    snprintf(cx->dis_comment, 255, "0x%02x", RK);
    break;
  case OPCODE_subi:
    snprintf(cx->dis_code, 255, "r%d -= %d;", Rd + 16, RK);
    snprintf(cx->dis_comment, 255, "0x%02x, no carry", RK);
    break;
  case OPCODE_sbci:
    snprintf(cx->dis_code, 255, "r%d -= %d;", Rd + 16, RK);
    snprintf(cx->dis_comment, 255, "0x%02x, with carry", RK);
    break;
  case OPCODE_sbr:
  case OPCODE_ori:
    snprintf(cx->dis_code, 255, "r%d |= %d;", Rd + 16, RK);
    snprintf(cx->dis_comment, 255, "0x%02x", RK);
    break;
  default:
    snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, Rd + 16, RK);
    snprintf(cx->dis_comment, 255, "%d", RK);
  }
}

void PC_Operation_Rd_K(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, Rd, RK);
  snprintf(cx->dis_comment, 255, "%d", RK);
}

void PC_Operation_RdW_K(AVR_opcode mnemo) {
  if(cx->dis_opts.CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
    snprintf(cx->dis_code, 255, "%-7s r%d:%d, 0x%02x", avr_opcodes[mnemo].opcode, Rd + 1, Rd, RK);
  } else {
    snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, Rd, RK);
  }
  snprintf(cx->dis_comment, 255, "%d", RK);
}

void PC_Operation_RdW_RrW(AVR_opcode mnemo) {
  if(cx->dis_opts.CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
    snprintf(cx->dis_code, 255, "%-7s r%d:%d, r%d:%d", avr_opcodes[mnemo].opcode, (2 * Rd) + 1, 2 * Rd, (2 * Rr) + 1, 2 * Rr);
  } else {
    snprintf(cx->dis_code, 255, "%-7s r%d, r%d", avr_opcodes[mnemo].opcode, 2 * Rd, 2 * Rr);
  }
}

void PC_Operation_s_k(AVR_opcode mnemo, int Position) {
  int Bits, Offset;

  Bits = Rs;
  Offset = (2 * Rk);
  if(Offset > 128)
    Offset -= 256;

  int Target = FixTargetAddress(Position + Offset + 2);

  Register_JumpCall(Position, Target, mnemo, 0);
  if(cx->dis_opts.Process_Labels == 0) {
    if(Offset > 0) {
      snprintf(cx->dis_code, 255, "%-7s %d, .+%d", avr_opcodes[mnemo].opcode, Bits, Offset);
    } else {
      snprintf(cx->dis_code, 255, "%-7s %d, .%d", avr_opcodes[mnemo].opcode, Bits, Offset);
    }
    snprintf(cx->dis_comment, 255, "0x%02x = %d -> 0x%02x", (1 << Bits), (1 << Bits), Target);
  } else {
    snprintf(cx->dis_code, 255, "%-7s %d, %s", avr_opcodes[mnemo].opcode, Bits, Get_Label_Name(Target, NULL));
    snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bits), (1 << Bits));
  }
}

void PC_Operation_r_b(AVR_opcode mnemo) {
  int Register, Bit;

  Register = Rr;
  Bit = Rb;
  snprintf(cx->dis_code, 255, "%-7s r%d, %d", avr_opcodes[mnemo].opcode, Register, Bit);
  snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_Rd_b(AVR_opcode mnemo) {
  int Register, Bit;

  Register = Rd;
  Bit = Rb;
  snprintf(cx->dis_code, 255, "%-7s r%d, %d", avr_opcodes[mnemo].opcode, Register, Bit);
  snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_A_b(AVR_opcode mnemo) {
  int Register, Bit;
  const char *Register_Name;
  char Register_Value[5];

  Register = RA;
  Bit = Rb;
  Register_Name = Resolve_IO_Register(Register);
  if(Register_Name == NULL) {
    snprintf(Register_Value, sizeof(Register_Value), "0x%02x", Register);
    Register_Name = Register_Value;
  }
  switch (mnemo) {
  case OPCODE_cbi:
    snprintf(cx->dis_code, 255, "IO[%s] &= ~(_BV(%d));", Register_Name, Bit);
    break;
  case OPCODE_sbi:
    snprintf(cx->dis_code, 255, "IO[%s] |= _BV(%d);", Register_Name, Bit);
    break;
  case OPCODE_sbis:
    snprintf(cx->dis_code, 255, "skipif (IO[%s] & _BV(%d))", Register_Name, Bit);
    break;
  case OPCODE_sbic:
    snprintf(cx->dis_code, 255, "skipif (!(IO[%s] & _BV(%d)))", Register_Name, Bit);
    break;
  default:
    snprintf(cx->dis_code, 255, "%-7s %s, %d", avr_opcodes[mnemo].opcode, Register_Name, Bit);
  }
  snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_s(AVR_opcode mnemo) {
  int Bit;

  Bit = Rs;
  snprintf(cx->dis_code, 255, "%-7s %d", avr_opcodes[mnemo].opcode, Bit);
  snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_k(AVR_opcode mnemo, int Position, char *Pseudocode) {
  int Offset;

  Offset = (2 * Rk);
  if(Offset > 128)
    Offset -= 256;

  int Target = FixTargetAddress(Position + Offset + 2);

  Register_JumpCall(Position, Target, mnemo, 0);
  if(cx->dis_opts.Process_Labels == 0) {
    if(Offset > 0) {
      snprintf(cx->dis_code, 255, "if (%s) goto .+%d;", Pseudocode, Offset);
    } else {
      snprintf(cx->dis_code, 255, "if (%s) goto .%d", Pseudocode, Offset);
    }
    snprintf(cx->dis_comment, 255, "0x%02x", Target);
  } else {
    snprintf(cx->dis_code, 255, "if (%s) goto %s;", Pseudocode, Get_Label_Name(Target, NULL));
  }
}

/************* Now to the callback functions *************/

CALLBACK(adc_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(add_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(sub_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(sbc_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(mov_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(brcc_Callback_PC) {
  PC_Operation_k(mnemo, Position, "!Carry");
}

CALLBACK(brcs_Callback_PC) {
  PC_Operation_k(mnemo, Position, "Carry");
}

CALLBACK(breq_Callback_PC) {
  PC_Operation_k(mnemo, Position, "c1 == c2");
}

CALLBACK(brge_Callback_PC) {
  PC_Operation_k(mnemo, Position, "c1 (signed)>= c2");
}

CALLBACK(brhc_Callback_PC) {
  PC_Operation_k(mnemo, Position, "!HalfCarry");
}

CALLBACK(brhs_Callback_PC) {
  PC_Operation_k(mnemo, Position, "HalfCarry");
}

CALLBACK(brid_Callback_PC) {
  PC_Operation_k(mnemo, Position, "Global_Interrupts_Disabled()");
}

CALLBACK(brie_Callback_PC) {
  PC_Operation_k(mnemo, Position, "Global_Interrupts_Enabled()");
}

CALLBACK(brlo_Callback_PC) {
  PC_Operation_k(mnemo, Position, "c1 (unsigned)< c2");
}

CALLBACK(brlt_Callback_PC) {
  PC_Operation_k(mnemo, Position, "c1 (signed)< c2");
}

CALLBACK(brmi_Callback_PC) {
  PC_Operation_k(mnemo, Position, "< 0");
}

CALLBACK(brne_Callback_PC) {
  PC_Operation_k(mnemo, Position, "c1 != c2");
}

CALLBACK(brpl_Callback_PC) {
  PC_Operation_k(mnemo, Position, "> 0");
}

CALLBACK(brsh_Callback_PC) {
  PC_Operation_k(mnemo, Position, "c1 (unsigned)>= c2");
}

CALLBACK(brtc_Callback_PC) {
  PC_Operation_k(mnemo, Position, "T == 0");
}

CALLBACK(brts_Callback_PC) {
  PC_Operation_k(mnemo, Position, "T == 1");
}

CALLBACK(brvc_Callback_PC) {
  PC_Operation_k(mnemo, Position, "Overflow == 0");
}

CALLBACK(brvs_Callback_PC) {
  PC_Operation_k(mnemo, Position, "Overflow == 1");
}

CALLBACK(out_Callback_PC) {
  int Register_Number;
  const char *Register_Name;

  Register_Number = RA;
  Register_Name = Resolve_IO_Register(Register_Number);
  if(Register_Name) {
    snprintf(cx->dis_code, 255, "IO[%s] = r%d;", Register_Name, Rr);
  } else {
    snprintf(cx->dis_code, 255, "IO[0x%02x] = r%d;", Register_Number, Rr);
    snprintf(cx->dis_comment, 255, "%d", RA);
  }
}

CALLBACK(in_Callback_PC) {
  int Register_Number;
  const char *Register_Name;

  Register_Number = RA;
  Register_Name = Resolve_IO_Register(Register_Number);
  if(Register_Name) {
    snprintf(cx->dis_code, 255, "r%d = IO[%s];", Rd, Register_Name);
  } else {
    snprintf(cx->dis_code, 255, "r%d = IO[0x%02x];", Rd, Register_Number);
    snprintf(cx->dis_comment, 255, "%d", RA);
  }
}

CALLBACK(cli_Callback_PC) {
  snprintf(cx->dis_code, 255, "Disable_Interrupts();");
}

CALLBACK(sei_Callback_PC) {
  snprintf(cx->dis_code, 255, "Enable_Interrupts();");
}

CALLBACK(ret_Callback_PC) {
  snprintf(cx->dis_code, 255, "return;");
  snprintf(cx->dis_after_code, 255, "\n");
}

CALLBACK(reti_Callback_PC) {
  snprintf(cx->dis_code, 255, "ireturn;");
  snprintf(cx->dis_after_code, 255, "\n");
}

CALLBACK(andi_Callback_PC) {
  PC_Operation_Rd16_K(mnemo);
}

CALLBACK(subi_Callback_PC) {
  PC_Operation_Rd16_K(mnemo);
}

CALLBACK(sbci_Callback_PC) {
  PC_Operation_Rd16_K(mnemo);
}

CALLBACK(sbr_Callback_PC) {
  PC_Operation_Rd16_K(mnemo);
}

CALLBACK(ori_Callback_PC) {
  PC_Operation_Rd16_K(mnemo);
}

CALLBACK(ldi_Callback_PC) {
  snprintf(cx->dis_code, 255, "r%d = %d;", Rd + 16, RK);
  snprintf(cx->dis_comment, 255, "0x%02x", RK);
}

CALLBACK(lds_Callback_PC) {
  const char *MemAddress;

  MemAddress = Tagfile_Resolve_Mem_Address(Rk);
  if(!MemAddress) {
    snprintf(cx->dis_code, 255, "r%d = Memory[0x%04x];", Rd, Rk);
    snprintf(cx->dis_comment, 255, "%d", Rk);
  } else {
    snprintf(cx->dis_code, 255, "r%d = %s;", Rd, MemAddress);
    snprintf(cx->dis_comment, 255, "0x%04x", Rk);
  }
}

CALLBACK(sts_Callback_PC) {
  const char *MemAddress;

  MemAddress = Tagfile_Resolve_Mem_Address(Rk);
  if(!MemAddress) {
    snprintf(cx->dis_code, 255, "Memory[0x%04x] = r%d;", Rk, Rd);
    snprintf(cx->dis_comment, 255, "%d", Rk);
  } else {
    snprintf(cx->dis_code, 255, "%s = r%d;", MemAddress, Rd);
    snprintf(cx->dis_comment, 255, "0x%04x", Rk);
  }
}

CALLBACK(call_Callback_PC) {
  int Pos;

  Pos = FixTargetAddress(2 * Rk);
  Register_JumpCall(Position, Pos, mnemo, 1);
  if(cx->dis_opts.Process_Labels == 0) {
    snprintf(cx->dis_code, 255, "0x%02x();", Pos);
  } else {
    char *LabelName;
    char *LabelComment = NULL;

    LabelName = Get_Label_Name(Pos, &LabelComment);
    snprintf(cx->dis_code, 255, "%s();", LabelName);
    if(LabelComment != NULL) {
      snprintf(cx->dis_comment, 255, "%s", LabelComment);
    }
  }
}

CALLBACK(rcall_Callback_PC) {
  int Offset;

  Offset = 2 * (Rk);
  if(Offset > 4096)
    Offset -= 8192;

  int Target = FixTargetAddress(Position + Offset + 2);

  Register_JumpCall(Position, Target, mnemo, 1);
  if(!cx->dis_opts.Process_Labels) {
    snprintf(cx->dis_comment, 255, "0x%02x();", Target);
  } else {
    char *LabelName;
    char *LabelComment = NULL;

    LabelName = Get_Label_Name(Target, &LabelComment);
    snprintf(cx->dis_code, 255, "%s();", LabelName);
    if(LabelComment != NULL) {
      snprintf(cx->dis_comment, 255, "%s", LabelComment);
    }
  }
}

CALLBACK(ror_Callback_PC) {
  PC_Operation_Rd(mnemo);
}

CALLBACK(lsr_Callback_PC) {
  PC_Operation_Rd(mnemo);
}

CALLBACK(swap_Callback_PC) {
  PC_Operation_Rd(mnemo);
}

CALLBACK(eor_Callback_PC) {
  if(Rd == Rr) {
    PC_Operation_Rd(OPCODE_clr);
  } else {
    PC_Operation_Rd_Rr(mnemo);
  }
}

CALLBACK(jmp_Callback_PC) {
  int Pos;

  Pos = FixTargetAddress(2 * Rk);
  if(cx->dis_opts.Process_Labels == 0) {
    snprintf(cx->dis_code, 255, "goto 0x%02x;", Pos);
  } else {
    snprintf(cx->dis_code, 255, "goto %s;", Get_Label_Name(Pos, NULL));
  }
  Register_JumpCall(Position, Pos, mnemo, 0);
}

CALLBACK(rjmp_Callback_PC) {
  int Offset;

  Offset = 2 * (Rk);
  if(Offset > 4096)
    Offset -= 8192;

  int Target = FixTargetAddress(Position + Offset + 2);

  Register_JumpCall(Position, Target, mnemo, 0);

  if(cx->dis_opts.Process_Labels == 0) {
    if(Offset > 0) {
      snprintf(cx->dis_code, 255, "goto .+%d;", Offset);
    } else {
      snprintf(cx->dis_code, 255, "goto .%d", Offset);
    }
    if(Target >= 0) {
      snprintf(cx->dis_comment, 255, "0x%02x", Target);
    } else {
      snprintf(cx->dis_comment, 255, "-0x%02x - Illegal jump position -- specify flash size!", -(Target));
    }
  } else {
    snprintf(cx->dis_code, 255, "goto %s;", Get_Label_Name(Position + Offset + 2, NULL));
  }
}

CALLBACK(cpi_Callback_PC) {
  if(RK == 0) {
    snprintf(cx->dis_code, 255, "cmp(r%d, 0);", Rd + 16);
  } else {
    snprintf(cx->dis_code, 255, "cmp(r%d, 0x%02x);", Rd + 16, RK);
    snprintf(cx->dis_comment, 255, "%d", RK);
  }
}

CALLBACK(asr_Callback_PC) {
  PC_Operation_Rd(mnemo);
}

CALLBACK(dec_Callback_PC) {
  snprintf(cx->dis_code, 255, "r%d--;", Rd);
}

CALLBACK(inc_Callback_PC) {
  snprintf(cx->dis_code, 255, "r%d++;", Rd);
}

CALLBACK(cp_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(cpc_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(cpse_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(and_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(or_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(mul_Callback_PC) {
  PC_Operation_Rd_Rr(mnemo);
}

CALLBACK(sbi_Callback_PC) {
  PC_Operation_A_b(mnemo);
}

CALLBACK(sbic_Callback_PC) {
  PC_Operation_A_b(mnemo);
}

CALLBACK(sbis_Callback_PC) {
  PC_Operation_A_b(mnemo);
}

CALLBACK(cbi_Callback_PC) {
  PC_Operation_A_b(mnemo);
}

CALLBACK(ser_Callback_PC) {
  snprintf(cx->dis_code, 255, "r%d = 0xff;", Rd + 16);
  snprintf(cx->dis_comment, 255, "255");
}

CALLBACK(adiw_Callback_PC) {
  if(RK != 1) {
    snprintf(cx->dis_code, 255, "[r%d:r%d] += 0x%02x;", 2 * Rd + 25, 2 * Rd + 24, RK);
    snprintf(cx->dis_comment, 255, "%d", RK);
  } else {
    snprintf(cx->dis_code, 255, "[r%d:r%d]++;", 2 * Rd + 25, 2 * Rd + 24);
  }
}

CALLBACK(movw_Callback_PC) {
  snprintf(cx->dis_code, 255, "[r%d:r%d] = [r%d:r%d];", (2 * Rd) + 1, 2 * Rd, (2 * Rr) + 1, 2 * Rr);
}

CALLBACK(lpm1_Callback_PC) {
  snprintf(cx->dis_code, 255, "r0 = Flash[r30:r31];");
}

CALLBACK(stx2_Callback_PC) {
  snprintf(cx->dis_code, 255, "Flash[[r26:r27]++] = r%d;", Rr);
}
