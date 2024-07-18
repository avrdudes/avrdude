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
#include <string.h>

#include "libavrdude.h"
#include "disasm_private.h"


// Wrap r/jmp around flash where possible
static int FixTargetAddress(int Address) {
  int flashsz = cx->dis_opts.FlashSize;

  // Flash size is a power of two: flash wraps round
  if(flashsz > 0 && !(flashsz & (flashsz - 1))) {
    Address %= flashsz;
    if(Address < 0)
      Address += flashsz;
  }
  return Address;
}


// Return the number of bits set in Number
static unsigned BitCount(unsigned n) {
  unsigned ret;

  // A la Kernighan (and Richie): iteratively clear the least significant bit set
  for(ret = 0; n; ret++)
    n &= n-1;

  return ret;
}

static void Operation_Simple(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%s", avr_opcodes[mnemo].opcode);
}

static void Operation_Rd(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d", avr_opcodes[mnemo].opcode, Rd);
}

static void Operation_Z_Rd(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s Z, r%d", avr_opcodes[mnemo].opcode, Rd);
}

static void Operation_Rd16(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d", avr_opcodes[mnemo].opcode, Rd + 16);
}

static void Operation_Rd_Rr(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d, r%d", avr_opcodes[mnemo].opcode, Rd, Rr);
}

static void Operation_Rd16_Rr16(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d, r%d", avr_opcodes[mnemo].opcode, Rd + 16, Rr + 16);
}

static void Operation_Rd16_K(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, Rd + 16, RK);
  snprintf(cx->dis_comment, 255, "%d", RK);
}

static void Operation_RdW_RrW(AVR_opcode mnemo) {
  if(cx->dis_opts.CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
    snprintf(cx->dis_code, 255, "%-7s r%d:%d, r%d:%d", avr_opcodes[mnemo].opcode, (2 * Rd) + 1, 2 * Rd, (2 * Rr) + 1, 2 * Rr);
  } else {
    snprintf(cx->dis_code, 255, "%-7s r%d, r%d", avr_opcodes[mnemo].opcode, 2 * Rd, 2 * Rr);
  }
}

static void Operation_s_k(AVR_opcode mnemo, int Position) {
  int Bits, Offset;
  int Target;

  Bits = Rs;
  Offset = (2 * Rk);
  if(Offset > 128)
    Offset -= 256;
  Target = FixTargetAddress(Position + Offset + 2);

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

static void Operation_r_b(AVR_opcode mnemo) {
  int Register, Bit;

  Register = Rr;
  Bit = Rb;
  snprintf(cx->dis_code, 255, "%-7s r%d, %d", avr_opcodes[mnemo].opcode, Register, Bit);
  snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

static void Operation_Rd_b(AVR_opcode mnemo) {
  int Register, Bit;

  Register = Rd;
  Bit = Rb;
  snprintf(cx->dis_code, 255, "%-7s r%d, %d", avr_opcodes[mnemo].opcode, Register, Bit);
  snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

static void Operation_A_b(AVR_opcode mnemo) {
  int Register, Bit;
  const char *Register_Name;

  Register = RA;
  Bit = Rb;
  Register_Name = Resolve_IO_Register(Register);
  if(Register_Name == NULL) {
    snprintf(cx->dis_code, 255, "%-7s 0x%02x, %d", avr_opcodes[mnemo].opcode, Register, Bit);
    snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
  } else {
    snprintf(cx->dis_code, 255, "%-7s %s, %d", avr_opcodes[mnemo].opcode, Register_Name, Bit);
    snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
  }
}

static void Operation_s(AVR_opcode mnemo) {
  int Bit;

  Bit = Rs;
  snprintf(cx->dis_code, 255, "%-7s %d", avr_opcodes[mnemo].opcode, Bit);
  snprintf(cx->dis_comment, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

static void Operation_K(AVR_opcode mnemo) {
  snprintf(cx->dis_code, 255, "%-7s %d", avr_opcodes[mnemo].opcode, RK);
}

static void Operation_k(AVR_opcode mnemo, int Position, const char *Pseudocode) {
  int Offset;
  int Target;

  Offset = (2 * Rk);
  if(Offset > 128)
    Offset -= 256;
  Target = FixTargetAddress(Position + Offset + 2);

  Register_JumpCall(Position, Target, mnemo, 0);
  if(cx->dis_opts.Process_Labels == 0) {
    if(Offset > 0) {
      snprintf(cx->dis_code, 255, "%-7s .+%d", avr_opcodes[mnemo].opcode, Offset);
    } else {
      snprintf(cx->dis_code, 255, "%-7s .%d", avr_opcodes[mnemo].opcode, Offset);
    }
    snprintf(cx->dis_comment, 255, "0x%02x", Target);
  } else {
    snprintf(cx->dis_code, 255, "%-7s %s", avr_opcodes[mnemo].opcode, Get_Label_Name(Target, NULL));
  }
}

/************* Now to the callback functions *************/

CALLBACK(adc_Callback) {
  if(Rd == Rr) {
    Operation_Rd(OPCODE_rol);
  } else {
    Operation_Rd_Rr(mnemo);
  }
}

CALLBACK(add_Callback) {
  if(Rd == Rr) {
    Operation_Rd(OPCODE_lsl);
  } else {
    Operation_Rd_Rr(mnemo);
  }
}

CALLBACK(adiw_Callback) {
  if(cx->dis_opts.CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
    snprintf(cx->dis_code, 255, "%-7s r%d:%d, 0x%02x", avr_opcodes[mnemo].opcode, 2 * Rd + 25, 2 * Rd + 24, RK);
  } else {
    snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, 2 * Rd + 24, RK);
  }
  snprintf(cx->dis_comment, 255, "%d", RK);
}

CALLBACK(and_Callback) {
  if(Rd == Rr) {
    Operation_Rd(OPCODE_tst);
  } else {
    Operation_Rd_Rr(mnemo);
  }
}

CALLBACK(andi_Callback) {
  if(BitCount(RK) < 4) {
    Operation_Rd16_K(mnemo);
  } else {
    RK = ~RK;
    RK &= 0xff;
    Operation_Rd16_K(OPCODE_cbr);
  }
}

CALLBACK(asr_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(bclr_Callback) {
  Operation_s(mnemo);
}

CALLBACK(bld_Callback) {
  Operation_Rd_b(mnemo);
}

CALLBACK(brbc_Callback) {
  Operation_s_k(mnemo, Position);
}

CALLBACK(brbs_Callback) {
  Operation_s_k(mnemo, Position);
}

CALLBACK(brcc_Callback) {
  Operation_k(mnemo, Position, "Carry == 0");
}

CALLBACK(brcs_Callback) {
  Operation_k(mnemo, Position, "Carry == 1");
}

CALLBACK(break_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(breq_Callback) {
  Operation_k(mnemo, Position, "c1 == c2");
}

CALLBACK(brge_Callback) {
  Operation_k(mnemo, Position, "c1 (signed)>= c2");
}

CALLBACK(brhc_Callback) {
  Operation_k(mnemo, Position, "HalfCarry == 0");
}

CALLBACK(brhs_Callback) {
  Operation_k(mnemo, Position, "HalfCarry == 1");
}

CALLBACK(brid_Callback) {
  Operation_k(mnemo, Position, "Global_Interrupts_Disabled()");
}

CALLBACK(brie_Callback) {
  Operation_k(mnemo, Position, "Global_Interrupts_Enabled()");
}

CALLBACK(brlo_Callback) {
  Operation_k(mnemo, Position, "c1 (unsigned)< c2");
}

CALLBACK(brlt_Callback) {
  Operation_k(mnemo, Position, "c1 (signed)< c2");
}

CALLBACK(brmi_Callback) {
  Operation_k(mnemo, Position, "< 0");
}

CALLBACK(brne_Callback) {
  Operation_k(mnemo, Position, "c1 != c2");
}

CALLBACK(brpl_Callback) {
  Operation_k(mnemo, Position, "> 0");
}

CALLBACK(brsh_Callback) {
  Operation_k(mnemo, Position, "c1 (unsigned)>= c2");
}

CALLBACK(brtc_Callback) {
  Operation_k(mnemo, Position, "T == 0");
}

CALLBACK(brts_Callback) {
  Operation_k(mnemo, Position, "T == 1");
}

CALLBACK(brvc_Callback) {
  Operation_k(mnemo, Position, "Overflow == 0");
}

CALLBACK(brvs_Callback) {
  Operation_k(mnemo, Position, "Overflow == 1");
}

CALLBACK(bset_Callback) {
  Operation_s(mnemo);
}

CALLBACK(bst_Callback) {
  Operation_Rd_b(mnemo);
}

CALLBACK(call_Callback) {
  int Pos;

  Pos = FixTargetAddress(2 * Rk);
  Register_JumpCall(Position, Pos, mnemo, 1);
  if(!cx->dis_opts.Process_Labels) {
    snprintf(cx->dis_code, 255, "%-7s 0x%02x", avr_opcodes[mnemo].opcode, Pos);
  } else {
    char *LabelComment = NULL;
    const char *LabelName = Get_Label_Name(Pos, &LabelComment);

    snprintf(cx->dis_code, 255, "%-7s %s", avr_opcodes[mnemo].opcode, LabelName);
    if(LabelComment != NULL)
      snprintf(cx->dis_comment, 255, "%s", LabelComment);
  }
}

CALLBACK(cbi_Callback) {
  Operation_A_b(mnemo);
}

CALLBACK(clc_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(clh_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(cli_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(cln_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(cls_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(clt_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(clv_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(clz_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(com_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(cp_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(cpc_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(cpi_Callback) {
  Operation_Rd16_K(mnemo);
}

CALLBACK(cpse_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(dec_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(des_Callback) {
  Operation_K(mnemo);
}

CALLBACK(eicall_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(eijmp_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(elpm1_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(elpm2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Z", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(elpm3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Z+", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(eor_Callback) {
  if(Rd == Rr) {
    Operation_Rd(OPCODE_clr);
  } else {
    Operation_Rd_Rr(mnemo);
  }
}

CALLBACK(fmul_Callback) {
  Operation_Rd16_Rr16(mnemo);
}

CALLBACK(fmuls_Callback) {
  Operation_Rd16_Rr16(mnemo);
}

CALLBACK(fmulsu_Callback) {
  Operation_Rd16_Rr16(mnemo);
}

CALLBACK(icall_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(ijmp_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(in_Callback) {
  int Register_Number;
  const char *Register_Name;

  Register_Number = RA;
  Register_Name = Resolve_IO_Register(Register_Number);
  if(Register_Name != NULL) {
    snprintf(cx->dis_code, 255, "%-7s r%d, %s", avr_opcodes[mnemo].opcode, Rd, Register_Name);
  } else {
    snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, Rd, Register_Number);
    snprintf(cx->dis_comment, 255, "%d", RA);
  }
}

CALLBACK(inc_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(jmp_Callback) {
  int Pos;

  Pos = FixTargetAddress(2 * Rk);
  if(!cx->dis_opts.Process_Labels) {
    snprintf(cx->dis_code, 255, "%-7s 0x%02x", avr_opcodes[mnemo].opcode, Pos);
  } else {
    snprintf(cx->dis_code, 255, "%-7s %s", avr_opcodes[mnemo].opcode, Get_Label_Name(Pos, NULL));
  }
  Register_JumpCall(Position, Pos, mnemo, 0);
}

CALLBACK(lac_Callback) {
  Operation_Z_Rd(mnemo);
}

CALLBACK(las_Callback) {
  Operation_Z_Rd(mnemo);
}

CALLBACK(lat_Callback) {
  Operation_Z_Rd(mnemo);
}

CALLBACK(ldx1_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, X", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldx2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, X+", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldx3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, -X", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldy1_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Y", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldy2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Y+", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldy3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, -Y", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldy4_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Y+%d", avr_opcodes[mnemo].opcode, Rd, Rq);
}

CALLBACK(ldz1_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Z", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldz2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Z+", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldz3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, -Z", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(ldz4_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Z+%d", avr_opcodes[mnemo].opcode, Rd, Rq);
}

CALLBACK(ldi_Callback) {
  Operation_Rd16_K(mnemo);
}

CALLBACK(lds_Callback) {
  const char *MemAddress;

  snprintf(cx->dis_code, 255, "%-7s r%d, 0x%04x", avr_opcodes[mnemo].opcode, Rd, Rk);
  MemAddress = Tagfile_Resolve_Mem_Address(Rk);
  if(MemAddress) {
    snprintf(cx->dis_comment, 255, "%s", MemAddress);
  }
}

CALLBACK(lds_rc_Callback) {
  /*
   * Address is limited to 0x40...0xbf for the reduced-core (TPI part)
   * ADDR[7:0] ← (/INST[8], INST[8], INST[10], INST[9], INST[3], INST[2], INST[1], INST[0])
   * ADDR[7:0] ← (/k[4], k[4], k[6], k[5], k[3], k[2], k[1], k[0])
   */
  int addr = (Rk & 0xf) | ((Rk >> 1) & 0x30) | ((Rk & 0x10) << 2) | (((Rk & 0x10) ^ 0x10) << 3);

  snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, Rd+16, addr);
  const char *MemAddress = Tagfile_Resolve_Mem_Address(addr);
  if(MemAddress)
    snprintf(cx->dis_comment, 255, "%s", MemAddress);
}

CALLBACK(lpm1_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(lpm2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Z", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(lpm3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s r%d, Z+", avr_opcodes[mnemo].opcode, Rd);
}

CALLBACK(lsr_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(mov_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(movw_Callback) {
  Operation_RdW_RrW(mnemo);
}

CALLBACK(mul_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(muls_Callback) {
  Operation_Rd16_Rr16(mnemo);
}

CALLBACK(mulsu_Callback) {
  Operation_Rd16_Rr16(mnemo);
}

CALLBACK(neg_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(nop_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(or_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(ori_Callback) {
  Operation_Rd16_K(BitCount(RK) < 4? mnemo: OPCODE_sbr);
}

CALLBACK(out_Callback) {
  int Register_Number;
  const char *Register_Name;

  Register_Number = RA;
  Register_Name = Resolve_IO_Register(Register_Number);
  if(Register_Name != NULL) {
    snprintf(cx->dis_code, 255, "%-7s %s, r%d", avr_opcodes[mnemo].opcode, Register_Name, Rr);
  } else {
    snprintf(cx->dis_code, 255, "%-7s 0x%02x, r%d", avr_opcodes[mnemo].opcode, Register_Number, Rr);
    snprintf(cx->dis_comment, 255, "%d", RA);
  }
}

CALLBACK(pop_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(push_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(rcall_Callback) {
  int Offset;
  int Target;

  Offset = 2 * (Rk);
  if(Offset > 4096)
    Offset -= 8192;
  Target = FixTargetAddress(Position + Offset + 2);

  Register_JumpCall(Position, Target, mnemo, 1);
  if(cx->dis_opts.Process_Labels == 0) {
    if(Offset > 0) {
      snprintf(cx->dis_code, 255, "%-7s .+%d", avr_opcodes[mnemo].opcode, Offset);
    } else {
      snprintf(cx->dis_code, 255, "%-7s .%d", avr_opcodes[mnemo].opcode, Offset);
    }
    snprintf(cx->dis_comment, 255, "0x%02x", Target);
  } else {
    char *LabelComment = NULL;
    const char *LabelName = Get_Label_Name(Target, &LabelComment);

    snprintf(cx->dis_code, 255, "%-7s %s", avr_opcodes[mnemo].opcode, LabelName);
    if(LabelComment != NULL)
      snprintf(cx->dis_comment, 255, "%s", LabelComment);
  }
}

CALLBACK(ret_Callback) {
  Operation_Simple(mnemo);
  snprintf(cx->dis_after_code, 255, "\n");
}

CALLBACK(reti_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(rjmp_Callback) {
  int Offset;
  int Target;

  Offset = 2 * (Rk);
  if(Offset > 4096)
    Offset -= 8192;
  Target = FixTargetAddress(Position + Offset + 2);

  Register_JumpCall(Position, Target, mnemo, 0);
  if(cx->dis_opts.Process_Labels == 0) {
    if(Offset > 0) {
      snprintf(cx->dis_code, 255, "%-7s .+%d", avr_opcodes[mnemo].opcode, Offset);
    } else {
      snprintf(cx->dis_code, 255, "%-7s .%d", avr_opcodes[mnemo].opcode, Offset);
    }
    if(Target >= 0) {
      snprintf(cx->dis_comment, 255, "0x%02x", Target);
    } else {
      snprintf(cx->dis_comment, 255, "-0x%02x - Illegal jump position?", -Target);
    }
  } else {
    snprintf(cx->dis_code, 255, "%-7s %s", avr_opcodes[mnemo].opcode, Get_Label_Name(Target, NULL));
  }
}

CALLBACK(ror_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(sbc_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(sbci_Callback) {
  Operation_Rd16_K(mnemo);
}

CALLBACK(sbi_Callback) {
  Operation_A_b(mnemo);
}

CALLBACK(sbic_Callback) {
  Operation_A_b(mnemo);
}

CALLBACK(sbis_Callback) {
  Operation_A_b(mnemo);
}

CALLBACK(sbiw_Callback) {
  if(cx->dis_opts.CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
    snprintf(cx->dis_code, 255, "%-7s r%d:%d, 0x%02x", avr_opcodes[mnemo].opcode, 2 * Rd + 25, 2 * Rd + 24, RK);
  } else {
    snprintf(cx->dis_code, 255, "%-7s r%d, 0x%02x", avr_opcodes[mnemo].opcode, 2 * Rd + 24, RK);
  }
  snprintf(cx->dis_comment, 255, "%d", RK);
}

CALLBACK(sbrc_Callback) {
  Operation_r_b(mnemo);
}

CALLBACK(sbrs_Callback) {
  Operation_r_b(mnemo);
}

CALLBACK(sec_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(seh_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(sei_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(sen_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(ser_Callback) {
  Operation_Rd16(mnemo);
}

CALLBACK(ses_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(set_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(sev_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(sez_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(sleep_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(spm_Callback) {
  Operation_Simple(mnemo);
}

CALLBACK(spm_zz_Callback) {
  snprintf(cx->dis_code, 255, "%-7s Z+", avr_opcodes[mnemo].opcode);
}

CALLBACK(stx1_Callback) {
  snprintf(cx->dis_code, 255, "%-7s X, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(stx2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s X+, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(stx3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s -X, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(sty1_Callback) {
  snprintf(cx->dis_code, 255, "%-7s Y, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(sty2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s Y+, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(sty3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s -Y, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(sty4_Callback) {
  snprintf(cx->dis_code, 255, "%-7s Y+%d, r%d", avr_opcodes[mnemo].opcode, Rq, Rr);
}

CALLBACK(stz1_Callback) {
  snprintf(cx->dis_code, 255, "%-7s Z, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(stz2_Callback) {
  snprintf(cx->dis_code, 255, "%-7s Z+, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(stz3_Callback) {
  snprintf(cx->dis_code, 255, "%-7s -Z, r%d", avr_opcodes[mnemo].opcode, Rr);
}

CALLBACK(stz4_Callback) {
  snprintf(cx->dis_code, 255, "%-7s Z+%d, r%d", avr_opcodes[mnemo].opcode, Rq, Rr);
}

CALLBACK(sts_Callback) {
  // The AVR instruction set 11/2005 defines operation as: "(k) <- Rr", however "(k) <- Rd" seems to be right
  const char *MemAddress;

  MemAddress = Tagfile_Resolve_Mem_Address(Rk);
  snprintf(cx->dis_code, 255, "%-7s 0x%04x, r%d", avr_opcodes[mnemo].opcode, Rk, Rd);
  if(MemAddress) {
    snprintf(cx->dis_comment, 255, "%s", MemAddress);
  }
}

CALLBACK(sts_rc_Callback) {
  /*
   * Address is limited to 0x40...0xbf for the reduced-core (TPI part)
   * ADDR[7:0] ← (/INST[8], INST[8], INST[10], INST[9], INST[3], INST[2], INST[1], INST[0])
   * ADDR[7:0] ← (/k[4], k[4], k[6], k[5], k[3], k[2], k[1], k[0])
   */
  int addr = (Rk & 0xf) | ((Rk >> 1) & 0x30) | ((Rk & 0x10) << 2) | (((Rk & 0x10) ^ 0x10) << 3);

  snprintf(cx->dis_code, 255, "%-7s 0x%02x, r%d", avr_opcodes[mnemo].opcode, addr, Rd+16);
  const char *MemAddress = Tagfile_Resolve_Mem_Address(addr);
  if(MemAddress)
    snprintf(cx->dis_comment, 255, "%s", MemAddress);
}

CALLBACK(sub_Callback) {
  Operation_Rd_Rr(mnemo);
}

CALLBACK(subi_Callback) {
  Operation_Rd16_K(mnemo);
}

CALLBACK(swap_Callback) {
  Operation_Rd(mnemo);
}

CALLBACK(xch_Callback) {
  Operation_Z_Rd(mnemo);
}

CALLBACK(wdr_Callback) {
  Operation_Simple(mnemo);
}
