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

#include "MNemonics.h"
#include "Globals.h"
#include "Callbacks_Assembly.h"
#include "JumpCall.h"
#include "IORegisters.h"
#include "Functions.h"
#include "Tools.h"
#include "Tagfile.h"

static char *Code_Line;
static char *Comment_Line;
static char *After_Code_Line;
static int *Registers;
static struct Options *Options;

void Activate_Callbacks(char *New_Code_Line, char *New_Comment_Line, char *New_After_Code_Line, int *New_Registers, struct Options *New_Options) {
	Code_Line = New_Code_Line;
	Comment_Line = New_Comment_Line;
	After_Code_Line = New_After_Code_Line;
	Registers = New_Registers;
	Options = New_Options;
}

void Operation_Simple(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%s", MNemonic[MNemonic_Int]);
}

void Operation_Rd(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d", MNemonic[MNemonic_Int], Rd);
}

void Operation_Rd16(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d", MNemonic[MNemonic_Int], Rd + 16);
}

void Operation_Rd_Rr(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d, r%d", MNemonic[MNemonic_Int], Rd, Rr);
}

void Operation_Rd16_Rr16(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d, r%d", MNemonic[MNemonic_Int], Rd + 16, Rr + 16);
}

void Operation_Rd16_K(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], Rd + 16, RK);
	snprintf(Comment_Line, 255, "%d", RK);
}

void Operation_Rd_K(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], Rd, RK);
	snprintf(Comment_Line, 255, "%d", RK);
}

void Operation_RdW_K(int MNemonic_Int) {
	if (Options->CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
		snprintf(Code_Line, 255, "%-7s r%d:%d, 0x%02x", MNemonic[MNemonic_Int], Rd + 1, Rd, RK);
	} else {
		snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], Rd, RK);
	}
	snprintf(Comment_Line, 255, "%d", RK);
}

void Operation_RdW_RrW(int MNemonic_Int) {
	if (Options->CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
		snprintf(Code_Line, 255, "%-7s r%d:%d, r%d:%d", MNemonic[MNemonic_Int], (2 * Rd) + 1, 2 * Rd, (2 * Rr) + 1, 2 * Rr);
	} else {
		snprintf(Code_Line, 255, "%-7s r%d, r%d", MNemonic[MNemonic_Int], 2 * Rd, 2 * Rr);
	}
}

void Operation_s_k(int MNemonic_Int, int Position) {
	int Bits, Offset;
	int Target; 
	
	Bits = Rs;
	Offset = (2 * Rk);
	if (Offset > 128) Offset -= 256;
	Target = FixTargetAddress(Position + Offset + 2);

	Register_JumpCall(Position, Target, MNemonic_Int, 0);
	if (Options->Process_Labels == 0) {
		if (Offset>0) {
			snprintf(Code_Line, 255, "%-7s %d, .+%d", MNemonic[MNemonic_Int], Bits, Offset);
		} else {
			snprintf(Code_Line, 255, "%-7s %d, .%d", MNemonic[MNemonic_Int], Bits, Offset);
		}
		snprintf(Comment_Line, 255, "0x%02x = %d -> 0x%02x", (1 << Bits), (1 << Bits), Target);
	} else {
		snprintf(Code_Line, 255, "%-7s %d, %s", MNemonic[MNemonic_Int], Bits, Get_Label_Name(Target, NULL));
		snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bits), (1 << Bits));
	}
}

void Operation_r_b(int MNemonic_Int) {
	int Register, Bit;
	Register = Rr;
	Bit = Rb;
	snprintf(Code_Line, 255, "%-7s r%d, %d", MNemonic[MNemonic_Int], Register, Bit);
	snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void Operation_Rd_b(int MNemonic_Int) {
	int Register, Bit;
	Register = Rd;
	Bit = Rb;
	snprintf(Code_Line, 255, "%-7s r%d, %d", MNemonic[MNemonic_Int], Register, Bit);
	snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void Operation_A_b(int MNemonic_Int) {
	int Register, Bit;
	const char *Register_Name;
	Register = RA;
	Bit = Rb;
	Register_Name = Resolve_IO_Register(Register);
	if (Register_Name == NULL) {
		snprintf(Code_Line, 255, "%-7s 0x%02x, %d", MNemonic[MNemonic_Int], Register, Bit);
		snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
	} else {
		snprintf(Code_Line, 255, "%-7s %s, %d", MNemonic[MNemonic_Int], Register_Name, Bit);
		snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
	}
}

void Operation_s(int MNemonic_Int) {
	int Bit;
	Bit = Rs;
	snprintf(Code_Line, 255, "%-7s %d", MNemonic[MNemonic_Int], Bit);
	snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void Operation_k(int MNemonic_Int, int Position, char *Pseudocode) {
	int Offset;
	int Target;
	
	Offset = (2 * Rk);
	if (Offset > 128) Offset -= 256;
	Target = FixTargetAddress(Position + Offset + 2);

	Register_JumpCall(Position, Target, MNemonic_Int, 0);
	if (Options->Process_Labels == 0) {
		if (Offset>0) {
			snprintf(Code_Line, 255, "%-7s .+%d", MNemonic[MNemonic_Int], Offset);
		} else {
			snprintf(Code_Line, 255, "%-7s .%d", MNemonic[MNemonic_Int], Offset);
		}
		snprintf(Comment_Line, 255, "0x%02x", Target);
	} else {
		snprintf(Code_Line, 255, "%-7s %s", MNemonic[MNemonic_Int], Get_Label_Name(Target, NULL));
	}
}

/************* Now to the callback functions *************/

CALLBACK(adc_Callback) {
	if (Rd == Rr) {
		Operation_Rd(OPCODE_rol);
	} else {
		Operation_Rd_Rr(MNemonic_Int);
	}
}

CALLBACK(add_Callback) {
	if (Rd == Rr) {
		Operation_Rd(OPCODE_lsl);
	} else {
		Operation_Rd_Rr(MNemonic_Int);
	}
}

CALLBACK(adiw_Callback) {
	if (Options->CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
		snprintf(Code_Line, 255, "%-7s r%d:%d, 0x%02x", MNemonic[MNemonic_Int], 2 * Rd + 25, 2 * Rd + 24, RK);
	} else {
		snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], 2 * Rd + 24, RK);
	}
	snprintf(Comment_Line, 255, "%d", RK);
}

CALLBACK(and_Callback) {
	if (Rd == Rr) {
		Operation_Rd(OPCODE_tst);
	} else {
		Operation_Rd_Rr(MNemonic_Int);
	}
}

CALLBACK(andi_Callback) {
	if (BitCount(RK) < 4) {
		Operation_Rd16_K(MNemonic_Int);
	} else {
		RK = ~RK;
		RK &= 0xff;
		Operation_Rd16_K(OPCODE_cbr);
	}
}

CALLBACK(asr_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(bclr_Callback) {
	Operation_s(MNemonic_Int);
}

CALLBACK(bld_Callback) {
	Operation_Rd_b(MNemonic_Int);
}

CALLBACK(brbc_Callback) {
	Operation_s_k(MNemonic_Int, Position);
}

CALLBACK(brbs_Callback) {
	Operation_s_k(MNemonic_Int, Position);
}

CALLBACK(brcc_Callback) {
	Operation_k(MNemonic_Int, Position, "Carry == 0");
}

CALLBACK(brcs_Callback) {
	Operation_k(MNemonic_Int, Position, "Carry == 1");
}

CALLBACK(break_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(breq_Callback) {
	Operation_k(MNemonic_Int, Position, "c1 == c2");
}

CALLBACK(brge_Callback) {
	Operation_k(MNemonic_Int, Position, "c1 (signed)>= c2");
}

CALLBACK(brhc_Callback) {
	Operation_k(MNemonic_Int, Position, "HalfCarry == 0");
}

CALLBACK(brhs_Callback) {
	Operation_k(MNemonic_Int, Position, "HalfCarry == 1");
}

CALLBACK(brid_Callback) {
	Operation_k(MNemonic_Int, Position, "Global_Interrupts_Disabled()");
}

CALLBACK(brie_Callback) {
	Operation_k(MNemonic_Int, Position, "Global_Interrupts_Enabled()");
}

CALLBACK(brlo_Callback) {
	Operation_k(MNemonic_Int, Position, "c1 (unsigned)< c2");
}

CALLBACK(brlt_Callback) {
	Operation_k(MNemonic_Int, Position, "c1 (signed)< c2");
}

CALLBACK(brmi_Callback) {
	Operation_k(MNemonic_Int, Position, "< 0");
}

CALLBACK(brne_Callback) {
	Operation_k(MNemonic_Int, Position, "c1 != c2");
}

CALLBACK(brpl_Callback) {
	Operation_k(MNemonic_Int, Position, "> 0");
}

CALLBACK(brsh_Callback) {
	Operation_k(MNemonic_Int, Position, "c1 (unsigned)>= c2");
}

CALLBACK(brtc_Callback) {
	Operation_k(MNemonic_Int, Position, "T == 0");
}

CALLBACK(brts_Callback) {
	Operation_k(MNemonic_Int, Position, "T == 1");
}

CALLBACK(brvc_Callback) {
	Operation_k(MNemonic_Int, Position, "Overflow == 0");
}

CALLBACK(brvs_Callback) {
	Operation_k(MNemonic_Int, Position, "Overflow == 1");
}

CALLBACK(bset_Callback) {
	Operation_s(MNemonic_Int);
}

CALLBACK(bst_Callback) {
	Operation_Rd_b(MNemonic_Int);
}

CALLBACK(call_Callback) {
	int Pos;
	Pos = FixTargetAddress(2 * Rk);
	Register_JumpCall(Position, Pos, MNemonic_Int, 1);
	if (!Options->Process_Labels) {
		snprintf(Code_Line, 255, "%-7s 0x%02x", MNemonic[MNemonic_Int], Pos);
	} else {
		char *LabelName;
		char *LabelComment = NULL;
		LabelName = Get_Label_Name(Pos, &LabelComment);
		snprintf(Code_Line, 255, "%-7s %s", MNemonic[MNemonic_Int], LabelName);
		if (LabelComment != NULL) snprintf(Comment_Line, 255, "%s", LabelComment);
	}
}

CALLBACK(cbi_Callback) {
	Operation_A_b(MNemonic_Int);
}

CALLBACK(clc_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(clh_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(cli_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(cln_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(cls_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(clt_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(clv_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(clz_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(com_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(cp_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(cpc_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(cpi_Callback) {
	Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(cpse_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(dec_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(eicall_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(eijmp_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(elpm1_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(elpm2_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Z", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(elpm3_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Z+", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(eor_Callback) {
	if (Rd == Rr) {
		Operation_Rd(OPCODE_clr);
	} else {
		Operation_Rd_Rr(MNemonic_Int);
	}
}

CALLBACK(fmul_Callback) {
	Operation_Rd16_Rr16(MNemonic_Int);
}

CALLBACK(fmuls_Callback) {
	Operation_Rd16_Rr16(MNemonic_Int);
}

CALLBACK(fmulsu_Callback) {
	Operation_Rd16_Rr16(MNemonic_Int);
}

CALLBACK(icall_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(ijmp_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(in_Callback) {
	int Register_Number;
	const char *Register_Name;
	Register_Number = RA;
	Register_Name = Resolve_IO_Register(Register_Number);
	if (Register_Name != NULL) {
		snprintf(Code_Line, 255, "%-7s r%d, %s", MNemonic[MNemonic_Int], Rd, Register_Name);
	} else {
		snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], Rd, Register_Number);
		snprintf(Comment_Line, 255, "%d", RA);
	}
}

CALLBACK(inc_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(jmp_Callback) {
	int Pos;
	Pos = FixTargetAddress(2 * Rk);
	if (!Options->Process_Labels) {
		snprintf(Code_Line, 255, "%-7s 0x%02x", MNemonic[MNemonic_Int], Pos);
	} else {
		snprintf(Code_Line, 255, "%-7s %s", MNemonic[MNemonic_Int], Get_Label_Name(Pos, NULL));
	}
	Register_JumpCall(Position, Pos, MNemonic_Int, 0);
}

CALLBACK(ld1_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, X", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ld2_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, X+", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ld3_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, -X", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ldy1_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Y", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ldy2_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Y+", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ldy3_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, -Y", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ldy4_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Y+%d", MNemonic[MNemonic_Int], Rd, Rq);
}

CALLBACK(ldz1_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Z", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ldz2_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Z+", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ldz3_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, -Z", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(ldz4_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Z+%d", MNemonic[MNemonic_Int], Rd, Rq);
}

CALLBACK(ldi_Callback) {
	Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(lds_Callback) {
	const char *MemAddress;
	snprintf(Code_Line, 255, "%-7s r%d, 0x%04x", MNemonic[MNemonic_Int], Rd, Rk);
	MemAddress = Tagfile_Resolve_Mem_Address(Rk);
	snprintf(Code_Line, 255, "%-7s 0x%04x, r%d", MNemonic[MNemonic_Int], Rk, Rd);
	if (MemAddress) {
		snprintf(Comment_Line, 255, "%s", MemAddress);
	}
}

CALLBACK(lpm1_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(lpm2_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Z", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(lpm3_Callback) {
	snprintf(Code_Line, 255, "%-7s r%d, Z+", MNemonic[MNemonic_Int], Rd);
}

CALLBACK(lsr_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(mov_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(movw_Callback) {
	Operation_RdW_RrW(MNemonic_Int);
}

CALLBACK(mul_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(muls_Callback) {
	Operation_Rd16_Rr16(MNemonic_Int);
}

CALLBACK(mulsu_Callback) {
	Operation_Rd16_Rr16(MNemonic_Int);
}

CALLBACK(neg_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(nop_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(or_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(ori_Callback) {
	Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(out_Callback) {
	int Register_Number;
	const char *Register_Name;
	Register_Number = RA;
	Register_Name = Resolve_IO_Register(Register_Number);
	if (Register_Name != NULL) {
		snprintf(Code_Line, 255, "%-7s %s, r%d", MNemonic[MNemonic_Int], Register_Name, Rr);
	} else {
		snprintf(Code_Line, 255, "%-7s 0x%02x, r%d", MNemonic[MNemonic_Int], Register_Number, Rr);
		snprintf(Comment_Line, 255, "%d", RA);
	}
}

CALLBACK(pop_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(push_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(rcall_Callback) {
	int Offset;
	int Target;
	
	Offset = 2 * (Rk);
	if (Offset > 4096) Offset -= 8192;
	Target = FixTargetAddress(Position + Offset + 2);

	Register_JumpCall(Position, Target, MNemonic_Int, 1);
	if (Options->Process_Labels == 0) {
		if (Offset>0) {
			snprintf(Code_Line, 255, "%-7s .+%d", MNemonic[MNemonic_Int], Offset);
		} else {
			snprintf(Code_Line, 255, "%-7s .%d", MNemonic[MNemonic_Int], Offset);
		}
		snprintf(Comment_Line, 255, "0x%02x", Target);
	} else {
		char *LabelName;
		char *LabelComment = NULL;
		LabelName = Get_Label_Name(Target, &LabelComment);
		snprintf(Code_Line, 255, "%-7s %s", MNemonic[MNemonic_Int], LabelName);
		if (LabelComment != NULL) snprintf(Comment_Line, 255, "%s", LabelComment);
	}
}

CALLBACK(ret_Callback) {
	Operation_Simple(MNemonic_Int);
	snprintf(After_Code_Line, 255, "\n");
}

CALLBACK(reti_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(rjmp_Callback) {
	int Offset;
	int Target;
	
	Offset = 2 * (Rk);
	if (Offset > 4096) Offset -= 8192;
	Target = FixTargetAddress(Position + Offset + 2);

	Register_JumpCall(Position, Target, MNemonic_Int, 0);

	if (Options->Process_Labels == 0) {
		if (Offset>0) {
			snprintf(Code_Line, 255, "%-7s .+%d", MNemonic[MNemonic_Int], Offset);
		} else {
			snprintf(Code_Line, 255, "%-7s .%d", MNemonic[MNemonic_Int], Offset);
		}
		if (Target >= 0) {
			snprintf(Comment_Line, 255, "0x%02x", Target);
		} else {
			snprintf(Comment_Line, 255, "-0x%02x - Illegal jump position -- specify flash size!", -Target);
		}
	} else {
		snprintf(Code_Line, 255, "%-7s %s", MNemonic[MNemonic_Int], Get_Label_Name(Target, NULL));
	}
}

CALLBACK(ror_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(sbc_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(sbci_Callback) {
	Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(sbi_Callback) {
	Operation_A_b(MNemonic_Int);
}

CALLBACK(sbic_Callback) {
	Operation_A_b(MNemonic_Int);
}

CALLBACK(sbis_Callback) {
	Operation_A_b(MNemonic_Int);
}

CALLBACK(sbiw_Callback) {
	if (Options->CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
		snprintf(Code_Line, 255, "%-7s r%d:%d, 0x%02x", MNemonic[MNemonic_Int], 2*Rd+25, 2*Rd+24, RK);
	} else {
		snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], 2*Rd+24, RK);
	}
	snprintf(Comment_Line, 255, "%d", RK);
}

CALLBACK(sbr_Callback) {
	Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(sbrc_Callback) {
	Operation_r_b(MNemonic_Int);
}

CALLBACK(sbrs_Callback) {
	Operation_r_b(MNemonic_Int);
}

CALLBACK(sec_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(seh_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(sei_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(sen_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(ser_Callback) {
	Operation_Rd16(MNemonic_Int);
}

CALLBACK(ses_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(set_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(sev_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(sez_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(sleep_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(spm_Callback) {
	Operation_Simple(MNemonic_Int);
}

CALLBACK(st1_Callback) {
	snprintf(Code_Line, 255, "%-7s X, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(st2_Callback) {
	snprintf(Code_Line, 255, "%-7s X+, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(st3_Callback) {
	snprintf(Code_Line, 255, "%-7s -X, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(sty1_Callback) {
	snprintf(Code_Line, 255, "%-7s Y, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(sty2_Callback) {
	snprintf(Code_Line, 255, "%-7s Y+, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(sty3_Callback) {
	snprintf(Code_Line, 255, "%-7s -Y, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(sty4_Callback) {
	snprintf(Code_Line, 255, "%-7s Y+%d, r%d", MNemonic[MNemonic_Int], Rq, Rr);
}

CALLBACK(stz1_Callback) {
	snprintf(Code_Line, 255, "%-7s Z, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(stz2_Callback) {
	snprintf(Code_Line, 255, "%-7s Z+, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(stz3_Callback) {
	snprintf(Code_Line, 255, "%-7s -Z, r%d", MNemonic[MNemonic_Int], Rr);
}

CALLBACK(stz4_Callback) {
	snprintf(Code_Line, 255, "%-7s Z+%d, r%d", MNemonic[MNemonic_Int], Rq, Rr);
}

CALLBACK(sts_Callback) {
	/* The AVR instruction set 11/2005 defines operation as: "(k) <- Rr", however "(k) <- Rd" seems to be right */
	const char *MemAddress;
	MemAddress = Tagfile_Resolve_Mem_Address(Rk);
	snprintf(Code_Line, 255, "%-7s 0x%04x, r%d", MNemonic[MNemonic_Int], Rk, Rd);
	if (MemAddress) {
		snprintf(Comment_Line, 255, "%s", MemAddress);
	}
}

CALLBACK(sub_Callback) {
	Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(subi_Callback) {
	Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(swap_Callback) {
	Operation_Rd(MNemonic_Int);
}

CALLBACK(wdr_Callback) {
	Operation_Simple(MNemonic_Int);
}

