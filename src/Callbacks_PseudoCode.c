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
#include "Callbacks_PseudoCode.h"
#include "JumpCall.h"
#include "IORegisters.h"
#include "Functions.h"
#include "Tagfile.h"

static char *Code_Line;
static char *Comment_Line;
static char *After_Code_Line;
static int *Registers;
static struct Options *Options;

void Activate_PC_Callbacks(char *New_Code_Line, char *New_Comment_Line, char *New_After_Code_Line, int *New_Registers, struct Options *New_Options) {
	Code_Line = New_Code_Line;
	Comment_Line = New_Comment_Line;
	After_Code_Line = New_After_Code_Line;
	Registers = New_Registers;
	Options = New_Options;
}

void PC_Operation_Simple(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%s", MNemonic[MNemonic_Int]);
}

void PC_Operation_Rd(int MNemonic_Int) {
	switch (MNemonic_Int) {
		case OPCODE_lsl:
			snprintf(Code_Line, 255, "r%d <<= 1;", Rd);
			snprintf(Comment_Line, 255, "_BV(0) = 0 (logical shift)");
			break;
		case OPCODE_lsr:
			snprintf(Code_Line, 255, "r%d >>= 1;", Rd);
			snprintf(Comment_Line, 255, "_BV(7) = 0, Carry = _BV(0) (logical shift)");
			break;
		case OPCODE_ror:
			snprintf(Code_Line, 255, "r%d >>= 1;", Rd);
			snprintf(Comment_Line, 255, "_BV(7) = Carry, Carry = _BV(0) (rotate right)");
			break;
		case OPCODE_asr:
/*			snprintf(Code_Line, 255, "r%d >>= 1;", Rd);*/
/*			snprintf(Comment_Line, 255, "_BV(7) = Sign (Arithmetic Shift)");*/
			snprintf(Code_Line, 255, "r%d /= 2;", Rd);
			snprintf(Comment_Line, 255, "arithmetic shift right");
			break;
		case OPCODE_swap:
			snprintf(Code_Line, 255, "r%d = ((r%d & 0xf0) >> 4) | ((r%d & 0x0f) << 4);", Rd, Rd, Rd);
			snprintf(Comment_Line, 255, "swap nibbles");
			break;
		case OPCODE_clr:
			snprintf(Code_Line, 255, "r%d = 0x00;", Rd);
			snprintf(Comment_Line, 255, "0");
			break;
		default:
			snprintf(Code_Line, 255, "%-7s r%d", MNemonic[MNemonic_Int], Rd);
		}
}

void PC_Operation_Rd16(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d", MNemonic[MNemonic_Int], Rd + 16);
}

void PC_Operation_Rd_Rr(int MNemonic_Int) {
	switch (MNemonic_Int) {
		case OPCODE_add:
			if (Rd != Rr) {
				snprintf(Code_Line, 255, "r%d += r%d;", Rd, Rr);
				snprintf(Comment_Line, 255, "No carry");
			} else {
				snprintf(Code_Line, 255, "r%d *= 2;", Rd);
			}
			break;
		case OPCODE_adc:
			snprintf(Code_Line, 255, "r%d += r%d;", Rd, Rr);
			snprintf(Comment_Line, 255, "With carry");
			break;
		case OPCODE_sub:
			snprintf(Code_Line, 255, "r%d -= r%d;", Rd, Rr);
			snprintf(Comment_Line, 255, "No carry");
			break;
		case OPCODE_sbc:
			snprintf(Code_Line, 255, "r%d += r%d;", Rd, Rr);
			snprintf(Comment_Line, 255, "With carry");
			break;
		case OPCODE_mul:
			snprintf(Code_Line, 255, "r1:r0 = r%d * r%d;", Rd, Rr);
			snprintf(Comment_Line, 255, "Unsigned");
			break;
		case OPCODE_mov:
			snprintf(Code_Line, 255, "r%d = r%d;", Rd, Rr);
			break;
		case OPCODE_eor:
			snprintf(Code_Line, 255, "r%d ^= r%d;", Rd, Rr);
			break;
		case OPCODE_and:
			if (Rd != Rr)	{
				snprintf(Code_Line, 255, "r%d &= r%d;", Rd, Rr);
			} else {
				snprintf(Code_Line, 255, "(r%d == 0) || (r%d < 0);", Rd, Rd);
				snprintf(Comment_Line, 255, "test r%d", Rd);
			}
			break;
		case OPCODE_or:
			snprintf(Code_Line, 255, "r%d |= r%d;", Rd, Rr);
			break;
		case OPCODE_cp:
			snprintf(Code_Line, 255, "cmp(r%d, r%d);", Rd, Rr);
			break;
		case OPCODE_cpc:
			snprintf(Code_Line, 255, "cmp(r%d, r%d);", Rd, Rr);
			snprintf(Comment_Line, 255, "with carry");
			break;
		case OPCODE_cpse:
			snprintf(Code_Line, 255, "skipif (r%d == r%d)", Rd, Rr);
			break;
		default:
			snprintf(Code_Line, 255, "%-7s r%d, r%d", MNemonic[MNemonic_Int], Rd, Rr);
	}
}

void PC_Operation_Rd16_Rr16(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d, r%d", MNemonic[MNemonic_Int], Rd + 16, Rr + 16);
}

void PC_Operation_Rd16_K(int MNemonic_Int) {
	switch (MNemonic_Int) {
		case OPCODE_andi:
			snprintf(Code_Line, 255, "r%d &= %d;", Rd + 16, RK);
			snprintf(Comment_Line, 255, "0x%02x", RK);
			break;
		case OPCODE_subi:
			snprintf(Code_Line, 255, "r%d -= %d;", Rd + 16, RK);
			snprintf(Comment_Line, 255, "0x%02x, no carry", RK);
			break;
		case OPCODE_sbci:
			snprintf(Code_Line, 255, "r%d -= %d;", Rd + 16, RK);
			snprintf(Comment_Line, 255, "0x%02x, with carry", RK);
			break;
		case OPCODE_sbr:
		case OPCODE_ori:
			snprintf(Code_Line, 255, "r%d |= %d;", Rd + 16, RK);
			snprintf(Comment_Line, 255, "0x%02x", RK);
			break;
		default:
			snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], Rd + 16, RK);
			snprintf(Comment_Line, 255, "%d", RK);
	}
}

void PC_Operation_Rd_K(int MNemonic_Int) {
	snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], Rd, RK);
	snprintf(Comment_Line, 255, "%d", RK);
}

void PC_Operation_RdW_K(int MNemonic_Int) {
	if (Options->CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
		snprintf(Code_Line, 255, "%-7s r%d:%d, 0x%02x", MNemonic[MNemonic_Int], Rd+1, Rd, RK);
	} else {
		snprintf(Code_Line, 255, "%-7s r%d, 0x%02x", MNemonic[MNemonic_Int], Rd, RK);
	}
	snprintf(Comment_Line, 255, "%d", RK);
}

void PC_Operation_RdW_RrW(int MNemonic_Int) {
	if (Options->CodeStyle == CODESTYLE_AVR_INSTRUCTION_SET) {
		snprintf(Code_Line, 255, "%-7s r%d:%d, r%d:%d", MNemonic[MNemonic_Int], (2*Rd)+1, 2*Rd, (2*Rr)+1, 2*Rr);
	} else {
		snprintf(Code_Line, 255, "%-7s r%d, r%d", MNemonic[MNemonic_Int], 2*Rd, 2*Rr);
	}
}

void PC_Operation_s_k(int MNemonic_Int, int Position) {
	int Bits, Offset;
	Bits = Rs;
	Offset = (2 * Rk);
	if (Offset > 128) Offset -= 256;
	
	int Target = FixTargetAddress(Position + Offset + 2);
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

void PC_Operation_r_b(int MNemonic_Int) {
	int Register, Bit;
	Register = Rr;
	Bit = Rb;
	snprintf(Code_Line, 255, "%-7s r%d, %d", MNemonic[MNemonic_Int], Register, Bit);
	snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_Rd_b(int MNemonic_Int) {
	int Register, Bit;
	Register = Rd;
	Bit = Rb;
	snprintf(Code_Line, 255, "%-7s r%d, %d", MNemonic[MNemonic_Int], Register, Bit);
	snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_A_b(int MNemonic_Int) {
	int Register, Bit;
	const char *Register_Name;
	char Register_Value[5];
	Register = RA;
	Bit = Rb;
	Register_Name = Resolve_IO_Register(Register);
	if (Register_Name == NULL) {
		snprintf(Register_Value, sizeof(Register_Value), "0x%02x", Register);
		Register_Name = Register_Value;
	} 
	switch (MNemonic_Int) {
		case OPCODE_cbi:
			snprintf(Code_Line, 255, "IO[%s] &= ~(_BV(%d));", Register_Name, Bit);
			break;
		case OPCODE_sbi:
			snprintf(Code_Line, 255, "IO[%s] |= _BV(%d);", Register_Name, Bit);
			break;
		case OPCODE_sbis:
			snprintf(Code_Line, 255, "skipif (IO[%s] & _BV(%d))", Register_Name, Bit);
			break;
		case OPCODE_sbic:
			snprintf(Code_Line, 255, "skipif (!(IO[%s] & _BV(%d)))",Register_Name, Bit);
			break;
		default:
			snprintf(Code_Line, 255, "%-7s %s, %d", MNemonic[MNemonic_Int], Register_Name, Bit);
	}
	snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_s(int MNemonic_Int) {
	int Bit;
	Bit = Rs;
	snprintf(Code_Line, 255, "%-7s %d", MNemonic[MNemonic_Int], Bit);
	snprintf(Comment_Line, 255, "0x%02x = %d", (1 << Bit), (1 << Bit));
}

void PC_Operation_k(int MNemonic_Int, int Position, char *Pseudocode) {
	int Offset;
	Offset = (2 * Rk);
	if (Offset > 128) Offset -= 256;

	int Target = FixTargetAddress(Position + Offset + 2);
	Register_JumpCall(Position, Target, MNemonic_Int, 0);
	if (Options->Process_Labels == 0) {
		if (Offset > 0) {
			snprintf(Code_Line, 255, "if (%s) goto .+%d;", Pseudocode, Offset);
		} else {
			snprintf(Code_Line, 255, "if (%s) goto .%d", Pseudocode, Offset);
		}
		snprintf(Comment_Line, 255, "0x%02x", Target);
	} else {
		snprintf(Code_Line, 255, "if (%s) goto %s;", Pseudocode, Get_Label_Name(Target, NULL));
	}
}

/************* Now to the callback functions *************/

CALLBACK(adc_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(add_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(sub_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(sbc_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(mov_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(brcc_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "!Carry");
}

CALLBACK(brcs_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "Carry");
}

CALLBACK(breq_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "c1 == c2");
}

CALLBACK(brge_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "c1 (signed)>= c2");
}

CALLBACK(brhc_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "!HalfCarry");
}

CALLBACK(brhs_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "HalfCarry");
}

CALLBACK(brid_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "Global_Interrupts_Disabled()");
}

CALLBACK(brie_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "Global_Interrupts_Enabled()");
}

CALLBACK(brlo_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "c1 (unsigned)< c2");
}

CALLBACK(brlt_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "c1 (signed)< c2");
}

CALLBACK(brmi_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "< 0");
}

CALLBACK(brne_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "c1 != c2");
}

CALLBACK(brpl_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "> 0");
}

CALLBACK(brsh_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "c1 (unsigned)>= c2");
}

CALLBACK(brtc_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "T == 0");
}

CALLBACK(brts_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "T == 1");
}

CALLBACK(brvc_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "Overflow == 0");
}

CALLBACK(brvs_Callback_PC) {
	PC_Operation_k(MNemonic_Int, Position, "Overflow == 1");
}

CALLBACK(out_Callback_PC) {
	int Register_Number;
	const char *Register_Name;
	Register_Number = RA;
	Register_Name = Resolve_IO_Register(Register_Number);
	if (Register_Name) {
		snprintf(Code_Line, 255, "IO[%s] = r%d;", Register_Name, Rr);
	} else {
		snprintf(Code_Line, 255, "IO[0x%02x] = r%d;", Register_Number, Rr);
		snprintf(Comment_Line, 255, "%d", RA);
	}
}

CALLBACK(in_Callback_PC) {
	int Register_Number;
	const char *Register_Name;
	Register_Number = RA;
	Register_Name = Resolve_IO_Register(Register_Number);
	if (Register_Name) {
		snprintf(Code_Line, 255, "r%d = IO[%s];", Rd, Register_Name);
	} else {
		snprintf(Code_Line, 255, "r%d = IO[0x%02x];", Rd, Register_Number);
		snprintf(Comment_Line, 255, "%d", RA);
	}
}

CALLBACK(cli_Callback_PC) {
	snprintf(Code_Line, 255, "Disable_Interrupts();");
}

CALLBACK(sei_Callback_PC) {
	snprintf(Code_Line, 255, "Enable_Interrupts();");
}

CALLBACK(ret_Callback_PC) {
	snprintf(Code_Line, 255, "return;");
	snprintf(After_Code_Line, 255, "\n");
}

CALLBACK(reti_Callback_PC) {
	snprintf(Code_Line, 255, "ireturn;");
	snprintf(After_Code_Line, 255, "\n");
}

CALLBACK(andi_Callback_PC) {
	PC_Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(subi_Callback_PC) {
	PC_Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(sbci_Callback_PC) {
	PC_Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(sbr_Callback_PC) {
	PC_Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(ori_Callback_PC) {
	PC_Operation_Rd16_K(MNemonic_Int);
}

CALLBACK(ldi_Callback_PC) {
	snprintf(Code_Line, 255, "r%d = %d;", Rd + 16, RK);
	snprintf(Comment_Line, 255, "0x%02x", RK);
}

CALLBACK(lds_Callback_PC) {
	const char *MemAddress;
	MemAddress = Tagfile_Resolve_Mem_Address(Rk);
	if (!MemAddress) {
		snprintf(Code_Line, 255, "r%d = Memory[0x%04x];", Rd, Rk);
		snprintf(Comment_Line, 255, "%d", Rk);
	} else {
		snprintf(Code_Line, 255, "r%d = %s;", Rd, MemAddress);
		snprintf(Comment_Line, 255, "0x%04x", Rk);
	}
}

CALLBACK(sts_Callback_PC) {
	const char *MemAddress;
	MemAddress = Tagfile_Resolve_Mem_Address(Rk);
	if (!MemAddress) {
		snprintf(Code_Line, 255, "Memory[0x%04x] = r%d;", Rk, Rd);
		snprintf(Comment_Line, 255, "%d", Rk);
	} else {
		snprintf(Code_Line, 255, "%s = r%d;", MemAddress, Rd);
		snprintf(Comment_Line, 255, "0x%04x", Rk);
	}
}

CALLBACK(call_Callback_PC) {
	int Pos;
	Pos = FixTargetAddress(2 * Rk);
	Register_JumpCall(Position, Pos, MNemonic_Int, 1);
	if (Options->Process_Labels == 0) {
		snprintf(Code_Line, 255, "0x%02x();", Pos);
	} else {
		char *LabelName;
		char *LabelComment = NULL;
		LabelName = Get_Label_Name(Pos, &LabelComment);
		snprintf(Code_Line, 255, "%s();", LabelName);
		if (LabelComment != NULL) {
			snprintf(Comment_Line, 255, "%s", LabelComment);
		} 
	}
}

CALLBACK(rcall_Callback_PC) {
	int Offset;
	Offset = 2 * (Rk);
	if (Offset > 4096) Offset -= 8192;

	int Target = FixTargetAddress(Position + Offset + 2);
	Register_JumpCall(Position, Target, MNemonic_Int, 1);
	if (!Options->Process_Labels) {
		snprintf(Comment_Line, 255, "0x%02x();", Target);
	} else {
		char *LabelName;
		char *LabelComment = NULL;
		LabelName = Get_Label_Name(Target, &LabelComment);
		snprintf(Code_Line, 255, "%s();", LabelName);
		if (LabelComment != NULL) {
			snprintf(Comment_Line, 255, "%s", LabelComment);
		} 
	}
}

CALLBACK(ror_Callback_PC) {
	PC_Operation_Rd(MNemonic_Int);
}

CALLBACK(lsr_Callback_PC) {
	PC_Operation_Rd(MNemonic_Int);
}

CALLBACK(swap_Callback_PC) {
	PC_Operation_Rd(MNemonic_Int);
}

CALLBACK(eor_Callback_PC) {
	if (Rd == Rr) {
		PC_Operation_Rd(OPCODE_clr);
	} else {
		PC_Operation_Rd_Rr(MNemonic_Int);
	}
}

CALLBACK(jmp_Callback_PC) {
	int Pos;
	Pos = FixTargetAddress(2 * Rk);
	if (Options->Process_Labels == 0) {
		snprintf(Code_Line, 255, "goto 0x%02x;", Pos);
	} else {
		snprintf(Code_Line, 255, "goto %s;", Get_Label_Name(Pos, NULL));
	}
	Register_JumpCall(Position, Pos, MNemonic_Int, 0);
}

CALLBACK(rjmp_Callback_PC) {
	int Offset;
	Offset = 2 * (Rk);
	if (Offset > 4096) Offset -= 8192;

	int Target = FixTargetAddress(Position + Offset + 2);
	Register_JumpCall(Position, Target, MNemonic_Int, 0);

	if (Options->Process_Labels == 0) {
		if (Offset>0) {
			snprintf(Code_Line, 255, "goto .+%d;", Offset);
		} else {
			snprintf(Code_Line, 255, "goto .%d", Offset);
		}
		if (Target >= 0) {
			snprintf(Comment_Line, 255, "0x%02x", Target);
		} else {
			snprintf(Comment_Line, 255, "-0x%02x - Illegal jump position -- specify flash size!", -(Target));
		}
	} else {
		snprintf(Code_Line, 255, "goto %s;", Get_Label_Name(Position + Offset + 2, NULL));
	}
}

CALLBACK(cpi_Callback_PC) {
	if (RK == 0) {
		snprintf(Code_Line, 255, "cmp(r%d, 0);", Rd + 16);
	} else {
		snprintf(Code_Line, 255, "cmp(r%d, 0x%02x);", Rd + 16, RK);
		snprintf(Comment_Line, 255, "%d", RK);
	}
}

CALLBACK(asr_Callback_PC) {
	PC_Operation_Rd(MNemonic_Int);
}

CALLBACK(dec_Callback_PC) {
	snprintf(Code_Line, 255, "r%d--;", Rd);
}

CALLBACK(inc_Callback_PC) {
	snprintf(Code_Line, 255, "r%d++;", Rd);
}

CALLBACK(cp_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(cpc_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(cpse_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(and_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(or_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(mul_Callback_PC) {
	PC_Operation_Rd_Rr(MNemonic_Int);
}

CALLBACK(sbi_Callback_PC) {
	PC_Operation_A_b(MNemonic_Int);
}

CALLBACK(sbic_Callback_PC) {
	PC_Operation_A_b(MNemonic_Int);
}

CALLBACK(sbis_Callback_PC) {
	PC_Operation_A_b(MNemonic_Int);
}

CALLBACK(cbi_Callback_PC) {
	PC_Operation_A_b(MNemonic_Int);
}

CALLBACK(ser_Callback_PC) {
	snprintf(Code_Line, 255, "r%d = 0xff;", Rd + 16);
	snprintf(Comment_Line, 255, "255");
}

CALLBACK(adiw_Callback_PC) {
	if (RK != 1) {
		snprintf(Code_Line, 255, "[r%d:r%d] += 0x%02x;", 2 * Rd + 25, 2 * Rd + 24, RK);
		snprintf(Comment_Line, 255, "%d", RK);
	} else {
		snprintf(Code_Line, 255, "[r%d:r%d]++;", 2 * Rd + 25, 2 * Rd + 24);
	}
}

CALLBACK(movw_Callback_PC) {
	snprintf(Code_Line, 255, "[r%d:r%d] = [r%d:r%d];", (2 * Rd) + 1, 2 * Rd, (2 * Rr) + 1, 2 * Rr);
}

CALLBACK(lpm1_Callback_PC) {
	snprintf(Code_Line, 255, "r0 = Flash[r30:r31];");
}

CALLBACK(st2_Callback_PC) {
	snprintf(Code_Line, 255, "Flash[[r26:r27]++] = r%d;", Rr);
}

