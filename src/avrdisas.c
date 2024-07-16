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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include "Globals.h"
#include "Callbacks_Assembly.h"
#include "Callbacks_PseudoCode.h"
#include "Options.h"
#include "JumpCall.h"
#include "IORegisters.h"
#include "MNemonics.h"
#include "Tagfile.h"
	
#define READ_BUFFER		131072			/* 128 kB */

static int Number_Opcodes = 0;
static struct Opcode Opcodes[256];
static int Registers[256];

struct Options Options;
static char Code_Line[256];
static char Comment_Line[256];
static char After_Code_Line[256];

void Display_Registers() {
	int i;
	printf("Register dump:\n");
	for (i = 0; i < 256; i++) {
		if (Registers[i] != 0) {
			printf("Registers[%3d] '%c': %d = 0x%x\n", i, i, Registers[i], Registers[i]);
		}
	}
	printf("End of register dump.\n");
}

int Compare_Opcode(char *Bitstream, char *Bitmask) {
	size_t i;
	char Bit;

	for (i = 0; i < strlen(Bitmask); i++) {
		if ((Bitmask[i] != 'x') && (Bitmask[i] != '1') && (Bitmask[i] != '0')) {
			fprintf(stderr, "Invalid Bitmask!\n");
			return 0;
		}
		
		if (Bitmask[i] == 'x') continue;	/* Ignore character */
		/* Retrieve the i-th Bit of Bitstream */
		Bit = (Bitstream[i / 8] >> (7 - (i % 8))) & 1;
/*		printf("Bit %d is %d [should be %c]\n",i,Bit,Bitmask[i]); */
		if ((Bitmask[i] == '1') && (Bit == 1)) continue;
		if ((Bitmask[i] == '0') && (Bit == 0)) continue;
		return 0;	/* No match */
	}
	return 1;		/* Match */
}

void Register_Opcode(void (*Callback)(char*, int, int), const char *New_Opcode_String, int New_MNemonic) {
	Number_Opcodes++;
	Opcodes[Number_Opcodes-1].Opcode_String = malloc(strlen(New_Opcode_String) + 1);
	strcpy(Opcodes[Number_Opcodes-1].Opcode_String, New_Opcode_String);
	Opcodes[Number_Opcodes-1].MNemonic = New_MNemonic;
	Opcodes[Number_Opcodes-1].Callback = Callback;
}

void Supersede_Opcode(void (*Callback)(char*, int, int), int New_MNemonic) {
	int i;
	for (i = 0; i < Number_Opcodes; i++) {
		if (Opcodes[i].MNemonic == New_MNemonic) {
			/* Supersede callback */
			Opcodes[i].Callback = Callback;
			return;
		}
	}
	fprintf(stderr, "Error: No callback to supersede opcode %d found (%s).\n", New_MNemonic, MNemonic[New_MNemonic]);
}

int Get_Bitmask_Length(char *Bitmask) {
	int Length = 0;
	size_t i;
	for (i = 0; i < strlen(Bitmask); i++) {
		if (Bitmask[i] != ' ') Length++;
	}
	return Length;
}

void Clear_Registers() {
	int i;
	for (i = 0; i < 256; i++) Registers[i] = 0;
}

char Get_From_Bitmask(char *Bitmask, int Byte, int Bit) {
	size_t i;
	int Cnt = 0;
	int GetBit;
	GetBit = (Byte * 8) + Bit;
	for (i = 0; i < strlen(Bitmask); i++) {
		if (Bitmask[i] != ' ') {
			if (Cnt == GetBit) return Bitmask[i];
			Cnt++;
		}
	}
	return '?';
}

void Display_Binary(char *Bitstream, int Count) {
	int i, j;
	for (i = 0; i < Count; i++) {
		for (j = 7; j >= 0; j--) {
			if ((Bitstream[i] & (1 << j)) != 0) printf("1");
				else printf("0");
			if (j == 4) printf(" ");
		}
		printf("  ");
		if ((((i + 1) % 2) == 0) && (i != 0)) printf("  ");
	}
	printf("\n");
}


int Match_Opcode(char *Bitmask, char *Bitstream) {
	int i;
	int Length;
	int Byte_Mask, Bit_Mask;
	int Byte_Stream, Bit_Stream;
	char Mask_Val, Stream_Val;
	
	Clear_Registers();
	Length = Get_Bitmask_Length(Bitmask);
	
	for (i = 0; i < Length; i++) {
		Byte_Mask = i / 8;
		Bit_Mask = i % 8;
		
		Byte_Stream = i / 8;
		Byte_Stream ^= 1;	/* Invert last bit */
		Bit_Stream = 7 - (i % 8);
		
		Mask_Val = Get_From_Bitmask(Bitmask, Byte_Mask, Bit_Mask);
		Stream_Val = (Bitstream[Byte_Stream] >> Bit_Stream) & 0x01;
		
/*		printf("Extracting Bit %2d: Maske = (%d, %d) [%c], Stream = (%d, %d) [%d] ",i,Byte_Mask,Bit_Mask,Mask_Val,Byte_Stream,Bit_Stream,Stream_Val); */
		if ((Mask_Val == '0') || (Mask_Val == '1')) {
			/* This Bit is a identification Bit */
			if (Mask_Val == '0') {
				if (Stream_Val == 1) {
/*					printf("\nMatch failed.\n");*/
					return 0;
				}
			} else {
				if (Stream_Val == 0) {
/*					printf("\nMatch failed.\n");*/
					return 0;
				}
			}
		} else {
			/* This Bit is a register Bit, set in appropriate place */
			Registers[(int)Mask_Val] <<= 1;
			Registers[(int)Mask_Val] |= Stream_Val;
/*			printf("-> %d Stored [%x]",Stream_Val,Registers[(int)Mask_Val]); */
		}
/*		printf("\n"); */
	}
	return 1;
}

int Get_Next_Opcode(char *Bitstream) {
	int i;
	for (i = 0; i < Number_Opcodes; i++) {
		if (Match_Opcode(Opcodes[i].Opcode_String, Bitstream) == 1) {
			return i;
		}
	}
	return -1;
}

void Disassemble(char *Filename) {
	FILE *f;
	char Bitstream[READ_BUFFER];
	int Read;
	int Pos;
	int Opcode;
	int i;

#ifndef DOS_BASED_OS
	f = fopen(Filename, "r");
#else
	f = fopen(Filename, "rb");
#endif
	if (!f) {
		fprintf(stderr, "Error opening file '%s' for disassembly: %s\n", Filename, strerror(errno));
		return;
	}

	if (Options.Show_PseudoCode == 1) {
		printf("#include <stdio.h>\n");
		printf("\n");
		printf("/* Disassembly of %s (pseudocode) */\n", Filename);
		printf("\n");	
		printf("int  r0,  r1,  r2,  r3,  r4,  r5,  r6,  r7,  r8,  r9;\n");
		printf("int r10, r11, r12, r13, r14, r15, r16, r17, r18, r19;\n");
		printf("int r20, r21, r22, r23, r24, r25, r26, r27, r28, r29;\n");
		printf("int r30, r31, r32;\n");
		printf("\n");
		printf("int main(int argc, char **argv) {\n");
	} else {
		if (Options.CodeStyle == CODESTYLE_AVRGCC) {
			printf("; Disassembly of %s (avr-gcc style)\n", Filename);
		} else {
			printf("; Disassembly of %s (AVR instruction set style)\n", Filename);
		}
		printf("\n");
	}
	
	Read = fread(Bitstream, 1, READ_BUFFER, f);
	Options.Pass = 1;
	Pos = 0;

	if ((Options.Process_Labels == 1) || ((!Options.Show_PseudoCode) && (Options.CodeStyle == CODESTYLE_AVRGCC))) {
		/* Preprocess to gather jump labels or to gain knowledge about registers which are being used */
		while (Pos < Read) {
			Opcode = Get_Next_Opcode(Bitstream + Pos);
			if (Opcode == -1) {
				Pos += 2;
			} else {
				Opcodes[Opcode].Callback(Bitstream + Pos, Pos, Opcodes[Opcode].MNemonic);
				Pos += Get_Bitmask_Length(Opcodes[Opcode].Opcode_String) / 8;
			}
		}
		Enumerate_Labels();
		Options.Pass = 2;
		Pos = 0;
	}

	if (Options.CodeStyle == CODESTYLE_AVRGCC) {
		Emit_Used_IO_Registers();
		printf(".text\n");
		printf("main:\n");
	}

	while (Pos < Read) {
		int Added;

		/* Check if this is actually code or maybe only data from tagfile */
		Added = Tagfile_Process_Data(Bitstream, Pos);
		if (Added != 0) {
			/* Data was added */
			Pos += Added;
			continue;
		}
		
		Opcode = Get_Next_Opcode(Bitstream + Pos);
		if (Opcode != -1) {
			Code_Line[0] = 0;
			Comment_Line[0] = 0;
			After_Code_Line[0] = 0;
			Opcodes[Opcode].Callback(Bitstream + Pos, Pos, Opcodes[Opcode].MNemonic);
			
			if (Options.Process_Labels) {
				Print_JumpCalls(Pos);
			}
				
			if (Options.Show_Addresses) printf("%4x:   ", Pos);
			if (Options.Show_Cycles) {
				const char *Cycle = Cycles[Opcodes[Opcode].MNemonic];
				if (!Cycle) printf("      ");
					else printf("[%-3s] ", Cycle);
			}

			if (Options.Show_Opcodes) {
				/* Now display the Opcode */
				for (i = 0; i < (Get_Bitmask_Length(Opcodes[Opcode].Opcode_String)) / 8; i++) {
					printf("%02x ", (unsigned char)(Bitstream[Pos+i]));
				}
				printf(" ");
				/* Missing spaces */
				for (i = 0; i < 5 - ((Get_Bitmask_Length(Opcodes[Opcode].Opcode_String)) / 8); i++) {
					printf("   ");
				}
			}
				
			if (Code_Line[0] == 0) {
				/* No code was generated? */
				printf("; - Not implemented opcode: %d -\n", Opcodes[Opcode].MNemonic);
			} else {
				if ((Comment_Line[0] == 0) || (!Options.Show_Comments)) {
					/* No comment */
					printf("%s\n", Code_Line);
				} else {
					/* Comment available */
					if (!Options.Show_PseudoCode) {
						printf("%-23s ; %s\n", Code_Line, Comment_Line);
					} else {
						printf("%-35s ; %s\n", Code_Line, Comment_Line);
					}
				}
			}
			printf("%s", After_Code_Line);
			
			Pos += Get_Bitmask_Length(Opcodes[Opcode].Opcode_String) / 8;
		} else {
			printf(".word 0x%02x%02x    ; Invalid opcode at 0x%04x (%d). Disassembler skipped two bytes.\n", ((unsigned char*)Bitstream)[Pos + 1], ((unsigned char *)Bitstream)[Pos], Pos, Pos);
			Pos += 2;
		}
	}
	fclose(f);
	
	if (Options.Show_PseudoCode) {
		printf("}\n");
		printf("\n");
	}
}

void Display_Opcodes() {
	unsigned int i;
	printf("%d opcodes registered:\n", Number_Opcodes);
	for (i = 0; i < Number_Opcodes; i++) {
		/* This invokes UB as a function pointer is converted to void* - beware */
		printf("%3d: '%-80s' -> %p\n", i, Opcodes[i].Opcode_String, (void*)Opcodes[i].Callback);
	}
}

int Get_Specifity(char *Opcode) {
	size_t i;
	int Specifity = 0;
	for (i = 0; i < strlen(Opcode); i++) {
		if ((Opcode[i] == '0') || (Opcode[i] == '1')) Specifity++;
	}
	return Specifity;
}

int Comparison(const void *Element1, const void *Element2) {
	struct Opcode *OC1, *OC2;
	int SP1, SP2;
	OC1 = (struct Opcode*)Element1;
	OC2 = (struct Opcode*)Element2;
	SP1 = Get_Specifity(OC1->Opcode_String);
	SP2 = Get_Specifity(OC2->Opcode_String);
	if (SP1 < SP2) return 1;
		else if (SP2 == SP1) return 0;
	return -1;
}

int main(int argc, char **argv) {
	Options_Default(&Options);
	if (!Options_ParseCmdLine(&Options, argc, argv)) return 1;

	Activate_Callbacks(Code_Line, Comment_Line, After_Code_Line, Registers, &Options);
	Activate_PC_Callbacks(Code_Line, Comment_Line, After_Code_Line, Registers, &Options);
	Register_Opcode(adc_Callback,		"0001 11rd  dddd rrrr",								OPCODE_adc);
	Register_Opcode(add_Callback,		"0000 11rd  dddd rrrr",								OPCODE_add);
	Register_Opcode(adiw_Callback,		"1001 0110  KKdd KKKK",								OPCODE_adiw);
	Register_Opcode(and_Callback,		"0010 00rd  dddd rrrr",								OPCODE_and);
	Register_Opcode(andi_Callback,		"0111 KKKK  dddd KKKK",								OPCODE_andi);
	Register_Opcode(asr_Callback,		"1001 010d  dddd 0101",								OPCODE_asr);
	Register_Opcode(bclr_Callback,		"1001 0100  1sss 1000",								OPCODE_bclr);
	Register_Opcode(bld_Callback,		"1111 100d  dddd 0bbb",								OPCODE_bld);
	Register_Opcode(brbc_Callback,		"1111 01kk  kkkk ksss",								OPCODE_brbc);
	Register_Opcode(brbs_Callback,		"1111 00kk  kkkk ksss",								OPCODE_brbs);
	Register_Opcode(brcc_Callback,		"1111 01kk  kkkk k000",								OPCODE_brcc);
	Register_Opcode(brcs_Callback,		"1111 00kk  kkkk k000",								OPCODE_brcs);
	Register_Opcode(break_Callback,		"1001 0101  1001 1000",								OPCODE_break);
	Register_Opcode(breq_Callback,		"1111 00kk  kkkk k001",								OPCODE_breq);
	Register_Opcode(brge_Callback,		"1111 01kk  kkkk k100",								OPCODE_brge);
	Register_Opcode(brhc_Callback,		"1111 01kk  kkkk k101",								OPCODE_brhc);
	Register_Opcode(brhs_Callback,		"1111 00kk  kkkk k101",								OPCODE_brhs);
	Register_Opcode(brid_Callback,		"1111 01kk  kkkk k111",								OPCODE_brid);
	Register_Opcode(brie_Callback,		"1111 00kk  kkkk k111",								OPCODE_brie);
	Register_Opcode(brlo_Callback,		"1111 00kk  kkkk k000",								OPCODE_brlo);
	Register_Opcode(brlt_Callback,		"1111 00kk  kkkk k100",								OPCODE_brlt);
	Register_Opcode(brmi_Callback,		"1111 00kk  kkkk k010",								OPCODE_brmi);
	Register_Opcode(brne_Callback,		"1111 01kk  kkkk k001",								OPCODE_brne);
	Register_Opcode(brpl_Callback,		"1111 01kk  kkkk k010",								OPCODE_brpl);
	Register_Opcode(brsh_Callback,		"1111 01kk  kkkk k000",								OPCODE_brsh);
	Register_Opcode(brtc_Callback,		"1111 01kk  kkkk k110",								OPCODE_brtc);
	Register_Opcode(brts_Callback,		"1111 00kk  kkkk k110",								OPCODE_brts);
	Register_Opcode(brvc_Callback,		"1111 01kk  kkkk k011",								OPCODE_brvc);
	Register_Opcode(brvs_Callback,		"1111 00kk  kkkk k011",								OPCODE_brvs);
	Register_Opcode(bset_Callback,		"1001 0100  0sss 1000",								OPCODE_bset);
	Register_Opcode(bst_Callback,		"1111 101d  dddd 0bbb",								OPCODE_bst);
	Register_Opcode(call_Callback,		"1001 010k  kkkk 111k    kkkk kkkk  kkkk kkkk",		OPCODE_call);
	Register_Opcode(cbi_Callback,		"1001 1000  AAAA Abbb",								OPCODE_cbi);
	Register_Opcode(clc_Callback,		"1001 0100  1000 1000",								OPCODE_clc);
	Register_Opcode(clh_Callback,		"1001 0100  1101 1000",								OPCODE_clh);
	Register_Opcode(cli_Callback,		"1001 0100  1111 1000",								OPCODE_cli);
	Register_Opcode(cln_Callback,		"1001 0100  1010 1000",								OPCODE_cln);
/*	Register_Opcode(clr_Callback,		"0010 01dd  dddd dddd",								OPCODE_clr); */		/* Implied by eor */
	Register_Opcode(cls_Callback,		"1001 0100  1100 1000",								OPCODE_cls);
	Register_Opcode(clt_Callback,		"1001 0100  1110 1000",								OPCODE_clt);
	Register_Opcode(clv_Callback,		"1001 0100  1011 1000",								OPCODE_clv);
	Register_Opcode(clz_Callback,		"1001 0100  1001 1000",								OPCODE_clz);
	Register_Opcode(com_Callback,		"1001 010d  dddd 0000",								OPCODE_com);
	Register_Opcode(cp_Callback,		"0001 01rd  dddd rrrr",								OPCODE_cp);
	Register_Opcode(cpc_Callback,		"0000 01rd  dddd rrrr",								OPCODE_cpc);
	Register_Opcode(cpi_Callback,		"0011 KKKK  dddd KKKK",								OPCODE_cpi);
	Register_Opcode(cpse_Callback,		"0001 00rd  dddd rrrr",								OPCODE_cpse);
	Register_Opcode(dec_Callback,		"1001 010d  dddd 1010",								OPCODE_dec);
	Register_Opcode(eicall_Callback,	"1001 0101  0001 1001",								OPCODE_eicall);
	Register_Opcode(eijmp_Callback,		"1001 0100  0001 1001",								OPCODE_eijmp);
	Register_Opcode(elpm1_Callback,		"1001 0101  1101 1000",								OPCODE_elpm_1);
	Register_Opcode(elpm2_Callback,		"1001 000d  dddd 0110",								OPCODE_elpm_2);
	Register_Opcode(elpm3_Callback,		"1001 000d  dddd 0111",								OPCODE_elpm_3);
	Register_Opcode(eor_Callback,		"0010 01rd  dddd rrrr",								OPCODE_eor);
	Register_Opcode(fmul_Callback,		"0000 0011  0ddd 1rrr",								OPCODE_fmul);
	Register_Opcode(fmuls_Callback,		"0000 0011  1ddd 0rrr",								OPCODE_fmuls);
	Register_Opcode(fmulsu_Callback,	"0000 0011  1ddd 1rrr",								OPCODE_fmulsu);
	Register_Opcode(icall_Callback,		"1001 0101  0000 1001",								OPCODE_icall);
	Register_Opcode(ijmp_Callback,		"1001 0100  0000 1001",								OPCODE_ijmp);
	Register_Opcode(in_Callback,		"1011 0AAd  dddd AAAA",								OPCODE_in);
	Register_Opcode(inc_Callback,		"1001 010d  dddd 0011",								OPCODE_inc);
	Register_Opcode(jmp_Callback,		"1001 010k  kkkk 110k    kkkk kkkk  kkkk kkkk",		OPCODE_jmp);
	Register_Opcode(ld1_Callback,		"1001 000d  dddd 1100",								OPCODE_ld_1);
	Register_Opcode(ld2_Callback,		"1001 000d  dddd 1101",								OPCODE_ld_2);
	Register_Opcode(ld3_Callback,		"1001 000d  dddd 1110",								OPCODE_ld_3);
	Register_Opcode(ldy1_Callback,		"1000 000d  dddd 1000",								OPCODE_ld_4);
	Register_Opcode(ldy2_Callback,		"1001 000d  dddd 1001",								OPCODE_ld_5);
	Register_Opcode(ldy3_Callback,		"1001 000d  dddd 1010",								OPCODE_ld_6);
	Register_Opcode(ldy4_Callback,		"10q0 qq0d  dddd 1qqq",								OPCODE_ldd_1);
	Register_Opcode(ldz1_Callback,		"1000 000d  dddd 0000",								OPCODE_ld_7);
	Register_Opcode(ldz2_Callback,		"1001 000d  dddd 0001",								OPCODE_ld_8);
	Register_Opcode(ldz3_Callback,		"1001 000d  dddd 0010",								OPCODE_ld_9);
	Register_Opcode(ldz4_Callback,		"10q0 qq0d  dddd 0qqq",								OPCODE_ldd_2);
	Register_Opcode(ldi_Callback,		"1110 KKKK  dddd KKKK",								OPCODE_ldi);
	Register_Opcode(lds_Callback,		"1001 000d  dddd 0000    kkkk kkkk  kkkk kkkk",		OPCODE_lds);
	Register_Opcode(lpm1_Callback,		"1001 0101  1100 1000",								OPCODE_lpm_1);
	Register_Opcode(lpm2_Callback,		"1001 000d  dddd 0100",								OPCODE_lpm_2);
	Register_Opcode(lpm3_Callback,		"1001 000d  dddd 0101",								OPCODE_lpm_3);
/*	Register_Opcode(lsl_Callback,		"0000 11dd  dddd dddd",								OPCODE_lsl);	*/		/* Implied by add */
	Register_Opcode(lsr_Callback,		"1001 010d  dddd 0110",								OPCODE_lsr);
	Register_Opcode(mov_Callback,		"0010 11rd  dddd rrrr",								OPCODE_mov);
	Register_Opcode(movw_Callback,		"0000 0001  dddd rrrr",								OPCODE_movw);
	Register_Opcode(mul_Callback,		"1001 11rd  dddd rrrr",								OPCODE_mul);
	Register_Opcode(muls_Callback,		"0000 0010  dddd rrrr",								OPCODE_muls);
	Register_Opcode(mulsu_Callback,		"0000 0011  0ddd 0rrr",								OPCODE_mulsu);
	Register_Opcode(neg_Callback,		"1001 010d  dddd 0001",								OPCODE_neg);
	Register_Opcode(nop_Callback,		"0000 0000  0000 0000",								OPCODE_nop);
	Register_Opcode(or_Callback,		"0010 10rd  dddd rrrr",								OPCODE_or);
	Register_Opcode(ori_Callback,		"0110 KKKK  dddd KKKK",								OPCODE_ori);
	Register_Opcode(out_Callback,		"1011 1AAr  rrrr AAAA",								OPCODE_out);
	Register_Opcode(pop_Callback,		"1001 000d  dddd 1111",								OPCODE_pop);
	Register_Opcode(push_Callback,		"1001 001d  dddd 1111",								OPCODE_push);
	Register_Opcode(rcall_Callback,		"1101 kkkk  kkkk kkkk",								OPCODE_rcall);
	Register_Opcode(ret_Callback,		"1001 0101  0000 1000",								OPCODE_ret);
	Register_Opcode(reti_Callback,		"1001 0101  0001 1000",								OPCODE_reti);
	Register_Opcode(rjmp_Callback,		"1100 kkkk  kkkk kkkk",								OPCODE_rjmp);
/*	Register_Opcode(rol_Callback,		"0001 11dd  dddd dddd",								OPCODE_rol);	*/		/* Implied by adc */
	Register_Opcode(ror_Callback,		"1001 010d  dddd 0111",								OPCODE_ror);
	Register_Opcode(sbc_Callback,		"0000 10rd  dddd rrrr",								OPCODE_sbc);
	Register_Opcode(sbci_Callback,		"0100 KKKK  dddd KKKK",								OPCODE_sbci);
	Register_Opcode(sbi_Callback,		"1001 1010  AAAA Abbb",								OPCODE_sbi);
	Register_Opcode(sbic_Callback,		"1001 1001  AAAA Abbb",								OPCODE_sbic);
	Register_Opcode(sbis_Callback,		"1001 1011  AAAA Abbb",								OPCODE_sbis);
	Register_Opcode(sbiw_Callback,		"1001 0111  KKdd KKKK",								OPCODE_sbiw);
	Register_Opcode(sbr_Callback,		"0110 KKKK  dddd KKKK",								OPCODE_sbr);
	Register_Opcode(sbrc_Callback,		"1111 110r  rrrr 0bbb",								OPCODE_sbrc);
	Register_Opcode(sbrs_Callback,		"1111 111r  rrrr 0bbb",								OPCODE_sbrs);
	Register_Opcode(sec_Callback,		"1001 0100  0000 1000",								OPCODE_sec);
	Register_Opcode(seh_Callback,		"1001 0100  0101 1000",								OPCODE_seh);
	Register_Opcode(sei_Callback,		"1001 0100  0111 1000",								OPCODE_sei);
	Register_Opcode(sen_Callback,		"1001 0100  0010 1000",								OPCODE_sen);
	Register_Opcode(ser_Callback,		"1110 1111  dddd 1111",								OPCODE_ser);
	Register_Opcode(ses_Callback,		"1001 0100  0100 1000",								OPCODE_ses);
	Register_Opcode(set_Callback,		"1001 0100  0110 1000",								OPCODE_set);
	Register_Opcode(sev_Callback,		"1001 0100  0011 1000",								OPCODE_sev);
	Register_Opcode(sez_Callback,		"1001 0100  0001 1000",								OPCODE_sez);
	Register_Opcode(sleep_Callback,		"1001 0101  1000 1000",								OPCODE_sleep);
	Register_Opcode(spm_Callback,		"1001 0101  1110 1000",								OPCODE_spm);
	Register_Opcode(st1_Callback,		"1001 001r  rrrr 1100",								OPCODE_st_1);
	Register_Opcode(st2_Callback,		"1001 001r  rrrr 1101",								OPCODE_st_2);
	Register_Opcode(st3_Callback,		"1001 001r  rrrr 1110",								OPCODE_st_3);
	Register_Opcode(sty1_Callback,		"1000 001r  rrrr 1000",								OPCODE_st_4);
	Register_Opcode(sty2_Callback,		"1001 001r  rrrr 1001",								OPCODE_st_5);
	Register_Opcode(sty3_Callback,		"1001 001r  rrrr 1010",								OPCODE_st_6);
	Register_Opcode(sty4_Callback,		"10q0 qq1r  rrrr 1qqq",								OPCODE_std_1);
	Register_Opcode(stz1_Callback,		"1000 001r  rrrr 0000",								OPCODE_st_7);
	Register_Opcode(stz2_Callback,		"1001 001r  rrrr 0001",								OPCODE_st_8);
	Register_Opcode(stz3_Callback,		"1001 001r  rrrr 0010",								OPCODE_st_9);
	Register_Opcode(stz4_Callback,		"10q0 qq1r  rrrr 0qqq",								OPCODE_std_2);
	Register_Opcode(sts_Callback,		"1001 001d  dddd 0000    kkkk kkkk  kkkk kkkk",		OPCODE_sts);
	Register_Opcode(sub_Callback,		"0001 10rd  dddd rrrr",								OPCODE_sub);
	Register_Opcode(subi_Callback,		"0101 KKKK  dddd KKKK",								OPCODE_subi);
	Register_Opcode(swap_Callback,		"1001 010d  dddd 0010",								OPCODE_swap);
/*	Register_Opcode(tst_Callback,		"0010 00dd  dddd dddd",								OPCODE_tst);	*/		/* Implied by and */
	Register_Opcode(wdr_Callback,		"1001 0101  1010 1000",								OPCODE_wdr);

	if (Options.Show_PseudoCode) {
		Supersede_Opcode(adc_Callback_PC,	OPCODE_adc);
		Supersede_Opcode(add_Callback_PC,	OPCODE_add);
		Supersede_Opcode(sub_Callback_PC,	OPCODE_sub);
		Supersede_Opcode(sbc_Callback_PC,	OPCODE_sbc);
		Supersede_Opcode(mov_Callback_PC,	OPCODE_mov);
		Supersede_Opcode(brcc_Callback_PC,	OPCODE_brcc);
		Supersede_Opcode(brcs_Callback_PC,	OPCODE_brcs);
		Supersede_Opcode(breq_Callback_PC,	OPCODE_breq);
		Supersede_Opcode(brge_Callback_PC,	OPCODE_brge);
		Supersede_Opcode(brhc_Callback_PC,	OPCODE_brhc);
		Supersede_Opcode(brhs_Callback_PC,	OPCODE_brhs);
		Supersede_Opcode(brid_Callback_PC,	OPCODE_brid);
		Supersede_Opcode(brie_Callback_PC,	OPCODE_brie);
		Supersede_Opcode(brlo_Callback_PC,	OPCODE_brlo);
		Supersede_Opcode(brlt_Callback_PC,	OPCODE_brlt);
		Supersede_Opcode(brmi_Callback_PC,	OPCODE_brmi);
		Supersede_Opcode(brne_Callback_PC,	OPCODE_brne);
		Supersede_Opcode(brpl_Callback_PC,	OPCODE_brpl);
		Supersede_Opcode(brsh_Callback_PC,	OPCODE_brsh);
		Supersede_Opcode(brtc_Callback_PC,	OPCODE_brtc);
		Supersede_Opcode(brts_Callback_PC,	OPCODE_brts);
		Supersede_Opcode(brvc_Callback_PC,	OPCODE_brvc);
		Supersede_Opcode(brvs_Callback_PC,	OPCODE_brvs);
		Supersede_Opcode(out_Callback_PC,	OPCODE_out);
		Supersede_Opcode(in_Callback_PC,	OPCODE_in);
		Supersede_Opcode(cli_Callback_PC,	OPCODE_cli);
		Supersede_Opcode(sei_Callback_PC,	OPCODE_sei);
		Supersede_Opcode(ret_Callback_PC,	OPCODE_ret);
		Supersede_Opcode(reti_Callback_PC,	OPCODE_reti);
		Supersede_Opcode(andi_Callback_PC,	OPCODE_andi);
		Supersede_Opcode(subi_Callback_PC,	OPCODE_subi);
		Supersede_Opcode(sbci_Callback_PC,	OPCODE_sbci);
		Supersede_Opcode(sbr_Callback_PC,	OPCODE_sbr);
		Supersede_Opcode(ori_Callback_PC,	OPCODE_ori);
		Supersede_Opcode(ldi_Callback_PC,	OPCODE_ldi);
		Supersede_Opcode(lds_Callback_PC,	OPCODE_lds);
		Supersede_Opcode(sts_Callback_PC,	OPCODE_sts);
		Supersede_Opcode(call_Callback_PC,	OPCODE_call);
		Supersede_Opcode(rcall_Callback_PC,	OPCODE_rcall);
		Supersede_Opcode(ror_Callback_PC,	OPCODE_ror);
		Supersede_Opcode(lsr_Callback_PC,	OPCODE_lsr);
		Supersede_Opcode(eor_Callback_PC,	OPCODE_eor);
		Supersede_Opcode(swap_Callback_PC,	OPCODE_swap);
		Supersede_Opcode(jmp_Callback_PC,	OPCODE_jmp);
		Supersede_Opcode(rjmp_Callback_PC,	OPCODE_rjmp);
		Supersede_Opcode(cpi_Callback_PC,	OPCODE_cpi);
		Supersede_Opcode(asr_Callback_PC,	OPCODE_asr);
		Supersede_Opcode(inc_Callback_PC,	OPCODE_inc);
		Supersede_Opcode(dec_Callback_PC,	OPCODE_dec);
		Supersede_Opcode(cp_Callback_PC,	OPCODE_cp);
		Supersede_Opcode(cpc_Callback_PC,	OPCODE_cpc);
		Supersede_Opcode(cpse_Callback_PC,	OPCODE_cpse);
		Supersede_Opcode(and_Callback_PC,	OPCODE_and);
		Supersede_Opcode(or_Callback_PC,	OPCODE_or);
		Supersede_Opcode(mul_Callback_PC,	OPCODE_mul);
		Supersede_Opcode(sbi_Callback_PC,	OPCODE_sbi);
		Supersede_Opcode(sbis_Callback_PC,	OPCODE_sbis);
		Supersede_Opcode(sbic_Callback_PC,	OPCODE_sbic);
		Supersede_Opcode(cbi_Callback_PC,	OPCODE_cbi);
		Supersede_Opcode(ser_Callback_PC,	OPCODE_ser);
		Supersede_Opcode(movw_Callback_PC,	OPCODE_movw);
		Supersede_Opcode(adiw_Callback_PC,	OPCODE_adiw);
		Supersede_Opcode(lpm1_Callback_PC,	OPCODE_lpm_1);
		Supersede_Opcode(st2_Callback_PC,	OPCODE_st_2);
	}

	qsort(Opcodes, Number_Opcodes, sizeof(struct Opcode), Comparison);
	
	Disassemble(Options.Filename);
	return 0;
}

