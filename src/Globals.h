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

#define Rd				(Registers[(int)'d'])
#define Rr				(Registers[(int)'r'])
#define Rk				(Registers[(int)'k'])
#define RK				(Registers[(int)'K'])
#define Rs				(Registers[(int)'s'])
#define RA				(Registers[(int)'A'])
#define Rb				(Registers[(int)'b'])
#define Rq				(Registers[(int)'q'])

#define CALLBACK(name)	void name(char *Bitstream, int Position, int MNemonic_Int)

#define CODESTYLE_AVR_INSTRUCTION_SET		0
#define CODESTYLE_AVRGCC					1

struct Opcode {
	char *Opcode_String;
	void (*Callback)(char*, int, int);
	int MNemonic;
};

struct JumpCall {
	int From;
	int To;
	int Type;
	unsigned int LabelNumber;
	unsigned char FunctionCall;
};

struct Options {
	char Show_Addresses;
	char Show_Opcodes;
	char Show_Comments;
	char Show_Cycles;
	char Show_PseudoCode;
	char Filename[256];
	char MCU[8];
	char Tagfile[256];
	char CodeStyle;
	char Process_Labels;
	char Pass;
	int FlashSize;
};

struct IO_Register {
	int Address;
	char Name[16];
	unsigned char Used;
};

#define OPCODE_adc	0
#define OPCODE_add	1
#define OPCODE_adiw	2
#define OPCODE_and	3
#define OPCODE_andi	4
#define OPCODE_asr	5
#define OPCODE_bclr	6
#define OPCODE_bld	7
#define OPCODE_brbc	8
#define OPCODE_brbs	9
#define OPCODE_brcc	10
#define OPCODE_brcs	11
#define OPCODE_break	12
#define OPCODE_breq	13
#define OPCODE_brge	14
#define OPCODE_brhc	15
#define OPCODE_brhs	16
#define OPCODE_brid	17
#define OPCODE_brie	18
#define OPCODE_brlo	19
#define OPCODE_brlt	20
#define OPCODE_brmi	21
#define OPCODE_brne	22
#define OPCODE_brpl	23
#define OPCODE_brsh	24
#define OPCODE_brtc	25
#define OPCODE_brts	26
#define OPCODE_brvc	27
#define OPCODE_brvs	28
#define OPCODE_bset	29
#define OPCODE_bst	30
#define OPCODE_call	31
#define OPCODE_cbi	32
#define OPCODE_cbr	33
#define OPCODE_clc	34
#define OPCODE_clh	35
#define OPCODE_cli	36
#define OPCODE_cln	37
#define OPCODE_clr	38
#define OPCODE_cls	39
#define OPCODE_clt	40
#define OPCODE_clv	41
#define OPCODE_clz	42
#define OPCODE_com	43
#define OPCODE_cp	44
#define OPCODE_cpc	45
#define OPCODE_cpi	46
#define OPCODE_cpse	47
#define OPCODE_dec	48
#define OPCODE_eicall	49
#define OPCODE_eijmp	50
#define OPCODE_elpm_1	51
#define OPCODE_elpm_2	52
#define OPCODE_elpm_3	53
#define OPCODE_eor	54
#define OPCODE_fmul	55
#define OPCODE_fmuls	56
#define OPCODE_fmulsu	57
#define OPCODE_icall	58
#define OPCODE_ijmp	59
#define OPCODE_in	60
#define OPCODE_inc	61
#define OPCODE_jmp	62
#define OPCODE_ld_1	63
#define OPCODE_ld_2	64
#define OPCODE_ld_3	65
#define OPCODE_ld_4	66
#define OPCODE_ld_5	67
#define OPCODE_ld_6	68
#define OPCODE_ldd_1	69
#define OPCODE_ld_7	70
#define OPCODE_ld_8	71
#define OPCODE_ld_9	72
#define OPCODE_ldd_2	73
#define OPCODE_ldi	74
#define OPCODE_lds	75
#define OPCODE_lpm_1	76
#define OPCODE_lpm_2	77
#define OPCODE_lpm_3	78
#define OPCODE_lsl	79
#define OPCODE_lsr	80
#define OPCODE_mov	81
#define OPCODE_movw	82
#define OPCODE_mul	83
#define OPCODE_muls	84
#define OPCODE_mulsu	85
#define OPCODE_neg	86
#define OPCODE_nop	87
#define OPCODE_or	88
#define OPCODE_ori	89
#define OPCODE_out	90
#define OPCODE_pop	91
#define OPCODE_push	92
#define OPCODE_rcall	93
#define OPCODE_ret	94
#define OPCODE_reti	95
#define OPCODE_rjmp	96
#define OPCODE_rol	97
#define OPCODE_ror	98
#define OPCODE_sbc	99
#define OPCODE_sbci	100
#define OPCODE_sbi	101
#define OPCODE_sbic	102
#define OPCODE_sbis	103
#define OPCODE_sbiw	104
#define OPCODE_sbr	105
#define OPCODE_sbrc	106
#define OPCODE_sbrs	107
#define OPCODE_sec	108
#define OPCODE_seh	109
#define OPCODE_sei	110
#define OPCODE_sen	111
#define OPCODE_ser	112
#define OPCODE_ses	113
#define OPCODE_set	114
#define OPCODE_sev	115
#define OPCODE_sez	116
#define OPCODE_sleep	117
#define OPCODE_spm	118
#define OPCODE_st_1	119
#define OPCODE_st_2	120
#define OPCODE_st_3	121
#define OPCODE_st_4	122
#define OPCODE_st_5	123
#define OPCODE_st_6	124
#define OPCODE_std_1	125
#define OPCODE_st_7	126
#define OPCODE_st_8	127
#define OPCODE_st_9	128
#define OPCODE_std_2	129
#define OPCODE_sts	130
#define OPCODE_sub	131
#define OPCODE_subi	132
#define OPCODE_swap	133
#define OPCODE_tst	134
#define OPCODE_wdr	135
