
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

#define Rd (cx->dis_regs['d'])
#define Rr (cx->dis_regs['r'])
#define Rk (cx->dis_regs['k'])
#define RK (cx->dis_regs['K'])
#define Rs (cx->dis_regs['s'])
#define RA (cx->dis_regs['A'])
#define Rb (cx->dis_regs['b'])
#define Rq (cx->dis_regs['q'])

#define CALLBACK(name)  void name(const char *Bitstream, int Position, int MNemonic_Int)

struct JumpCall {
  int From;
  int To;
  int Type;
  unsigned int LabelNumber;
  unsigned char FunctionCall;
};

struct IO_Register {
  int Address;
  char Name[16];
  unsigned char Used;
};

enum {
  OPCODE_adc,
  OPCODE_add,
  OPCODE_adiw,
  OPCODE_and,
  OPCODE_andi,
  OPCODE_asr,
  OPCODE_bclr,
  OPCODE_bld,
  OPCODE_brbc,
  OPCODE_brbs,
  OPCODE_brcc,
  OPCODE_brcs,
  OPCODE_break,
  OPCODE_breq,
  OPCODE_brge,
  OPCODE_brhc,
  OPCODE_brhs,
  OPCODE_brid,
  OPCODE_brie,
  OPCODE_brlo,
  OPCODE_brlt,
  OPCODE_brmi,
  OPCODE_brne,
  OPCODE_brpl,
  OPCODE_brsh,
  OPCODE_brtc,
  OPCODE_brts,
  OPCODE_brvc,
  OPCODE_brvs,
  OPCODE_bset,
  OPCODE_bst,
  OPCODE_call,
  OPCODE_cbi,
  OPCODE_cbr,
  OPCODE_clc,
  OPCODE_clh,
  OPCODE_cli,
  OPCODE_cln,
  OPCODE_clr,
  OPCODE_cls,
  OPCODE_clt,
  OPCODE_clv,
  OPCODE_clz,
  OPCODE_com,
  OPCODE_cp,
  OPCODE_cpc,
  OPCODE_cpi,
  OPCODE_cpse,
  OPCODE_dec,
  OPCODE_eicall,
  OPCODE_eijmp,
  OPCODE_elpm_1,
  OPCODE_elpm_2,
  OPCODE_elpm_3,
  OPCODE_eor,
  OPCODE_fmul,
  OPCODE_fmuls,
  OPCODE_fmulsu,
  OPCODE_icall,
  OPCODE_ijmp,
  OPCODE_in,
  OPCODE_inc,
  OPCODE_jmp,
  OPCODE_ld_1,
  OPCODE_ld_2,
  OPCODE_ld_3,
  OPCODE_ld_4,
  OPCODE_ld_5,
  OPCODE_ld_6,
  OPCODE_ldd_1,
  OPCODE_ld_7,
  OPCODE_ld_8,
  OPCODE_ld_9,
  OPCODE_ldd_2,
  OPCODE_ldi,
  OPCODE_lds,
  OPCODE_lpm_1,
  OPCODE_lpm_2,
  OPCODE_lpm_3,
  OPCODE_lsl,
  OPCODE_lsr,
  OPCODE_mov,
  OPCODE_movw,
  OPCODE_mul,
  OPCODE_muls,
  OPCODE_mulsu,
  OPCODE_neg,
  OPCODE_nop,
  OPCODE_or,
  OPCODE_ori,
  OPCODE_out,
  OPCODE_pop,
  OPCODE_push,
  OPCODE_rcall,
  OPCODE_ret,
  OPCODE_reti,
  OPCODE_rjmp,
  OPCODE_rol,
  OPCODE_ror,
  OPCODE_sbc,
  OPCODE_sbci,
  OPCODE_sbi,
  OPCODE_sbic,
  OPCODE_sbis,
  OPCODE_sbiw,
  OPCODE_sbr,
  OPCODE_sbrc,
  OPCODE_sbrs,
  OPCODE_sec,
  OPCODE_seh,
  OPCODE_sei,
  OPCODE_sen,
  OPCODE_ser,
  OPCODE_ses,
  OPCODE_set,
  OPCODE_sev,
  OPCODE_sez,
  OPCODE_sleep,
  OPCODE_spm,
  OPCODE_st_1,
  OPCODE_st_2,
  OPCODE_st_3,
  OPCODE_st_4,
  OPCODE_st_5,
  OPCODE_st_6,
  OPCODE_std_1,
  OPCODE_st_7,
  OPCODE_st_8,
  OPCODE_st_9,
  OPCODE_std_2,
  OPCODE_sts,
  OPCODE_sub,
  OPCODE_subi,
  OPCODE_swap,
  OPCODE_tst,
  OPCODE_wdr,
};
