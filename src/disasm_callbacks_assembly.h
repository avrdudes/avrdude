
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

void adc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void add_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void adiw_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void and_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void andi_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void asr_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void bclr_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void bld_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brbc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brbs_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brcc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brcs_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void break_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void breq_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brge_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brhc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brhs_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brid_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brie_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brlo_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brlt_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brmi_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brne_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brpl_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brsh_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brtc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brts_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brvc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void brvs_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void bset_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void bst_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void call_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cbi_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void clc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void clh_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cli_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cln_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cls_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void clt_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void clv_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void clz_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void com_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cp_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cpc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cpi_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void cpse_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void dec_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void eicall_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void eijmp_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void elpm1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void elpm2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void elpm3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void eor_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void fmul_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void fmuls_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void fmulsu_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void icall_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ijmp_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void in_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void inc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void jmp_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldx1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldx2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldx3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldy1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldy2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldy3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldy4_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldz1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldz2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldz3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldz4_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldi_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void lds_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void lpm1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void lpm2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void lpm3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void lsr_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void mov_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void movw_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void mul_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void muls_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void mulsu_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void neg_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void nop_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void or_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ori_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void out_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void pop_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void push_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void rcall_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ret_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void reti_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void rjmp_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ror_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbci_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbi_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbic_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbis_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbiw_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbrc_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbrs_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sec_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void seh_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sei_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sen_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ser_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void ses_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void set_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sev_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sez_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sleep_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void spm_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void stx1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void stx2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void stx3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sty1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sty2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sty3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sty4_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void stz1_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void stz2_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void stz3_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void stz4_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sts_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void sub_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void subi_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void swap_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
void wdr_Callback(const char *Bitstream, int Position, AVR_opcode mnemo);
