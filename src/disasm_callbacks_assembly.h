
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

void Operation_Simple(int MNemonic_Int);
void Operation_Rd(int MNemonic_Int);
void Operation_Rd16(int MNemonic_Int);
void Operation_Rd_Rr(int MNemonic_Int);
void Operation_Rd16_Rr16(int MNemonic_Int);
void Operation_Rd16_K(int MNemonic_Int);
void Operation_Rd_K(int MNemonic_Int);
void Operation_RdW_K(int MNemonic_Int);
void Operation_RdW_RrW(int MNemonic_Int);
void Operation_s_k(int MNemonic_Int, int Position);
void Operation_r_b(int MNemonic_Int);
void Operation_Rd_b(int MNemonic_Int);
void Operation_A_b(int MNemonic_Int);
void Operation_s(int MNemonic_Int);
void Operation_k(int MNemonic_Int, int Position, const char *Pseudocode);
void adc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void add_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void adiw_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void and_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void andi_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void asr_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void bclr_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void bld_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brbc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brbs_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brcc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brcs_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void break_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void breq_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brge_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brhc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brhs_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brid_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brie_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brlo_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brlt_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brmi_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brne_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brpl_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brsh_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brtc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brts_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brvc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void brvs_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void bset_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void bst_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void call_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cbi_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void clc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void clh_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cli_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cln_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cls_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void clt_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void clv_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void clz_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void com_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cp_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cpc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cpi_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void cpse_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void dec_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void eicall_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void eijmp_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void elpm1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void elpm2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void elpm3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void eor_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void fmul_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void fmuls_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void fmulsu_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void icall_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ijmp_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void in_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void inc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void jmp_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ld1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ld2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ld3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldy1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldy2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldy3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldy4_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldz1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldz2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldz3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldz4_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ldi_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void lds_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void lpm1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void lpm2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void lpm3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void lsr_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void mov_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void movw_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void mul_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void muls_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void mulsu_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void neg_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void nop_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void or_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ori_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void out_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void pop_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void push_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void rcall_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ret_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void reti_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void rjmp_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ror_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbci_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbi_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbic_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbis_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbiw_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbr_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbrc_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sbrs_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sec_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void seh_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sei_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sen_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ser_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void ses_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void set_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sev_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sez_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sleep_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void spm_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void st1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void st2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void st3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sty1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sty2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sty3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sty4_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void stz1_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void stz2_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void stz3_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void stz4_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sts_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void sub_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void subi_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void swap_Callback(const char *Bitstream, int Position, int MNemonic_Int);
void wdr_Callback(const char *Bitstream, int Position, int MNemonic_Int);
