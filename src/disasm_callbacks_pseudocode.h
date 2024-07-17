
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

void PC_Operation_Simple(AVR_opcode mnemo);
void PC_Operation_Rd(AVR_opcode mnemo);
void PC_Operation_Rd16(AVR_opcode mnemo);
void PC_Operation_Rd_Rr(AVR_opcode mnemo);
void PC_Operation_Rd16_Rr16(AVR_opcode mnemo);
void PC_Operation_Rd16_K(AVR_opcode mnemo);
void PC_Operation_Rd_K(AVR_opcode mnemo);
void PC_Operation_RdW_K(AVR_opcode mnemo);
void PC_Operation_RdW_RrW(AVR_opcode mnemo);
void PC_Operation_s_k(AVR_opcode mnemo, int Position);
void PC_Operation_r_b(AVR_opcode mnemo);
void PC_Operation_Rd_b(AVR_opcode mnemo);
void PC_Operation_A_b(AVR_opcode mnemo);
void PC_Operation_s(AVR_opcode mnemo);
void PC_Operation_k(AVR_opcode mnemo, int Position, char *Pseudocode);
void adc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void add_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sub_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void mov_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brcc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brcs_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void breq_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brge_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brhc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brhs_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brid_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brie_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brlo_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brlt_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brmi_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brne_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brpl_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brsh_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brtc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brts_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brvc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void brvs_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void out_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void in_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void cli_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sei_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void ret_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void reti_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void andi_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void subi_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbci_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbr_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void ori_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void ldi_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void lds_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sts_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void call_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void rcall_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void ror_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void lsr_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void swap_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void eor_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void jmp_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void rjmp_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void cpi_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void asr_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void dec_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void inc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void cp_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void cpc_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void cpse_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void and_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void or_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void mul_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbi_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbic_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void sbis_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void cbi_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void ser_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void adiw_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void movw_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void lpm1_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
void stx2_Callback_PC(const char *Bitstream, int Position, AVR_opcode mnemo);
