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

void Activate_PC_Callbacks(char *New_Code_Line, char *New_Comment_Line, char *New_After_Code_Line, int *New_Registers, struct Options *New_Options);
void PC_Operation_Simple(int MNemonic_Int);
void PC_Operation_Rd(int MNemonic_Int);
void PC_Operation_Rd16(int MNemonic_Int);
void PC_Operation_Rd_Rr(int MNemonic_Int);
void PC_Operation_Rd16_Rr16(int MNemonic_Int);
void PC_Operation_Rd16_K(int MNemonic_Int);
void PC_Operation_Rd_K(int MNemonic_Int);
void PC_Operation_RdW_K(int MNemonic_Int);
void PC_Operation_RdW_RrW(int MNemonic_Int);
void PC_Operation_s_k(int MNemonic_Int, int Position);
void PC_Operation_r_b(int MNemonic_Int);
void PC_Operation_Rd_b(int MNemonic_Int);
void PC_Operation_A_b(int MNemonic_Int);
void PC_Operation_s(int MNemonic_Int);
void PC_Operation_k(int MNemonic_Int, int Position, char *Pseudocode);
void adc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void add_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sub_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sbc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void mov_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brcc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brcs_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void breq_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brge_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brhc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brhs_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brid_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brie_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brlo_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brlt_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brmi_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brne_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brpl_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brsh_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brtc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brts_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brvc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void brvs_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void out_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void in_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void cli_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sei_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void ret_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void reti_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void andi_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void subi_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sbci_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sbr_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void ori_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void ldi_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void lds_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sts_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void call_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void rcall_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void ror_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void lsr_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void swap_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void eor_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void jmp_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void rjmp_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void cpi_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void asr_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void dec_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void inc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void cp_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void cpc_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void cpse_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void and_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void or_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void mul_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sbi_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sbic_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void sbis_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void cbi_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void ser_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void adiw_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void movw_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void lpm1_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
void st2_Callback_PC(char *Bitstream, int Position, int MNemonic_Int);
