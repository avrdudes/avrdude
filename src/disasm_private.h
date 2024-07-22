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

#ifndef disasm_private_h
#define disasm_private_h

#define TYPE_BYTE       1
#define TYPE_WORD       2
#define TYPE_ASTRING    3
#define TYPE_STRING     4

void disasm_init_regfile(const AVRPART *p);
int disasm_wrap(int addr);
int Tagfile_FindLabelAddress(int Address);
char *Tagfile_GetLabel(int TagIndex);
char *Tagfile_GetLabelComment(int TagIndex);
int Tagfile_FindPGMAddress(int Address);
const char *Tagfile_Resolve_Mem_Address(int Address);
int Tagfile_Process_Data(const char *Bitstream, int Position, int offset);
const char *Resolve_IO_Register(int Number);
void Emit_Used_IO_Registers();

void Register_JumpCall(int From, int To, int mnemo, unsigned char FunctionCall);
void Enumerate_Labels();
const char *Get_Label_Name(int Destination, char **LabelComment);
void Print_JumpCalls(int Position);

#endif
