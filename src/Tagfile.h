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

#define TYPE_BYTE		1
#define TYPE_WORD		2
#define TYPE_ASTRING	3
#define TYPE_STRING		4

int Read_Tagfile(const char *Filename);
int Tagfile_FindLabelAddress(int Address);
char *Tagfile_GetLabel(int TagIndex);
char *Tagfile_GetLabelComment(int TagIndex);
int Tagfile_FindPGMAddress(int Address);
const char* Tagfile_Resolve_Mem_Address(int Address);
int Tagfile_Process_Data(char *Bitstream, int Position);

