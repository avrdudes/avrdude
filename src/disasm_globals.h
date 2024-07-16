
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

#define CALLBACK(name)  void name(const char *Bitstream, int Position, AVR_opcode mnemo)

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

