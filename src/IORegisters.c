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
#include <string.h>
#include <stdlib.h>

#include "Globals.h"

extern struct Options Options;

static struct IO_Register *KnownIORegisters = NULL;
static unsigned int KnownIORegistersCount = 0;

unsigned int ReadIORegisterFile() {
	FILE *f;
	f = fopen("/etc/avrdisas.conf", "r");
	if (!f) {
		f = fopen("avrdisas.conf", "r");
	}
	if (!f) {
		fprintf(stderr, "Could not read any configuration file.\n");
		return 0;
	}


	{
		char Buffer[256];
		int CurrentMCU = 0;
		while (fgets(Buffer, sizeof(Buffer), f)) {
			char *Token;
			struct IO_Register TempRegister;
			
			if (!(Token = strtok(Buffer, "\t\n"))) continue;
			if (!strcmp(Token, "Register")) {
				if (!CurrentMCU) continue;
				if (!(Token = strtok(NULL, "\t\n"))) continue;
				if (strlen(Token) == 0) continue;
				if (Token[1] == 'x') {
					TempRegister.Address = strtol(Token + 2, NULL, 16);
				} else {
					TempRegister.Address = atoi(Token);
				}
				if (!(Token = strtok(NULL, "\t\n"))) continue;
				strncpy(TempRegister.Name, Token, 16);
				TempRegister.Used = 0;

				KnownIORegisters = (struct IO_Register*)realloc(KnownIORegisters, sizeof(struct IO_Register) * (KnownIORegistersCount + 1));
				KnownIORegisters[KnownIORegistersCount] = TempRegister;
				KnownIORegistersCount++;
			} else if (!strcmp(Token, "Unit")) {
				if (!(Token = strtok(NULL, "\t\n"))) continue;
				CurrentMCU = (!strcmp(Token, Options.MCU)) || (!strcmp(Token, "Global"));
			}
		}
	}

	fclose(f);
	return KnownIORegistersCount;
}

const char* Resolve_IO_Register(int Number) {
	int Resolved;
	unsigned int i;

	if (!strcmp(Options.MCU, "None")) return NULL;
	
	Resolved = -1;
	for (i = 0; i < KnownIORegistersCount; i++) {
		if (KnownIORegisters[i].Address == Number) {
			Resolved = i;
			break;
		}
	}

	if (Resolved != -1) {
		KnownIORegisters[Resolved].Used = 1;
		return KnownIORegisters[Resolved].Name;
	} else {
		return NULL;
	}
}

void Emit_Used_IO_Registers() {
	unsigned int i;
	if (Options.Show_PseudoCode) return;
	for (i = 0; i < KnownIORegistersCount; i++) {
		if (KnownIORegisters[i].Used) printf(".equ %s, 0x%x\n", KnownIORegisters[i].Name, KnownIORegisters[i].Address);
	}
}

