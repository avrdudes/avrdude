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
#include "IORegisters.h"
#include "Tagfile.h"
#include "Options.h"

#ifndef AVRDISAS_VERSION
#define AVRDISAS_VERSION			"dev"
#endif

void Options_Default(struct Options *Options) {
	Options->Show_Addresses = 0;
	Options->Show_Opcodes = 0;
	Options->Show_Comments = 1;
	Options->Show_Cycles = 0;
	Options->Show_PseudoCode = 0;
	Options->Filename[0] = 0;
	Options->MCU[0] = 0;
	Options->Tagfile[0] = 0;
	Options->CodeStyle = 1;		/* 0 = AVR Instruction Set,   1 = avr-gcc	*/
	Options->Process_Labels = 1;
	Options->Pass = 1;
	Options->FlashSize = 0;
}

int StringStart(const char *String1, const char *String2) {
	size_t Length1, Length2;
	int MinLen;
		
	Length1 = strlen(String1);
	Length2 = strlen(String2);
	if (Length1 < Length2) MinLen = Length1;
		else MinLen = Length2;

	return strncmp(String1, String2, MinLen);
}

void Show_Help(char *ProgramName) {
	fprintf(stderr, "%s [Options] [Filename]\n", ProgramName);
	fprintf(stderr, "'n' can either be 0 or 1 and means 'off' or 'on':\n");
	fprintf(stderr, "    -an      Do or don't show addresses\n");
	fprintf(stderr, "    -on      Do or don't show opcodes\n");
	fprintf(stderr, "    -cn      Do or don't show comments\n");
	fprintf(stderr, "    -qn      Do or don't show call cycles\n");
	fprintf(stderr, "    -sn      Do or don't use avr-gcc codestyle\n");
	fprintf(stderr, "    -pn      Do or don't include pseodocode\n");
	fprintf(stderr, "    -ln      Do or don't do jump/call-preprocessing\n");
	fprintf(stderr, "    -mMCU    Set the MCU for IO-register resolving\n");
	fprintf(stderr, "    -tFile   Set the tagfile to be used\n");
	fprintf(stderr, "    -fSize   Set the size of the flash area\n");
	fprintf(stderr, "    --help   Show this helppage\n");
	fprintf(stderr, "    -h       Show this helppage\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Example:\n");
	fprintf(stderr, "%s -a0 -o1 -c0 -m2313 -s1 -l1 -ttags.txt -f8192 Myfile.bin\n", ProgramName);
	fprintf(stderr, "\n");
	fprintf(stderr, "Version: " AVRDISAS_VERSION "\n");
}

int SetVal(char *Argument, char *Destination, char *Description, char Min, char Max) {
	int Value;
	Value = atoi(Argument);
	if ((Value < Min) || (Value > Max)) {
		fprintf(stderr, "Invalid argument for '%s': must be between %d and %d.\n", Description, Min, Max);
		return 0;
	}
	*Destination = Value;
	return 1;
}

int SetValInt(const char *Argument, int *Destination, char *Description, int Min, int Max) {
	int Value;
	Value = atoi(Argument);
	if ((Value < Min) || (Value > Max)) {
		fprintf(stderr, "Invalid argument for '%s': must be between %d and %d.\n", Description, Min, Max);
		return 0;
	}
	*Destination = Value;
	return 1;
}

int SetMCU(char *Argument, char *Destination) {
	strncpy(Destination, Argument, 7);
	if (ReadIORegisterFile() == 0) {
		fprintf(stderr, "Invalid argument for MCU! Must be one of the following:\n");
	/*	List_Supported_MCUs();	*/
		return 0;
	}
	return 1;
}

char Options_ParseCmdLine(struct Options *Options, int argc, char **argv) {
	int i;
	char *Argument;
	
	for (i = 1; i < argc; i++) {
		Argument = argv[i];
		
		if (StringStart(Argument, "-a") == 0) {
			if (SetVal(Argument + 2, &(Options->Show_Addresses), "show addresses", 0, 1) == 0) return 0; 
		} else if (StringStart(Argument, "-o") == 0) {
			if (SetVal(Argument + 2, &(Options->Show_Opcodes), "show opcodes", 0, 1) == 0) return 0; 
		} else if (StringStart(Argument, "-c") == 0) {
			if (SetVal(Argument + 2, &(Options->Show_Comments), "show comments", 0, 1) == 0) return 0; 
		} else if (StringStart(Argument, "-q") == 0) {
			if (SetVal(Argument + 2, &(Options->Show_Cycles), "show cycles", 0, 1) == 0) return 0; 
		} else if (StringStart(Argument, "-p") == 0) {
			if (SetVal(Argument + 2, &(Options->Show_PseudoCode), "show pseudocode", 0, 1) == 0) return 0; 
		} else if (StringStart(Argument, "-s") == 0) {
			if (SetVal(Argument + 2, &(Options->CodeStyle), "codestyle", 0, 2) == 0) return 0; 
		} else if (StringStart(Argument, "-l") == 0) {
			if (SetVal(Argument + 2, &(Options->Process_Labels), "process labels", 0, 1) == 0) return 0; 
		} else if (StringStart(Argument, "-m") == 0) {
			if (SetMCU(Argument + 2, Options->MCU) == 0) return 0; 
		} else if (StringStart(Argument, "-t") == 0) {
			strncpy(Options->Tagfile, Argument + 2, 256);
		} else if (StringStart(Argument, "-f") == 0) {
			if (SetValInt(Argument + 2, &(Options->FlashSize), "flash size", 32, 1024 * 1024) == 0) return 0; 
		} else if (StringStart(Argument, "--help") == 0) {
			Show_Help(argv[0]); return 0;
		} else if (StringStart(Argument, "-h") == 0) {
			Show_Help(argv[0]); return 0;
		} else {
			if (Options->Filename[0] != 0) {
				fprintf(stderr, "Filename '%s' was already supplied!\n", Options->Filename);
				return 0;
			}
			strncpy(Options->Filename, Argument, 255);
		}
	}

	if (Options->Filename[0] == 0) {
		fprintf(stderr, "Error: No filename supplied.\n");
		return 0;
	}

	if (Options->Tagfile[0] != 0) {
		if (!Read_Tagfile(Options->Tagfile)) return 0;
	}
	
	return 1;
}

