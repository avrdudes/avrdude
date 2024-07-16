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
#include <stdlib.h>
#include <string.h>

#include "Globals.h"
#include "MNemonics.h"
#include "Tagfile.h"
#include "JumpCall.h"

extern struct Options Options;
static int JumpCall_Count;
static struct JumpCall *JumpCalls;

void Display_JumpCalls() {
	int i;
	printf("%d jumps/calls found:\n", JumpCall_Count);
	for (i = 0; i < JumpCall_Count; i++) {
		printf("%3d: 0x%-4x -> 0x%-4x     %s (%d)\n", i, (unsigned int)JumpCalls[i].From, (unsigned int)JumpCalls[i].To, MNemonic[JumpCalls[i].Type], JumpCalls[i].FunctionCall);
	}
}

int FixTargetAddress(int Address) {
	if (Options.FlashSize) {
		Address %= Options.FlashSize;
		if (Address < 0) {
			Address += Options.FlashSize;
		}
	}
	return Address;
}

void Register_JumpCall(int From, int To, int Type, unsigned char FunctionCall) {
	if ((Options.Process_Labels == 1) && (Options.Pass == 1)) {
		JumpCall_Count++;
		JumpCalls = realloc(JumpCalls, sizeof(struct JumpCall) * (JumpCall_Count));
		JumpCalls[JumpCall_Count - 1].From = From;
		JumpCalls[JumpCall_Count - 1].To = To;
		JumpCalls[JumpCall_Count - 1].Type = Type;
		JumpCalls[JumpCall_Count - 1].LabelNumber = 0;
		JumpCalls[JumpCall_Count - 1].FunctionCall = FunctionCall;
	}
}

int JC_Comparison(const void *Element1, const void *Element2) {
	struct JumpCall *JC1, *JC2;
	JC1 = (struct JumpCall*)Element1;
	JC2 = (struct JumpCall*)Element2;
	if ((JC1->To) > (JC2->To)) return 1;
		else if ((JC1->To) == (JC2->To)) return 0;
	return -1;
}

void Sort_JumpCalls() {
	qsort(JumpCalls, JumpCall_Count, sizeof(struct JumpCall), JC_Comparison);
}

void Correct_Label_Types(void) {
	int i, j;
	int LastIdx = 0;
	int LastDest = JumpCalls[0].To;
	char CurType = JumpCalls[0].FunctionCall;
	
	for (i = 1; i < JumpCall_Count; i++) {
		if (JumpCalls[i].To != LastDest) {
			for (j = LastIdx; j < i; j++) JumpCalls[j].FunctionCall = CurType;
			LastIdx = i;
			LastDest = JumpCalls[i].To;
			CurType = 0;
		}
		CurType = (CurType || JumpCalls[i].FunctionCall);
	}
	for (j = LastIdx; j < JumpCall_Count; j++) JumpCalls[j].FunctionCall = CurType;
}

void Enumerate_Labels(void) {
	int i;
	int CurrentLabelNumber = 0;
	int CurrentFunctionNumber = 0;
	int Destination;

	if (JumpCall_Count < 2) return;
	
	Sort_JumpCalls();
	Correct_Label_Types();	
	
	Destination = JumpCalls[0].To;
	if (JumpCalls[0].FunctionCall) CurrentFunctionNumber++;
		else CurrentLabelNumber++;
	for (i = 0; i < JumpCall_Count; i++) {
		if (Destination != JumpCalls[i].To) {
			if (JumpCalls[i].FunctionCall) CurrentFunctionNumber++;
				else CurrentLabelNumber++;
			Destination = JumpCalls[i].To;
		}
		if (JumpCalls[i].FunctionCall) JumpCalls[i].LabelNumber = CurrentFunctionNumber;
			else JumpCalls[i].LabelNumber = CurrentLabelNumber;
	}
}

char *Get_Label_Name(int Destination, char **LabelComment) {
	int i;
	static char Buffer[256];
	int TagIndex;
	char *TagLabel;

	TagIndex = Tagfile_FindLabelAddress(Destination);
	if (TagIndex != -1) {
		TagLabel = Tagfile_GetLabel(TagIndex);
		snprintf(Buffer, sizeof(Buffer), "%s", TagLabel);
		if (LabelComment != NULL) *LabelComment = Tagfile_GetLabelComment(TagIndex);
		return Buffer;
	}
	
	for (i = 0; i < JumpCall_Count; i++) {
		if ((JumpCalls[i].To) == Destination) {
			if (JumpCalls[i].FunctionCall) {
				snprintf(Buffer, sizeof(Buffer), "Function%d", JumpCalls[i].LabelNumber);
			} else {
				snprintf(Buffer, sizeof(Buffer), "Label%d", JumpCalls[i].LabelNumber);
			}
			return Buffer;
		}
	}
	
	snprintf(Buffer, sizeof(Buffer), "UNKNOWN");
	return Buffer;
}

/* Show all references which refer to "Position" as destination */
void Print_JumpCalls(int Position) {
	int i;
	int Match = 0;
	
	for (i = 0; i < JumpCall_Count; i++) {
		if ((JumpCalls[i].To) == Position) {
			if (Match == 0) {
				printf("\n");
				Match = 1;
			}
			printf("; Referenced from offset 0x%02x by %s\n", JumpCalls[i].From, MNemonic[JumpCalls[i].Type]);
		}
	}
	if (Match == 1) {
		char *LabelName;
		char *LabelComment = NULL;
		LabelName = Get_Label_Name(Position, &LabelComment);
		if (LabelComment == NULL) {
			 printf("%s:\n", LabelName);
		} else {
			 printf("%s:     ; %s\n", LabelName, LabelComment);
		}
	}
}

