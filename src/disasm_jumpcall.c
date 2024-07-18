
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

#include "avrdude.h"
#include "libavrdude.h"
#include "disasm_private.h"

void Display_JumpCalls() {
  int i;

  printf("%d jumps/calls found:\n", cx->dis_JumpCallN);
  for(i = 0; i < cx->dis_JumpCallN; i++) {
    printf("%3d: 0x%-4x -> 0x%-4x     %s (%d)\n", i, (unsigned int) cx->dis_JumpCalls[i].From, (unsigned int) cx->dis_JumpCalls[i].To,
      avr_opcodes[cx->dis_JumpCalls[i].Type].opcode, cx->dis_JumpCalls[i].FunctionCall);
  }
}

int FixTargetAddress(int Address) {
  int flashsz = cx->dis_opts.FlashSize;

  // Flash size is a power of two: flash wraps round
  if(flashsz > 0 && !(flashsz & (flashsz - 1))) {
    Address %= flashsz;
    if(Address < 0)
      Address += flashsz;
  }
  return Address;
}

void Register_JumpCall(int From, int To, int Type, unsigned char FunctionCall) {
  if((cx->dis_opts.Process_Labels == 1) && (cx->dis_opts.Pass == 1)) {
    cx->dis_JumpCallN++;
    cx->dis_JumpCalls = mmt_realloc(cx->dis_JumpCalls, sizeof(Disasm_JumpCall) * (cx->dis_JumpCallN));
    cx->dis_JumpCalls[cx->dis_JumpCallN - 1].From = From;
    cx->dis_JumpCalls[cx->dis_JumpCallN - 1].To = To;
    cx->dis_JumpCalls[cx->dis_JumpCallN - 1].Type = Type;
    cx->dis_JumpCalls[cx->dis_JumpCallN - 1].LabelNumber = 0;
    cx->dis_JumpCalls[cx->dis_JumpCallN - 1].FunctionCall = FunctionCall;
  }
}

int JC_Comparison(const void *Element1, const void *Element2) {
  Disasm_JumpCall *JC1, *JC2;

  JC1 = (Disasm_JumpCall *) Element1;
  JC2 = (Disasm_JumpCall *) Element2;
  if((JC1->To) > (JC2->To))
    return 1;
  else if((JC1->To) == (JC2->To))
    return 0;
  return -1;
}

void Sort_JumpCalls() {
  qsort(cx->dis_JumpCalls, cx->dis_JumpCallN, sizeof(Disasm_JumpCall), JC_Comparison);
}

void Correct_Label_Types(void) {
  int i, j;
  int LastIdx = 0;
  int LastDest = cx->dis_JumpCalls[0].To;
  char CurType = cx->dis_JumpCalls[0].FunctionCall;

  for(i = 1; i < cx->dis_JumpCallN; i++) {
    if(cx->dis_JumpCalls[i].To != LastDest) {
      for(j = LastIdx; j < i; j++)
        cx->dis_JumpCalls[j].FunctionCall = CurType;
      LastIdx = i;
      LastDest = cx->dis_JumpCalls[i].To;
      CurType = 0;
    }
    CurType = (CurType || cx->dis_JumpCalls[i].FunctionCall);
  }
  for(j = LastIdx; j < cx->dis_JumpCallN; j++)
    cx->dis_JumpCalls[j].FunctionCall = CurType;
}

void Enumerate_Labels(void) {
  int i;
  int CurrentLabelNumber = 0;
  int CurrentFunctionNumber = 0;
  int Destination;

  if(cx->dis_JumpCallN < 2)
    return;

  Sort_JumpCalls();
  Correct_Label_Types();

  Destination = cx->dis_JumpCalls[0].To;
  if(cx->dis_JumpCalls[0].FunctionCall)
    CurrentFunctionNumber++;
  else
    CurrentLabelNumber++;
  for(i = 0; i < cx->dis_JumpCallN; i++) {
    if(Destination != cx->dis_JumpCalls[i].To) {
      if(cx->dis_JumpCalls[i].FunctionCall)
        CurrentFunctionNumber++;
      else
        CurrentLabelNumber++;
      Destination = cx->dis_JumpCalls[i].To;
    }
    if(cx->dis_JumpCalls[i].FunctionCall)
      cx->dis_JumpCalls[i].LabelNumber = CurrentFunctionNumber;
    else
      cx->dis_JumpCalls[i].LabelNumber = CurrentLabelNumber;
  }
}

const char *Get_Label_Name(int Destination, char **LabelComment) {
  int TagIndex;

  TagIndex = Tagfile_FindLabelAddress(Destination);
  if(TagIndex != -1) {
    if(LabelComment)
      *LabelComment = Tagfile_GetLabelComment(TagIndex);
    return str_ccprintf("%s", Tagfile_GetLabel(TagIndex));
  }

  for(int i = 0; i < cx->dis_JumpCallN; i++)
    if(cx->dis_JumpCalls[i].To == Destination)
      return str_ccprintf("%s%d", cx->dis_JumpCalls[i].FunctionCall? "Function": "Label", cx->dis_JumpCalls[i].LabelNumber);

  return "UNKNOWN";
}

// Show all references which refer to "Position" as destination
void Print_JumpCalls(int Position) {
  int i;
  int Match = 0;

  for(i = 0; i < cx->dis_JumpCallN; i++) {
    if((cx->dis_JumpCalls[i].To) == Position) {
      if(Match == 0) {
        printf("\n");
        Match = 1;
      }
      printf("; Referenced from offset 0x%02x by %s\n", cx->dis_JumpCalls[i].From, avr_opcodes[cx->dis_JumpCalls[i].Type].opcode);
    }
  }
  if(Match == 1) {
    char *LabelComment = NULL;
    const char *LabelName = Get_Label_Name(Position, &LabelComment);
    if(LabelComment == NULL) {
      printf("%s:\n", LabelName);
    } else {
      printf("%s:     ; %s\n", LabelName, LabelComment);
    }
  }
}
