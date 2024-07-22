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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "disasm_private.h"


void disasm_zap_JumpCalls() {
  if(cx->dis_JumpCalls) {
    mmt_free(cx->dis_JumpCalls);
    cx->dis_JumpCalls = NULL;
  }
  cx->dis_JumpCallN = 0;
}

void Register_JumpCall(int From, int To, int mnemo, unsigned char FunctionCall) {
  if(cx->dis_opts.Process_Labels) {
    Disasm_JumpCall *jc = cx->dis_JumpCalls;
    int N = cx->dis_JumpCallN;

    // Already entered this JC?
    for(int i = 0; i < N; i++)
      if(jc[i].From == From && jc[N].To == To && jc[N].mnemo == mnemo)
        return;

    jc = mmt_realloc(jc, sizeof(Disasm_JumpCall) * (N+1));
    jc[N].From = From;
    jc[N].To = To;
    jc[N].mnemo = mnemo;
    jc[N].LabelNumber = 0;
    jc[N].FunctionCall = FunctionCall;

    cx->dis_JumpCalls = jc;
    cx->dis_JumpCallN++;
  }
}

static int JC_Comparison(const void *Element1, const void *Element2) {
  Disasm_JumpCall *JC1, *JC2;

  JC1 = (Disasm_JumpCall *) Element1;
  JC2 = (Disasm_JumpCall *) Element2;
  if((JC1->To) > (JC2->To))
    return 1;
  else if((JC1->To) == (JC2->To))
    return 0;
  return -1;
}

static void Correct_Label_IsFunct(void) {
  int i, j;
  int LastIdx = 0;
  int LastDest = cx->dis_JumpCalls[0].To;
  char CurIsFunct = cx->dis_JumpCalls[0].FunctionCall;

  for(i = 1; i < cx->dis_JumpCallN; i++) {
    if(cx->dis_JumpCalls[i].To != LastDest) {
      for(j = LastIdx; j < i; j++)
        cx->dis_JumpCalls[j].FunctionCall = CurIsFunct;
      LastIdx = i;
      LastDest = cx->dis_JumpCalls[i].To;
      CurIsFunct = 0;
    }
    CurIsFunct = CurIsFunct || cx->dis_JumpCalls[i].FunctionCall;
  }
  for(j = LastIdx; j < cx->dis_JumpCallN; j++)
    cx->dis_JumpCalls[j].FunctionCall = CurIsFunct;
}

void Enumerate_Labels(void) {
  int i;
  int CurrentLabelNumber = 0;
  int CurrentFunctionNumber = 0;
  int Destination;

  if(cx->dis_JumpCallN < 2)
    return;

  qsort(cx->dis_JumpCalls, cx->dis_JumpCallN, sizeof(Disasm_JumpCall), JC_Comparison);
  Correct_Label_IsFunct();

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
        term_out("\n");
        Match = 1;
      }
      term_out("; Referenced from 0x%0*x by %s\n", cx->dis_addrwidth, 
        cx->dis_JumpCalls[i].From, avr_opcodes[cx->dis_JumpCalls[i].mnemo].opcode);
    }
  }
  if(Match == 1) {
    char *LabelComment = NULL;
    const char *LabelName = Get_Label_Name(Position, &LabelComment);
    if(LabelComment == NULL) {
      term_out("%s:\n", LabelName);
    } else {
      term_out("%-23s ; %s\n", str_ccprintf("%s:", LabelName), LabelComment);
    }
  }
}
