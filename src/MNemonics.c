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

const char* MNemonic[] = {
	"adc",
	"add",
	"adiw",
	"and",
	"andi",
	"asr",
	"bclr",
	"bld",
	"brbc",
	"brbs",
	"brcc",
	"brcs",
	"break",
	"breq",
	"brge",
	"brhc",
	"brhs",
	"brid",
	"brie",
	"brlo",
	"brlt",
	"brmi",
	"brne",
	"brpl",
	"brsh",
	"brtc",
	"brts",
	"brvc",
	"brvs",
	"bset",
	"bst",
	"call",
	"cbi",
	"cbr",
	"clc",
	"clh",
	"cli",
	"cln",
	"clr",
	"cls",
	"clt",
	"clv",
	"clz",
	"com",
	"cp",
	"cpc",
	"cpi",
	"cpse",
	"dec",
	"eicall",
	"eijmp",
	"elpm",
	"elpm",
	"elpm",
	"eor",
	"fmul",
	"fmuls",
	"fmulsu",
	"icall",
	"ijmp",
	"in",
	"inc",
	"jmp",
	"ld",
	"ld",
	"ld",
	"ld",
	"ld",
	"ld",
	"ldd",
	"ld",
	"ld",
	"ld",
	"ldd",
	"ldi",
	"lds",
	"lpm",
	"lpm",
	"lpm",
	"lsl",
	"lsr",
	"mov",
	"movw",
	"mul",
	"muls",
	"mulsu",
	"neg",
	"nop",
	"or",
	"ori",
	"out",
	"pop",
	"push",
	"rcall",
	"ret",
	"reti",
	"rjmp",
	"rol",
	"ror",
	"sbc",
	"sbci",
	"sbi",
	"sbic",
	"sbis",
	"sbiw",
	"sbr",
	"sbrc",
	"sbrs",
	"sec",
	"seh",
	"sei",
	"sen",
	"ser",
	"ses",
	"set",
	"sev",
	"sez",
	"sleep",
	"spm",
	"st",
	"st",
	"st",
	"st",
	"st",
	"st",
	"std",
	"st",
	"st",
	"st",
	"std",
	"sts",
	"sub",
	"subi",
	"swap",
	"tst",
	"wdr"
};

const char *Cycles[] = {
	"1",		/* adc */
	"1",		/* add */
	"2",		/* adiw */
	"1",		/* and */
	"1",		/* andi */
	"1",		/* asr */
	"1",		/* bclr */
	"1",		/* bld */
	"1/2",		/* brbc */
	"1/2",		/* brbs */
	"1/2",		/* brcc */
	"1/2",		/* brcs */
	"N/A",		/* break */
	"1/2",		/* breq */
	"1/2",		/* brge */
	"1/2",		/* brhc */
	"1/2",		/* brhs */
	"1/2",		/* brid */
	"1/2",		/* brie */
	"1/2",		/* brlo */
	"1/2",		/* brlt */
	"1/2",		/* brmi */
	"1/2",		/* brne */
	"1/2",		/* brpl */
	"1/2",		/* brsh */
	"1/2",		/* brtc */
	"1/2",		/* brts */
	"1/2",		/* brvc */
	"1/2",		/* brvs */
	"1",		/* bset */
	"1",		/* bst */
	"4",		/* call */
	"2",		/* cbi */
	"1",		/* cbr */
	"1",		/* clc */
	"",			/* clh */
	"1",		/* cli */
	"1",		/* cln */
	"1",		/* clr */
	"1",		/* cls */
	"1",		/* clt */
	"1",		/* clv */
	"1",		/* clz */
	"1",		/* com */
	"1",		/* cp */
	"1",		/* cpc */
	"1",		/* cpi */
	"1-3",		/* cpse */
	"1",		/* dec */
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	"1",		/* eor */
	"2",		/* fmul */
	"2",		/* fmuls */
	"2",		/* fmulsu */
	"3",		/* icall */
	"2",		/* ijmp */
	"1",		/* in */
	"1",		/* inc */
	"3",		/* jmp */
	"2",		/* ld */
	"2",		/* ld */
	"2",		/* ld */
	"2",		/* ld */
	"2",		/* ld */
	"2",		/* ld */
	"2",		/* ldd */
	"2",		/* ld */
	"2",		/* ld */
	"2",		/* ld */
	"2",		/* ldd */
	"1",		/* ldi */
	"2",		/* lds */
	"3",		/* lpm */
	"3",		/* lpm */
	"3",		/* lpm */
	"1",		/* lsl */
	"1",		/* lsr */
	"1",		/* mov */
	"1",		/* movw */
	"2",		/* mul */
	"2",		/* muls */
	"2",		/* mulsu */
	"1",		/* neg */
	"1",		/* nop */
	"1",		/* or */
	"1",		/* ori */
	"1",		/* out */
	"2",		/* pop */
	"2",		/* push */
	"3",		/* rcall */
	"4",		/* ret */
	"4",		/* reti */
	"2",		/* rjmp */
	"1",		/* rol */
	"1",		/* ror */
	"1",		/* sbc */
	"1",		/* sbci */
	"2",		/* sbi */
	"1-3",		/* sbic */
	"1-3",		/* sbis */
	"2",		/* sbiw */
	"1",		/* sbr */
	"1-3",		/* sbrc */
	"1-3",		/* sbrs */
	"1",		/* sec */
	"1",		/* seh */
	"1",		/* sei */
	"1",		/* sen */
	"1",		/* ser */
	"1",		/* ses */
	"1",		/* set */
	"1",		/* sev */
	"1",		/* sez */
	"1",		/* sleep */
	"-",		/* spm */
	"2",		/* st */
	"2",		/* st */
	"2",		/* st */
	"2",		/* st */
	"2",		/* st */
	"2",		/* st */
	"2",		/* std */
	"2",		/* st */
	"2",		/* st */
	"2",		/* st */
	"2",		/* std */
	"2",		/* sts */
	"1",		/* sub */
	"1",		/* subi */
	"1",		/* swap */
	"1",		/* tst */
	"1"			/* wdr */
};

/*
adc
add
adiw
and
andi
asr
bclr
bld
brbc
brbs
brcc
brcs
break
breq
brge
brhc
brhs
brid
brie
brlo
brlt
brmi
brne
brpl
brsh
brtc
brts
brvc
brvs
bset
bst
call
cbi
cbr
clc
clh
cli
cln
clr
cls
clt
clv
clz
com
cp
cpc
cpi
cpse
dec
eicall
eijmp
elpm
elpm
elpm
eor
fmul
fmuls
fmulsu
icall
ijmp
in
inc
jmp
ld
ld
ld
ld
ld
ld
ldd
ld
ld
ld
ldd
ldi
lds
lpm
lpm
lpm
lsl
lsr
mov
movw
mul
muls
mulsu
neg
nop
or
ori
out
pop
push
rcall
ret
reti
rjmp
rol
ror
sbc
sbci
sbi
sbic
sbis
sbiw
sbr
sbrc
sbrs
sec
seh
sei
sen
ser
ses
set
sev
sez
sleep
spm
st
st
st
st
st
st
std
st
st
st
std
sts
sub
subi
swap
tst
wdr
*/
