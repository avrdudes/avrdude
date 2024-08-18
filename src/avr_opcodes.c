/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2024 Stefan Rueger <stefan.rueger@urclocks.com>
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

#include <stdio.h>

#include "libavrdude.h"

/*
 * AVR opcode table
 *
 * - Order of enums MNEMO_...  in libavedude.h must align with table
 *
 * - Order makes the first match of a 16-bit opcode a "good" one
 *     + Undocumented opcodes come last
 *     + Specific reduced-core sts/lds opcodes are penultimate
 *     + More specific opcodes before less specific ones (clr before eor)
 *     + Opcodes labelled alias come behind those not labelled so
 *
 * - The operand field has a very specific syntax as follows
 *
 *     A  5-bit I/O address (sbic, sbis, sbi, cbi)
 *        6-bit I/O address (in, out)
 *
 *     a  7-bit encoded address 0x40..0xbf for reduced-core (lds, sts)
 *
 *     b  bit number 0..7 (sbrc, sbrs, sbic, sbis, sbi, cbi, bst, bld, u/bld,
 *           u/bst, u/sbrc, u/sbrs)
 *
 *     k  signed 7-bit for 2*k byte PC additional offset in [.-128, .+126]
 *           (brcs, brlo, breq, brmi, brvs, brlt, brhs, brts, brie, brcc, brsh,
 *           brne, brpl, brvc, brge, brhc, brtc, brid, brbs, brbc)
 *        signed 12-bit for 2*k bytes PC additional offset in [.-4096, .+4094]
 *           (rjmp, rcall)
 *        16-bit absolute byte address (lds, sts)
 *        22-bit absolute word address for the PC (jmp, call)
 *
 *     K  4-bit encryption round index 0..15 (des)
 *        6-bit constant 0..63 (adiw, sbiw)
 *        8-bit constant 0..255 (subi, sbci, andi, ori, sbr, cbr, cpi, ldi)
 *
 *     q  6-bit displacement 0..63 (ldd, ldd, std, std)
 *
 *     Rd 2-bit destination register in r24, r26, r28, r30 (adiw, sbiw)
 *        3-bit dest register in r16, ..., r23 (mulsu, fmul, fmuls, fmulsu)
 *        4-bit destination register in r16, ..., r31 (subi, sbci, andi, ori,
 *              sbr, cbr, ser, muls, cpi, ldi, lds)
 *        4-bit destination register in r0, r2, ..., r30 (movw)
 *        5-bit destination register in r0, r1, ..., r31 (add, adc, sub, sbc,
 *              and, or, eor, com, neg, inc, dec, clr, mul, cpse, cp, cpc, mov,
 *              lds, ld, ldd, lpm, elpm, in, pop, xch, las, lac, lat, lsl, lsr,
 *              rol, ror, asr, swap, bld, u/bld)
 *
 *     Rr 3-bit source register in r16, ..., r23 (mulsu, fmul, fmuls, fmulsu)
 *        4-bit source register in r16, ..., r31 (muls, sts)
 *        4-bit source register in r0, r2, ..., r30 (movw)
 *        5-bit source register in r0, r1, ..., r31 (add, adc, sub, sbc, and,
 *              or, eor, tst, mul, cpse, cp, cpc, sbrc, sbrs, mov, sts, st,
 *              std, out, push, bst, u/bst, u/sbrc, u/sbrs)
 *
 *     s  SREG bit number 0..7 (brbs, brbc, bset, bclr)
 *
 * - Source: Table was curated from https://github.com/nlitsme/AVRInstructionSet
 */

const AVR_opcode avr_opcodes[MNEMO_N] = {
#define OP_ID(nam) MNEMO_##nam, #nam

  // Arithmetic and Logic Instructions
  {OP_ID(lsl), 0xfc00, 0x0c00, 1, OP_AVR1, "0000 11d=  dddd ====", OTY_ALBI | OTY_RALL | OTY_CONSTRAINT,
      "lsl", "Rd", "logical shift left", "C <-- Rd(7) <-- Rd(6) ... Rd(1) <-- Rd(0) <-- 0", "--H-VNZC",
    {"1", "1", "1", "1"}, "alias for add Rd, Rd"},
  {OP_ID(add), 0xfc00, 0x0c00, 1, OP_AVR1, "0000 11rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "add", "Rd, Rr", "add without carry", "Rd <-- Rd + Rr", "--HSVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(rol), 0xfc00, 0x1c00, 1, OP_AVR1, "0001 11d=  dddd ====", OTY_ALBI | OTY_RALL | OTY_CONSTRAINT,
      "rol", "Rd", "rotate left through carry", "C <-- Rd(7) <-- Rd(6) ... Rd(1) <-- Rd(0) <-- C", "--H-VNZC",
    {"1", "1", "1", "1"}, "alias for adc Rd, Rd"},
  {OP_ID(adc), 0xfc00, 0x1c00, 1, OP_AVR1, "0001 11rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "adc", "Rd, Rr", "add with carry", "Rd <-- Rd + Rr + C", "--HSVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(ror), 0xfe0f, 0x9407, 1, OP_AVR1, "1001 010d  dddd 0111", OTY_ALBI | OTY_RALL,
      "ror", "Rd", "rotate right through carry", "C --> Rd(7) --> Rd(6) ... Rd(1) --> Rd(0) --> C", "----VNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(asr), 0xfe0f, 0x9405, 1, OP_AVR1, "1001 010d  dddd 0101", OTY_ALBI | OTY_RALL,
      "asr", "Rd", "arithmetic shift right", "Rd(7) --> Rd(7) --> Rd(6) ... Rd(1) --> Rd(0) --> C", "----VNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(adiw), 0xff00, 0x9600, 1, OP_AVR2nRC, "1001 0110  KKdd KKKK", OTY_ALBI | OTY_RW24,
      "adiw", "Rd, K", "add immediate to word", "Rd+1:Rd <-- Rd+1:Rd + K", "---SVNZC",
    {"2", "2", "2", "n/a"}, "d in {24, 26, 28, 30}"},
  {OP_ID(sub), 0xfc00, 0x1800, 1, OP_AVR1, "0001 10rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "sub", "Rd, Rr", "subtract without carry", "Rd <-- Rd - Rr", "--HSVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(subi), 0xf000, 0x5000, 1, OP_AVR1, "0101 KKKK  dddd KKKK", OTY_ALBI | OTY_RUPP,
      "subi", "Rd, K", "subtract immediate", "Rd <-- Rd - K", "--HSVNZC",
    {"1", "1", "1", "1"}, "d = 16..31"},
  {OP_ID(sbc), 0xfc00, 0x0800, 1, OP_AVR1, "0000 10rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "sbc", "Rd, Rr", "subtract with carry", "Rd <-- Rd - Rr - C", "--HSVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(sbci), 0xf000, 0x4000, 1, OP_AVR1, "0100 KKKK  dddd KKKK", OTY_ALBI | OTY_RUPP,
      "sbci", "Rd, K", "subtract immediate with carry", "Rd <-- Rd - K - C", "--HSVNZC",
    {"1", "1", "1", "1"}, "d = 16..31"},
  {OP_ID(sbiw), 0xff00, 0x9700, 1, OP_AVR2nRC, "1001 0111  KKdd KKKK", OTY_ALBI | OTY_RW24,
      "sbiw", "Rd, K", "subtract immediate from word", "Rd+1:Rd <-- Rd+1:Rd - K", "---SVNZC",
    {"2", "2", "2", "n/a"}, "d in {24, 26, 28, 30}"},
  {OP_ID(tst), 0xfc00, 0x2000, 1, OP_AVR1, "0010 00r=  rrrr ====", OTY_ALBI | OTY_RALL | OTY_CONSTRAINT,
      "tst", "Rr", "test for zero or minus", "Rr <-- Rr & Rr", "---SVNZ-",
    {"1", "1", "1", "1"}, "alias for and Rd, Rd"},
  {OP_ID(and), 0xfc00, 0x2000, 1, OP_AVR1, "0010 00rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "and", "Rd, Rr", "logical and", "Rd <-- Rd & Rr", "---SVNZ-",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(andi), 0xf000, 0x7000, 1, OP_AVR1, "0111 KKKK  dddd KKKK", OTY_ALBI | OTY_RUPP,
      "andi", "Rd, K", "logical and with immediate", "Rd <-- Rd & K", "---SVNZ-",
    {"1", "1", "1", "1"}, "d = 16..31"},
  {OP_ID(cbr), 0xf000, 0x7000, 1, OP_AVR1, "0111 KKKK  dddd KKKK", OTY_ALBI | OTY_RUPP | OTY_ALIAS,
      "cbr", "Rd, K", "clear bit(s) in register", "Rd <-- Rd & (0xff - K)", "---SVNZ-",
    {"1", "1", "1", "1"}, "alias for andi Rd, (0xff - K); d = 16..31"},
  {OP_ID(or), 0xfc00, 0x2800, 1, OP_AVR1, "0010 10rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "or", "Rd, Rr", "logical or", "Rd <-- Rd | Rr", "---SVNZ-",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(ori), 0xf000, 0x6000, 1, OP_AVR1, "0110 KKKK  dddd KKKK", OTY_ALBI | OTY_RUPP,
      "ori", "Rd, K", "logical or with immediate", "Rd <-- Rd | K", "---SVNZ-",
    {"1", "1", "1", "1"}, "d = 16..31"},
  {OP_ID(sbr), 0xf000, 0x6000, 1, OP_AVR1, "0110 KKKK  dddd KKKK", OTY_ALBI | OTY_RUPP | OTY_ALIAS,
      "sbr", "Rd, K", "set bit(s) in register", "Rd <-- Rd | K", "---SVNZ-",
    {"1", "1", "1", "1"}, "alias for ori Rd, K; d = 16..31"},
  {OP_ID(clr), 0xfc00, 0x2400, 1, OP_AVR1, "0010 01d=  dddd ====", OTY_ALBI | OTY_RALL | OTY_CONSTRAINT,
      "clr", "Rd", "clear register", "Rd <-- Rd ^ Rd", "---SVNZ-",
    {"1", "1", "1", "1"}, "alias for eor Rd, Rd"},
  {OP_ID(eor), 0xfc00, 0x2400, 1, OP_AVR1, "0010 01rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "eor", "Rd, Rr", "exclusive or", "Rd <-- Rd ^ Rr", "---SVNZ-",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(com), 0xfe0f, 0x9400, 1, OP_AVR1, "1001 010d  dddd 0000", OTY_ALBI | OTY_RALL,
      "com", "Rd", "one's complement", "Rd <-- 0xff - Rd", "---SVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(neg), 0xfe0f, 0x9401, 1, OP_AVR1, "1001 010d  dddd 0001", OTY_ALBI | OTY_RALL,
      "neg", "Rd", "two's complement", "Rd <-- 0x00 - Rd", "--HSVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(inc), 0xfe0f, 0x9403, 1, OP_AVR1, "1001 010d  dddd 0011", OTY_ALBI | OTY_RALL,
      "inc", "Rd", "increment", "Rd <-- Rd + 1", "---SVNZ-",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(dec), 0xfe0f, 0x940a, 1, OP_AVR1, "1001 010d  dddd 1010", OTY_ALBI | OTY_RALL,
      "dec", "Rd", "decrement", "Rd <-- Rd - 1", "---SVNZ-",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(mul), 0xfc00, 0x9c00, 1, OP_AVR4, "1001 11rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "mul", "Rd, Rr", "multiply unsigned", "R1:R0 <-- Rd x Rr (UU)", "------ZC",
    {"2", "2", "2", "n/a"}, ""},
  {OP_ID(muls), 0xff00, 0x0200, 1, OP_AVR4, "0000 0010  dddd rrrr", OTY_ALBI | OTY_RUPP,
      "muls", "Rd, Rr", "multiply signed", "R1:R0 <-- Rd x Rr (SS)", "------ZC",
    {"2", "2", "2", "n/a"}, "d, r = 16..31"},
  {OP_ID(mulsu), 0xff88, 0x0300, 1, OP_AVR4, "0000 0011  0ddd 0rrr", OTY_ALBI | OTY_RUPP,
      "mulsu", "Rd, Rr", "multiply signed with unsigned", "R1:R0 <-- Rd x Rr (SU)", "------ZC",
    {"2", "2", "2", "n/a"}, "d, r = 16..23"},
  {OP_ID(fmul), 0xff88, 0x0308, 1, OP_AVR4, "0000 0011  0ddd 1rrr", OTY_ALBI | OTY_RUPP,
      "fmul", "Rd, Rr", "fractional multiply unsigned", "R1:R0 <-- Rd x Rr << 1 (UU)", "------ZC",
    {"2", "2", "2", "n/a"}, "d, r = 16..23"},
  {OP_ID(fmuls), 0xff88, 0x0380, 1, OP_AVR4, "0000 0011  1ddd 0rrr", OTY_ALBI | OTY_RUPP,
      "fmuls", "Rd, Rr", "fractional multiply signed", "R1:R0 <-- Rd x Rr << 1 (SS)", "------ZC",
    {"2", "2", "2", "n/a"}, "d, r = 16..23"},
  {OP_ID(fmulsu), 0xff88, 0x0388, 1, OP_AVR4, "0000 0011  1ddd 1rrr", OTY_ALBI | OTY_RUPP,
      "fmulsu", "Rd, Rr", "fractional multiply signed with unsigned", "R1:R0 <-- Rd x Rr << 1 (SU)", "------ZC",
    {"2", "2", "2", "n/a"}, "d, r = 16..23"},
  {OP_ID(des), 0xff0f, 0x940b, 1, OP_AVR_XM, "1001 0100  KKKK 1011", OTY_ALBI,
      "des", "K", "data encryption round",
      "if (H = 0) then R15:R0 <-- Encrypt(R15:R0, K) if (H = 1) then R15:R0 <-- Decrypt(R15:R0, K)", "--------",
    {"n/a", "1/2", "n/a", "n/a"}, ""},

  // Branch Instructions (and compare)
  {OP_ID(rjmp), 0xf000, 0xc000, 1, OP_AVR1, "1100 kkkk  kkkk kkkk", OTY_RJMI,
      "rjmp", "k", "relative jump", "PC <-- PC + k + 1", "--------",
    {"2", "2", "2", "2"}, ""},
  {OP_ID(ijmp), 0xffff, 0x9409, 1, OP_AVR2, "1001 0100  0000 1001", OTY_JMPI | OTY_ZWORD,
      "ijmp", "", "indirect jump to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- 0", "--------",
    {"2", "2", "2", "2"}, ""},
  {OP_ID(eijmp), 0xffff, 0x9419, 1, OP_AVR_XL, "1001 0100  0001 1001", OTY_JMPX | OTY_ZWORD,
      "eijmp", "", "extended indirect jump to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- EIND", "--------",
    {"2", "2", "2", "n/a"}, ""},
  {OP_ID(jmp), 0xfe0e, 0x940c, 2, OP_AVR_M, "1001 010k  kkkk 110k   kkkk kkkk  kkkk kkkk", OTY_JMPI,
      "jmp", "k", "jump", "PC <-- k", "--------",
    {"3", "3", "3", "n/a"}, ""},
  {OP_ID(rcall), 0xf000, 0xd000, 1, OP_AVR1, "1101 kkkk  kkkk kkkk", OTY_RJMX,
      "rcall", "k", "relative call subroutine", "PC <-- PC + k + 1", "--------",
    {"3+", "2+", "2+", "3"}, ""},
  {OP_ID(icall), 0xffff, 0x9509, 1, OP_AVR2, "1001 0101  0000 1001", OTY_JMPX | OTY_ZWORD,
      "icall", "", "indirect call to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- 0", "--------",
    {"3+", "2+", "2+", "3"}, ""},
  {OP_ID(eicall), 0xffff, 0x9519, 1, OP_AVR_XL, "1001 0101  0001 1001", OTY_JMPX | OTY_ZWORD,
      "eicall", "", "extended indirect call to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- EIND", "--------",
    {"4", "3", "3", "n/a"}, ""},
  {OP_ID(call), 0xfe0e, 0x940e, 2, OP_AVR_M, "1001 010k  kkkk 111k   kkkk kkkk  kkkk kkkk", OTY_JMPX,
      "call", "k", "call subroutine", "PC <-- k, STACK <-- PC, SP <-- SP - 2", "--------",
    {"4+", "3+", "3+", "n/a"}, ""},
  {OP_ID(ret), 0xffff, 0x9508, 1, OP_AVR1, "1001 0101  0000 1000", OTY_JMPX,
      "ret", "", "subroutine return", "SP <-- SP + 2, PC <-- STACK", "--------",
    {"4+", "4+", "4+", "6"}, ""},
  {OP_ID(reti), 0xffff, 0x9518, 1, OP_AVR1, "1001 0101  0001 1000", OTY_JMPX,
      "reti", "", "interrupt return", "SP <-- SP + 2, PC <-- STACK", "I-------",
    {"4+", "4+", "4+", "6"}, ""},
  {OP_ID(cpse), 0xfc00, 0x1000, 1, OP_AVR1, "0001 00rd  dddd rrrr", OTY_SKPI | OTY_RALL,
      "cpse", "Rd, Rr", "compare and skip if equal", "if(Rd=Rr) PC <-- PC + 2/3", "--------",
    {"1-3", "1-3", "1-3", "1/2"}, ""},
  {OP_ID(cp), 0xfc00, 0x1400, 1, OP_AVR1, "0001 01rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "cp", "Rd, Rr", "compare", "Rd - Rr", "--HSVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(cpc), 0xfc00, 0x0400, 1, OP_AVR1, "0000 01rd  dddd rrrr", OTY_ALBI | OTY_RALL,
      "cpc", "Rd, Rr", "compare with carry", "Rd - Rr - C", "--HSVNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(cpi), 0xf000, 0x3000, 1, OP_AVR1, "0011 KKKK  dddd KKKK", OTY_ALBI | OTY_RUPP,
      "cpi", "Rd, K", "compare with immediate", "Rd - K", "--HSVNZC",
    {"1", "1", "1", "1"}, "d = 16..31"},
  {OP_ID(sbrc), 0xfe08, 0xfc00, 1, OP_AVR1, "1111 110r  rrrr 0bbb", OTY_SKPI | OTY_RALL,
      "sbrc", "Rr, b", "skip if bit in register cleared", "if(Rr(b)=0) PC <-- PC + 2/3", "--------",
    {"1-3", "1-3", "1-3", "1/2"}, ""},
  {OP_ID(sbrs), 0xfe08, 0xfe00, 1, OP_AVR1, "1111 111r  rrrr 0bbb", OTY_SKPI | OTY_RALL,
      "sbrs", "Rr, b", "skip if bit in register set", "if(Rr(b)=1) PC <-- PC + 2/3", "--------",
    {"1-3", "1-3", "1-3", "1/2"}, ""},
  {OP_ID(sbic), 0xff00, 0x9900, 1, OP_AVR1, "1001 1001  AAAA Abbb", OTY_SKPX,
      "sbic", "A, b", "skip if bit in I/O register cleared", "if(I/O(A,b)=0) PC <-- PC + 2/3", "--------",
    {"1-3", "2-4", "1-3", "1/2"}, ""},
  {OP_ID(sbis), 0xff00, 0x9b00, 1, OP_AVR1, "1001 1011  AAAA Abbb", OTY_SKPX,
      "sbis", "A, b", "skip if bit in I/O register set", "If(I/O(A,b)=1) PC <-- PC + 2/3", "--------",
    {"1-3", "2-4", "1-3", "1/2"}, ""},
  {OP_ID(brcs), 0xfc07, 0xf000, 1, OP_AVR1, "1111 00kk  kkkk k000", OTY_BRAI,
      "brcs", "k", "branch if carry set", "if(C=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 0, k (C Carry)"},
  {OP_ID(brlo), 0xfc07, 0xf000, 1, OP_AVR1, "1111 00kk  kkkk k000", OTY_BRAI | OTY_ALIAS,
      "brlo", "k", "branch if lower", "if(C=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 0, k (C Carry)"},
  {OP_ID(breq), 0xfc07, 0xf001, 1, OP_AVR1, "1111 00kk  kkkk k001", OTY_BRAI,
      "breq", "k", "branch if equal", "if(Z=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 1, k (Z Zero)"},
  {OP_ID(brmi), 0xfc07, 0xf002, 1, OP_AVR1, "1111 00kk  kkkk k010", OTY_BRAI,
      "brmi", "k", "branch if minus", "if(N=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 2, k (N Negative)"},
  {OP_ID(brvs), 0xfc07, 0xf003, 1, OP_AVR1, "1111 00kk  kkkk k011", OTY_BRAI,
      "brvs", "k", "branch if overflow flag is set", "if(V=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 3, k (V Overflow in two's complement)"},
  {OP_ID(brlt), 0xfc07, 0xf004, 1, OP_AVR1, "1111 00kk  kkkk k100", OTY_BRAI,
      "brlt", "k", "branch if less than (signed)", "if(N^V=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 4, k (S Sign)"},
  {OP_ID(brhs), 0xfc07, 0xf005, 1, OP_AVR1, "1111 00kk  kkkk k101", OTY_BRAI,
      "brhs", "k", "branch if half-carry flag set", "if(H=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 5, k (H Half carry)"},
  {OP_ID(brts), 0xfc07, 0xf006, 1, OP_AVR1, "1111 00kk  kkkk k110", OTY_BRAI,
      "brts", "k", "branch if T flag set", "if(T=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 6, k (T Transfer bit)"},
  {OP_ID(brie), 0xfc07, 0xf007, 1, OP_AVR1, "1111 00kk  kkkk k111", OTY_BRAI,
      "brie", "k", "branch if interrupt enabled", "if(I=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbs 7, k (I Interrupt enable)"},
  {OP_ID(brbs), 0xfc00, 0xf000, 1, OP_AVR1, "1111 00kk  kkkk ksss", OTY_BRAI,
      "brbs", "s, k", "branch if status flag set", "if(SREG(s)=1) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, ""},
  {OP_ID(brcc), 0xfc07, 0xf400, 1, OP_AVR1, "1111 01kk  kkkk k000", OTY_BRAI,
      "brcc", "k", "branch if carry cleared", "if(C=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 0, k (C Carry)"},
  {OP_ID(brsh), 0xfc07, 0xf400, 1, OP_AVR1, "1111 01kk  kkkk k000", OTY_BRAI | OTY_ALIAS,
      "brsh", "k", "branch if same or higher", "if(C=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 0, k (C Carry)"},
  {OP_ID(brne), 0xfc07, 0xf401, 1, OP_AVR1, "1111 01kk  kkkk k001", OTY_BRAI,
      "brne", "k", "branch if not equal", "if(Z=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 1, k (Z Zero)"},
  {OP_ID(brpl), 0xfc07, 0xf402, 1, OP_AVR1, "1111 01kk  kkkk k010", OTY_BRAI,
      "brpl", "k", "branch if plus", "if(N=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 2, k (N Negative)"},
  {OP_ID(brvc), 0xfc07, 0xf403, 1, OP_AVR1, "1111 01kk  kkkk k011", OTY_BRAI,
      "brvc", "k", "branch if overflow flag is cleared", "if(V=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 3, k (V Overflow in two's complement)"},
  {OP_ID(brge), 0xfc07, 0xf404, 1, OP_AVR1, "1111 01kk  kkkk k100", OTY_BRAI,
      "brge", "k", "branch if greater or equal (signed)", "if(N^V=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 4, k (S Sign)"},
  {OP_ID(brhc), 0xfc07, 0xf405, 1, OP_AVR1, "1111 01kk  kkkk k101", OTY_BRAI,
      "brhc", "k", "branch if half-carry flag cleared", "if(H=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 5, k (H Half carry)"},
  {OP_ID(brtc), 0xfc07, 0xf406, 1, OP_AVR1, "1111 01kk  kkkk k110", OTY_BRAI,
      "brtc", "k", "branch if T flag cleared", "if(T=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 6, k (T Transfer bit)"},
  {OP_ID(brid), 0xfc07, 0xf407, 1, OP_AVR1, "1111 01kk  kkkk k111", OTY_BRAI,
      "brid", "k", "branch if interrupt disabled", "if(I=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, "alias for brbc 7, k (I Interrupt enable)"},
  {OP_ID(brbc), 0xfc00, 0xf400, 1, OP_AVR1, "1111 01kk  kkkk ksss", OTY_BRAI,
      "brbc", "s, k", "branch if status flag cleared", "if(SREG(s)=0) then PC <-- PC + k + 1", "--------",
    {"1/2", "1/2", "1/2", "1/2"}, ""},

  // Data Transfer Instructions
  {OP_ID(mov), 0xfc00, 0x2c00, 1, OP_AVR1, "0010 11rd  dddd rrrr", OTY_XFRI | OTY_RALL,
      "mov", "Rd, Rr", "copy register", "Rd <-- Rr", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(movw), 0xff00, 0x0100, 1, OP_AVR25, "0000 0001  dddd rrrr", OTY_XFRI | OTY_REVN,
      "movw", "Rd, Rr", "copy register pair", "Rd+1:Rd <-- Rr+1:Rr", "--------",
    {"1", "1", "1", "n/a"}, "d, r in {0, 2, ..., 30}"},
  {OP_ID(ser), 0xff0f, 0xef0f, 1, OP_AVR1, "1110 1111  dddd 1111", OTY_ALBI | OTY_RUPP,
      "ser", "Rd", "set register", "Rd <-- 0xff", "--------",
    {"1", "1", "1", "1"}, "alias for ldi Rd, 0xff; d = 16..31"},
  {OP_ID(ldi), 0xf000, 0xe000, 1, OP_AVR1, "1110 KKKK  dddd KKKK", OTY_XFRI | OTY_RUPP,
      "ldi", "Rd, K", "load immediate", "Rd <-- K", "--------",
    {"1", "1", "1", "1"}, "d = 16..31"},
  {OP_ID(lds), 0xfe0f, 0x9000, 2, OP_AVR2nRC, "1001 000d  dddd 0000   kkkk kkkk  kkkk kkkk", OTY_XFRX | OTY_RALL,
      "lds", "Rd, k", "load direct from data space", "Rd <-- (k)", "--------",
    {"2", "3", "3", "2"}, ""},
  {OP_ID(ld_x), 0xfe0f, 0x900c, 1, OP_AVR2, "1001 000d  dddd 1100", OTY_XFRX | OTY_RALL,
      "ld", "Rd, X", "load indirect", "Rd <-- (X)", "--------",
    {"2", "2", "2", "1/2"}, ""},
  {OP_ID(ld_xp), 0xfe0f, 0x900d, 1, OP_AVR2, "1001 000d  dddd 1101", OTY_XFRX | OTY_RALL | OTY_XWRN,
      "ld", "Rd, X+", "load indirect and post-increment", "Rd <-- (X), X <-- X + 1", "--------",
    {"2", "2", "2", "2/3"}, ""},
  {OP_ID(ld_mx), 0xfe0f, 0x900e, 1, OP_AVR2, "1001 000d  dddd 1110", OTY_XFRX | OTY_RALL | OTY_XWRN,
      "ld", "Rd, -X", "load indirect and pre-decrement", "X <-- X - 1, Rd <-- (X)", "--------",
    {"2", "3", "2", "2/3"}, ""},
  {OP_ID(ld_y), 0xfe0f, 0x8008, 1, OP_AVR2, "1000 000d  dddd 1000", OTY_XFRX | OTY_RALL,
      "ld", "Rd, Y", "load indirect", "Rd <-- (Y) <-- (Y)", "--------",
    {"2", "2", "2", "1/2"}, "alias for ldd Rd, Y+0"},
  {OP_ID(ld_yp), 0xfe0f, 0x9009, 1, OP_AVR2, "1001 000d  dddd 1001", OTY_XFRX | OTY_RALL | OTY_YWRN,
      "ld", "Rd, Y+", "load indirect and post-increment", "Rd <-- (Y), Y <-- Y + 1", "--------",
    {"2", "2", "2", "2/3"}, ""},
  {OP_ID(ld_my), 0xfe0f, 0x900a, 1, OP_AVR2, "1001 000d  dddd 1010", OTY_XFRX | OTY_RALL | OTY_YWRN,
      "ld", "Rd, -Y", "load indirect and pre-decrement", "Y <-- Y - 1 Rd <-- (Y)", "--------",
    {"2", "3", "2", "2/3"}, ""},
  {OP_ID(ldd_y), 0xd208, 0x8008, 1, OP_AVR2nRC, "10q0 qq0d  dddd 1qqq", OTY_XFRX | OTY_RALL,
      "ldd", "Rd, Y+q", "load indirect with displacement", "Rd <-- (Y+q)", "--------",
    {"2", "3", "2", "n/a"}, ""},
  {OP_ID(ld_z), 0xfe0f, 0x8000, 1, OP_AVR1, "1000 000d  dddd 0000", OTY_XFRX | OTY_RALL,
      "ld", "Rd, Z", "load indirect", "Rd <-- (Z)", "--------",
    {"2", "2", "2", "1/2"}, "alias for ldd Rd, Z+0"},
  {OP_ID(ld_zp), 0xfe0f, 0x9001, 1, OP_AVR1, "1001 000d  dddd 0001", OTY_XFRX | OTY_RALL | OTY_ZWRN,
      "ld", "Rd, Z+", "load indirect and post-increment", "Rd <-- (Z), Z <-- Z + 1", "--------",
    {"2", "2", "2", "2/3"}, ""},
  {OP_ID(ld_mz), 0xfe0f, 0x9002, 1, OP_AVR1, "1001 000d  dddd 0010", OTY_XFRX | OTY_RALL | OTY_ZWRN,
      "ld", "Rd, -Z", "load indirect and pre-decrement", "Z <-- Z - 1, Rd <-- (Z)", "--------",
    {"2", "3", "2", "2/3"}, ""},
  {OP_ID(ldd_z), 0xd208, 0x8000, 1, OP_AVR2nRC, "10q0 qq0d  dddd 0qqq", OTY_XFRX | OTY_RALL,
      "ldd", "Rd, Z+q", "load indirect with displacement", "Rd <-- (Z+q)", "--------",
    {"2", "3", "2", "n/a"}, ""},
  {OP_ID(sts), 0xfe0f, 0x9200, 2, OP_AVR2nRC, "1001 001r  rrrr 0000   kkkk kkkk  kkkk kkkk", OTY_XFRX | OTY_RALL,
      "sts", "k, Rr", "store direct to data space", "(k) <-- Rr", "--------",
    {"2", "2", "2", "1"}, ""},
  {OP_ID(st_x), 0xfe0f, 0x920c, 1, OP_AVR2, "1001 001r  rrrr 1100", OTY_XFRX | OTY_RALL,
      "st", "X, Rr", "store indirect", "(X) <-- Rr", "--------",
    {"2", "1", "1", "1"}, ""},
  {OP_ID(st_xp), 0xfe0f, 0x920d, 1, OP_AVR2, "1001 001r  rrrr 1101", OTY_XFRX | OTY_RALL | OTY_XWRN,
      "st", "X+, Rr", "store indirect and post-increment", "(X) <-- Rr, X <-- X + 1", "--------",
    {"2", "1", "1", "1"}, ""},
  {OP_ID(st_mx), 0xfe0f, 0x920e, 1, OP_AVR2, "1001 001r  rrrr 1110", OTY_XFRX | OTY_RALL | OTY_XWRN,
      "st", "-X, Rr", "store indirect and pre-decrement", "X <-- X - 1, (X) <-- Rr", "--------",
    {"2", "2", "1", "1"}, ""},
  {OP_ID(st_y), 0xfe0f, 0x8208, 1, OP_AVR2, "1000 001r  rrrr 1000", OTY_XFRX | OTY_RALL,
      "st", "Y, Rr", "store indirect", "(Y) <-- Rr", "--------",
    {"2", "1", "1", "1"}, "alias for std Y+0, Rr"},
  {OP_ID(st_yp), 0xfe0f, 0x9209, 1, OP_AVR2, "1001 001r  rrrr 1001", OTY_XFRX | OTY_RALL | OTY_YWRN,
      "st", "Y+, Rr", "store indirect and post-increment", "(Y) <-- Rr, Y <-- Y + 1", "--------",
    {"2", "1", "1", "1"}, ""},
  {OP_ID(st_my), 0xfe0f, 0x920a, 1, OP_AVR2, "1001 001r  rrrr 1010", OTY_XFRX | OTY_RALL | OTY_YWRN,
      "st", "-Y, Rr", "store indirect and pre-decrement", "Y <-- Y - 1, (Y) <-- Rr", "--------",
    {"2", "2", "1", "1"}, ""},
  {OP_ID(std_y), 0xd208, 0x8208, 1, OP_AVR2nRC, "10q0 qq1r  rrrr 1qqq", OTY_XFRX | OTY_RALL,
      "std", "Y+q, Rr", "store indirect with displacement", "(Y+q) <-- Rr", "--------",
    {"2", "2", "1", "n/a"}, ""},
  {OP_ID(st_z), 0xfe0f, 0x8200, 1, OP_AVR1, "1000 001r  rrrr 0000", OTY_XFRX | OTY_RALL,
      "st", "Z, Rr", "store indirect", "(Z) <-- Rr", "--------",
    {"2", "1", "1", "1"}, "alias for std Z+0, Rr"},
  {OP_ID(st_zp), 0xfe0f, 0x9201, 1, OP_AVR1, "1001 001r  rrrr 0001", OTY_XFRX | OTY_RALL | OTY_ZWRN,
      "st", "Z+, Rr", "store indirect and post-increment", "(Z) <-- Rr, Z <-- Z + 1", "--------",
    {"2", "1", "1", "1"}, ""},
  {OP_ID(st_mz), 0xfe0f, 0x9202, 1, OP_AVR1, "1001 001r  rrrr 0010", OTY_XFRX | OTY_RALL | OTY_ZWRN,
      "st", "-Z, Rr", "store indirect and pre-decrement", "Z <-- Z - 1, (Z) <-- Rr", "--------",
    {"2", "2", "1", "1"}, ""},
  {OP_ID(std_z), 0xd208, 0x8200, 1, OP_AVR2nRC, "10q0 qq1r  rrrr 0qqq", OTY_XFRX | OTY_RALL,
      "std", "Z+q, Rr", "store indirect with displacement", "(Z+q) <-- Rr", "--------",
    {"2", "2", "1", "n/a"}, ""},
  {OP_ID(lpm_0), 0xffff, 0x95c8, 1, OP_AVR1nRC, "1001 0101  1100 1000", OTY_XFRX,
      "lpm", "", "load program memory", "R0 <-- (Z)", "--------",
    {"3", "3", "3", "n/a"}, ""},
  {OP_ID(lpm_z), 0xfe0f, 0x9004, 1, OP_AVR25, "1001 000d  dddd 0100", OTY_XFRX | OTY_RALL,
      "lpm", "Rd, Z", "load program memory", "Rd <-- (Z)", "--------",
    {"3", "3", "3", "n/a"}, ""},
  {OP_ID(lpm_zp), 0xfe0f, 0x9005, 1, OP_AVR25, "1001 000d  dddd 0101", OTY_XFRX | OTY_RALL | OTY_ZWRN,
      "lpm", "Rd, Z+", "load program memory and post-increment", "Rd <-- (Z), Z <-- Z + 1", "--------",
    {"3", "3", "3", "n/a"}, ""},
  {OP_ID(elpm_0), 0xffff, 0x95d8, 1, OP_AVR_L, "1001 0101  1101 1000", OTY_XFRX,
      "elpm", "", "extended load program memory", "R0 <-- (RAMPZ:Z)", "--------",
    {"3", "3", "3", "n/a"}, ""},
  {OP_ID(elpm_z), 0xfe0f, 0x9006, 1, OP_AVR_L, "1001 000d  dddd 0110", OTY_XFRX | OTY_RALL,
      "elpm", "Rd, Z", "extended load program memory", "Rd <-- (RAMPZ:Z)", "--------",
    {"3", "3", "3", "n/a"}, ""},
  {OP_ID(elpm_zp), 0xfe0f, 0x9007, 1, OP_AVR_L, "1001 000d  dddd 0111", OTY_XFRX | OTY_RALL | OTY_ZWRN,
      "elpm", "Rd, Z+", "extended load program memory and post-increment", "Rd <-- (RAMPZ:Z), Z <-- Z + 1", "--------",
    {"3", "3", "3", "n/a"}, ""},
  {OP_ID(spm), 0xffff, 0x95e8, 1, OP_AVR25, "1001 0101  1110 1000", OTY_XFRX,
      "spm", "", "store program memory", "(RAMPZ:Z) <-- R1:R0", "--------",
    {"-", "-", "-", "-"}, ""},
  {OP_ID(spm_zp), 0xffff, 0x95f8, 1, OP_AVR_XTM, "1001 0101  1111 1000", OTY_XFRX,
      "spm", "Z+", "store program memory and post-increment by 2", "(RAMPZ:Z) <-- R1:R0, Z <-- Z + 2", "--------",
    {"n/a", "-", "-", "n/a"}, ""},
  {OP_ID(in), 0xf800, 0xb000, 1, OP_AVR1, "1011 0AAd  dddd AAAA", OTY_XFRX | OTY_RALL,
      "in", "Rd, A", "in from I/O location", "Rd <-- I/O(A)", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(out), 0xf800, 0xb800, 1, OP_AVR1, "1011 1AAr  rrrr AAAA", OTY_XFRX | OTY_RALL,
      "out", "A, Rr", "out to I/O location", "I/O(A) <-- Rr", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(push), 0xfe0f, 0x920f, 1, OP_AVR2, "1001 001r  rrrr 1111", OTY_XFRX | OTY_RALL,
      "push", "Rr", "push register on stack", "STACK <-- Rr, SP <-- SP - 2", "--------",
    {"2", "1", "1", "1"}, ""},
  {OP_ID(pop), 0xfe0f, 0x900f, 1, OP_AVR2, "1001 000d  dddd 1111", OTY_XFRX | OTY_RALL,
      "pop", "Rd", "pop register from stack", "SP <-- SP + 2, Rd <-- STACK", "--------",
    {"2", "2", "2", "3"}, ""},
  {OP_ID(xch), 0xfe0f, 0x9204, 1, OP_AVR_XM, "1001 001d  dddd 0100", OTY_XFRX | OTY_RALL,
      "xch", "Z, Rd", "exchange", "(Z) <-- Rd, Rd <-- (Z)", "--------",
    {"n/a", "1", "n/a", "n/a"}, ""},
  {OP_ID(las), 0xfe0f, 0x9205, 1, OP_AVR_XM, "1001 001d  dddd 0101", OTY_ALBX | OTY_RALL,
      "las", "Z, Rd", "load and set", "(Z) <-- Rd | (Z), Rd <-- (Z)", "--------",
    {"n/a", "1", "n/a", "n/a"}, ""},
  {OP_ID(lac), 0xfe0f, 0x9206, 1, OP_AVR_XM, "1001 001d  dddd 0110", OTY_ALBX | OTY_RALL,
      "lac", "Z, Rd", "load and clear", "(Z) <-- (0xff - Rd) & (Z), Rd <-- (Z)", "--------",
    {"n/a", "1", "n/a", "n/a"}, ""},
  {OP_ID(lat), 0xfe0f, 0x9207, 1, OP_AVR_XM, "1001 001d  dddd 0111", OTY_ALBX | OTY_RALL,
      "lat", "Z, Rd", "load and toggle", "(Z) <-- Rd ^ (Z), Rd <-- (Z)", "--------",
    {"n/a", "1", "n/a", "n/a"}, ""},

  // Bit and Bit-test Instructions
  {OP_ID(lsr), 0xfe0f, 0x9406, 1, OP_AVR1, "1001 010d  dddd 0110", OTY_ALBI | OTY_RALL,
      "lsr", "Rd", "logical shift right", "0 --> Rd(7) --> Rd(6) ... Rd(1) --> Rd(0) --> C", "----VNZC",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(swap), 0xfe0f, 0x9402, 1, OP_AVR1, "1001 010d  dddd 0010", OTY_ALBI | OTY_RALL,
      "swap", "Rd", "swap nibbles", "Rd(3..0) <--> Rd(7..4)", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(sbi), 0xff00, 0x9a00, 1, OP_AVR1, "1001 1010  AAAA Abbb", OTY_ALBX,
      "sbi", "A, b", "set bit in I/O register", "I/O(A,b) <-- 1", "--------",
    {"2", "1", "1", "1"}, ""},
  {OP_ID(cbi), 0xff00, 0x9800, 1, OP_AVR1, "1001 1000  AAAA Abbb", OTY_ALBX,
      "cbi", "A, b", "clear bit in I/O register", "I/O(A,b) <-- 0", "--------",
    {"2", "1", "1", "1"}, ""},
  {OP_ID(bst), 0xfe08, 0xfa00, 1, OP_AVR1, "1111 101r  rrrr 0bbb", OTY_ALBI | OTY_RALL,
      "bst", "Rr, b", "bit store from register to T", "T <-- Rr(b)", "-T------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(bld), 0xfe08, 0xf800, 1, OP_AVR1, "1111 100d  dddd 0bbb", OTY_ALBI | OTY_RALL,
      "bld", "Rd, b", "bit load from T to register", "Rd(b) <-- T", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(sec), 0xffff, 0x9408, 1, OP_AVR1, "1001 0100  0000 1000", OTY_ALBI,
      "sec", "", "set carry", "C <-- 1", "-------C",
    {"1", "1", "1", "1"}, "alias for bset 0"},
  {OP_ID(clc), 0xffff, 0x9488, 1, OP_AVR1, "1001 0100  1000 1000", OTY_ALBI,
      "clc", "", "clear carry", "C <-- 0", "-------C",
    {"1", "1", "1", "1"}, "alias for bclr 0"},
  {OP_ID(sen), 0xffff, 0x9428, 1, OP_AVR1, "1001 0100  0010 1000", OTY_ALBI,
      "sen", "", "set negative flag", "N <-- 1", "-----N--",
    {"1", "1", "1", "1"}, "alias for bset 2"},
  {OP_ID(cln), 0xffff, 0x94a8, 1, OP_AVR1, "1001 0100  1010 1000", OTY_ALBI,
      "cln", "", "clear negative flag", "N <-- 0", "-----N--",
    {"1", "1", "1", "1"}, "alias for bclr 2"},
  {OP_ID(sez), 0xffff, 0x9418, 1, OP_AVR1, "1001 0100  0001 1000", OTY_ALBI,
      "sez", "", "set zero flag", "Z <-- 1", "------Z-",
    {"1", "1", "1", "1"}, "alias for bset 1"},
  {OP_ID(clz), 0xffff, 0x9498, 1, OP_AVR1, "1001 0100  1001 1000", OTY_ALBI,
      "clz", "", "clear zero flag", "Z <-- 0", "------Z-",
    {"1", "1", "1", "1"}, "alias for bclr 1"},
  {OP_ID(sei), 0xffff, 0x9478, 1, OP_AVR1, "1001 0100  0111 1000", OTY_ALBX,
      "sei", "", "global interrupt enable", "I <-- 1", "I-------",
    {"1", "1", "1", "1"}, "alias for bset 7"},
  {OP_ID(cli), 0xffff, 0x94f8, 1, OP_AVR1, "1001 0100  1111 1000", OTY_ALBI,
      "cli", "", "global interrupt disable", "I <-- 0", "I-------",
    {"1", "1", "1", "1"}, "alias for bclr 7"},
  {OP_ID(ses), 0xffff, 0x9448, 1, OP_AVR1, "1001 0100  0100 1000", OTY_ALBI,
      "ses", "", "set signed test flag", "S <-- 1", "---S----",
    {"1", "1", "1", "1"}, "alias for bset 4"},
  {OP_ID(cls), 0xffff, 0x94c8, 1, OP_AVR1, "1001 0100  1100 1000", OTY_ALBI,
      "cls", "", "clear signed test flag", "S <-- 0", "---S----",
    {"1", "1", "1", "1"}, "alias for bclr 4"},
  {OP_ID(sev), 0xffff, 0x9438, 1, OP_AVR1, "1001 0100  0011 1000", OTY_ALBI,
      "sev", "", "set two's complement overflow", "V <-- 1", "----V---",
    {"1", "1", "1", "1"}, "alias for bset 3"},
  {OP_ID(clv), 0xffff, 0x94b8, 1, OP_AVR1, "1001 0100  1011 1000", OTY_ALBI,
      "clv", "", "clear two's complement overflow", "V <-- 0", "----V---",
    {"1", "1", "1", "1"}, "alias for bclr 3"},
  {OP_ID(set), 0xffff, 0x9468, 1, OP_AVR1, "1001 0100  0110 1000", OTY_ALBI,
      "set", "", "set T in sreg", "T <-- 1", "-T------",
    {"1", "1", "1", "1"}, "alias for bset 6"},
  {OP_ID(clt), 0xffff, 0x94e8, 1, OP_AVR1, "1001 0100  1110 1000", OTY_ALBI,
      "clt", "", "clear T in sreg", "T <-- 0", "-T------",
    {"1", "1", "1", "1"}, "alias for bclr 6"},
  {OP_ID(seh), 0xffff, 0x9458, 1, OP_AVR1, "1001 0100  0101 1000", OTY_ALBI,
      "seh", "", "set half-carry flag in sreg", "H <-- 1", "--H-----",
    {"1", "1", "1", "1"}, "alias for bset 5"},
  {OP_ID(clh), 0xffff, 0x94d8, 1, OP_AVR1, "1001 0100  1101 1000", OTY_ALBI,
      "clh", "", "clear half-carry flag in sreg", "H <-- 0", "--H-----",
    {"1", "1", "1", "1"}, "alias for bclr 5"},
  {OP_ID(bset), 0xff8f, 0x9408, 1, OP_AVR1, "1001 0100  0sss 1000", OTY_ALBX,
      "bset", "s", "flag set", "SREG(s) <-- 1", "SREG-bit",
    {"1", "1", "1", "1"}, "s = 0-7 = C,Z,N,V,S,H,T,I"},
  {OP_ID(bclr), 0xff8f, 0x9488, 1, OP_AVR1, "1001 0100  1sss 1000", OTY_ALBI,
      "bclr", "s", "flag clear", "SREG(s) <-- 0", "SREG-bit",
    {"1", "1", "1", "1"}, "s = 0-7 = C,Z,N,V,S,H,T,I"},

  // MCU Control Instructions
  {OP_ID(break), 0xffff, 0x9598, 1, OP_AVR1, "1001 0101  1001 1000", OTY_MCUX,
      "break", "", "break", "on-chip debug system breakpoint", "--------",
    {"-", "-", "-", "-"}, "not available on all parts"},
  {OP_ID(nop), 0xffff, 0x0000, 1, OP_AVR1, "0000 0000  0000 0000", OTY_MCUI,
      "nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(sleep), 0xffff, 0x9588, 1, OP_AVR1, "1001 0101  1000 1000", OTY_MCUX,
      "sleep", "", "sleep", "sleep mode defined by the MCU control register", "--------",
    {"-", "-", "-", "-"}, ""},
  {OP_ID(wdr), 0xffff, 0x95a8, 1, OP_AVR1, "1001 0101  1010 1000", OTY_MCUI,
      "wdr", "", "watchdog reset", "reset the watchdog timer", "--------",
    {"1", "1", "1", "1"}, ""},

  /*
   * Special 16-bit lds/sts opcodes for reduced-core ATtinys only; they conflict with
   * regular ldd Rd, Y+q; ldd Rd, Z+q; std Y+q, Rr; std Z+q, Rr opcodes
   */
  {OP_ID(lds_rc), 0xf800, 0xa000, 1, OP_AVR_RC, "1010 0aaa  dddd aaaa", OTY_XFRX | OTY_RUPP,
      "lds", "Rd, a", "load direct from data space", "Rd <-- (a)", "--------",
    {"n/a", "n/a", "n/a", "2"}, "AVRrc only (TPI parts); a = 0x40..0xbf"},
  {OP_ID(sts_rc), 0xf800, 0xa800, 1, OP_AVR_RC, "1010 1aaa  rrrr aaaa", OTY_XFRX | OTY_RUPP,
      "sts", "a, Rr", "store direct to data space", "(a) <-- Rr", "--------",
    {"n/a", "n/a", "n/a", "1"}, "AVRrc only (TPI parts); a = 0x40..0xbf"},

  // Undocumented opcodes: they are said(!) to do the following
  {OP_ID(u_nop_1), 0xff00, 0x0000, 1, OP_AVR_ILL, "0000 0000  xxxx xxxx", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, "xxxx xxxx != 0000 0000"},
  {OP_ID(u_nop_2), 0xfe0f, 0x9003, 1, OP_AVR_ILL, "1001 000x  xxxx 0011", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_nop_3), 0xfe0f, 0x9008, 1, OP_AVR_ILL, "1001 000x  xxxx 1000", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_nop_4), 0xfe0f, 0x900b, 1, OP_AVR_ILL, "1001 000x  xxxx 1011", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_nop_5), 0xfe0f, 0x9203, 1, OP_AVR_ILL, "1001 001x  xxxx 0011", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_nop_6), 0xfe0f, 0x9208, 1, OP_AVR_ILL, "1001 001x  xxxx 1000", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_nop_7), 0xfe0f, 0x920b, 1, OP_AVR_ILL, "1001 001x  xxxx 1011", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_icall), 0xff1f, 0x9509, 1, OP_AVR_ILL, "1001 0101  xxx0 1001", OTY_JMPX | OTY_ZWORD,
      "u/icall", "", "indirect call to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- 0", "--------",
    {"3+", "2+", "2+", "3"}, "xxx != 000"},
  {OP_ID(u_eicall), 0xff1f, 0x9519, 1, OP_AVR_ILL, "1001 0101  xxx1 1001", OTY_JMPX | OTY_ZWORD,
      "u/eicall", "", "extended indirect call to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- EIND", "--------",
    {"4", "3", "3", "n/a"}, "xxx != 000"},
  {OP_ID(u_ret), 0xff9f, 0x9508, 1, OP_AVR_ILL, "1001 0101  0xx0 1000", OTY_JMPX,
      "u/ret", "", "subroutine return", "SP --> SP + 2, PC <-- STACK", "--------",
    {"4+", "4+", "4+", "6"}, "xx != 00"},
  {OP_ID(u_reti), 0xff9f, 0x9518, 1, OP_AVR_ILL, "1001 0101  0xx1 1000", OTY_JMPX,
      "u/reti", "", "interrupt return", "SP -> SP + 2, PC <-- STACK", "I-------",
    {"4+", "4+", "4+", "6"}, "xx != 00"},
  {OP_ID(u_nop_8), 0xffff, 0x95b8, 1, OP_AVR_ILL, "1001 0101  1011 1000", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_nop_9), 0xfe0f, 0x9404, 1, OP_AVR_ILL, "1001 010x  xxxx 0100", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_nop_a), 0xff0f, 0x950b, 1, OP_AVR_ILL, "1001 0101  xxxx 1011", OTY_MCUI,
      "u/nop", "", "no operation", "-", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_ijmp), 0xff1f, 0x9409, 1, OP_AVR_ILL, "1001 0100  xxx0 1001", OTY_JMPI | OTY_ZWORD,
      "u/ijmp", "", "indirect jump to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- 0", "--------",
    {"2", "2", "2", "2"}, "xxx != 000"},
  {OP_ID(u_eijmp), 0xff1f, 0x9419, 1, OP_AVR_ILL, "1001 0100  xxx1 1001", OTY_JMPX | OTY_ZWORD,
      "u/eijmp", "", "extended indirect jump to (Z)", "PC(15:0) <-- Z, PC(21:16) <-- EIND", "--------",
    {"2", "2", "2", "n/a"}, "xxx != 000"},
  {OP_ID(u_bld), 0xfe08, 0xf808, 1, OP_AVR_ILL, "1111 100d  dddd 1bbb", OTY_ALBI | OTY_RALL,
      "u/bld", "Rd, b", "bit load from T to register", "Rd(b) <-- T", "--------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_bst), 0xfe08, 0xfa08, 1, OP_AVR_ILL, "1111 101r  rrrr 1bbb", OTY_ALBI | OTY_RALL,
      "u/bst", "Rr, b", "bit store from register to T", "T <-- Rr(b)", "-T------",
    {"1", "1", "1", "1"}, ""},
  {OP_ID(u_sbrc), 0xfe08, 0xfc08, 1, OP_AVR_ILL, "1111 110r  rrrr 1bbb", OTY_SKPI | OTY_RALL,
      "u/sbrc", "Rr, b", "skip if bit in register cleared", "if(Rr(b)=0) PC <-- PC + 2/3", "--------",
    {"1-3", "1-3", "1-3", "1/2"}, ""},
  {OP_ID(u_sbrs), 0xfe08, 0xfe08, 1, OP_AVR_ILL, "1111 111r  rrrr 1bbb", OTY_SKPI | OTY_RALL,
      "u/sbrs", "Rr, b", "skip if bit in register set", "if(Rr(b)=1) PC <-- PC + 2/3", "--------",
    {"1-3", "1-3", "1-3", "1/2"}, ""},
};

// Return whether or not the given 16-bit opcode is the mnemonic
int op16_is_mnemo(int op, AVR_mnemo mnemo) {
  return mnemo < 0 || mnemo >= MNEMO_N? 0:
    (op & avr_opcodes[mnemo].mask) != avr_opcodes[mnemo].value? 0:
    !(avr_opcodes[mnemo].type & OTY_CONSTRAINT)? 1:
    (op >> 0 & 15) == (op >> 4 & 15) && (op >> 9 & 1) == (op >> 8 & 1);  // Constraint Rd == Rr
}

// Return whether or not the given 16-bit opcode has a 16-bit address argument
int is_opcode32(int op) {
  return
    op16_is_mnemo(op, MNEMO_call) || op16_is_mnemo(op, MNEMO_jmp) ||
    op16_is_mnemo(op, MNEMO_sts) || op16_is_mnemo(op, MNEMO_lds);
}

// Return width of opcode in bytes (2 or 4)
int op_width(int op16) {
  return is_opcode32(op16)? 4: 2;
}

// Return the register number of the 16-bit ldi opcode and 0 if it's not ldi
int ldi_Rd(int op) {
  return op16_is_mnemo(op, MNEMO_ldi)? 16 + ((op >> 4) & 15): 0;
}

// Return the constant number in the 16-bit ldi opcode and -1 if it's not ldi
int ldi_K(int op) {
  return op16_is_mnemo(op, MNEMO_ldi)? ((op & 0x0f00) >> 4) | (op & 0x0f): -1;
}

// Returns bitmask of first character chr in bits
static int bitmask_first_chr(const char *bits, int chr) {
  int ret = 0x8000;

  for(const char *c = bits; *c && *c != chr && ret; c++)
    if(*c != ' ')
      ret >>= 1;
  return ret;
}

// Return first match of opcode that is compatible with avrlevel or MNEMO_NONE
AVR_mnemo opcode_mnemo(int op, int avrlevel) {
  for(AVR_mnemo i = 0; i < MNEMO_N; i++)
    if(avr_opcodes[i].avrlevel & avrlevel)
      if(op16_is_mnemo(op, i)) {
        if(avrlevel == PART_AVR_RC && (avr_opcodes[i].type & OTY_REG_MASK) == OTY_RALL) {
          // Reduced-core ATtiny does not have registers r0, ..., r15
          int bmask;            // Assert highest bit in 5-bit r/d is set

          if((bmask = bitmask_first_chr(avr_opcodes[i].bits, 'r')) && !(op & bmask))
            return MNEMO_NONE;
          if((bmask = bitmask_first_chr(avr_opcodes[i].bits, 'd')) && !(op & bmask))
            return MNEMO_NONE;
        }
        return i;
      }
  return MNEMO_NONE;
}

// Is 16-bit opcode valid for AVR part with avrlevel architecture?
int op16_is_valid(int op16, int avrlevel) {
  int mnemo = opcode_mnemo(op16, avrlevel);

  return mnemo >= 0 && mnemo <= MNEMO_N;
}

// Is 16-bit opcode valid and benign for AVR part with avrlevel architecture?
int op16_is_benign(int op16, int avrlevel) {    // Benign means no I/O or SRAM is being read/written
  int mnemo = opcode_mnemo(op16, avrlevel);

  return mnemo >= 0 && mnemo <= MNEMO_N && !(avr_opcodes[mnemo].type & OTY_EXTERNAL);
}

// Opcodes in avr_opcodes[] that a part ought to be able to run
int avr_get_archlevel(const AVRPART *p) {
  int ret =
    is_updi(p)? PART_AVR_XT: is_pdi(p)? PART_AVR_XM: is_tpi(p)? PART_AVR_RC: 0;

  if(!ret) {                    // Non-TPI classic part
    switch(p->archnum) {
    case 1:
      ret = PART_AVR1;
      break;
    default:                   // If AVRDUE doesn't know, it's probably rare & old
    case 2:
      ret = PART_AVR2;
      break;
    case 25:
      ret = PART_AVR25;
      break;
    case 3:
    case 31:
    case 35:                   // Sic
      ret = PART_AVR3;
      break;
      break;
    case 4:
      ret = PART_AVR4;
      break;
    case 5:
      ret = PART_AVR5;
      break;
    case 51:
      ret = PART_AVR51;
      break;
    case 6:
      ret = PART_AVR6;
    }
  }

  AVRMEM *mem = avr_locate_flash(p);

  if(mem) {                     // Add opcodes needed for large parts in any case
    if(mem->size > 8192)
      ret |= OP_AVR_M;          // JMP, CALL
    if(mem->size > 65536)
      ret |= OP_AVR_L;          // ELPM
    if(mem->size > 128*1024)
      ret |= OP_AVR_XL;         // EIJMP, EICALL
  }

  return ret;
}

// Index in the avr_opcodes[].clock[] array for timings of an opcode
AVR_cycle_index avr_get_cycle_index(const AVRPART *p) {
  return is_updi(p)? OP_AVRxt: is_pdi(p)? OP_AVRxm: is_tpi(p)? OP_AVRrc: OP_AVRe;
}

// Return the mnemonic string of an opcode
const char *mnemo_str(int op) {
  AVR_mnemo mnemo = opcode_mnemo(op, PART_ALL | OP_AVR_ILL);

  if(mnemo < 0 || mnemo >= MNEMO_N)
    return "illegal";
  return avr_opcodes[mnemo].opcode;
}

/*
 * Return
 *    2 if opcode treats Z register as word address
 *    1 if opcode treats Z as byte address
 *    0 otherwise (Z not involved in opcode)
 */
int z_width(int op16, AVR_mnemo *mnemop) {
  AVR_mnemo mlist[] = {
    MNEMO_ijmp, MNEMO_eijmp, MNEMO_icall, MNEMO_eicall, MNEMO_u_icall, MNEMO_u_eicall,
    MNEMO_u_ijmp, MNEMO_u_eijmp, MNEMO_ld_z, MNEMO_ld_zp, MNEMO_ld_mz, MNEMO_ldd_z,
    MNEMO_st_z, MNEMO_st_zp, MNEMO_st_mz, MNEMO_std_z, MNEMO_lpm_0, MNEMO_lpm_z,
    MNEMO_lpm_zp, MNEMO_elpm_0, MNEMO_elpm_z, MNEMO_elpm_zp, MNEMO_spm, MNEMO_spm_zp,
    MNEMO_xch, MNEMO_las, MNEMO_lac, MNEMO_lat,
  };

  for(AVR_mnemo m = 0; m < (AVR_mnemo) (sizeof mlist/sizeof *mlist); m++)
    if(op16_is_mnemo(op16, mlist[m])) {
      if(mnemop)
        *mnemop = mlist[m];
      return avr_opcodes[mlist[m]].type & OTY_ZWORD? 2: 1;
    }

  return 0;
}

// Where else could the PC of the 16-bit opcode op16 at address here move to other than here + 2?
int op16_target(int here, int op16) {
  AVR_mnemo mnemo = opcode_mnemo(op16, PART_ALL | OP_AVR_ILL);

  if(mnemo >= 0 && mnemo < MNEMO_N) {
    switch(avr_opcodes[mnemo].type & OTY_TYPE_MASK) {
    case OTY_RJMX:             // Relative call rcall, range [.-4096, .+4094] bytes
    case OTY_RJMI:             // Relative jump rjmp, range [.-4096, .+4094] bytes
      return here + 2 + ((int16_t) (op16 << 4) >> 3);

    case OTY_JMPI:             // Jump to potentially anywhere in flash (jmp, ijmp, eijmp)
    case OTY_JMPX:             // Jump to potentially anywhere in flash (calls and ret/i)
      return INT_MIN;

    case OTY_BRAI:             // Conditional branch, range [.-128, .+126] bytes
      return here + 2 + (int8_t) ((op16 & 0x3f8) >> 2);

    case OTY_SKPI:             // Conditional skip, range [.+0, .+4] (cpse, sbrc, sbrs)
    case OTY_SKPX:             // Conditional skip, range [.+0, .+4] (sbic, sbis)
      return here + 2 + 4;
    }
  }

  return here + 2;
}

// Rjmp opcode from byte distance; 0xcfff is an endless loop, 0xc000 is a nop
int dist2rjmp(int dist) {
  return 0xc000 | (((dist >> 1) - 1) & 0x0fff);
}
