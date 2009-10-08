/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id$ */

%{
/* need this for the call to atof() below */
#include <math.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "avrdude.h"

#include "config.h"
#include "config_gram.h"
#include "lists.h"

#define YY_NO_UNPUT

%}

DIGIT    [0-9]
HEXDIGIT [0-9a-fA-F]
ID       [_a-zA-Z][_a-zA-Z0-9]*
SIGN     [+-]

%x strng
%x incl
%x comment

/* Bump resources for classic lex. */
%e2000
%p10000
%n1000

%%

{SIGN}*{DIGIT}+            { yylval = number(yytext); return TKN_NUMBER; }
{SIGN}*{DIGIT}+"."{DIGIT}* { yylval = number(yytext); return TKN_NUMBER; }
{SIGN}*"."{DIGIT}*         { yylval = number(yytext); return TKN_NUMBER; }

"\""      { string_buf_ptr = string_buf; BEGIN(strng); }

0x{HEXDIGIT}+ { yylval = hexnumber(yytext); return TKN_NUMBER; }



#   { /* The following eats '#' style comments to end of line */
       BEGIN(comment); }
<comment>[^\n] { /* eat comments */ }
<comment>\n { lineno++; BEGIN(INITIAL); }


"/*" {  /* The following eats multiline C style comments */
        int c;
        int comment_start;
        
        comment_start = lineno;
        while (1) {
          while (((c = input()) != '*') && (c != EOF)) {
            /* eat up text of comment, but keep counting lines */
            if (c == '\n')
              lineno++;
          }
          
          if (c == '*') {
            while ((c = input()) == '*')
              ;
            if (c == '/')
              break;    /* found the end */
          }
          
          if (c == EOF) {
            fprintf(stderr, "error at %s:%d: EOF in comment\n", 
                    infile, lineno);
            fprintf(stderr, "    comment started on line %d\n", 
                    comment_start);
            exit(1);
            break;
          }
        }
     }


<strng>\" { *string_buf_ptr = 0; string_buf_ptr = string_buf;
             yylval = string(string_buf_ptr); BEGIN(INITIAL); return TKN_STRING; }
<strng>\\n  *string_buf_ptr++ = '\n';
<strng>\\t  *string_buf_ptr++ = '\t';
<strng>\\r  *string_buf_ptr++ = '\r';
<strng>\\b  *string_buf_ptr++ = '\b';
<strng>\\f  *string_buf_ptr++ = '\f';
<strng>\\(.|\n)  *(string_buf_ptr++) = yytext[1];
<strng>[^\\\n\"]+ { char *yptr = yytext; while (*yptr) 
                                         *(string_buf_ptr++) = *(yptr++); }
<strng>\n { fprintf(stderr, "error at line %d: unterminated character constant\n",
            lineno); 
            exit(1); }

allowfullpagebitstream { yylval=NULL; return K_ALLOWFULLPAGEBITSTREAM; }
arduino          { yylval=NULL; return K_ARDUINO; }
avr910           { yylval=NULL; return K_AVR910; }
avr910_devcode   { yylval=NULL; return K_AVR910_DEVCODE; }
usbasp           { yylval=NULL; return K_USBASP; }
usbtiny          { yylval=NULL; return K_USBTINY; }
bank_size        { yylval=NULL; return K_PAGE_SIZE; }
banked           { yylval=NULL; return K_PAGED; }
baudrate         { yylval=NULL; return K_BAUDRATE; }
bs2              { yylval=NULL; return K_BS2; }
buff             { yylval=NULL; return K_BUFF; }
buspirate        { yylval=NULL; return K_BUSPIRATE; }
butterfly        { yylval=NULL; return K_BUTTERFLY; }
chip_erase_delay { yylval=NULL; return K_CHIP_ERASE_DELAY; }
desc             { yylval=NULL; return K_DESC; }
default_parallel { yylval=NULL; return K_DEFAULT_PARALLEL; }
default_programmer { yylval=NULL; return K_DEFAULT_PROGRAMMER; }
default_serial   { yylval=NULL; return K_DEFAULT_SERIAL; }
devicecode       { yylval=NULL; return K_DEVICECODE; }
dragon_dw        { yylval=NULL; return K_DRAGON_DW; }
dragon_hvsp      { yylval=NULL; return K_DRAGON_HVSP; }
dragon_isp       { yylval=NULL; return K_DRAGON_ISP; }
dragon_jtag      { yylval=NULL; return K_DRAGON_JTAG; }
dragon_pp        { yylval=NULL; return K_DRAGON_PP; }
eecr             { yylval=NULL; return K_EECR; }
eeprom           { yylval=NULL; return K_EEPROM; }
enablepageprogramming { yylval=NULL; return K_ENABLEPAGEPROGRAMMING; }
errled           { yylval=NULL; return K_ERRLED; }
flash            { yylval=NULL; return K_FLASH; }
has_jtag         { yylval=NULL; return K_HAS_JTAG; }
has_debugwire    { yylval=NULL; return K_HAS_DW; }
has_pdi          { yylval=NULL; return K_HAS_PDI; }
id               { yylval=NULL; return K_ID; }
idr              { yylval=NULL; return K_IDR; }
jtagmki          { yylval=NULL; return K_JTAG_MKI; }
jtagmkii         { yylval=NULL; return K_JTAG_MKII; }
jtagmkii_dw      { yylval=NULL; return K_JTAG_MKII_DW; }
jtagmkii_isp     { yylval=NULL; return K_JTAG_MKII_ISP; }
max_write_delay  { yylval=NULL; return K_MAX_WRITE_DELAY; }
memory           { yylval=NULL; return K_MEMORY; }
min_write_delay  { yylval=NULL; return K_MIN_WRITE_DELAY; }
miso             { yylval=NULL; return K_MISO; }
mosi             { yylval=NULL; return K_MOSI; }
num_banks        { yylval=NULL; return K_NUM_PAGES; }
num_pages        { yylval=NULL; return K_NUM_PAGES; }
nvm_base         { yylval=NULL; return K_NVM_BASE; }
offset           { yylval=NULL; return K_OFFSET; }
page_size        { yylval=NULL; return K_PAGE_SIZE; }
paged            { yylval=NULL; return K_PAGED; }
pagel            { yylval=NULL; return K_PAGEL; }
par              { yylval=NULL; return K_PAR; }
parallel         { yylval=NULL; return K_PARALLEL; }
part             { yylval=NULL; return K_PART; }
pgmled           { yylval=NULL; return K_PGMLED; }
programmer       { yylval=NULL; return K_PROGRAMMER; }
pwroff_after_write { yylval=NULL; return K_PWROFF_AFTER_WRITE; }
rampz            { yylval=NULL; return K_RAMPZ; }
rdyled           { yylval=NULL; return K_RDYLED; }
readback_p1      { yylval=NULL; return K_READBACK_P1; }
readback_p2      { yylval=NULL; return K_READBACK_P2; }
retry_pulse      { yylval=NULL; return K_RETRY_PULSE; }
serbb            { yylval=NULL; return K_SERBB; }
serial           { yylval=NULL; return K_SERIAL; }
signature        { yylval=NULL; return K_SIGNATURE; }
size             { yylval=NULL; return K_SIZE; }
spmcr            { yylval=NULL; return K_SPMCR; }
stk500           { yylval=NULL; return K_STK500; }
stk500hvsp       { yylval=NULL; return K_STK500HVSP; }
stk500pp         { yylval=NULL; return K_STK500PP; }
stk500v2         { yylval=NULL; return K_STK500V2; }
stk500generic    { yylval=NULL; return K_STK500GENERIC; }
stk500_devcode   { yylval=NULL; return K_STK500_DEVCODE; }
stk600           { yylval=NULL; return K_STK600; }
stk600hvsp       { yylval=NULL; return K_STK600HVSP; }
stk600pp         { yylval=NULL; return K_STK600PP; }
type             { yylval=NULL; return K_TYPE; }
vcc              { yylval=NULL; return K_VCC; }
vfyled           { yylval=NULL; return K_VFYLED; }

timeout          { yylval=NULL; return K_TIMEOUT; }
stabdelay        { yylval=NULL; return K_STABDELAY; }
cmdexedelay      { yylval=NULL; return K_CMDEXEDELAY; }
hvspcmdexedelay  { yylval=NULL; return K_HVSPCMDEXEDELAY; }
synchloops       { yylval=NULL; return K_SYNCHLOOPS; }
bytedelay        { yylval=NULL; return K_BYTEDELAY; }
pollvalue        { yylval=NULL; return K_POLLVALUE; }
pollindex        { yylval=NULL; return K_POLLINDEX; }
predelay         { yylval=NULL; return K_PREDELAY; }
postdelay        { yylval=NULL; return K_POSTDELAY; }
pollmethod       { yylval=NULL; return K_POLLMETHOD; }
mode             { yylval=NULL; return K_MODE; }
delay            { yylval=NULL; return K_DELAY; }
blocksize        { yylval=NULL; return K_BLOCKSIZE; }
readsize        { yylval=NULL; return K_READSIZE; }
pp_controlstack  { yylval=NULL; return K_PP_CONTROLSTACK; }
hvsp_controlstack  { yylval=NULL; return K_HVSP_CONTROLSTACK; }
hventerstabdelay { yylval=NULL; return K_HVENTERSTABDELAY; }
progmodedelay    { yylval=NULL; return K_PROGMODEDELAY; }
latchcycles      { yylval=NULL; return K_LATCHCYCLES; }
togglevtg        { yylval=NULL; return K_TOGGLEVTG; }
poweroffdelay    { yylval=NULL; return K_POWEROFFDELAY; }
resetdelayms     { yylval=NULL; return K_RESETDELAYMS; }
resetdelayus     { yylval=NULL; return K_RESETDELAYUS; }
hvleavestabdelay { yylval=NULL; return K_HVLEAVESTABDELAY; }
resetdelay       { yylval=NULL; return K_RESETDELAY; }
synchcycles      { yylval=NULL; return K_SYNCHCYCLES; }
chiperasepulsewidth { yylval=NULL; return K_CHIPERASEPULSEWIDTH; }
chiperasepolltimeout { yylval=NULL; return K_CHIPERASEPOLLTIMEOUT; }
chiperasetime    { yylval=NULL; return K_CHIPERASETIME; }
programfusepulsewidth { yylval=NULL; return K_PROGRAMFUSEPULSEWIDTH; }
programfusepolltimeout { yylval=NULL; return K_PROGRAMFUSEPOLLTIMEOUT; }
programlockpulsewidth { yylval=NULL; return K_PROGRAMLOCKPULSEWIDTH; }
programlockpolltimeout { yylval=NULL; return K_PROGRAMLOCKPOLLTIMEOUT; }
flash_instr      { yylval=NULL; return K_FLASH_INSTR; }
eeprom_instr     { yylval=NULL; return K_EEPROM_INSTR; }

dedicated        { yylval=new_token(K_DEDICATED); return K_DEDICATED; }
io               { yylval=new_token(K_IO); return K_IO; }
pseudo           { yylval=new_token(K_PSEUDO); return K_PSEUDO; }

reset            { yylval=new_token(K_RESET); return K_RESET; }
sck              { yylval=new_token(K_SCK); return K_SCK; }

read             { yylval=new_token(K_READ); return K_READ; }
write            { yylval=new_token(K_WRITE); return K_WRITE; }
read_lo          { yylval=new_token(K_READ_LO); return K_READ_LO; }
read_hi          { yylval=new_token(K_READ_HI); return K_READ_HI; }
write_lo         { yylval=new_token(K_WRITE_LO); return K_WRITE_LO; }
write_hi         { yylval=new_token(K_WRITE_HI); return K_WRITE_HI; }
loadpage_lo      { yylval=new_token(K_LOADPAGE_LO); return K_LOADPAGE_LO; }
loadpage_hi      { yylval=new_token(K_LOADPAGE_HI); return K_LOADPAGE_HI; }
load_ext_addr    { yylval=new_token(K_LOAD_EXT_ADDR); return K_LOAD_EXT_ADDR; }
writepage        { yylval=new_token(K_WRITEPAGE); return K_WRITEPAGE; }
chip_erase       { yylval=new_token(K_CHIP_ERASE); return K_CHIP_ERASE; }
pgm_enable       { yylval=new_token(K_PGM_ENABLE); return K_PGM_ENABLE; }

no               { yylval=new_token(K_NO); return K_NO; }
yes              { yylval=new_token(K_YES); return K_YES; }

","       { yylval = NULL; pyytext(); return TKN_COMMA; }
"="       { yylval = NULL; pyytext(); return TKN_EQUAL; }
";"       { yylval = NULL; pyytext(); return TKN_SEMI; }
"~"       { yylval = NULL; pyytext(); return TKN_TILDE; }

"\n"      { lineno++; }
[ \r\t]+  { /* ignore whitespace */ }

c: { fprintf(stderr, "error at %s:%d: possible old-style config file entry\n",
             infile, lineno);
     fprintf(stderr, "  Update your config file (see %s%s for a sample)\n",
             CONFIG_DIR, "/avrdude.conf.sample");
     exit(1); }

. { fprintf(stderr, "error at %s:%d unrecognized character: \"%s\"\n", 
            infile, lineno, yytext); exit(1); }

%%

