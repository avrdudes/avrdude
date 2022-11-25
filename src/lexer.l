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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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

#include "ac_cfg.h"
#include "avrdude.h"
#include "libavrdude.h"
#include "config.h"

#include "config_gram.h"

#ifndef YYERRCODE
#define YYERRCODE 256
#endif

/* capture lvalue keywords to associate comments with that assignment */
#define ccap() capture_lvalue_kw(yytext, cfg_lineno)

static void adjust_cfg_lineno(const char *p) {
  while(*p)
    if(*p++ == '\n')
      cfg_lineno++;
}

%}

DIGIT    [0-9]
HEXDIGIT [0-9a-fA-F]
SIGN     [+-]

%option nounput

/* Bump resources for classic lex. */
%e2000
%p10000
%n1000

%%

{DIGIT}+                   { yylval = new_number(yytext); return TKN_NUMBER; /* sign is treated in grammar */ }
{SIGN}?{DIGIT}+"."{DIGIT}* { yylval = new_number_real(yytext); return TKN_NUMBER_REAL; }
{SIGN}?"."{DIGIT}+         { yylval = new_number_real(yytext); return TKN_NUMBER_REAL; }

["]([^"\\\n]|\\.|\\\n)*["] {
  char *str= cfg_strdup("lexer.l", yytext);
  cfg_unescape(str, str+1);
  size_t len = strlen(str);
  if(len)
    str[len-1] = 0;
  yylval = new_string(str);
  free(str);
  return TKN_STRING;
}

0x{HEXDIGIT}+ { yylval = new_hexnumber(yytext); return TKN_NUMBER; }

#\n#\ PROGRAMMER\ DEFINITIONS\n#\n+ { /* Record comments so far as prologue and skip */
  cfg_capture_prologue();
  adjust_cfg_lineno(yytext);
}

#\n#\ PART\ DEFINITIONS\n#\n+ { /* Ignore part definions header */
  adjust_cfg_lineno(yytext);
}

[ \t]*#[^\n]*\n+ { /* Record and skip # comments including preceding white space */
  capture_comment_str(yytext, cfg_lineno);
  adjust_cfg_lineno(yytext);
}

"/*" {  /* The following eats multiline C style comments, they are not captured */
        int c;
        int comment_start;
        
        comment_start = cfg_lineno;
        while (1) {
          while (((c = input()) != '*') && (c != EOF)) {
            /* eat up text of comment, but keep counting lines */
            if (c == '\n')
              cfg_lineno++;
          }
          
          if (c == '*') {
            while ((c = input()) == '*')
              ;
            if (c == '/')
              break;    /* found the end */
          }
          
          if (c == EOF) {
            yyerror("EOF in comment (started on line %d)", comment_start);
            return YYERRCODE;
          }
        }
     }


(?x: prog_modes | mcuid | n_interrupts | n_page_erase | n_word_writes | n_boot_sections |
  boot_section_size | autobaud_sync) { /* Components for assignment  */

  Component_t *cp = cfg_comp_search(yytext, current_strct);
  if(!cp) {
    yyerror("Unknown component %s in %s", yytext, cfg_strct_name(current_strct));
    return YYERRCODE;
  }
  yylval = new_token(TKN_COMPONENT);
  yylval->value.comp = cp;
  ccap();
  return TKN_COMPONENT;
}

PM_(SPM|TPI|ISP|PDI|UPDI|HVSP|HVPP|debugWIRE|JTAG|JTAGmkI|XMEGAJTAG|AVR32JTAG|aWire) { /* Constants */
  yylval = new_constant(yytext);
  return TKN_NUMBER;
}

alias            { yylval=NULL; return K_ALIAS; }
allowfullpagebitstream { yylval=NULL; ccap(); return K_ALLOWFULLPAGEBITSTREAM; }
avr910_devcode   { yylval=NULL; ccap(); return K_AVR910_DEVCODE; }
bank_size        { yylval=NULL; return K_PAGE_SIZE; }
banked           { yylval=NULL; return K_PAGED; }
baudrate         { yylval=NULL; ccap(); return K_BAUDRATE; }
blocksize        { yylval=NULL; ccap(); return K_BLOCKSIZE; }
bs2              { yylval=NULL; ccap(); return K_BS2; }
buff             { yylval=NULL; ccap(); return K_BUFF; }
bytedelay        { yylval=NULL; ccap(); return K_BYTEDELAY; }
chip_erase       { yylval=new_token(K_CHIP_ERASE); ccap(); return K_CHIP_ERASE; }
chip_erase_delay { yylval=NULL; ccap(); return K_CHIP_ERASE_DELAY; }
chiperasepolltimeout { yylval=NULL; ccap(); return K_CHIPERASEPOLLTIMEOUT; }
chiperasepulsewidth { yylval=NULL; ccap(); return K_CHIPERASEPULSEWIDTH; }
chiperasetime    { yylval=NULL; ccap(); return K_CHIPERASETIME; }
cmdexedelay      { yylval=NULL; ccap(); return K_CMDEXEDELAY; }
connection_type  { yylval=NULL; ccap(); return K_CONNTYPE; }
dedicated        { yylval=new_token(K_DEDICATED); return K_DEDICATED; }
default_bitclock { yylval=NULL; return K_DEFAULT_BITCLOCK; }
default_parallel { yylval=NULL; return K_DEFAULT_PARALLEL; }
default_programmer { yylval=NULL; return K_DEFAULT_PROGRAMMER; }
default_serial   { yylval=NULL; return K_DEFAULT_SERIAL; }
default_spi      { yylval=NULL; return K_DEFAULT_SPI; }
delay            { yylval=NULL; ccap(); return K_DELAY; }
desc             { yylval=NULL; ccap(); return K_DESC; }
devicecode       { yylval=NULL; ccap(); return K_DEVICECODE; }
eecr             { yylval=NULL; ccap(); return K_EECR; }
eeprom           { yylval=NULL; return K_EEPROM; }
eeprom_instr     { yylval=NULL; ccap(); return K_EEPROM_INSTR; }
enablepageprogramming { yylval=NULL; ccap(); return K_ENABLEPAGEPROGRAMMING; }
errled           { yylval=NULL; ccap(); return K_ERRLED; }
family_id        { yylval=NULL; ccap(); return K_FAMILY_ID; }
flash            { yylval=NULL; return K_FLASH; }
flash_instr      { yylval=NULL; ccap(); return K_FLASH_INSTR; }
has_debugwire    { yylval=NULL; ccap(); return K_HAS_DW; }
has_jtag         { yylval=NULL; ccap(); return K_HAS_JTAG; }
has_pdi          { yylval=NULL; ccap(); return K_HAS_PDI; }
has_tpi          { yylval=NULL; ccap(); return K_HAS_TPI; }
has_updi         { yylval=NULL; ccap(); return K_HAS_UPDI; }
hventerstabdelay { yylval=NULL; ccap(); return K_HVENTERSTABDELAY; }
hvleavestabdelay { yylval=NULL; ccap(); return K_HVLEAVESTABDELAY; }
hvsp_controlstack  { yylval=NULL; ccap(); return K_HVSP_CONTROLSTACK; }
hvspcmdexedelay  { yylval=NULL; ccap(); return K_HVSPCMDEXEDELAY; }
hvupdi_support   { yylval=NULL; ccap(); return K_HVUPDI_SUPPORT; }
hvupdi_variant   { yylval=NULL; ccap(); return K_HVUPDI_VARIANT; }
id               { yylval=NULL; ccap(); return K_ID; }
idr              { yylval=NULL; ccap(); return K_IDR; }
io               { yylval=new_token(K_IO); return K_IO; }
is_at90s1200     { yylval=NULL; ccap(); return K_IS_AT90S1200; }
is_avr32         { yylval=NULL; ccap(); return K_IS_AVR32; }
latchcycles      { yylval=NULL; ccap(); return K_LATCHCYCLES; }
load_ext_addr    { yylval=new_token(K_LOAD_EXT_ADDR); ccap(); return K_LOAD_EXT_ADDR; }
loadpage_hi      { yylval=new_token(K_LOADPAGE_HI); ccap(); return K_LOADPAGE_HI; }
loadpage_lo      { yylval=new_token(K_LOADPAGE_LO); ccap(); return K_LOADPAGE_LO; }
max_write_delay  { yylval=NULL; ccap(); return K_MAX_WRITE_DELAY; }
mcu_base         { yylval=NULL; ccap(); return K_MCU_BASE; }
memory           { yylval=NULL; ccap(); return K_MEMORY; }
min_write_delay  { yylval=NULL; ccap(); return K_MIN_WRITE_DELAY; }
miso             { yylval=NULL; ccap(); return K_SDI; } // Deprecated
mode             { yylval=NULL; ccap(); return K_MODE; }
mosi             { yylval=NULL; ccap(); return K_SDO; } // Deprecated
no               { yylval=new_token(K_NO); return K_NO; }
NULL             { yylval=NULL; return K_NULL; }
num_banks        { yylval=NULL; return K_NUM_PAGES; }
num_pages        { yylval=NULL; ccap(); return K_NUM_PAGES; }
nvm_base         { yylval=NULL; ccap(); return K_NVM_BASE; }
ocd_base         { yylval=NULL; ccap(); return K_OCD_BASE; }
ocdrev           { yylval=NULL; ccap(); return K_OCDREV; }
offset           { yylval=NULL; ccap(); return K_OFFSET; }
paged            { yylval=NULL; ccap(); return K_PAGED; }
pagel            { yylval=NULL; ccap(); return K_PAGEL; }
page_size        { yylval=NULL; ccap(); return K_PAGE_SIZE; }
parallel         { yylval=NULL; ccap(); return K_PARALLEL; }
parent           { yylval=NULL; return K_PARENT; }
part             { yylval=NULL; ccap(); return K_PART; }
pgm_enable       { yylval=new_token(K_PGM_ENABLE); ccap(); return K_PGM_ENABLE; }
pgmled           { yylval=NULL; ccap(); return K_PGMLED; }
pollindex        { yylval=NULL; ccap(); return K_POLLINDEX; }
pollmethod       { yylval=NULL; ccap(); return K_POLLMETHOD; }
pollvalue        { yylval=NULL; ccap(); return K_POLLVALUE; }
postdelay        { yylval=NULL; ccap(); return K_POSTDELAY; }
poweroffdelay    { yylval=NULL; ccap(); return K_POWEROFFDELAY; }
pp_controlstack  { yylval=NULL; ccap(); return K_PP_CONTROLSTACK; }
predelay         { yylval=NULL; ccap(); return K_PREDELAY; }
progmodedelay    { yylval=NULL; ccap(); return K_PROGMODEDELAY; }
programfusepolltimeout { yylval=NULL; ccap(); return K_PROGRAMFUSEPOLLTIMEOUT; }
programfusepulsewidth { yylval=NULL; ccap(); return K_PROGRAMFUSEPULSEWIDTH; }
programlockpolltimeout { yylval=NULL; ccap(); return K_PROGRAMLOCKPOLLTIMEOUT; }
programlockpulsewidth { yylval=NULL; ccap(); return K_PROGRAMLOCKPULSEWIDTH; }
programmer       { yylval=NULL; ccap(); return K_PROGRAMMER; }
pseudo           { yylval=new_token(K_PSEUDO); return K_PSEUDO; }
pwroff_after_write { yylval=NULL; ccap(); return K_PWROFF_AFTER_WRITE; }
rampz            { yylval=NULL; ccap(); return K_RAMPZ; }
rdyled           { yylval=NULL; ccap(); return K_RDYLED; }
read             { yylval=new_token(K_READ); ccap(); return K_READ; }
read_hi          { yylval=new_token(K_READ_HI); ccap(); return K_READ_HI; }
read_lo          { yylval=new_token(K_READ_LO); ccap(); return K_READ_LO; }
readback         { yylval=NULL; ccap(); return K_READBACK; }
readback_p1      { yylval=NULL; ccap(); return K_READBACK_P1; }
readback_p2      { yylval=NULL; ccap(); return K_READBACK_P2; }
readsize        { yylval=NULL; ccap(); return K_READSIZE; }
reset            { yylval=new_token(K_RESET); ccap(); return K_RESET; }
resetdelay       { yylval=NULL; ccap(); return K_RESETDELAY; }
resetdelayms     { yylval=NULL; ccap(); return K_RESETDELAYMS; }
resetdelayus     { yylval=NULL; ccap(); return K_RESETDELAYUS; }
retry_pulse      { yylval=NULL; ccap(); return K_RETRY_PULSE; }
sck              { yylval=new_token(K_SCK); ccap(); return K_SCK; }
sdi              { yylval=NULL; ccap(); return K_SDI; }
sdo              { yylval=NULL; ccap(); return K_SDO; }
serial           { yylval=NULL; ccap(); return K_SERIAL; }
signature        { yylval=NULL; ccap(); return K_SIGNATURE; }
size             { yylval=NULL; ccap(); return K_SIZE; }
spi              { yylval=NULL; return K_SPI; }
spmcr            { yylval=NULL; ccap(); return K_SPMCR; }
stabdelay        { yylval=NULL; ccap(); return K_STABDELAY; }
stk500_devcode   { yylval=NULL; ccap(); return K_STK500_DEVCODE; }
synchcycles      { yylval=NULL; ccap(); return K_SYNCHCYCLES; }
synchloops       { yylval=NULL; ccap(); return K_SYNCHLOOPS; }
timeout          { yylval=NULL; ccap(); return K_TIMEOUT; }
togglevtg        { yylval=NULL; ccap(); return K_TOGGLEVTG; }
type             { yylval=NULL; ccap(); return K_TYPE; }
usb              { yylval=NULL; return K_USB; }
usbdev           { yylval=NULL; ccap(); return K_USBDEV; }
usbpid           { yylval=NULL; ccap(); return K_USBPID; }
usbproduct       { yylval=NULL; ccap(); return K_USBPRODUCT; }
usbsn            { yylval=NULL; ccap(); return K_USBSN; }
usbvendor        { yylval=NULL; ccap(); return K_USBVENDOR; }
usbvid           { yylval=NULL; ccap(); return K_USBVID; }
vcc              { yylval=NULL; ccap(); return K_VCC; }
vfyled           { yylval=NULL; ccap(); return K_VFYLED; }
write            { yylval=new_token(K_WRITE); ccap(); return K_WRITE; }
write_hi         { yylval=new_token(K_WRITE_HI); ccap(); return K_WRITE_HI; }
write_lo         { yylval=new_token(K_WRITE_LO); ccap(); return K_WRITE_LO; }
writepage        { yylval=new_token(K_WRITEPAGE); ccap(); return K_WRITEPAGE; }
yes              { yylval=new_token(K_YES); return K_YES; }

","       { yylval = NULL; pyytext(); return TKN_COMMA; }
"="       { yylval = NULL; pyytext(); return TKN_EQUAL; }
";"       { yylval = NULL; pyytext(); return TKN_SEMI; }

"|"       { yylval = NULL; pyytext(); return OP_OR; }
"^"       { yylval = NULL; pyytext(); return OP_XOR; }
"&"       { yylval = NULL; pyytext(); return OP_AND; }
"+"       { yylval = NULL; pyytext(); return OP_PLUS; }
"-"       { yylval = NULL; pyytext(); return OP_MINUS; }
"*"       { yylval = NULL; pyytext(); return OP_TIMES; }
"/"       { yylval = NULL; pyytext(); return OP_DIVIDE; }
"%"       { yylval = NULL; pyytext(); return OP_MODULO; }
"~"       { yylval = NULL; pyytext(); return OP_TILDE; }

"("       { yylval = NULL; pyytext(); return TKN_LEFT_PAREN; }
")"       { yylval = NULL; pyytext(); return TKN_RIGHT_PAREN; }

"\n"      { cfg_lineno++; }
[ \r\t]+  { /* ignore whitespace */ }

c: { yyerror("possible old-style config file entry\n"
             "  Update your config file (see " CONFIG_DIR 
               "/avrdude.conf.sample for a sample)");
     return YYERRCODE; }

. { return YYERRCODE; }

%%

