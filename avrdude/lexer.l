/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
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

#include "config.h"
#include "config_gram.h"
#include "lists.h"

extern int    lineno;
extern char * infile;

void pyytext(void);

#define YY_NO_UNPUT

%}

DIGIT    [0-9]
HEXDIGIT [0-9a-fA-F]
ID       [_a-zA-Z][_a-zA-Z0-9]*
SIGN     [+-]

%x str
%x incl
%x comment

%%

{SIGN}*{DIGIT}+            { yylval = number(yytext); return TKN_NUMBER; }
{SIGN}*{DIGIT}+"."{DIGIT}* { yylval = number(yytext); return TKN_NUMBER; }
{SIGN}*"."{DIGIT}*         { yylval = number(yytext); return TKN_NUMBER; }

"\""      { string_buf_ptr = string_buf; BEGIN(str); }

0x{HEXDIGIT}+ { yylval = hexnumber(yytext); return TKN_NUMBER; }



#   { /* The following eats '#' style comments to end of line */
       BEGIN(comment); }
<comment>[^\n] /* eat comments */ 
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


<str>{
  \" { *string_buf_ptr = 0; string_buf_ptr = string_buf; 
         yylval = string(string_buf_ptr); BEGIN(INITIAL); return TKN_STRING; }
  \\n  *string_buf_ptr++ = '\n';
  \\t  *string_buf_ptr++ = '\t';
  \\r  *string_buf_ptr++ = '\r';
  \\b  *string_buf_ptr++ = '\b';
  \\f  *string_buf_ptr++ = '\f';
  \\(.|\n)  *(string_buf_ptr++) = yytext[1];
  [^\\\n\"]+ { char *yptr = yytext; while (*yptr) 
                                         *(string_buf_ptr++) = *(yptr++); }
  \n { fprintf(stderr, "error at line %d: unterminated character constant\n",
         lineno); 
         exit(1); }
}

bank_size        { yylval=NULL; return K_PAGE_SIZE; }
banked           { yylval=NULL; return K_PAGED; }
baudrate         { yylval=NULL; return K_BAUDRATE; }
bs2              { yylval=NULL; return K_BS2; }
buff             { yylval=NULL; return K_BUFF; }
chip_erase_delay { yylval=NULL; return K_CHIP_ERASE_DELAY; }
desc             { yylval=NULL; return K_DESC; }
default_parallel { yylval=NULL; return K_DEFAULT_PARALLEL; }
default_programmer { yylval=NULL; return K_DEFAULT_PROGRAMMER; }
default_serial   { yylval=NULL; return K_DEFAULT_SERIAL; }
devicecode       { yylval=NULL; return K_DEVICECODE; }
stk500_devcode   { yylval=NULL; return K_STK500_DEVCODE; }
avr910_devcode   { yylval=NULL; return K_AVR910_DEVCODE; }
eeprom           { yylval=NULL; return K_EEPROM; }
errled           { yylval=NULL; return K_ERRLED; }
flash            { yylval=NULL; return K_FLASH; }
id               { yylval=NULL; return K_ID; }
max_write_delay  { yylval=NULL; return K_MAX_WRITE_DELAY; }
memory           { yylval=NULL; return K_MEMORY; }
min_write_delay  { yylval=NULL; return K_MIN_WRITE_DELAY; }
miso             { yylval=NULL; return K_MISO; }
mosi             { yylval=NULL; return K_MOSI; }
num_banks        { yylval=NULL; return K_NUM_PAGES; }
num_pages        { yylval=NULL; return K_NUM_PAGES; }
page_size        { yylval=NULL; return K_PAGE_SIZE; }
paged            { yylval=NULL; return K_PAGED; }
pagel            { yylval=NULL; return K_PAGEL; }
par              { yylval=NULL; return K_PAR; }
parallel         { yylval=NULL; return K_PARALLEL; }
part             { yylval=NULL; return K_PART; }
pgmled           { yylval=NULL; return K_PGMLED; }
programmer       { yylval=NULL; return K_PROGRAMMER; }
pwroff_after_write { yylval=NULL; return K_PWROFF_AFTER_WRITE; }
rdyled           { yylval=NULL; return K_RDYLED; }
readback_p1      { yylval=NULL; return K_READBACK_P1; }
readback_p2      { yylval=NULL; return K_READBACK_P2; }
retry_pulse      { yylval=NULL; return K_RETRY_PULSE; }
serial           { yylval=NULL; return K_SERIAL; }
size             { yylval=NULL; return K_SIZE; }
stk500           { yylval=NULL; return K_STK500; }
avr910           { yylval=NULL; return K_AVR910; }
butterfly        { yylval=NULL; return K_BUTTERFLY; }
type             { yylval=NULL; return K_TYPE; }
vcc              { yylval=NULL; return K_VCC; }
vfyled           { yylval=NULL; return K_VFYLED; }


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
writepage        { yylval=new_token(K_WRITEPAGE); return K_WRITEPAGE; }
chip_erase       { yylval=new_token(K_CHIP_ERASE); return K_CHIP_ERASE; }
pgm_enable       { yylval=new_token(K_PGM_ENABLE); return K_PGM_ENABLE; }

no               { yylval=new_token(K_NO); return K_NO; }
yes              { yylval=new_token(K_YES); return K_YES; }

","       { yylval = NULL; pyytext(); return TKN_COMMA; }
"="       { yylval = NULL; pyytext(); return TKN_EQUAL; }
";"       { yylval = NULL; pyytext(); return TKN_SEMI; }

"\n"      { lineno++; }
[ \r\t]+  /* ignore whitespace */

c: { fprintf(stderr, "error at %s:%d: possible old-style config file entry\n",
             infile, lineno);
     fprintf(stderr, "  Update your config file (see %s%s for a sample)\n",
             CONFIG_DIR, "/avrdude.conf.sample");
     exit(1); }

. { fprintf(stderr, "error at %s:%d unrecognized character: \"%s\"\n", 
            infile, lineno, yytext); exit(1); }

%%

