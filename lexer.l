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
#include "y.tab.h"
#include "lists.h"

extern int    lineno;
extern char * infile;

void pyytext(void);

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

bank_size        { yylval=NULL; return K_BANK_SIZE; }
banked           { yylval=NULL; return K_BANKED; }
buff             { yylval=NULL; return K_BUFF; }
chip_erase_delay { yylval=NULL; return K_CHIP_ERASE_DELAY; }
desc             { yylval=NULL; return K_DESC; }
eeprom           { yylval=NULL; return K_EEPROM; }
errled           { yylval=NULL; return K_ERRLED; }
flash            { yylval=NULL; return K_FLASH; }
id               { yylval=NULL; return K_ID; }
max_write_delay  { yylval=NULL; return K_MAX_WRITE_DELAY; }
min_write_delay  { yylval=NULL; return K_MIN_WRITE_DELAY; }
miso             { yylval=NULL; return K_MISO; }
mosi             { yylval=NULL; return K_MOSI; }
num_banks        { yylval=NULL; return K_NUM_BANKS; }
part             { yylval=NULL; return K_PART; }
pgmled           { yylval=NULL; return K_PGMLED; }
programmer       { yylval=NULL; return K_PROGRAMMER; }
rdyled           { yylval=NULL; return K_RDYLED; }
readback_p1      { yylval=NULL; return K_READBACK_P1; }
readback_p2      { yylval=NULL; return K_READBACK_P2; }
reset            { yylval=NULL; return K_RESET; }
sck              { yylval=NULL; return K_SCK; }
size             { yylval=NULL; return K_SIZE; }
vcc              { yylval=NULL; return K_VCC; }
vfyled           { yylval=NULL; return K_VFYLED; }

no               { yylval=new_token(K_NO); return K_NO; }
yes              { yylval=new_token(K_YES); return K_YES; }

","       { yylval = NULL; pyytext(); return TKN_COMMA; }
"="       { yylval = NULL; pyytext(); return TKN_EQUAL; }
";"       { yylval = NULL; pyytext(); return TKN_SEMI; }

"\n"      { lineno++; }
[ \t]+  /* ignore whitespace */

. { fprintf(stderr, "error at %s:%d unrecognized character: \"%s\"\n", 
            infile, lineno, yytext); exit(1); }

%%

