/*
 * Copyright 2000  Brian S. Dean <bsd@bsdhome.com>
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BRIAN S. DEAN ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BRIAN S. DEAN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * 
 */

/* $Id$ */


#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "avr.h"


extern char * progname;
extern char   progbuf[];


struct command {
  char * name;
  int (*func)(int fd, struct avrpart * p, int argc, char *argv[]);
  char * desc;
};


int cmd_dump  (int fd, struct avrpart * p, int argc, char *argv[]);

int cmd_write (int fd, struct avrpart * p, int argc, char *argv[]);

int cmd_erase (int fd, struct avrpart * p, int argc, char *argv[]);

int cmd_sig   (int fd, struct avrpart * p, int argc, char *argv[]);

int cmd_part  (int fd, struct avrpart * p, int argc, char *argv[]);

int cmd_help  (int fd, struct avrpart * p, int argc, char *argv[]);

int cmd_quit  (int fd, struct avrpart * p, int argc, char *argv[]);


struct command cmd[] = {
  { "dump",  cmd_dump,  "dump memory  : %s [eeprom|flash] <addr> <N-Bytes>" },
  { "write", cmd_write, "write memory : %s [eeprom|flash] <addr> <b1> <b2> ... <bN>" },
  { "erase", cmd_erase, "perform a chip erase" },
  { "sig",   cmd_sig,   "display device signature bytes" },
  { "part",  cmd_part,  "display the current part settings" },
  { "help",  cmd_help,  "help" },
  { "?",     cmd_help,  "help" },
  { "quit",  cmd_quit,  "quit" }
};

#define NCMDS (sizeof(cmd)/sizeof(struct command))





int nexttok ( char * buf, char ** tok, char ** next )
{
  char * q, * n;

  q = buf;
  while (isspace(*q))
    q++;
  
  /* isolate first token */
  n = q+1;
  while (*n && !isspace(*n))
    n++;

  if (*n) {
    *n = 0;
    n++;
  }

  /* find start of next token */
  while (isspace(*n))
    n++;

  *tok  = q;
  *next = n;

  return 0;
}


int hexdump_line ( char * buffer, unsigned char * p, int n, int pad )
{
  char * hexdata = "0123456789abcdef";
  char * b;
  int i, j;

  b = buffer;

  j = 0;
  for (i=0; i<n; i++) {
    if (i && ((i % 8) == 0))
      b[j++] = ' ';
    b[j++] = hexdata[(p[i] & 0xf0) >> 4];
    b[j++] = hexdata[(p[i] & 0x0f)];
    if (i < 15)
      b[j++] = ' ';
  }

  for (i=j; i<pad; i++)
    b[i] = ' ';

  b[i] = 0;

  for (i=0; i<pad; i++) {
    if (!((b[i] == '0') || (b[i] == ' ')))
      return 0;
  }

  return 1;
}


int chardump_line ( char * buffer, unsigned char * p, int n, int pad )
{
  int i;
  char b [ 128 ];

  for (i=0; i<n; i++) {
    memcpy ( b, p, n );
    buffer[i] = '.';
    if (isalpha(b[i]) || isdigit(b[i]) || ispunct(b[i]))
      buffer[i] = b[i];
    else if (isspace(b[i]))
      buffer[i] = ' ';
  }

  for (i=n; i<pad; i++)
    buffer[i] = ' ';

  buffer[i] = 0;

  return 0;
}


int hexdump_buf ( FILE * f, int startaddr, char * buf, int len )
{
  int addr;
  int i, n;
  unsigned char * p;
  char dst1[80];
  char dst2[80];

  addr = startaddr;
  i = 0;
  p = (unsigned char *)buf;
  while (len) {
    n = 16;
    if (n > len)
      n = len;
    hexdump_line(dst1, p, n, 48);
    chardump_line(dst2, p, n, 16);
    fprintf(stdout, "%04x  %s  |%s|\n", addr, dst1, dst2);
    len -= n;
    addr += n;
    p += n;
  }

  return 0;
}


int cmd_dump ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  char * e;
  int i, j, l;
  unsigned short daddr;
  char * buf;
  int maxsize;
  static AVRMEM memtype=AVR_FLASH;
  static unsigned short addr=0;
  static int len=64;

  if (argc == 1) {
    addr += len;
  }
  else {
    if (argc != 4) {
      fprintf(stderr, "Usage: dump flash|eeprom <addr> <len>\n");
      return -1;
    }

    l = strlen(argv[1]);
    if (strncasecmp(argv[1],"flash",l)==0) {
      memtype = AVR_FLASH;
    }
    else if (strncasecmp(argv[1],"eeprom",l)==0) {
      memtype = AVR_EEPROM;
    }
    else {
      fprintf(stderr, "%s (dump): invalid memory type \"%s\"\n",
              progname, argv[1]);
      return -1;
    }

    addr = strtoul(argv[2], &e, 0);
    if (*e || (e == argv[2])) {
      fprintf(stderr, "%s (dump): can't parse address \"%s\"\n",
              progname, argv[2]);
      return -1;
    }

    len = strtol(argv[3], &e, 0);
    if (*e || (e == argv[3])) {
      fprintf(stderr, "%s (dump): can't parse length \"%s\"\n",
              progname, argv[3]);
      return -1;
    }
  }

  switch (memtype) {
    case AVR_FLASH  : maxsize = p->flash_size; break;
    case AVR_EEPROM : maxsize = p->eeprom_size; break;
    default : return -1; /* this can't happen, but is silences gcc
                            warnings */
  }

  if (addr > maxsize) {
    fprintf(stderr, 
            "%s (dump): address 0x%04x is out of range for %s memory\n",
            progname, addr, avr_memtstr(memtype));
    return -1;
  }

  /* trim len if nessary to not read past the end of memory */
  if ((addr + len) > maxsize)
    len = maxsize - addr;

  buf = malloc(len);
  if (buf == NULL) {
    fprintf(stderr, "%s (dump): out of memory\n", progname);
    return -1;
  }

  j     = 0;
  daddr = addr;
  if (memtype == AVR_FLASH) {
    daddr = addr / 2;
    if (addr & 0x01) {
      buf[j++] = avr_read_byte( fd, p, AVR_FLASH_HI, daddr);
      daddr++;
    }
  }

  i = daddr;
  while (j < len) {
    if (memtype == AVR_FLASH) {
      buf[j++] = avr_read_byte( fd, p, AVR_FLASH_LO, i);
      if (j < len) {
        buf[j++] = avr_read_byte( fd, p, AVR_FLASH_HI, i);
      }
    }
    else {
      buf[j++] = avr_read_byte( fd, p, AVR_EEPROM, i);
    }
    i++;
  }

  hexdump_buf(stdout, addr, buf, len);

  fprintf(stdout, "\n");

  free(buf);

  return 0;
}

int cmd_write ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  char * e;
  int i, j, l;
  int len, maxsize;
  AVRMEM memtype;
  unsigned short addr, daddr;
  char * buf;
  int rc;

  if (argc < 4) {
    fprintf(stderr, 
            "Usage: write flash|eeprom <addr> <byte1> <byte2> ... byteN>\n");
    return -1;
  }

  l = strlen(argv[1]);
  if (strncasecmp(argv[1],"flash",l)==0) {
    memtype = AVR_FLASH;
    maxsize = p->flash_size;
  }
  else if (strncasecmp(argv[1],"eeprom",l)==0) {
    memtype = AVR_EEPROM;
    maxsize = p->eeprom_size;
  }
  else {
    fprintf(stderr, "%s (write): invalid memory type \"%s\"\n",
            progname, argv[1]);
    return -1;
  }

  addr = strtoul(argv[2], &e, 0);
  if (*e || (e == argv[2])) {
    fprintf(stderr, "%s (write): can't parse address \"%s\"\n",
            progname, argv[2]);
    return -1;
  }

  if (addr > maxsize) {
    fprintf(stderr, 
            "%s (write): address 0x%04x is out of range for %s memory\n",
            progname, addr, avr_memtstr(memtype));
    return -1;
  }

  /* number of bytes to write at the specified address */
  len = argc - 3;

  if ((addr + len) > maxsize) {
    fprintf(stderr, 
            "%s (write): selected address and # bytes exceed "
            "range for %s memory\n", 
            progname, avr_memtstr(memtype));
    return -1;
  }

  buf = malloc(len);
  if (buf == NULL) {
    fprintf(stderr, "%s (write): out of memory\n", progname);
    return -1;
  }

  for (i=3; i<argc; i++) {
    buf[i-3] = strtoul(argv[i], &e, 0);
    if (*e || (e == argv[i])) {
      fprintf(stderr, "%s (write): can't parse byte \"%s\"\n",
              progname, argv[i]);
      return -1;
    }
  }

  j     = 0;
  daddr = addr;
  if (memtype == AVR_FLASH) {
    daddr = addr / 2;
    if (addr & 0x01) {
      /* handle odd numbered memory locations in the flash area */
      rc = avr_write_byte(fd, p, AVR_FLASH_HI, daddr, buf[j++]);
      if (rc) {
        fprintf(stderr, "%s (write): error writing 0x%02x at 0x%04x\n",
                progname, buf[j-1], daddr*2+1);
      }
      daddr++;
    }
  }

  i = daddr;
  while (j < len) {
    if (memtype == AVR_FLASH) {
      rc = avr_write_byte( fd, p, AVR_FLASH_LO, i, buf[j++]);
      if (rc) {
        fprintf(stderr, "%s (write): error writing 0x%02x at 0x%04x\n",
                progname, buf[j-1], i*2);
      }
      if (j < len) {
        rc = avr_write_byte( fd, p, AVR_FLASH_HI, i, buf[j++]);
        if (rc) {
          fprintf(stderr, "%s (write): error writing 0x%02x at 0x%04x\n",
                  progname, buf[j-1], i*2+1);
        }
      }
    }
    else {
      rc = avr_write_byte( fd, p, AVR_EEPROM, i, buf[j++]);
      if (rc) {
        fprintf(stderr, "%s (write): error writing 0x%02x at 0x%04x\n",
                progname, buf[j-1], i);
      }
    }
    i++;
  }

  free(buf);

  fprintf(stdout, "\n");

  return 0;
}


int cmd_erase ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  fprintf(stderr, "%s: erasing chip\n", progname );
  avr_chip_erase(fd,p);
  return 0;
}


int cmd_part ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  fprintf(stdout, "\n");
  avr_display(stdout, p, "");
  fprintf(stdout, "\n");

  return 0;
}


int cmd_sig ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  unsigned char sig[4];      /* AVR signature bytes */
  int i;

  avr_signature(fd, sig);
  fprintf(stdout, "\nDevice signature = 0x");
  for (i=0; i<4; i++)
    fprintf(stdout, "%02x", sig[i]);
  fprintf(stdout, "\n\n");

  return 0;
}


int cmd_quit ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  return 1;
}


int cmd_help ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  int i;

  fprintf(stdout, "Valid commands:\n\n" );
  for (i=0; i<NCMDS; i++) {
    fprintf(stdout, "  %-6s : ", cmd[i].name );
    fprintf(stdout, cmd[i].desc, cmd[i].name);
    fprintf(stdout, "\n");
  }
  fprintf(stdout, "\n");

  return 0;
}


int tokenize ( char * s, char *** argv )
{
  int     i, n, l, nargs, offset;
  int     len, slen;
  char  * buf;
  int     bufsize;
  char ** bufv;
  char  * q, * r;
  char  * nbuf;
  char ** av;

  slen = strlen(s);

  /* 
   * initialize allow for 20 arguments, use realloc to grow this if
   * necessary 
   */
  nargs   = 20;
  bufsize = slen + 20;
  buf     = malloc(bufsize);
  bufv    = (char **) malloc(nargs*sizeof(char *));
  for (i=0; i<nargs; i++) {
    bufv[i] = NULL;
  }
  buf[0] = 0;

  n    = 0;
  l    = 0;
  nbuf = buf;
  r    = s;
  while (*r) {
    nexttok(r, &q, &r);
    strcpy(nbuf, q);
    bufv[n]  = nbuf;
    len      = strlen(q);
    l       += len + 1;
    nbuf    += len + 1;
    nbuf[0]  = 0;
    n++;
    if ((n % 20) == 0) {
      /* realloc space for another 20 args */
      bufsize += 20;
      nargs   += 20;
      buf      = realloc(buf, bufsize);
      bufv     = realloc(bufv, nargs*sizeof(char *));
      nbuf     = &buf[l];
      for (i=n; i<nargs; i++)
        bufv[i] = NULL;
    }
  }

  /* 
   * We have parsed all the args, n == argc, bufv contains an array of
   * pointers to each arg, and buf points to one memory block that
   * contains all the args, back to back, seperated by a nul
   * terminator.  Consilidate bufv and buf into one big memory block
   * so that the code that calls us, will have an easy job of freeing
   * this memory.
   */
  av = (char **) malloc(slen + n + (n+1)*sizeof(char *));
  q  = (char *)&av[n+1];
  memcpy(q, buf, l);
  for (i=0; i<n; i++) {
    offset = bufv[i] - buf;
    av[i] = q + offset;
  }
  av[i] = NULL;

  free(buf);
  free(bufv);

  *argv = av;

  return n;
}


int do_cmd ( int fd, struct avrpart * p, int argc, char * argv[] )
{
  int i;
  int hold;
  int len;

  len = strlen(argv[0]);
  hold = -1;
  for (i=0; i<NCMDS; i++) {
    if (strcasecmp(argv[0], cmd[i].name) == 0) {
      return cmd[i].func(fd, p, argc, argv);
    }
    else if (strncasecmp(argv[0], cmd[i].name, len)==0) {
      if (hold != -1) {
        fprintf(stderr, "%s: command \"%s\" is ambiguous\n",
                progname, argv[0]);
        return -1;
      }
      hold = i;
    }
  }

  if (hold != -1)
    return cmd[hold].func(fd, p, argc, argv);

  fprintf(stderr, "%s: invalid command \"%s\"\n",
          progname, argv[0]);

  return -1;
}


int terminal_mode ( int fd, struct avrpart * p )
{
  char  * cmdbuf;
  int     i, len;
  char  * q;
  int     rc;
  int     argc;
  char ** argv;

  rc = 0;
  while ((cmdbuf = readline("avrprog> ")) != NULL) {
    len = strlen(cmdbuf);
    if (len >= 1)
      add_history(cmdbuf);

    /* 
     * find the start of the command, skipping any white space
     */
    q = cmdbuf;
    while (*q && isspace(*q))
      q++;

    /* skip blank lines and comments */
    if (!*q || (*q == '#'))
      continue;

    /* tokenize command line */
    argc = tokenize(q, &argv);

    fprintf(stdout, ">>> ");
    for (i=0; i<argc; i++)
      fprintf(stdout, "%s ", argv[i]);
    fprintf(stdout, "\n");

    /* run the command */
    rc = do_cmd(fd, p, argc, argv);
    free(argv);
    if (rc > 0) {
      rc = 0;
      break;
    }
    free(cmdbuf);
  }

  return rc;
}


