/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000, 2001, 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "avr.h"
#include "config.h"
#include "lists.h"
#include "pgm.h"
#include "pindefs.h"
#include "ppi.h"


extern char       * progname;
extern char         progbuf[];
extern PROGRAMMER * pgm;


struct command {
  char * name;
  int (*func)(PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);
  char * desc;
};


int cmd_dump  (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);

int cmd_write (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);

int cmd_erase (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);

int cmd_sig   (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);

int cmd_part  (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);

int cmd_help  (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);

int cmd_quit  (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);

int cmd_send  (PROGRAMMER * pgm, struct avrpart * p, int argc, char *argv[]);


struct command cmd[] = {
  { "dump",  cmd_dump,  "dump memory  : %s <memtype> <addr> <N-Bytes>" },
  { "read",  cmd_dump,  "alias for dump" },
  { "write", cmd_write, "write memory : %s <memtype> <addr> <b1> <b2> ... <bN>" },
  { "erase", cmd_erase, "perform a chip erase" },
  { "sig",   cmd_sig,   "display device signature bytes" },
  { "part",  cmd_part,  "display the current part information" },
  { "send",  cmd_send,  "send a raw command : %s <b1> <b2> <b3> <b4>" },
  { "help",  cmd_help,  "help" },
  { "?",     cmd_help,  "help" },
  { "quit",  cmd_quit,  "quit" }
};

#define NCMDS (sizeof(cmd)/sizeof(struct command))





int nexttok(char * buf, char ** tok, char ** next)
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


int hexdump_line(char * buffer, unsigned char * p, int n, int pad)
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


int chardump_line(char * buffer, unsigned char * p, int n, int pad)
{
  int i;
  char b [ 128 ];

  for (i=0; i<n; i++) {
    memcpy(b, p, n);
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


int hexdump_buf(FILE * f, int startaddr, unsigned char * buf, int len)
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


int cmd_dump(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  static char prevmem[128] = {0};
  char * e;
  unsigned char * buf;
  int maxsize;
  unsigned long i;
  static unsigned long addr=0;
  static int len=64;
  AVRMEM * mem;
  char * memtype = NULL;
  int rc;

  if (!((argc == 2) || (argc == 4))) {
    fprintf(stderr, "Usage: dump <memtype> [<addr> <len>]\n");
    return -1;
  }

  memtype = argv[1];

  if (strncmp(prevmem, memtype, strlen(memtype)) != 0) {
    addr = 0;
    len  = 64;
    strncpy(prevmem, memtype, sizeof(prevmem)-1);
    prevmem[sizeof(prevmem)-1] = 0;
  }

  mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    fprintf(stderr, "\"%s\" memory type not defined for part \"%s\"\n",
            memtype, p->desc);
    return -1;
  }

  if (argc == 4) {
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

  maxsize = mem->size;

  if (addr >= maxsize) {
    if (argc == 2) {
      /* wrap around */
      addr = 0;
    }
    else {
      fprintf(stderr, 
              "%s (dump): address 0x%05lx is out of range for %s memory\n",
              progname, addr, mem->desc);
      return -1;
    }
  }

  /* trim len if nessary to not read past the end of memory */
  if ((addr + len) > maxsize)
    len = maxsize - addr;

  buf = malloc(len);
  if (buf == NULL) {
    fprintf(stderr, "%s (dump): out of memory\n", progname);
    return -1;
  }

  for (i=0; i<len; i++) {
    rc = avr_read_byte(pgm, p, mem, addr+i, &buf[i]);
    if (rc != 0) {
      fprintf(stderr, "error reading %s address 0x%05lx of part %s\n",
              mem->desc, addr+i, p->desc);
      if (rc == -1)
        fprintf(stderr, "read operation not supported on memory type \"%s\"\n",
                mem->desc);
      return -1;
    }
  }

  hexdump_buf(stdout, addr, buf, len);

  fprintf(stdout, "\n");

  free(buf);

  addr = addr + len;

  return 0;
}


int cmd_write(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  char * e;
  int len, maxsize;
  char * memtype;
  unsigned long addr, i;
  unsigned char * buf;
  unsigned char b;
  int rc;
  int werror;
  AVRMEM * mem;

  if (argc < 4) {
    fprintf(stderr, "Usage: write <memtype> <addr> <byte1> "
            "<byte2> ... byteN>\n");
    return -1;
  }

  memtype = argv[1];

  mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    fprintf(stderr, "\"%s\" memory type not defined for part \"%s\"\n",
            memtype, p->desc);
    return -1;
  }

  maxsize = mem->size;

  addr = strtoul(argv[2], &e, 0);
  if (*e || (e == argv[2])) {
    fprintf(stderr, "%s (write): can't parse address \"%s\"\n",
            progname, argv[2]);
    return -1;
  }

  if (addr > maxsize) {
    fprintf(stderr, 
            "%s (write): address 0x%05lx is out of range for %s memory\n",
            progname, addr, memtype);
    return -1;
  }

  /* number of bytes to write at the specified address */
  len = argc - 3;

  if ((addr + len) > maxsize) {
    fprintf(stderr, 
            "%s (write): selected address and # bytes exceed "
            "range for %s memory\n", 
            progname, memtype);
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
      free(buf);
      return -1;
    }
  }

  pgm->err_led(pgm, OFF);
  for (werror=0, i=0; i<len; i++) {

    rc = avr_write_byte(pgm, p, mem, addr+i, buf[i]);
    if (rc) {
      fprintf(stderr, "%s (write): error writing 0x%02x at 0x%05lx, rc=%d\n",
              progname, buf[i], addr+i, rc);
      if (rc == -1)
        fprintf(stderr, 
                "write operation not supported on memory type \"%s\"\n",
                mem->desc);
      werror = 1;
    }

    rc = avr_read_byte(pgm, p, mem, addr+i, &b);
    if (b != buf[i]) {
      fprintf(stderr, 
              "%s (write): error writing 0x%02x at 0x%05lx cell=0x%02x\n",
              progname, buf[i], addr+i, b);
      werror = 1;
    }

    if (werror) {
      pgm->err_led(pgm, ON);
    }
  }

  free(buf);

  fprintf(stdout, "\n");

  return 0;
}


int cmd_send(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  unsigned char cmd[4], res[4];
  char * e;
  int i;
  int len;

  if (argc != 5) {
    fprintf(stderr, "Usage: send <byte1> <byte2> <byte3> <byte4>\n");
    return -1;
  }

  /* number of bytes to write at the specified address */
  len = argc - 1;

  /* load command bytes */
  for (i=1; i<argc; i++) {
    cmd[i-1] = strtoul(argv[i], &e, 0);
    if (*e || (e == argv[i])) {
      fprintf(stderr, "%s (send): can't parse byte \"%s\"\n",
              progname, argv[i]);
      return -1;
    }
  }

  pgm->err_led(pgm, OFF);

  pgm->cmd(pgm, cmd, res);

  /*
   * display results
   */
  fprintf(stderr, "results:");
  for (i=0; i<len; i++)
    fprintf(stderr, " %02x", res[i]);
  fprintf(stderr, "\n");

  fprintf(stdout, "\n");

  return 0;
}


int cmd_erase(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  fprintf(stderr, "%s: erasing chip\n", progname);
  pgm->chip_erase(pgm, p);
  return 0;
}


int cmd_part(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  fprintf(stdout, "\n");
  avr_display(stdout, p, "", 0);
  fprintf(stdout, "\n");

  return 0;
}


int cmd_sig(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  int i;
  int rc;
  AVRMEM * m;

  rc = avr_signature(pgm, p);
  if (rc != 0) {
    fprintf(stderr, "error reading signature data, rc=%d\n",
            rc);
  }

  m = avr_locate_mem(p, "signature");
  if (m == NULL) {
    fprintf(stderr,
            "signature data not defined for device \"%s\"\n",
            p->desc);
  }
  else {
    fprintf(stdout, "Device signature = 0x");
    for (i=0; i<m->size; i++)
      fprintf(stdout, "%02x", m->buf[i]);
    fprintf(stdout, "\n\n");
  }

  return 0;
}


int cmd_quit(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  return 1;
}


int cmd_help(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  int i;

  fprintf(stdout, "Valid commands:\n\n");
  for (i=0; i<NCMDS; i++) {
    fprintf(stdout, "  %-6s : ", cmd[i].name);
    fprintf(stdout, cmd[i].desc, cmd[i].name);
    fprintf(stdout, "\n");
  }
  fprintf(stdout, 
          "\nUse the 'part' command to display valid memory types for use with the\n"
          "'dump' and 'write' commands.\n\n");

  return 0;
}


int tokenize(char * s, char *** argv)
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


int do_cmd(PROGRAMMER * pgm, struct avrpart * p, int argc, char * argv[])
{
  int i;
  int hold;
  int len;

  len = strlen(argv[0]);
  hold = -1;
  for (i=0; i<NCMDS; i++) {
    if (strcasecmp(argv[0], cmd[i].name) == 0) {
      return cmd[i].func(pgm, p, argc, argv);
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
    return cmd[hold].func(pgm, p, argc, argv);

  fprintf(stderr, "%s: invalid command \"%s\"\n",
          progname, argv[0]);

  return -1;
}


int terminal_mode(PROGRAMMER * pgm, struct avrpart * p)
{
  char  * cmdbuf;
  int     i, len;
  char  * q;
  int     rc;
  int     argc;
  char ** argv;

  rc = 0;
  while ((cmdbuf = readline("avrdude> ")) != NULL) {
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
    rc = do_cmd(pgm, p, argc, argv);
    free(argv);
    if (rc > 0) {
      rc = 0;
      break;
    }
    free(cmdbuf);
  }

  return rc;
}


