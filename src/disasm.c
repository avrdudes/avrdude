/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2007 Johannes Bauer <JohannesBauer@gmx.de>
 * Copyright (C) 2024 by Stefan Rueger <stefan.rueger@urclocks.com>
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

/*
 * This disassembly code originates from the avrdisas disassembler written in
 * 2007 by Johannes Bauer. It has been rewritten by Stefan Rueger to
 *  - Enable disassembly of small memory chunks in AVRDUDE's terminal
 *  - Drive disassembly from the avr_opcodes[] table alone
 *  - Generate a compilable source
 *  - Find symbolic values for ldi constants that initialise register pais
 *
 * Like the ship of Theseus there is little of the avrdisas orginal code that
 * has remained, but it is fair to say that without it the AVRDUDE disasm
 * command would not have happened.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "avrdude.h"
#include "libavrdude.h"

#define TYPE_BYTE             1 // 1 byte
#define TYPE_WORD             2 // 2 bytes
#define TYPE_ASTRING          3 // Autoaligned string
#define TYPE_STRING           4 // String

#define buf2op16(i) ((buf[i] & 0xff) | (buf[(i)+1] & 0xff)<<8)

static void zap_symbols() {
  if(cx->dis_symbols) {
    for(int i = 0; i < cx->dis_symbolN; i++) {
      mmt_free(cx->dis_symbols[i].comment);
      mmt_free(cx->dis_symbols[i].name);
    }
    mmt_free(cx->dis_symbols);
    cx->dis_symbols = NULL;
  }
  cx->dis_symbolN = 0;
}

static int type_order(int type) {
  switch(type) {
  case 'I':
    return '1';
  case 'M':
    return '2';
  case 'L':
    return '3';
  case 'P':
    return '4';
  default:
    return type;
  }
}

static int symbol_sort(const void *v1, const void *v2) {
  const Dis_symbol *p1 = v1, *p2 = v2;
  int diff;

  if((diff = type_order(p1->type) - type_order(p2->type)))
    return diff;
  return p1->address - p2->address;
}

static int symbol_stable_qsort(const void *v1, const void *v2) {
  int diff = symbol_sort(v1, v2);

  if(diff)
    return diff;
  return (char *) v1 - (char *) v2;     // Keep original order if same keys (stable sort)
}

static char *cleanup(char *str) {
  for(char *s = str; *s; s++)
    *s = *s == '.' || isalnum(*s & 0xff)? *s: '_';
  return str;
}

// Width of memory a symbol covers
static int symbol_width(Dis_symbol *s) {
  return s->count*(s->subtype == TYPE_WORD? 2: 1);
}

static Dis_symbol *find_symbol(int type, int address) {
  Dis_symbol key, *s = cx->dis_symbols, *found;

  key.type = type;
  key.address = address;
  if(!(found = bsearch(&key, s, cx->dis_symbolN, sizeof(Dis_symbol), symbol_sort)))
    return NULL;

  // Determine m as first matching symbol that has smallest width
  int k = found - s, m = k, w, width = symbol_width(s + k);

  for(int i = k - 1; i >= 0 && symbol_sort(s + i, s + k) == 0; i--)
    if((w = symbol_width(s + i)) <= width)      // Want first entry of those with same min width
      m = i, width = w;
  for(int i = k + 1; i < cx->dis_symbolN && symbol_sort(s + i, s + k) == 0; i++)
    if((w = symbol_width(s + i)) < width)       // < is deliberate, see above
      m = i, width = w;

  return s + m;
}

static void add_symbol(int addr, int type, int sub, int count, const char *name, const char *com) {
  int N = cx->dis_symbolN++;

  if(N%1024 == 0)
    cx->dis_symbols = (Dis_symbol *) mmt_realloc(cx->dis_symbols, sizeof(Dis_symbol)*(N + 1024));
  cx->dis_symbols[N].address = addr;
  cx->dis_symbols[N].type = type;
  cx->dis_symbols[N].subtype = sub;
  cx->dis_symbols[N].count = count;
  cx->dis_symbols[N].used = 0;
  cx->dis_symbols[N].printed = 0;
  cx->dis_symbols[N].name = name? cleanup(str_rtrim(mmt_strdup(str_ltrim(name)))): NULL;
  cx->dis_symbols[N].comment = com? str_rtrim(mmt_strdup(str_ltrim(com))): NULL;
}

/*
 * Tokenising of a tagfile line returning (argc, argv); parsing ends when
 *  - A token starts with a comment character #
 *  - The comment field after the name of the symbol is encountered
 *  - The end of the string is encountered
 *
 * Argv is allocated once, so the caller only needs to mmt_free argv.
 * On error NULL is returned (when input line was too long).
 *
 */
static int tagfile_tokenize(char *s, int *argcp, const char ***argvp) {
  size_t slen;
  int n, nargs;
  const char **argv;
  char *buf, *q, *r;

  // Upper estimate of the number of arguments
  for(nargs = 0, q = s; *q; nargs++) {
    while(*q && !isspace((unsigned char) *q))
      q++;
    while(*q && isspace((unsigned char) *q))
      q++;
  }
  slen = q - s;

  // Limit input line to some 186 Megabytes as max nargs is (slen+1)/2
  if(slen > 2*((INT_MAX - 2*sizeof(char *))/(sizeof(char *) + 3)))
    return 0;

  // Allocate once for pointers and contents, so caller only needs to mmt_free(argv)
  argv = mmt_malloc((nargs + 2)*sizeof(char *) + slen + nargs);
  buf = (char *) (argv + nargs + 1);

  for(n = 0, r = s; *r;) {
    q = str_nexttok(r, " \t\n\r\v\f", &r);
    size_t len = strlen(q);

    if(*q == '#') {             // Inline comment: ignore rest of line
      r = q + len;
      break;
    }
    strcpy(buf, q);
    if(*buf)                    // Don't record empty arguments
      argv[n++] = buf;

    if(n > 1 && n == (str_eq(argv[1], "L")? 3: 5)) {  // Stop parsing after symbol name
      if(*r)
        argv[n] = r;            // Comment, if any
      break;
    }

    buf += len + 1;
  }

  *argcp = n;
  *argvp = argv;
  return 1;
}

#define Return(fmt, ...) do { \
  pmsg_error("tagfile line %d " fmt, lineno, __VA_ARGS__); msg_error("\n"); \
  return -1; \
} while(0)

static int tagfile_readline(char *line, int lineno, const char *const *isrnames, int ni) {
  int type, subtype, vn, address, count, argc = 0;
  const char *errptr, **argv = NULL;

  if(!tagfile_tokenize(line, &argc, &argv))
    Return("%s", "is too long");
  if(argc == 0)
    return 0;
  if(argc < 3)
    Return("%s", "needs at least address, symbol type (L/P/M) and name");

  address = str_int(argv[0], STR_INT32, &errptr);
  if(errptr)
    Return("address %s: %s", argv[0], errptr);

  if(strlen(argv[1]) != 1 || !strchr("LPM", *argv[1]))
    Return("%s", "2nd argument must be L, P or M");
  type = *argv[1];

  if(type == 'L') {
    const char *name = argv[2];

    if(str_starts(name, "__vector_") && looks_like_number(name + 9))
      if((vn = strtol(name + 9, NULL, 0)) > 0 && vn < ni)       // Don't replace __vectors_0
        name = str_lc((char *) str_ccprintf("__vector_%s", isrnames[vn]));
    add_symbol(address, 'L', TYPE_BYTE, 1, name, argv[3]);
    return 0;
  }

  if(argc < 5 || strlen(argv[2]) != 1 || !strchr("BWAS", *argv[2]))
    Return("needs to be <address> %c [%s] <count> <name>", type, type == 'M'? "BW": "BWAS");

  switch(*argv[2]) {
  default:
    subtype = TYPE_BYTE;
    break;
  case 'W':
    subtype = TYPE_WORD;
    break;
  case 'A':
    subtype = TYPE_ASTRING;
    break;
  case 'S':
    subtype = TYPE_STRING;
  }

  if(type == 'M' && subtype != TYPE_BYTE && subtype != TYPE_WORD) {
    pmsg_error("memory label type can only be B(yte) or W(ord)");
    return -1;
  }

  count = str_int(argv[3], STR_INT32, &errptr);
  if(errptr)
    Return("count %s: %s\n", argv[3], errptr);
  if(count < 1)
    Return("tagfile line %d has invalid count %d", lineno, count);

  add_symbol(address, type, subtype, count, argv[4], argv[5]);
  mmt_free(argv);
  return 0;
}

// Allocate, copy, append a suffix (H, L, 0...8 or nothing), cleanup name and return
static char *regname(const char *pre, const char *reg, int suf) {
  char *ret =
    suf <= -1? mmt_sprintf("%s%s", pre, reg):
    suf == 'h' || suf == 'l'? mmt_sprintf("%s%s%c", pre, reg, suf): mmt_sprintf("%s%s%d", pre, reg, suf);

  return cleanup(ret);
}

// Return the basename of a register, ie, the part after the first . (if any)
static const char *regbase(const char *reg) {
  const char *ret = strchr(reg, '.');

  return ret? ret + 1: reg;
}

// Return the basename of rf[i].reg if that's unique amongst the nr register entries
static const char *shortrname(const Register_file *rf, int nr, int i) {
  const char *f = rf[i].reg, *s = regbase(f);

  if(f != s)
    for(int k = 0; k < nr; k++)
      if(k != i && str_eq(s, regbase(rf[k].reg)))
        return f;

  return s;
}

static void add_register(int io_off, int addr, const char *name, int suffix) {
  add_symbol(io_off + addr, 'M', TYPE_BYTE, 1, regname(io_off? "mem.": "", name, suffix), NULL);
  if(addr < 0x40 && io_off)     // Only keep I/O addresses separate if mem addresses have an offset
    add_symbol(addr, 'I', TYPE_BYTE, 1, regname(io_off? "io.": "", name, suffix), NULL);
}

// Initialise cx->dis_symbols from part register file
static void init_regfile(const AVRPART *p) {
  AVRMEM *mem;
  int nr = 0, io_off = cx->dis_io_offset;
  const Register_file *rf = avr_locate_register_file(p, &nr);

  if((mem = avr_locate_sram(p)) && mem->size > 1 && mem->offset <= 0x200) {
    add_symbol(mem->offset, 'M', TYPE_BYTE, mem->size, "sram.start", NULL);
    add_symbol(mem->offset + mem->size - 1, 'M', TYPE_BYTE, 1, "sram.end", NULL);
  }
  if(rf) {
    for(int i = 0; i < nr; i++) {
      const char *rname = io_off? shortrname(rf, nr, i): rf[i].reg;

      if(rf[i].size == 1) {
        add_register(io_off, rf[i].addr, rname, -1);
      } else if(rf[i].size == 2) {
        add_register(io_off, rf[i].addr, rname, 'l');
        add_register(io_off, rf[i].addr + 1, rname, 'h');
      } else if(rf[i].size > 2) {
        for(int k = 0; k < rf[i].size; k++)
          add_register(io_off, rf[i].addr + k, rname, k);
      }
    }
    qsort(cx->dis_symbols, cx->dis_symbolN, sizeof(Dis_symbol), symbol_stable_qsort);
  }
}

int disasm_init_tagfile(const AVRPART *p, const char *fname) {
  FILE *inf = fopen(fname, "r");
  int ni = 0, lineno = 1;
  const char *errstr;
  const char *const *isrnames = avr_locate_isrtable(p, &ni);

  if(!inf) {
    pmsg_ext_error("cannot open tagfile %s: %s\n", fname, strerror(errno));
    return -1;
  }

  zap_symbols();
  init_regfile(p);

  for(char *buffer; (buffer = str_fgets(inf, &errstr)); mmt_free(buffer))
    if(tagfile_readline(buffer, lineno++, isrnames, ni) < 0)
      goto error;

  if(errstr) {
    pmsg_error("read error in tag file %s: %s\n", fname, errstr);
    goto error;
  }

  fclose(inf);
  qsort(cx->dis_symbols, cx->dis_symbolN, sizeof(Dis_symbol), symbol_stable_qsort);
  return 0;

error:
  fclose(inf);
  return -1;
}

static const char *resolve_address(int type, int address) {
  Dis_symbol *s = find_symbol(type == 'I' && !cx->dis_io_offset? 'M': type, address);

  if(s && s->name)
    s->used = 1;

  return s? s->name: NULL;
}

// Increase cycle number by 1 if it's a 3 byte PC
static const char *cycles(int mnemo) {
  if(mnemo < 0)
    return "---";

  const char *ret = avr_opcodes[mnemo].clock[cx->dis_cycle_index];

  // A plus sign after the cycle number means add one for 3-byte PC
  if(*ret && ret[1] == '+')
    return str_ccprintf("%c", cx->dis_flashsz > 128*1024? *ret + 1: *ret);

  return ret;
}

static const char *get_label_name(int destination, const char **commentp) {
  Dis_symbol *s = find_symbol('L', destination);

  if(s && s->name) {
    if(commentp)
      *commentp = s->comment;
    s->printed = 1;             // Will be printed in pass 2
    return s->name;
  }

  for(int i = 0; i < cx->dis_jumpcallN; i++)
    if(cx->dis_jumpcalls[i].to == destination)
      return str_ccprintf("%s%d", cx->dis_jumpcalls[i].is_func? "Subroutine": "Label", cx->dis_jumpcalls[i].labelno);

  return NULL;
}

// Wrap around flash
static int disasm_wrap(int addr) {
  if(cx->dis_flashsz2)
    addr &= cx->dis_flashsz2 - 1;

  return addr;
}

#define disasm_out(...) do { \
  if(cx->dis_pass != 2) \
    break; \
  if(cx->dis_para > 0) \
    term_out("\n"); \
  cx->dis_para = 0; \
  term_out(__VA_ARGS__); \
} while(0)

#define LINE_N 256
typedef struct {
  char label[LINE_N], code[LINE_N], comment[LINE_N];
} Dis_line;

// Opcode starts in codecol()
static int codecol() {
  int ret = 0;

  if(cx->dis_opts.addresses)
    ret += 3 + cx->dis_addrwidth;
  if(cx->dis_opts.sreg_flags)
    ret += 9;
  if(cx->dis_opts.cycles)
    ret += 4;
  if(cx->dis_opts.opcode_bytes)
    ret += 12;

  return (ret? ret + 1: 2);
}

// Comments start in commentcol() + 1
static int commentcol() {
  return codecol() + cx->dis_codewidth;
}

static int is_jumpable(int address) {
  if(!cx->dis_jumpable || address < cx->dis_start || address > cx->dis_end)
    return 0;

  int n = sizeof(int)*8, idx = (address - cx->dis_start)/2;

  return cx->dis_jumpable[idx/n] & (1 << (idx%n));
}

// Format output for a label list r referenced by mnemonic m
static void output_references(const char *m, char *r) {
  disasm_out("; %c%s from ", toupper(*m & 0xff), m + 1);
  for(char *s = r;;) {
    char *c = strchr(s + 1, ',');

    if(c && c - r > 80 && s > r) {
      *s = 0;
      disasm_out("%s\n; %*s ", r, (int) strlen(m), "");
      r = (char *) str_ltrim(s + 1);
      s = r;
    } else if(c && c - r > 70) {
      *c = 0;
      disasm_out("%s\n; %*s ", r, (int) strlen(m), "");
      r = (char *) str_ltrim(c + 1);
      s = r;
    } else if(c) {
      s = c;
    } else {
      disasm_out("%s\n", r);
      break;
    }
  }
}

// Unified printing of a line
static void lineout(const char *code, const char *comment,
  int mnemo, int oplen, const char *buf, int pos, int addr, int showlabel) {
  Dis_jumpcall *jc = cx->dis_jumpcalls;
  int here = disasm_wrap(pos + addr);

  if(cx->dis_opts.labels && showlabel) {
    int match = 0, first = -1;
    const char *comment = NULL, *name;

    for(int i = 0; i < cx->dis_jumpcallN; i++)
      if(jc[i].to == here)
        if(!match++)
          first = i;

    if(cx->dis_pass == 2 && match) {
      cx->dis_para++;
      char *reflist = mmt_malloc(match*(3 + 64)), *r = reflist;       // Worst case length
      int mne = jc[first].mnemo, one_mne = 1;

      for(int i = first; i < cx->dis_jumpcallN && jc[i].to == here; i++) {
        if(mne != jc[i].mnemo) {        // More than one mnemonic reference this line
          one_mne = 0;
          output_references(avr_opcodes[mne].opcode, reflist);
          mne = jc[i].mnemo;
          r = reflist;
          *r = 0;
        }
        strcpy(r, str_ccprintf(&", L%0*x"[2*(r == reflist)], cx->dis_addrwidth, jc[i].from));
        r += strlen(r);
      }
      name = get_label_name(here, &comment);
      if(!comment && strlen(reflist) + commentcol() < 70 && one_mne) {  // Refs line with label
        const char *mnestr = avr_opcodes[mne].opcode;

        disasm_out("%-*s ; %s\n", commentcol(), str_ccprintf("%s:", name),
          str_ccprintf("%c%s from %s", toupper(*mnestr & 0xff), mnestr + 1, reflist));
      } else {
        output_references(avr_opcodes[mne].opcode, reflist);
        if(comment)
          disasm_out("%-*s ; %s\n", commentcol(), str_ccprintf("%s:", name), comment);
        else
          disasm_out("%s:\n", name);
      }
      cx->dis_para = -1;
      mmt_free(reflist);
    } else if(match) {
      (void) get_label_name(here, &comment);
    }
  }

  if(cx->dis_opts.addresses)
    disasm_out("L%0*x: ", cx->dis_addrwidth, here);
  if(cx->dis_opts.sreg_flags)
    disasm_out("%s ", mnemo < 0? "--------": avr_opcodes[mnemo].flags);
  if(cx->dis_opts.cycles)
    disasm_out("%3s ", cycles(mnemo));
  if(cx->dis_opts.opcode_bytes)
    for(int i = 0; i < 4; i++)
      disasm_out(i < oplen? "%02x ": "   ", buf[pos + i] & 0xff);
  disasm_out(codecol() > 2? " ": "  ");
  if(!comment || !*comment || !cx->dis_opts.comments)
    disasm_out("%s\n", code);
  else
    disasm_out("%-*s ; %s\n", cx->dis_codewidth, code, comment);
  if(mnemo == MNEMO_ret || mnemo == MNEMO_u_ret || mnemo == MNEMO_reti || mnemo == MNEMO_u_reti)
    cx->dis_para++;
}

// Process 1, 2 or 4 byte number
static int process_num(const char *buf, int buflen, int nbytes, int pos, int offset) {
  if(buflen - pos < nbytes)
    nbytes = buflen - pos;
  while(nbytes & (nbytes - 1))  // Round down to next power of 2
    nbytes &= nbytes - 1;

  const char *str =
    nbytes == 1? str_ccprintf(".byte   0x%02x", buf[pos] & 0xff):
    nbytes == 2? str_ccprintf(".word   0x%04x", buf2op16(pos)):
    nbytes == 4? str_ccprintf(".long   0x%04x%04x", buf2op16(pos + 2), buf2op16(pos)): "nbytes?";

  lineout(str, NULL, -1, 1, buf, pos, offset, 0);
  return nbytes;
}

static int process_fill0xff(const char *buf, int buflen, int nbytes, int pos, int offset) {
  cx->dis_para++;
  lineout(str_ccprintf(".fill   %d, 2, 0xffff", nbytes/2), NULL, -1, 1, buf, pos, offset, 0);
  return nbytes/2*2;
}

// Output quoted string
static int process_string(const char *buf, int buflen, int pos, int offset) {
  char *code, *out;
  int i = pos;

  while(i < buflen && buf[i])
    i++;

  if(i == buflen) {             // Ran out of buffer: string not terminated
    char *str = mmt_malloc(i - pos + 1);

    memcpy(str, buf + pos, i - pos);
    str[i - pos] = 0;
    out = cfg_escape(str);
    mmt_free(str);
    code = mmt_sprintf(".ascii  %s", out);
  } else {                      // Nul terminated string
    out = cfg_escape(buf + pos);
    code = mmt_sprintf(".asciz  %s", out);
    i++;
  }

  lineout(code, NULL, -1, i - pos, buf, pos, offset, 0);
  mmt_free(out);
  mmt_free(code);

  return i - pos;
}

// Returns number of bytes of PGM data at this position, printing them in pass 2
static int process_data(const char *buf, int buflen, int pos, int offset) {
  int ret = 0;
  Dis_symbol *s = find_symbol('P', disasm_wrap(pos + offset));

  if(!s) {
    if(pos + 1 >= buflen)
      return 0;

    if(!(s = find_symbol('P', disasm_wrap(pos + offset + 1)))) {        // No PGM label, check for fill block
      int k = 0;

      if((buf[pos] & 0xff) == 0xff && (buf[pos + 1] & 0xff) == 0xff)
        for(k = pos + 2; k < buflen; k++)
          if((buf[k] & 0xff) != 0xff || find_symbol('P', disasm_wrap(k + offset)))
            break;
      k &= ~1;
      return !k || k - pos < 4? 0: process_fill0xff(buf, buflen, k - pos, pos, offset);
    }
    // Found PGM label at odd address, print byte before label and continue
    process_num(buf, buflen, 1, pos, offset);
    ret = 1;
  }

  if(s->name) {
    cx->dis_para++;
    s->printed = 1;             // Will be printed in pass 2
    disasm_out("%s:\n", s->name);
  }

  for(int i = 0; i < s->count && pos + ret < buflen; i++) {
    switch(s->subtype) {
    case TYPE_BYTE:
    case TYPE_WORD:
      ret += process_num(buf, buflen, s->subtype == TYPE_WORD? 2: 1, pos + ret, offset);
      break;
    case TYPE_ASTRING:
    case TYPE_STRING:
      ret += process_string(buf, buflen, pos + ret, offset);
    }
  }

  if(s->subtype == TYPE_ASTRING) {      // Autoaligned string
    if(ret%2) {
      if(buf[pos + ret])
        pmsg_warning("autoalignment expects 0x00 padding but got 0x%02x\n", buf[pos + ret] & 0xff);
      lineout(str_ccprintf(".byte   0x%02x", buf[pos + ret] & 0xff),
        "String autoalignment", -1, 1, buf, pos + ret, offset, 0);
      ret++;
    }
  }
  return ret;
}

static void emit_used_symbols() {
  Dis_symbol *s = cx->dis_symbols;
  int len, maxlen = 0;

  for(int i = 0; i < cx->dis_symbolN; i++)
    if(s[i].used && !s[i].printed && s[i].name)
      if((len = strlen(s[i].name)) > maxlen)
        maxlen = len;

  for(int i = 0; i < cx->dis_symbolN; i++)
    if(s[i].used && !s[i].printed && s[i].name) {
      const char *equ = str_ccprintf(".equ    %s,%*s 0x%02x", s[i].name,
        (int) (maxlen - strlen(s[i].name)), "", s[i].address);

      if(s[i].comment)
        disasm_out("%*s%-*s ; %s\n", codecol(), "", cx->dis_codewidth, equ, s[i].comment);
      else
        disasm_out("%*s%s\n", codecol(), "", equ);
    }
}

void disasm_zap_jumpcalls() {
  mmt_free(cx->dis_jumpcalls);
  cx->dis_jumpcalls = NULL;
  cx->dis_jumpcallN = 0;
}

static void register_jumpcall(int from, int to, int mnemo, int is_func) {
  if(cx->dis_opts.labels) {
    Dis_jumpcall *jc = cx->dis_jumpcalls;
    int N = cx->dis_jumpcallN;

    // Already entered this jumpcall?
    for(int i = 0; i < N; i++)
      if(jc[i].from == from && jc[i].to == to && jc[i].mnemo == mnemo)
        return;

    if(N%1024 == 0)
      jc = mmt_realloc(jc, sizeof(Dis_jumpcall)*(N + 1024));
    jc[N].from = from;
    jc[N].to = to;
    jc[N].mnemo = mnemo;
    jc[N].labelno = 0;
    jc[N].is_func = is_func;

    cx->dis_jumpcalls = jc;
    cx->dis_jumpcallN++;
  }
}

static void correct_is_funct(void) {
  int last_idx = 0;
  int last_dest = cx->dis_jumpcalls[0].to;
  int cur_is_func = cx->dis_jumpcalls[0].is_func;

  for(int i = 1; i < cx->dis_jumpcallN; i++) {
    if(cx->dis_jumpcalls[i].to != last_dest) {
      for(int j = last_idx; j < i; j++)
        cx->dis_jumpcalls[j].is_func = cur_is_func;
      last_idx = i;
      last_dest = cx->dis_jumpcalls[i].to;
      cur_is_func = 0;
    }
    cur_is_func = cur_is_func || cx->dis_jumpcalls[i].is_func;
  }
  for(int j = last_idx; j < cx->dis_jumpcallN; j++)
    cx->dis_jumpcalls[j].is_func = cur_is_func;
}

static int jumpcall_sort(const void *v1, const void *v2) {
  const Dis_jumpcall *p1 = v1, *p2 = v2;
  int diff;

  if((diff = p1->to - p2->to))
    return diff;
  if((diff = p1->mnemo - p2->mnemo))
    return diff;
  return p1->from - p2->from;
}

static void enumerate_labels(void) {
  if(cx->dis_jumpcallN > 1) {
    qsort(cx->dis_jumpcalls, cx->dis_jumpcallN, sizeof(Dis_jumpcall), jumpcall_sort);
    correct_is_funct();

    int dest = 987654321, cur_labelno = 0, cur_funcno = 0;

    for(int i = 0; i < cx->dis_jumpcallN; i++) {
      if(!is_jumpable(cx->dis_jumpcalls[i].to))
        continue;
      cx->dis_jumpcalls[i].labelno = cx->dis_jumpcalls[i].is_func? cur_funcno: cur_labelno;
      if(dest != cx->dis_jumpcalls[i].to) {
        if(cx->dis_jumpcalls[i].is_func)
          cur_funcno++;
        else
          cur_labelno++;
        dest = cx->dis_jumpcalls[i].to;
      }
    }
  }
}

#define Ra (regs['a'])
#define Rd (regs['d'])
#define Rr (regs['r'])
#define Rk (regs['k'])
#define RK (regs['K'])
#define Rs (regs['s'])
#define RA (regs['A'])
#define Rb (regs['b'])
#define Rq (regs['q'])

#define Na (bits['a'])
#define Nd (bits['d'])
#define Nr (bits['r'])
#define Nk (bits['k'])
#define NK (bits['K'])
#define Ns (bits['s'])
#define NA (bits['A'])
#define Nb (bits['b'])
#define Nq (bits['q'])

static char *add_comment(Dis_line *line, const char *comment) {
  int len = strlen(line->comment), rem = LINE_N - len - 1;
  char *p = line->comment + len;

  if(len && *comment && rem > 2)
    strcpy(p, ", "), p += 2, rem -= 2;
  strncpy(p, comment, rem);
  p[rem] = 0;

  return p + strlen(p);
}

static const char *regstyle(int n, int regword) {
  if(regword && !cx->dis_opts.avrgcc_style)
    return str_ccprintf("%d:%d", n + 1, n);
  return str_ccprintf("%d", n);
}

// Return the number of bits set in Number
static unsigned bitcount(unsigned n) {
  unsigned ret;

  // A la Kernighan (and Richie): iteratively clear the least significant bit set
  for(ret = 0; n; ret++)
    n &= n - 1;

  return ret;
}

typedef struct {
  int from, is_func, is_lpm, preop, postop, zwd;
} Op_context;

static const char *get_ldi_name(int op1, int op2, Op_context *oxp) {
  Dis_symbol *s;
  char buf[2];

  int ra = ldi_Rd(op1), rb = ldi_Rd(op2);

  if((ra ^ rb) == 1) {          // Two successive ldi opcodes initialise a register pair
    buf[ra & 1] = ldi_K(op1);
    buf[rb & 1] = ldi_K(op2);
    int addr = buf2op16(0);     // Address of register pair

    // Assume address width is 2 if ldi acts on Z and is followed by e/icall within 5 opcodes
    int awidth = (ra | 1) == 31 && oxp->zwd == 2? 2: 1;

    for(const char *c = awidth == 2 || oxp->is_lpm? "LP": "MLP"; *c; c++)
      if((s = find_symbol(*c, addr*awidth)))
        break;
    if(s && s->name) {          // Label matches the address loaded into register pair
      s->used = 1;
      return str_ccprintf("%s%s(%s)", awidth == 2? "pm_": "", ra & 1? "hi8": "lo8", s->name);
    }
    if((ra | 1) == 31 && oxp->is_lpm && addr >= cx->dis_start && addr < cx->dis_end) {
      if(cx->dis_pass == 1)
        register_jumpcall(oxp->from, addr, MNEMO_ldi, 0);
      const char *name = get_label_name(addr, NULL);

      if(name && cx->dis_opts.labels && is_jumpable(addr))
        return str_ccprintf("%s(%s)", ra & 1? "hi8": "lo8", name);
    } else if(awidth == 2 && 2*addr >= cx->dis_start && 2*addr < cx->dis_end) {
      if(cx->dis_pass == 1)
        register_jumpcall(oxp->from, 2*addr, MNEMO_ldi, oxp->is_func);
      const char *name = get_label_name(2*addr, NULL);

      if(name && cx->dis_opts.labels && is_jumpable(2*addr))
        return str_ccprintf("pm_%s(%s)", ra & 1? "hi8": "lo8", name);
    }
  }
  return NULL;
}

static const char *get_ldi_context(Op_context *oxp, int opcode) {
  const char *ret;

  if(oxp->preop >= 0 && (ret = get_ldi_name(opcode, oxp->preop, oxp)))
    return ret;
  if(oxp->postop >= 0 && (ret = get_ldi_name(opcode, oxp->postop, oxp)))
    return ret;
  return NULL;
}

static void disassemble(const char *buf, int addr, int opcode, AVR_mnemo mnemo, Op_context *oxp, Dis_line *line) {

  memset(line, 0, sizeof *line);
  if(mnemo < 0) {
    add_comment(line, "Invalid opcode");
    snprintf(line->code, LINE_N, ".word   0x%02x%02x", buf[1] & 0xff, buf[0] & 0xff);
    return;
  }

  const AVR_opcode *oc = avr_opcodes + mnemo;
  const char *lsym = NULL;

  if(op16_is_mnemo(opcode, MNEMO_ldi) && (lsym = get_ldi_context(oxp, opcode))) {
    mnemo = MNEMO_ldi;          // Could have been ser
    oc = avr_opcodes + mnemo;
  }

  int regs[128] = { 0 }, bits[128] = { 0 };
  unsigned bmask = 0x8000;

  for(const char *p = oc->bits; *p && bmask; p++) {
    if(*p == ' ')
      continue;
    bits[*p & 0x7f]++;
    regs[*p & 0x7f] <<= 1;
    regs[*p & 0x7f] |= !!(opcode & bmask);
    bmask >>= 1;
  }

  // Treat 32 bit opcodes
  if(oc->nwords == 2) {
    bits['k'] += 16;
    regs['k'] <<= 16;
    regs['k'] |= buf2op16(2);
  }

  // Some sanity checks for things the code relies on
  if(NA && NA != 5 && NA != 6)
    pmsg_warning("unexpected number of A bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Na && Na != 7)
    pmsg_warning("unexpected number of a bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nb && Nb != 3)
    pmsg_warning("unexpected number of b bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nk && Nk != 7 && Nk != 12 && Nk != 16 && Nk != 22)
    pmsg_warning("unexpected number of k bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(NK && NK != 4 && NK != 6 && NK != 8)
    pmsg_warning("unexpected number of  bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nq && Nq != 6)
    pmsg_warning("unexpected number of q bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nd && (Nd < 2 || Nd > 5))
    pmsg_warning("unexpected number of Rd bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Nr && (Nr < 3 || Nr > 5))
    pmsg_warning("unexpected number of Rr bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);
  if(Ns && Ns != 3)
    pmsg_warning("unexpected number of s bits in avr_opcodes table for OP_ID(%s)\n", oc->idname);

  // Cbr r17, 0x06 is marginally easier to read than andi r17, 0xf9
  if(mnemo == MNEMO_andi && bitcount(RK) >= 4) {
    RK = ~RK & 0xff;
    mnemo = MNEMO_cbr;
    oc = avr_opcodes + mnemo;
  }

  // Apply register formula
  int regword = 0;

  switch(oc->type & OTY_REG_MASK) {
  case OTY_REVN:                // Even registers r0, r2, ..., r30
    Rd *= 2, Rr *= 2;
    regword = 1;                // movw
    break;
  case OTY_RUPP:                // Upper registers only r16, ..., r31
    Rd += 16, Rr += 16;
    break;
  case OTY_RW24:                // r24, r26, r28, r30 only
    Rd = 2*Rd + 24;
    regword = 1;                // adiw, sbiw
    break;
  }

  int awd = cx->dis_addrwidth, swd = cx->dis_sramwidth;

  int target = 0, offset = 0, is_jumpcall = 0, is_relative = 0;
  int is_function = !!(oc->type & OTY_EXTERNAL);        // Call/rcall affects stack memory
  const char *name, *ksym = NULL, *asym = NULL, *rsym = NA? resolve_address('I', RA): NULL;

  if(Na) {
    /*
     * Address is limited to 0x40...0xbf for the reduced-core (TPI part)
     * ADDR[7:0] ← (/INST[8], INST[8], INST[10], INST[9], INST[3], INST[2], INST[1], INST[0])
     * ADDR[7:0] ← (/a[4], a[4], a[6], a[5], a[3], a[2], a[1], a[0])
     */
    Ra = (Ra & 0xf) | ((Ra >> 1) & 0x30) | ((Ra & 0x10) << 2) | (((Ra & 0x10) ^ 0x10) << 3);
    asym = resolve_address('M', Ra);
  }

  switch(Nk) {
  case 0:
    break;
  case 7:                      // Branches
    offset = (int8_t) (Rk << 1);        // Sign-extend and multiply by 2
    target = disasm_wrap(addr + offset + 2);
    if(cx->dis_pass == 1 && offset)
      register_jumpcall(addr, target, mnemo, 0);
    is_jumpcall = 1;
    is_relative = 1;
    break;                      // rjmp/rcall
  case 12:
    offset = (int16_t) (Rk << 4) >> 3;  // Sign extend and multiply by 2
    target = disasm_wrap(addr + offset + 2);
    if(cx->dis_pass == 1 && offset)
      register_jumpcall(addr, target, mnemo, is_function);
    is_jumpcall = 1;
    is_relative = 1;
    break;
  case 16:                     // lds/sts
    ksym = resolve_address('M', Rk);
    break;
  case 22:
    if(cx->dis_flashsz && 2*Rk > cx->dis_flashsz)
      add_comment(line, str_ccprintf("Warning: destination outside flash [0, 0x%0*x]", awd, cx->dis_flashsz - 1));
    target = 2*Rk;            // disasm_wrap(2*Rk);
    if(cx->dis_pass == 1)
      register_jumpcall(addr, target, mnemo, is_function);
    is_jumpcall = 1;
    break;
  }

  snprintf(line->code, LINE_N, "%-7s ", oc->opcode);
  char *lc = line->code + strlen(line->code);

#define add_operand(lc, ...) snprintf((lc), LINE_N - ((lc) - line->code),  __VA_ARGS__)

  // Check for opcodes with undefined results
  switch(oc->type & OTY_WARN_MASK) {
  case OTY_XWRN:
    if(Rd == 26 || Rd == 27 || Rr == 26 || Rr == 27)
      add_comment(line, "Warning: the result of this operation is undefined");
    break;
  case OTY_YWRN:
    if(Rd == 28 || Rd == 29 || Rr == 28 || Rr == 29)
      add_comment(line, "Warning: the result of this operation is undefined");
    break;
  case OTY_ZWRN:
    if(Rd == 30 || Rd == 31 || Rr == 30 || Rr == 31)
      add_comment(line, "Warning: the result of this operation is undefined");
    break;
  }

  for(const char *o = oc->operands; *o && lc - line->code < LINE_N - 1; o++) {
    switch(*o) {
    case 'R':
      *lc++ = 'r', *lc = 0;
      break;
    default:
      *lc++ = *o, *lc = 0;
      break;
    case 'A':
      if(rsym)
        add_operand(lc, "%s", rsym);
      else
        add_operand(lc, "0x%02x", RA);
      break;
    case 'a':
      if(asym)
        add_operand(lc, "%s", asym);
      else
        add_operand(lc, "0x%02x", Ra);
      break;
    case 'k':
      if(is_jumpcall) {
        name = get_label_name(target, NULL);
        if(name && target != disasm_wrap(addr + 2) && cx->dis_opts.labels && is_jumpable(target)) {
          add_operand(lc, "%s", name);
          if(cx->dis_opts.addresses)
            add_comment(line, str_ccprintf("L%0*x", awd, target));
        } else {
          if(is_relative) {
            add_operand(lc, ".%+d", offset);
            if(cx->dis_opts.addresses)
              add_comment(line, str_ccprintf("L%0*x", awd, target));
          } else
            add_operand(lc, "0x%0*x", awd, target);
        }
      } else {
        if(ksym)
          add_operand(lc, "%s", ksym);
        else
          add_operand(lc, "0x%0*x", swd, Rk);
      }
      break;
    case 'b':
      add_operand(lc, "%d", Rb);
      add_comment(line, str_ccprintf("Bit %d = 0x%02x", Rb, 1 << Rb));
      break;
    case 's':
      add_operand(lc, "%d", Rs);
      break;
    case 'd':
      add_operand(lc, "%s", regstyle(Rd, regword));
      break;
    case 'r':
      add_operand(lc, "%s", regstyle(Rr, regword));
      break;
    case 'K':
      if(NK == 4)
        add_operand(lc, "%d", RK);
      else {
        if(mnemo == MNEMO_ldi && lsym) {
          add_operand(lc, "%s", lsym);
          add_comment(line, str_ccprintf("0x%02x = %d", RK, RK));
        } else {
          add_operand(lc, "0x%02x", RK);
          add_comment(line, str_ccprintf("%d", RK));
        }
      }
      break;
    case 'q':
      add_operand(lc, "%d", Rq);
      break;
    }
    lc += strlen(lc);
  }
  if(cx->dis_opts.op_names)
    add_comment(line, avr_opcodes[mnemo].description);
  if(cx->dis_opts.op_explanations)
    add_comment(line, avr_opcodes[mnemo].operation);
  // Trim trailing spaces
  while(--lc >= line->code && *lc == ' ')
    *lc = 0;
}

// Is there a label called main and it is used as a destination by disasm()?
static int have_own_main() {
  int mainaddr = -1;
  Dis_symbol *s = cx->dis_symbols;

  for(int i = 0; i < cx->dis_symbolN; i++)
    if(s[i].type == 'L' && s[i].name && str_eq(s[i].name, "main"))
      mainaddr = s[i].address;
  if(mainaddr >= 0)
    for(int i = 0; i < cx->dis_jumpcallN; i++)
      if(cx->dis_jumpcalls[i].to == mainaddr)
        return 1;
  return 0;
}

static void set_context(Op_context *oxp, const char *buf, int pos, int buflen, int addr, int leadin, int leadout) {

  // Compute initial context structure: the opcode before and the following one
  oxp->from = disasm_wrap(pos + addr);
  oxp->is_func = 0;             // Next Z-opcode ahead is an icall/eicall
  oxp->is_lpm = 0;              // Next Z-opcode ahead is a lpm/elpm
  oxp->preop = pos + leadin > 1? buf2op16(pos - 2): -1;
  oxp->postop = -1;
  oxp->zwd = 0;                 // 2: next Z-opcode ahead uses Z as word addr, 1: as byte addr
  int k = 0, op16, i = pos + op_width(buf2op16(pos));

  if(i < buflen + leadout - 1) {
    AVR_mnemo z = 0;

    oxp->postop = op16 = buf2op16(i);
    // Check whether there is an opcode ahead that uses the Z register
    for(k = 0, i += op_width(op16); k < 6 && i < buflen + leadout - 1; k++, i += op_width(op16)) {
      if(op16_is_mnemo(op16, MNEMO_rjmp) || op16_is_mnemo(op16, MNEMO_jmp) ||
        op16_is_mnemo(op16, MNEMO_ret) || op16_is_mnemo(op16, MNEMO_reti) ||
        op16_is_mnemo(op16, MNEMO_u_ret) || op16_is_mnemo(op16, MNEMO_u_reti) ||
        (oxp->zwd = z_width((op16 = buf2op16(i)), &z))) {

        break;
      }
    }
    if(oxp->zwd == 2)
      oxp->is_func = z == MNEMO_icall || z == MNEMO_eicall || z == MNEMO_u_icall || z == MNEMO_u_eicall;
    else
      oxp->is_lpm = z >= MNEMO_lpm_0 && z <= MNEMO_elpm_zp;
  }
}

/*
 * Disassemble buflen bytes at buf which corresponds to address addr
 *
 *  - Caller is responsible that buflen does not split an opcode
 *  - Before(!) the location buf there are leadin bytes available (0-2)
 *  - After the location buf+readlen there are leadout bytes available (0-16)
 */
int disasm(const char *buf, int buflen, int addr, int leadin, int leadout) {
  int pos, opcode, mnemo, oplen;
  Dis_line line = { 0 };
  Op_context ox = { 0 };

  for(int i = 0; i < cx->dis_symbolN; i++)      // Clear used/printed state of symbols
    cx->dis_symbols[i].used = cx->dis_symbols[i].printed = 0;

  cx->dis_jumpable = mmt_malloc((buflen + 1)/2/8);  // Allocate one bit per word address
  cx->dis_start = addr, cx->dis_end = addr + buflen - 1;

  // Make two passes: the first gathers labels, the second outputs the assembler code
  for(cx->dis_pass = 1; cx->dis_pass < 3; cx->dis_pass++) {
    if(cx->dis_pass == 2) {
      cx->dis_para = 0;
      enumerate_labels();
      if(cx->dis_opts.avrgcc_style)
        emit_used_symbols();
      if(cx->dis_opts.gcc_source) {
        cx->dis_para++;
        disasm_out("%*s.text\n%s", codecol(), "", have_own_main()? "": "main:\n");
        cx->dis_para = -1;
      }
    }
    for(pos = 0; pos < buflen; pos += oplen) {
      // Check if this is actually code or maybe only data from tagfile
      if((oplen = process_data(buf, buflen, pos, addr))) {
        cx->dis_para++;
        continue;
      }

      if(pos & 1) {             // Last of PGM data items left off at odd address
        oplen = process_num(buf, buflen, 1, pos, addr);
        continue;
      }

      opcode = buf2op16(pos);
      mnemo = opcode_mnemo(opcode, cx->dis_opts.avrlevel);
      oplen = mnemo < 0? 2: 2*avr_opcodes[mnemo].nwords;

      if(op16_is_mnemo(opcode, MNEMO_ldi))
        set_context(&ox, buf, pos, buflen, addr, leadin, leadout);
      disassemble(buf + pos, disasm_wrap(pos + addr), opcode, mnemo, &ox, &line);
      lineout(line.code, line.comment, mnemo, oplen, buf, pos, addr, 1);
      if(cx->dis_pass == 1) {   // Mark this position as potential jump/call destination
        int n = sizeof(int)*8, idx = pos/2;

        cx->dis_jumpable[idx/n] |= (1 << (idx%n));
      }
    }
  }

  mmt_free(cx->dis_jumpable);
  cx->dis_jumpable = NULL;
  return 0;
}

// Should be called once per terminal session
int disasm_init(const AVRPART *p) {
  AVRMEM *mem;

  // Sanity check (problems only occur if avr_opcodes was changed)
  for(size_t i = 0; i < sizeof avr_opcodes/sizeof *avr_opcodes; i++)
    if(avr_opcodes[i].mnemo != (AVR_mnemo) i) {
      msg_error("avr_opcodes[] table broken (this should never happen)\n");
      return -1;
    }

  cx->dis_flashsz = 0;          // Flash size
  cx->dis_flashsz2 = 0;         // Flash size rounded up to next power of two
  cx->dis_addrwidth = 4;        // Number of hex digits needed for flash addresses
  cx->dis_sramwidth = 4;        // Number of hex digits needed for sram addresses
  cx->dis_codewidth = 28;       // Width of the code column (eg, ldi r17, 0x32)

  if((mem = avr_locate_flash(p)) && mem->size > 1) {
    int nbits = intlog2(mem->size - 1) + 1;

    cx->dis_flashsz = mem->size;
    cx->dis_flashsz2 = 1 << nbits;
    cx->dis_addrwidth = (nbits + 3)/4;
  }

  if((mem = avr_locate_sram(p)) && mem->size > 1) {
    int size = mem->size;

    if(mem->offset > 0 && mem->offset <= 0x200)
      size += mem->offset;
    cx->dis_sramwidth = (intlog2(size - 1) + 1 + 3)/4;
  }

  cx->dis_cycle_index = avr_get_cycle_index(p);
  cx->dis_io_offset = (mem = avr_locate_io(p))? mem->offset: 0;
  init_regfile(p);
  return 0;
}
