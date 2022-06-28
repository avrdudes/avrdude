/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022, Stefan Rueger <smr@theblueorange.space>
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

/*
 * Code to program an Atmel AVR device through one of the supported
 * programmers.
 *
 * For parallel port connected programmers, the pin definitions can be
 * changed via a config file.  See the config file for instructions on
 * how to add a programmer definition.
 *
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <whereami.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "developer_opts.h"
#include "developer_opts_private.h"

static char cmdbitchar(CMDBIT cb) {
  switch(cb.type) {
  case AVR_CMDBIT_IGNORE:
    return 'x';
  case AVR_CMDBIT_VALUE:
    return cb.value? '1': '0';
  case AVR_CMDBIT_ADDRESS:
    return 'a';
  case AVR_CMDBIT_INPUT:
    return 'i';
  case AVR_CMDBIT_OUTPUT:
    return 'o';
  default:
    return '?';
  }
}


static const char *opcodename(int what) {
  switch(what) {
  case AVR_OP_READ:
    return "read";
  case AVR_OP_WRITE:
    return "write";
  case AVR_OP_READ_LO:
    return "read_lo";
  case AVR_OP_READ_HI:
    return "read_hi";
  case AVR_OP_WRITE_LO:
    return "write_lo";
  case AVR_OP_WRITE_HI:
    return "write_hi";
  case AVR_OP_LOADPAGE_LO:
    return "loadpage_lo";
  case AVR_OP_LOADPAGE_HI:
    return "loadpage_hi";
  case AVR_OP_LOAD_EXT_ADDR:
    return "load_ext_addr";
  case AVR_OP_WRITEPAGE:
    return "writepage";
  case AVR_OP_CHIP_ERASE:
    return "chip_erase";
  case AVR_OP_PGM_ENABLE:
    return "pgm_enable";
  default:
    return "???";
  }
}


char *opcode2str(OPCODE *op, int detailed) {
  char cb, space[1024], *sp = space;

  if(detailed)
    *sp++ = '"';
  for(int i=31; i >= 0; i--) {
    *sp++ = cb = cmdbitchar(op->bit[i]);
    if(detailed && cb == 'a') {
      sprintf(sp, "%d", op->bit[i].bitno);
      sp += strlen(sp);
    }
    if(i) {
      if(detailed)
        *sp++ = ' ';
      if(i%8 == 0)
        *sp++ = ' ';
    }
  }
  if(detailed)
    *sp++ = '"';
  *sp = 0;

  return strdup(space);
}

static void printopcode(AVRPART *p, const char *d, OPCODE *op, int what) {
  unsigned char cmd[4];
  int i;

  if(op) {
    memset(cmd, 0, sizeof cmd);
    avr_set_bits(op, cmd);

    dev_info(".op %s %s %s 0x%02x%02x%02x%02x ", p->desc, d, opcodename(what), cmd[0], cmd[1], cmd[2], cmd[3]);
    for(i=31; i >= 0; i--) {
      dev_info("%c", cmdbitchar(op->bit[i]));
      if(i%8 == 0)
        dev_info("%c", i? ' ': '\n');
    }
  }
}

static void printallopcodes(AVRPART *p, const char *d, OPCODE **opa) {
  for(int i=0; i<AVR_OP_MAX; i++)
    printopcode(p, d, opa[i], i);
}


// returns position 0..31 of highest bit set or INT_MIN if no bit is set
static int intlog2(unsigned int n) {
  int ret;

  if(!n)
    return INT_MIN;

  for(ret = 0; n >>= 1; ret++)
    continue;

  return ret;
}


// mnemonic characterisation of flags
static char *parttype(AVRPART *p) {
  static char type[1024];

  switch(p->flags & (AVRPART_HAS_PDI | AVRPART_AVR32 | AVRPART_HAS_TPI | AVRPART_HAS_UPDI)) {
  case 0:                strcpy(type, "ISP"); break;
  case AVRPART_HAS_PDI:  strcpy(type, "PDI"); break;
  case AVRPART_AVR32:    strcpy(type, "AVR32"); break;
  case AVRPART_HAS_TPI:  strcpy(type, "TPI"); break;
  case AVRPART_HAS_UPDI: strcpy(type, "UPDI"); break;
  default:               strcpy(type, "UNKNOWN"); break;
  }
  if((p->flags & AVRPART_SERIALOK) == 0)
    strcat(type, "|NOTSERIAL");
  if((p->flags & AVRPART_PARALLELOK) == 0)
    strcat(type, "|NOTPARALLEL");
  if(p->flags & AVRPART_PSEUDOPARALLEL)
    strcat(type, "|PSEUDOPARALLEL");
  if(p->flags & AVRPART_IS_AT90S1200)
    strcat(type, "|IS_AT90S1200");

  if(p->flags & AVRPART_HAS_DW)
    strcat(type, "|DW");

  if(p->flags & AVRPART_HAS_JTAG)
    strcat(type, "|JTAG");
  if(p->flags & AVRPART_ALLOWFULLPAGEBITSTREAM)
    strcat(type, "|PAGEBITSTREAM");
  if((p->flags & AVRPART_ENABLEPAGEPROGRAMMING) == 0)
    strcat(type, "|NOPAGEPROGRAMMING");

  return type;
}


// check whether address bits are where they should be in ISP commands
static void checkaddr(int memsize, int pagesize, int what, OPCODE *op, AVRPART *p, AVRMEM *m) {
  int i, lo, hi;
  const char *whatstr = opcodename(what);

  lo = intlog2(pagesize);
  hi = intlog2(memsize-1);

  // address bits should be between positions lo and hi (and fall in line), outside should be 0 or don't care
  for(i=0; i<16; i++) {         // ISP programming only deals with 16-bit addresses (words for flash, bytes for eeprom)
    if(i < lo || i > hi) {
      if(op->bit[i+8].type != AVR_CMDBIT_IGNORE && !(op->bit[i+8].type == AVR_CMDBIT_VALUE && op->bit[i+8].value == 0)) {
        dev_info(".cmdbit%d %s %s-%s outside addressable space should be x or 0, but is %c", i+8, p->desc, m->desc, whatstr, cmdbitchar(op->bit[i+8]));
        if(op->bit[i+8].type == AVR_CMDBIT_ADDRESS)
          dev_info("%d", op->bit[i+8].bitno);
        dev_info("\n");
      }
    } else {
      if(op->bit[i+8].type != AVR_CMDBIT_ADDRESS)
        dev_info(".cmdbit%d %s %s-%s is %c but should be a\n", i+8, p->desc, m->desc, whatstr, cmdbitchar(op->bit[i+8]));
      else if(op->bit[i+8].bitno != i)
        dev_info(".cmdbit%d %s %s-%s inconsistent: a%d specified as a%d\n", i+8, p->desc, m->desc, whatstr, i, op->bit[i+8].bitno);
    }
  }
  for(i=0; i<32; i++)           // command bits 8..23 should not contain address bits
    if((i<8 || i>23) && op->bit[i].type == AVR_CMDBIT_ADDRESS)
      dev_info(".cmdbit%d %s %s-%s contains a%d which it shouldn't\n", i, p->desc, m->desc, whatstr, op->bit[i].bitno);
}


void dev_stack_out(bool dot, AVRPART *p, char *name, unsigned char *stack, int ns) {
  if(dot)
    dev_info("%s\t%s\t", p->desc, name);
  else
    dev_info("    %-19s = ", name);
  for(int i=0; i<ns; i++)
    dev_info("0x%02x%s", stack[ns], i+1<ns? " ": dot? "\n": ";\n");
}


// -p */[cdosw*]
void dev_output_part_defs(char *partdesc) {
  bool first = 1, cmdok, waits, opspi, descs, strct, all, dot;

  if(!*partdesc)
    all = 1;
  else if(*partdesc != '/' || (*partdesc == '/' && partdesc[1] && !strchr("cdosw*", partdesc[1]))) {
    dev_info("%s: flags for developer option -p \\* not recognised\n", progname);
    dev_info("  -p \\*/c  check address bits in SPI commands\n");
    dev_info("  -p \\*/d  description of core part features\n");
    dev_info("  -p \\*/o  opcodes for SPI programming parts and memories\n");
    dev_info("  -p \\*/s  show avrdude.conf entries of parts\n");
    dev_info("  -p \\*/ss show full avrdude.conf entry as tab separated table\n");
    dev_info("  -p \\*/w  wd_... constants for ISP parts\n");
    dev_info("  -p \\*/\\* all of the above except -p \\*/s\n");
    dev_info("  -p \\*    same as -p\\*/\\*\n");
    return;
  } else {
    partdesc++;
    all = !!strchr(partdesc, '*') || !strlen(partdesc);
  }

  // redirect stderr to stdout
  fflush(stderr); fflush(stdout); dup2(1, 2);

  cmdok = all || !!strchr(partdesc, 'c');
  descs = all || !!strchr(partdesc, 'd');
  opspi = all || !!strchr(partdesc, 'o');
  strct = all || !!strchr(partdesc, 's');
  waits = all || !!strchr(partdesc, 'w');
  dot = strlen(partdesc) != 1;

  for(LNODEID ln1 = lfirst(part_list); ln1; ln1 = lnext(ln1)) {
    AVRPART *p = ldata(ln1);
    int flashsize, flashoffset, flashpagesize, eepromsize , eepromoffset, eeprompagesize;

    if(strct) {
      char space[1024], real_config_file[PATH_MAX];

      if(!first)
        dev_info("\n");
      first = 0;

      if(!realpath(p->config_file, real_config_file))
        memcpy(real_config_file, p->config_file, sizeof real_config_file);
      dev_info("# %s %d\n", real_config_file, p->lineno);
      
      if(!dot)
        dev_info("part\n");
      dev_partout("\"%s\"", desc);
      dev_partout("\"%s\"", id);
      dev_partout("\"%s\"", family_id);
      dev_partout("0x%02x", stk500_devcode);
      dev_partout("0x%02x", avr910_devcode);
      dev_partout("%d",     chip_erase_delay);
      dev_partout("0x%02x", pagel);
      dev_partout("0x%02x", bs2);
      sprintf(space, "0x%02x 0x%02x 0x%02x", p->signature[0], p->signature[1], p->signature[2]);
      dev_partout_str(space, signature);
      dev_partout("0x%04x", usbpid);
      sprintf(space, "%s", p->reset_disposition == RESET_DEDICATED? "dedicated": p->reset_disposition == RESET_IO? "io": "unknown");
      dev_partout_str(space, reset);
      sprintf(space, "%s", p->retry_pulse == PIN_AVR_RESET? "reset": p->retry_pulse == PIN_AVR_SCK? "sck": "unknown");
      dev_partout_str(space, retry_pulse);

      if(dot)
        dev_info(".part\t%s\tflags\t0x%04x\n", p->desc, p->flags);
      else {
        dev_flagout("has_jtag", AVRPART_HAS_JTAG);
        dev_flagout("has_debugwire", AVRPART_HAS_DW);
        dev_flagout("has_pdi", AVRPART_HAS_PDI);
        dev_flagout("has_updi", AVRPART_HAS_UPDI);
        dev_flagout("has_tpi", AVRPART_HAS_TPI);
        dev_flagout("is_at90s1200", AVRPART_IS_AT90S1200);
        dev_flagout("is_avr32", AVRPART_AVR32);
        dev_flagout("allowfullpagebitstream", AVRPART_ALLOWFULLPAGEBITSTREAM);
        dev_flagout("enablepageprogramming", AVRPART_ENABLEPAGEPROGRAMMING);
        dev_flagout("serial", AVRPART_SERIALOK);

        switch(p->flags & (AVRPART_PARALLELOK | AVRPART_PSEUDOPARALLEL)) {
        case 0:                      strcpy(space, "no"); break;
        case AVRPART_PSEUDOPARALLEL: strcpy(space, "unknown"); break;
        case AVRPART_PARALLELOK:     strcpy(space, "yes"); break;
        default:                     strcpy(space, "pseudo"); break;
        }
        dev_info("    %-19s = %s;\n", "parallel", space);
      }

      dev_partout("%d",     timeout);
      dev_partout("%d",     stabdelay);
      dev_partout("%d",     cmdexedelay);
      dev_partout("%d",     synchloops);
      dev_partout("%d",     bytedelay);
      dev_partout("%d",     pollindex);
      dev_partout("0x%02x", pollvalue);
      dev_partout("%d",     predelay);
      dev_partout("%d",     postdelay);
      dev_partout("%d",     pollmethod);

      sprintf(space, "%s", p->ctl_stack_type == CTL_STACK_PP? "pp_controlstack": p->ctl_stack_type == CTL_STACK_HVSP? "hvsp_controlstack": "unknown_controlstack");
      if(p->ctl_stack_type != CTL_STACK_NONE)
        dev_stack_out(dot, p, space, p->controlstack, CTL_STACK_SIZE);
      dev_stack_out(dot, p, "flash_instr", p->flash_instr, FLASH_INSTR_SIZE);
      dev_stack_out(dot, p, "eeprom_instr", p->eeprom_instr, EEPROM_INSTR_SIZE);

/*
  unsigned char controlstack[CTL_STACK_SIZE]; // stk500v2 PP/HVSP ctl stack
  unsigned char flash_instr[FLASH_INSTR_SIZE]; // flash instructions (debugWire, JTAG)
  unsigned char eeprom_instr[EEPROM_INSTR_SIZE]; // EEPROM instructions (debugWire, JTAG)
*/

      dev_partout("%d",     hventerstabdelay);
      dev_partout("%d",     progmodedelay);
      dev_partout("%d",     latchcycles);
      dev_partout("%d",     togglevtg);
      dev_partout("%d",     poweroffdelay);
      dev_partout("%d",     resetdelayms);
      dev_partout("%d",     resetdelayus);
      dev_partout("%d",     hvleavestabdelay);
      dev_partout("%d",     resetdelay);
      dev_partout("%d",     chiperasepulsewidth);
      dev_partout("%d",     chiperasepolltimeout);
      dev_partout("%d",     chiperasetime);
      dev_partout("%d",     programfusepulsewidth);
      dev_partout("%d",     programfusepolltimeout);
      dev_partout("%d",     programlockpulsewidth);
      dev_partout("%d",     programlockpolltimeout);
      dev_partout("%d",     synchcycles);
      dev_partout("%d",     hvspcmdexedelay);

      dev_partout("0x%02x", idr);
      dev_partout("0x%02x", rampz);
      dev_partout("0x%02x", spmcr);
      dev_partout("0x%02x", eecr);  // why is eecr an unsigned short?

      dev_partout("0x%04x", mcu_base);
      dev_partout("0x%04x", nvm_base);
      dev_partout("0x%04x", ocd_base);

      if(p->ocdrev >= 0)
        dev_partout("%d", ocdrev);
      else
        dev_partout("0x%8x", ocdrev);
      
      for(int i=0; i < AVR_OP_MAX; i++) {
        if(p->op[i]) {
          char *opc = opcode2str(p->op[i], !dot);

          if(dot)
            dev_info(".partop\t%s\t%s\t%s\n", p->desc, opcodename(i), opc? opc: "error");
          else
            dev_info("    %-19s = %s;\n", opcodename(i), opc? opc: "error");

          if(opc)
            free(opc);
        }
      }
      if(p->mem) {
        for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
          AVRMEM *m = ldata(lnm);
          if(!dot)
            dev_info("\n    memory \"%s\"\n", m->desc);

          dev_memout_yn(paged);
          if(m->size > 8192)
            dev_memout("0x%x", size);
          else
            dev_memout("%d", size);
          dev_memout("%d", page_size);
          dev_memout("%d", num_pages); // why can AVRDUDE not compute this?
          dev_memout("0x%x", offset);
          dev_memout("%d", min_write_delay);
          dev_memout("%d", max_write_delay);
          dev_memout_yn(pwroff_after_write);
          sprintf(space, "0x%02x 0x%02x", m->readback[0], m->readback[1]);
          dev_memout_str(space, readback);
          dev_memout("%d", mode);
          dev_memout("%d", delay);
          dev_memout("%d", blocksize);
          dev_memout("%d", readsize);
          dev_memout("%d", pollindex);

          for(int i=0; i < AVR_OP_MAX; i++) {
            if(m->op[i]) {
              char *opc = opcode2str(m->op[i], !dot);

              if(dot)
                dev_info(".pmemop\t%s\t%s\t%s\t%s\n", p->desc, m->desc, opcodename(i), opc? opc: "error");
              else
                dev_info("        %-15s = %s;\n", opcodename(i), opc? opc: "error");

              if(opc)
                free(opc);
            }
          }

          if(!dot)
            dev_info("    ;\n");
          for(LNODEID lnm=lfirst(p->mem_alias); lnm; lnm=lnext(lnm)) {
            AVRMEM_ALIAS *ma = ldata(lnm);
            if(ma->aliased_mem && !strcmp(ma->aliased_mem->desc, m->desc)) {
              if(dot)
                dev_info(".pmem\t%s\t%s\talias\t%s\n", p->desc, ma->desc, m->desc);
              else
                dev_info("\n    memory \"%s\"\n        alias \"%s\";\n    ;\n", ma->desc, m->desc);
            }
          }
        }
      }

      if(!dot)
        dev_info(";\n");
    }


    // identify core flash and eeprom parameters

    flashsize = flashoffset = flashpagesize = eepromsize = eepromoffset = eeprompagesize = 0;
    if(p->mem) {
      for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
        AVRMEM *m = ldata(lnm);
        if(!flashsize && m->desc && 0==strcmp(m->desc, "flash")) {
          flashsize = m->size;
          flashpagesize = m->page_size;
          flashoffset = m->offset;
        }
        if(!eepromsize && m->desc && 0==strcmp(m->desc, "eeprom")) {
          eepromsize = m->size;
          eepromoffset = m->offset;
          eeprompagesize = m->page_size;
        }
      }
    }

    // "real" entries don't seem to have a space in their desc (a bit hackey)
    if(flashsize && !index(p->desc, ' ')) {
      int ok, nfuses;
      AVRMEM *m;
      OPCODE *oc;

      if(!first && all)
        dev_info("\n");
      first = 0;

      ok = 2047;
      nfuses = 0;

      if(!p->op[AVR_OP_PGM_ENABLE])
        ok &= ~DEV_SPI_EN_CE_SIG;

      if(!p->op[AVR_OP_CHIP_ERASE])
        ok &= ~DEV_SPI_EN_CE_SIG;

      if((m = avr_locate_mem(p, "flash"))) {
        if((oc = m->op[AVR_OP_LOAD_EXT_ADDR])) {
          // @@@ to do: check whether address is put at lsb of third byte
        } else
         ok &= ~DEV_SPI_LOAD_EXT_ADDR;

        if((oc = m->op[AVR_OP_READ_HI])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_READ_HI, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_READ_LO])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_READ_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_WRITE_HI])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_WRITE_HI, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_WRITE_LO])) {
          if(cmdok)
            checkaddr(m->size>>1, 1, AVR_OP_WRITE_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM;

        if((oc = m->op[AVR_OP_LOADPAGE_HI])) {
          if(cmdok)
            checkaddr(m->page_size>>1, 1, AVR_OP_LOADPAGE_HI, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM_PAGED;

        if((oc = m->op[AVR_OP_LOADPAGE_LO])) {
          if(cmdok)
            checkaddr(m->page_size>>1, 1, AVR_OP_LOADPAGE_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM_PAGED;

        if((oc = m->op[AVR_OP_WRITEPAGE])) {
          if(cmdok)
            checkaddr(m->size>>1, m->page_size>>1, AVR_OP_WRITEPAGE, oc, p, m);
        } else
          ok &= ~DEV_SPI_PROGMEM_PAGED;
      } else
        ok &= ~(DEV_SPI_PROGMEM_PAGED | DEV_SPI_PROGMEM);

      if((m = avr_locate_mem(p, "eeprom"))) {
        if((oc = m->op[AVR_OP_READ])) {
          if(cmdok)
            checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM;

        if((oc = m->op[AVR_OP_WRITE])) {
          if(cmdok)
            checkaddr(m->size, 1, AVR_OP_WRITE, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM;

        if((oc = m->op[AVR_OP_LOADPAGE_LO])) {
          if(cmdok)
            checkaddr(m->page_size, 1, AVR_OP_LOADPAGE_LO, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM_PAGED;

        if((oc = m->op[AVR_OP_WRITEPAGE])) {
          if(cmdok)
            checkaddr(m->size, m->page_size, AVR_OP_WRITEPAGE, oc, p, m);
        } else
          ok &= ~DEV_SPI_EEPROM_PAGED;
      } else
        ok &= ~(DEV_SPI_EEPROM_PAGED | DEV_SPI_EEPROM);

      if((m = avr_locate_mem(p, "signature")) && (oc = m->op[AVR_OP_READ])) {
        if(cmdok)
          checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
      } else
        ok &= ~DEV_SPI_EN_CE_SIG;

      if((m = avr_locate_mem(p, "calibration")) && (oc = m->op[AVR_OP_READ])) {
        if(cmdok)
          checkaddr(m->size, 1, AVR_OP_READ, oc, p, m);
      } else
        ok &= ~DEV_SPI_CALIBRATION;

      // actually, some AT90S... parts cannot read, only write lock bits :-0
      if( ! ((m = avr_locate_mem(p, "lock")) && m->op[AVR_OP_WRITE]))
        ok &= ~DEV_SPI_LOCK;

      if(((m = avr_locate_mem(p, "fuse")) || (m = avr_locate_mem(p, "lfuse"))) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_LFUSE;

      if((m = avr_locate_mem(p, "hfuse")) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_HFUSE;

      if((m = avr_locate_mem(p, "efuse")) && m->op[AVR_OP_READ] && m->op[AVR_OP_WRITE])
        nfuses++;
      else
        ok &= ~DEV_SPI_EFUSE;

      if(descs) {
        int len = 16-strlen(p->desc);
        dev_info(".desc '%s' =>%*s [0x%02X, 0x%02X, 0x%02X, 0x%08x, 0x%05x, 0x%03x, 0x%06x, 0x%04x, 0x%03x, %d, 0x%03x, 0x%04x, '%s'], # %s %d\n",
          p->desc, len > 0? len: 0, "",
          p->signature[0], p->signature[1], p->signature[2],
          flashoffset, flashsize, flashpagesize,
          eepromoffset, eepromsize, eeprompagesize,
          nfuses,
          ok,
          p->flags,
          parttype(p),
          p->config_file, p->lineno
        );
      }
    }

    if(opspi) {
      printallopcodes(p, "part", p->op);
      if(p->mem) {
        for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
          AVRMEM *m = ldata(lnm);
          if(m)
            printallopcodes(p, m->desc, m->op);
        }
      }
    }

    // print wait delays for AVR family parts
    if(waits) {
      if(!(p->flags & (AVRPART_HAS_PDI | AVRPART_HAS_UPDI | AVRPART_HAS_TPI | AVRPART_AVR32)))
        dev_info(".wd_chip_erase %.3f ms %s\n", p->chip_erase_delay/1000.0, p->desc);
      if(p->mem) {
        for(LNODEID lnm=lfirst(p->mem); lnm; lnm=lnext(lnm)) {
          AVRMEM *m = ldata(lnm);
          // write delays not needed for read-only calibration and signature memories
          if(strcmp(m->desc, "calibration") && strcmp(m->desc, "signature")) {
            if(!(p->flags & (AVRPART_HAS_PDI | AVRPART_HAS_UPDI | AVRPART_HAS_TPI | AVRPART_AVR32))) {
              if(m->min_write_delay == m->max_write_delay)
                 dev_info(".wd_%s %.3f ms %s\n", m->desc, m->min_write_delay/1000.0, p->desc);
              else {
                 dev_info(".wd_min_%s %.3f ms %s\n", m->desc, m->min_write_delay/1000.0, p->desc);
                 dev_info(".wd_max_%s %.3f ms %s\n", m->desc, m->max_write_delay/1000.0, p->desc);
              }
            }
          }
        }
      }
    }
  }
}
