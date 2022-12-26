/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2000-2004  Brian S. Dean <bsd@bsdhome.com>
 * Copyright (C) 2011 Darell Tan <darell.tan@gmail.com>
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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "tpi.h"

FP_UpdateProgress update_progress;

#define DEBUG 0

/* TPI: returns 1 if NVM controller busy, 0 if free */
int avr_tpi_poll_nvmbsy(const PROGRAMMER *pgm) {
  unsigned char cmd;
  unsigned char res;

  cmd = TPI_CMD_SIN | TPI_SIO_ADDR(TPI_IOREG_NVMCSR);
  (void)pgm->cmd_tpi(pgm, &cmd, 1, &res, 1);
  return (res & TPI_IOREG_NVMCSR_NVMBSY);
}

/* TPI chip erase sequence */
int avr_tpi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  int err;
  AVRMEM *mem;

  if (p->prog_modes & PM_TPI) {
    pgm->pgm_led(pgm, ON);

    /* Set Pointer Register */
    mem = avr_locate_mem(p, "flash");
    if (mem == NULL) {
      pmsg_error("no flash memory to erase for part %s\n", p->desc);
      return -1;
    }

    unsigned char cmd[] = {
      /* write pointer register high byte */
      (TPI_CMD_SSTPR | 0),
      ((mem->offset & 0xFF) | 1),
      /* and low byte */
      (TPI_CMD_SSTPR | 1),
      ((mem->offset >> 8) & 0xFF),
      /* write CHIP_ERASE command to NVMCMD register */
      (TPI_CMD_SOUT | TPI_SIO_ADDR(TPI_IOREG_NVMCMD)),
      TPI_NVMCMD_CHIP_ERASE,
      /* write dummy value to start erase */
      TPI_CMD_SST,
      0xFF
    };

    while (avr_tpi_poll_nvmbsy(pgm))
        ;

    err = pgm->cmd_tpi(pgm, cmd, sizeof(cmd), NULL, 0);
    if(err)
      return err;

    while (avr_tpi_poll_nvmbsy(pgm));

    pgm->pgm_led(pgm, OFF);

    return 0;
  } else {
    pmsg_error("part has no TPI\n");
    return -1;
  }
}

/* TPI program enable sequence */
int avr_tpi_program_enable(const PROGRAMMER *pgm, const AVRPART *p, unsigned char guard_time) {
  int err, retry;
  unsigned char cmd[2];
  unsigned char response;

  if(p->prog_modes & PM_TPI) {
    /* set guard time */
    cmd[0] = (TPI_CMD_SSTCS | TPI_REG_TPIPCR);
    cmd[1] = guard_time;

    err = pgm->cmd_tpi(pgm, cmd, sizeof(cmd), NULL, 0);
    if(err)
      return err;

    /* read TPI ident reg */
    cmd[0] = (TPI_CMD_SLDCS | TPI_REG_TPIIR);
    err = pgm->cmd_tpi(pgm, cmd, 1, &response, sizeof(response));
    if (err || response != TPI_IDENT_CODE) {
      pmsg_error("TPIIR not correct\n");
      return -1;
    }

    /* send SKEY command + SKEY */
    err = pgm->cmd_tpi(pgm, tpi_skey_cmd, sizeof(tpi_skey_cmd), NULL, 0);
    if(err)
      return err;

    /* check if device is ready */
    for(retry = 0; retry < 10; retry++)
    {
      cmd[0] =  (TPI_CMD_SLDCS | TPI_REG_TPISR);
      err = pgm->cmd_tpi(pgm, cmd, 1, &response, sizeof(response));
      if(err || !(response & TPI_REG_TPISR_NVMEN))
        continue;

      return 0;
    }

    pmsg_error("target does not reply when enabling TPI external programming mode\n");
    return -1;

  } else {
    pmsg_error("part has no TPI\n");
    return -1;
  }
}

/* TPI: setup NVMCMD register and pointer register (PR) for read/write/erase */
static int avr_tpi_setup_rw(const PROGRAMMER *pgm, const AVRMEM *mem,
  unsigned long addr, unsigned char nvmcmd) {

  unsigned char cmd[4];
  int rc;

  /* set NVMCMD register */
  cmd[0] = TPI_CMD_SOUT | TPI_SIO_ADDR(TPI_IOREG_NVMCMD);
  cmd[1] = nvmcmd;
  rc = pgm->cmd_tpi(pgm, cmd, 2, NULL, 0);
  if (rc == -1)
    return -1;

  /* set Pointer Register (PR) */
  cmd[0] = TPI_CMD_SSTPR | 0;
  cmd[1] = (mem->offset + addr) & 0xFF;
  rc = pgm->cmd_tpi(pgm, cmd, 2, NULL, 0);
  if (rc == -1)
    return -1;

  cmd[0] = TPI_CMD_SSTPR | 1;
  cmd[1] = ((mem->offset + addr) >> 8) & 0xFF;
  rc = pgm->cmd_tpi(pgm, cmd, 2, NULL, 0);
  if (rc == -1)
    return -1;

  return 0;
}

int avr_read_byte_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                          unsigned long addr, unsigned char * value)
{
  unsigned char cmd[4];
  unsigned char res[4];
  unsigned char data;
  int r;
  OPCODE * readop, * lext;

  if (pgm->cmd == NULL) {
    pmsg_error("%s programmer uses avr_read_byte_default() but does not\n", pgm->type);
    imsg_error("provide a cmd() method\n");
    return -1;
  }

  pgm->pgm_led(pgm, ON);
  pgm->err_led(pgm, OFF);

  if (p->prog_modes & PM_TPI) {
    if (pgm->cmd_tpi == NULL) {
      pmsg_error("%s programmer does not support TPI\n", pgm->type);
      return -1;
    }

    while (avr_tpi_poll_nvmbsy(pgm));

    /* setup for read */
    avr_tpi_setup_rw(pgm, mem, addr, TPI_NVMCMD_NO_OPERATION);

    /* load byte */
    cmd[0] = TPI_CMD_SLD;
    r = pgm->cmd_tpi(pgm, cmd, 1, value, 1);
    if (r == -1) 
      return -1;

    return 0;
  }

  /*
   * figure out what opcode to use
   */
  if (mem->op[AVR_OP_READ_LO]) {
    if (addr & 0x00000001)
      readop = mem->op[AVR_OP_READ_HI];
    else
      readop = mem->op[AVR_OP_READ_LO];
    addr = addr / 2;
  }
  else {
    readop = mem->op[AVR_OP_READ];
  }

  if (readop == NULL) {
#if DEBUG
    pmsg_error("operation not supported on memory type %s\n", mem->desc);
#endif
    return -1;
  }

  /*
   * If this device has a "load extended address" command, issue it.
   */
  lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
  if (lext != NULL) {
    memset(cmd, 0, sizeof(cmd));

    avr_set_bits(lext, cmd);
    avr_set_addr(lext, cmd, addr);
    r = pgm->cmd(pgm, cmd, res);
    if (r < 0)
      return r;
  }

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(readop, cmd);
  avr_set_addr(readop, cmd, addr);
  r = pgm->cmd(pgm, cmd, res);
  if (r < 0)
    return r;
  data = 0;
  avr_get_output(readop, res, &data);

  pgm->pgm_led(pgm, OFF);

  *value = data;

  return 0;
}


/*
 * Return the number of "interesting" bytes in a memory buffer,
 * "interesting" being defined as up to the last non-0xff data
 * value. This is useful for determining where to stop when dealing
 * with "flash" memory, since writing 0xff to flash is typically a
 * no-op. Always return an even number since flash is word addressed.
 * Only apply this optimisation on flash-type memory.
 */
int avr_mem_hiaddr(const AVRMEM * mem)
{
  int i, n;
  static int disableffopt;

  /* calling once with NULL disables any future trailing-0xff optimisation */
  if(!mem) {
    disableffopt = 1;
    return 0;
  }

  if(disableffopt)
    return mem->size;

  /* if the memory is not a flash-type memory do not remove trailing 0xff */
  if(!avr_mem_is_flash_type(mem))
    return mem->size;

  /* return the highest non-0xff address regardless of how much
     memory was read */
  for (i=mem->size-1; i>0; i--) {
    if (mem->buf[i] != 0xff) {
      n = i+1;
      if (n & 0x01)
        return n+1;
      else
        return n;
    }
  }

  return 0;
}


/*
 * Read the entirety of the specified memory type into the corresponding
 * buffer of the avrpart pointed to by p. If v is non-NULL, verify against
 * v's memory area, only those cells that are tagged TAG_ALLOCATED are
 * verified.
 *
 * Return the number of bytes read, or < 0 if an error occurs.
 */
int avr_read(const PROGRAMMER *pgm, const AVRPART *p, const char *memtype, const AVRPART *v) {
  AVRMEM *mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    pmsg_error("no %s memory for part %s\n", memtype, p->desc);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return avr_read_mem(pgm, p, mem, v);
}


/*
 * Read the entirety of the specified memory into the corresponding buffer of
 * the avrpart pointed to by p. If v is non-NULL, verify against v's memory
 * area, only those cells that are tagged TAG_ALLOCATED are verified.
 *
 * Return the number of bytes read, or < 0 if an error occurs.
 */
int avr_read_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, const AVRPART *v) {
  unsigned long i, lastaddr;
  unsigned char cmd[4];
  AVRMEM *vmem = NULL;
  int rc;

  if (v != NULL)
      vmem = avr_locate_mem(v, mem->desc);

  if(mem->size < 0)             // Sanity check
    return -1;

  /*
   * start with all 0xff
   */
  memset(mem->buf, 0xff, mem->size);

  /* supports "paged load" thru post-increment */
  if ((p->prog_modes & PM_TPI) && mem->page_size > 1 &&
      mem->size % mem->page_size == 0 && pgm->cmd_tpi != NULL) {

    while (avr_tpi_poll_nvmbsy(pgm));

    /* setup for read (NOOP) */
    avr_tpi_setup_rw(pgm, mem, 0, TPI_NVMCMD_NO_OPERATION);

    /* load bytes */
    for (lastaddr = i = 0; i < (unsigned long) mem->size; i++) {
      if (vmem == NULL ||
          (vmem->tags[i] & TAG_ALLOCATED) != 0)
      {
        if (lastaddr != i) {
          /* need to setup new address */
          avr_tpi_setup_rw(pgm, mem, i, TPI_NVMCMD_NO_OPERATION);
          lastaddr = i;
        }
        cmd[0] = TPI_CMD_SLD_PI;
        rc = pgm->cmd_tpi(pgm, cmd, 1, mem->buf + i, 1);
        lastaddr++;
        if (rc == -1) {
          pmsg_error("unable to read address 0x%04lx\n", i);
          return -1;
        }
      }
      report_progress(i, mem->size, NULL);
    }
    return avr_mem_hiaddr(mem);
  }

  // HW programmers need a page size > 1, bootloader typ only offer paged r/w
  if ((pgm->paged_load && mem->page_size > 1 && mem->size % mem->page_size == 0) ||
     ((pgm->prog_modes & PM_SPM) && avr_has_paged_access(pgm, mem))) {
    /*
     * the programmer supports a paged mode read
     */
    int need_read, failure;
    unsigned int pageaddr;
    unsigned int npages, nread;

    /* quickly scan number of pages to be written to first */
    for (pageaddr = 0, npages = 0;
         pageaddr < (unsigned int) mem->size;
         pageaddr += mem->page_size) {
      /* check whether this page must be read */
      for (i = pageaddr;
           i < pageaddr + mem->page_size;
           i++)
        if (vmem == NULL /* no verify, read everything */ ||
            (mem->tags[i] & TAG_ALLOCATED) != 0 /* verify, do only
                                                    read pages that
                                                    are needed in
                                                    input file */) {
          npages++;
          break;
        }
    }

    for (pageaddr = 0, failure = 0, nread = 0;
         !failure && pageaddr < (unsigned int) mem->size;
         pageaddr += mem->page_size) {
      /* check whether this page must be read */
      for (i = pageaddr, need_read = 0;
           i < pageaddr + mem->page_size;
           i++)
        if (vmem == NULL /* no verify, read everything */ ||
            (vmem->tags[i] & TAG_ALLOCATED) != 0 /* verify, do only
                                                    read pages that
                                                    are needed in
                                                    input file */) {
          need_read = 1;
          break;
        }
      if (need_read) {
        rc = pgm->paged_load(pgm, p, mem, mem->page_size,
                            pageaddr, mem->page_size);
        if (rc < 0)
          /* paged load failed, fall back to byte-at-a-time read below */
          failure = 1;
      } else {
        pmsg_debug("avr_read_mem(): skipping page %u: no interesting data\n", pageaddr / mem->page_size);
      }
      nread++;
      report_progress(nread, npages, NULL);
    }
    if (!failure)
      return avr_mem_hiaddr(mem);
    /* else: fall back to byte-at-a-time read, for historical reasons */
  }

  if (strcmp(mem->desc, "signature") == 0) {
    if (pgm->read_sig_bytes) {
      return pgm->read_sig_bytes(pgm, p, mem);
    }
  }

  for (i=0; i < (unsigned long) mem->size; i++) {
    if (vmem == NULL || (vmem->tags[i] & TAG_ALLOCATED) != 0) {
      rc = pgm->read_byte(pgm, p, mem, i, mem->buf + i);
      if (rc != LIBAVRDUDE_SUCCESS) {
        pmsg_error("unable to read byte at address 0x%04lx\n", i);
        if (rc == LIBAVRDUDE_GENERAL_FAILURE) {
          pmsg_error("read operation not supported for memory %s\n", mem->desc);
          return LIBAVRDUDE_NOTSUPPORTED;
        }
        pmsg_error("read operation failed for memory %s\n", mem->desc);
        return LIBAVRDUDE_SOFTFAIL;
      }
    }
    report_progress(i, mem->size, NULL);
  }

  return avr_mem_hiaddr(mem);
}



/*
 * write a page data at the specified address
 */
int avr_write_page(const PROGRAMMER *pgm, const AVRPART *p_unused, const AVRMEM *mem,
  unsigned long addr) {

  unsigned char cmd[4];
  unsigned char res[4];
  OPCODE * wp, * lext;

  if (pgm->cmd == NULL) {
    pmsg_error("%s programmer uses avr_write_page() but does not\n", pgm->type);
    imsg_error("provide a cmd() method\n");
    return -1;
  }

  wp = mem->op[AVR_OP_WRITEPAGE];
  if (wp == NULL) {
    pmsg_error("memory %s not configured for page writes\n", mem->desc);
    return -1;
  }

  /*
   * if this memory is word-addressable, adjust the address
   * accordingly
   */
  if ((mem->op[AVR_OP_LOADPAGE_LO]) || (mem->op[AVR_OP_READ_LO]))
    addr = addr / 2;

  pgm->pgm_led(pgm, ON);
  pgm->err_led(pgm, OFF);

  /*
   * If this device has a "load extended address" command, issue it.
   */
  lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
  if (lext != NULL) {
    memset(cmd, 0, sizeof(cmd));

    avr_set_bits(lext, cmd);
    avr_set_addr(lext, cmd, addr);
    pgm->cmd(pgm, cmd, res);
  }

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(wp, cmd);
  avr_set_addr(wp, cmd, addr);
  pgm->cmd(pgm, cmd, res);

  /*
   * since we don't know what voltage the target AVR is powered by, be
   * conservative and delay the max amount the spec says to wait
   */
  usleep(mem->max_write_delay);

  pgm->pgm_led(pgm, OFF);
  return 0;
}


// Return us since program start, rolls over after ca 1h 12min
unsigned long avr_ustimestamp() {
  struct timeval tv;

  memset(&tv, 0, sizeof tv);
  if(gettimeofday(&tv, NULL) == 0) {
    static unsigned long long epoch;
    static int init;
    unsigned long long now;

    now = tv.tv_sec*1000000ULL + tv.tv_usec;
    if(!init) {
      epoch = now;
      init = 1;
    }
    return now - epoch;
  }

  return 0;
}

// Return ms since program start, rolls over after ca 49d 17h
unsigned long avr_mstimestamp() {
  return avr_ustimestamp()/1000;
}

// Return s since program start as double
double avr_timestamp() {
  return avr_ustimestamp()/1e6;
}


int avr_write_byte_default(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                   unsigned long addr, unsigned char data)
{
  unsigned char cmd[4];
  unsigned char res[4];
  unsigned char r;
  int ready;
  int tries;
  unsigned long start_time;
  unsigned long prog_time;
  unsigned char b;
  unsigned short caddr;
  OPCODE * writeop;
  int rc;
  int readok=0;

  if (pgm->cmd == NULL) {
    pmsg_error("%s programmer uses avr_write_byte_default() but does not\n", pgm->type);
    imsg_error("provide a cmd() method\n");
    return -1;
  }

  if (p->prog_modes & PM_TPI) {
    if (pgm->cmd_tpi == NULL) {
      pmsg_error("%s programmer does not support TPI\n", pgm->type);
      return -1;
    }

    if (strcmp(mem->desc, "flash") == 0) {
      pmsg_error("writing a byte to flash is not supported for %s\n", p->desc);
      return -1;
    } else if ((mem->offset + addr) & 1) {
      pmsg_error("writing a byte to an odd location is not supported for %s\n", p->desc);
      return -1;
    }

    while (avr_tpi_poll_nvmbsy(pgm));

    /* must erase fuse first */
    if (strcmp(mem->desc, "fuse") == 0) {
      /* setup for SECTION_ERASE (high byte) */
      avr_tpi_setup_rw(pgm, mem, addr | 1, TPI_NVMCMD_SECTION_ERASE);

      /* write dummy byte */
      cmd[0] = TPI_CMD_SST;
      cmd[1] = 0xFF;
      rc = pgm->cmd_tpi(pgm, cmd, 2, NULL, 0);

      while (avr_tpi_poll_nvmbsy(pgm));
    }

    /* setup for WORD_WRITE */
    avr_tpi_setup_rw(pgm, mem, addr, TPI_NVMCMD_WORD_WRITE);

    cmd[0] = TPI_CMD_SST_PI;
    cmd[1] = data;
    rc = pgm->cmd_tpi(pgm, cmd, 2, NULL, 0);
    /* dummy high byte to start WORD_WRITE */
    cmd[0] = TPI_CMD_SST_PI;
    cmd[1] = data;
    rc = pgm->cmd_tpi(pgm, cmd, 2, NULL, 0);

    while (avr_tpi_poll_nvmbsy(pgm));

    return 0;
  }

  if (!mem->paged && (p->flags & AVRPART_IS_AT90S1200) == 0) {
    /* 
     * check to see if the write is necessary by reading the existing
     * value and only write if we are changing the value; we can't
     * use this optimization for paged addressing.
     *
     * For mysterious reasons, on the AT90S1200, this read operation
     * sometimes causes the high byte of the same word to be
     * programmed to the value of the low byte that has just been
     * programmed before.  Avoid that optimization on this device.
     */
    rc = pgm->read_byte(pgm, p, mem, addr, &b);
    if (rc != 0) {
      if (rc != -1) {
        return -2;
      }
      /*
       * the read operation is not support on this memory type
       */
    }
    else {
      readok = 1;
      if (b == data) {
        return 0;
      }
    }
  }

  /*
   * determine which memory opcode to use
   */
  if (mem->op[AVR_OP_WRITE_LO]) {
    if (addr & 0x01)
      writeop = mem->op[AVR_OP_WRITE_HI];
    else
      writeop = mem->op[AVR_OP_WRITE_LO];
    caddr = addr / 2;
  }
  else if (mem->paged && mem->op[AVR_OP_LOADPAGE_LO]) {
    if (addr & 0x01)
      writeop = mem->op[AVR_OP_LOADPAGE_HI];
    else
      writeop = mem->op[AVR_OP_LOADPAGE_LO];
    caddr = addr / 2;
  }
  else {
    writeop = mem->op[AVR_OP_WRITE];
    caddr = addr;
  }

  if (writeop == NULL) {
#if DEBUG
    pmsg_error("write not supported for memory type %s\n", mem->desc);
#endif
    return -1;
  }


  pgm->pgm_led(pgm, ON);
  pgm->err_led(pgm, OFF);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(writeop, cmd);
  avr_set_addr(writeop, cmd, caddr);
  avr_set_input(writeop, cmd, data);
  pgm->cmd(pgm, cmd, res);

  if (mem->paged) {
    /*
     * in paged addressing, single bytes to be written to the memory
     * page complete immediately, we only need to delay when we commit
     * the whole page via the avr_write_page() routine.
     */
    pgm->pgm_led(pgm, OFF);
    return 0;
  }

  if (readok == 0) {
    /*
     * read operation not supported for this memory type, just wait
     * the max programming time and then return 
     */
    usleep(mem->max_write_delay); /* maximum write delay */
    pgm->pgm_led(pgm, OFF);
    return 0;
  }

  tries = 0;
  ready = 0;
  while (!ready) {

    if ((data == mem->readback[0]) ||
        (data == mem->readback[1])) {
      /* 
       * use an extra long delay when we happen to be writing values
       * used for polled data read-back.  In this case, polling
       * doesn't work, and we need to delay the worst case write time
       * specified for the chip.
       */
      usleep(mem->max_write_delay);
      rc = pgm->read_byte(pgm, p, mem, addr, &r);
      if (rc != 0) {
        pgm->pgm_led(pgm, OFF);
        pgm->err_led(pgm, OFF);
        return -5;
      }
    }
    else {
      start_time = avr_ustimestamp();
      do {
        // Do polling, but timeout after max_write_delay
        rc = pgm->read_byte(pgm, p, mem, addr, &r);
        if (rc != 0) {
          pgm->pgm_led(pgm, OFF);
          pgm->err_led(pgm, ON);
          return -4;
        }
        prog_time = avr_ustimestamp();
      } while (r != data &&  mem->max_write_delay >= 0 &&
          prog_time - start_time < (unsigned long) mem->max_write_delay);
    }

    /*
     * At this point we either have a valid readback or the
     * max_write_delay is expired.
     */
    
    if (r == data) {
      ready = 1;
    }
    else if (mem->pwroff_after_write) {
      /*
       * The device has been flagged as power-off after write to this
       * memory type.  The reason we don't just blindly follow the
       * flag is that the power-off advice may only apply to some
       * memory bits but not all.  We only actually power-off the
       * device if the data read back does not match what we wrote.
       */
      pgm->pgm_led(pgm, OFF);
      pmsg_info("this device must be powered off and back on to continue\n");
      if ((pgm->pinno[PPI_AVR_VCC] & PIN_MASK) <= PIN_MAX) {
        pmsg_info("attempting to do this now ...\n");
        pgm->powerdown(pgm);
        usleep(250000);
        rc = pgm->initialize(pgm, p);
        if (rc < 0) {
          pmsg_error("initialization failed, rc=%d\n", rc);
          imsg_error("cannot re-initialize device after programming the %s bits\n", mem->desc);
          imsg_error("you must manually power-down the device and restart %s to continue\n", progname);
          return -3;
        }
        
        pmsg_info("device was successfully re-initialized\n");
        return 0;
      }
    }

    tries++;
    if (!ready && tries > 5) {
      /*
       * we wrote the data, but after waiting for what should have
       * been plenty of time, the memory cell still doesn't match what
       * we wrote.  Indicate a write error.
       */
      pgm->pgm_led(pgm, OFF);
      pgm->err_led(pgm, ON);
      
      return -6;
    }
  }

  pgm->pgm_led(pgm, OFF);
  return 0;
}


/*
 * write a byte of data at the specified address
 */
int avr_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                   unsigned long addr, unsigned char data)
{
  return pgm->write_byte(pgm, p, mem, addr, data);
}


/*
 * Write the whole memory region of the specified memory from its buffer of
 * the avrpart pointed to by p to the device.  Write up to size bytes from
 * the buffer.  Data is only written if the corresponding tags byte is set.
 * Data beyond size bytes are not affected.
 *
 * Return the number of bytes written, or LIBAVRDUDE_GENERAL_FAILURE on error.
 */
int avr_write(const PROGRAMMER *pgm, const AVRPART *p, const char *memtype, int size, int auto_erase) {
  AVRMEM *m = avr_locate_mem(p, memtype);
  if (m == NULL) {
    pmsg_error("no %s memory for part %s\n", memtype, p->desc);
    return LIBAVRDUDE_GENERAL_FAILURE;
  }

  return avr_write_mem(pgm, p, m, size, auto_erase);
}

/*
 * Write the whole memory region of the specified memory from its buffer of
 * the avrpart pointed to by p to the device.  Write up to size bytes from
 * the buffer.  Data is only written if the corresponding tags byte is set.
 * Data beyond size bytes are not affected.
 *
 * Return the number of bytes written, or LIBAVRDUDE_GENERAL_FAILURE on error.
 */
int avr_write_mem(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m, int size, int auto_erase) {
  int              rc;
  int              newpage, page_tainted, flush_page, do_write;
  int              wsize;
  unsigned int     i, lastaddr;
  unsigned char    data;
  int              werror;
  unsigned char    cmd[4];

  pgm->err_led(pgm, OFF);

  werror  = 0;

  wsize = m->size;
  if (size < wsize) {
    wsize = size;
  } else if (size > wsize) {
    pmsg_warning("%d bytes requested, but memory region is only %d bytes\n", size, wsize);
    imsg_warning("Only %d bytes will actually be written\n", wsize);
  }

  if(wsize <= 0)
    return wsize;

  if ((p->prog_modes & PM_TPI) && m->page_size > 1 && pgm->cmd_tpi) {
    unsigned int    chunk; /* number of words for each write command */
    unsigned int    j, writeable_chunk;

    if (wsize == 1) {
      /* fuse (configuration) memory: only single byte to write */
      return avr_write_byte(pgm, p, m, 0, m->buf[0]) == 0? 1: LIBAVRDUDE_GENERAL_FAILURE;
    }

    while (avr_tpi_poll_nvmbsy(pgm));

    /* setup for WORD_WRITE */
    avr_tpi_setup_rw(pgm, m, 0, TPI_NVMCMD_WORD_WRITE);

    /*
     * Some TPI devices can only program 2 or 4 words (4 or 8 bytes) at a time.
     * This is set by the n_word_writes option of the AVRMEM config section.
     * Ensure that we align our write size to this boundary.
     */
    if (m->n_word_writes < 0 || m->n_word_writes > 4 || m->n_word_writes == 3) {
      msg_error("\n");
      pmsg_error("unsupported n_word_writes value of %d for %s memory\n",
        m->n_word_writes, m->desc);
      return LIBAVRDUDE_GENERAL_FAILURE;
    }
    chunk = m->n_word_writes > 0 ? 2*m->n_word_writes : 2;
    wsize = (wsize+chunk-1) / chunk * chunk;

    /* write words in chunks, low byte first */
    for (lastaddr = i = 0; i < (unsigned int) wsize; i += chunk) {
      /* check that at least one byte in this chunk is allocated */
      for (writeable_chunk = j = 0; !writeable_chunk && j < chunk; j++) {
        writeable_chunk = m->tags[i+j] & TAG_ALLOCATED;
      }

      if (writeable_chunk) {
        if (lastaddr != i) {
          /* need to setup new address */
          avr_tpi_setup_rw(pgm, m, i, TPI_NVMCMD_WORD_WRITE);
          lastaddr = i;
        }

        // Write each byte of the chunk. Unallocated bytes should read
        // as 0xFF, which should no-op.
        cmd[0] = TPI_CMD_SST_PI;
        for (j = 0; j < chunk; j++) {
          cmd[1] = m->buf[i+j];
          rc = pgm->cmd_tpi(pgm, cmd, 2, NULL, 0);
        }

        lastaddr += chunk;

        while (avr_tpi_poll_nvmbsy(pgm));
      }
      report_progress(i, wsize, NULL);
    }

    return i;
  }

  // HW programmers need a page size > 1, bootloader typ only offer paged r/w
  if ((pgm->paged_load && m->page_size > 1 && m->size % m->page_size == 0) ||
     ((pgm->prog_modes & PM_SPM) && avr_has_paged_access(pgm, m))) {
    /*
     * the programmer supports a paged mode write
     */
    int need_write, failure, nset;
    unsigned int pageaddr;
    unsigned int npages, nwritten;

    /*
     * Not all paged memory looks like NOR memory to AVRDUDE, particularly
     *  - EEPROM
     *  - when talking to a bootloader
     *  - handling write via a part-programmer combo that can do page erase
     *
     * Hence, read in from the chip all pages with holes to fill them in. The
     * small cost of doing so is outweighed by the benefit of not potentially
     * overwriting bytes with 0xff outside the input file.
     *
     * Also consider that the effective page size for *SPM* erasing of parts
     * can be 4 times the page size for SPM writing (eg, ATtiny1634). Thus
     * ensure the holes cover the effective page size for SPM programming.
     * Benefits -c arduino with input files with holes on 4-page-erase parts.
     */

    AVRMEM *cm = avr_dup_mem(m);

    // Establish and sanity check effective page size
    int pgsize = (pgm->prog_modes & PM_SPM) && p->n_page_erase > 0?
      p->n_page_erase*cm->page_size: cm->page_size;
    if((pgsize & (pgsize-1)) || pgsize < 1) {
      pmsg_error("effective page size %d implausible\n", pgsize);
      avr_free_mem(cm);
      return -1;
    }

    uint8_t *spc = cfg_malloc(__func__, cm->page_size);

    // Set cwsize as rounded-up wsize
    int cwsize = (wsize + pgsize-1)/pgsize*pgsize;

    for(pageaddr = 0; pageaddr < (unsigned int) cwsize; pageaddr += pgsize) {
      for(i = pageaddr, nset = 0; i < pageaddr + pgsize; i++)
        if(cm->tags[i] & TAG_ALLOCATED)
          nset++;

      if(nset && nset != pgsize) { // Effective page has holes
        for(int np=0; np < pgsize/cm->page_size; np++) { // page by page
          unsigned int beg = pageaddr + np*cm->page_size;
          unsigned int end = beg + cm->page_size;

          for(i = beg; i < end; i++)
            if(!(cm->tags[i] & TAG_ALLOCATED))
              break;

          if(i >= end)          // Memory page has no holes
             continue;

          // Read flash contents to separate memory spc and fill in holes
          if(avr_read_page_default(pgm, p, cm, beg, spc) >= 0) {
            pmsg_notice2("padding %s [0x%04x, 0x%04x]\n", cm->desc, beg, end-1);
            for(i = beg; i < end; i++)
              if(!(cm->tags[i] & TAG_ALLOCATED)) {
                cm->tags[i] |= TAG_ALLOCATED;
                cm->buf[i] = spc[i-beg];
              }
          } else {
            pmsg_notice2("cannot read %s [0x%04x, 0x%04x] to pad page\n",
              cm->desc, beg, end-1);
          }
        }
      }
    }

    // Quickly scan number of pages to be written to
    for(pageaddr = 0, npages = 0; pageaddr < (unsigned int) cwsize; pageaddr += cm->page_size) {
      for(i = pageaddr; i < pageaddr + cm->page_size; i++)
        if(cm->tags[i] & TAG_ALLOCATED) {
          npages++;
          break;
        }
    }

    for (pageaddr = 0, failure = 0, nwritten = 0;
      !failure && pageaddr < (unsigned int) cwsize;
      pageaddr += cm->page_size) {

      // Check whether this page must be written to
      for (i = pageaddr, need_write = 0; i < pageaddr + cm->page_size; i++)
        if ((cm->tags[i] & TAG_ALLOCATED) != 0) {
          need_write = 1;
          break;
        }

      if (need_write) {
        rc = 0;
        if (auto_erase)
          rc = pgm->page_erase(pgm, p, cm, pageaddr);
        if (rc >= 0)
          rc = pgm->paged_write(pgm, p, cm, cm->page_size, pageaddr, cm->page_size);
        if (rc < 0)
          /* paged write failed, fall back to byte-at-a-time write below */
          failure = 1;
      } else {
        pmsg_debug("avr_write_mem(): skipping page %u: no interesting data\n", pageaddr / cm->page_size);
      }
      nwritten++;
      report_progress(nwritten, npages, NULL);
    }

    avr_free_mem(cm);
    free(spc);

    if (!failure)
      return wsize;
    /* else: fall back to byte-at-a-time write, for historical reasons */
  }

  if (pgm->write_setup) {
      pgm->write_setup(pgm, p, m);
  }

  newpage = 1;
  page_tainted = 0;
  flush_page = 0;

  for (i = 0; i < (unsigned int) wsize; i++) {
    data = m->buf[i];
    report_progress(i, wsize, NULL);

    /*
     * Find out whether the write action must be invoked for this
     * byte.
     *
     * For non-paged memory, this only happens if TAG_ALLOCATED is
     * set for the byte.
     *
     * For paged memory, TAG_ALLOCATED also invokes the write
     * operation, which is actually a page buffer fill only.  This
     * "taints" the page, and upon encountering the last byte of each
     * tainted page, the write operation must also be invoked in order
     * to actually write the page buffer to memory.
     */
    do_write = (m->tags[i] & TAG_ALLOCATED) != 0;
    if (m->paged) {
      if (newpage) {
        page_tainted = do_write;
      } else {
        page_tainted |= do_write;
      }
      if (i % m->page_size == (unsigned int) m->page_size - 1 ||
          i == (unsigned int) wsize - 1) {
        /* last byte this page */
        flush_page = page_tainted;
        newpage = 1;
      } else {
        flush_page = newpage = 0;
      }
    }

    if (!do_write && !flush_page) {
      continue;
    }

    if (do_write) {
      rc = avr_write_byte(pgm, p, m, i, data);
      if (rc) {
        msg_error(" ***failed;\n");
        pgm->err_led(pgm, ON);
        werror = 1;
      }
    }

    /*
     * check to see if it is time to flush the page with a page
     * write
     */
    if (flush_page) {
      rc = avr_write_page(pgm, p, m, i);
      if (rc) {
        msg_error(" *** page %d (addresses 0x%04x - 0x%04x) failed to write\n\n",
          i / m->page_size, i - m->page_size + 1, i);
        pgm->err_led(pgm, ON);
          werror = 1;
      }
    }

    if (werror) {
      /*
       * make sure the error led stay on if there was a previous write
       * error, otherwise it gets cleared in avr_write_byte()
       */
      pgm->err_led(pgm, ON);
    }
  }

  return i;
}



/*
 * read the AVR device's signature bytes
 */
int avr_signature(const PROGRAMMER *pgm, const AVRPART *p) {
  int rc;

  if(verbose > 1)
    report_progress(0, 1, "Reading");
  rc = avr_read(pgm, p, "signature", 0);
  if (rc < LIBAVRDUDE_SUCCESS) {
    pmsg_error("unable to read signature data for part %s, rc=%d\n", p->desc, rc);
    return rc;
  }
  report_progress(1, 1, NULL);

  return LIBAVRDUDE_SUCCESS;
}

static uint8_t get_fuse_bitmask(AVRMEM * m) {
  uint8_t bitmask_r = 0;
  uint8_t bitmask_w = 0;
  int i;

  if (!m || m->size > 1) {
    // not a fuse, compare bytes directly
    return 0xFF;
  }

  if (m->op[AVR_OP_WRITE] == NULL ||
      m->op[AVR_OP_READ] == NULL)
    // no memory operations provided by configuration, compare directly
    return 0xFF;

  // For fuses, only compare bytes that are actually written *and* read.
  for (i = 0; i < 32; i++) {
    if (m->op[AVR_OP_WRITE]->bit[i].type == AVR_CMDBIT_INPUT)
      bitmask_w |= (1 << m->op[AVR_OP_WRITE]->bit[i].bitno);
    if (m->op[AVR_OP_READ]->bit[i].type == AVR_CMDBIT_OUTPUT)
      bitmask_r |= (1 << m->op[AVR_OP_READ]->bit[i].bitno);
  }
  return bitmask_r & bitmask_w;
}

int compare_memory_masked(AVRMEM * m, uint8_t b1, uint8_t b2) {
  uint8_t bitmask = get_fuse_bitmask(m);
  return (b1 & bitmask) != (b2 & bitmask);
}

/*
 * Verify the memory buffer of p with that of v.  The byte range of v,
 * may be a subset of p.  The byte range of p should cover the whole
 * chip's memory size.
 *
 * Return the number of bytes verified, or -1 if they don't match.
 */
int avr_verify(const PROGRAMMER *pgm, const AVRPART *p, const AVRPART *v, const char *memtype, int size) {
  int i;
  unsigned char * buf1, * buf2;
  int vsize;
  AVRMEM * a, * b;

  a = avr_locate_mem(p, memtype);
  if (a == NULL) {
    pmsg_error("memory type %s not defined for part %s\n", memtype, p->desc);
    return -1;
  }

  b = avr_locate_mem(v, memtype);
  if (b == NULL) {
    pmsg_error("memory type %s not defined for part %s\n", memtype, v->desc);
    return -1;
  }

  buf1  = a->buf;
  buf2  = b->buf;
  vsize = a->size;

  if (vsize < size) {
    pmsg_warning("requested verification for %d bytes\n", size);
    imsg_warning("%s memory region only contains %d bytes\n", memtype, vsize);
    imsg_warning("only %d bytes will be verified\n", vsize);
    size = vsize;
  }

  int verror = 0, vroerror = 0, maxerrs = verbose >= MSG_DEBUG? size+1: 10;
  for (i=0; i<size; i++) {
    if ((b->tags[i] & TAG_ALLOCATED) != 0 && buf1[i] != buf2[i]) {
      uint8_t bitmask = get_fuse_bitmask(a);
      if(pgm->readonly && pgm->readonly(pgm, p, a, i)) {
        if(quell_progress < 2) {
          if(vroerror < 10) {
            if(!(verror + vroerror))
              pmsg_warning("verification mismatch%s\n",
                avr_mem_is_flash_type(a)? " in r/o areas, expected for vectors and/or bootloader": "");
            imsg_warning("device 0x%02x != input 0x%02x at addr 0x%04x (read only location)\n",
              buf1[i], buf2[i], i);
          } else if(vroerror == 10)
            imsg_warning("suppressing further mismatches in read-only areas\n");
        }
        vroerror++;
      } else if((buf1[i] & bitmask) != (buf2[i] & bitmask)) {
        // Mismatch is not just in unused bits
        if(verror < maxerrs) {
          if(!(verror + vroerror))
            pmsg_warning("verification mismatch\n");
          imsg_error("device 0x%02x != input 0x%02x at addr 0x%04x (error)\n", buf1[i], buf2[i], i);
        } else if(verror == maxerrs) {
          imsg_warning("suppressing further verification errors\n");
        }
        verror++;
        if(verbose < 1)
          return -1;
      } else {
        // Mismatch is only in unused bits
        if ((buf1[i] | bitmask) != 0xff) {
          // Programmer returned unused bits as 0, must be the part/programmer
          pmsg_warning("ignoring mismatch in unused bits of %s\n", memtype);
          imsg_warning("(device 0x%02x != input 0x%02x); to prevent this warning fix\n", buf1[i], buf2[i]);
          imsg_warning("the part or programmer definition in the config file\n");
        } else {
          // Programmer returned unused bits as 1, must be the user
          pmsg_warning("ignoring mismatch in unused bits of %s\n", memtype);
          imsg_warning("(device 0x%02x != input 0x%02x); to prevent this warning set\n", buf1[i], buf2[i]);
          imsg_warning("unused bits to 1 when writing (double check with datasheet)\n");
        }
      }
    }
  }

  return verror? -1: size;
}


int avr_get_cycle_count(const PROGRAMMER *pgm, const AVRPART *p, int *cycles) {
  AVRMEM * a;
  unsigned int cycle_count = 0;
  unsigned char v1;
  int rc;
  int i;

  a = avr_locate_mem(p, "eeprom");
  if (a == NULL) {
    return -1;
  }

  for (i=4; i>0; i--) {
    rc = pgm->read_byte(pgm, p, a, a->size-i, &v1);
  if (rc < 0) {
    pmsg_warning("cannot read memory for cycle count, rc=%d\n", rc);
    return -1;
  }
    cycle_count = (cycle_count << 8) | v1;
  }

   /*
   * If the EEPROM is erased, the cycle count reads 0xffffffff.
   * In this case we return a cycle_count of zero.
   * So, the calling function don't have to care about whether or not
   * the cycle count was initialized.
   */
  if (cycle_count == 0xffffffff) {
    cycle_count = 0;
  }

  *cycles = (int) cycle_count;

  return 0;
}


int avr_put_cycle_count(const PROGRAMMER *pgm, const AVRPART *p, int cycles) {
  AVRMEM * a;
  unsigned char v1;
  int rc;
  int i;

  a = avr_locate_mem(p, "eeprom");
  if (a == NULL) {
    return -1;
  }

  for (i=1; i<=4; i++) {
    v1 = cycles & 0xff;
    cycles = cycles >> 8;

    rc = avr_write_byte(pgm, p, a, a->size-i, v1);
    if (rc < 0) {
      pmsg_warning("cannot write memory for cycle count, rc=%d\n", rc);
      return -1;
    }
  }

  return 0;
}


// Typical order in which memories show in avrdude.conf, runtime adds unknown ones (if any)
const char *avr_mem_order[100] = {
  "eeprom",       "flash",        "application",  "apptable",
  "boot",         "lfuse",        "hfuse",        "efuse",
  "fuse",         "fuse0",        "wdtcfg",       "fuse1",
  "bodcfg",       "fuse2",        "osccfg",       "fuse3",
  "fuse4",        "tcd0cfg",      "fuse5",        "syscfg0",
  "fuse6",        "syscfg1",      "fuse7",        "append",
  "codesize",     "fuse8",        "fuse9",        "bootend",
  "bootsize",     "fuses",        "lock",         "lockbits",
  "tempsense",    "signature",    "prodsig",      "sernum",
  "calibration",  "osccal16",     "osccal20",     "osc16err",
  "osc20err",     "usersig",      "userrow",      "data",
};

void avr_add_mem_order(const char *str) {
  for(size_t i=0; i < sizeof avr_mem_order/sizeof *avr_mem_order; i++) {
    if(avr_mem_order[i] && !strcmp(avr_mem_order[i], str))
      return;
    if(!avr_mem_order[i]) {
      avr_mem_order[i] = cfg_strdup("avr_mem_order()", str);
      return;
    }
  }
  pmsg_error("avr_mem_order[] under-dimensioned in avr.c; increase and recompile\n");
  exit(1);
}

int avr_memtype_is_flash_type(const char *memtype) {
  return memtype && (
     strcmp(memtype, "flash") == 0 ||
     strcmp(memtype, "application") == 0 ||
     strcmp(memtype, "apptable") == 0 ||
     strcmp(memtype, "boot") == 0);
}

int avr_mem_is_flash_type(const AVRMEM *mem) {
  return avr_memtype_is_flash_type(mem->desc);
}

int avr_memtype_is_eeprom_type(const char *memtype) {
  return memtype && strcmp(memtype, "eeprom") == 0;
}

int avr_mem_is_eeprom_type(const AVRMEM *mem) {
  return avr_memtype_is_eeprom_type(mem->desc);
}

int avr_mem_is_known(const char *str) {
  if(str && *str)
    for(size_t i=0; i < sizeof avr_mem_order/sizeof *avr_mem_order; i++)
      if(avr_mem_order[i] && !strcmp(avr_mem_order[i], str))
        return 1;
  return 0;
}

int avr_mem_might_be_known(const char *str) {
  if(str && *str)
    for(size_t i=0; i < sizeof avr_mem_order/sizeof *avr_mem_order; i++)
      if(avr_mem_order[i] && !strncmp(avr_mem_order[i], str, strlen(str)))
        return 1;
  return 0;
}


int avr_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  return pgm->chip_erase(pgm, p);
}

int avr_unlock(const PROGRAMMER *pgm, const AVRPART *p) {
  int rc = -1;

  if (pgm->unlock)
    rc = pgm->unlock(pgm, p);

  return rc;
}

/*
 * Report the progress of a read or write operation from/to the device
 *
 * The first call of report_progress() should look like this (for a write):
 *
 * report_progress(0, 1, "Writing");
 *
 * Then hdr should be passed NULL on subsequent calls *
 * report_progress(k, n, NULL); // k/n signifies proportion of work done
 *
 * with 0 <= k < n, while the operation is progressing. Once the operation is
 * complete, a final call should be made as such to ensure proper termination
 * of the progress report; choose one of the following three forms:
 *
 * report_progress(n, n, NULL); // finished OK, terminate with double \n
 * report_progress(1, 0, NULL); // finished OK, do not print terminating \n
 * report_progress(1, -1, NULL); // finished not OK, print double \n
 *
 * It is OK to call report_progress(1, -1, NULL) in a subroutine when
 * encountering a fatal error to terminate the reporting here and there even
 * though no report may have been started.
 */

void report_progress(int completed, int total, const char *hdr) {
  static int last;
  static double start_time;
  int percent;
  double t;

  if (update_progress == NULL)
    return;

  percent =
    completed >= total || total <= 0? 100:
    completed < 0? 0:
    completed < INT_MAX/100? 100*completed/total: completed/(total/100);

  t = avr_timestamp();

  if(hdr || !start_time)
    start_time = t;

  if(hdr || percent > last) {
    last = percent;
    update_progress(percent, t - start_time, hdr, total < 0? -1: !!total);
  }
}
