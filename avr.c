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

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "avrdude.h"

#include "avr.h"
#include "lists.h"
#include "pindefs.h"
#include "ppi.h"
#include "safemode.h"
#include "update.h"

FP_UpdateProgress update_progress;

#define DEBUG 0

int avr_read_byte_default(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem, 
                          unsigned long addr, unsigned char * value)
{
  unsigned char cmd[4];
  unsigned char res[4];
  unsigned char data;
  OPCODE * readop, * lext;

  if (pgm->cmd == NULL) {
    fprintf(stderr,
	    "%s: Error: %s programmer uses avr_read_byte_default() but does not\n"
	    "provide a cmd() method.\n",
	    progname, pgm->type);
    return -1;
  }

  pgm->pgm_led(pgm, ON);
  pgm->err_led(pgm, OFF);

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
    fprintf(stderr, 
            "avr_read_byte(): operation not supported on memory type \"%s\"\n",
            p->desc);
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
    pgm->cmd(pgm, cmd, res);
  }

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(readop, cmd);
  avr_set_addr(readop, cmd, addr);
  pgm->cmd(pgm, cmd, res);
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
 */
int avr_mem_hiaddr(AVRMEM * mem)
{
  int i, n;

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
 * Read the entirety of the specified memory type into the
 * corresponding buffer of the avrpart pointed to by 'p'.  If size =
 * 0, read the entire contents, otherwise, read 'size' bytes.
 *
 * Return the number of bytes read, or < 0 if an error occurs.  
 */
int avr_read(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size, 
             int verbose)
{
  unsigned char    rbyte;
  unsigned long    i;
  unsigned char  * buf;
  AVRMEM * mem;
  int rc;

  mem = avr_locate_mem(p, memtype);
  if (mem == NULL) {
    fprintf(stderr, "No \"%s\" memory for part %s\n",
            memtype, p->desc);
    return -1;
  }

  buf  = mem->buf;
  if (size == 0) {
    size = mem->size;
  }

  /*
   * start with all 0xff
   */
  memset(buf, 0xff, size);

  if ((strcmp(mem->desc, "eeprom")==0) || 
      (strcmp(mem->desc, "flash")==0) ||
      (strcmp(mem->desc, "application")==0) ||
      (strcmp(mem->desc, "apptable")==0) ||
      (strcmp(mem->desc, "boot")==0) ||
      (strcmp(mem->desc, "usersig")==0) ||
      (strcmp(mem->desc, "prodsig")==0)) {
    if (pgm->paged_load != NULL && mem->page_size != 0) {
      /*
       * the programmer supports a paged mode read, perhaps more
       * efficiently than we can read it directly, so use its routine
       * instead
       */
      rc = pgm->paged_load(pgm, p, mem, mem->page_size, size);
      if (rc >= 0) {
	if (strcasecmp(mem->desc, "flash") == 0)
	  return avr_mem_hiaddr(mem);
	else
	  return rc;
      }
    }
  }

  if (strcmp(mem->desc, "signature") == 0) {
    if (pgm->read_sig_bytes) {
      return pgm->read_sig_bytes(pgm, p, mem);
    }
  }

  for (i=0; i<size; i++) {
    rc = pgm->read_byte(pgm, p, mem, i, &rbyte);
    if (rc != 0) {
      fprintf(stderr, "avr_read(): error reading address 0x%04lx\n", i);
      if (rc == -1) 
        fprintf(stderr, 
                "    read operation not supported for memory \"%s\"\n",
                memtype);
      return -2;
    }
    buf[i] = rbyte;
    report_progress(i, size, NULL);
  }

  if (strcasecmp(mem->desc, "flash") == 0)
    return avr_mem_hiaddr(mem);
  else
    return i;
}


/*
 * write a page data at the specified address
 */
int avr_write_page(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem, 
                   unsigned long addr)
{
  unsigned char cmd[4];
  unsigned char res[4];
  OPCODE * wp, * lext;

  if (pgm->cmd == NULL) {
    fprintf(stderr,
	    "%s: Error: %s programmer uses avr_write_page() but does not\n"
	    "provide a cmd() method.\n",
	    progname, pgm->type);
    return -1;
  }

  wp = mem->op[AVR_OP_WRITEPAGE];
  if (wp == NULL) {
    fprintf(stderr, 
            "avr_write_page(): memory \"%s\" not configured for page writes\n",
            mem->desc);
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


int avr_write_byte_default(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
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
  struct timeval tv;

  if (pgm->cmd == NULL) {
    fprintf(stderr,
	    "%s: Error: %s programmer uses avr_write_byte_default() but does not\n"
	    "provide a cmd() method.\n",
	    progname, pgm->type);
    return -1;
  }

  if (!mem->paged) {
    /* 
     * check to see if the write is necessary by reading the existing
     * value and only write if we are changing the value; we can't
     * use this optimization for paged addressing.
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
    fprintf(stderr, 
            "avr_write_byte(): write not supported for memory type \"%s\"\n",
            mem->desc);
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
      gettimeofday (&tv, NULL);
      start_time = (tv.tv_sec * 1000000) + tv.tv_usec;
      do {
        /*
         * Do polling, but timeout after max_write_delay.
	 */
        rc = pgm->read_byte(pgm, p, mem, addr, &r);
        if (rc != 0) {
          pgm->pgm_led(pgm, OFF);
          pgm->err_led(pgm, ON);
          return -4;
        }
        gettimeofday (&tv, NULL);
        prog_time = (tv.tv_sec * 1000000) + tv.tv_usec;
      } while ((r != data) &&
               ((prog_time-start_time) < mem->max_write_delay));
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
      fprintf(stderr,
              "%s: this device must be powered off and back on to continue\n",
              progname);
      if (pgm->pinno[PPI_AVR_VCC]) {
        fprintf(stderr, "%s: attempting to do this now ...\n", progname);
        pgm->powerdown(pgm);
        usleep(250000);
        rc = pgm->initialize(pgm, p);
        if (rc < 0) {
          fprintf(stderr, "%s: initialization failed, rc=%d\n", progname, rc);
          fprintf(stderr, 
                  "%s: can't re-initialize device after programming the "
                  "%s bits\n", progname, mem->desc);
          fprintf(stderr,
                  "%s: you must manually power-down the device and restart\n"
                  "%s:   %s to continue.\n",
                  progname, progname, progname);
          return -3;
        }
        
        fprintf(stderr, "%s: device was successfully re-initialized\n",
                progname);
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
int avr_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * mem,
                   unsigned long addr, unsigned char data)
{

  unsigned char safemode_lfuse;
  unsigned char safemode_hfuse;
  unsigned char safemode_efuse;
  unsigned char safemode_fuse;

  /* If we write the fuses, then we need to tell safemode that they *should* change */
  safemode_memfuses(0, &safemode_lfuse, &safemode_hfuse, &safemode_efuse, &safemode_fuse);

  if (strcmp(mem->desc, "fuse")==0) {
      safemode_fuse = data;
  }
  if (strcmp(mem->desc, "lfuse")==0) {
      safemode_lfuse = data;
  }
  if (strcmp(mem->desc, "hfuse")==0) {
      safemode_hfuse = data;
  }
  if (strcmp(mem->desc, "efuse")==0) {
      safemode_efuse = data;
  }
  
  safemode_memfuses(1, &safemode_lfuse, &safemode_hfuse, &safemode_efuse, &safemode_fuse);

  return pgm->write_byte(pgm, p, mem, addr, data);
}


/*
 * Write the whole memory region of the specified memory from the
 * corresponding buffer of the avrpart pointed to by 'p'.  Write up to
 * 'size' bytes from the buffer.  Data is only written if the new data
 * value is different from the existing data value.  Data beyond
 * 'size' bytes is not affected.
 *
 * Return the number of bytes written, or -1 if an error occurs.
 */
int avr_write(PROGRAMMER * pgm, AVRPART * p, char * memtype, int size, 
              int verbose)
{
  int              rc;
  int              wsize;
  unsigned long    i;
  unsigned char    data;
  int              werror;
  AVRMEM         * m;

  m = avr_locate_mem(p, memtype);
  if (m == NULL) {
    fprintf(stderr, "No \"%s\" memory for part %s\n",
            memtype, p->desc);
    return -1;
  }

  pgm->err_led(pgm, OFF);

  werror  = 0;

  wsize = m->size;
  if (size < wsize) {
    wsize = size;
  }
  else if (size > wsize) {
    fprintf(stderr, 
            "%s: WARNING: %d bytes requested, but memory region is only %d"
            "bytes\n"
            "%sOnly %d bytes will actually be written\n",
            progname, size, wsize,
            progbuf, wsize);
  }

  if ((strcmp(m->desc, "application")==0) || 
      (strcmp(m->desc, "apptable")==0) ||
      (strcmp(m->desc, "boot")==0) ||
      (strcmp(m->desc, "flash")==0) ||
      (strcmp(m->desc, "prodsig")==0) ||
      (strcmp(m->desc, "usersig")==0)) {
    if (pgm->paged_write != NULL && m->page_size != 0) {
      /*
       * the programmer supports a paged mode write, perhaps more
       * efficiently than we can read it directly, so use its routine
       * instead
       */
      if ((i = pgm->paged_write(pgm, p, m, m->page_size, size)) >= 0)
	return i;
    }
  }

  if (pgm->write_setup) {
      pgm->write_setup(pgm, p, m);
  }

  for (i=0; i<wsize; i++) {
    data = m->buf[i];
    report_progress(i, wsize, NULL);

    rc = avr_write_byte(pgm, p, m, i, data);
    if (rc) {
      fprintf(stderr, " ***failed;  ");
      fprintf(stderr, "\n");
      pgm->err_led(pgm, ON);
      werror = 1;
    }

    if (m->paged) {
      /*
       * check to see if it is time to flush the page with a page
       * write
       */
      if (((i % m->page_size) == m->page_size-1) ||
          (i == wsize-1)) {
        rc = avr_write_page(pgm, p, m, i);
        if (rc) {
          fprintf(stderr,
                  " *** page %ld (addresses 0x%04lx - 0x%04lx) failed "
                  "to write\n",
                  i % m->page_size, 
                  i - m->page_size + 1, i);
          fprintf(stderr, "\n");
          pgm->err_led(pgm, ON);
          werror = 1;
        }
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
int avr_signature(PROGRAMMER * pgm, AVRPART * p)
{
  int rc;

  report_progress (0,1,"Reading");
  rc = avr_read(pgm, p, "signature", 0, 0);
  if (rc < 0) {
    fprintf(stderr,
            "%s: error reading signature data for part \"%s\", rc=%d\n",
            progname, p->desc, rc);
    return -1;
  }
  report_progress (1,1,NULL);

  return 0;
}


/*
 * Verify the memory buffer of p with that of v.  The byte range of v,
 * may be a subset of p.  The byte range of p should cover the whole
 * chip's memory size.
 *
 * Return the number of bytes verified, or -1 if they don't match.
 */
int avr_verify(AVRPART * p, AVRPART * v, char * memtype, int size)
{
  int i;
  unsigned char * buf1, * buf2;
  int vsize;
  AVRMEM * a, * b;

  a = avr_locate_mem(p, memtype);
  if (a == NULL) {
    fprintf(stderr, 
            "avr_verify(): memory type \"%s\" not defined for part %s\n",
            memtype, p->desc);
    return -1;
  }

  b = avr_locate_mem(v, memtype);
  if (b == NULL) {
    fprintf(stderr, 
            "avr_verify(): memory type \"%s\" not defined for part %s\n",
            memtype, v->desc);
    return -1;
  }

  buf1  = a->buf;
  buf2  = b->buf;
  vsize = a->size;

  if (vsize < size) {
    fprintf(stderr, 
            "%s: WARNING: requested verification for %d bytes\n"
            "%s%s memory region only contains %d bytes\n"
            "%sOnly %d bytes will be verified.\n",
            progname, size,
            progbuf, memtype, vsize,
            progbuf, vsize);
    size = vsize;
  }

  for (i=0; i<size; i++) {
    if (buf1[i] != buf2[i]) {
      fprintf(stderr, 
              "%s: verification error, first mismatch at byte 0x%04x\n"
              "%s0x%02x != 0x%02x\n",
              progname, i, 
              progbuf, buf1[i], buf2[i]);
      return -1;
    }
  }

  return size;
}


int avr_get_cycle_count(PROGRAMMER * pgm, AVRPART * p, int * cycles)
{
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
    fprintf(stderr, "%s: WARNING: can't read memory for cycle count, rc=%d\n",
            progname, rc);
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


int avr_put_cycle_count(PROGRAMMER * pgm, AVRPART * p, int cycles)
{
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
      fprintf(stderr, "%s: WARNING: can't write memory for cycle count, rc=%d\n",
              progname, rc);
      return -1;
    }
  }

  return 0;
  }

int avr_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  int cycles;
  int rc;

  if (do_cycles) {
    rc = avr_get_cycle_count(pgm, p, &cycles);
    /*
     * Don't update the cycle counter, if read failed
     */
    if(rc != 0) {
      do_cycles = 0;
    }
  }

  rc = pgm->chip_erase(pgm, p);

  /*
   * Don't update the cycle counter, if erase failed
   */
  if (do_cycles && (rc == 0)) {
    cycles++;
    fprintf(stderr, "%s: erase-rewrite cycle count is now %d\n",
            progname, cycles);
    avr_put_cycle_count(pgm, p, cycles);
  }

  return rc;
}

/*
 * Report the progress of a read or write operation from/to the
 * device.
 *
 * The first call of report_progress() should look like this (for a write op):
 *
 * report_progress (0, 1, "Writing");
 *
 * Then hdr should be passed NULL on subsequent calls while the
 * operation is progressing. Once the operation is complete, a final
 * call should be made as such to ensure proper termination of the
 * progress report:
 *
 * report_progress (1, 1, NULL);
 *
 * It would be nice if we could reduce the usage to one and only one
 * call for each of start, during and end cases. As things stand now,
 * that is not possible and makes maintenance a bit more work.
 */
void report_progress (int completed, int total, char *hdr)
{
  static int last = 0;
  static double start_time;
  int percent = (completed * 100) / total;
  struct timeval tv;
  double t;

  if (update_progress == NULL)
    return;

  gettimeofday(&tv, NULL);
  t = tv.tv_sec + ((double)tv.tv_usec)/1000000;

  if (hdr) {
    last = 0;
    start_time = t;
    update_progress (percent, t - start_time, hdr);
  }

  if (percent > 100)
    percent = 100;

  if (percent > last) {
    last = percent;
    update_progress (percent, t - start_time, hdr);
  }

  if (percent == 100)
    last = 0;                   /* Get ready for next time. */
}
