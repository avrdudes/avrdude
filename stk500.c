/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2002, 2003  Brian S. Dean <bsd@bsdhome.com>
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

/*
 * avrdude interface for Atmel STK500 programmer
 *
 * Note: most commands use the "universal command" feature of the
 * programmer in a "pass through" mode, exceptions are "program
 * enable", "paged read", and "paged write".
 *
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "avr.h"
#include "pgm.h"
#include "stk500_private.h"
#include "serial.h"


extern int    verbose;
extern char * progname;
extern int do_cycles;


static int stk500_getparm(PROGRAMMER * pgm, unsigned parm, unsigned * value);
static int stk500_setparm(PROGRAMMER * pgm, unsigned parm, unsigned value);
static void stk500_print_parms1(PROGRAMMER * pgm, char * p);

static int stk500_send(PROGRAMMER * pgm, char * buf, size_t len)
{
  return serial_send(pgm->fd, buf, len);
}


static int stk500_recv(PROGRAMMER * pgm, char * buf, size_t len)
{
  return serial_recv(pgm->fd, buf, len);
}


static int stk500_drain(PROGRAMMER * pgm, int display)
{
  return serial_drain(pgm->fd, display);
}


static int stk500_getsync(PROGRAMMER * pgm)
{
  unsigned char buf[32], resp[32];

  /*
   * get in sync */
  buf[0] = Cmnd_STK_GET_SYNC;
  buf[1] = Sync_CRC_EOP;
  stk500_send(pgm, buf, 2);
  stk500_recv(pgm, resp, 1);
  if (resp[0] != Resp_STK_INSYNC) {
    fprintf(stderr, 
            "%s: stk500_getsync(): not in sync: resp=0x%02x\n",
            progname, resp[0]);
    stk500_drain(pgm, 0);
    exit(1);
  }

  stk500_recv(pgm, resp, 1);
  if (resp[0] != Resp_STK_OK) {
    fprintf(stderr, 
            "%s: stk500_getsync(): can't communicate with device: "
            "resp=0x%02x\n",
            progname, resp[0]);
    exit(1);
  }

  return 0;
}


static int stk500_rdy_led(PROGRAMMER * pgm, int value)
{
  return 0;
}

static int stk500_err_led(PROGRAMMER * pgm, int value)
{
  return 0;
}

static int stk500_pgm_led(PROGRAMMER * pgm, int value)
{
  return 0;
}

static int stk500_vfy_led(PROGRAMMER * pgm, int value)
{
  return 0;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
static int stk500_cmd(PROGRAMMER * pgm, unsigned char cmd[4], 
                      unsigned char res[4])
{
  unsigned char buf[32];

  buf[0] = Cmnd_STK_UNIVERSAL;
  buf[1] = cmd[0];
  buf[2] = cmd[1];
  buf[3] = cmd[2];
  buf[4] = cmd[3];
  buf[5] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 6);

  stk500_recv(pgm, buf, 1);
  if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr, "%s: stk500_cmd(): programmer is out of sync\n", progname);
    exit(1);
  }

  res[0] = cmd[1];
  res[1] = cmd[2];
  res[2] = cmd[3];
  stk500_recv(pgm, &res[3], 1);

  stk500_recv(pgm, buf, 1);
  if (buf[0] != Resp_STK_OK) {
    fprintf(stderr, "%s: stk500_cmd(): protocol error\n", progname);
    exit(1);
  }

  return 0;
}



/*
 * issue the 'chip erase' command to the AVR device
 */
static int stk500_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char cmd[4];
  unsigned char res[4];
  int cycles;
  int rc;

  if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
    fprintf(stderr, "chip erase instruction not defined for part \"%s\"\n",
            p->desc);
    return -1;
  }

  rc = avr_get_cycle_count(pgm, p, &cycles);

  /*
   * only print out the current cycle count if we aren't going to
   * display it below 
   */
  if (!do_cycles && ((rc >= 0) && (cycles != 0xffffffff))) {
    fprintf(stderr,
            "%s: current erase-rewrite cycle count is %d%s\n",
            progname, cycles, 
            do_cycles ? "" : " (if being tracked)");
  }

  pgm->pgm_led(pgm, ON);

  memset(cmd, 0, sizeof(cmd));

  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);

  pgm->pgm_led(pgm, OFF);

  if (do_cycles && (cycles != -1)) {
    if (cycles == 0x00ffff) {
      cycles = 0;
    }
    cycles++;
    fprintf(stderr, "%s: erase-rewrite cycle count is now %d\n", 
            progname, cycles);
    avr_put_cycle_count(pgm, p, cycles);
  }

  return 0;
}

/*
 * issue the 'program enable' command to the AVR device
 */
static int stk500_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[16];
  int tries=0;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_ENTER_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 2);
  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "%s: stk500_program_enable(): can't get into sync\n",
              progname);
      return -1;
    }
    stk500_getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "%s: stk500_program_enable(): protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_OK) {
    return 0;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    fprintf(stderr, "%s: stk500_program_enable(): no device\n",
            progname);
    return -1;
  }

  if(buf[0] == Resp_STK_FAILED)
  {
      fprintf(stderr, 
	      "%s: stk500_program_enable(): failed to enter programming mode\n", 
		  progname);
	  return -1;
  }


  fprintf(stderr, "%s: stk500_program_enable(): unknown response=0x%02x\n",
          progname, buf[0]);

  return -1;
}


/*
 * apply power to the AVR processor
 */
static void stk500_powerup(PROGRAMMER * pgm)
{
  return;
}


/*
 * remove power from the AVR processor
 */
static void stk500_powerdown(PROGRAMMER * pgm)
{
  return;
}


static int stk500_set_extended_parms(PROGRAMMER * pgm, int n, 
                                     unsigned char * cmd)
{
  unsigned char buf[16];
  int tries=0;
  int i;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_SET_DEVICE_EXT;
  for (i=0; i<n; i++) {
    buf[1+i] = cmd[i];
  }
  i++;
  buf[i] = Sync_CRC_EOP;

  stk500_send(pgm, buf, i+1);
  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "%s: stk500_set_extended_parms(): can't get into sync\n",
              progname);
      return -1;
    }
    stk500_getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "%s: stk500_set_extended_parms(): protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_OK) {
    return 0;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    fprintf(stderr, "%s: stk500_set_extended_parms(): no device\n",
            progname);
    return -1;
  }

  if(buf[0] == Resp_STK_FAILED)
  {
      fprintf(stderr, 
	      "%s: stk500_set_extended_parms(): failed to set extended "
              "device programming parameters\n", 
              progname);
	  return -1;
  }


  fprintf(stderr, "%s: stk500_set_extended_parms(): unknown response=0x%02x\n",
          progname, buf[0]);

  return -1;
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
static int stk500_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[32];
  AVRMEM * m;
  int tries;
  unsigned maj, min;
  int rc;
  int n_extparms = 3;

  stk500_getparm(pgm, Parm_STK_SW_MAJOR, &maj);
  stk500_getparm(pgm, Parm_STK_SW_MINOR, &min);

  if ((maj > 1) || ((maj == 1) && (min > 10)))
    n_extparms = 4;

  tries = 0;

 retry:
  tries++;

  memset(buf, 0, sizeof(buf));

  /*
   * set device programming parameters
   */
  buf[0] = Cmnd_STK_SET_DEVICE;

  buf[1] = p->stk500_devcode;
  buf[2] = 0; /* device revision */

  if ((p->flags & AVRPART_SERIALOK) && (p->flags & AVRPART_PARALLELOK))
    buf[3] = 0; /* device supports parallel and serial programming */
  else
    buf[3] = 1; /* device supports parallel only */

  if (p->flags & AVRPART_PARALLELOK) {
    if (p->flags & AVRPART_PSEUDOPARALLEL) {
      buf[4] = 0; /* pseudo parallel interface */
      n_extparms = 0;
    }
    else {
      buf[4] = 1; /* full parallel interface */
    }
  }

#if 0
  fprintf(stderr, "%s: stk500_initialize(): n_extparms = %d\n", 
          progname, n_extparms);
#endif
    
  buf[5] = 1; /* polling supported - XXX need this in config file */
  buf[6] = 1; /* programming is self-timed - XXX need in config file */

  m = avr_locate_mem(p, "lock");
  if (m)
    buf[7] = m->size;
  else
    buf[7] = 0;

  /*
   * number of fuse bytes
   */
  buf[8] = 0;
  m = avr_locate_mem(p, "fuse");
  if (m)
    buf[8] += m->size;
  m = avr_locate_mem(p, "lfuse");
  if (m)
    buf[8] += m->size;
  m = avr_locate_mem(p, "hfuse");
  if (m)
    buf[8] += m->size;
  m = avr_locate_mem(p, "efuse");
  if (m)
    buf[8] += m->size;

  m = avr_locate_mem(p, "flash");
  if (m) {
    buf[9] = m->readback[0];
    buf[10] = m->readback[1];
    if (m->paged) {
      buf[13] = (m->page_size >> 8) & 0x00ff;
      buf[14] = m->page_size & 0x00ff;
    }
    buf[17] = (m->size >> 24) & 0xff;
    buf[18] = (m->size >> 16) & 0xff;
    buf[19] = (m->size >> 8) & 0xff;
    buf[20] = m->size & 0xff;
  }
  else {
    buf[9]  = 0xff;
    buf[10]  = 0xff;
    buf[13] = 0;
    buf[14] = 0;
    buf[17] = 0;
    buf[18] = 0;
    buf[19] = 0;
    buf[20] = 0;
  }

  m = avr_locate_mem(p, "eeprom");
  if (m) {
    buf[11] = m->readback[0];
    buf[12] = m->readback[1];
    buf[15] = (m->size >> 8) & 0x00ff;
    buf[16] = m->size & 0x00ff;
  }
  else {
    buf[11] = 0xff;
    buf[12] = 0xff;
    buf[15] = 0;
    buf[16] = 0;
  }

  buf[21] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 22);
  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    fprintf(stderr,
            "%s: stk500_initialize(): programmer not in sync, resp=0x%02x\n", 
            progname, buf[0]);
    if (tries > 33)
      return -1;
    stk500_getsync(pgm);
    goto retry;
    return -1;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "%s: stk500_initialize(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  stk500_recv(pgm, buf, 1);
  if (buf[0] != Resp_STK_OK) {
    fprintf(stderr,
            "%s: stk500_initialize(): (b) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_OK, buf[0]);
    return -1;
  }

  if (n_extparms) {
    if ((p->pagel == 0) || (p->bs2 == 0)) {
      fprintf(stderr, 
              "%s: please define PAGEL and BS2 signals in the configuration "
              "file for part %s\n", 
              progname, p->desc);
    }
    else {
      buf[0] = n_extparms+1;

      /*
       * m is currently pointing to eeprom memory if the part has it
       */
      if (m)
        buf[1] = m->page_size;
      else
        buf[1] = 0;
      
      buf[2] = p->pagel;
      buf[3] = p->bs2;
      
      if (n_extparms == 4) {
        if (p->reset_disposition == RESET_DEDICATED)
          buf[4] = 0;
        else
          buf[4] = 1;
      }
      
      rc = stk500_set_extended_parms(pgm, n_extparms+1, buf);
      if (rc) {
        fprintf(stderr, "%s: stk500_initialize(): failed\n", progname);
        exit(1);
      }
    }
  }

  pgm->program_enable(pgm, p);

  /*
   * Return success even if program_enable() failed.  Otherwise, if
   * someone has turned off the STK500 oscillator (or set it to an
   * unreasonably slow master clock), they were hosed at this point,
   * since to reset fosc to a reasonable value, they at least need to
   * get avrdude to start up in terminal mode.  The luser has already
   * seen a "failed to enter programming mode" message in that case,
   * and he also needs to specify -F at program startup since the
   * device ID bytes cannot be read in this situation.
   */
  return 0;
}


static int stk500_save(PROGRAMMER * pgm)
{
  return 0;
}

static void stk500_restore(PROGRAMMER * pgm)
{
  return;
}

static void stk500_disable(PROGRAMMER * pgm)
{
  unsigned char buf[16];
  int tries=0;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_LEAVE_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 2);
  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "%s: stk500_disable(): can't get into sync\n",
              progname);
      return;
    }
    stk500_getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "%s: stk500_disable(): protocol error, expect=0x%02x, "
            "resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return;
  }

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_OK) {
    return;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    fprintf(stderr, "%s: stk500_disable(): no device\n",
            progname);
    return;
  }

  fprintf(stderr, "%s: stk500_disable(): unknown response=0x%02x\n",
          progname, buf[0]);

  return;
}

static void stk500_enable(PROGRAMMER * pgm)
{
  return;
}


static void stk500_open(PROGRAMMER * pgm, char * port)
{
  strcpy(pgm->port, port);
  pgm->fd = serial_open(port, 115200);

  /*
   * drain any extraneous input
   */
  stk500_drain(pgm, 0);

  stk500_getsync(pgm);

  stk500_drain(pgm, 0);
}


static void stk500_close(PROGRAMMER * pgm)
{
  serial_close(pgm->fd);
  pgm->fd = -1;
}


static int stk500_loadaddr(PROGRAMMER * pgm, unsigned int addr)
{
  unsigned char buf[16];
  int tries;

  tries = 0;
 retry:
  tries++;
  buf[0] = Cmnd_STK_LOAD_ADDRESS;
  buf[1] = addr & 0xff;
  buf[2] = (addr >> 8) & 0xff;
  buf[3] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 4);

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "%s: stk500_loadaddr(): can't get into sync\n",
              progname);
      return -1;
    }
    stk500_getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "%s: stk500_loadaddr(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_OK) {
    return 0;
  }

  fprintf(stderr,
          "%s: loadaddr(): (b) protocol error, "
          "expect=0x%02x, resp=0x%02x\n", 
          progname, Resp_STK_INSYNC, buf[0]);

  return -1;
}


static int stk500_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  unsigned char buf[16];
  int memtype;
  unsigned int addr;
  int a_div;
  int i;
  int tries;
  unsigned int n;

  if (page_size == 0) {
    page_size = 128;
  }

  if (strcmp(m->desc, "flash") == 0) {
    memtype = 'F';
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    memtype = 'E';
  }
  else {
    return -2;
  }

  if ((m->op[AVR_OP_LOADPAGE_LO]) || (m->op[AVR_OP_READ_LO]))
    a_div = 2;
  else
    a_div = 1;

  if (n_bytes > m->size) {
    n_bytes = m->size;
    n = m->size;
  }
  else {
    if ((n_bytes % page_size) != 0) {
      n = n_bytes + page_size - (n_bytes % page_size);
    }
    else {
      n = n_bytes;
    }
  }

#if 0
  fprintf(stderr, 
          "n_bytes   = %d\n"
          "n         = %u\n"
          "a_div     = %d\n"
          "page_size = %d\n",
          n_bytes, n, a_div, page_size);
#endif     

  for (addr = 0; addr < n; addr += page_size) {
    report_progress (addr, n_bytes, NULL);
    tries = 0;
  retry:
    tries++;
    stk500_loadaddr(pgm, addr/a_div);
    buf[0] = Cmnd_STK_PROG_PAGE;
    buf[1] = (page_size >> 8) & 0xff;
    buf[2] = page_size & 0xff;
    buf[3] = memtype;
    stk500_send(pgm, buf, 4);
    for (i=0; i<page_size; i++) {
      buf[0] = m->buf[addr + i];
      stk500_send(pgm, buf, 1);
    }
    buf[0] = Sync_CRC_EOP;
    stk500_send(pgm, buf, 1);

    stk500_recv(pgm, buf, 1);
    if (buf[0] == Resp_STK_NOSYNC) {
      if (tries > 33) {
        fprintf(stderr, "\n%s: stk500_paged_write(): can't get into sync\n",
                progname);
        return -3;
      }
      stk500_getsync(pgm);
      goto retry;
    }
    else if (buf[0] != Resp_STK_INSYNC) {
      fprintf(stderr,
              "\n%s: stk500_paged_write(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -4;
    }
    
    stk500_recv(pgm, buf, 1);
    if (buf[0] != Resp_STK_OK) {
      fprintf(stderr,
              "\n%s: stk500_paged_write(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -5;
    }
  }

  return n;
}


static int stk500_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                             int page_size, int n_bytes)
{
  unsigned char buf[16];
  int memtype;
  unsigned int addr;
  int a_div;
  int tries;
  unsigned int n;

  if (strcmp(m->desc, "flash") == 0) {
    memtype = 'F';
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    memtype = 'E';
  }
  else {
    return -2;
  }

  if ((m->op[AVR_OP_LOADPAGE_LO]) || (m->op[AVR_OP_READ_LO]))
    a_div = 2;
  else
    a_div = 1;

  if (n_bytes > m->size) {
    n_bytes = m->size;
    n = m->size;
  }
  else {
    if ((n_bytes % page_size) != 0) {
      n = n_bytes + page_size - (n_bytes % page_size);
    }
    else {
      n = n_bytes;
    }
  }

  for (addr = 0; addr < n; addr += page_size) {
    report_progress (addr, n_bytes, NULL);
    tries = 0;
  retry:
    tries++;
    stk500_loadaddr(pgm, addr/a_div);
    buf[0] = Cmnd_STK_READ_PAGE;
    buf[1] = (page_size >> 8) & 0xff;
    buf[2] = page_size & 0xff;
    buf[3] = memtype;
    buf[4] = Sync_CRC_EOP;
    stk500_send(pgm, buf, 5);

    stk500_recv(pgm, buf, 1);
    if (buf[0] == Resp_STK_NOSYNC) {
      if (tries > 33) {
        fprintf(stderr, "\n%s: stk500_paged_load(): can't get into sync\n",
                progname);
        return -3;
      }
      stk500_getsync(pgm);
      goto retry;
    }
    else if (buf[0] != Resp_STK_INSYNC) {
      fprintf(stderr,
              "\n%s: stk500_paged_load(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -4;
    }

    stk500_recv(pgm, &m->buf[addr], page_size);

    stk500_recv(pgm, buf, 1);
    if (buf[0] != Resp_STK_OK) {
      fprintf(stderr,
              "\n%s: stk500_paged_load(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -5;
    }
  }

  return n;
}


static int stk500_set_vtarget(PROGRAMMER * pgm, double v)
{
  unsigned uaref, utarg;

  utarg = (unsigned)((v + 0.049) * 10);

  if (stk500_getparm(pgm, Parm_STK_VADJUST, &uaref) != 0) {
    fprintf(stderr,
	    "%s: stk500_set_vtarget(): cannot obtain V[aref]\n",
	    progname);
    return -1;
  }

  if (uaref > utarg) {
    fprintf(stderr,
	    "%s: stk500_set_vtarget(): reducing V[aref] from %.1f to %.1f\n",
	    progname, uaref / 10.0, v);
    if (stk500_setparm(pgm, Parm_STK_VADJUST, utarg)
	!= 0)
      return -1;
  }
  return stk500_setparm(pgm, Parm_STK_VTARGET, utarg);
}


static int stk500_set_varef(PROGRAMMER * pgm, double v)
{
  unsigned uaref, utarg;

  uaref = (unsigned)((v + 0.049) * 10);

  if (stk500_getparm(pgm, Parm_STK_VTARGET, &utarg) != 0) {
    fprintf(stderr,
	    "%s: stk500_set_varef(): cannot obtain V[target]\n",
	    progname);
    return -1;
  }

  if (uaref > utarg) {
    fprintf(stderr,
	    "%s: stk500_set_varef(): V[aref] must not be greater than "
	    "V[target] = %.1f\n",
	    progname, utarg / 10.0);
    return -1;
  }
  return stk500_setparm(pgm, Parm_STK_VADJUST, uaref);
}


static int stk500_set_fosc(PROGRAMMER * pgm, double v)
{
#define fbase 7372800U
  unsigned prescale, cmatch, fosc;
  static unsigned ps[] = {
    1, 8, 32, 64, 128, 256, 1024
  };
  int idx, rc;

  if (v <= 0.0) {
    prescale = cmatch = 0;
    goto setclock;
  }
  if (v > fbase / 2) {
    const char *unit;
    if (v > 1e6) {
      v /= 1e6;
      unit = "MHz";
    } else if (v > 1e3) {
      v /= 1e3;
      unit = "kHz";
    } else
      unit = "Hz";
    fprintf(stderr,
	    "%s: stk500_set_fosc(): f = %.3f %s too high, using %.3f MHz\n",
	    progname, v, unit, fbase / 2e6);
    fosc = fbase / 2;
  } else
    fosc = (unsigned)v;

  for (idx = 0; idx < sizeof(ps) / sizeof(ps[0]); idx++) {
    if (fosc >= fbase / (256 * ps[idx] * 2)) {
      /* this prescaler value can handle our frequency */
      prescale = idx + 1;
      cmatch = (unsigned)(fbase / (2 * v * ps[idx]));
      break;
    }
  }
  if (idx == sizeof(ps) / sizeof(ps[0])) {
    fprintf(stderr, "%s: stk500_set_fosc(): f = %u Hz too low, %u Hz min\n",
	    progname, fosc, fbase / (256 * 1024 * 2));
    return -1;
  }
 setclock:
  if ((rc = stk500_setparm(pgm, Parm_STK_OSC_PSCALE, prescale)) != 0
      || (rc = stk500_setparm(pgm, Parm_STK_OSC_CMATCH, cmatch)) != 0)
    return rc;
  
  return 0;
}


static int stk500_getparm(PROGRAMMER * pgm, unsigned parm, unsigned * value)
{
  unsigned char buf[16];
  unsigned v;
  int tries = 0;

 retry:
  tries++;
  buf[0] = Cmnd_STK_GET_PARAMETER;
  buf[1] = parm;
  buf[2] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 3);

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "\n%s: stk500_getparm(): can't get into sync\n",
              progname);
      return -1;
    }
    stk500_getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "\n%s: stk500_getparm(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -2;
  }

  stk500_recv(pgm, buf, 1);
  v = buf[0];

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_FAILED) {
    fprintf(stderr,
            "\n%s: stk500_getparm(): parameter 0x%02x failed\n",
            progname, v);
    return -3;
  }
  else if (buf[0] != Resp_STK_OK) {
    fprintf(stderr,
            "\n%s: stk500_getparm(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -3;
  }

  *value = v;

  return 0;
}

  
static int stk500_setparm(PROGRAMMER * pgm, unsigned parm, unsigned value)
{
  unsigned char buf[16];
  int tries = 0;

 retry:
  tries++;
  buf[0] = Cmnd_STK_SET_PARAMETER;
  buf[1] = parm;
  buf[2] = value;
  buf[3] = Sync_CRC_EOP;

  stk500_send(pgm, buf, 4);

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "\n%s: stk500_setparm(): can't get into sync\n",
              progname);
      return -1;
    }
    stk500_getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "\n%s: stk500_setparm(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -2;
  }

  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_OK)
    return 0;

  parm = buf[0];	/* if not STK_OK, we've been echoed parm here */
  stk500_recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_FAILED) {
    fprintf(stderr,
            "\n%s: stk500_setparm(): parameter 0x%02x failed\n",
            progname, parm);
    return -3;
  }
  else {
    fprintf(stderr,
            "\n%s: stk500_setparm(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -3;
  }
}

  
static void stk500_display(PROGRAMMER * pgm, char * p)
{
  unsigned maj, min, hdw, topcard;

  stk500_getparm(pgm, Parm_STK_HW_VER, &hdw);
  stk500_getparm(pgm, Parm_STK_SW_MAJOR, &maj);
  stk500_getparm(pgm, Parm_STK_SW_MINOR, &min);
  stk500_getparm(pgm, Param_STK500_TOPCARD_DETECT, &topcard);

  fprintf(stderr, "%sHardware Version: %d\n", p, hdw);
  fprintf(stderr, "%sFirmware Version: %d.%d\n", p, maj, min);
  if (topcard < 3) {
    const char *n = "Unknown";

    switch (topcard) {
      case 1:
	n = "STK502";
	break;

      case 2:
	n = "STK501";
	break;
    }
    fprintf(stderr, "%sTopcard         : %s\n", p, n);
  }
  stk500_print_parms1(pgm, p);

  return;
}


static void stk500_print_parms1(PROGRAMMER * pgm, char * p)
{
  unsigned vtarget, vadjust, osc_pscale, osc_cmatch;

  stk500_getparm(pgm, Parm_STK_VTARGET, &vtarget);
  stk500_getparm(pgm, Parm_STK_VADJUST, &vadjust);
  stk500_getparm(pgm, Parm_STK_OSC_PSCALE, &osc_pscale);
  stk500_getparm(pgm, Parm_STK_OSC_CMATCH, &osc_cmatch);

  fprintf(stderr, "%sVtarget         : %.1f V\n", p, vtarget / 10.0);
  fprintf(stderr, "%sVaref           : %.1f V\n", p, vadjust / 10.0);
  fprintf(stderr, "%sOscillator      : ", p);
  if (osc_pscale == 0)
    fprintf(stderr, "Off\n");
  else {
    int prescale = 1;
    double f = 3.6864e6;
    const char *unit;

    switch (osc_pscale) {
      case 2: prescale = 8; break;
      case 3: prescale = 32; break;
      case 4: prescale = 64; break;
      case 5: prescale = 128; break;
      case 6: prescale = 256; break;
      case 7: prescale = 1024; break;
    }
    f /= prescale;
    f /= (osc_cmatch + 1);
    if (f > 1e6) {
      f /= 1e6;
      unit = "MHz";
    } else if (f > 1e3) {
      f /= 1000;
      unit = "kHz";
    } else
      unit = "Hz";
    fprintf(stderr, "%.3f %s\n", f, unit);
  }

  return;
}


static void stk500_print_parms(PROGRAMMER * pgm)
{
  stk500_print_parms1(pgm, "");
}


void stk500_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK500");

  /*
   * mandatory functions
   */
  pgm->rdy_led        = stk500_rdy_led;
  pgm->err_led        = stk500_err_led;
  pgm->pgm_led        = stk500_pgm_led;
  pgm->vfy_led        = stk500_vfy_led;
  pgm->initialize     = stk500_initialize;
  pgm->display        = stk500_display;
  pgm->save           = stk500_save;
  pgm->restore        = stk500_restore;
  pgm->enable         = stk500_enable;
  pgm->disable        = stk500_disable;
  pgm->powerup        = stk500_powerup;
  pgm->powerdown      = stk500_powerdown;
  pgm->program_enable = stk500_program_enable;
  pgm->chip_erase     = stk500_chip_erase;
  pgm->cmd            = stk500_cmd;
  pgm->open           = stk500_open;
  pgm->close          = stk500_close;

  /*
   * optional functions
   */
  pgm->paged_write    = stk500_paged_write;
  pgm->paged_load     = stk500_paged_load;
  pgm->print_parms    = stk500_print_parms;
  pgm->set_vtarget    = stk500_set_vtarget;
  pgm->set_varef      = stk500_set_varef;
  pgm->set_fosc       = stk500_set_fosc;
  pgm->page_size      = 256;
}
