/*
 * Copyright 2002  Brian S. Dean <bsd@bsdhome.com>
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
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/time.h>

#include "avr.h"
#include "pgm.h"
#include "stk500_private.h"


extern char * progname;
extern int do_cycles;

char * stk500_version = "$Id$";


int static send(PROGRAMMER * pgm, char * buf, int buflen)
{
  struct timeval timeout;
  fd_set wfds;
  int nfds;
  int rc;

  if (!buflen)
    return 0;

  timeout.tv_sec = 0;
  timeout.tv_usec = 500000;

  while (buflen) {
    FD_ZERO(&wfds);
    FD_SET(pgm->fd, &wfds);

  reselect:
    nfds = select(pgm->fd+1, NULL, &wfds, NULL, &timeout);
    if (nfds == 0) {
      fprintf(stderr, "%s: send(): programmer is not responding on %s\n",
              progname, pgm->port);
      exit(1);
    }
    else if (nfds == -1) {
      if (errno == EINTR) {
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: send(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = write(pgm->fd, buf, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: send(): write error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    buf++;
    buflen--;
  }

  return 0;
}

      
  
int static recv(PROGRAMMER * pgm, char * buf, int n)
{
  struct timeval timeout;
  fd_set rfds;
  int nfds;
  int rc;

  timeout.tv_sec = 0;
  timeout.tv_usec = 500000;

  while (n) {
    FD_ZERO(&rfds);
    FD_SET(pgm->fd, &rfds);

  reselect:
    nfds = select(pgm->fd+1, &rfds, NULL, NULL, &timeout);
    if (nfds == 0) {
      fprintf(stderr, "%s: recv(): programmer is not responding on %s\n",
              progname, pgm->port);
      exit(1);
    }
    else if (nfds == -1) {
      if (errno == EINTR) {
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: recv(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = read(pgm->fd, buf, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: recv(): read error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    buf++;
    n--;
  }

  return 0;
}

      
int static drain(PROGRAMMER * pgm, int display)
{
  struct timeval timeout;
  fd_set rfds;
  int nfds;
  int rc;
  unsigned char buf;

  timeout.tv_sec = 0;
  timeout.tv_usec = 250000;

  if (display) {
    fprintf(stderr, "drain>");
  }

  while (1) {
    FD_ZERO(&rfds);
    FD_SET(pgm->fd, &rfds);

  reselect:
    nfds = select(pgm->fd+1, &rfds, NULL, NULL, &timeout);
    if (nfds == 0) {
      if (display) {
        fprintf(stderr, "<drain\n");
      }
      
      return 0;
    }
    else if (nfds == -1) {
      if (errno == EINTR) {
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: drain(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = read(pgm->fd, &buf, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: drain(): read error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    if (display) {
      fprintf(stderr, "%02x ", buf);
    }
  }
}


static int getsync(PROGRAMMER * pgm)
{
  unsigned char buf[32], resp[32];

  /*
   * get in sync
   */
  buf[0] = Cmnd_STK_GET_SYNC;
  buf[1] = Sync_CRC_EOP;
  send(pgm, buf, 2);
  recv(pgm, resp, 1);
  if (resp[0] != Resp_STK_INSYNC) {
    fprintf(stderr, 
            "%s: stk500_open(): not in sync: resp=0x%02x\n",
            progname, resp[0]);
    drain(pgm, 0);
    exit(1);
  }

  recv(pgm, resp, 1);
  if (resp[0] != Resp_STK_OK) {
    fprintf(stderr, 
            "%s: stk500_open(): can't communicate with device: resp=0x%02x\n",
            progname, resp[0]);
    exit(1);
  }

  return 0;
}


int stk500_rdy_led(PROGRAMMER * pgm, int value)
{
  return 0;
}

int stk500_err_led(PROGRAMMER * pgm, int value)
{
  return 0;
}

int stk500_pgm_led(PROGRAMMER * pgm, int value)
{
  return 0;
}

int stk500_vfy_led(PROGRAMMER * pgm, int value)
{
  return 0;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
int stk500_cmd(PROGRAMMER * pgm, unsigned char cmd[4], unsigned char res[4])
{
  unsigned char buf[32];

  buf[0] = Cmnd_STK_UNIVERSAL;
  buf[1] = cmd[0];
  buf[2] = cmd[1];
  buf[3] = cmd[2];
  buf[4] = cmd[3];
  buf[5] = Sync_CRC_EOP;

  send(pgm, buf, 6);

  recv(pgm, buf, 1);
  if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr, "%s: programmer is out of sync\n", progname);
    exit(1);
  }

  res[0] = cmd[1];
  res[1] = cmd[2];
  res[2] = cmd[3];
  recv(pgm, &res[3], 1);

  recv(pgm, buf, 1);
  if (buf[0] != Resp_STK_OK) {
    fprintf(stderr, "%s: protocol error\n", progname);
    exit(1);
  }

  return 0;
}



/*
 * issue the 'chip erase' command to the AVR device
 */
int stk500_chip_erase(PROGRAMMER * pgm, AVRPART * p)
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
int stk500_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[16];
  int tries=0;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_ENTER_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  send(pgm, buf, 2);
  recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "%s: stk500_program_enable(): can't get into sync\n",
              progname);
      return -1;
    }
    getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "%s: stk500_initialize(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return -1;
  }

  recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_OK) {
    return 0;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    fprintf(stderr, "%s: stk500_program_enable(): no device\n",
            progname);
    return -1;
  }

  fprintf(stderr, "%s: stk500_program_enamble(): unkown response=0x%02x\n",
          progname, buf[0]);

  return -1;
}


/*
 * apply power to the AVR processor
 */
void stk500_powerup(PROGRAMMER * pgm)
{
  return;
}


/*
 * remove power from the AVR processor
 */
void stk500_powerdown(PROGRAMMER * pgm)
{
  return;
}


/*
 * initialize the AVR device and prepare it to accept commands
 */
int stk500_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char buf[32];
  AVRMEM * m;
  int tries;

  /*
   * set device programming parameters
   */
  buf[0] = Cmnd_STK_SET_DEVICE;

  buf[1] = p->devicecode;
  buf[2] = 0; /* device revision */
  buf[3] = 0; /* parallel and serial programming */
  buf[4] = 1; /* full parallel interface */
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

  tries = 0;
 retry:
  tries++;
  send(pgm, buf, 22);
  recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    fprintf(stderr,
            "%s: stk500_initialize(): programmer not in sync, resp=0x%02x\n", 
            progname, buf[0]);
    if (tries > 33)
      return -1;
    getsync(pgm);
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

  recv(pgm, buf, 1);
  if (buf[0] != Resp_STK_OK) {
    fprintf(stderr,
            "%s: stk500_initialize(): (b) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_OK, buf[0]);
    return -1;
  }

  return pgm->program_enable(pgm, p);
}


int stk500_save(PROGRAMMER * pgm)
{
  return 0;
}

void stk500_restore(PROGRAMMER * pgm)
{
  return;
}

void stk500_disable(PROGRAMMER * pgm)
{
  unsigned char buf[16];
  int tries=0;

 retry:
  
  tries++;

  buf[0] = Cmnd_STK_LEAVE_PROGMODE;
  buf[1] = Sync_CRC_EOP;

  send(pgm, buf, 2);
  recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_NOSYNC) {
    if (tries > 33) {
      fprintf(stderr, "%s: stk500_program_enable(): can't get into sync\n",
              progname);
      return;
    }
    getsync(pgm);
    goto retry;
  }
  else if (buf[0] != Resp_STK_INSYNC) {
    fprintf(stderr,
            "%s: stk500_initialize(): (a) protocol error, "
            "expect=0x%02x, resp=0x%02x\n", 
            progname, Resp_STK_INSYNC, buf[0]);
    return;
  }

  recv(pgm, buf, 1);
  if (buf[0] == Resp_STK_OK) {
    return;
  }
  else if (buf[0] == Resp_STK_NODEVICE) {
    fprintf(stderr, "%s: stk500_program_enable(): no device\n",
            progname);
    return;
  }

  fprintf(stderr, "%s: stk500_program_enamble(): unkown response=0x%02x\n",
          progname, buf[0]);

  return;
}

void stk500_enable(PROGRAMMER * pgm)
{
  return;
}


int static set_tty_attr(int fd)
{
  int rc;
  struct termios termios;
  
  if (!isatty(fd))
    return -1;
  
  /*
   * initialize terminal modes
   */
  rc = tcgetattr(fd, &termios);
  if (rc < 0) {
    fprintf(stderr, "%s: tcgetattr() failed, %s", 
            progname, strerror(errno));
    return -errno;
  }

#if 1
  termios.c_iflag &= ~(INPCK | IXOFF | IXON);
  termios.c_cflag &= ~(HUPCL | CSTOPB | CRTSCTS);
  termios.c_cflag |= (CLOCAL | CREAD);
  termios.c_cc [VMIN] = 1;
  termios.c_cc [VTIME] = 0;
#else
  termios.c_iflag = 0;
  termios.c_oflag = 0;
  termios.c_cflag &= ~ (PARENB | CSIZE | CSTOPB);
  termios.c_cflag |=   (CS8 | HUPCL | CREAD | CLOCAL);
  termios.c_lflag = 0;
  termios.c_cc[VMIN]  = 1;
  termios.c_cc[VTIME] = 0;
#endif

  cfsetospeed(&termios, B115200);
  cfsetispeed(&termios, B115200);
  
  rc = tcsetattr(fd, TCSANOW, &termios);
  if (rc < 0) {
    fprintf(stderr, "%s: tcsetattr() failed, %s", progname, strerror(errno));
    return -errno;
  }

  return 0;
}


void stk500_open(PROGRAMMER * pgm, char * port)
{
  int rc;

  strcpy(pgm->port, port);

  /*
   * open the serial port
   */
  pgm->fd = open(port, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);
  if (pgm->fd < 0) {
    fprintf(stderr, "%s: can't open device \"%s\": %s\n",
            progname, port, strerror(errno));
    exit(1);
  }

  /*
   * set serial line attributes
   */
  rc = set_tty_attr(pgm->fd);
  if (rc) {
    fprintf(stderr, "%s: can't set attributes for device \"%s\"\n",
            progname, port);
    exit(1);
  }

  /*
   * drain any extraneous input
   */
  drain(pgm, 0);

  getsync(pgm);

  drain(pgm, 0);
}


void stk500_close(PROGRAMMER * pgm)
{
  close(pgm->fd);
  pgm->fd = -1;
}


void stk500_display(PROGRAMMER * pgm, char * p)
{
  return;
}


void stk500_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "STK500");

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
}


