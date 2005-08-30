/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
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
 * Posix serial interface for avrdude.
 */

#if !defined(WIN32NATIVE)


#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "serial.h"

extern char *progname;
extern int verbose;

long serial_recv_timeout = 5000; /* ms */

struct baud_mapping {
  long baud;
  speed_t speed;
};

/* There are a lot more baud rates we could handle, but what's the point? */

static struct baud_mapping baud_lookup_table [] = {
  { 1200,   B1200 },
  { 2400,   B2400 },
  { 4800,   B4800 },
  { 9600,   B9600 },
  { 19200,  B19200 },
  { 38400,  B38400 },
  { 57600,  B57600 },
  { 115200, B115200 },
  { 230400, B230400 },
  { 0,      0 }                 /* Terminator. */
};

static speed_t serial_baud_lookup(long baud)
{
  struct baud_mapping *map = baud_lookup_table;

  while (map->baud) {
    if (map->baud == baud)
      return map->speed;
    map++;
  }

  fprintf(stderr, "%s: serial_baud_lookup(): unknown baud rate: %ld", 
          progname, baud);
  exit(1);
}

static int ser_setspeed(int fd, long baud)
{
  int rc;
  struct termios termios;
  speed_t speed = serial_baud_lookup (baud);
  
  if (!isatty(fd))
    return -1;
  
  /*
   * initialize terminal modes
   */
  rc = tcgetattr(fd, &termios);
  if (rc < 0) {
    fprintf(stderr, "%s: ser_setspeed(): tcgetattr() failed, %s", 
            progname, strerror(errno));
    return -errno;
  }

  termios.c_iflag = 0;
  termios.c_oflag = 0;
  termios.c_cflag = 0;
  termios.c_cflag |=   (CS8 | CREAD | CLOCAL);
  termios.c_lflag = 0;
  termios.c_cc[VMIN]  = 1;
  termios.c_cc[VTIME] = 0;

  cfsetospeed(&termios, speed);
  cfsetispeed(&termios, speed);
  
  rc = tcsetattr(fd, TCSANOW, &termios);
  if (rc < 0) {
    fprintf(stderr, "%s: ser_setspeed(): tcsetattr() failed, %s", 
            progname, strerror(errno));
    return -errno;
  }

#if 0
  /*
   * set non blocking mode
   */
  rc = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, rc | O_NONBLOCK);
#endif

  return 0;
}


static int ser_open(char * port, long baud)
{
  int rc;
  int fd;

  /*
   * open the serial port
   */
  fd = open(port, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);
  if (fd < 0) {
    fprintf(stderr, "%s: ser_open(): can't open device \"%s\": %s\n",
            progname, port, strerror(errno));
    exit(1);
  }

  /*
   * set serial line attributes
   */
  rc = ser_setspeed(fd, baud);
  if (rc) {
    fprintf(stderr, 
            "%s: ser_open(): can't set attributes for device \"%s\"\n",
            progname, port);
    exit(1);
  }

  return fd;
}


static void ser_close(int fd)
{
  /* FIXME: Should really restore the terminal to original state here. */

  close(fd);
}


static int ser_send(int fd, unsigned char * buf, size_t buflen)
{
  struct timeval timeout, to2;
  fd_set wfds;
  int nfds;
  int rc;
  unsigned char * p = buf;
  size_t len = buflen;

  if (!len)
    return 0;

  if (verbose > 3)
  {
      fprintf(stderr, "%s: Send: ", progname);

      while (buflen) {
        unsigned char c = *buf;
        if (isprint(c)) {
          fprintf(stderr, "%c ", c);
        }
        else {
          fprintf(stderr, ". ");
        }
        fprintf(stderr, "[%02x] ", c);

        buf++;
        buflen--;
      }

      fprintf(stderr, "\n");
  }

  timeout.tv_sec = 0;
  timeout.tv_usec = 500000;
  to2 = timeout;

  while (len) {
  reselect:
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);

    nfds = select(fd+1, NULL, &wfds, NULL, &to2);
    if (nfds == 0) {
      if (verbose >= 1)
	fprintf(stderr,
		"%s: ser_send(): programmer is not responding\n",
		progname);
      exit(1);
    }
    else if (nfds == -1) {
      if (errno == EINTR || errno == EAGAIN) {
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: ser_send(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = write(fd, p, (len > 1024) ? 1024 : len);
    if (rc < 0) {
      fprintf(stderr, "%s: ser_send(): write error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    p += rc;
    len -= rc;
  }

  return 0;
}


static int ser_recv(int fd, unsigned char * buf, size_t buflen)
{
  struct timeval timeout, to2;
  fd_set rfds;
  int nfds;
  int rc;
  unsigned char * p = buf;
  size_t len = 0;

  timeout.tv_sec  = serial_recv_timeout / 1000L;
  timeout.tv_usec = (serial_recv_timeout % 1000L) * 1000;
  to2 = timeout;

  while (len < buflen) {
  reselect:
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    nfds = select(fd+1, &rfds, NULL, NULL, &to2);
    if (nfds == 0) {
      if (verbose > 1)
	fprintf(stderr,
		"%s: ser_recv(): programmer is not responding\n",
		progname);
      return -1;
    }
    else if (nfds == -1) {
      if (errno == EINTR || errno == EAGAIN) {
	fprintf(stderr,
		"%s: ser_recv(): programmer is not responding,reselecting\n",
		progname);
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: ser_recv(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = read(fd, p, (buflen - len > 1024) ? 1024 : buflen - len);
    if (rc < 0) {
      fprintf(stderr, "%s: ser_recv(): read error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    p += rc;
    len += rc;
  }

  p = buf;

  if (verbose > 3)
  {
      fprintf(stderr, "%s: Recv: ", progname);

      while (len) {
        unsigned char c = *p;
        if (isprint(c)) {
          fprintf(stderr, "%c ", c);
        }
        else {
          fprintf(stderr, ". ");
        }
        fprintf(stderr, "[%02x] ", c);

        p++;
        len--;
      }
      fprintf(stderr, "\n");
  }

  return 0;
}


static int ser_drain(int fd, int display)
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
    FD_SET(fd, &rfds);

  reselect:
    nfds = select(fd+1, &rfds, NULL, NULL, &timeout);
    if (nfds == 0) {
      if (display) {
        fprintf(stderr, "<drain\n");
      }
      
      break;
    }
    else if (nfds == -1) {
      if (errno == EINTR) {
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: ser_drain(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = read(fd, &buf, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: ser_drain(): read error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    if (display) {
      fprintf(stderr, "%02x ", buf);
    }
  }

  return 0;
}

struct serial_device serial_serdev =
{
  .open = ser_open,
  .setspeed = ser_setspeed,
  .close = ser_close,
  .send = ser_send,
  .recv = ser_recv,
  .drain = ser_drain,
};

struct serial_device *serdev = &serial_serdev;

#endif  /* WIN32NATIVE */
