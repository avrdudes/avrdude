/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003  Theodore A. Roth  <troth@openavr.org>
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

extern char *progname;
extern int verbose;

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

static int serial_setattr(int fd, long baud)
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
    fprintf(stderr, "%s: serial_setattr(): tcgetattr() failed, %s", 
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
    fprintf(stderr, "%s: serial_setattr(): tcsetattr() failed, %s", 
            progname, strerror(errno));
    return -errno;
  }

  return 0;
}


int serial_open(char * port, int baud)
{
  int rc;
  int fd;

  /*
   * open the serial port
   */
  fd = open(port, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);
  if (fd < 0) {
    fprintf(stderr, "%s: serial_open(): can't open device \"%s\": %s\n",
            progname, port, strerror(errno));
    exit(1);
  }

  /*
   * set serial line attributes
   */
  rc = serial_setattr(fd, baud);
  if (rc) {
    fprintf(stderr, 
            "%s: serial_open(): can't set attributes for device \"%s\"\n",
            progname, port);
    exit(1);
  }

  return fd;
}


void serial_close(int fd)
{
  /* FIXME: Should really restore the terminal to original state here. */

  close(fd);
}


int serial_send(int fd, char * buf, size_t buflen)
{
  struct timeval timeout;
  fd_set wfds;
  int nfds;
  int rc;

  char * p = buf;
  size_t len = buflen;

  if (!len)
    return 0;

  if (verbose > 3)
  {
      fprintf(stderr, "%s: Send: ", progname);

      while (buflen) {
        char c = *buf;
        if (isprint(c)) {
          fprintf(stderr, "%c ", c);
        }
        else {
          fprintf(stderr, ". ");
        }
        fprintf(stderr, "[%02x] ", (unsigned int)c);

        buf++;
        buflen--;
      }

      fprintf(stderr, "\n");
  }

  timeout.tv_sec = 0;
  timeout.tv_usec = 500000;

  while (len) {
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);

  reselect:
    nfds = select(fd+1, NULL, &wfds, NULL, &timeout);
    if (nfds == 0) {
      fprintf(stderr,
              "%s: serial_send(): programmer is not responding\n",
              progname);
      exit(1);
    }
    else if (nfds == -1) {
      if (errno == EINTR) {
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: serial_send(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = write(fd, p, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: serial_send(): write error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    p++;
    len--;
  }

  return 0;
}


int serial_recv(int fd, char * buf, size_t buflen)
{
  struct timeval timeout;
  fd_set rfds;
  int nfds;
  int rc;

  char * p = buf;
  size_t len = 0;

  timeout.tv_sec  = 5;
  timeout.tv_usec = 0;

  while (len < buflen) {
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

  reselect:
    nfds = select(fd+1, &rfds, NULL, NULL, &timeout);
    if (nfds == 0) {
      fprintf(stderr, 
              "%s: serial_recv(): programmer is not responding\n",
              progname);
      exit(1);
    }
    else if (nfds == -1) {
      if (errno == EINTR) {
        goto reselect;
      }
      else {
        fprintf(stderr, "%s: serial_recv(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = read(fd, p, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: serial_recv(): read error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    p++;
    len++;
  }

  p = buf;

  if (verbose > 3)
  {
      fprintf(stderr, "%s: Recv: ", progname);

      while (len) {
        char c = *p;
        if (isprint(c)) {
          fprintf(stderr, "%c ", c);
        }
        else {
          fprintf(stderr, ". ");
        }
        fprintf(stderr, "[%02x] ", ((unsigned int)c &0xff));

        p++;
        len--;
      }
      fprintf(stderr, "\n");
  }

  return 0;
}


int serial_drain(int fd, int display)
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
        fprintf(stderr, "%s: serial_drain(): select(): %s\n",
                progname, strerror(errno));
        exit(1);
      }
    }

    rc = read(fd, &buf, 1);
    if (rc < 0) {
      fprintf(stderr, "%s: serial_drain(): read error: %s\n",
              progname, strerror(errno));
      exit(1);
    }
    if (display) {
      fprintf(stderr, "%02x ", buf);
    }
  }

  return 0;
}
