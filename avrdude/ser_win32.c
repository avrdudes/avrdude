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

extern char *progname;

#if 0

int serial_open(char * port, int baud)
{
  return fd;
}


void serial_close(int fd)
{
}


int serial_send(int fd, char * buf, size_t buflen)
{
  return 0;
}


int serial_recv(int fd, char * buf, size_t buflen)
{
  return 0;
}


int serial_drain(int fd, int display)
{
  return 0;
}

#endif
