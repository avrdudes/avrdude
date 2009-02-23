/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003, 2004  Martin J. Thomas  <mthomas@rhrk.uni-kl.de>
 * Copyright (C) 2006  Joerg Wunsch <j@uriah.heep.sax.de>
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
 * Native Win32 serial interface for avrdude.
 */

#include "avrdude.h"

#if defined(WIN32NATIVE)

#include <windows.h>
#include <stdio.h>
#include <ctype.h>   /* for isprint */

#include "serial.h"

long serial_recv_timeout = 5000; /* ms */

#define W32SERBUFSIZE 1024

struct baud_mapping {
  long baud;
  DWORD speed;
};

/* HANDLE hComPort=INVALID_HANDLE_VALUE; */

static struct baud_mapping baud_lookup_table [] = {
  { 1200,   CBR_1200 },
  { 2400,   CBR_2400 },
  { 4800,   CBR_4800 },
  { 9600,   CBR_9600 },
  { 19200,  CBR_19200 },
  { 38400,  CBR_38400 },
  { 57600,  CBR_57600 },
  { 115200, CBR_115200 },
  { 0,      0 }                 /* Terminator. */
};

static DWORD serial_baud_lookup(long baud)
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


static BOOL serial_w32SetTimeOut(HANDLE hComPort, DWORD timeout) // in ms
{
	COMMTIMEOUTS ctmo;
	ZeroMemory (&ctmo, sizeof(COMMTIMEOUTS));
	ctmo.ReadIntervalTimeout = timeout;
	ctmo.ReadTotalTimeoutMultiplier = timeout;
	ctmo.ReadTotalTimeoutConstant = timeout;

	return SetCommTimeouts(hComPort, &ctmo);
}

static int ser_setspeed(union filedescriptor *fd, long baud)
{
	DCB dcb;
	HANDLE hComPort = (HANDLE)fd->pfd;

	ZeroMemory (&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = serial_baud_lookup (baud);
	dcb.fBinary = 1;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	if (!SetCommState(hComPort, &dcb))
		return -1;

	return 0;
}


static void ser_open(char * port, long baud, union filedescriptor *fdp)
{
	LPVOID lpMsgBuf;
	HANDLE hComPort=INVALID_HANDLE_VALUE;

	/*
	 * If the port is of the form "net:<host>:<port>", then
	 * handle it as a TCP connection to a terminal server.
	 *
	 * This is curently not implemented for Win32.
	 */
	if (strncmp(port, "net:", strlen("net:")) == 0) {
		fprintf(stderr,
			"%s: ser_open(): network connects are currently not"
			"implemented for Win32 environments\n",
			progname);
		exit(1);
	}

	/* if (hComPort!=INVALID_HANDLE_VALUE) 
		fprintf(stderr, "%s: ser_open(): \"%s\" is already open\n",
				progname, port);
	*/

	hComPort = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (hComPort == INVALID_HANDLE_VALUE) {
		FormatMessage( 
			FORMAT_MESSAGE_ALLOCATE_BUFFER | 
			FORMAT_MESSAGE_FROM_SYSTEM | 
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			GetLastError(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
			(LPTSTR) &lpMsgBuf,
			0,
			NULL);
		fprintf(stderr, "%s: ser_open(): can't open device \"%s\": %s\n",
				progname, port, (char*)lpMsgBuf);
		LocalFree( lpMsgBuf );
		exit(1);
	}

	if (!SetupComm(hComPort, W32SERBUFSIZE, W32SERBUFSIZE))
	{
		CloseHandle(hComPort);
		fprintf(stderr, "%s: ser_open(): can't set buffers for \"%s\"\n",
				progname, port);
		exit(1);
	}

        fdp->pfd = (void *)hComPort;
	if (ser_setspeed(fdp, baud) != 0)
	{
		CloseHandle(hComPort);
		fprintf(stderr, "%s: ser_open(): can't set com-state for \"%s\"\n",
				progname, port);
		exit(1);
	}

	if (!serial_w32SetTimeOut(hComPort,0))
	{
		CloseHandle(hComPort);
		fprintf(stderr, "%s: ser_open(): can't set initial timeout for \"%s\"\n",
				progname, port);
		exit(1);
	}
}


static void ser_close(union filedescriptor *fd)
{
	HANDLE hComPort=(HANDLE)fd->pfd;
	if (hComPort != INVALID_HANDLE_VALUE)
		CloseHandle (hComPort);

	hComPort = INVALID_HANDLE_VALUE;
}


static int ser_send(union filedescriptor *fd, unsigned char * buf, size_t buflen)
{
	size_t len = buflen;
	unsigned char c='\0';
	DWORD written;
        unsigned char * b = buf;

	HANDLE hComPort=(HANDLE)fd->pfd;

	if (hComPort == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "%s: ser_send(): port not open\n",
              progname); 
		exit(1);
	}

	if (!len)
  return 0;

	if (verbose > 3)
	{
		fprintf(stderr, "%s: Send: ", progname);

		while (len) {
			c = *b;
			if (isprint(c)) {
				fprintf(stderr, "%c ", c);
			}
			else {
				fprintf(stderr, ". ");
			}
			fprintf(stderr, "[%02x] ", c);
			b++;
			len--;
		}
      fprintf(stderr, "\n");
	}
	
	serial_w32SetTimeOut(hComPort,500);

	if (!WriteFile (hComPort, buf, buflen, &written, NULL)) {
		fprintf(stderr, "%s: ser_send(): write error: %s\n",
              progname, "sorry no info avail"); // TODO
		exit(1);
	}

	if (written != buflen) {
		fprintf(stderr, "%s: ser_send(): size/send mismatch\n",
              progname); 
		exit(1);
	}

	return 0;
}


static int ser_recv(union filedescriptor *fd, unsigned char * buf, size_t buflen)
{
	unsigned char c;
	unsigned char * p = buf;
	DWORD read;

	HANDLE hComPort=(HANDLE)fd->pfd;
	
	if (hComPort == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "%s: ser_read(): port not open\n",
              progname); 
		exit(1);
	}
	
	serial_w32SetTimeOut(hComPort, serial_recv_timeout);
	
	if (!ReadFile(hComPort, buf, buflen, &read, NULL)) {
		LPVOID lpMsgBuf;
		FormatMessage( 
			FORMAT_MESSAGE_ALLOCATE_BUFFER | 
			FORMAT_MESSAGE_FROM_SYSTEM | 
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			GetLastError(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
			(LPTSTR) &lpMsgBuf,
			0,
			NULL 	);
		fprintf(stderr, "%s: ser_recv(): read error: %s\n",
			      progname, (char*)lpMsgBuf);
		LocalFree( lpMsgBuf );
		exit(1);
	}

	p = buf;

	if (verbose > 3)
	{
		fprintf(stderr, "%s: Recv: ", progname);

		while (read) {
			c = *p;
			if (isprint(c)) {
				fprintf(stderr, "%c ", c);
			}
			else {
				fprintf(stderr, ". ");
			}
			fprintf(stderr, "[%02x] ", c);

			p++;
			read--;
		}
		fprintf(stderr, "\n");
	}
  return 0;
}


static int ser_drain(union filedescriptor *fd, int display)
{
	// int rc;
	unsigned char buf[10];
	BOOL readres;
	DWORD read;

	HANDLE hComPort=(HANDLE)fd->pfd;

  	if (hComPort == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "%s: ser_drain(): port not open\n",
              progname); 
		exit(1);
	}

	serial_w32SetTimeOut(hComPort,250);
  
	if (display) {
		fprintf(stderr, "drain>");
	}

	while (1) {
		readres=ReadFile(hComPort, buf, 1, &read, NULL);
		if (!readres) {
			LPVOID lpMsgBuf;
			FormatMessage( 
				FORMAT_MESSAGE_ALLOCATE_BUFFER | 
				FORMAT_MESSAGE_FROM_SYSTEM | 
				FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL,
				GetLastError(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
				(LPTSTR) &lpMsgBuf,
				0,
				NULL 	);
			fprintf(stderr, "%s: ser_drain(): read error: %s\n",
					  progname, (char*)lpMsgBuf);
			LocalFree( lpMsgBuf );
			exit(1);
		}

		if (read) { // data avail
			if (display) fprintf(stderr, "%02x ", buf[0]);
		}
		else { // no more data
			if (display) fprintf(stderr, "<drain\n");
			break;
		}
	} // while
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
  .flags = SERDEV_FL_CANSETSPEED,
};

struct serial_device *serdev = &serial_serdev;

#endif /* WIN32NATIVE */
