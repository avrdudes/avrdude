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
 * Native Win32 serial interface for avrdude.
 */

#if defined(WIN32NATIVE)

#include <windows.h>
#include <ctype.h>   /* for isprint */
#include "serial.h"

extern char *progname;
extern int verbose;

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


BOOL serial_w32SetTimeOut(HANDLE hComPort, DWORD timeout) // in ms
{
	COMMTIMEOUTS ctmo;
	ZeroMemory (&ctmo, sizeof(COMMTIMEOUTS));
	ctmo.ReadIntervalTimeout = timeout;
	ctmo.ReadTotalTimeoutMultiplier = timeout;
	ctmo.ReadTotalTimeoutConstant = timeout;

	return SetCommTimeouts(hComPort, &ctmo);
}

int serial_open(char * port, long baud)
{
	DCB dcb;
	LPVOID lpMsgBuf;
	HANDLE hComPort=INVALID_HANDLE_VALUE;

	/* if (hComPort!=INVALID_HANDLE_VALUE) 
		fprintf(stderr, "%s: serial_open(): \"%s\" is already open\n",
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
		fprintf(stderr, "%s: serial_open(): can't open device \"%s\": %s\n",
				progname, port, (char*)lpMsgBuf);
		LocalFree( lpMsgBuf );
		exit(1);
	}

	if (!SetupComm(hComPort, W32SERBUFSIZE, W32SERBUFSIZE))
	{
		CloseHandle(hComPort);
		fprintf(stderr, "%s: serial_open(): can't set buffers for \"%s\"\n",
				progname, port);
		exit(1);
	}

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
	{
		CloseHandle(hComPort);
		fprintf(stderr, "%s: serial_open(): can't set com-state for \"%s\"\n",
				progname, port);
		exit(1);
	}

	if (!serial_w32SetTimeOut(hComPort,0))
	{
		CloseHandle(hComPort);
		fprintf(stderr, "%s: serial_open(): can't set initial timeout for \"%s\"\n",
				progname, port);
		exit(1);
	}

  return (int)hComPort;
}


void serial_close(int fd)
{
	HANDLE hComPort=(HANDLE)fd;
	if (hComPort != INVALID_HANDLE_VALUE)
		CloseHandle (hComPort);

	hComPort = INVALID_HANDLE_VALUE;
}


int serial_send(int fd, char * buf, size_t buflen)
{
	size_t len = buflen;
	unsigned char c='\0';
	DWORD written;

	HANDLE hComPort=(HANDLE)fd;

	if (hComPort == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "%s: serial_send(): port not open\n",
              progname); 
		exit(1);
	}

	if (!len)
  return 0;

	if (verbose > 3)
	{
		fprintf(stderr, "%s: Send: ", progname);

		while (buflen) {
			c = *buf;
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
	
	serial_w32SetTimeOut(hComPort,500);

	if (!WriteFile (hComPort, buf, buflen, &written, NULL)) {
		fprintf(stderr, "%s: serial_send(): write error: %s\n",
              progname, "sorry no info avail"); // TODO
		exit(1);
	}

	if (written != buflen) {
		fprintf(stderr, "%s: serial_send(): size/send mismatch\n",
              progname); 
		exit(1);
	}

	return 0;
}


int serial_recv(int fd, char * buf, size_t buflen)
{
	unsigned char c;
	char * p = buf;
	size_t len = 0;
	DWORD read;

	HANDLE hComPort=(HANDLE)fd;
	
	if (hComPort == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "%s: serial_read(): port not open\n",
              progname); 
		exit(1);
	}
	
	serial_w32SetTimeOut(hComPort,5000);
	
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
		fprintf(stderr, "%s: serial_recv(): read error: %s\n",
			      progname, (char*)lpMsgBuf);
		LocalFree( lpMsgBuf );
		exit(1);
	}

	p = buf;

	if (verbose > 3)
	{
		fprintf(stderr, "%s: Recv: ", progname);

		while (len) {
			c = *p;
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


int serial_drain(int fd, int display)
{
	// int rc;
	unsigned char buf[10];
	BOOL readres;
	DWORD read;

	HANDLE hComPort=(HANDLE)fd;

  	if (hComPort == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "%s: serial_drain(): port not open\n",
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
			fprintf(stderr, "%s: serial_drain(): read error: %s\n",
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

#endif /* WIN32NATIVE */
