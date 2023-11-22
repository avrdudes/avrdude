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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Native Win32 serial interface for avrdude.
 */

#include "ac_cfg.h"

#if defined(WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>   /* for isprint */
#include <errno.h>   /* ENOTTY */

#include "avrdude.h"
#include "libavrdude.h"

long serial_recv_timeout = 5000; /* ms */
long serial_drain_timeout = 250; /* ms */

#define W32SERBUFSIZE 1024

struct baud_mapping {
  long baud;
  DWORD speed;
};

static unsigned char serial_over_ethernet = 0;

/* HANDLE hComPort=INVALID_HANDLE_VALUE; */

static struct baud_mapping baud_lookup_table [] = {
  { 300,    CBR_300 },
  { 600,    CBR_600 },
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

  /*
   * If a non-standard BAUD rate is used, issue
   * a warning (if we are verbose) and return the raw rate
   */
  pmsg_notice2("serial_baud_lookup(): using non-standard baud rate: %ld", baud);

  return baud;
}


static BOOL serial_w32SetTimeOut(HANDLE hComPort, DWORD timeout) // in ms
{
	COMMTIMEOUTS ctmo;
	ZeroMemory (&ctmo, sizeof(COMMTIMEOUTS));
	ctmo.ReadIntervalTimeout = 0;
	ctmo.ReadTotalTimeoutMultiplier = 0;
	ctmo.ReadTotalTimeoutConstant = timeout;

	return SetCommTimeouts(hComPort, &ctmo);
}

static int ser_setparams(const union filedescriptor *fd, long baud, unsigned long cflags) {
	if (serial_over_ethernet) {
		return -ENOTTY;
	} else {
		DCB dcb;
		HANDLE hComPort = (HANDLE)fd->pfd;

		ZeroMemory (&dcb, sizeof(DCB));
		dcb.DCBlength = sizeof(DCB);
		dcb.BaudRate = serial_baud_lookup (baud);
		dcb.fBinary = 1;
		dcb.fDtrControl = DTR_CONTROL_DISABLE;
		dcb.fRtsControl = RTS_CONTROL_DISABLE;
		switch ((cflags & (SERIAL_CS5 | SERIAL_CS6 | SERIAL_CS7 | SERIAL_CS8))) {
			case SERIAL_CS5:
				dcb.ByteSize = 5;
				break;
			case SERIAL_CS6:
				dcb.ByteSize = 6;
				break;
			case SERIAL_CS7:
				dcb.ByteSize = 7;
				break;
			case SERIAL_CS8:
				dcb.ByteSize = 8;
				break;
		}
		switch ((cflags & (SERIAL_NO_PARITY | SERIAL_PARENB | SERIAL_PARODD))) {
			case SERIAL_NO_PARITY:
				dcb.Parity = NOPARITY;
				break;
			case SERIAL_PARENB:
				dcb.Parity = EVENPARITY;
				break;
			case SERIAL_PARODD:
				dcb.Parity = ODDPARITY;
				break;
		}
		switch ((cflags & (SERIAL_NO_CSTOPB | SERIAL_CSTOPB))) {
			case SERIAL_NO_CSTOPB:
				dcb.StopBits = ONESTOPBIT;
				break;
			case SERIAL_CSTOPB:
				dcb.StopBits = TWOSTOPBITS;
				break;
		}

		if (!SetCommState(hComPort, &dcb))
			return -1;

		return 0;
	}
}

static int net_open(const char *port, union filedescriptor *fdp) {
	WSADATA wsaData;
	LPVOID lpMsgBuf;

	char *hstr, *pstr, *end;
	unsigned int pnum;
	int fd;
	struct sockaddr_in sockaddr;
	struct hostent *hp;

	if (WSAStartup(MAKEWORD(2, 0), &wsaData) != 0) {
		pmsg_error("WSAStartup() failed\n");
		return -1;
	}

	if ((hstr = strdup(port)) == NULL) {
		pmsg_error("out of memory\n");
		return -1;
	}

	if (((pstr = strchr(hstr, ':')) == NULL) || (pstr == hstr)) {
		pmsg_error("mangled host:port string %s\n", hstr);
		free(hstr);
		return -1;
	}

	/*
	 * Terminate the host section of the description.
	 */
	*pstr++ = '\0';

	pnum = strtoul(pstr, &end, 10);

	if ((*pstr == '\0') || (*end != '\0') || (pnum == 0) || (pnum > 65535)) {
		pmsg_error("bad port number %s\n", pstr);
		free(hstr);
		return -1;
	}

	if ((hp = gethostbyname(hstr)) == NULL) {
		pmsg_error("unknown host %s\n", hstr);
		free(hstr);
		return -1;
	}

	free(hstr);

	if ((fd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER |
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			WSAGetLastError(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR)&lpMsgBuf,
			0,
			NULL);
		pmsg_error("cannot open socket: %s\n", (char *) lpMsgBuf);
		LocalFree(lpMsgBuf);
		return -1;
	}

	memset(&sockaddr, 0, sizeof(struct sockaddr_in));
	sockaddr.sin_family = AF_INET;
	sockaddr.sin_port = htons(pnum);
	memcpy(&(sockaddr.sin_addr.s_addr), hp->h_addr, sizeof(struct in_addr));

	if (connect(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr))) {
		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER |
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			WSAGetLastError(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR)&lpMsgBuf,
			0,
			NULL);
		pmsg_error("connect failed: %s\n", (char *) lpMsgBuf);
		LocalFree(lpMsgBuf);
		return -1;
	}

	fdp->ifd = fd;

	serial_over_ethernet = 1;
	return 0;
}


static int ser_open(const char *port, union pinfo pinfo, union filedescriptor *fdp) {
	LPVOID lpMsgBuf;
	HANDLE hComPort=INVALID_HANDLE_VALUE;
	char *newname = 0;

	/*
	 * If the port is of the form "net:<host>:<port>", then
	 * handle it as a TCP connection to a terminal server.
	 */
	if (str_starts(port, "net:")) {
		return net_open(port + strlen("net:"), fdp);
	}

	if (str_casestarts(port, "com")) {

	    // prepend "\\\\.\\" to name, required for port # >= 10
	    newname = cfg_malloc(__func__, strlen("\\\\.\\") + strlen(port) + 1);
	    strcpy(newname, "\\\\.\\");
	    strcat(newname, port);

	    port = newname;
	}

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
		pmsg_error("cannot open port %s: %s\n", port, (char*) lpMsgBuf);
		LocalFree( lpMsgBuf );
		return -1;
	}

	if (!SetupComm(hComPort, W32SERBUFSIZE, W32SERBUFSIZE))
	{
		CloseHandle(hComPort);
		pmsg_error("cannot set buffers for %s\n", port);
		return -1;
	}

        fdp->pfd = (void *)hComPort;
	if (ser_setparams(fdp, pinfo.serialinfo.baud, pinfo.serialinfo.cflags) != 0)
	{
		CloseHandle(hComPort);
		pmsg_error("cannot set com-state for %s\n", port);
		return -1;
	}

	if (!serial_w32SetTimeOut(hComPort,0))
	{
		CloseHandle(hComPort);
		pmsg_error("cannot set initial timeout for %s\n", port);
		return -1;
	}

	if (newname != 0) {
	    free(newname);
	}
	return 0;
}


static void ser_close(union filedescriptor *fd) {
	if (serial_over_ethernet) {
		closesocket(fd->ifd);
		WSACleanup();
	} else {
		HANDLE hComPort=(HANDLE)fd->pfd;
		if (hComPort != INVALID_HANDLE_VALUE)
			CloseHandle (hComPort);

		hComPort = INVALID_HANDLE_VALUE;
	}
}

static int ser_set_dtr_rts(const union filedescriptor *fd, int is_on) {
	if (serial_over_ethernet) {
		return 0;
	} else {
		HANDLE hComPort=(HANDLE)fd->pfd;

		if (is_on) {
			EscapeCommFunction(hComPort, SETDTR);
			EscapeCommFunction(hComPort, SETRTS);
		} else {
			EscapeCommFunction(hComPort, CLRDTR);
			EscapeCommFunction(hComPort, CLRRTS);
		}
		return 0;
	}
}

static int net_send(const union filedescriptor *fd, const unsigned char *buf, size_t len) {
	LPVOID lpMsgBuf;
	int rc;

	if (fd->ifd < 0) {
		pmsg_notice("net_send(): connection not open\n");
		return -1;
	}

	if (!len)
		return 0;

	if (verbose > 3)
		trace_buffer(__func__, buf, len);

	while (len) {
		rc = send(fd->ifd, (const char *) buf, len > 1024? 1024: len, 0);
		if (rc < 0) {
			FormatMessage(
				FORMAT_MESSAGE_ALLOCATE_BUFFER |
				FORMAT_MESSAGE_FROM_SYSTEM |
				FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL,
				WSAGetLastError(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
				(LPTSTR)&lpMsgBuf,
				0,
				NULL);
			pmsg_error("unable to send: %s\n", (char *) lpMsgBuf);
			LocalFree(lpMsgBuf);
			return -1;
		}
		buf += rc;
		len -= rc;
	}

	return 0;
}


static int ser_send(const union filedescriptor *fd, const unsigned char *buf, size_t len) {
	if (serial_over_ethernet)
		return net_send(fd, buf, len);

	DWORD written;

	HANDLE hComPort=(HANDLE)fd->pfd;

	if (hComPort == INVALID_HANDLE_VALUE) {
		pmsg_error("port not open\n");
		return -1;
	}

	if (!len)
		return 0;

	if (verbose > 3)
		trace_buffer(__func__, buf, len);
	
	serial_w32SetTimeOut(hComPort,500);

	if (!WriteFile (hComPort, buf, len, &written, NULL)) {
		pmsg_error("unable to write: %s\n", "sorry no info avail"); // TODO
		return -1;
	}

	if (written != len) {
		pmsg_error("size/send mismatch\n");
		return -1;
	}

	return 0;
}


static int net_recv(const union filedescriptor *fd, unsigned char *buf, size_t buflen) {
	LPVOID lpMsgBuf;
	struct timeval timeout, to2;
	fd_set rfds;
	int nfds;
	int rc;
	unsigned char *p = buf;
	size_t len = 0;

	if (fd->ifd < 0) {
		pmsg_error("connection not open\n");
		return -1;
	}

	timeout.tv_sec  = serial_recv_timeout / 1000L;
	timeout.tv_usec = (serial_recv_timeout % 1000L) * 1000;
	to2 = timeout;

	while (len < buflen) {
reselect:
		FD_ZERO(&rfds);
		FD_SET(fd->ifd, &rfds);

		nfds = select(fd->ifd + 1, &rfds, NULL, NULL, &to2);
		if (nfds == 0) {
			if (verbose > 1) {
				pmsg_notice("net_recv(): programmer is not responding\n");
			}
			return -1;
		} else if (nfds == -1) {
			if (WSAGetLastError() == WSAEINTR || WSAGetLastError() == WSAEINPROGRESS) {
				pmsg_notice("net_recv(): programmer is not responding, reselecting\n");
				goto reselect;
			} else {
				FormatMessage(
					FORMAT_MESSAGE_ALLOCATE_BUFFER |
					FORMAT_MESSAGE_FROM_SYSTEM |
					FORMAT_MESSAGE_IGNORE_INSERTS,
					NULL,
					WSAGetLastError(),
					MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
					(LPTSTR)&lpMsgBuf,
					0,
					NULL);
				pmsg_error("select(): %s\n", (char *) lpMsgBuf);
				LocalFree(lpMsgBuf);
				return -1;
			}
		}

		rc = recv(fd->ifd, (char *) p, (buflen - len > 1024)? 1024: buflen - len, 0);
		if (rc < 0) {
			FormatMessage(
				FORMAT_MESSAGE_ALLOCATE_BUFFER |
				FORMAT_MESSAGE_FROM_SYSTEM |
				FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL,
				WSAGetLastError(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
				(LPTSTR)&lpMsgBuf,
				0,
				NULL);
			pmsg_error("unable to read: %s\n", (char *) lpMsgBuf);
			LocalFree(lpMsgBuf);
			return -1;
		}
		p += rc;
		len += rc;
	}

	if (verbose > 3)
		trace_buffer(__func__, buf, len);

	return 0;
}

static int ser_recv(const union filedescriptor *fd, unsigned char *buf, size_t buflen) {
	if (serial_over_ethernet)
		return net_recv(fd, buf, buflen);

	DWORD read;

	HANDLE hComPort=(HANDLE)fd->pfd;
	
	if (hComPort == INVALID_HANDLE_VALUE) {
		pmsg_error("port not open\n");
		return -1;
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
			NULL);
		pmsg_error("unable to read: %s\n", (char*) lpMsgBuf);
		LocalFree(lpMsgBuf);
		return -1;
	}

	/* time out detected */
	if (read < buflen) {
		pmsg_notice2("ser_recv(): programmer is not responding\n");
		return -1;
	}

	if (verbose > 3)
		trace_buffer(__func__, buf, read);

	return 0;
}

static int net_drain(const union filedescriptor *fd, int display) {
	LPVOID lpMsgBuf;
	struct timeval timeout;
	fd_set rfds;
	int nfds;
	unsigned char buf;
	int rc;

	if (fd->ifd < 0) {
		pmsg_error("connection not open\n");
		return -1;
	}

	if (display) {
		msg_info("drain>");
	}

	timeout.tv_sec  = 0;
	timeout.tv_usec = serial_drain_timeout*1000L;

	while (1) {
		FD_ZERO(&rfds);
		FD_SET(fd->ifd, &rfds);

	reselect:
		nfds = select(fd->ifd + 1, &rfds, NULL, NULL, &timeout);
		if (nfds == 0) {
			if (display) {
				msg_info("<drain\n");
			}
			break;
		}
		else if (nfds == -1) {
			if (WSAGetLastError() == WSAEINTR || WSAGetLastError() == WSAEINPROGRESS) {
				pmsg_notice("ser_drain(): programmer is not responding, reselecting\n");
				goto reselect;
			} else {
				FormatMessage(
					FORMAT_MESSAGE_ALLOCATE_BUFFER |
					FORMAT_MESSAGE_FROM_SYSTEM |
					FORMAT_MESSAGE_IGNORE_INSERTS,
					NULL,
					WSAGetLastError(),
					MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
					(LPTSTR)&lpMsgBuf,
					0,
					NULL);
				pmsg_error("select(): %s\n", (char *) lpMsgBuf);
				LocalFree(lpMsgBuf);
				return -1;
			}
		}

		rc = recv(fd->ifd, (char *) &buf, 1, 0);
		if (rc < 0) {
			FormatMessage(
				FORMAT_MESSAGE_ALLOCATE_BUFFER |
				FORMAT_MESSAGE_FROM_SYSTEM |
				FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL,
				WSAGetLastError(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
				(LPTSTR)&lpMsgBuf,
				0,
				NULL);
			pmsg_error("unable to read: %s\n", (char *) lpMsgBuf);
			LocalFree(lpMsgBuf);
			return -1;
		}

		if (display) {
			msg_info("%02x ", buf);
		}
	}

	return 0;
}

static int ser_drain(const union filedescriptor *fd, int display) {
	if (serial_over_ethernet) {
		return net_drain(fd, display);
	}

	// int rc;
	unsigned char buf[10];
	BOOL readres;
	DWORD read;

	HANDLE hComPort=(HANDLE)fd->pfd;

  	if (hComPort == INVALID_HANDLE_VALUE) {
		pmsg_error("port not open\n");
		return -1;
	}

	serial_w32SetTimeOut(hComPort, serial_drain_timeout);
  
	if (display) {
		msg_info("drain>");
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
			pmsg_error("unable to read: %s\n", (char*) lpMsgBuf);
			LocalFree( lpMsgBuf );
			return -1;
		}

		if (read) { // data avail
			if (display)
				msg_info("%02x ", buf[0]);
		}
		else { // no more data
			if (display)
				msg_info("<drain\n");
			break;
		}
	} // while
  return 0;
}

struct serial_device serial_serdev =
{
  .open = ser_open,
  .setparams = ser_setparams,
  .close = ser_close,
  .rawclose = ser_close,
  .send = ser_send,
  .recv = ser_recv,
  .drain = ser_drain,
  .set_dtr_rts = ser_set_dtr_rts,
  .flags = SERDEV_FL_CANSETSPEED,
};

struct serial_device *serdev = &serial_serdev;

#endif /* WIN32 */
