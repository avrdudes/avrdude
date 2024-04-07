/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
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

/*
 * Posix serial interface for avrdude.
 */

#if !defined(WIN32)

#include "ac_cfg.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"
#include <LibSerial.h>

long serial_recv_timeout = 5000; /* ms */
long serial_drain_timeout = 250; /* ms */

struct baud_mapping {
    long baud;
    speed_t speed;
};

/* There are a lot more baud rates we could handle, but what's the point? */

static struct baud_mapping baud_lookup_table [] = {
        { 300,    B300 },
        { 600,    B600 },
        { 1200,   B1200 },
        { 2400,   B2400 },
        { 4800,   B4800 },
        { 9600,   B9600 },
        { 19200,  B19200 },
        { 38400,  B38400 },
#ifdef B57600
        { 57600,  B57600 },
#endif
#ifdef B115200
        { 115200, B115200 },
#endif
#ifdef B230400
        { 230400, B230400 },
#endif
#ifdef B250000
        { 250000, B250000 },
#endif
#ifdef B460800
        { 460800, B460800 },
#endif
#ifdef B500000
        { 500000, B500000 },
#endif
#ifdef B576000
        { 576000, B576000 },
#endif
#ifdef B921600
        { 921600, B921600 },
#endif
#ifdef B1000000
        { 1000000, B1000000 },
#endif
#ifdef B1152000
        { 1152000, B1152000 },
#endif
#ifdef B1500000
        { 1500000, B1500000 },
#endif
#ifdef B2000000
        { 2000000, B2000000 },
#endif
#ifdef B2500000
        { 2500000, B2500000 },
#endif
#ifdef B3000000
        { 3000000, B3000000 },
#endif
#ifdef B3500000
        { 3500000, B3500000 },
#endif
#ifdef B4000000
        { 4000000, B4000000 },
#endif
        { 0,      0 }                 /* Terminator. */
};

static struct termios original_termios;
static int saved_original_termios;

static speed_t serial_baud_lookup(long baud, bool *nonstandard) {
    struct baud_mapping *map = baud_lookup_table;

    *nonstandard = false;

    while (map->baud) {
        if (map->baud == baud)
            return map->speed;
        map++;
    }

    /*
     * If a non-standard BAUD rate is used, issue
     * a warning (if we are verbose) and return the raw rate
     */
    pmsg_notice2("serial_baud_lookup(): using non-standard baud rate: %ld\n", baud);

    *nonstandard = true;

    return baud;
}

static int ser_setparams(const union filedescriptor *fd, long baud, unsigned long cflags) {
    int rc;
    struct termios termios;
    bool nonstandard;
    speed_t speed = serial_baud_lookup (baud, &nonstandard);

    if (!isatty(fd->ifd))
        return -ENOTTY;

    /*
     * initialize terminal modes
     */
    rc = tcgetattr(fd->ifd, &termios);
    if (rc < 0) {
        int ret = -errno;
        pmsg_ext_error("tcgetattr() failed\n");
        return ret;
    }

    /*
     * copy termios for ser_close if we haven't already
     */
    if (! saved_original_termios++) {
        original_termios = termios;
    }

    if (cflags & SERIAL_CREAD) {
        termios.c_cflag |= CREAD;
    }
    if (cflags & SERIAL_CLOCAL) {
        termios.c_cflag |= CLOCAL;
    }
    termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
#ifdef ECHOCTL
    termios.c_lflag &= ~ECHOCTL;
#endif /* ECHOCTL */
#ifdef ECHOKE
    termios.c_lflag &= ~ECHOKE;
#endif /* ECHOKE */
    termios.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    termios.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
    termios.c_iflag &= ~IUCLC;
#endif /* IUCLC */
#ifdef PARMRK
    termios.c_iflag &= ~PARMRK;
#endif /* PARMRK */

    // MacOS doesn't handle nonstandard baudrate values in
    // normal tcsetattr(), sigh.
#ifdef __APPLE__
    if (!nonstandard) {
#endif
    cfsetospeed(&termios, speed);
    cfsetispeed(&termios, speed);
#ifdef __APPLE__
    }
#endif

    termios.c_cflag &= ~CSIZE;
    if (cflags & SERIAL_CS8) {
        termios.c_cflag |= CS8;
    }
    if (cflags & SERIAL_CS7) {
        termios.c_cflag |= CS7;
    }
    if (cflags & SERIAL_CS6) {
        termios.c_cflag |= CS6;
    }
    if (cflags & SERIAL_CS5) {
        termios.c_cflag |= CS5;
    }

    if (cflags & SERIAL_CSTOPB) {
        termios.c_cflag |= CSTOPB;
    } else {
        termios.c_cflag &= ~CSTOPB;
    }

    termios.c_iflag &= ~(INPCK | ISTRIP);

    if (cflags & (SERIAL_PARENB | SERIAL_PARODD)) {
        termios.c_cflag |= PARENB;
    } else {
        termios.c_cflag &= ~PARENB;
    }

    if (cflags & SERIAL_PARODD) {
        termios.c_cflag |= PARODD;
    } else {
        termios.c_cflag &= ~PARODD;
    }

#ifdef IXANY
    termios.c_iflag &= ~IXANY;
#endif /* IXANY */
    termios.c_iflag &= ~(IXON | IXOFF);

#ifdef CRTSCTS
    termios.c_iflag &= ~CRTSCTS;
#endif /* CRTSCTS */

#ifdef CNEW_RTSCTS
    termios.c_iflag &= ~CNEW_RTSCTS;
#endif /* CRTSCTS */


    rc = tcsetattr(fd->ifd, TCSANOW, &termios);
    if (rc < 0) {
        int ret = -errno;
        pmsg_ext_error("tcsetattr() failed\n");
        return ret;
    }

#ifdef __APPLE__
    // handle nonstandard speed values the MacOS way
  if (nonstandard) {
    if (ioctl(fd->ifd, IOSSIOSPEED, &speed) < 0) {
      int ret = -errno;
      pmsg_ext_error("ioctrl(IOSSIOSPEED) failed\n");
      return ret;
    }
  }
#endif // __APPLE__

    tcflush(fd->ifd, TCIFLUSH);

    return 0;
}

/*
 * Given a port description of the form <host>:<port>, open a TCP
 * connection to the specified destination, which is assumed to be a
 * terminal/console server with serial parameters configured
 * appropriately (e. g. 115200-8-N-1 for a STK500.)
 */
static int net_open(const char *port, union filedescriptor *fdp) {
    char *hp, *hstr, *pstr;
    int s, fd, ret = -1;
    struct addrinfo hints;
    struct addrinfo *result, *rp;

    if ((hstr = hp = strdup(port)) == NULL) {
        pmsg_error("out of memory\n");
        return -1;
    }

    /*
     * As numeric IPv6 addresses use colons as separators, we need to
     * look for the last colon here, which separates the port number or
     * service name from the host or IP address.
     */
    if (((pstr = strrchr(hstr, ':')) == NULL) || (pstr == hstr)) {
        pmsg_error("mangled host:port string %s\n", hstr);
        goto error;
    }

    /*
     * Remove brackets from the host part, if present.
     */
    if (*hstr == '[' && *(pstr-1) == ']') {
        hstr++;
        *(pstr-1) = '\0';
    }

    /*
     * Terminate the host section of the description.
     */
    *pstr++ = '\0';

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    s = getaddrinfo(hstr, pstr, &hints, &result);

    if (s != 0) {
        pmsg_ext_error("cannot resolve host=\"%s\", port=\"%s\": %s\n",
                       hstr, pstr, gai_strerror(s));
        goto error;
    }
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd == -1) {
            /* This one failed, loop over */
            continue;
        }
        if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1) {
            /* Success, we are connected */
            break;
        }
        close(fd);
    }
    if (rp == NULL) {
        pmsg_ext_error("cannot connect: %s\n", strerror(errno));
    }
    else {
        fdp->ifd = fd;
        ret = 0;
    }
    freeaddrinfo(result);

    error:
    free(hp);
    return ret;
}

static int ser_set_dtr_rts(const union filedescriptor *fdp, int is_on) {
    bool is_on_bool = is_on != 0;
    setDtrRts(is_on_bool);
    return 0;
}

static int ser_open(const char *port, union pinfo pinfo, union filedescriptor *fdp) {
    int rc;
    int fd;

    /*
     * If the port is of the form "net:<host>:<port>", then
     * handle it as a TCP connection to a terminal server.
     */
    if (str_starts(port, "net:")) {
        return net_open(port + strlen("net:"), fdp);
    }

    /*
     * open the serial port
     */
    serialPortOpen(pinfo.serialinfo.baud);
    return 0;
}

static void ser_close(union filedescriptor *fd) {
    serialPortClose();
}

// Close but don't restore attributes
static void ser_rawclose(union filedescriptor *fd) {
    saved_original_termios = 0;
    close(fd->ifd);
}

static int ser_send(const union filedescriptor *fd, const unsigned char *buf, size_t len) {
    serialPortWrite(buf, len);
    return 0;
}


static int ser_recv(const union filedescriptor *fd, unsigned char *buf, size_t buflen) {
    int re = serialPortRecv(buf, buflen, serial_recv_timeout);
    if (re == -1) { // buf[0] == 1 means no data was received
        printf("Nothing received\n");
        return -1;
    }
    return 0;
}


static int ser_drain(const union filedescriptor *fd, int display) {
    serialPortDrain(serial_drain_timeout);
    return 0;
}

struct serial_device serial_serdev =
        {
                .open = ser_open,
                .setparams = ser_setparams,
                .close = ser_close,
                .rawclose = ser_rawclose,
                .send = ser_send,
                .recv = ser_recv,
                .drain = ser_drain,
                .set_dtr_rts = ser_set_dtr_rts,
                .flags = SERDEV_FL_CANSETSPEED,
        };

struct serial_device *serdev = &serial_serdev;

#endif  /* WIN32 */

