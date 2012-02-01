
/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * some code:
 * Copyright (C) 2011-2012 Roger E. Wolff <R.E.Wolff@BitWizard.nl>
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

/* ft245r -- FT245R/FT232R Synchronous BitBangMode Programmer
  default pin assign
               FT232R / FT245R
  miso  = 2;  # RxD   / D1
  sck   = 1;  # RTS   / D0
  mosi  = 3;  # TxD   / D2
  reset = 5;  # DTR   / D4
*/

/*
  The ft232r is very similar, or even "identical" in the synchronous
  bitbang mode that we use here.

  This allows boards that have an ft232r for communication and an avr
  as the processor to function as their own "ICSP". Boards that fit
  this description include the Arduino Duemilanove, Arduino Diecimila,
  Arduino NG (http://arduino.cc/it/main/boards) and the BitWizard
  ftdi_atmega board (http://www.bitwizard.nl/wiki/index.php/FTDI_ATmega)

  The Arduinos have to be patched to bring some of the control lines
  to the ICSP header. The BitWizard board already has the neccessary
  wiring on the PCB.

  How to add the wires to an arduino is documented here:
  http://www.geocities.jp/arduino_diecimila/bootloader/index_en.html
*/


#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "avr.h"
#include "avrpart.h"
#include "pindefs.h"
#include "pgm.h"
#include "config.h"
#include "bitbang.h"
#include "ft245r.h"

#include <pthread.h>
#include <semaphore.h>

#ifdef HAVE_LIBFTDI

#if defined(_WIN32)
#include <windows.h>
#endif

#include <ftdi.h>


#define FT245R_CYCLES	2
#define FT245R_FRAGMENT_SIZE  512
#define REQ_OUTSTANDINGS	10
//#define USE_INLINE_WRITE_PAGE

#define FT245R_DEBUG	1

static struct ftdi_context *handle;

static unsigned char ft245r_ddr;
static unsigned char ft245r_sck;
static unsigned char ft245r_mosi;
static unsigned char ft245r_reset;
static unsigned char ft245r_miso;

#define BUFSIZE 0x2000

// libftdi / libftd2xx compatibility functions.

static pthread_t readerthread;
static sem_t buf_data, buf_space;
static unsigned char buffer[BUFSIZE];
static int head, tail;

static void add_to_buf (unsigned char c) {
    int nh;

    sem_wait (&buf_space);
    if (head == (BUFSIZE -1)) nh = 0;
    else                      nh = head + 1;

    if (nh == tail) {
        fprintf (stderr, "buffer overflow. Cannot happen!\n");
        //exit (1);
    }
    buffer[head] = c;
    head = nh;
    sem_post (&buf_data);
}

static void *reader (void *arg) {
    struct ftdi_context *handle = (struct ftdi_context *)(arg);
    unsigned char buf[0x1000];
    int br, i;

    while (1) {
        br = ftdi_read_data (handle, buf, sizeof(buf));
        for (i=0; i<br; i++)
            add_to_buf (buf[i]);
    }
    return NULL;
}

static inline void setmybits(unsigned char *data, int pins, int v) {
    if (v) {
        *data |= (pins >> 1);
    } else {
        *data &= ~(pins >> 1);
    }
}

static inline void setmybit(unsigned char *data, int pinno, int v) {
    if (v) {
        *data |= (1 << (pinno-1));
    } else {
        *data &= ~(1 <<(pinno-1));
    }
}

static int ft245r_send(PROGRAMMER * pgm, unsigned char * buf, size_t len) {
    int rv;

    rv = ftdi_write_data(handle, buf, len);
    if (len != rv) return -1;
    return 0;
}

static int ft245r_recv(PROGRAMMER * pgm, unsigned char * buf, size_t len) {
    int i;

    // Copy over data from the circular buffer..
    // XXX This should timeout, and return error if there isn't enough
    // data.
    for (i=0; i<len; i++) {
        sem_wait (&buf_data);
        buf[i] = buffer[tail];
        if (tail == (BUFSIZE -1)) tail = 0;
        else                      tail++;
        sem_post (&buf_space);
    }

    return 0;
}


static int ft245r_drain(PROGRAMMER * pgm, int display) {
    int r;
    unsigned char t;

    // flush the buffer in the chip by changing the mode.....
    r = ftdi_set_bitmode(handle, 0, BITMODE_RESET); 	// reset
    if (r) return -1;
    r = ftdi_set_bitmode(handle, ft245r_ddr, BITMODE_SYNCBB); // set Synchronuse BitBang
    if (r) return -1;

    // drain our buffer.
    while (head != tail) {
        ft245r_recv (pgm, &t, 1);
    }
    return 0;
}

static inline int ft245r_sync(PROGRAMMER * pgm) {
    //printf ("sync.\n");
    // The old code did something that evaluated to a no-op.

    return 0;
}

static int ft245r_chip_erase(PROGRAMMER * pgm, AVRPART * p) {
    unsigned char cmd[4];
    unsigned char res[4];

    if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
        fprintf(stderr, "chip erase instruction not defined for part \"%s\"\n",
                p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));

    avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
    pgm->cmd(pgm, cmd, res);
    usleep(p->chip_erase_delay);
    pgm->initialize(pgm, p);

    return 0;
}

static unsigned char saved_signature[3];

static void ft245r_set_bitclock(PROGRAMMER * pgm) {
    int r;
    int rate = 0;

    /* bitclock is second. 1us = 0.000001. Max rate for ft232r 750000 */
    if(pgm->bitclock) {
        rate = (uint32_t)(1.0/pgm->bitclock) * 2;
    } else if (pgm->baudrate) {
        rate = pgm->baudrate * 2;
    } else {
        rate = 150000; /* should work for all ftdi chips and the avr default internal clock of 1MHz */
    }

    if ((verbose>=1) || FT245R_DEBUG) {
        fprintf(stderr," ft245r:  spi bitclk %d -> ft baudrate %d\n",
                rate / 2, rate);
    }
    r = ftdi_set_baudrate(handle, rate);
    if (r) {
        fprintf(stderr, "Set baudrate (%d) failed with error '%s'.\n",
                rate, ftdi_get_error_string (handle));
        exit (1);
    }
}

static int set_reset(PROGRAMMER * pgm, int val) {
    unsigned char buf[1];

    buf[0] = 0;
    if (val) buf[0] |= ft245r_reset;

    ft245r_send (pgm, buf, 1);
    ft245r_recv (pgm, buf, 1);
    return 0;
}

static int ft245r_cmd(PROGRAMMER * pgm, unsigned char cmd[4],
                      unsigned char res[4]);
/*
 * issue the 'program enable' command to the AVR device
 */
static int ft245r_program_enable(PROGRAMMER * pgm, AVRPART * p) {
    int retry_count = 0;
    unsigned char cmd[4];
    unsigned char res[4];
    int i,reset_ok;

    ft245r_set_bitclock(pgm);

retry:
    reset_ok = 0;
    set_reset(pgm, 0);
    usleep(5000); // 5ms
    set_reset(pgm, 1);
    usleep(5000); // 5ms
    set_reset(pgm, 0);
    usleep(5000); // 5ms

    cmd[0] = 0xAC;
    cmd[1] = 0x53;
    cmd[2] = 0;
    cmd[3] = 0;
    ft245r_cmd(pgm, cmd, res);
    if (res[2] == 0x53 ) reset_ok = 1;
    for (i=0; i<3; i++) {
        cmd[0] = 0x30;
        cmd[1] = 0;
        cmd[2] = i;
        cmd[3] = 0;
        ft245r_cmd(pgm, cmd, res);
        saved_signature[i] = res[3];
    }
    if (reset_ok && (saved_signature[0] == 0x1e)) // success
        return 0;

    if (retry_count < 5) {
        if (retry_count == 3) {
            ft245r_drain (pgm, 0);
            tail = head;
        }
        retry_count++;
        goto retry;
    }
    if ((verbose>=1) || FT245R_DEBUG) {
        fprintf(stderr,
                "%s: ft245r_program_enable: failed\n", progname);
        fflush(stderr);
    }
    return -1;
}

static int ft245r_read_sig_bytes(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m) {
    m->buf[0] = saved_signature[0];
    m->buf[1] = saved_signature[1];
    m->buf[2] = saved_signature[2];
    return 3;
}

#define check_pin(s) {\
    if ((pgm->pinno[s] & PIN_MASK) == 0) {\
        fprintf(stderr,\
                "%s: pin %s is not set\n",\
                progname,#s);\
        exit(1);\
    }\
    if ((pgm->pinno[s] & PIN_INVERSE) != 0) {\
        fprintf(stderr,\
                "%s: pin %s inverse is not supported.\n",\
                progname,#s);\
        exit(1);\
    }\
}

/*
 * initialize the AVR device and prepare it to accept commands
 */
static int ft245r_initialize(PROGRAMMER * pgm, AVRPART * p) {
    check_pin(PIN_AVR_SCK);
    check_pin(PIN_AVR_MOSI);
    check_pin(PIN_AVR_MISO);
    check_pin(PIN_AVR_RESET);
    return ft245r_program_enable(pgm, p);
}

static void ft245r_disable(PROGRAMMER * pgm) {
    return;
}


static void ft245r_enable(PROGRAMMER * pgm) {
    /* Do nothing. */
    return;
}

static inline int set_data(unsigned char *buf, unsigned char data) {
    int j;
    int buf_pos = 0;
    unsigned char bit = 0x80;

    for (j=0; j<8; j++) {
        buf[buf_pos] = 0;
        if (data & bit) buf[buf_pos] |= ft245r_mosi;
        buf_pos++;

        buf[buf_pos] = 0;
        if (data & bit) buf[buf_pos] |= ft245r_mosi;
        buf[buf_pos] |= ft245r_sck;
        buf_pos++;

        bit >>= 1;
    }
    return buf_pos;
}

static inline unsigned char extract_data(unsigned char *buf, int offset) {
    int j;
    int buf_pos = 1;
    unsigned char bit = 0x80;
    unsigned char r = 0;

    buf += offset * (8 * FT245R_CYCLES);
    for (j=0; j<8; j++) {
        if (buf[buf_pos] & ft245r_miso) {
            r |= bit;
        }
        buf_pos += FT245R_CYCLES;
        bit >>= 1;
    }
    return r;
}

/* to check data */
static inline unsigned char extract_data_out(unsigned char *buf, int offset) {
    int j;
    int buf_pos = 1;
    unsigned char bit = 0x80;
    unsigned char r = 0;

    buf += offset * (8 * FT245R_CYCLES);
    for (j=0; j<8; j++) {
        if (buf[buf_pos] & ft245r_mosi) {
            r |= bit;
        }
        buf_pos += FT245R_CYCLES;
        bit >>= 1;
    }
    return r;
}


/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
static int ft245r_cmd(PROGRAMMER * pgm, unsigned char cmd[4],
                      unsigned char res[4]) {
    int i,buf_pos;
    unsigned char buf[128];

    buf_pos = 0;
    for (i=0; i<4; i++) {
        buf_pos += set_data(buf+buf_pos, cmd[i]);
    }
    buf[buf_pos] = 0;
    buf_pos++;

    ft245r_send (pgm, buf, buf_pos);
    ft245r_recv (pgm, buf, buf_pos);
    res[0] = extract_data(buf, 0);
    res[1] = extract_data(buf, 1);
    res[2] = extract_data(buf, 2);
    res[3] = extract_data(buf, 3);

    return 0;
}


static int ft245r_open(PROGRAMMER * pgm, char * port) {
    int rv;
    int devnum = -1;
    strcpy(pgm->port, port);

    if (strcmp(port,DEFAULT_USB) != 0) {
        if (strncasecmp("ft", port, 2) == 0) {
            char *startptr = port + 2;
            char *endptr = NULL;
            devnum = strtol(startptr,&endptr,10);
            if ((startptr==endptr) || (*endptr != '\0')) {
                devnum = -1;
            }
        }
        if (devnum < 0) {
            fprintf(stderr,
                    "%s: invalid portname '%s': use 'ft[0-9]+'\n",
                    progname,port);
            exit(1);
        }
    } else {
        devnum = 0;
    }

    handle = malloc (sizeof (struct ftdi_context));
    ftdi_init(handle);
    rv = ftdi_usb_open_desc_index(handle,
                                  pgm->usbvid?pgm->usbvid:0x0403,
                                  pgm->usbpid?pgm->usbpid:0x6001,
                                  pgm->usbproduct[0]?pgm->usbproduct:NULL,
                                  pgm->usbsn[0]?pgm->usbsn:NULL,
                                  devnum);
    if (rv) {
        fprintf (stderr, "can't open ftdi device %d. (%s)\n", devnum, ftdi_get_error_string(handle));
        ftdi_deinit (handle);
        free(handle);
        exit (1);
    }

    /* We start a new thread to read the output from the FTDI. This is
     * necessary because otherwise we'll deadlock. We cannot finish
     * writing because the ftdi cannot send the results because we
     * haven't provided a read buffer yet. */

    sem_init (&buf_data, 0, 0);
    sem_init (&buf_space, 0, BUFSIZE);
    pthread_create (&readerthread, NULL, reader, handle);

    ft245r_ddr = 0;
    setmybit(&ft245r_ddr, pgm->pinno[PIN_AVR_SCK], 1);
    setmybit(&ft245r_ddr, pgm->pinno[PIN_AVR_MOSI], 1);
    setmybit(&ft245r_ddr, pgm->pinno[PIN_AVR_RESET], 1);

    ft245r_sck = 0;
    setmybit(&ft245r_sck, pgm->pinno[PIN_AVR_SCK], 1);
    ft245r_mosi = 0;
    setmybit(&ft245r_mosi, pgm->pinno[PIN_AVR_MOSI], 1);
    ft245r_reset = 0;
    setmybit(&ft245r_reset, pgm->pinno[PIN_AVR_RESET], 1);
    ft245r_miso = 0;
    setmybit(&ft245r_miso, pgm->pinno[PIN_AVR_MISO], 1);

    rv = ftdi_set_bitmode(handle, ft245r_ddr, BITMODE_SYNCBB); // set Synchronous BitBang

    if (rv) {
        fprintf(stderr,
                "%s: Synchronous BitBangMode is not supported (%s)\n",
                progname, ftdi_get_error_string(handle));
        ftdi_usb_close(handle);
        ftdi_deinit (handle);
        free(handle);
        exit(1);
    }
    /*
     * drain any extraneous input
     */
    ft245r_drain (pgm, 0);

    return 0;
}


static void ft245r_close(PROGRAMMER * pgm) {
    // I think the switch to BB mode and back flushes the buffer.
    ftdi_set_bitmode(handle, 0, BITMODE_SYNCBB); // set Synchronous BitBang, all in puts
    ftdi_set_bitmode(handle, 0, BITMODE_RESET); // disable Synchronous BitBang
    ftdi_usb_close(handle);
    ftdi_deinit (handle);
    free(handle);
}

static void ft245r_display(PROGRAMMER * pgm, const char * p) {
    fprintf(stderr, "%sPin assignment  : 1..8 = DBUS0..7, 9..12 = GPIO0..3\n",p);
    pgm_display_generic_mask(pgm, p, SHOW_AVR_PINS);
}

static int ft245r_paged_write_gen(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                  unsigned int page_size, unsigned int addr,
                                  unsigned int n_bytes) {
    unsigned long i, pa;
    int rc;

    for (i=0; i<n_bytes; i++, addr++) {
        rc = avr_write_byte_default(pgm, p, m, addr, m->buf[addr]);
        if (rc != 0) {
            return -2;
        }

        if (m->paged) {
            // Can this piece of code ever be activated?? Do AVRs exist that
            // have paged non-flash memories? -- REW
            // XXX Untested code below.
            /*
             * check to see if it is time to flush the page with a page
             * write
             */

            if (((addr % m->page_size) == m->page_size-1) || (i == n_bytes-1)) {
                pa = addr - (addr % m->page_size);

                rc = avr_write_page(pgm, p, m, pa);
                if (rc != 0) {
                    return -2;
                }
            }
        }
    }
    return i;
}

static struct ft245r_request {
    int addr;
    int bytes;
    int n;
    struct ft245r_request *next;
} *req_head,*req_tail,*req_pool;

static void put_request(int addr, int bytes, int n) {
    struct ft245r_request *p;
    if (req_pool) {
        p = req_pool;
        req_pool = p->next;
    } else {
        p = malloc(sizeof(struct ft245r_request));
        if (!p) {
            fprintf(stderr, "can't alloc memory\n");
            exit(1);
        }
    }
    memset(p, 0, sizeof(struct ft245r_request));
    p->addr = addr;
    p->bytes = bytes;
    p->n = n;
    if (req_tail) {
        req_tail->next = p;
        req_tail = p;
    } else {
        req_head = req_tail = p;
    }
}

static int do_request(PROGRAMMER * pgm, AVRMEM *m) {
    struct ft245r_request *p;
    int addr, bytes, j, n;
    unsigned char buf[FT245R_FRAGMENT_SIZE+1+128];

    if (!req_head) return 0;
    p = req_head;
    req_head = p->next;
    if (!req_head) req_tail = req_head;

    addr = p->addr;
    bytes = p->bytes;
    n = p->n;
    memset(p, 0, sizeof(struct ft245r_request));
    p->next = req_pool;
    req_pool = p;

    ft245r_recv(pgm, buf, bytes);
    for (j=0; j<n; j++) {
        m->buf[addr++] = extract_data(buf , (j * 4 + 3));
    }
    return 1;
}

static int ft245r_paged_write_flash(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                    int page_size, int addr, int n_bytes) {
    unsigned int    i,j;
    int addr_save,buf_pos,do_page_write,req_count;
    unsigned char buf[FT245R_FRAGMENT_SIZE+1+128];

    req_count = 0;
    for (i=0; i<n_bytes; ) {
        addr_save = addr;
        buf_pos = 0;
        do_page_write = 0;
        for (j=0; j< FT245R_FRAGMENT_SIZE/8/FT245R_CYCLES/4; j++) {
            buf_pos += set_data(buf+buf_pos, (addr & 1)?0x48:0x40 );
            buf_pos += set_data(buf+buf_pos, (addr >> 9) & 0xff );
            buf_pos += set_data(buf+buf_pos, (addr >> 1) & 0xff );
            buf_pos += set_data(buf+buf_pos, m->buf[addr]);
            addr ++;
            i++;
            if ( (m->paged) &&
                    (((i % m->page_size) == 0) || (i == n_bytes))) {
                do_page_write = 1;
                break;
            }
        }
#if defined(USE_INLINE_WRITE_PAGE)
        if (do_page_write) {
            int addr_wk = addr_save - (addr_save % m->page_size);
            /* If this device has a "load extended address" command, issue it. */
            if (m->op[AVR_OP_LOAD_EXT_ADDR]) {
                unsigned char cmd[4];
                OPCODE *lext = m->op[AVR_OP_LOAD_EXT_ADDR];

                memset(cmd, 0, 4);
                avr_set_bits(lext, cmd);
                avr_set_addr(lext, cmd, addr_wk/2);
                buf_pos += set_data(buf+buf_pos, cmd[0]);
                buf_pos += set_data(buf+buf_pos, cmd[1]);
                buf_pos += set_data(buf+buf_pos, cmd[2]);
                buf_pos += set_data(buf+buf_pos, cmd[3]);
            }
            buf_pos += set_data(buf+buf_pos, 0x4C); /* Issue Page Write */
            buf_pos += set_data(buf+buf_pos,(addr_wk >> 9) & 0xff);
            buf_pos += set_data(buf+buf_pos,(addr_wk >> 1) & 0xff);
            buf_pos += set_data(buf+buf_pos, 0);
        }
#endif
        if (i >= n_bytes) {
            buf[buf_pos++] = 0; // sck down
        }
        ft245r_send(pgm, buf, buf_pos);
        put_request(addr_save, buf_pos, 0);
        //ft245r_sync(pgm);
#if 0
        fprintf(stderr, "send addr 0x%04x bufsize %d [%02x %02x] page_write %d\n",
                addr_save,buf_pos,
                extract_data_out(buf , (0*4 + 3) ),
                extract_data_out(buf , (1*4 + 3) ),
                do_page_write);
#endif
        req_count++;
        if (req_count > REQ_OUTSTANDINGS)
            do_request(pgm, m);
        if (do_page_write) {
#if defined(USE_INLINE_WRITE_PAGE)
            while (do_request(pgm, m))
                ;
            usleep(m->max_write_delay);
#else
            int addr_wk = addr_save - (addr_save % m->page_size);
            int rc;
            while (do_request(pgm, m))
                ;
            rc = avr_write_page(pgm, p, m, addr_wk);
            if (rc != 0) {
                return -2;
            }
#endif
            req_count = 0;
        }
    }
    while (do_request(pgm, m))
        ;
    return i;
}


static int ft245r_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                              unsigned int page_size, unsigned int addr, unsigned int n_bytes) {
    if (strcmp(m->desc, "flash") == 0) {
        return ft245r_paged_write_flash(pgm, p, m, page_size, addr, n_bytes);
    } else if (strcmp(m->desc, "eeprom") == 0) {
        return ft245r_paged_write_gen(pgm, p, m, page_size, addr, n_bytes);
    } else {
        return -2;
    }
}

static int ft245r_paged_load_gen(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                 unsigned int page_size, unsigned int addr,
                                 int n_bytes) {
    unsigned char    rbyte;
    unsigned long    i;
    int rc;

    for (i=0; i<n_bytes; i++) {
        rc = avr_read_byte_default(pgm, p, m, i+addr, &rbyte);
        if (rc != 0) {
            return -2;
        }
        m->buf[i+addr] = rbyte;
    }
    return 0;
}

static int ft245r_paged_load_flash(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                   unsigned int page_size, unsigned int addr,
                                   unsigned int n_bytes) {
    unsigned long    i,j,n;
    int addr_save,buf_pos;
    int req_count = 0;
    unsigned char buf[FT245R_FRAGMENT_SIZE+1];

    for (i=0; i<n_bytes; ) {
        buf_pos = 0;
        addr_save = addr;
        for (j=0; j< FT245R_FRAGMENT_SIZE/8/FT245R_CYCLES/4; j++) {
            if (i >= n_bytes) break;
            buf_pos += set_data(buf+buf_pos, (addr & 1)?0x28:0x20 );
            buf_pos += set_data(buf+buf_pos, (addr >> 9) & 0xff );
            buf_pos += set_data(buf+buf_pos, (addr >> 1) & 0xff );
            buf_pos += set_data(buf+buf_pos, 0);
            addr ++;
            i++;
        }
        if (i >= n_bytes) {
            buf[buf_pos++] = 0; // sck down
        }
        n = j;
        ft245r_send(pgm, buf, buf_pos);
        put_request(addr_save, buf_pos, n);
        req_count++;
        if (req_count > REQ_OUTSTANDINGS)
            do_request(pgm, m);

    }
    while (do_request(pgm, m))
        ;
    return 0;
}

static int ft245r_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                             unsigned int page_size, unsigned int addr,
                             unsigned int n_bytes) {
    if (strcmp(m->desc, "flash") == 0) {
        return ft245r_paged_load_flash(pgm, p, m, page_size, addr, n_bytes);
    } else if (strcmp(m->desc, "eeprom") == 0) {
        return ft245r_paged_load_gen(pgm, p, m, page_size, addr, n_bytes);
    } else {
        return -2;
    }
}

void ft245r_initpgm(PROGRAMMER * pgm) {
    strcpy(pgm->type, "ftdi_syncbb");

    /*
     * mandatory functions
     */
    pgm->initialize     = ft245r_initialize;
    pgm->display        = ft245r_display;
    pgm->enable         = ft245r_enable;
    pgm->disable        = ft245r_disable;
    pgm->program_enable = ft245r_program_enable;
    pgm->chip_erase     = ft245r_chip_erase;
    pgm->cmd            = ft245r_cmd;
    pgm->open           = ft245r_open;
    pgm->close          = ft245r_close;
    pgm->read_byte      = avr_read_byte_default;
    pgm->write_byte     = avr_write_byte_default;

    /*
     * optional functions
     */
    pgm->paged_write = ft245r_paged_write;
    pgm->paged_load = ft245r_paged_load;

    pgm->read_sig_bytes = ft245r_read_sig_bytes;
}

#else
static int ft245r_noftdi_open (struct programmer_t *pgm, char * name) {
    fprintf(stderr,
            "%s: error: no ftdi support. Please compile again with libftdi installed.\n",
            progname);

    exit(1);
}

void ft245r_initpgm(PROGRAMMER * pgm) {
    strcpy(pgm->type, "ftdi_syncbb");
    pgm->open = ft245r_noftdi_open;
}
#endif

const char ft245r_desc[] = "FT245R/FT232R Synchronous BitBangMode Programmer";
