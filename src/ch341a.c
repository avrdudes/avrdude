/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 *
 * avrdude support for CH341
 * Copyright (C) 2016  Alexey Sadkov
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
 * Interface to the CH341A programmer.
 *
 */
#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "ch341a.h"
#include "usbdevs.h"

#if defined(HAVE_LIBUSB_1_0)

#ifdef HAVE_LIBUSB_1_0
# define USE_LIBUSB_1_0
# if defined(HAVE_LIBUSB_1_0_LIBUSB_H)
#  include <libusb-1.0/libusb.h>
# else
#  include <libusb.h>
# endif
#endif

#ifdef USE_LIBUSB_1_0

static libusb_context *ctx = NULL;

static int libusb_to_errno(int result)
{
    switch (result) {
    case LIBUSB_SUCCESS:
        return 0;
    case LIBUSB_ERROR_IO:
        return EIO;
    case LIBUSB_ERROR_INVALID_PARAM:
        return EINVAL;
    case LIBUSB_ERROR_ACCESS:
        return EACCES;
    case LIBUSB_ERROR_NO_DEVICE:
        return ENXIO;
    case LIBUSB_ERROR_NOT_FOUND:
        return ENOENT;
    case LIBUSB_ERROR_BUSY:
        return EBUSY;
#ifdef ETIMEDOUT
    case LIBUSB_ERROR_TIMEOUT:
        return ETIMEDOUT;
#endif
#ifdef EOVERFLOW
    case LIBUSB_ERROR_OVERFLOW:
        return EOVERFLOW;
#endif
    case LIBUSB_ERROR_PIPE:
        return EPIPE;
    case LIBUSB_ERROR_INTERRUPTED:
        return EINTR;
    case LIBUSB_ERROR_NO_MEM:
        return ENOMEM;
    case LIBUSB_ERROR_NOT_SUPPORTED:
        return ENOSYS;
    default:
        return ERANGE;
    }
}

#endif

/*
 * Private data for this programmer.
 */
struct pdata
{
  libusb_device_handle *usbhandle;
  int sckfreq_hz;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

// ----------------------------------------------------------------------
/* Prototypes */
static void ch341a_setup(PROGRAMMER * pgm);
static void ch341a_teardown(PROGRAMMER * pgm);
static int  ch341a_open(PROGRAMMER * pgm, char * port);
static void ch341a_close(PROGRAMMER * pgm);
static int ch341a_initialize(PROGRAMMER * pgm, AVRPART * p);
static int ch341a_spi_cmd(PROGRAMMER * pgm, const unsigned char *cmd, unsigned char *res);
static int ch341a_spi(PROGRAMMER * pgm, const unsigned char *in, unsigned char *out, unsigned int size);
static int ch341a_spi_program_enable(PROGRAMMER * pgm, AVRPART * p);
static int ch341a_spi_chip_erase(PROGRAMMER * pgm, AVRPART * p);
static int ch341a_spi_transfer(PROGRAMMER * pgm, const unsigned char *cmd, unsigned char *res);
// dummy functions
static void ch341a_disable(PROGRAMMER * pgm);
static void ch341a_enable(PROGRAMMER * pgm);
static void ch341a_display(PROGRAMMER * pgm, const char * p);


/* ch341 requires LSB first, swap the bit order before send and after receive */
static unsigned char swap_byte(unsigned char byte) {
    byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xaa);
    byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xcc);
    byte = ((byte >> 4) & 0x0f) | ((byte << 4) & 0xf0);
    return byte;
}

static int CH341USBTransferPart(PROGRAMMER * pgm, enum libusb_endpoint_direction dir, unsigned char *buff, unsigned int size) {
    int ret, bytestransferred;

    if (!PDATA(pgm)->usbhandle)
        return 0;

    if ((ret = libusb_bulk_transfer(PDATA(pgm)->usbhandle, CH341A_USB_BULK_ENDPOINT | dir, buff, size, &bytestransferred, CH341A_USB_TIMEOUT))) {
        avrdude_message(MSG_INFO, "%s: Error: libusb_bulk_transfer for IN_EP failed: %d (%s)\n", progname, ret, libusb_error_name(ret));
        return -1;
    }
    return bytestransferred;
}

static bool CH341USBTransfer(PROGRAMMER * pgm, enum libusb_endpoint_direction dir, unsigned char *buff, unsigned int size)
{
    int pos = 0, bytestransferred;
    while (size) {
        bytestransferred = CH341USBTransferPart(pgm, dir, buff + pos, size);
        if (bytestransferred <= 0)
            return false;
        pos += bytestransferred;
        size -= bytestransferred;
    }
    return true;
}
/* The assumed map between UIO command bits, pins on CH341A chip and pins on SPI chip:
 * UIO  CH341A  SPI CH341A SPI name
 * 0    D0/15   CS/1    (CS0)
 * 1    D1/16   unused  (CS1)
 * 2    D2/17   unused  (CS2)
 * 3    D3/18   SCK/6   (DCK)
 * 4    D4/19   unused  (DOUT2)
 * 5    D5/20   SI/5    (DOUT)
 * - The UIO stream commands seem to only have 6 bits of output, and D6/D7 are the SPI inputs,
 *  mapped as follows:
 *  D6/21   unused  (DIN2)
 *  D7/22   SO/2    (DIN)
 */
bool CH341ChipSelect(PROGRAMMER * pgm, unsigned int cs, bool enable) {
    unsigned char res[4];
    unsigned char cmd[4];
    memset(cmd, 0, sizeof(cmd));
    memset(res, 0, sizeof(res));
    avrdude_message(MSG_DEBUG, "%s: ch341a_ChipSelect()\n", progname);
    if (cs > 2) {
        avrdude_message(MSG_INFO, "%s: Error: invalid CS pin %d, 0~2 are available\n", progname, cs);
        return false;
    }
    cmd[0] = CH341A_CMD_UIO_STREAM;
    if (enable)
        cmd[1] = CH341A_CMD_UIO_STM_OUT | ( 0x37 & ~(1<<cs));
    else
        cmd[1] = CH341A_CMD_UIO_STM_OUT | 0x37;
    cmd[2] = CH341A_CMD_UIO_STM_DIR | 0x3F;
    cmd[3] = CH341A_CMD_UIO_STM_END;
    return CH341USBTransferPart(pgm, LIBUSB_ENDPOINT_OUT, cmd, 4);
}

static int ch341a_open(PROGRAMMER * pgm, char * port) {
    LNODEID usbpid = lfirst(pgm->usbpid);
    int                   pid, vid, j, r;
    int                   errorCode = USB_ERROR_NOTFOUND;
    libusb_device_handle *handle = NULL;
    static int            didUsbInit = 0;

    avrdude_message(MSG_DEBUG, "%s: ch341a_open(\"%s\")\n", progname, port);

    if(!didUsbInit) {
        didUsbInit = 1;
        libusb_init(&ctx);
    }

    if (usbpid) {
        pid = *(int *)(ldata(usbpid));
        if (lnext(usbpid))
            avrdude_message(MSG_INFO, "%s: Warning: using PID 0x%04x, ignoring remaining PIDs in list\n",
                            progname, pid);
    } else {
        pid = CH341A_PID;
    }
    vid = pgm->usbvid? pgm->usbvid: CH341A_VID;

    libusb_device **dev_list;
    int dev_list_len = libusb_get_device_list(ctx, &dev_list);

    for (j=0; j<dev_list_len; ++j) {
        libusb_device *dev = dev_list[j];
        struct libusb_device_descriptor descriptor;
        libusb_get_device_descriptor(dev, &descriptor);
        if (descriptor.idVendor == vid && descriptor.idProduct == pid) {
            r = libusb_open(dev, &handle);
            if (!handle) {
                errorCode = USB_ERROR_ACCESS;
                avrdude_message(MSG_INFO, "%s: Warning: cannot open USB device: %s\n",
                                progname, strerror(libusb_to_errno(r)));
                continue;
            }
        }
    }
    libusb_free_device_list(dev_list,1);
    if (handle != NULL) {
        errorCode = 0;
        PDATA(pgm)->usbhandle = handle;
    }

    if (errorCode!= 0) {
        avrdude_message(MSG_INFO, "%s: error: could not find USB device with vid=0x%x pid=0x%x",
                        progname, vid, pid);
        avrdude_message(MSG_INFO, "\n");
        return -1;
    }
    if ((r = libusb_claim_interface(PDATA(pgm)->usbhandle, 0))) {
        fprintf(stderr, "%s: error: libusb_claim_interface failed: %d (%s)\n", progname, r, libusb_error_name(r));
        libusb_close(PDATA(pgm)->usbhandle);
        libusb_exit(ctx);
    }
    return 0;
}

static void ch341a_close(PROGRAMMER * pgm) {
    avrdude_message(MSG_DEBUG, "%s: ch341a_close()\n", progname);
    CH341ChipSelect(pgm, 0,false);

    if (PDATA(pgm)->usbhandle!=NULL) {
        libusb_release_interface(PDATA(pgm)->usbhandle, 0);
        libusb_close(PDATA(pgm)->usbhandle);
    }
    libusb_exit(ctx);
}

static int ch341a_initialize(PROGRAMMER * pgm, AVRPART * p) {
    avrdude_message(MSG_DEBUG, "%s: ch341a_initialize()\n", progname);
    CH341ChipSelect(pgm, 0,false);
    usleep(20 * 1000);
    CH341ChipSelect(pgm, 0,true);
    return pgm->program_enable(pgm, p);
}

static int ch341a_spi_transfer(PROGRAMMER * pgm, const unsigned char *cmd, unsigned char *res) {
    unsigned char pkt[CH341A_PACKET_LENGTH];
    unsigned int i;
    int ret, bytestransferred;
    int size=sizeof(cmd);

    avrdude_message(MSG_DEBUG, "%s: ch341a_spi_transfer(0x%02x, 0x%02x, 0x%02x, 0x%02x)%s",
                    progname, cmd[0], cmd[1], cmd[2], cmd[3], verbose > 3? "...\n": "\n");

    if (size > CH341A_PACKET_LENGTH - 1)
        size = CH341A_PACKET_LENGTH - 1;

    for (i = 0; i < size; i++) {
        pkt[i] = cmd[i];
    }

    if (CH341USBTransferPart(pgm, LIBUSB_ENDPOINT_OUT, pkt, size)<=0 || CH341USBTransferPart(pgm, LIBUSB_ENDPOINT_IN, pkt, size)<=0) {
        avrdude_message(MSG_INFO, "%s: failed to transfer data to/from CH341\n", progname);
        return -1;
    }

    for (i = 0; i < size; i++) {
        res[i] = pkt[i];
    }
    return size;
}
static int ch341a_spi_cmd(PROGRAMMER * pgm, const unsigned char *cmd, unsigned char *res) {
    return pgm->spi(pgm, cmd, res, 4);
}
static int ch341a_spi(PROGRAMMER * pgm, const unsigned char *in, unsigned char *out, unsigned int size) {
    unsigned char pkt[CH341A_PACKET_LENGTH];
    unsigned int i;

    if (!size)
        return 0;

    if (size > CH341A_PACKET_LENGTH - 1)
        size = CH341A_PACKET_LENGTH - 1;

    pkt[0] = CH341A_CMD_SPI_STREAM;

    for (i = 0; i < size; i++)
        pkt[i + 1] = swap_byte(in[i]);

    if (!CH341USBTransfer(pgm, LIBUSB_ENDPOINT_OUT, pkt, size +1))
    {
        fprintf(stderr, "Error: failed to transfer data to CH341\n");
        return -1;
    }

    if (!CH341USBTransfer(pgm, LIBUSB_ENDPOINT_IN, pkt, size))
    {
        fprintf(stderr, "Error: failed to transfer data from CH341\n");
        return -1;
    }

    for (i = 0; i < size; i++)
        out[i] = swap_byte(pkt[i]);

    return size;
}
static int ch341a_spi_program_enable(PROGRAMMER * pgm, AVRPART * p) {
    unsigned char res[4];
    unsigned char cmd[4];
    memset(cmd, 0, sizeof(cmd));
    memset(res, 0, sizeof(res));

    cmd[0] = 0;
    avrdude_message(MSG_DEBUG, "%s: ch341a_program_enable() %s\n",
                    progname, p->op[AVR_OP_PGM_ENABLE]);

    if (p->op[AVR_OP_PGM_ENABLE] == NULL) {
        avrdude_message(MSG_INFO, "program enable instruction not defined for part \"%s\"\n", p->desc);
        return -1;
    }
    avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
    pgm->cmd(pgm, cmd, res);

    avrdude_message(MSG_DEBUG, "program_enable(): sending command. Resp = %x %x %x %x \n", (int)res[0], (int)res[1], (int)res[2], (int)res[3]);
    // check for sync character
    if (res[2] != cmd[1])
        return -2;
    return 0;
}
static int  ch341a_spi_chip_erase(struct programmer_t * pgm, AVRPART * p) {
    unsigned char cmd[4];
    unsigned char res[4];
    if (p->op[AVR_OP_CHIP_ERASE] == NULL) {
        avrdude_message(MSG_INFO, "chip erase instruction not defined for part \"%s\"\n", p->desc);
        return -1;
    }
    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
    pgm->cmd(pgm, cmd, res);
    usleep(p->chip_erase_delay);
    pgm->initialize(pgm, p);
    return 0;
}


void ch341a_initpgm(PROGRAMMER * pgm) {
    strcpy(pgm->type, "ch341a");
    /*
    * mandatory functions
    */
    pgm->initialize     = ch341a_initialize;
    pgm->display        = ch341a_display;
    pgm->enable         = ch341a_enable;
    pgm->disable        = ch341a_disable;
    pgm->program_enable = ch341a_spi_program_enable;
    pgm->chip_erase     = ch341a_spi_chip_erase;
    pgm->cmd            = ch341a_spi_cmd;
    pgm->spi            = ch341a_spi;
    pgm->open           = ch341a_open;
    pgm->close          = ch341a_close;
    pgm->read_byte      = avr_read_byte_default;
    pgm->write_byte     = avr_write_byte_default;

    /*
    * optional functions
    */

    //pgm->paged_write    = ch341a_spi_paged_write;
    //pgm->paged_load     = ch341a_spi_paged_load;
    pgm->setup          = ch341a_setup;
    pgm->teardown       = ch341a_teardown;

}

/* Interface - management */
static void ch341a_setup(PROGRAMMER * pgm) {
    if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
        avrdude_message(MSG_INFO, "%s: ch341a_setup(): Out of memory allocating private data\n",
                        progname);
        exit(1);
    }
    memset(pgm->cookie, 0, sizeof(struct pdata));
}

static void ch341a_teardown(PROGRAMMER * pgm) {
    free(pgm->cookie);
}
/* Dummy functions */
static void ch341a_disable(PROGRAMMER * pgm) {
    /* Do nothing. */
    return;
}

static void ch341a_enable(PROGRAMMER * pgm) {
    /* Do nothing. */
    return;
}

static void ch341a_display(PROGRAMMER * pgm, const char * p) {
    return;
}


// ----------------------------------------------------------------------
#else /* HAVE_LIBUSB */

static int ch341a_nousb_open (struct programmer_t *pgm, char * name) {
    avrdude_message(MSG_INFO, "%s: error: no usb support. please compile again with libusb installed.\n",
                    progname);
    return -1;
}

void ch341a_initpgm(PROGRAMMER * pgm) {
    strcpy(pgm->type, "ch341a");
    pgm->open = ch341a_nousb_open;
}

#endif  /* HAVE_LIBUSB */
const char ch341a_desc[] = "Driver for \"ch341a\"-type programmers";
