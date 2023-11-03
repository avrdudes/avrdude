/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2005 Erik Walthinsen
 * Copyright (C) 2002-2004 Brian S. Dean <bsd@bdmicro.com>
 * Copyright (C) 2006 David Moore
 * Copyright (C) 2006,2007 Joerg Wunsch <j@uriah.heep.sax.de>
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

/* $Id: pickit2.c 2010-05-03 dbrown$ */
/* Based on Id: stk500v2.c 836 2009-07-10 22:39:37Z joerg_wunsch */

/*
 * avrdude interface for PicKit2 programmer
 *
 * The PicKit2 programmer is a cheap device capable
 * of 2 (bidirectional data line), 3, 4 wire SPI comms
 *
 * The PICkit2 software license doesn't allow the source to be
 * modified to program other devices - nor can we distribute
 * their source code. This program is not derived from nor does it
 * contain any of the pickit2 source and should be exempt from any
 * licensing issues.
 *
 * ISP Pinout (AVR - PICKit2 (pin)):
 * RST  - VPP/MCLR (1)
 * VDD  - VDD Target (2) -- possibly optional if AVR self powered
 * GND  - GND (3)
 * SDI  - PGD (4)
 * SCLK - PDC (5)
 * SDO  - AUX (6)
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#if defined(HAVE_LIBUSB) || defined(WIN32)

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <hidsdi.h>
#include <setupapi.h>
#else
#if defined(HAVE_USB_H)
#  include <usb.h>
#elif defined(HAVE_LUSB0_USB_H)
#  include <lusb0_usb.h>
#else
#  error "libusb needs either <usb.h> or <lusb0_usb.h>"
#endif
#endif

#if 0
#define DEBUG(...) do { msg_debug(__VA_ARGS__); } while(0)
#else
#define DEBUG(...) ((void)0)
#endif

#if 0
#define DEBUGRECV(...) do { msg_debug(__VA_ARGS__); } while(0)
#else
#define DEBUGRECV(...) ((void)0)
#endif

#define PICKIT2_VID 0x04d8
#define PICKIT2_PID 0x0033

#define SPI_MAX_CHUNK (64 - 10)    // max packet size less the command overhead

#ifdef WIN32
static HANDLE open_hid(unsigned short vid, unsigned short pid);
static const char *usb_strerror()
{
    return "";
}
#else
static int usb_open_device(struct usb_dev_handle **dev, int vid, int pid);
//#define INVALID_HANDLE_VALUE NULL
#define USB_ERROR_NONE      0
#define USB_ERROR_ACCESS    1
#define USB_ERROR_NOTFOUND  2
#define USB_ERROR_BUSY      16
#define USB_ERROR_IO        5
#endif  // WIN32

static int pickit2_write_report(const PROGRAMMER *pgm, const unsigned char report[65]);
static int pickit2_read_report(const PROGRAMMER *pgm, unsigned char report[65]);

#ifndef MIN
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#endif

/*
 * Private data for this programmer.
 */
struct pdata
{
#ifdef WIN32
    HANDLE usb_handle, write_event, read_event;
#else
    struct usb_dev_handle *usb_handle;     // LIBUSB STUFF
#endif
    uint8_t clock_period;  // SPI clock period in us
    int transaction_timeout;    // usb trans timeout in ms
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

#define CMD_NOP             0x5A
#define CMD_GET_VERSION     0x76
#define CMD_SET_VDD_4(v)    0xA0, (uint8_t)((v)*2048+672), (uint8_t)(((v)*2048+672)/256), (uint8_t)((v)*36)
#define CMD_SET_VPP_4(v)    0xA1, 0x40, (uint8_t)((v)*18.61), (uint8_t)((v)*13)
#define CMD_READ_VDD_VPP    0xA3
#define CMD_EXEC_SCRIPT_2(len)  0xA6, (len)
#define CMD_CLR_DLOAD_BUFF  0xA7
#define CMD_DOWNLOAD_DATA_2(len)  0xA8, (len)
#define CMD_CLR_ULOAD_BUFF  0xA9
#define CMD_UPLOAD_DATA     0xAA
#define CMD_UPLOAD_DATA_NO_LEN     0xAC
#define CMD_END_OF_BUFFER   0xAD

#define SCR_VDD_ON          0xFF
#define SCR_VDD_OFF         0xFE
#define SCR_VPP_ON          0xFB
#define SCR_VPP_OFF         0xFA
#define SCR_VPP_PWM_ON      0xF9
#define SCR_VPP_PWM_OFF     0xF8
#define SCR_MCLR_GND_ON     0xF7
#define SCR_MCLR_GND_OFF    0xF6
#define SCR_BUSY_LED_ON     0xF5
#define SCR_BUSY_LED_OFF    0xF4
#define SCR_SET_ICSP_DELAY_2(us) 0xEA,(us)
#define SCR_SET_PINS_2(dd, cd, dv, cv) 0xF3, (((cd)!=0) | (((dd)!=0)<<1) | (((cv)!=0)<<2) | (((dv)!=0)<<3))
#define SCR_GET_PINS        0xDC
#define SCR_LOOP_3(rel, cnt)    0xE9, rel, cnt
#define SCR_DELAY_2(sec)    ((sec)>0.0054528?0xE8:0xE7), (uint8_t)((sec)>0.0054528?(.999+(sec)/.00546):(.999+(sec)/.0000213))
#define SCR_SET_AUX_2(ad, av)   0xCF, (((ad)!=0) | (((av)!=0)<<1))
#define SCR_SPI_SETUP_PINS_4    SCR_SET_PINS_2(1,0,0,0), SCR_SET_AUX_2(0,0)
#define SCR_SPI             0xC3
#define SCR_SPI_LIT_2(v)    0xC7,(v)

static void pickit2_setup(PROGRAMMER * pgm)
{
    if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0)
    {
        pmsg_error("out of memory allocating private data\n");
        exit(1);
    }
    memset(pgm->cookie, 0, sizeof(struct pdata));

    PDATA(pgm)->transaction_timeout = 1500;    // default value, may be overridden with -x timeout=ms
    PDATA(pgm)->clock_period = 10;    // default value, may be overridden with -x clockrate=us or -B or -i
}

static void pickit2_teardown(PROGRAMMER * pgm)
{
    free(pgm->cookie);
}

static int pickit2_open(PROGRAMMER *pgm, const char *port) {
#ifdef WIN32
    PDATA(pgm)->usb_handle = open_hid(PICKIT2_VID, PICKIT2_PID);

    if (PDATA(pgm)->usb_handle == INVALID_HANDLE_VALUE)
    {
        /* no PICkit2 found */
        pmsg_error("cannot find PICkit2 with vid=0x%x pid=0x%x\n", PICKIT2_VID, PICKIT2_PID);
        return -1;
    }
    else
    {
        // Get the device description while we're at it and overlay it on pgm->desc
        short wbuf[80-1];
        char *cbuf = cfg_malloc("pickit2_open()", sizeof wbuf/sizeof*wbuf + (pgm->desc? strlen(pgm->desc): 0) + 2);
        HidD_GetProductString(PDATA(pgm)->usb_handle, wbuf, sizeof wbuf/sizeof*wbuf);

        if(pgm->desc && *pgm->desc)
          strcpy(cbuf, pgm->desc);

        // Convert from wide chars and overlay over initial part of desc
        for(size_t i = 0; i < sizeof wbuf/sizeof*wbuf && wbuf[i]; i++)
          cbuf[i] = (char) wbuf[i]; // TODO what about little/big endian???
        pgm->desc = cache_string(cbuf);
    }
#else
    if (usb_open_device(&(PDATA(pgm)->usb_handle), PICKIT2_VID, PICKIT2_PID) < 0)
    {
        /* no PICkit2 found */
        pmsg_error("cannot find PICkit2 with vid=0x%x pid=0x%x\n", PICKIT2_VID, PICKIT2_PID);
        return -1;
    }
#endif

    if (pgm->ispdelay > 0)
    {
        PDATA(pgm)->clock_period = MIN(pgm->ispdelay, 255);
    }
    else if (pgm->bitclock > 0.0)
    {
        PDATA(pgm)->clock_period = MIN(pgm->bitclock * 1e6, 255);
    }

    return 0;
}


static void pickit2_close(PROGRAMMER * pgm)
{
#ifdef WIN32
    CloseHandle(PDATA(pgm)->usb_handle);
    CloseHandle(PDATA(pgm)->read_event);
    CloseHandle(PDATA(pgm)->write_event);
#else
    usb_close(PDATA(pgm)->usb_handle);
#endif  // WIN32
}


static int pickit2_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
    unsigned char temp[4];
    memset(temp, 0, sizeof(temp));

    int errorCode = 0;

    /* set sck period */
    if (pgm->set_sck_period)
        pgm->set_sck_period(pgm, pgm->bitclock);

    /* connect to target device -- we'll just ask for the firmware version */
    static const unsigned char report[65] = {0, CMD_GET_VERSION, CMD_END_OF_BUFFER};
    if ((errorCode = pickit2_write_report(pgm, report)) > 0)
    {
        unsigned char report[65] = {0};
        //memset(report, 0, sizeof(report));
        if ((errorCode = pickit2_read_report(pgm, report)) >= 4)
        {
            pmsg_notice("%s firmware version %d.%d.%d\n", pgm->desc, (int)report[1], (int)report[2], (int)report[3]);

            // set the pins, apply reset,
            // TO DO: apply vtarget (if requested though -x option)
            unsigned char report[65] =
            {
                0, CMD_SET_VDD_4(5),
                CMD_SET_VPP_4(5),
                CMD_EXEC_SCRIPT_2(24),
                SCR_SPI_SETUP_PINS_4,   // SDO, SDI, SCK
                SCR_SET_ICSP_DELAY_2(PDATA(pgm)->clock_period),    // slow down the SPI
                SCR_VDD_ON,
                SCR_MCLR_GND_OFF,       // let reset float high
                SCR_VPP_PWM_ON,
                SCR_DELAY_2(.1),
                SCR_VPP_ON,
                SCR_DELAY_2(.1),
                SCR_VPP_OFF,
                SCR_DELAY_2(.01),

                SCR_MCLR_GND_ON,        // reset low - programming mode
                SCR_DELAY_2(.1),

                SCR_BUSY_LED_ON,
                SCR_DELAY_2(.3),
                SCR_BUSY_LED_OFF,

                CMD_CLR_DLOAD_BUFF,
                CMD_CLR_ULOAD_BUFF,

                CMD_END_OF_BUFFER
            };

            if (pickit2_write_report(pgm, report) < 0)
            {
                pmsg_error("pickit2_read_report failed (ec %d). %s\n", errorCode, usb_strerror());
                return -1;
            }
        }
        else
        {
            pmsg_error("pickit2_read_report failed (ec %d). %s\n", errorCode, usb_strerror());
            return -1;
        }
    }
    else
    {
        pmsg_error("pickit2_write_report failed (ec %d). %s\n", errorCode, usb_strerror());
        return -1;
    }

    if (pgm->program_enable)
        return pgm->program_enable(pgm, p);
    else
        return -1;
}

static void pickit2_disable(const PROGRAMMER *pgm) {
    /* make sure all pins are floating & all voltages are off */
    static const unsigned char report[65] =
    {
        0, CMD_EXEC_SCRIPT_2(8),
        SCR_SET_PINS_2(1,1,0,0),
        SCR_SET_AUX_2(1,0),
        SCR_MCLR_GND_OFF,
        SCR_VPP_OFF,
        SCR_VDD_OFF,
        SCR_VPP_PWM_OFF,
        SCR_DELAY_2(.01),
        SCR_BUSY_LED_OFF,
        CMD_END_OF_BUFFER
    };

    pickit2_write_report(pgm, report);

    return;
}

static void pickit2_enable(PROGRAMMER *pgm, const AVRPART *p) {
    return;
}

static void pickit2_display(const PROGRAMMER *pgm, const char *p) {
    DEBUG("%s: found %s version %d.%d.%d\n", progname, p, 1, 1, 1);
    return;
}

#define sendReport(x)
#define readReport(x) 0

#if 0
static int  pickit2_rdy_led(const PROGRAMMER *pgm, int value) {
    // no rdy led
    return 0;
}

static int  pickit2_err_led(const PROGRAMMER *pgm, int value) {
    // there is no error led, so just flash the busy led a few times
    uint8_t report[65] =
    {
        0, CMD_EXEC_SCRIPT_2(9),
        SCR_BUSY_LED_ON,
        SCR_DELAY_2(.2),
        SCR_BUSY_LED_OFF,
        SCR_DELAY_2(.2),
        SCR_LOOP_3(6, 9),
        CMD_END_OF_BUFFER
    };

    // busy stops flashing by itself, so just return
    if (!value)
    {
        return 0;
    }

    return pickit2_write_report(pgm, report) != -1;
}
#endif

static int  pickit2_pgm_led(const PROGRAMMER *pgm, int value) {
    // script to set busy led appropriately
    uint8_t report[65] = {0, CMD_EXEC_SCRIPT_2(1),
                        value ? SCR_BUSY_LED_ON : SCR_BUSY_LED_OFF,
                        CMD_END_OF_BUFFER
                       };

    return pickit2_write_report(pgm, report) != -1;
}

static int  pickit2_vfy_led(const PROGRAMMER *pgm, int value) {
    // no such thing
    return 0;
}

static void pickit2_powerup(const PROGRAMMER *pgm) {
    // turn vdd on?
}

static void pickit2_powerdown(const PROGRAMMER *pgm) {
    // do what?
    pgm->disable(pgm);
}

static int  pickit2_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
    unsigned char cmd[4];
    unsigned char res[4];

    if (p->op[AVR_OP_PGM_ENABLE] == NULL)
    {
        pmsg_error("program enable instruction not defined for part %s\n", p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
    pgm->cmd(pgm, cmd, res);

    {
        int i;
        msg_debug("program_enable(): sending command. Resp = ");

        for (i = 0; i < 4; i++)
        {
            msg_debug("%x ", (int)res[i]);
        }
        msg_debug("\n");
    }

    // check for sync character
    if (res[2] != cmd[1])
        return -2;

    return 0;
}

static int  pickit2_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
    unsigned char cmd[4];
    unsigned char res[4];

    if (p->op[AVR_OP_CHIP_ERASE] == NULL)
    {
        pmsg_error("chip erase instruction not defined for part %s\n", p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));

    avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
    pgm->cmd(pgm, cmd, res);
    usleep(p->chip_erase_delay);
    pgm->initialize(pgm, p);

    return 0;
}

static int pickit2_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

    // only supporting flash & eeprom page reads
    if ((!mem->paged || page_size <= 1) || (!mem_is_flash(mem) && !mem_is_eeprom(mem)))
    {
        return -1;
    }

    DEBUG( "paged read ps %d, mem %s\n", page_size, mem->desc);

    OPCODE *readop = 0, *lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
    uint8_t data = 0, cmd[SPI_MAX_CHUNK], res[SPI_MAX_CHUNK];
    unsigned int addr_base;
    unsigned int max_addr = addr + n_bytes;

    if (lext) {
       memset(cmd, 0, sizeof(cmd));
       avr_set_bits(lext, cmd);
       avr_set_addr(lext, cmd, addr/2);
       pgm->cmd(pgm, cmd, res);
    }

    for (addr_base = addr; addr_base < max_addr; )
    {
        // bytes to send in the next packet -- not necessary as pickit2_spi() handles breaking up
        // the data into packets -- but we need to keep transfers frequent so that we can update the
        // status indicator bar
        uint32_t blockSize = MIN(65536 - (addr_base % 65536), MIN(max_addr - addr_base, SPI_MAX_CHUNK / 4));

        memset(cmd, 0, sizeof(cmd));
        memset(res, 0, sizeof(res));

        uint8_t addr_off;
        for (addr_off = 0; addr_off < blockSize; addr_off++)
        {
            int addr = addr_base + addr_off, caddr = addr;

            if (mem->op[AVR_OP_READ_LO] != NULL && mem->op[AVR_OP_READ_HI] != NULL)
            {
                if (addr & 0x00000001)
                    readop = mem->op[AVR_OP_READ_HI];
                else
                    readop = mem->op[AVR_OP_READ_LO];

                caddr /= 2;
            }
            else if (mem->op[AVR_OP_READ] != NULL)
            {
                readop = mem->op[AVR_OP_READ];
            }
            else
            {
                pmsg_error("no read command specified\n");
                return -1;
            }

            avr_set_bits(readop, &cmd[addr_off*4]);
            avr_set_addr(readop, &cmd[addr_off*4], caddr);
        }

        int bytes_read = pgm->spi(pgm, cmd, res, blockSize*4);

        if (bytes_read < 0)
        {
            pmsg_error("failed @ pgm->spi()\n");
            return -1;
        }

        DEBUG( "\npaged_load @ %X, wrote: %d, read: %d bytes\n", addr_base, blockSize*4, bytes_read);

        for (addr_off = 0; addr_off < bytes_read / 4; addr_off++)
        {
            data = 0;
            avr_get_output(readop, &res[addr_off*4], &data);
            mem->buf[addr_base + addr_off] = data;

            DEBUG( "%2X(%c)", (int)data, data<0x20?'.':data);
        }
        DEBUG( "\n");

        addr_base += blockSize;
    }

    return n_bytes;
}


static int pickit2_commit_page(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                        unsigned long addr)
{
    OPCODE * wp, * lext;

    wp = mem->op[AVR_OP_WRITEPAGE];
    if (wp == NULL)
    {
        pmsg_error("memory %s not configured for page writes\n", mem->desc);
        return -1;
    }

    // adjust the address if this memory is word-addressable
    if ((mem->op[AVR_OP_LOADPAGE_LO]) || (mem->op[AVR_OP_READ_LO]))
        addr /= 2;

    unsigned char cmd[8];
    memset(cmd, 0, sizeof(cmd));

    // use the "load extended address" command, if available
    lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
    if (lext != NULL)
    {
        avr_set_bits(lext, cmd);
        avr_set_addr(lext, cmd, addr);
    }

    // make up the write page command in the 2nd cmd position
    avr_set_bits(wp, &cmd[4]);
    avr_set_addr(wp, &cmd[4], addr);

    if (lext != NULL)
    {
        // write the load extended address cmd && the write_page cmd
        pgm->spi(pgm, cmd, NULL, 8);
    }
    else
    {
        // write just the write_page cmd
        pgm->spi(pgm, &cmd[4], NULL, 4);
    }

    // just delay the max (we could do the delay in the PICkit2 if we wanted)
    usleep(mem->max_write_delay);

    return 0;
}

// not actually a paged write, but a bulk/batch write
static int  pickit2_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
                         unsigned int page_size, unsigned int addr, unsigned int n_bytes)
{
    // only paged write for flash implemented
    if (!mem_is_flash(mem) && !mem_is_eeprom(mem))
    {
        pmsg_error("part does not support %d paged write of %s\n", page_size, mem->desc);
        return -1;
    }

    DEBUG( "page size %d mem %s supported: %d\n", page_size, mem->desc, mem->paged);
    DEBUG( "loadpagehi %x, loadpagelow %x, writepage %x\n", (int)mem->op[AVR_OP_LOADPAGE_HI], (int)mem->op[AVR_OP_LOADPAGE_LO], (int)mem->op[AVR_OP_WRITEPAGE]);

    OPCODE *writeop;
    uint8_t cmd[SPI_MAX_CHUNK], res[SPI_MAX_CHUNK];
    unsigned int addr_base;
    unsigned int max_addr = addr + n_bytes;

    for (addr_base = addr; addr_base < max_addr; )
    {
        uint32_t blockSize;

        if (mem->paged)
        {
            blockSize = MIN(page_size - (addr_base % page_size), MIN(max_addr - addr_base, SPI_MAX_CHUNK/4) );     // bytes remaining in page
        }
        else
        {
            blockSize = 1;
        }

        memset(cmd, 0, sizeof(cmd));
        memset(res, 0, sizeof(res));

        uint8_t addr_off;
        for (addr_off = 0; addr_off < blockSize; addr_off++)
        {
            int addr = addr_base + addr_off;
            int caddr = 0;

            /*
             * determine which memory opcode to use
             */
            if (mem->paged && mem->op[AVR_OP_LOADPAGE_HI] && mem->op[AVR_OP_LOADPAGE_LO])
            {
                if (addr & 0x01)
                    writeop = mem->op[AVR_OP_LOADPAGE_HI];
                else
                    writeop = mem->op[AVR_OP_LOADPAGE_LO];
                caddr = addr / 2;
            }
            else if (mem->paged && mem->op[AVR_OP_LOADPAGE_LO])
            {
                writeop = mem->op[AVR_OP_LOADPAGE_LO];
                caddr = addr;
            }
            else if (mem->op[AVR_OP_WRITE_LO])
            {
                writeop = mem->op[AVR_OP_WRITE_LO];
                caddr = addr;       // maybe this should divide by 2 & use the write_high opcode also

                pmsg_error("%s AVR_OP_WRITE_LO defined only (where is the HIGH command?)\n", mem->desc);
                return -1;
            }
            else
            {
                writeop = mem->op[AVR_OP_WRITE];
                caddr = addr;
            }

            if (writeop == NULL)
            {
                // not supported!
                return -1;
            }

            avr_set_bits(writeop, &cmd[addr_off*4]);
            avr_set_addr(writeop, &cmd[addr_off*4], caddr);
            avr_set_input(writeop, &cmd[addr_off*4], mem->buf[addr]);
        }

        int bytes_read = pgm->spi(pgm, cmd, res, blockSize*4);

        if (bytes_read < 0)
        {
            pmsg_error("failed @ pgm->spi()\n");
            return -1;
        }

        addr_base += blockSize;

        // write the page - this function looks after extended address also
        if (mem->paged && (((addr_base % page_size) == 0) || (addr_base == max_addr)))
        {
            DEBUG( "Calling pickit2_commit_page()\n");
            pickit2_commit_page(pgm, p, mem, addr_base-1);
        }
        else if (!mem->paged)
        {
            usleep(mem->max_write_delay);
        }
    }

    return n_bytes;
}


static int pickit2_cmd(const PROGRAMMER *pgm, const unsigned char *cmd,
                unsigned char *res)
{
    return pgm->spi(pgm, cmd, res, 4);
}

// breaks up the cmd[] data into  packets & sends to the pickit2. Data shifted in is stored in res[].
static int pickit2_spi(const PROGRAMMER *pgm, const unsigned char *cmd,
                unsigned char *res, int n_bytes)
{
    int retval = 0, temp1 = 0, temp2 = 0, count = n_bytes;

    while (count > 0)
    {
        uint8_t i, blockSize = MIN(count, SPI_MAX_CHUNK);
        uint8_t report[65] = {0, CMD_DOWNLOAD_DATA_2(blockSize)};
        uint8_t *repptr = report + 3;

        memset(report + 3, CMD_END_OF_BUFFER, sizeof(report) - 3);

        // append some data to write to SPI
        for (i = 0; i < blockSize; i++)
        {
            *repptr++ = *cmd++;
            count--;    // 1 less byte to pack
        }

        if (blockSize == 1)
        {
            *repptr++ = 0xa6;       //CMD_EXECUTE_SCRIPT;
            *repptr++ = 1;
            *repptr++ = SCR_SPI;
        }
        else
        {
            *repptr++ = 0xa6;       //CMD_EXECUTE_SCRIPT_2;
            *repptr++ = 4;
            *repptr++ = SCR_SPI;
            *repptr++ = 0xe9;       //SCR_LOOP_3;
            *repptr++ = 1;
            *repptr++ = blockSize - 1;
        }

        // request the data read to be sent to us
        *repptr++ = CMD_UPLOAD_DATA;

        // check return values
        if ((temp1=pickit2_write_report(pgm, report)) < 0 ||
                (temp2=pickit2_read_report(pgm, report)) < 0)
        {
            return -1;
        }/*
        else
        {
            int i;
            DEBUG( "in spi. wrote %d, read %d\n", temp1, temp2);

            for (i = 0; i < temp2; i++)
            {
                  DEBUG( "%2.2x ", report[i]);
            }

            DEBUG( "\n");
        }*/

        retval = report[1]; // upload-length field
        repptr = &report[2];    // actual data starts here

        if (res)                // copy data if user has specified a storage location
        {
            memcpy(res, repptr, retval);
            res += retval;
        }
    }

    return n_bytes;
}

#ifdef WIN32
/*
    Func: open_hid()
    Desc: finds & opens device having specified VID & PID.
    Retn: Handle of open device or INVALID_HANDLE_VALUE on fail

    Note this routine is a modified function from:
        usbhidiocDlg.cpp : implementation file
        Project: usbhidioc.cpp
        Version: 3.0
        Date: 7/18/05
        by Jan Axelson (jan@Lvr.com)
*/
static HANDLE open_hid(unsigned short vid, unsigned short pid)
{
    //Use a series of API calls to find a HID with a specified Vendor IF and Product ID.
    HANDLE                              returnHandle = INVALID_HANDLE_VALUE;
    HIDD_ATTRIBUTES                     Attributes;
//    DWORD                               DeviceUsage;
    SP_DEVICE_INTERFACE_DATA            devInfoData;
    BOOL                                LastDevice = FALSE;
    int                                 MemberIndex = 0;
    LONG                                Result;

    // were global, now just local scrap
    DWORD                               Length = 0;
    PSP_DEVICE_INTERFACE_DETAIL_DATA    detailData = NULL;
    HANDLE                              DeviceHandle=NULL;
    GUID                                HidGuid;
    HANDLE                              hDevInfo;
    ULONG                               Required;
    BOOL                                MyDeviceDetected = 0;

    /*
    API function: HidD_GetHidGuid
    Get the GUID for all system HIDs.
    Returns: the GUID in HidGuid.
    */

    HidD_GetHidGuid(&HidGuid);
    DEBUG("\nHidD_GetHidGuid returned.\n");

    /*
    API function: SetupDiGetClassDevs
    Returns: a handle to a device information set for all installed devices.
    Requires: the GUID returned by GetHidGuid.
    */

    hDevInfo=SetupDiGetClassDevs
             (&HidGuid,
              NULL,
              NULL,
              DIGCF_PRESENT|DIGCF_INTERFACEDEVICE);

    DEBUG("\nSetupDiGetClassDevs returned 0x%x\n", hDevInfo);
    devInfoData.cbSize = sizeof(devInfoData);

    //Step through the available devices looking for the one we want.
    //Quit on detecting the desired device or checking all available devices without success.

    MemberIndex = 0;
    LastDevice = FALSE;

    do
    {
        /*
        API function: SetupDiEnumDeviceInterfaces
        On return, MyDeviceInterfaceData contains the handle to a
        SP_DEVICE_INTERFACE_DATA structure for a detected device.
        Requires:
        The DeviceInfoSet returned in SetupDiGetClassDevs.
        The HidGuid returned in GetHidGuid.
        An index to specify a device.
        */


        Result=SetupDiEnumDeviceInterfaces
               (hDevInfo,
                0,
                &HidGuid,
                MemberIndex,
                &devInfoData);

        DEBUG("\nSetupDiEnumDeviceInterfaces returned 0x%x\n", Result);

        if (Result != 0)
        {
            //A device has been detected, so get more information about it.

            /*
            API function: SetupDiGetDeviceInterfaceDetail
            Returns: an SP_DEVICE_INTERFACE_DETAIL_DATA structure
            containing information about a device.
            To retrieve the information, call this function twice.
            The first time returns the size of the structure in Length.
            The second time returns a pointer to the data in DeviceInfoSet.
            Requires:
            A DeviceInfoSet returned by SetupDiGetClassDevs
            The SP_DEVICE_INTERFACE_DATA structure returned by SetupDiEnumDeviceInterfaces.

            The final parameter is an optional pointer to an SP_DEV_INFO_DATA structure.
            This application doesn't retrieve or use the structure.
            If retrieving the structure, set
            MyDeviceInfoData.cbSize = length of MyDeviceInfoData.
            and pass the structure's address.
            */

            //Get the Length value.
            //The call will return with a "buffer too small" error which can be ignored.
            Result = SetupDiGetDeviceInterfaceDetail
                     (hDevInfo,
                      &devInfoData,
                      NULL,
                      0,
                      &Length,
                      NULL);

            DEBUG("\nSetupDiGetDeviceInterfaceDetail returned 0x%x\n", Result);

            //Allocate memory for the hDevInfo structure, using the returned Length.

            detailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(Length);

            //Set cbSize in the detailData structure.

            detailData -> cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

            //Call the function again, this time passing it the returned buffer size.

            Result = SetupDiGetDeviceInterfaceDetail
                     (hDevInfo,
                      &devInfoData,
                      detailData,
                      Length,
                      &Required,
                      NULL);

            // Open a handle to the device.
            // To enable retrieving information about a system mouse or keyboard,
            // don't request Read or Write access for this handle.

            /*
            API function: CreateFile
            Returns: a handle that enables reading and writing to the device.
            Requires:
            The DevicePath in the detailData structure
            returned by SetupDiGetDeviceInterfaceDetail.
            */

            DeviceHandle=CreateFile
                         (detailData->DevicePath,
                          0,
                          FILE_SHARE_READ|FILE_SHARE_WRITE,
                          (LPSECURITY_ATTRIBUTES)NULL,
                          OPEN_EXISTING,
                          0,
                          NULL);

            DEBUG("CreateFile(): %s\n", detailData->DevicePath);
            /*
            API function: HidD_GetAttributes
            Requests information from the device.
            Requires: the handle returned by CreateFile.
            Returns: a HIDD_ATTRIBUTES structure containing
            the Vendor ID, Product ID, and Product Version Number.
            Use this information to decide if the detected device is
            the one we're looking for.
            */

            //Set the Size to the number of bytes in the structure.

            Attributes.Size = sizeof(Attributes);

            Result = HidD_GetAttributes
                     (DeviceHandle,
                      &Attributes);

            DEBUG("HidD_GetAttributes returned 0x%x\n", Result);
            DEBUG("VID: %.4X PID: %.4X\n", Attributes.VendorID, Attributes.ProductID);

            //Is it the desired device?
            MyDeviceDetected = FALSE;
            if (Attributes.VendorID == vid)
            {
                if (Attributes.ProductID == pid)
                {
                    //Both the Vendor ID and Product ID match.

                    MyDeviceDetected = TRUE;

                    // Get a handle for us to use.

                    returnHandle = CreateFile
                                   (detailData->DevicePath,
                                    GENERIC_WRITE | GENERIC_READ,
                                    FILE_SHARE_READ|FILE_SHARE_WRITE,
                                    (LPSECURITY_ATTRIBUTES)NULL,
                                    OPEN_EXISTING,
                                    FILE_FLAG_OVERLAPPED,
                                    NULL);

                } //if (Attributes.ProductID == ProductID)

                else
                    //The Product ID doesn't match.

                    CloseHandle(DeviceHandle);

            } //if (Attributes.VendorID == VendorID)

            else
                //The Vendor ID doesn't match.

                CloseHandle(DeviceHandle);

            //Free the memory used by the detailData structure (no longer needed).

            free(detailData);

        }  //if (Result != 0)

        else
            //SetupDiEnumDeviceInterfaces returned 0, so there are no more devices to check.

            LastDevice=TRUE;

        //If we haven't found the device yet, and haven't tried every available device,
        //try the next one.

        MemberIndex = MemberIndex + 1;

    } //do

    while ((LastDevice == FALSE) && (MyDeviceDetected == FALSE));

    if (MyDeviceDetected == FALSE)
        DEBUG("Device not detected\n");
    else
        DEBUG("Device detected\n");

    //Free the memory reserved for hDevInfo by SetupDiClassDevs.

    DEBUG("Calling SetupDiDestroyDeviceInfoList\n");
    SetupDiDestroyDeviceInfoList(hDevInfo);

    return returnHandle;
}

// simple read with timeout
static int usb_read_interrupt(const PROGRAMMER *pgm, void *buff, int size, int timeout) {
    OVERLAPPED ovr;
    DWORD bytesRead = 0;

    if (PDATA(pgm)->read_event == NULL)
    {
        PDATA(pgm)->read_event = CreateEvent(0, 0, 0, 0);
    }

    memset(&ovr, 0, sizeof(ovr));
    ovr.hEvent = PDATA(pgm)->read_event;

    ReadFile(PDATA(pgm)->usb_handle, buff, size, &bytesRead, &ovr);
    if (WaitForSingleObject(PDATA(pgm)->read_event, timeout) == WAIT_TIMEOUT)
    {
        CancelIo(PDATA(pgm)->usb_handle);
        return -1;
    }

    GetOverlappedResult(PDATA(pgm)->usb_handle, &ovr, &bytesRead, 0);

    return bytesRead > 0? (int) bytesRead: -1;
}

// simple write with timeout
static int usb_write_interrupt(const PROGRAMMER *pgm, const void *buff, int size, int timeout) {
    OVERLAPPED ovr;
    DWORD bytesWritten = 0;

    if (PDATA(pgm)->write_event == NULL)
    {
        PDATA(pgm)->write_event = CreateEvent(0, 0, 0, 0);
    }

    memset(&ovr, 0, sizeof(ovr));
    ovr.hEvent = PDATA(pgm)->write_event;

    WriteFile(PDATA(pgm)->usb_handle, buff, size, &bytesWritten, &ovr);
    if (WaitForSingleObject(PDATA(pgm)->write_event, timeout) == WAIT_TIMEOUT)
    {
        CancelIo(PDATA(pgm)->usb_handle);
        return -1;
    }

    GetOverlappedResult(PDATA(pgm)->usb_handle, &ovr, &bytesWritten, 0);

    return bytesWritten > 0? (int) bytesWritten: -1;
}

static int pickit2_write_report(const PROGRAMMER *pgm, const unsigned char report[65]) {
    return usb_write_interrupt(pgm, report, 65, PDATA(pgm)->transaction_timeout); // XXX
}

static int pickit2_read_report(const PROGRAMMER *pgm, unsigned char report[65]) {
    return usb_read_interrupt(pgm, report, 65, PDATA(pgm)->transaction_timeout);
}

#else   // WIN32
/* taken (modified) from avrdude usbasp.c */
static int usb_open_device(struct usb_dev_handle **device, int vendor, int product)
{
    struct usb_bus      *bus;
    struct usb_device   *dev;
    usb_dev_handle      *handle = NULL;
    int                 errorCode = USB_ERROR_NOTFOUND;
    static int          didUsbInit = 0;

    if (!didUsbInit)
    {
        didUsbInit = 1;
        usb_init();
    }
    usb_find_busses();
    usb_find_devices();
    for (bus=usb_get_busses(); bus; bus=bus->next)
    {
        for (dev=bus->devices; dev; dev=dev->next)
        {
            DEBUG( "Enumerating device list.. VID: 0x%4.4x, PID: 0x%4.4x\n", dev->descriptor.idVendor, dev->descriptor.idProduct);
            if (dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product)
            {
                /* we need to open the device in order to query strings */
                handle = usb_open(dev);
                if (handle == NULL)
                {
                    errorCode = USB_ERROR_ACCESS;
                    pmsg_warning("cannot open USB device: %s\n", usb_strerror());
                    continue;
                }

                // return with opened device handle
                else
                {
                    msg_notice("device %p seemed to open OK\n", handle);

                    if ((errorCode = usb_set_configuration(handle, 1)) < 0)
                    {
                        pmsg_ext_error("cannot set configuration, error code %d, %s\n"
                          "you may need to run avrdude as root or set up correct usb port permissions",
                          errorCode, usb_strerror());
                    }

                    if ((errorCode = usb_claim_interface(handle, 0)) < 0)
                    {
                        pmsg_ext_error("cannot claim interface, error code %d, %s\n"
                           "You may need to run avrdude as root or set up correct usb port permissions.",
                           errorCode, usb_strerror());
                    }

                    errorCode = 0;
                    *device = handle;
                    return 0;
                }
            }
        }
    }

    return -1;
}

static int pickit2_write_report(const PROGRAMMER *pgm, const unsigned char report[65]) {
    // endpoint 1 OUT??
    return usb_interrupt_write(PDATA(pgm)->usb_handle, USB_ENDPOINT_OUT | 1, (char*)(report+1), 64, PDATA(pgm)->transaction_timeout);
}

static int pickit2_read_report(const PROGRAMMER *pgm, unsigned char report[65]) {
    // endpoint 1 IN??
    return usb_interrupt_read(PDATA(pgm)->usb_handle, USB_ENDPOINT_IN | 1, (char*)(report+1), 64, PDATA(pgm)->transaction_timeout);
}
#endif  // WIN32

static int  pickit2_parseextparams(const PROGRAMMER *pgm, const LISTID extparms) {
    LNODEID ln;
    const char *extended_param;
    int rv = 0;

    for (ln = lfirst(extparms); ln; ln = lnext(ln))
    {
        extended_param = ldata(ln);

        if (str_starts(extended_param, "clockrate="))
        {
            int clock_rate;
            if (sscanf(extended_param, "clockrate=%i", &clock_rate) != 1 || clock_rate <= 0)
            {
                pmsg_error("invalid clockrate '%s'\n", extended_param);
                rv = -1;
                continue;
            }

            int clock_period = MIN(1000000 / clock_rate, 255);    // max period is 255
            clock_rate = (int)(1000000 / (clock_period + 5e-7));    // assume highest speed is 2MHz - should probably check this

            pmsg_notice2("pickit2_parseextparms(): clockrate set to 0x%02x\n", clock_rate);
            PDATA(pgm)->clock_period = clock_period;

            continue;
        }

        if (str_starts(extended_param, "timeout="))
        {
            int timeout;
            if (sscanf(extended_param, "timeout=%i", &timeout) != 1 || timeout <= 0)
            {
                pmsg_error("invalid timeout '%s'\n", extended_param);
                rv = -1;
                continue;
            }

            pmsg_notice2("pickit2_parseextparms(): usb timeout set to 0x%02x\n", timeout);
            PDATA(pgm)->transaction_timeout = timeout;

            continue;
        }
        if (str_eq(extended_param, "help")) {
            msg_error("%s -c %s extended options:\n", progname, pgmid);
            msg_error("  -xclockrate=<arg> Set the SPI clocking rate in <arg> [Hz]\n");
            msg_error("  -xtimeout=<arg>   Set the timeout for USB read/write to <arg> [ms]\n");
            msg_error("  -xhelp            Show this help menu and exit\n");
            exit(0);
        }

        pmsg_error("invalid extended parameter '%s'\n", extended_param);
        rv = -1;
    }

    return rv;
}


void pickit2_initpgm(PROGRAMMER *pgm) {
    /*
     * mandatory functions - these are called without checking to see
     * whether they are assigned or not
     */

    pgm->initialize     = pickit2_initialize;
    pgm->display        = pickit2_display;
    pgm->enable         = pickit2_enable;
    pgm->disable        = pickit2_disable;
    pgm->powerup        = pickit2_powerup;
    pgm->powerdown      = pickit2_powerdown;
    pgm->program_enable = pickit2_program_enable;
    pgm->chip_erase     = pickit2_chip_erase;
    pgm->open           = pickit2_open;
    pgm->close          = pickit2_close;

    pgm->read_byte      = avr_read_byte_default;
    pgm->write_byte     = avr_write_byte_default;

    /*
     * predefined functions - these functions have a valid default
     * implementation. Hence, they don't need to be defined in
     * the programmer.
     */
    //pgm->rdy_led        = pickit2_rdy_led;
    //pgm->err_led        = pickit2_err_led;
    pgm->pgm_led        = pickit2_pgm_led;
    pgm->vfy_led        = pickit2_vfy_led;

    /*
     * optional functions - these are checked to make sure they are
     * assigned before they are called
     */

    pgm->cmd            = pickit2_cmd;
    pgm->spi            = pickit2_spi;
    pgm->paged_write    = pickit2_paged_write;
    pgm->paged_load     = pickit2_paged_load;
    //pgm->write_setup    = NULL;
    //pgm->read_sig_bytes = NULL;
    //pgm->set_vtarget    = NULL;//pickit2_vtarget;
    //pgm->set_varef      = NULL;
    //pgm->set_fosc       = NULL;
    //pgm->perform_osccal = NULL;

    pgm->parseextparams = pickit2_parseextparams;

    pgm->setup          = pickit2_setup;
    pgm->teardown       = pickit2_teardown;
    // pgm->page_size      = 256;        // not sure what this does ... maybe the max page size that the page read/write function can handle

    strncpy(pgm->type, "pickit2", sizeof(pgm->type));
}
#else
static int pickit2_nousb_open(PROGRAMMER *pgm, const char *name) {
    pmsg_error(
#ifdef WIN32
            "no usb or hid support; please compile again with libusb or HID support from Win32 DDK installed\n"
#else
            "no usb support; please compile again with libusb installed\n"
#endif
     );

    return -1;
}

void pickit2_initpgm(PROGRAMMER *pgm) {
    /*
     * mandatory functions - these are called without checking to see
     * whether they are assigned or not
     */

    pgm->open           = pickit2_nousb_open;

    strncpy(pgm->type, "pickit2", sizeof(pgm->type));
}

#endif /* defined(HAVE_LIBUSB) || defined(WIN32) */

const char pickit2_desc[] = "Microchip's PICkit2 Programmer";

