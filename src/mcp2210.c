/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2026 David Hubner
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
 *
 * This code is not affiliated in any way with Microchip®
 */

#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#if defined(HAVE_LIBHIDAPI)

#include <hidapi/hidapi.h>


#define MCP2210_CHIP_STATUS 0x10
#define MCP2210_SPI_CANCEL_TRANSFER 0x11
#define MCP2210_SET_SETTINGS 0x40
#define MCP2210_GET_SETTINGS 0x41
#define MCP2210_SPI_XFER_COMMAND 0x42

#define MCP2210_SUCCESS 0x00
#define MCP2210_TRANSFER_IN_PROGRESS 0xF8
#define MCP2210_SPI_BUS_UNAVALIABLE 0xF7
#define MCP2210_XFER_DONE 0x10
#define MCP2210_XFER_STARTED 0x20
#define MCP2210_XFER_NOT_FINISHED 0x30

#define MCP2210_DEFAULT_VID 0x04d8
#define MCP2210_DEFAULT_PID 0x00DE
#define HID_REPORT_LENGTH 65
#define TRANSFER_RETRIES 30
#define SPI_MAX_CHUNK 60

static bool mcp2210_run_query(const PROGRAMMER *p, const unsigned char request_report[65], unsigned char *response_report);
static bool mcp2210_cancel_transfer(const PROGRAMMER *p);

struct mcp {
    hid_device *handle;
    uint32_t bitrate;
};

#define mine ((struct mcp *)(p->cookie))

static void mcp2210_setup(PROGRAMMER *p) {
    int sizeof_mcp = sizeof(struct mcp);

    if ((p->cookie = malloc(sizeof_mcp)) == 0) {
        msg_error("unable to allocate %i bytes, exiting\n", sizeof_mcp);
        exit(1);
    }

    memset(p->cookie, 0, sizeof_mcp);

    mine->bitrate = 0;
}

static bool mcp2210_set_settings(const PROGRAMMER *p, unsigned short len) {
    unsigned char settings_request[HID_REPORT_LENGTH], settings_response[HID_REPORT_LENGTH];
    uint32_t *current_bit_rate = &mine->bitrate;

    memset(settings_request,0, HID_REPORT_LENGTH);

    if(!*current_bit_rate) {
        unsigned char current_settings[HID_REPORT_LENGTH];

        if(mcp2210_run_query(p, (const unsigned char[65]){ 0x00, MCP2210_GET_SETTINGS }, current_settings)) {
            msg_error("unable to run query to fetch current settings\n");
            return false;
        }

        current_bit_rate = (uint32_t *)&current_settings[5];
        mine->bitrate = *current_bit_rate;
    }

    settings_request[1] = MCP2210_SET_SETTINGS;
    settings_request[5] = (uint8_t)(*current_bit_rate >> 0) & 0xFF;
    settings_request[6] = (uint8_t)(*current_bit_rate >> 8) & 0xFF;
    settings_request[7] = (uint8_t)(*current_bit_rate >> 16) & 0xFF;
    settings_request[8] = (uint8_t)(*current_bit_rate >> 24) & 0xFF;
    settings_request[19] = (len >> 0) & 0xFF;
    settings_request[20] = (len >> 8) & 0xFF;

    if(mcp2210_run_query(p, settings_request, settings_response)) {
        msg_error("unable to run query to set settings\n");
        return false;
    }

    if(settings_response[2] != MCP2210_SUCCESS){
        if(settings_response[2] == MCP2210_TRANSFER_IN_PROGRESS) {
            msg_error("USB transfer in progress - Settings not written\n");
        } else
            msg_error("unknown error, return code: %x\n", settings_response[2]);

        return false;
    }

    return true;
}

static void mcp2210_teardown(PROGRAMMER *p) {
    free(p->cookie);
}

static int mcp2210_open(PROGRAMMER *p, const char * port) {
    LNODEID usbpid = lfirst(p->usbpid);
    int vid = p->usbvid ? p->usbvid : MCP2210_DEFAULT_VID, pid;

    if(usbpid) {
        pid = *(int *) ldata(usbpid);

        if(lnext(usbpid))
            msg_warning("using PID 0x%04x, ignoring remaining PIDs in list\n", pid);
    } else
            pid = MCP2210_DEFAULT_PID;

    mine->handle = hid_open(vid, pid, NULL);
    if (mine->handle == NULL) {
        msg_error("could not find mcp2210 device with vid=0x%x pid=0x%x\n", vid, pid);
        return -1;
    }

    return 0;
}

static void mcp2210_close(PROGRAMMER *p) {
    if(!mcp2210_cancel_transfer(p))
        return;

    hid_close(mine->handle);
}

static int mcp2210_initialize(const PROGRAMMER *p, const AVRPART *a) {
    if (p->program_enable) {
        return p->program_enable(p, a);
    } else
        return -1;
}

static bool mcp2210_cancel_transfer(const PROGRAMMER *p) {
    unsigned char cancel_response[HID_REPORT_LENGTH];

    if(mcp2210_run_query(p, (const unsigned char[65]){ 0x00, MCP2210_SPI_CANCEL_TRANSFER }, cancel_response)) {
        msg_error("unable to run query to cancel response\n");
        return false;
    }

    if(cancel_response[2] != MCP2210_SUCCESS) {
        msg_error("unable to cancel transfer - error code: %i", cancel_response[3]);
        return false;
    }

    return true;
}

static void mcp2210_disable(const PROGRAMMER *p) {
    return;
}

static void mcp2210_enable(PROGRAMMER *p, const AVRPART *a) {
    return;
}

static void mcp2210_display(const PROGRAMMER *p, const char * s) {
    uint8_t status_request[HID_REPORT_LENGTH] = {0, 0x10},  status_response[HID_REPORT_LENGTH];

    if(mcp2210_run_query(p, status_request, status_response)) {
        msg_error("unable to run query to get status\n");
        return;
    }

    if(status_response[2] != MCP2210_SUCCESS) {
        msg_error("unable to get chip status\n");
        return;
    }

    msg_info("SPI Bus Current Owner: ");
    switch(status_response[4]) {
        case 0x00:
            msg_info("no Owner\n");
            break;
        case 0x01:
            msg_info("spi Bridge\n");
            break;
        case 0x02:
            msg_info("external Host\n");
            break;
        default:
            msg_info("unknown Response\n");
    }

    msg_info("password Attempts: %i\n", status_response[5]);
    return;
}

static void mcp2210_powerup(const PROGRAMMER *p) {
    return;
}

static void mcp2210_powerdown(const PROGRAMMER *p) {
    p->disable(p);
}

static int mcp2210_program_enable(const PROGRAMMER *p, const AVRPART *a) {
    unsigned char cmd[4], res[4];

    if (a->op[AVR_OP_PGM_ENABLE] == NULL) {
        msg_error("program enable instruction not defined for part \"%s\"\n", p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));
    avr_set_bits(a->op[AVR_OP_PGM_ENABLE], cmd);
    p->cmd(p, cmd, res);

    if (res[2] != cmd[1])
        return -2;

    return 0;
}

static int mcp2210_chip_erase(const PROGRAMMER *p, const AVRPART *a) {
    unsigned char cmd[4], res[4];

    if (a->op[AVR_OP_CHIP_ERASE] == NULL) {
        msg_error("chip erase instruction not defined for part \"%s\"\n",
                p->desc);
        return -1;
    }

    memset(cmd, 0, sizeof(cmd));

    avr_set_bits(a->op[AVR_OP_CHIP_ERASE], cmd);
    p->cmd(p, cmd, res);
    usleep(a->chip_erase_delay);
    p->initialize(p, a);

    return 0;
}

static int mcp2210_paged_load(const PROGRAMMER *p, const AVRPART *a, const AVRMEM * mem,
                        unsigned int page_size, unsigned int addr, unsigned int n_bytes) {
    // Only supporting flash & eeprom page reads
    if ((!mem->paged || page_size <= 1) || (!mem_is_flash(mem) && !mem_is_eeprom(mem)))
        return -1;

    OPCODE *readop = 0, *lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
    uint8_t data = 0, cmd[SPI_MAX_CHUNK], res[SPI_MAX_CHUNK];
    unsigned int addr_base;
    unsigned int max_addr = addr + n_bytes;

    if (lext) {
        memset(cmd, 0, sizeof(cmd));
        avr_set_bits(lext, cmd);
        avr_set_addr(lext, cmd, addr/2);
        p->cmd(p, cmd, res);
    }

    for (addr_base = addr; addr_base < max_addr;) {
        uint32_t blockSize = minm(65536 - (addr_base % 65536), minm(max_addr - addr_base, SPI_MAX_CHUNK / 4));

        memset(cmd, 0, sizeof(cmd));
        memset(res, 0, sizeof(res));

        uint8_t addr_off;

        for (addr_off = 0; addr_off < blockSize; addr_off++) {
            int addr = addr_base + addr_off, caddr = addr;

            if (mem->op[AVR_OP_READ_LO] != NULL && mem->op[AVR_OP_READ_HI] != NULL) {
                if (addr & 0x00000001)
                    readop = mem->op[AVR_OP_READ_HI];
                else
                    readop = mem->op[AVR_OP_READ_LO];

                caddr /= 2;
            } else if (mem->op[AVR_OP_READ] != NULL) {
                readop = mem->op[AVR_OP_READ];
            } else {
                msg_error("no read command specified\n");
                return -1;
            }

            avr_set_bits(readop, &cmd[addr_off*4]);
            avr_set_addr(readop, &cmd[addr_off*4], caddr);
        }

        int bytes_read = p->spi(p, cmd, res, blockSize*4);

        if (bytes_read < 0) {
            msg_error("Failed @ p->spi()\n");
            return -1;
        }

        for (addr_off = 0; addr_off < bytes_read / 4; addr_off++) {
            data = 0;
            avr_get_output(readop, &res[addr_off*4], &data);
            mem->buf[addr_base + addr_off] = data;
        }

        addr_base += blockSize;
    }

    return n_bytes;
}

static int mcp2210_commit_page(const PROGRAMMER *p, const AVRPART *a, const AVRMEM * mem, unsigned long addr) {
    OPCODE * wp, * lext;

    wp = mem->op[AVR_OP_WRITEPAGE];
    if (wp == NULL) {
        msg_error("memory \"%s\" not configured for page writes\n", mem->desc);
        return -1;
    }

    // Adjust the address if this memory is word-addressable
    if ((mem->op[AVR_OP_LOADPAGE_LO]) || (mem->op[AVR_OP_READ_LO]))
        addr /= 2;

    unsigned char cmd[8];
    memset(cmd, 0, sizeof(cmd));

    // Use the "load extended address" command, if available
    lext = mem->op[AVR_OP_LOAD_EXT_ADDR];
    if (lext != NULL) {
        avr_set_bits(lext, cmd);
        avr_set_addr(lext, cmd, addr);
    }

    // Make up the write page command in the 2nd cmd position
    avr_set_bits(wp, &cmd[4]);
    avr_set_addr(wp, &cmd[4], addr);

    if (lext != NULL) {
        // Write the load extended address cmd && the write_page cmd
        p->spi(p, cmd, NULL, 8);
    }
    else {
        // Write just the write_page cmd
        p->spi(p, &cmd[4], NULL, 4);
    }

    // Just delay the max
    usleep(mem->max_write_delay);

    return 0;
}

static int mcp2210_paged_write(const PROGRAMMER *p, const AVRPART *a, const AVRMEM * mem, unsigned int page_size, unsigned int addr, unsigned int n_bytes) {
    if (!mem_is_flash(mem) && !mem_is_eeprom(mem)) {
        msg_info("part does not support %d paged write of %s\n", page_size, mem->desc);
        return -1;
    }

    OPCODE *writeop;
    uint8_t cmd[SPI_MAX_CHUNK], res[SPI_MAX_CHUNK];
    unsigned int addr_base;
    unsigned int max_addr = addr + n_bytes;

    for (addr_base = addr; addr_base < max_addr; ) {
        uint32_t blockSize;

        if (mem->paged) {
            blockSize = minm(page_size - (addr_base % page_size), minm(max_addr - addr_base, SPI_MAX_CHUNK/4) );     // Bytes remaining in page
        } else
            blockSize = 1;

        memset(cmd, 0, sizeof(cmd));
        memset(res, 0, sizeof(res));

        uint8_t addr_off;
        for (addr_off = 0; addr_off < blockSize; addr_off++) {
            int addr = addr_base + addr_off;
            int caddr = 0;

            /*
             * Determine which memory opcode to use
             */
            if (mem->paged && mem->op[AVR_OP_LOADPAGE_HI] && mem->op[AVR_OP_LOADPAGE_LO]) {
                if (addr & 0x01)
                    writeop = mem->op[AVR_OP_LOADPAGE_HI];
                else
                    writeop = mem->op[AVR_OP_LOADPAGE_LO];
                caddr = addr / 2;
            } else if (mem->paged && mem->op[AVR_OP_LOADPAGE_LO]) {
                writeop = mem->op[AVR_OP_LOADPAGE_LO];
                caddr = addr;
            } else if (mem->op[AVR_OP_WRITE_LO]) {
                writeop = mem->op[AVR_OP_WRITE_LO];
                caddr = addr;

                msg_error("error AVR_OP_WRITE_LO defined only (where's the HIGH command?)\n");
                return -1;
            } else {
                writeop = mem->op[AVR_OP_WRITE];
                caddr = addr;
            }

            if (writeop == NULL)
                return -1;

            avr_set_bits(writeop, &cmd[addr_off*4]);
            avr_set_addr(writeop, &cmd[addr_off*4], caddr);
            avr_set_input(writeop, &cmd[addr_off*4], mem->buf[addr]);
        }

        int bytes_read = p->spi(p, cmd, res, blockSize*4);

        if (bytes_read < 0) {
            msg_error("failed @ p->spi()\n");
            return -1;
        }

        addr_base += blockSize;

        if (mem->paged && (((addr_base % page_size) == 0) || (addr_base == max_addr))) {
            mcp2210_commit_page(p, a, mem, addr_base-blockSize);
        } else if (!mem->paged) {
            usleep(mem->max_write_delay);
        }
    }

    return n_bytes;
}

static int mcp2210_cmd(const PROGRAMMER *p, const unsigned char *cmd, unsigned char *res) {
    return p->spi(p, cmd, res, 4);
}

static int mcp2210_spi(const PROGRAMMER *p, const unsigned char *cmd, unsigned char *res, int n_bytes) {
    if(n_bytes > SPI_MAX_CHUNK) {
        msg_error("spi chunk longer than %i", n_bytes);
        return -1;
    }

    mcp2210_set_settings(p, n_bytes);

    uint8_t transfer_request[HID_REPORT_LENGTH] = {0, MCP2210_SPI_XFER_COMMAND, (uint8_t)n_bytes},  transfer_response[HID_REPORT_LENGTH];
    int retries = 0, timeout = (100000/mine->bitrate) + 1;
    bool success = false;

    for (int i = 5; i < n_bytes + 5; i++)
        transfer_request[i] = *cmd++;

    if(mcp2210_run_query(p, transfer_request, transfer_response)) {
        msg_error("unable to run query to send spi command\n");
        return -1;
    }

    while(retries < TRANSFER_RETRIES)
    {
        usleep(timeout * 1000);

        if(mcp2210_run_query(p, (const unsigned char[65]){ 0x00, MCP2210_SPI_XFER_COMMAND, 0 }, transfer_response)) {
            msg_error("unable to run query to send spi command\n");
            return -1;
        }

        if(transfer_response[2] != MCP2210_SUCCESS) {
            if(transfer_response[2] == MCP2210_TRANSFER_IN_PROGRESS)
                msg_debug("SPI transfer in progress - cannot accept any data for the moment\n");

            if(transfer_response[2] == MCP2210_SPI_BUS_UNAVALIABLE) {
                msg_error("SPI bus not available\n");
                return -1;
            }
        } else {
            if(transfer_response[4] == MCP2210_XFER_DONE) {
                msg_debug("SPI transfer finished - no more data to send\n");
                success = true;
                break;
            }

            if(transfer_response[4] -= MCP2210_XFER_NOT_FINISHED)
                msg_debug("SPI transfer not finished; received data available\n");

            if(transfer_response[4] == MCP2210_XFER_STARTED)
                msg_debug("SPI transfer started - no data to receive\n");
        }

        retries++;
    }

    if(!success) {
        msg_error("transfer failed\n");
        return -1;
    }

    if (res)
        memcpy(res, &transfer_response[5], transfer_response[3]);

    return n_bytes;
}

static bool mcp2210_run_query(const PROGRAMMER *p, const unsigned char request_report[HID_REPORT_LENGTH], unsigned char *response_report) {
    if(hid_write(mine->handle, request_report, HID_REPORT_LENGTH) == -1) {
        msg_error("HID Write has failed with error: %ls", hid_error(mine->handle));
        return true;
    }

    memset(response_report, 0, sizeof(HID_REPORT_LENGTH));

    if(hid_read(mine->handle, (response_report + 1), (HID_REPORT_LENGTH - 1)) == -1) {
        msg_error("HID read has failed with error: %ls", hid_error(mine->handle));
        return true;
    }

    return false;
}

static int mcp2210_parseextparams(const PROGRAMMER *p, LISTID extparms) {
    const char *extended_param;

    for (LNODEID ln = lfirst(extparms); ln; ln = lnext(ln)) {
        extended_param = ldata(ln);

        if (strncmp(extended_param, "bitrate=", strlen("bitrate=")) == 0) {
            if (sscanf(extended_param, "bitrate==%u", &mine->bitrate) != 1) {
                msg_error("invalid bitrate: %s\n", extended_param);
                return -1;
            }
        }
    }

    return 0;
}

void mcp2210_initpgm(PROGRAMMER *p) {
    /*
     * Mandatory functions - these are called without checking to see
     * whether they are assigned or not
     */

    p->initialize     = mcp2210_initialize;
    p->display        = mcp2210_display;
    p->enable         = mcp2210_enable;
    p->disable        = mcp2210_disable;
    p->powerup        = mcp2210_powerup;
    p->powerdown      = mcp2210_powerdown;
    p->program_enable = mcp2210_program_enable;
    p->chip_erase     = mcp2210_chip_erase;
    p->open           = mcp2210_open;
    p->close          = mcp2210_close;

    p->read_byte      = avr_read_byte_default;
    p->write_byte     = avr_write_byte_default;

    /*
     * Optional functions - these are checked to make sure they are
     * assigned before they are called
     */

    p->cmd            = mcp2210_cmd;
    p->spi            = mcp2210_spi;
    p->paged_write    = mcp2210_paged_write;
    p->paged_load     = mcp2210_paged_load;
    p->parseextparams = mcp2210_parseextparams;
    p->setup          = mcp2210_setup;
    p->teardown       = mcp2210_teardown;
}

#else

static int mcp2210_nohidapi_open (const PROGRAMMER *p, char * name) {
    msg_info("no hidapi support. Please compile again with hidapi installed.\n");
    return -1;
}

void mcp2210_initpgm(const PROGRAMMER *p) {
    /*
     * Mandatory functions - these are called without checking to see
     * whether they are assigned or not
     */

    p->open           = mcp2210_nohidapi_open;
}

#endif

const char mcp2210_desc[] = "MCP2210 IC";

