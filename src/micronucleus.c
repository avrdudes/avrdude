/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2019 Marius Greuel
 * Portions Copyright (C) 2014 T. Bo"scke
 * Portions Copyright (C) 2012 ihsan Kehribar
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

// Notes:
// This file adds support for the Micronucleus bootloader V1 and V2,
// so you do no longer need the Micronucleus command-line utility.
//
// This bootloader is typically used on small ATtiny boards,
// such as Digispark (ATtiny85), Digispark Pro (ATtiny167),
// and the respective clones.
// By default, it bootloader uses the VID/PID 16d0:0753 (MCS Digistump).
//
// As the micronucleus bootloader is optimized for size, it implements
// writing to flash memory only. Since it does not support reading,
// use the -V option to prevent avrdude from verifing the flash memory.
// To have avrdude wait for the device to be connected, use the
// extended option '-x wait'.
//
// Example:
// avrdude -c micronucleus -p t85 -x wait -V -U flash:w:main.hex

#include "ac_cfg.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include "avrdude.h"
#include "micronucleus.h"
#include "usbdevs.h"

#if defined(HAVE_LIBUSB)

#if defined(HAVE_USB_H)
#include <usb.h>
#elif defined(HAVE_LUSB0_USB_H)
#include <lusb0_usb.h>
#else
#error "libusb needs either <usb.h> or <lusb0_usb.h>"
#endif

//-----------------------------------------------------------------------------

#define MICRONUCLEUS_VID 0x16D0
#define MICRONUCLEUS_PID 0x0753

#define MICRONUCLEUS_CONNECT_WAIT 100

#define MICRONUCLEUS_CMD_INFO 0
#define MICRONUCLEUS_CMD_TRANSFER 1
#define MICRONUCLEUS_CMD_ERASE 2
#define MICRONUCLEUS_CMD_PROGRAM 3
#define MICRONUCLEUS_CMD_START 4

#define MICRONUCLEUS_DEFAULT_TIMEOUT 500
#define MICRONUCLEUS_MAX_MAJOR_VERSION 2

#define PDATA(pgm) ((pdata_t*)(pgm->cookie))

//-----------------------------------------------------------------------------

typedef struct pdata
{
    usb_dev_handle* usb_handle;
    // Extended parameters
    bool wait_until_device_present;
    int wait_timout;            // in seconds
    // Bootloader version
    uint8_t major_version;
    uint8_t minor_version;
    // Bootloader info (via USB request)
    uint16_t flash_size;        // programmable size (in bytes) of flash
    uint8_t page_size;          // size (in bytes) of page
    uint8_t write_sleep;        // milliseconds
    uint8_t signature1;         // only used in protocol v2
    uint8_t signature2;         // only used in protocol v2
    // Calculated bootloader info
    uint16_t pages;             // total number of pages to program
    uint16_t bootloader_start;  // start of the bootloader (at page boundary)
    uint16_t erase_sleep;       // milliseconds
    // State
    uint16_t user_reset_vector; // reset vector of user program
    bool write_last_page;       // last page already programmed
    bool start_program;         // require start after flash
} pdata_t;

//-----------------------------------------------------------------------------

static void delay_ms(uint32_t duration)
{
    usleep(duration * 1000);
}

static int micronucleus_check_connection(pdata_t* pdata)
{
    if (pdata->major_version >= 2)
    {
        uint8_t buffer[6] = { 0 };
        int result = usb_control_msg(
            pdata->usb_handle,
            USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
            MICRONUCLEUS_CMD_INFO,
            0, 0,
            (char*)buffer, sizeof(buffer),
            MICRONUCLEUS_DEFAULT_TIMEOUT);
        return result == sizeof(buffer) ? 0 : -1;
    }
    else
    {
        uint8_t buffer[4] = { 0 };
        int result = usb_control_msg(
            pdata->usb_handle,
            USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
            MICRONUCLEUS_CMD_INFO,
            0, 0,
            (char*)buffer, sizeof(buffer),
            MICRONUCLEUS_DEFAULT_TIMEOUT);
        return result == sizeof(buffer) ? 0 : -1;
    }
}

static bool micronucleus_is_device_responsive(pdata_t* pdata, struct usb_device* device)
{
    pdata->usb_handle = usb_open(device);
    if (pdata->usb_handle == NULL)
    {
        return false;
    }

    int result = micronucleus_check_connection(pdata);

    usb_close(pdata->usb_handle);
    pdata->usb_handle = NULL;

    return result >= 0;
}

static int micronucleus_reconnect(pdata_t* pdata)
{
    struct usb_device* device = usb_device(pdata->usb_handle);

    usb_close(pdata->usb_handle);
    pdata->usb_handle = NULL;

    for (int i = 0; i < 25; i++)
    {
        pmsg_notice("trying to reconnect ...\n");

        pdata->usb_handle = usb_open(device);
        if (pdata->usb_handle != NULL)
            return 0;

        delay_ms(MICRONUCLEUS_CONNECT_WAIT);
    }

    return -1;
}

static int micronucleus_get_bootloader_info_v1(pdata_t* pdata)
{
    uint8_t buffer[4] = { 0 };
    int result = usb_control_msg(
        pdata->usb_handle,
        USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
        MICRONUCLEUS_CMD_INFO,
        0, 0,
        (char*)buffer, sizeof(buffer),
        MICRONUCLEUS_DEFAULT_TIMEOUT);
    if (result < 0)
    {
        pmsg_warning("unable to get bootloader info block: %s\n", usb_strerror());
        return result;
    }
    else if ((size_t) result < sizeof(buffer))
    {
        pmsg_warning("received invalid bootloader info block size: %d\n", result);
        return -1;
    }

    pdata->flash_size = (buffer[0] << 8) | buffer[1];
    pdata->page_size = buffer[2];
    pdata->write_sleep = buffer[3] & 127;

    // Take a wild guess on the part ID, so that we can supply it for device verification
    if (pdata->page_size == 128)
    {
        // ATtiny167
        pdata->signature1 = 0x94;
        pdata->signature2 = 0x87;
    }
    else if (pdata->page_size == 64)
    {
        if (pdata->flash_size > 4096)
        {
            // ATtiny85
            pdata->signature1 = 0x93;
            pdata->signature2 = 0x0B;
        }
        else
        {
            // ATtiny45
            pdata->signature1 = 0x92;
            pdata->signature2 = 0x06;
        }
    }
    else if (pdata->page_size == 16)
    {
        // ATtiny841
        pdata->signature1 = 0x93;
        pdata->signature2 = 0x15;
    }
    else
    {
        // Unknown device
        pdata->signature1 = 0;
        pdata->signature2 = 0;
    }

    pdata->pages = (pdata->flash_size + pdata->page_size - 1) / pdata->page_size;
    pdata->bootloader_start = pdata->pages * pdata->page_size;
    pdata->erase_sleep = pdata->write_sleep * pdata->pages;

    return 0;
}

static int micronucleus_get_bootloader_info_v2(pdata_t* pdata)
{
    uint8_t buffer[6] = { 0 };
    int result = usb_control_msg(
        pdata->usb_handle,
        USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
        MICRONUCLEUS_CMD_INFO,
        0, 0,
        (char*)buffer, sizeof(buffer),
        MICRONUCLEUS_DEFAULT_TIMEOUT);
    if (result < 0)
    {
        pmsg_warning("unable to get bootloader info block: %s\n", usb_strerror());
        return result;
    }
    else if ((size_t) result < sizeof(buffer))
    {
        pmsg_warning("received invalid bootloader info block size: %d\n", result);
        return -1;
    }

    pdata->flash_size = (buffer[0] << 8) + buffer[1];
    pdata->page_size = buffer[2];
    pdata->write_sleep = (buffer[3] & 127) + 2;
    pdata->signature1 = buffer[4];
    pdata->signature2 = buffer[5];

    pdata->pages = (pdata->flash_size + pdata->page_size - 1) / pdata->page_size;
    pdata->bootloader_start = pdata->pages * pdata->page_size;
    pdata->erase_sleep = pdata->write_sleep * pdata->pages;

    // if bit 7 of write sleep time is set, divide the erase time by four to
    // accomodate to the 4*page erase of the ATtiny841/441
    if ((buffer[3] & 128) != 0)
    {
        pdata->erase_sleep /= 4;
    }

    return 0;
}

static int micronucleus_get_bootloader_info(pdata_t* pdata)
{
    if (pdata->major_version >= 2)
    {
        return micronucleus_get_bootloader_info_v2(pdata);
    }
    else
    {
        return micronucleus_get_bootloader_info_v1(pdata);
    }
}

static void micronucleus_dump_device_info(pdata_t* pdata)
{
    pmsg_notice("Bootloader version: %d.%d\n", pdata->major_version, pdata->minor_version);
    imsg_notice("Available flash size: %u\n", pdata->flash_size);
    imsg_notice("Page size: %u\n", pdata->page_size);
    imsg_notice("Bootloader start: 0x%04X\n", pdata->bootloader_start);
    imsg_notice("Write sleep: %ums\n", pdata->write_sleep);
    imsg_notice("Erase sleep: %ums\n", pdata->erase_sleep);
    imsg_notice("Signature1: 0x%02X\n", pdata->signature1);
    imsg_notice("Signature2: 0x%02X\n", pdata->signature2);
}

static int micronucleus_erase_device(pdata_t* pdata)
{
    pmsg_debug("micronucleus_erase_device()\n");

    int result = usb_control_msg(
        pdata->usb_handle,
        USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
        MICRONUCLEUS_CMD_ERASE,
        0, 0,
        NULL, 0,
        MICRONUCLEUS_DEFAULT_TIMEOUT);
    if (result < 0)
    {
        switch (result)
        {
        case -EIO:
        case -EPIPE:
            pmsg_notice("ignoring last error of erase command: %s\n", usb_strerror());
            break;
        default:
            pmsg_warning("erase command failed, code %d: %s\n", result, usb_strerror());
            return result;
        }
    }

    delay_ms(pdata->erase_sleep);

    result = micronucleus_check_connection(pdata);
    if (result < 0)
    {
        pmsg_notice("connection dropped, trying to reconnect ...\n");

        result = micronucleus_reconnect(pdata);
        if (result < 0)
        {
            pmsg_warning("unable to reconnect USB device: %s\n", usb_strerror());
            return result;
        }
    }

    return 0;
}

static int micronucleus_patch_reset_vector(pdata_t* pdata, uint8_t* buffer)
{
    // Save user reset vector.
    uint16_t word0 = (buffer[1] << 8) | buffer[0];
    uint16_t word1 = (buffer[3] << 8) | buffer[2];

    if (word0 == 0x940C)
    {
        // long jump
        pdata->user_reset_vector = word1;
    }
    else if ((word0 & 0xF000) == 0xC000)
    {
        // rjmp
        pdata->user_reset_vector = (word0 & 0x0FFF) + 1;
    }
    else
    {
        pmsg_error("the reset vector of the user program does not contain a branch instruction\n");
        return -1;
    }

    // Patch in jmp to bootloader.
    if (pdata->bootloader_start > 0x2000)
    {
        // jmp
        uint16_t data = 0x940C;
        buffer[0] = (uint8_t)(data >> 0);
        buffer[1] = (uint8_t)(data >> 8);
        buffer[2] = (uint8_t)(pdata->bootloader_start >> 0);
        buffer[3] = (uint8_t)(pdata->bootloader_start >> 8);
    }
    else
    {
        // rjmp
        uint16_t data = 0xC000 | ((pdata->bootloader_start / 2 - 1) & 0x0FFF);
        buffer[0] = (uint8_t)(data >> 0);
        buffer[1] = (uint8_t)(data >> 8);
    }

    return 0;
}

static void micronucleus_patch_user_vector(pdata_t* pdata, uint8_t* buffer)
{
    uint16_t user_reset_addr = pdata->bootloader_start - 4;
    uint16_t address = pdata->bootloader_start - pdata->page_size;
    if (user_reset_addr > 0x2000)
    {
        //  jmp
        uint16_t data = 0x940C;
        buffer[user_reset_addr - address + 0] = (uint8_t)(data >> 0);
        buffer[user_reset_addr - address + 1] = (uint8_t)(data >> 8);
        buffer[user_reset_addr - address + 2] = (uint8_t)(pdata->user_reset_vector >> 0);
        buffer[user_reset_addr - address + 3] = (uint8_t)(pdata->user_reset_vector >> 8);
    }
    else
    {
        // rjmp
        uint16_t data = 0xC000 | ((pdata->user_reset_vector - user_reset_addr / 2 - 1) & 0x0FFF);
        buffer[user_reset_addr - address + 0] = (uint8_t)(data >> 0);
        buffer[user_reset_addr - address + 1] = (uint8_t)(data >> 8);
    }
}

static int micronucleus_write_page_v1(pdata_t* pdata, uint32_t address, uint8_t* buffer, uint32_t size)
{
    int result = usb_control_msg(
        pdata->usb_handle,
        USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
        MICRONUCLEUS_CMD_TRANSFER,
        size, address,
        (char*)buffer, size,
        MICRONUCLEUS_DEFAULT_TIMEOUT);
    if (result < 0)
    {
        pmsg_error("unable to transfer page: %s\n", usb_strerror());
        return result;
    }

    return 0;
}

static int micronucleus_write_page_v2(pdata_t* pdata, uint32_t address, uint8_t* buffer, uint32_t size)
{
    int result = usb_control_msg(
        pdata->usb_handle,
        USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
        MICRONUCLEUS_CMD_TRANSFER,
        size, address,
        NULL, 0,
        MICRONUCLEUS_DEFAULT_TIMEOUT);
    if (result < 0)
    {
        pmsg_error("unable to transfer page: %s\n", usb_strerror());
        return result;
    }

    for (uint32_t i = 0; i < size; i += 4)
    {
        int w1 = (buffer[i + 1] << 8) | (buffer[i + 0] << 0);
        int w2 = (buffer[i + 3] << 8) | (buffer[i + 2] << 0);
        result = usb_control_msg(
            pdata->usb_handle,
            USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
            MICRONUCLEUS_CMD_PROGRAM,
            w1, w2,
            NULL, 0,
            MICRONUCLEUS_DEFAULT_TIMEOUT);
        if (result < 0)
        {
            pmsg_error("unable to transfer page: %s\n", usb_strerror());
            return result;
        }
    }

    return 0;
}

static int micronucleus_write_page(pdata_t* pdata, uint32_t address, uint8_t* buffer, uint32_t size)
{
    pmsg_debug("micronucleus_write_page(address=0x%04X, size=%d)\n", address, size);

    if (address == 0)
    {
        if (pdata->major_version >= 2)
        {
            int result = micronucleus_patch_reset_vector(pdata, buffer);
            if (result < 0)
            {
                return result;
            }
        }

        // Require last page (with application reset vector) to be written.
        pdata->write_last_page = true;

        // Require software start.
        pdata->start_program = true;
    }
    else if (address >= (uint32_t) (pdata->bootloader_start - pdata->page_size))
    {
        if (pdata->major_version >= 2)
        {
            micronucleus_patch_user_vector(pdata, buffer);
        }

        // Mark last page as written.
        pdata->write_last_page = false;
    }

    int result;
    if (pdata->major_version >= 2)
    {
        result = micronucleus_write_page_v2(pdata, address, buffer, size);
    }
    else
    {
        result = micronucleus_write_page_v1(pdata, address, buffer, size);
    }

    if (result < 0)
    {
        return result;
    }

    delay_ms(pdata->write_sleep);

    return 0;
}

static int micronucleus_start(pdata_t* pdata)
{
    pmsg_debug("micronucleus_start()\n");

    int result = usb_control_msg(
        pdata->usb_handle,
        USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
        MICRONUCLEUS_CMD_START,
        0, 0,
        NULL, 0,
        MICRONUCLEUS_DEFAULT_TIMEOUT);
    if (result < 0)
    {
        pmsg_warning("start command failed: %s\n", usb_strerror());
        return result;
    }

    return 0;
}

//-----------------------------------------------------------------------------

static void micronucleus_setup(PROGRAMMER* pgm)
{
    pmsg_debug("micronucleus_setup()\n");

    if ((pgm->cookie = malloc(sizeof(pdata_t))) == 0)
    {
        pmsg_error("out of memory allocating private data\n");
        exit(1);
    }

    memset(pgm->cookie, 0, sizeof(pdata_t));
}

static void micronucleus_teardown(PROGRAMMER* pgm)
{
    pmsg_debug("micronucleus_teardown()\n");
    free(pgm->cookie);
}

static int micronucleus_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
    pmsg_debug("micronucleus_initialize()\n");

    pdata_t* pdata = PDATA(pgm);

    int result = micronucleus_get_bootloader_info(pdata);
    if (result < 0)
        return result;

    micronucleus_dump_device_info(pdata);

    return 0;
}

static void micronucleus_display(const PROGRAMMER *pgm, const char *prefix) {
    pmsg_debug("micronucleus_display()\n");
}

static void micronucleus_powerup(const PROGRAMMER *pgm) {
    pmsg_debug("micronucleus_powerup()\n");
}

static void micronucleus_powerdown(const PROGRAMMER *pgm) {
    pmsg_debug("micronucleus_powerdown()\n");

    pdata_t* pdata = PDATA(pgm);
    if (pdata->write_last_page)
    {
        pdata->write_last_page = false;

        uint8_t* buffer = (unsigned char*)malloc(pdata->page_size);
        if (buffer != NULL)
        {
            memset(buffer, 0xFF, pdata->page_size);
            micronucleus_write_page(pdata, pdata->bootloader_start - pdata->page_size, buffer, pdata->page_size);
            free(buffer);
        }
    }

    if (pdata->start_program)
    {
        pdata->start_program = false;

        micronucleus_start(pdata);
    }
}

static void micronucleus_enable(PROGRAMMER *pgm, const AVRPART *p) {
    pmsg_debug("micronucleus_enable()\n");
}

static void micronucleus_disable(const PROGRAMMER *pgm) {
    pmsg_debug("micronucleus_disable()\n");
}

static int micronucleus_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
    pmsg_debug("micronucleus_program_enable()\n");
    return 0;
}

static int micronucleus_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem) {
    pmsg_debug("micronucleus_read_sig_bytes()\n");

    if (mem->size < 3)
    {
        pmsg_error("memory size %d < 3 too small for read_sig_bytes", mem->size);
        return -1;
    }

    pdata_t* pdata = PDATA(pgm);
    mem->buf[0] = 0x1E;
    mem->buf[1] = pdata->signature1;
    mem->buf[2] = pdata->signature2;
    return 0;
}

static int micronucleus_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
    pmsg_debug("micronucleus_chip_erase()\n");

    pdata_t* pdata = PDATA(pgm);
    return micronucleus_erase_device(pdata);
}

static int micronucleus_open(PROGRAMMER* pgm, const char *port) {
    pmsg_debug("micronucleus_open(\"%s\")\n", port);

    pdata_t* pdata = PDATA(pgm);
    const char *bus_name = NULL;
    char* dev_name = NULL;

    // if no -P was given or '-P usb' was given
    if (str_eq(port, "usb"))
    {
        port = NULL;
    }
    else
    {
        // calculate bus and device names from -P option
        if (str_starts(port, "usb") && ':' == port[3])
        {
            bus_name = port + 4;
            dev_name = strchr(bus_name, ':');
            if (dev_name != NULL)
            {
                *dev_name = '\0';
                dev_name++;
            }
        }
    }

    if (port != NULL && dev_name == NULL)
    {
        pmsg_error("invalid -P value %s\n", port);
        imsg_error("use -P usb:bus:device\n");
        return -1;
    }

    // Determine VID/PID
    int vid = pgm->usbvid ? pgm->usbvid : MICRONUCLEUS_VID;
    int pid = MICRONUCLEUS_PID;

    LNODEID usbpid = lfirst(pgm->usbpid);
    if (usbpid != NULL)
    {
        pid = *(int*)(ldata(usbpid));
        if (lnext(usbpid))
        {
            pmsg_warning("using PID 0x%04x, ignoring remaining PIDs in list\n", pid);
        }
    }

    usb_init();

    bool show_retry_message = true;
    bool show_unresponsive_device_message = true;

    time_t start_time = time(NULL);
    for (;;)
    {
        usb_find_busses();
        usb_find_devices();

        pdata->usb_handle = NULL;

        // Search for device
        struct usb_bus* bus = NULL;
        for (bus = usb_busses; bus != NULL && pdata->usb_handle == NULL; bus = bus->next)
        {
            struct usb_device* device = NULL;
            for (device = bus->devices; device != NULL && pdata->usb_handle == NULL; device = device->next)
            {
                if (device->descriptor.idVendor == vid && device->descriptor.idProduct == pid)
                {
                    pdata->major_version = (uint8_t)(device->descriptor.bcdDevice >> 8);
                    pdata->minor_version = (uint8_t)(device->descriptor.bcdDevice >> 0);

                    if (!micronucleus_is_device_responsive(pdata, device))
                    {
                        if (show_unresponsive_device_message)
                        {
                            pmsg_warning("unresponsive Micronucleus device detected, please reconnect ...\n");

                            show_unresponsive_device_message = false;
                        }

                        continue;
                    }

                    pmsg_notice("found device with Micronucleus V%d.%d, bus:device: %s:%s\n",
                        pdata->major_version, pdata->minor_version,
                        bus->dirname, device->filename);

                    // if -P was given, match device by device name and bus name
                    if (port != NULL)
                    {
                        if (dev_name == NULL || !str_eq(bus->dirname, bus_name) || !str_eq(device->filename, dev_name))
                        {
                            continue;
                        }
                    }

                    if (pdata->major_version > MICRONUCLEUS_MAX_MAJOR_VERSION)
                    {
                        pmsg_warning("device with unsupported Micronucleus version V%d.%d\n",
                            pdata->major_version, pdata->minor_version);
                        continue;
                    }

                    pdata->usb_handle = usb_open(device);
                    if (pdata->usb_handle == NULL)
                    {
                        pmsg_error("unable to open USB device: %s\n", usb_strerror());
                    }
                }
            }
        }

        if (pdata->usb_handle == NULL && pdata->wait_until_device_present)
        {
            if (show_retry_message)
            {
                if (pdata->wait_timout < 0)
                {
                    pmsg_error("no device found, waiting for device to be plugged in ...\n");
                }
                else
                {
                    pmsg_error("no device found, waiting %d seconds for device to be plugged in ...\n",
                        pdata->wait_timout);
                }

                pmsg_error("press CTRL-C to terminate\n");
                show_retry_message = false;
            }

            if (pdata->wait_timout < 0 || (time(NULL) - start_time) < pdata->wait_timout)
            {
                delay_ms(MICRONUCLEUS_CONNECT_WAIT);
                continue;
            }
        }

        break;
    }

    if (!pdata->usb_handle)
    {
        pmsg_error("cannot find device with Micronucleus bootloader (%04X:%04X)\n", vid, pid);
        return -1;
    }

    return 0;
}

static void micronucleus_close(PROGRAMMER* pgm)
{
    pmsg_debug("micronucleus_close()\n");

    pdata_t* pdata = PDATA(pgm);
    if (pdata->usb_handle != NULL)
    {
        usb_close(pdata->usb_handle);
        pdata->usb_handle = NULL;
    }
}

static int micronucleus_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned long addr, unsigned char* value)
{
    pmsg_debug("micronucleus_read_byte(desc=%s, addr=0x%04lX)\n", mem->desc, addr);

    if (mem_is_a_fuse(mem) || mem_is_lock(mem))
    {
        *value = 0xFF;
        return 0;
    }
    else
    {
        pmsg_notice("reading not supported for %s memory\n", mem->desc);
        return -1;
    }
}

static int micronucleus_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned long addr, unsigned char value)
{
    pmsg_debug("micronucleus_write_byte(desc=%s, addr=0x%04lX)\n", mem->desc, addr);
    return -1;
}

static int micronucleus_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned int page_size,
    unsigned int addr, unsigned int n_bytes)
{
    pmsg_debug("micronucleus_paged_load(page_size=0x%X, addr=0x%X, n_bytes=0x%X)\n", page_size, addr, n_bytes);
    return -1;
}

static int micronucleus_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned int page_size,
    unsigned int addr, unsigned int n_bytes)
{
    pmsg_debug("micronucleus_paged_write(page_size=0x%X, addr=0x%X, n_bytes=0x%X)\n", page_size, addr, n_bytes);

    if (mem_is_flash(mem))
    {
        pdata_t* pdata = PDATA(pgm);

        if (n_bytes > page_size)
        {
            pmsg_error("buffer size %u exceeds page size %u\n", n_bytes, page_size);
            return -1;
        }

        if (addr + n_bytes > pdata->flash_size)
        {
            pmsg_error("program size %u exceeds flash size %u\n", addr + n_bytes, pdata->flash_size);
            return -1;
        }

        uint8_t* page_buffer = (uint8_t*)malloc(pdata->page_size);
        if (page_buffer == NULL)
        {
            pmsg_error("unable to allocate memory\n");
            return -1;
        }

        // Note: Page size reported by the bootloader may be smaller than device page size as configured in avrdude.conf.
        int result = 0;
        while (n_bytes > 0)
        {
            size_t chunk_size = n_bytes < pdata->page_size ? n_bytes : pdata->page_size;

            memcpy(page_buffer, mem->buf + addr, chunk_size);
            memset(page_buffer + chunk_size, 0xFF, pdata->page_size - chunk_size);

            result = micronucleus_write_page(pdata, addr, page_buffer, pdata->page_size);
            if (result < 0)
            {
                break;
            }

            addr += chunk_size;
            n_bytes -= chunk_size;
        }

        free(page_buffer);
        return result;
    }
    else
    {
        pmsg_error("unsupported memory %s\n", mem->desc);
        return -1;
    }
}

static int micronucleus_parseextparams(const PROGRAMMER *pgm, const LISTID xparams) {
    pmsg_debug("micronucleus_parseextparams()\n");

    pdata_t* pdata = PDATA(pgm);
    for (LNODEID node = lfirst(xparams); node != NULL; node = lnext(node))
    {
        const char* param = ldata(node);

        if (str_eq(param, "wait"))
        {
            pdata->wait_until_device_present = true;
            pdata->wait_timout = -1;
        }
        else if (str_starts(param, "wait="))
        {
            pdata->wait_until_device_present = true;
            pdata->wait_timout = atoi(param + 5);
        }
        else if (str_eq(param, "help"))
        {
            msg_error("%s -c %s extended options:\n", progname, pgmid);
            msg_error("  -xwait       Wait for the device to be plugged in if not connected\n");
            msg_error("  -xwait=<arg> Wait <arg> [s] for the device to be plugged in if not connected\n");
            msg_error("  -xhelp       Show this help menu and exit\n");
            exit(0);
        }
        else
        {
            pmsg_error("invalid extended parameter '%s'\n", param);
            return -1;
        }
    }

    return 0;
}

void micronucleus_initpgm(PROGRAMMER *pgm) {
    strcpy(pgm->type, "Micronucleus V2.0");

    pgm->setup = micronucleus_setup;
    pgm->teardown = micronucleus_teardown;
    pgm->initialize = micronucleus_initialize;
    pgm->display = micronucleus_display;
    pgm->powerup = micronucleus_powerup;
    pgm->powerdown = micronucleus_powerdown;
    pgm->enable = micronucleus_enable;
    pgm->disable = micronucleus_disable;
    pgm->program_enable = micronucleus_program_enable;
    pgm->read_sig_bytes = micronucleus_read_sig_bytes;
    pgm->chip_erase = micronucleus_chip_erase;
    pgm->cmd = NULL;
    pgm->open = micronucleus_open;
    pgm->close = micronucleus_close;
    pgm->read_byte = micronucleus_read_byte;
    pgm->write_byte = micronucleus_write_byte;
    pgm->paged_load = micronucleus_paged_load;
    pgm->paged_write = micronucleus_paged_write;
    pgm->parseextparams = micronucleus_parseextparams;
}

#else /* !HAVE_LIBUSB */

 // Give a proper error if we were not compiled with libusb
static int micronucleus_nousb_open(PROGRAMMER* pgm, const char* name) {
    pmsg_error("no usb support; please compile again with libusb installed\n");
    return -1;
}

void micronucleus_initpgm(PROGRAMMER *pgm) {
    strcpy(pgm->type, "micronucleus");
    pgm->open = micronucleus_nousb_open;
}

#endif /* HAVE_LIBUSB */

const char micronucleus_desc[] = "Micronucleus Bootloader";
