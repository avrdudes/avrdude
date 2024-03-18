/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2020 Marius Greuel
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
// This file adds support for the HalfKay bootloader,
// so you do no longer need the Teensy loader utility.
//
// This HalfKay bootloader is used on various PJRC Teensy boards,
// such as Teensy 2.0 (ATmega32U4), Teensy++ 2.0 (AT90USB1286),
// and the respective clones.
// By default, it bootloader uses the VID/PID 16C0:0478 (VOTI).
//
// As the Teensy bootloader is optimized for size, it implements
// writing to flash memory only. Since it does not support reading,
// use the -V option to prevent avrdude from verifing the flash memory.
// To have avrdude wait for the device to be connected, use the
// extended option '-x wait'.
//
// Example:
// avrdude -c teensy -p m32u4 -x wait -V -U flash:w:main.hex:i

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
#include "teensy.h"
#include "usbdevs.h"

#if defined(HAVE_LIBHIDAPI)

#include <hidapi/hidapi.h>

//-----------------------------------------------------------------------------

#define TEENSY_VID 0x16C0
#define TEENSY_PID 0x0478

#define TEENSY_CONNECT_WAIT 100

#define PDATA(pgm) ((pdata_t*)(pgm->cookie))

//-----------------------------------------------------------------------------

typedef struct pdata
{
    hid_device* hid_handle;
    uint16_t hid_usage;
    // Extended parameters
    bool wait_until_device_present;
    int wait_timout;            // in seconds
    // Bootloader info (from hid_usage)
    const char* board;
    uint32_t flash_size;
    uint16_t page_size;
    uint8_t sig_bytes[3];
    // State
    bool erase_flash;
    bool reboot;
} pdata_t;

//-----------------------------------------------------------------------------

static void delay_ms(uint32_t duration)
{
    usleep(duration * 1000);
}

static int teensy_get_bootloader_info(pdata_t* pdata, const AVRPART* p) {
    switch (pdata->hid_usage)
    {
    case 0x19:
        pdata->board = "Teensy 1.0 (AT90USB162)";
        pdata->flash_size = 0x4000 - 0x200;
        pdata->page_size = 128;
        pdata->sig_bytes[0] = 0x1E;
        pdata->sig_bytes[1] = 0x94;
        pdata->sig_bytes[2] = 0x82;
        break;
    case 0x1A:
        pdata->board = "Teensy++ 1.0 (AT90USB646)";
        pdata->flash_size = 0x10000 - 0x400;
        pdata->page_size = 256;
        pdata->sig_bytes[0] = 0x1E;
        pdata->sig_bytes[1] = 0x96;
        pdata->sig_bytes[2] = 0x82;
        break;
    case 0x1B:
        pdata->board = "Teensy 2.0 (ATmega32U4)";
        pdata->flash_size = 0x8000 - 0x200;
        pdata->page_size = 128;
        pdata->sig_bytes[0] = 0x1E;
        pdata->sig_bytes[1] = 0x95;
        pdata->sig_bytes[2] = 0x87;
        break;
    case 0x1C:
        pdata->board = "Teensy++ 2.0 (AT90USB1286)";
        pdata->flash_size = 0x20000 - 0x400;
        pdata->page_size = 256;
        pdata->sig_bytes[0] = 0x1E;
        pdata->sig_bytes[1] = 0x97;
        pdata->sig_bytes[2] = 0x82;
        break;
    default:
        if (pdata->hid_usage == 0)
        {
            // On Linux, libhidapi does not seem to return the HID usage from the report descriptor.
            // We try to infer the board from the part information, until somebody fixes libhidapi.
            // To use this workaround, the -F option is required.
            pmsg_error("cannot detect board type (HID usage is 0)\n");

            AVRMEM* mem = avr_locate_flash(p);
            if (mem == NULL)
            {
                pmsg_error("no flash memory defined for part %s\n", p->desc);
                return -1;
            }

            pdata->board = "Unknown Board";
            pdata->flash_size = mem->size - (mem->size < 0x10000 ? 0x200 : 0x400);
            pdata->page_size = mem->page_size;

            // Pass an invalid signature to require -F option.
            pdata->sig_bytes[0] = 0x1E;
            pdata->sig_bytes[1] = 0x00;
            pdata->sig_bytes[2] = 0x00;
        }
        else
        {
            pmsg_error("Teensy board not supported (HID usage 0x%02X)\n", pdata->hid_usage);
            return -1;
        }
    }

    return 0;
}

static void teensy_dump_device_info(pdata_t* pdata)
{
    pmsg_notice("HID usage: 0x%02X\n", pdata->hid_usage);
    pmsg_notice("Board: %s\n", pdata->board);
    pmsg_notice("Available flash size: %u\n", pdata->flash_size);
    pmsg_notice("Page size: %u\n", pdata->page_size);
    pmsg_notice("Signature: 0x%02X%02X%02X\n",
      pdata->sig_bytes[0], pdata->sig_bytes[1], pdata->sig_bytes[2]);
}

static int teensy_write_page(pdata_t* pdata, uint32_t address, const uint8_t* buffer, uint32_t size, bool suppress_warning)
{
    pmsg_debug("teensy_write_page(address=0x%06X, size=%d)\n", address, size);

    if (size > pdata->page_size)
    {
        pmsg_error("invalid page size: %u\n", pdata->page_size);
        return -1;
    }

    size_t report_size = 1 + 2 + (size_t)pdata->page_size;
    uint8_t* report = (uint8_t*)malloc(report_size);
    if (report == NULL)
    {
        pmsg_error("unable to allocate memory\n");
        return -1;
    }

    report[0] = 0; // report number
    if (pdata->page_size <= 256 && pdata->flash_size < 0x10000)
    {
        report[1] = (uint8_t)(address >> 0);
        report[2] = (uint8_t)(address >> 8);
    }
    else
    {
        report[1] = (uint8_t)(address >> 8);
        report[2] = (uint8_t)(address >> 16);
    }

    if (size > 0)
    {
        memcpy(report + 1 + 2, buffer, size);
    }

    memset(report + 1 + 2 + size, 0xFF, report_size - (1 + 2 + size));

    int result = hid_write(pdata->hid_handle, report, report_size);
    free(report);
    if (result < 0)
    {
        if (!suppress_warning)
            pmsg_error("unable to write page: %ls\n", hid_error(pdata->hid_handle));

        return result;
    }

    return 0;
}

static int teensy_erase_flash(pdata_t* pdata)
{
    pmsg_debug("teensy_erase_flash()\n");

    // Write a dummy page at address 0 to explicitly erase the flash.
    return teensy_write_page(pdata, 0, NULL, 0, false);
}

static int teensy_reboot(pdata_t* pdata)
{
    pmsg_debug("teensy_reboot()\n");

    // Write a dummy page at address -1 to reboot the Teensy.
    return teensy_write_page(pdata, 0xFFFFFFFF, NULL, 0, true);
}

//-----------------------------------------------------------------------------

static void teensy_setup(PROGRAMMER* pgm)
{
    pmsg_debug("teensy_setup()\n");

    if ((pgm->cookie = malloc(sizeof(pdata_t))) == NULL)
    {
        pmsg_error("unable to allocate memory\n");
        exit(1);
    }

    memset(pgm->cookie, 0, sizeof(pdata_t));
}

static void teensy_teardown(PROGRAMMER* pgm)
{
    pmsg_debug("teensy_teardown()\n");
    free(pgm->cookie);
}

static int teensy_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
    pmsg_debug("teensy_initialize()\n");

    pdata_t* pdata = PDATA(pgm);

    int result = teensy_get_bootloader_info(pdata, p);
    if (result < 0)
        return result;

    teensy_dump_device_info(pdata);

    return 0;
}

static void teensy_display(const PROGRAMMER *pgm, const char *prefix) {
    pmsg_debug("teensy_display()\n");
}

static void teensy_powerup(const PROGRAMMER *pgm) {
    pmsg_debug("teensy_powerup()\n");
}

static void teensy_powerdown(const PROGRAMMER *pgm) {
    pmsg_debug("teensy_powerdown()\n");

    pdata_t* pdata = PDATA(pgm);

    if (pdata->erase_flash)
    {
        teensy_erase_flash(pdata);
        pdata->erase_flash = false;
    }

    if (pdata->reboot)
    {
        teensy_reboot(pdata);
        pdata->reboot = false;
    }
}

static void teensy_enable(PROGRAMMER* pgm, const AVRPART *p) {
    pmsg_debug("teensy_enable()\n");
}

static void teensy_disable(const PROGRAMMER *pgm) {
    pmsg_debug("teensy_disable()\n");
}

static int teensy_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
    pmsg_debug("teensy_program_enable()\n");
    return 0;
}

static int teensy_read_sig_bytes(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem) {
    pmsg_debug("teensy_read_sig_bytes()\n");

    if (mem->size < 3)
    {
        pmsg_error("memory size too small for read_sig_bytes\n");
        return -1;
    }

    pdata_t* pdata = PDATA(pgm);
    memcpy(mem->buf, pdata->sig_bytes, sizeof(pdata->sig_bytes));

    return 0;
}

static int teensy_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
    pmsg_debug("teensy_chip_erase()\n");

    pdata_t* pdata = PDATA(pgm);

    // Schedule a chip erase, either at first write or on powerdown.
    pdata->erase_flash = true;

    return 0;
}

static int teensy_open(PROGRAMMER *pgm, const char *port) {
    pmsg_debug("teensy_open(\"%s\")\n", port);

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
        pmsg_error("invalid -P value: '%s'\n", port);
        imsg_error("Use -P usb:bus:device\n");
        return -1;
    }

    // Determine VID/PID
    int vid = pgm->usbvid ? pgm->usbvid : TEENSY_VID;
    int pid = TEENSY_PID;

    LNODEID usbpid = lfirst(pgm->usbpid);
    if (usbpid != NULL)
    {
        pid = *(int*)(ldata(usbpid));
        if (lnext(usbpid))
        {
            pmsg_error("using PID 0x%04x, ignoring remaining PIDs in list\n", pid);
        }
    }

    bool show_retry_message = true;

    time_t start_time = time(NULL);
    for (;;)
    {
        // Search for device
        struct hid_device_info* devices = hid_enumerate(vid, pid);
        struct hid_device_info* device = devices;

        while (device)
        {
            if (device->vendor_id == vid && device->product_id == pid)
            {
                pdata->hid_handle = hid_open_path(device->path);
                if (pdata->hid_handle == NULL)
                {
                    pmsg_error("found HID device, but hid_open_path() failed\n");
                }
                else
                {
                    pdata->hid_usage = device->usage;
                    break;
                }
            }

            device = device->next;
        }

        hid_free_enumeration(devices);

        if (pdata->hid_handle == NULL && pdata->wait_until_device_present)
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
                delay_ms(TEENSY_CONNECT_WAIT);
                continue;
            }
        }

        break;
    }

    if (!pdata->hid_handle)
    {
        pmsg_error("cannot find device with Teensy bootloader (%04X:%04X)\n", vid, pid);
        return -1;
    }

    return 0;
}

static void teensy_close(PROGRAMMER* pgm)
{
    pmsg_debug("teensy_close()\n");

    pdata_t* pdata = PDATA(pgm);
    if (pdata->hid_handle != NULL)
    {
        hid_close(pdata->hid_handle);
        pdata->hid_handle = NULL;
    }
}

static int teensy_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned long addr, unsigned char* value)
{
    pmsg_debug("teensy_read_byte(desc=%s, addr=0x%04lX)\n", mem->desc, addr);

    if (mem_is_a_fuse(mem) || mem_is_lock(mem))
    {
        *value = 0xFF;
        return 0;
    }
    else
    {
        pmsg_error("unsupported memory %s\n", mem->desc);
        return -1;
    }
}

static int teensy_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned long addr, unsigned char value)
{
    pmsg_debug("teensy_write_byte(desc=%s, addr=0x%04lX)\n", mem->desc, addr);
    return -1;
}

static int teensy_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned int page_size,
    unsigned int addr, unsigned int n_bytes)
{
    pmsg_debug("teensy_paged_load(page_size=0x%X, addr=0x%X, n_bytes=0x%X)\n", page_size, addr, n_bytes);
    return -1;
}

static int teensy_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
    unsigned int page_size,
    unsigned int addr, unsigned int n_bytes)
{
    pmsg_debug("teensy_paged_write(page_size=0x%X, addr=0x%X, n_bytes=0x%X)\n", page_size, addr, n_bytes);

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

        if (pdata->erase_flash)
        {
            // Writing page 0 will automatically erase the flash.
            // If mem does not contain a page at address 0, write a dummy page at address 0.
            if (addr != 0)
            {
                int result = teensy_erase_flash(pdata);
                if (result < 0)
                {
                    return result;
                }
            }

            pdata->erase_flash = false;
        }

        int result = teensy_write_page(pdata, addr, mem->buf + addr, n_bytes, false);
        if (result < 0)
        {
            return result;
        }

        // Schedule a reboot.
        pdata->reboot = true;

        return result;
    }
    else
    {
        pmsg_error("unsupported memory %s\n", mem->desc);
        return -1;
    }
}

static int teensy_parseextparams(const PROGRAMMER *pgm, const LISTID xparams) {
    pmsg_debug("teensy_parseextparams()\n");

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

void teensy_initpgm(PROGRAMMER *pgm) {
    strcpy(pgm->type, "teensy");

    pgm->setup = teensy_setup;
    pgm->teardown = teensy_teardown;
    pgm->initialize = teensy_initialize;
    pgm->display = teensy_display;
    pgm->powerup = teensy_powerup;
    pgm->powerdown = teensy_powerdown;
    pgm->enable = teensy_enable;
    pgm->disable = teensy_disable;
    pgm->program_enable = teensy_program_enable;
    pgm->read_sig_bytes = teensy_read_sig_bytes;
    pgm->chip_erase = teensy_chip_erase;
    pgm->cmd = NULL;
    pgm->open = teensy_open;
    pgm->close = teensy_close;
    pgm->read_byte = teensy_read_byte;
    pgm->write_byte = teensy_write_byte;
    pgm->paged_load = teensy_paged_load;
    pgm->paged_write = teensy_paged_write;
    pgm->parseextparams = teensy_parseextparams;
}

#else /* !HAVE_LIBHIDAPI */

 // Give a proper error if we were not compiled with libhidapi
static int teensy_nousb_open(PROGRAMMER *pgm, const char *name) {
    pmsg_error("no HID support; please compile again with libhidapi installed\n");
    return -1;
}

void teensy_initpgm(PROGRAMMER *pgm) {
    strcpy(pgm->type, "teensy");
    pgm->open = teensy_nousb_open;
}

#endif /* HAVE_LIBHIDAPI */

const char teensy_desc[] = "Teensy Bootloader";
