/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2012 Joerg Wunsch <j@uriah.heep.sax.de>
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

#ifndef jtag3_h
#define jtag3_h

#ifdef __cplusplus
extern "C" {
#endif

int  jtag3_open_common(PROGRAMMER *pgm, const char *port, int mode_switch);
int  jtag3_send(const PROGRAMMER *pgm, unsigned char *data, size_t len);
int  jtag3_recv(const PROGRAMMER *pgm, unsigned char **msg);
void jtag3_close(PROGRAMMER * pgm);
int  jtag3_getsync(const PROGRAMMER *pgm, int mode);
int  jtag3_getparm(const PROGRAMMER *pgm, unsigned char scope,
                   unsigned char section, unsigned char parm,
                   unsigned char *value, unsigned char length);
int jtag3_setparm(const PROGRAMMER *pgm, unsigned char scope,
                  unsigned char section, unsigned char parm,
                  unsigned char *value, unsigned char length);
int jtag3_command(const PROGRAMMER *pgm, unsigned char *cmd, unsigned int cmdlen,
                  unsigned char **resp, const char *descr);
void jtag3_display(const PROGRAMMER *pgm, const char *p);
void jtag3_print_parms1(const PROGRAMMER *pgm, const char *p, FILE *fp);
int jtag3_set_vtarget(const PROGRAMMER *pgm, double voltage);
int jtag3_get_vtarget(const PROGRAMMER *pgm, double *voltage);
extern const char jtag3_desc[];
extern const char jtag3_dw_desc[];
extern const char jtag3_pdi_desc[];
extern const char jtag3_updi_desc[];
extern const char jtag3_tpi_desc[];
void jtag3_initpgm(PROGRAMMER *pgm);
void jtag3_dw_initpgm(PROGRAMMER *pgm);
void jtag3_pdi_initpgm(PROGRAMMER *pgm);
void jtag3_updi_initpgm(PROGRAMMER *pgm);
void jtag3_tpi_initpgm(PROGRAMMER *pgm);

/*
 * These functions are referenced from stk500v2.c for JTAGICE3 in
 * one of the STK500v2 modi.
 */
void jtag3_setup(PROGRAMMER * pgm);
void jtag3_teardown(PROGRAMMER * pgm);

#ifdef __cplusplus
}
#endif

#endif

