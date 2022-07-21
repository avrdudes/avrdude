/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2022, Stefan Rueger <smr@theblueorange.space>
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

#ifndef developer_opts_h
#define developer_opts_h

char cmdbitchar(CMDBIT cb);
char *cmdbitstr(CMDBIT cb);
const char *opcodename(int opcode);
int intlog2(unsigned int n);
int part_match(const char *pattern, const char *string);
void dev_output_part_defs(char *partdesc);

#endif
