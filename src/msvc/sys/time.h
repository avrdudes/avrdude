/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2018 Marius Greuel
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

#ifndef _TIME_H_
#define _TIME_H_

// If you need both <windows.h> and <sys/time.h>,
// make sure you include <windows.h> first.
#ifndef _WINSOCKAPI_
#ifndef _TIMEVAL_DEFINED
#define _TIMEVAL_DEFINED
struct timeval
{
    long tv_sec;
    long tv_usec;
};
#endif /* _TIMEVAL_DEFINED */
#endif /* _WINSOCKAPI_ */

struct timezone
{
    int tz_minuteswest;
    int tz_dsttime;
};

int __cdecl gettimeofday(struct timeval* p, void* z);

#endif
