#!/usr/bin/env python
#
# Simple test program for testing the SWIG interface.
#
# Best call this as "python -i src/swigtest.py",
# then run the interactive interpreter.

# Example:
# a=search_avrpart('m328')
# print(a)
# show_avrmem(a['mem'])

import sys
import os
import pathlib

builddir = None
if os.name == 'posix':
    # Linux, *BSD, MacOS
    sysname = os.uname()[0].lower()
    builddir = f'build_{sysname}/src'
elif os.name == 'nt':
    # Windows
    for candidate in ['build_msvc/src', 'build_msys64/src']:
        if os.path.exists(candidate):
            builddir = candidate
            break

if builddir == None:
    print("Cannot determine build directory, module loading might fail.", file=sys.stderr)
else:
    sys.path.append(builddir)

import swig_avrdude as ad

ad.init_config()

found = False
for d in [builddir, "/etc", "/usr/local/etc"]:
    p = pathlib.Path(d + "/avrdude.conf")
    if p.is_file():
        print(f"Found avrdude.conf in {d}")
        ad.read_config(d + "/avrdude.conf")
        found = True
        break

if not found:
    print("Sorry, no avrdude.conf could be found.")
    sys.exit(1)

def search_avrpart(name):
    '''Search list of AVR parts from config for matching "desc" or "id"'''

    partlist = ad.cvar.part_list

    part = ad.lfirst(partlist)
    while part:
        d = ad.ldata(part)
        avrpart = ad.cast_avrpart(d)
        if avrpart['desc'] == name or avrpart['id'] == name:
            return avrpart
        part = ad.lnext(part)

    return None

def show_avrmem(mem):
    '''List all memories defined at avrpart["mem"]'''

    m = ad.lfirst(mem)
    while m:
        d = ad.ldata(m)
        print(ad.cast_avrmem(d))
        m = ad.lnext(m)
