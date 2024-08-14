#!/usr/bin/env python
#
# Simple test program for testing the SWIG interface.
#
# Best call this as "python -i src/swigtest.py",
# then run the interactive interpreter.

# Examples:

# getavr("m128")

# ad.set_msg_callback(msg_callback)
# ad.set_progress_callback(progress_callback)
# ad.cvar.verbose=2
# p = ad.locate_part(ad.cvar.part_list, 'm168pb')
# ad.avr_initmem(p)
# pgm = start_programmer('xplainedmini', 'usb', p)
# m = ad.avr_locate_mem(p, 'signature')
# ad.avr_read_mem(pgm, p, m)
# m.get(3)
# m.get(3) == p.signature
# stop_programmer(pgm)

# ad.fileio(ad.FIO_WRITE, "test.hex", ad.fileio_format("i"), p, "flash", -1)

# cfg=ad.get_config_table('atmega128')
# l=dissect_fuse(cfg, 'lfuse', 0xe0)
# hex(synthesize_fuse(cfg, 'lfuse', l))

# for AVR parts with aliased mem (avrX):
# ad.avr_locate_memalias(p, 'wdtcfg').aliased_mem.desc
# alist = ad.lfirst(p.mem_alias)
# ad.ldata_avrmem_alias(alist).desc
# ad.ldata_avrmem_alias(alist).aliased_mem.desc
# alist = ad.lnext(alist) # ...

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

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
    for candidate in ['build_msvc/src', 'build_mingw64/src']:
        if os.path.exists(candidate):
            builddir = candidate
            os.add_dll_directory(os.path.realpath(candidate))
            break

if builddir == None:
    print("Cannot determine build directory, module loading might fail.", file=sys.stderr)
else:
    sys.path.append(builddir)

import swig_avrdude as ad

ad.init_cx()
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

def avrpart_to_dict(avrpart):

    if str(type(avrpart)).find('AVRPART') < 0:
        raise Exception(f"wrong argument: {type(avrpart)}, expecting swig_avrdude.AVRPART")

    d = {}
    d['desc'] = avrpart.desc
    d['id'] = avrpart.id
    d['family_id'] = avrpart.family_id
    d['config_file'] = avrpart.config_file
    d['lineno'] = avrpart.lineno
    d['variants'] = avrpart.variants
    d['mem'] = avrpart.mem

    return d

def avrmem_to_dict(mem):

    if str(type(mem)).find('AVRMEM') < 0:
        raise Exception(f"wrong argument: {type(mem)}, expecting swig_avrdude.AVRMEM")

    d = {}
    d['desc'] = mem.desc
    d['size'] = mem.size
    d['paged'] = mem.paged
    d['page_size'] = mem.page_size
    d['num_pages'] = mem.num_pages

    return d

def avrpart_to_mem(avrpart):

    if str(type(avrpart)).find('AVRPART') < 0:
        raise Exception(f"wrong argument: {type(avrpart)}, expecting swig_avrdude.AVRPART")

    res = []
    m = ad.lfirst(avrpart.mem)
    while m:
        mm = ad.ldata_avrmem((m))
        res.append(avrmem_to_dict(mm))
        m = ad.lnext(m)

    return res

def getavr(name: str):

    p = ad.locate_part(ad.cvar.part_list, name)
    if not p:
        print(f"No part named {name} found")
        return
    pp = avrpart_to_dict(p)
    mm = avrpart_to_mem(p)
    print(f"AVR part {name} found as {pp['desc']}, or {pp['id']}")
    print(f"Definition in {pp['config_file']}, line {pp['lineno']}")

    print("")
    print("Memory overview:")
    print( "Name        size   paged   page_size num_pages")
    for m in mm:
        print(f"{m['desc']:11s} {m['size']:6d}  {str(m['paged']):5s}   {m['page_size']:4d}      {m['num_pages']:3d}")

    print("")
    print("Device variants:")
    v = ad.lfirst(p.variants)
    while v:
        vv = ad.ldata_string(v)
        print(vv)
        v = ad.lnext(v)
    print("")

def start_programmer(programmer: str, port: str, avrpart):
    g = ad.locate_programmer(ad.cvar.programmers, programmer)
    g.initpgm()
    g.setup()
    g.open(port)
    g.enable(p)
    g.initialize(p)

    return g

def stop_programmer(g):
    g.disable()
    g.close()
    g.teardown()

def message_type(msglvl: int):
    tnames = ('OS error', 'error', 'warning', 'info', 'notice',
              'notice2', 'debug', 'trace', 'trace2')
    msglvl -= ad.MSG_EXT_ERROR # rebase to 0
    if msglvl > len(tnames):
        return 'unknown msglvl'
    else:
        return tnames[msglvl]

# rough equivalent of avrdude_message2()
# first argument is either "stdout" or "stderr"
#
# install callback with ad.set_msg_callback(msg_callback)
def msg_callback(target: str, lno: int, fname: str, func: str,
                 msgmode: int, msglvl: int, msg: str, backslash_v: bool):
    if ad.cvar.verbose >= msglvl:
        s = ""
        if msgmode & ad.MSG2_PROGNAME:
            s += ad.cvar.progname
            if ad.cvar.verbose >= ad.MSG_NOTICE and (msgmode & ad.MSG2_FUNCTION) != 0:
                s += " " + func + "()"
            if ad.cvar.verbose >= ad.MSG_DEBUG and (msgmode & ad.MSG2_FILELINE) != 0:
                n = os.path.basename(fname)
                s += f" [{n}:{lno}]"
            if (msgmode & ad.MSG2_TYPE) != 0:
                s += " " + message_type(msglvl)
            s += ": "
        elif (msgmode & ad.MSG2_INDENT1) != 0:
            s = (len(ad.cvar.progname) + 1) * ' '
        elif (msgmode & ad.MSG2_INDENT2) != 0:
            s = (len(ad.cvar.progname) + 2) * ' '
        s += msg
        try:
            print(s, end='')
        except UnicodeDecodeError:
            print("message contained invalid characters")

# very simplified progress callback
global prog_hdr
prog_hdr = ""
def progress_callback(percent: int, etime: float, hdr: str, finish: int):
    if hdr:
        global prog_hdr
        prog_hdr = hdr
    print(f"{prog_hdr} {percent:3d} %", end='\r', file=sys.stderr, flush=True)
    if (percent == 100):
        print("", file=sys.stderr)

ad.set_msg_callback(msg_callback)
ad.set_progress_callback(progress_callback)
ad.cvar.verbose=2

def dissect_fuse(config: list, fuse: str, val: int):
    '''
    Analyze the value of a particular fuse

    config: configuration list from ad.get_config_table()
    fuse: string of fuse to handle
    val: integer value of the fuse

    Prints fuse information, and returns a list of items, useful
    for synthesize_fuse()
    '''
    result = []
    for i in config:
        # 'name', 'vlist', 'memstr', 'memoffset', 'mask', 'lsh', 'initval', 'ccomment'
        if i['memstr'] != fuse:
            continue
        name = i['name']
        vlist = i['vlist']
        thisval = val & i['mask']
        thisval = thisval >> i['lsh']
        ccmt = i['ccomment']
        print(f"bitfield {name}, value {thisval}, {ccmt}")
        for j in vlist:
            # 'value', 'label', 'vcomment'
            if j['value'] == thisval:
                lbl = j['label']
                vcmt = j['vcomment']
                print(f"    {lbl}, {vcmt}")
                result.append((name, lbl))
    return result

def synthesize_fuse(config: list, fuse: str, vallist: list) -> int:
    '''
    Synthesize the value of a particular fuse

    config: configuration list from ad.get_config_table()
    fuse: string of fuse to handle
    vallist: list of items to set (from dissect_fuse())

    Returns synthesized fuse value.
    '''
    itemlist = []
    for ele in vallist:
        itemlist.append(ele[1])
    resval = 0
    for i in config:
        # 'name', 'vlist', 'memstr', 'memoffset', 'mask', 'lsh', 'initval', 'ccomment'
        if i['memstr'] != fuse:
            continue
        vlist = i['vlist']
        shift = i['lsh']
        for j in vlist:
            # 'value', 'label', 'vcomment'
            if j['label'] in itemlist:
                thisval = j['value'] << shift
                resval |= thisval
    return resval

def default_fuse(config: list, fuse: str):
    '''
    Synthesize the default value for a fuse

    config: configuration list from ad.get_config_table()
    fuse: string of fuse to handle
    '''
    resval = 0xff
    for i in config:
        # 'name', 'vlist', 'memstr', 'memoffset', 'mask', 'lsh', 'initval', 'ccomment'
        if i['memstr'] != fuse:
            continue
        shift = i['lsh']
        initval = i['initval']
        mask = i['mask']
        resval &= ~mask
        value = initval << shift
        resval |= value

    return resval
