#!/usr/bin/env python

# This is a simple GUI demonstrator for the SWIG_Avrdude Python
# bindings to libavrdude.

# Its main purpose is to demonstrate that these Python bindings
# provide all the functionality that is needed for a full-featured AVR
# programming tool with similar features as the CLI version. It is not
# meant to be complete though, or to be a full replacement for the CLI
# tool.

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
import re
import time

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
    #print("Cannot determine build directory, module loading might fail.", file=sys.stderr)
    pass
else:
    sys.path.append(builddir)
    sys.path.append(builddir + '/python')

import swig_avrdude as ad

def avrdude_init():
    ad.init_cx()
    ad.init_config()

    found = False
    for d in [builddir, "/etc", "/usr/local/etc"]:
        p = pathlib.Path(d + "/avrdude.conf")
        if p.is_file():
            ad.read_config(d + "/avrdude.conf")
            return (True, f"Found avrdude.conf in {d}")

    return (False, "Sorry, no avrdude.conf could be found.")

def classify_devices():
    result = {
        'at90': [],
        'attiny': [],
        'atmega': [],
        'atxmega': [],
        'avr_de': [],
        'other': []
    }
    avr_de_re = re.compile(r'AVR\d+[DE][A-Z]\d+')
    part = ad.lfirst(ad.cvar.part_list)
    while part:
        p = ad.ldata_avrpart(part)
        part = ad.lnext(part)
        if not p.id.startswith('.'):
            if p.desc.startswith('AT90'):
                result['at90'].append(p.desc)
            elif p.desc.startswith('ATtiny'):
                result['attiny'].append(p.desc)
            elif p.desc.startswith('ATmega'):
                result['atmega'].append(p.desc)
            elif p.desc.startswith('ATxmega'):
                result['atxmega'].append(p.desc)
            elif avr_de_re.match(p.desc):
                result['avr_de'].append(p.desc)
            else:
                result['other'].append(p.desc)
    return result

def classify_programmers():
    result = {
        'isp': [],
        'tpi': [],
        'pdi': [],
        'updi': [],
        'jtag': [],
        'spm': [],
        'hv': [],
        'other': []
    }
    pgm = ad.lfirst(ad.cvar.programmers)
    while pgm:
        p = ad.ldata_programmer(pgm)
        pgm = ad.lnext(pgm)
        names = []
        l = ad.lfirst(p.id)
        while l:
            names.append(ad.ldata_string(l))
            l = ad.lnext(l)
        pm = p.prog_modes
        matched = False
        if (pm & ad.PM_ISP) != 0:
            for name in names:
                result['isp'].append(name)
            matched = True
        if (pm & ad.PM_TPI) != 0:
            for name in names:
                result['tpi'].append(name)
            matched = True
        if (pm & ad.PM_PDI) != 0:
            for name in names:
                result['pdi'].append(name)
            matched = True
        if (pm & ad.PM_UPDI) != 0:
            for name in names:
                result['updi'].append(name)
            matched = True
        if (pm & (ad.PM_JTAG | ad.PM_JTAGmkI | ad.PM_XMEGAJTAG)) != 0:
            for name in names:
                result['jtag'].append(name)
            matched = True
        if (pm & ad.PM_SPM) != 0:
            for name in names:
                result['spm'].append(name)
            matched = True
        if (pm & (ad.PM_HVSP | ad.PM_HVPP)) != 0:
            for name in names:
                result['hv'].append(name)
            matched = True
        if not matched:
            for name in names:
                result['other'].append(name)
    return result

def size_to_str(size: int):
    if size >= 1024:
        return f"{size // 1024} KiB"
    return f"{size} B"

def yesno(val: bool):
    if val:
        return "Y"
    return "N"

def avrpart_to_mem(avrpart):

    if str(type(avrpart)).find('AVRPART') < 0:
        raise Exception(f"wrong argument: {type(avrpart)}, expecting swig_avrdude.AVRPART")

    res = []
    m = ad.lfirst(avrpart.mem)
    while m:
        mm = ad.ldata_avrmem((m))
        res.append(mm)
        m = ad.lnext(m)
    return res

def find_mem_alias(p, name):
    '''Find aliased memory name if any. Return original name otherwise.'''
    m = ad.avr_locate_mem(p, name)
    a = ad.avr_find_memalias(p, m)
    if not a:
        return name
    return a.desc

def dissect_fuse(config: list, fuse: str, val: int):
    '''
    Analyze the value of a particular fuse

    config: configuration list from ad.get_config_table()
    fuse: string of fuse to handle
    val: integer value of the fuse
    '''
    result = []
    for i in config:
        # 'name', 'vlist', 'memstr', 'memoffset', 'mask', 'lsh', 'initval', 'ccomment'
        s = i['memstr']
        if s == 'lock':
            continue
        if s.find('fuse') == -1:
            # avrX where fuseN is an alias only
            s = 'fuse' + str(i['memoffset']) # make it fuseN
        if s != fuse:
            continue
        name = i['name']
        vlist = i['vlist']
        thisval = val & i['mask']
        thisval = thisval >> i['lsh']
        ccmt = i['ccomment']
        for j in vlist:
            # 'value', 'label', 'vcomment'
            if j['value'] == thisval:
                lbl = j['label']
                vcmt = j['vcomment']
                result.append((name, lbl))
    return result

def default_fuse(config: list, fuse: str):
    '''
    Synthesize the default value for a fuse

    config: configuration list from ad.get_config_table()
    fuse: string of fuse to handle
    '''
    resval = 0xff
    for i in config:
        # 'name', 'vlist', 'memstr', 'memoffset', 'mask', 'lsh', 'initval', 'ccomment'
        s = i['memstr']
        if s == 'lock':
            continue
        if s.find('fuse') == -1:
            # avrX where fuseN is an alias only
            s = 'fuse' + str(i['memoffset']) # make it fuseN
        if s != fuse:
            continue
        shift = i['lsh']
        initval = i['initval']
        mask = i['mask']
        resval &= ~mask
        value = initval << shift
        resval |= value

    return resval

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
    resval = 0xff
    for i in config:
        # 'name', 'vlist', 'memstr', 'memoffset', 'mask', 'lsh', 'initval', 'ccomment'
        s = i['memstr']
        if s == 'lock':
            continue
        if s.find('fuse') == -1:
            # avrX where fuseN is an alias only
            s = 'fuse' + str(i['memoffset']) # make it fuseN
        if s != fuse:
            continue
        resval &= ~i['mask']
        vlist = i['vlist']
        shift = i['lsh']
        for j in vlist:
            # 'value', 'label', 'vcomment'
            if j['label'] in itemlist:
                thisval = j['value'] << shift
                resval |= thisval
    return resval

# actual GUI part starts here

global pyside
try:
    from PySide6.QtWidgets import *
    from PySide6.QtGui import *
    from PySide6.QtCore import *
    from PySide6.QtUiTools import QUiLoader
    from functools import partial
    pyside = 6
except ModuleNotFoundError:
    try:
        from PySide2.QtWidgets import *
        from PySide2.QtGui import *
        from PySide2.QtCore import *
        from PySide2.QtUiTools import QUiLoader
        from functools import partial
        pyside = 2
    except ModuleNotFoundError:
        print("Neither PySide6 (Qt6) nor PySide2 (Qt5) found, cannot create a GUI.",
              file = sys.stderr)
        sys.exit(1)


class EnterFilter(QObject):
    '''Filters <Key_Enter> and <Key_Return> events'''
    def __init__(self, callback = None):
        super().__init__()
        self.callback = callback

    def eventFilter(self, source, event):
        # catch <Enter> or <Return> keys, finish QLineEdit editing,
        # but prevent propagating the event to avoid "accept"ing the
        # entire dialog
        if event.type() == QEvent.KeyPress and \
           (event.key() == Qt.Key_Return or
            event.key() == Qt.Key_Enter):
            source.editingFinished.emit()
            return True
        # catch context menu action on fuse edits: used to pop up a
        # dialog with fuse details rather than the standard context
        # menu (select/copy/paste)
        if event.type() == QEvent.ContextMenu:
            if source.isVisible():
                if self.callback:
                    self.callback(source)
                    return True
        return False

class FusePopup():
    def __init__(self, lineedit, config, fusename):
        self.lineedit = lineedit
        self.config = config
        self.fusename = fusename

        Dialog = QDialog()
        self.dialog = Dialog
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.setWindowModality(Qt.ApplicationModal)
        Dialog.setWindowTitle(f"Fuse value selection for {fusename}")
        Dialog.resize(800, 600)
        self.verticalLayout = QVBoxLayout(Dialog)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.scrollArea = QScrollArea(Dialog)
        self.scrollArea.setObjectName(u"scrollArea")
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, 0, 378, 248))
        self.gridLayout = QGridLayout(self.scrollAreaWidgetContents)
        self.gridLayout.setObjectName(u"gridLayout")
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout.addWidget(self.scrollArea)
        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName(u"buttonBox")
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.verticalLayout.addWidget(self.buttonBox)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.accepted.connect(self.get_entries)
        self.buttonBox.rejected.connect(Dialog.reject)
        QMetaObject.connectSlotsByName(Dialog)
        self.construct_gui()
        self.update_entries()
        self.dialog.show()

    def construct_gui(self):
        rownumber = 0 # row in grid layout
        self.widgets = []
        for i in self.config:
            # 'name', 'vlist', 'memstr', 'memoffset', 'mask', 'lsh', 'initval', 'ccomment'
            s = i['memstr']
            if s == 'lock':
                continue
            if s.find('fuse') == -1:
                # avrX where fuseN is an alias only
                s = 'fuse' + str(i['memoffset']) # make it fuseN
            if s != self.fusename:
                continue
            # add one row to grid layout, label with name, combobox with values
            name = i['name']
            vlist = i['vlist']
            ccmt = i['ccomment']
            l = QLabel(self.scrollAreaWidgetContents)
            l.setObjectName(f"Label{rownumber}")
            l.setText(ccmt)
            l.setToolTip(name)
            self.widgets.append(l)
            c = QComboBox(self.scrollAreaWidgetContents)
            c.setObjectName(f"combobox_{rownumber}")
            self.widgets.append(c)
            self.gridLayout.addWidget(l, rownumber, 0, 1, 1)
            self.gridLayout.addWidget(c, rownumber, 1, 1, 1)
            for j in vlist:
                # 'value', 'label', 'vcomment'
                lbl = j['label']
                vcmt = j['vcomment']
                c.addItem(vcmt, lbl)
            rownumber += 1

    def update_entries(self):
        entry = self.lineedit.text()
        default = default_fuse(self.config, self.fusename)
        if entry == "":
            value = default
        elif entry.isspace():
            value = default
        else:
            try:
                value = int(entry, 16)
            except ValueError:
                self.log(f"Invalid entry '{entry}' for {self.fusename}, reverting to default",
                         ad.MSG_WARNING)
                value = default
        vallist = dissect_fuse(self.config, self.fusename, value)
        for ele in vallist:
            name = ele[0]
            val = ele[1]
            widgets = self.widgets
            while w := widgets[:2]:
                label = w[0]
                combo = w[1]
                if label.toolTip() == name:
                    idx = combo.findData(val)
                    if idx == -1:
                        self.log(f"Could not find combobox data for {val} in {name}",
                                 ad.MSG_WARNING)
                    else:
                        combo.setCurrentIndex(idx)
                    break
                widgets = widgets[2:]

    def get_entries(self):
        result = []
        widgets = self.widgets
        while w := widgets[:2]:
            label = w[0]
            combo = w[1]
            name = label.toolTip()
            value = combo.currentData()
            result.append((name, value))
            widgets = widgets[2:]
        fuseval = synthesize_fuse(self.config, self.fusename, result)
        self.lineedit.clear()
        self.lineedit.setText(f"{fuseval:02X}")

class listValidator(QValidator):

    def __init__(self, liste, buttonbox, combobox):
        super().__init__()
        self.list = liste
        self.buttonbox = buttonbox
        self.combobox = combobox

    def validate(self, string, position):
        if string == "":
            # empty string could always become a real match
            self.buttonbox.button(QDialogButtonBox.Ok).setEnabled(False)
            return QValidator.Intermediate, string, position

        s = string.lower()

        for i in self.list:
            i = i.lower()

            if i == s:
                # exact match
                self.buttonbox.button(QDialogButtonBox.Ok).setEnabled(True)
                return QValidator.Acceptable, string, position

        for i in self.list:
            i = i.lower()

            if i.find(s) != -1:
                # could become a real match some day
                self.buttonbox.button(QDialogButtonBox.Ok).setEnabled(False)
                # adjust the elements visible in the dropdown list
                self.combobox.setEditable(False)
                self.combobox.clear()
                for j in self.list:
                    if j.lower().find(s) != -1:
                        self.combobox.addItem(j)
                self.combobox.setEditable(True)
                self.combobox.setEditText(s)
                self.combobox.setValidator(self)
                return QValidator.Intermediate, string, position

        # no match at all, invalid input
        self.buttonbox.button(QDialogButtonBox.Ok).setEnabled(False)
        return QValidator.Invalid, string, position

    def fixup(self, string):
        # possibly change case of string for an exact match
        s = string.lower()

        for i in self.list:
            if s == i.lower():
                return i
        # hmm, not found - use the first partial match
        for i in self.list:
            if i.lower().find(s) != -1:
                return i

        # nothing at all
        return string

class adgui(QObject):
    def __init__(self, argv):
        super().__init__()

        # members for logging
        self.logstring = "<font color='#000060'><strong>Welcome to AVRDUDE!</strong></font><br>\n"
        self.at_bol = { 'stdout': True, 'stderr': True }
        self.debuglog = ""
        self.debug_bol = True

        # the main Qt app
        self.app = QApplication(sys.argv)

        self.port = None
        self.pgm = None
        self.dev = None
        self.dev_selected = None
        self.prog_selected = None
        self.connected = False

        self.flash_size = 0

        ad.set_msg_callback(self.msg_callback)
        ad.set_progress_callback(self.progress_callback)

        p = pathlib.Path(argv[0])
        srcdir = str(p.parent)
        for f in [ "adgui.ui", "about.ui", "device.ui",
                   "devinfo.ui", "loglevel.ui", "programmer.ui",
                   "memories.ui", "askfuse.ui", "help.ui" ]:
            ui = QFile(srcdir + '/' + f)
            if not ui.open(QFile.ReadOnly):
                print(f"Cannot open {f}: {ui.errorString()}", file = sys.stderr)
                sys.exit(1)
            loader = QUiLoader()
            widgetname = f[:-3] # strip .ui suffix
            self.__dict__[widgetname] = loader.load(ui)
            ui.close()
            if not self.__dict__[widgetname]:
                print(loader.errorString(), file = sys.stderr)

        # Create pixmap for AVR logo from above, create a QGraphicsScene
        # out of it, and populate the QGraphicsView items with it.
        self.avrlogo()
        logo = QPixmap()
        logo.loadFromData(self.avrlogo)
        gsc = QGraphicsScene()
        gsc.addPixmap(logo)
        self.memories.avr.setScene(gsc)
        self.memories.ee_avr.setScene(gsc)
        self.memories.fuse_avr.setScene(gsc)

        self.disable_fuses()

        self.helptext()
        self.help.textBrowser.setHtml(self.helptext)

        self.adgui.show()

        self.adgui.actionAbout.triggered.connect(self.about.show)
        self.adgui.actionUsage.triggered.connect(self.help.show)
        self.adgui.actionDevice.triggered.connect(self.device.show)
        self.app.lastWindowClosed.connect(self.cleanup)
        self.adgui.actionProgrammer.triggered.connect(self.programmer.show)
        self.adgui.loggingArea.setHtml(self.logstring)
        self.adgui.actionSave_log.triggered.connect(self.save_logfile)

        (success, message) = avrdude_init()
        self.initialized = success
        self.log(message)
        self.loglevel.radioButton.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_2.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_3.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_4.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_5.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_6.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_7.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_8.toggled.connect(self.loglevel_changed)
        self.loglevel.radioButton_9.toggled.connect(self.loglevel_changed)
        self.adgui.actionLog_level.triggered.connect(self.loglevel.show)
        if not success:
            self.adgui.actionDevice.setEnabled(False)
            self.adgui.actionProgrammer.setEnabled(False)
            # essentially, only Exit and Help work anymore
        else:
            self.adgui.actionAttach.triggered.connect(self.start_programmer)
            self.adgui.actionDetach.triggered.connect(self.stop_programmer)
            self.devices = classify_devices()
            self.update_device_cb()
            self.device.at90.stateChanged.connect(self.update_device_cb)
            self.device.attiny.stateChanged.connect(self.update_device_cb)
            self.device.atmega.stateChanged.connect(self.update_device_cb)
            self.device.atxmega.stateChanged.connect(self.update_device_cb)
            self.device.avr_de.stateChanged.connect(self.update_device_cb)
            self.device.other.stateChanged.connect(self.update_device_cb)
            self.device.buttonBox.accepted.connect(self.device_selected)
            self.programmers = classify_programmers()
            self.update_programmer_cb()
            self.programmer.isp.stateChanged.connect(self.update_programmer_cb)
            self.programmer.tpi.stateChanged.connect(self.update_programmer_cb)
            self.programmer.pdi.stateChanged.connect(self.update_programmer_cb)
            self.programmer.updi.stateChanged.connect(self.update_programmer_cb)
            self.programmer.hv.stateChanged.connect(self.update_programmer_cb)
            self.programmer.other.stateChanged.connect(self.update_programmer_cb)
            self.programmer.buttonBox.accepted.connect(self.programmer_selected)
            self.programmer.programmers.currentTextChanged.connect(self.programmer_update_port)
            self.adgui.actionDevice_Info.triggered.connect(self.devinfo.show)
            self.adgui.actionProgramming.triggered.connect(self.memories.show)
            self.memories.readSig.pressed.connect(self.read_signature)
            self.memories.choose.pressed.connect(self.ask_flash_file)
            self.memories.read.pressed.connect(self.flash_read)
            self.memories.program.pressed.connect(self.flash_write)
            self.memories.save.pressed.connect(self.flash_save)
            self.memories.load.pressed.connect(self.flash_load)
            self.memories.erase.pressed.connect(self.chip_erase)
            self.memories.clear.pressed.connect(self.clear_buffer)
            self.memories.filename.editingFinished.connect(self.detect_flash_file)
            self.memories.ee_choose.pressed.connect(self.ask_eeprom_file)
            self.memories.ee_read.pressed.connect(self.eeprom_read)
            self.memories.ee_program.pressed.connect(self.eeprom_write)
            self.memories.ee_save.pressed.connect(self.eeprom_save)
            self.memories.ee_load.pressed.connect(self.eeprom_load)
            self.memories.ee_filename.editingFinished.connect(self.detect_eeprom_file)
            self.memories.fuse_choose.pressed.connect(self.ask_fuses_file)
            self.memories.fuse_read.pressed.connect(self.read_fuses)
            self.memories.fuse_program.pressed.connect(self.program_fuses)
            self.memories.fuse_save.pressed.connect(self.fuses_save)
            self.memories.fuse_load.pressed.connect(self.fuses_load)
            self.memories.fuse_filename.editingFinished.connect(self.detect_fuses_file)
            for w in self.memories.groupBox_13.children():
                if w.objectName().startswith('fval'):
                    # functools.partial() is black magic, it allows to
                    # pass an argument to the called slot
                    w.editingFinished.connect(partial(self.fuseval_changed, w))
            self.load_settings()
            self.enter_filter = EnterFilter()
            self.memories.filename.installEventFilter(self.enter_filter)
            self.memories.ee_filename.installEventFilter(self.enter_filter)
            self.memories.fuse_filename.installEventFilter(self.enter_filter)
            self.fuse_enter_filter = EnterFilter(self.fuse_popup)
            for obj in self.memories.groupBox_13.children():
                obj.installEventFilter(self.fuse_enter_filter)

        self.buffer_empty = 'background-color: rgb(255,240,240);'
        self.buffer_full = 'background-color: rgb(240,255,240);'

    def log(self, s: str, level: int = ad.MSG_INFO, no_nl: bool = False):
        # level to color mapping
        colors = [
            '#804040', # MSG_EXT_ERROR
            '#A03030', # MSG_ERROR
            '#A08000', # MSG_WARNING
            '#000000', # MSG_INFO
            '#006000', # MSG_NOTICE
            '#005030', # MSG_NOTICE2
            '#808080', # MSG_DEBUG
            '#60A060', # MSG_TRACE - not used
            '#6060A0', # MSG_TRACE2 - not used
        ]
        color = colors[level - ad.MSG_EXT_ERROR]
        html = None
        if level <= ad.MSG_WARNING:
            html = f"<font color={color}><strong>{s}</strong></font>"
        elif level < ad.MSG_TRACE:
            html = f"<font color={color}>{s}</font>"
        if html and (not no_nl or s[-1] == '\n'):
            html += "<br>\n"
        if s != "" and s != "\n":
            new_bol = not no_nl or (s[-1] == '\n')
            if not no_nl:
                s += '\n'
            # always save non-empty messages to debug log
            # prepend timestamp when at beginning of line
            if self.debug_bol:
                tstamp = time.strftime('%Y-%m-%dT%H:%M:%S')
                self.debuglog += f"{tstamp} {s}"
            else:
                self.debuglog += s
            self.debug_bol = new_bol
        if html:
            # only update loggingArea if not trace message
            self.logstring += html
            self.adgui.loggingArea.setHtml(self.logstring)
            self.adgui.loggingArea.moveCursor(QTextCursor.End)

    def message_type(self, msglvl: int):
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
    def msg_callback(self, target: str, lno: int, fname: str, func: str,
                     msgmode: int, msglvl: int, msg: str, backslash_v: bool):
        if ad.cvar.verbose >= msglvl:
            s = ""
            if msgmode & ad.MSG2_PROGNAME:
                if not self.at_bol[target]:
                    s += "\n"
                    self.at_bol[target] = True
                s += ad.cvar.progname + ": "
                if ad.cvar.verbose >= ad.MSG_NOTICE and (msgmode & ad.MSG2_FUNCTION) != 0:
                    s += " " + func + "()"
                if ad.cvar.verbose >= ad.MSG_DEBUG and (msgmode & ad.MSG2_FILELINE) != 0:
                    n = os.path.basename(fname)
                    s += f" [{n}:{lno}]"
                if (msgmode & ad.MSG2_TYPE) != 0:
                    s += " " + self.message_type(msglvl)
                    s += ": "
            elif (msgmode & ad.MSG2_INDENT1) != 0:
                s = (len(ad.cvar.progname) + 1) * ' '
            elif (msgmode & ad.MSG2_INDENT2) != 0:
                s = (len(ad.cvar.progname) + 2) * ' '
            if backslash_v and not self.at_bol[target]:
                s += "\n"
            s += msg
            try:
                self.at_bol[target] = s[-1] == '\n'
            except UnicodeDecodeError:
                s = str(s)
                self.at_bol[target] = s[-1] == '\n'
            self.log(s, msglvl, no_nl = True)

    def progress_callback(self, percent: int, etime: float, hdr: str, finish: int):
        if hdr:
            self.adgui.operation.setText(hdr)
            self.adgui.progressBar.setEnabled(True)
        if percent == 100:
            if finish != -1:
                # "normal" end: reset and turn off progress bar
                self.adgui.progressBar.setValue(0)
                self.adgui.time.setText("-:--")
                self.adgui.operation.setText("")
            #else: freeze progress bar at previous value
            # but disable it anyway
            self.adgui.progressBar.setEnabled(False)
        else:
            self.adgui.progressBar.setValue(percent)
            secs = int(etime % 60)
            mins = int(etime / 60)
            self.adgui.time.setText(f"{mins}:{secs:02d}")
        self.app.processEvents()

    def load_settings(self):
        self.settings = QSettings(QSettings.NativeFormat, QSettings.UserScope, 'avrdude', 'adgui')
        s = self.settings
        name = s.fileName()
        k = s.allKeys()
        if (amnt := len(k)) == 0:
            # new file
            self.log(f"Settings file: {name}", ad.MSG_NOTICE)
        else:
            # we loaded something
            self.log(f"Loaded {amnt} settings from {name}", ad.MSG_INFO)
            if 'settings/log_level' in k:
                ll = int(s.value('settings/log_level'))
                ad.cvar.verbose = ll
                found = False
                for obj in self.loglevel.groupBox.children():
                    if not obj.objectName().startswith('radioButton'):
                        continue
                    tt = obj.toolTip()
                    if tt and (int(tt) == ll):
                        obj.setChecked(True)
                        found = True
                        break
                if not found:
                    # appropriate level not found, default to INFO
                    self.loglevel.radioButton_4.setChecked(True)
                    self.cvar.verbose = 0
                    ll = 0
                self.log(f"Log level set to {ll}", ad.MSG_INFO)
            if 'file/device' in k:
                n = s.value('file/device')
                idx = self.device.devices.findText(n)
                if idx != -1:
                    self.device.devices.setCurrentIndex(idx)
                    self.dev = ad.locate_part(ad.cvar.part_list, n)
                    self.dev_selected = n
                    self.update_device_info()
                    self.adgui.actionDevice_Info.setEnabled(True)
                    self.enable_fuses()
                    self.log(f"Device set to {n}", ad.MSG_INFO)
                    self.devcfg = ad.get_config_table(self.dev.desc)
                    if not self.devcfg:
                        self.log("No configuration table found", ad.MSG_WARNING)
            if 'file/programmer' in k:
                n = s.value('file/programmer')
                idx = self.programmer.programmers.findText(n)
                if idx != -1:
                    self.programmer.programmers.setCurrentIndex(idx)
                    self.pgm = ad.locate_programmer(ad.cvar.programmers, n)
                    self.prog_selected = n
                    self.log(f"Programmer set to {n}", ad.MSG_INFO)
            if 'file/port' in k:
                n = s.value('file/port')
                self.port = n
                self.programmer.port.setText(n)
                self.log(f"Port set to {n}")
            if self.port != "set_this" and self.pgm and self.dev:
                self.adgui.actionAttach.setEnabled(True)

    def update_device_cb(self):
        fams = list(self.devices.keys())
        #fams.sort()
        self.device.devices.clear()
        l = []
        for f in fams:
            obj = eval('self.device.' + f + '.isChecked()')
            if obj:
                for d in self.devices[f]:
                    self.device.devices.addItem(d)
                    l.append(d)
        self.dev_validator = listValidator(l, self.device.buttonBox, self.device.devices)
        self.device.devices.setValidator(self.dev_validator)

    def update_programmer_cb(self):
        fams = list(self.programmers.keys())
        self.programmer.programmers.clear()
        l = {}
        for f in fams:
            obj = eval('self.programmer.' + f + '.isChecked()')
            if obj:
                for d in self.programmers[f]:
                    l[d] = True
        l = list(l.keys())
        l.sort()
        for k in l:
            self.programmer.programmers.addItem(k)
        self.pgm_validator = listValidator(l, self.programmer.buttonBox,
                                           self.programmer.programmers)
        self.programmer.programmers.setValidator(self.pgm_validator)

    def update_device_info(self):
        p = ad.locate_part(ad.cvar.part_list, self.dev_selected)
        if not p:
            self.log(f"Could not find {self.dev_selected} again, confused\n")
            return
        ad.avr_initmem(p)
        self.devinfo.label_2.setText(p.desc)
        self.devinfo.label_4.setText(p.id)
        self.devinfo.label_6.setText(p.config_file)
        self.devinfo.label_8.setText(str(p.lineno))
        model = QStandardItemModel()
        model.setHorizontalHeaderLabels(['Name', 'Size', 'Paged', 'Page Size', '# Pages'])
        mm = avrpart_to_mem(p)
        row = 0
        for m in mm:
            model.setItem(row, 0, QStandardItem(m.desc))
            sz = QStandardItem(size_to_str(m.size))
            sz.setTextAlignment(Qt.Alignment(int(Qt.AlignRight) | int(Qt.AlignVCenter)))
            model.setItem(row, 1, sz)
            pg = QStandardItem(yesno(m.paged))
            pg.setTextAlignment(Qt.Alignment(int(Qt.AlignHCenter) | int(Qt.AlignVCenter)))
            model.setItem(row, 2, pg)
            if m.paged:
                sz = QStandardItem(size_to_str(m.page_size))
                sz.setTextAlignment(Qt.Alignment(int(Qt.AlignRight) | int(Qt.AlignVCenter)))
                model.setItem(row, 3, sz)
                pg = QStandardItem(str(m.num_pages))
                pg.setTextAlignment(Qt.Alignment(int(Qt.AlignRight) | int(Qt.AlignVCenter)))
                model.setItem(row, 4, pg)
            row += 1
        self.devinfo.tableMemories.setModel(model)
        self.devinfo.tableMemories.resizeColumnsToContents()
        self.devinfo.tableMemories.resizeRowsToContents()
        self.devinfo.listVariants.clear()
        v = ad.lfirst(p.variants)
        while v:
            vv = ad.ldata_string(v)
            self.devinfo.listVariants.addItem(vv)
            v = ad.lnext(v)
        # update signature TAB in memories popup
        sig = p.signature
        sigstr = sig.hex(' ').upper()
        self.memories.configSig.setText(sigstr)

    def device_selected(self):
        self.dev_selected = self.device.devices.currentText()
        self.dev = ad.locate_part(ad.cvar.part_list, self.dev_selected)
        if not self.dev:
            self.log(f"Invalid device selection: {self.dev_selected}")
            return
        self.log(f"Selected device: {self.dev_selected}")
        self.settings.setValue('file/device', self.dev_selected)
        self.update_device_info()
        self.adgui.actionDevice_Info.setEnabled(True)
        self.enable_fuses()
        self.devcfg = ad.get_config_table(self.dev.desc)
        if not self.devcfg:
            self.log("No configuration table found", ad.MSG_WARNING)
        if self.port != "set_this" and self.prog_selected and self.dev_selected:
            self.adgui.actionAttach.setEnabled(True)

    def programmer_selected(self):
        self.prog_selected = self.programmer.programmers.currentText()
        self.pgm = ad.locate_programmer(ad.cvar.programmers, self.prog_selected)
        if not self.pgm:
            self.log(f"Invalid programmer selection: {self.prog_selected}")
            return
        self.port = self.programmer.port.text()
        self.log(f"Selected programmer: {self.pgm.desc} ({self.prog_selected})")
        self.settings.setValue('file/programmer', self.prog_selected)
        self.log(f"Selected port: {self.port}")
        self.settings.setValue('file/port', self.port)
        if self.port != "set_this" and self.prog_selected and self.dev_selected:
            self.adgui.actionAttach.setEnabled(True)

    def programmer_update_port(self):
        selected = self.programmer.programmers.currentText()
        pgm = ad.locate_programmer(ad.cvar.programmers, selected)
        if not pgm:
            return
        if pgm.conntype == ad.CONNTYPE_USB:
            self.programmer.port.clear()
            self.programmer.port.insert("usb")
        elif pgm.conntype == ad.CONNTYPE_LINUXGPIO:
            self.programmer.port.clear()
            self.programmer.port.insert("dummy")

    def loglevel_changed(self, checked: bool):
        btn = self.sender()
        if checked:
            # we abuse the tooltip for the verbosity value
            val = int(btn.toolTip())
            ad.cvar.verbose = val
            self.settings.setValue('settings/log_level', val)

    def start_programmer(self):
        if self.connected:
            return
        ad.init_cx(self.pgm)
        self.pgm.initpgm()
        self.pgm.setup()
        rv = self.pgm.open(self.port)
        if rv == -1:
            self.log('Could not open programmer', ad.MSG_ERROR)
        else:
            self.pgm.enable(self.dev)
            self.pgm.initialize(self.dev)
            self.log('Programmer successfully started')
            self.adgui.actionProgramming.setEnabled(True)
            self.adgui.actionAttach.setEnabled(False)
            self.adgui.actionDetach.setEnabled(True)
            self.connected = True
            self.fuses_warned = {}

    def stop_programmer(self):
        if self.connected:
            self.pgm.disable()
            self.pgm.close()
            self.pgm.teardown()
            self.log('Programmer stopped')
            self.adgui.actionAttach.setEnabled(True)
            self.adgui.actionDetach.setEnabled(False)
            self.adgui.actionProgramming.setEnabled(False)
            self.connected = False

    def cleanup(self):
        self.settings.sync()
        self.stop_programmer()

    def read_signature(self):
        sig_ok = "background-color: rgb(255, 255, 255);\ncolor: rgb(0, 100, 0);"
        sig_bad = "background-color: rgb(255, 255, 255);\ncolor: rgb(150, 0, 0);"
        if self.connected:
            m = ad.avr_locate_mem(self.dev, 'signature')
            if m:
                ad.avr_read_mem(self.pgm, self.dev, m)
                self.progress_callback(100, 0, "", 0) # clear progress bar
                read_sig = m.get(3)
                if read_sig == self.dev.signature:
                    self.memories.deviceSig.setStyleSheet(sig_ok)
                else:
                    self.memories.deviceSig.setStyleSheet(sig_bad)
                    self.log("Signature read from device does not match config file",
                             ad.MSG_WARNING)
                self.memories.flash.setEnabled(True)
                self.memories.eeprom.setEnabled(True)
                self.memories.fuses.setEnabled(True)
                sigstr = read_sig.hex(' ').upper()
                self.memories.deviceSig.setText(sigstr)
                p = ad.locate_part_by_signature(ad.cvar.part_list, read_sig)
                if p:
                    self.memories.candidate.setText(p.desc)
                else:
                    self.memories.candidate.setText("???")
            else:
                ad.log("Could not find signature memory", ad.MSG_ERROR)

    def ask_flash_file(self):
        dlg = QFileDialog(caption = "Select file",
                          filter = "Load files (*.elf *.hex *.eep *.srec *.bin);; All Files (*)")
        if dlg.exec():
            self.memories.filename.setText(dlg.selectedFiles()[0])
            self.detect_flash_file()

    def detect_flash_file(self):
        # If file exists, try finding out real format. If file doesn't
        # exist, try guessing the intended file format based on the
        # suffix.
        fname = self.memories.filename.text()
        if len(fname) > 0:
            self.flashname = fname
            self.memories.load.setEnabled(True)
            self.memories.save.setEnabled(True)
        else:
            # no filename, disable load/save buttons
            self.flashname = None
            self.memories.load.setEnabled(False)
            self.memories.save.setEnabled(False)
            return
        p = pathlib.Path(fname)
        if p.is_file():
            fmt = ad.fileio_fmt_autodetect(fname)
            if fmt == ad.FMT_ELF:
                self.memories.ffELF.setChecked(True)
            elif fmt == ad.FMT_IHEX:
                self.memories.ffIhex.setChecked(True)
            elif fmt == ad.FMT_SREC:
                self.memories.ffSrec.setChecked(True)
        else:
            if fname.endswith('.hex') or fname.endswith('.ihex') \
               or fname.endswith('.eep'): # common name for EEPROM Intel hex files
                self.memories.ffIhex.setChecked(True)
            elif fname.endswith('.srec'):
                self.memories.ffSrec.setChecked(True)
            elif fname.endswith('.bin'):
                self.memories.ffRbin.setChecked(True)

    def flash_read(self):
        self.adgui.progressBar.setEnabled(True)
        m = ad.avr_locate_mem(self.dev, 'flash')
        if not m:
            self.log("Could not find 'flash' memory", ad.MSG_ERROR)
            return
        amnt = ad.avr_read_mem(self.pgm, self.dev, m)
        self.flash_size = amnt
        self.log(f"Read {amnt} bytes")
        if amnt > 0:
            self.memories.buffer.setStyleSheet(self.buffer_full)

    def flash_write(self):
        self.adgui.progressBar.setEnabled(True)
        m = ad.avr_locate_mem(self.dev, 'flash')
        if not m:
            self.log("Could not find 'flash' memory", ad.MSG_ERROR)
            return
        if self.flash_size == 0:
            self.log("No data to write into 'flash' memory", ad.MSG_WARNING)
            return
        amnt = ad.avr_write_mem(self.pgm, self.dev, m, self.flash_size)
        self.log(f"Programmed {amnt} bytes")

    def clear_buffer(self):
        m = ad.avr_locate_mem(self.dev, 'flash')
        if not m:
            self.log("Could not find 'flash' memory", ad.MSG_ERROR)
            return
        m.clear(m.size)
        self.log(f"Cleared {m.size} bytes of buffer, and allocation flags")
        self.flash_size = 0
        self.memories.buffer.setStyleSheet(self.buffer_empty)

    def flash_save(self):
        if self.memories.ffAuto.isChecked() or \
           self.memories.ffELF.isChecked():
            self.log("Auto or ELF are not valid for saving files", ad.MSG_ERROR)
            return
        if self.memories.ffIhex.isChecked():
            fmt = ad.FMT_IHEX
        elif self.memories.ffSrec.isChecked():
            fmt = ad.FMT_SREC
        elif self.memories.ffRbin.isChecked():
            fmt = ad.FMT_RBIN
        else:
            self.log("Internal error: cannot determine file format", ad.MSG_ERROR)
            return
        fname = self.flashname
        p = pathlib.Path(fname)
        if p.is_file():
            result = QMessageBox.question(self.memories,
                                          f"Overwrite {fname}?",
                                          f"Do you want to overwrite {fname}?")
            if result != QMessageBox.StandardButton.Yes:
                return
        if self.flash_size != 0:
            amnt = self.flash_size
        else:
            amnt = -1
        amnt = ad.fileio(ad.FIO_WRITE, self.flashname, fmt, self.dev, "flash", amnt)
        self.log(f"Wrote {amnt} bytes to {self.flashname}")

    def flash_load(self):
        if self.memories.ffAuto.isChecked():
            fmt = ad.FMT_AUTO
        elif self.memories.ffELF.isChecked():
            fmt = ad.FMT_ELF
        elif self.memories.ffIhex.isChecked():
            fmt = ad.FMT_IHEX
        elif self.memories.ffSrec.isChecked():
            fmt = ad.FMT_SREC
        elif self.memories.ffRbin.isChecked():
            fmt = ad.FMT_RBIN
        else:
            self.log("Internal error: cannot determine file format", ad.MSG_ERROR)
            return
        amnt = ad.fileio(ad.FIO_READ, self.flashname, fmt, self.dev, "flash", -1)
        self.log(f"Read {amnt} bytes from {self.flashname}")
        self.flash_size = amnt
        if amnt > 0:
            self.memories.buffer.setStyleSheet(self.buffer_full)

    def chip_erase(self):
        result = QMessageBox.question(self.memories,
                                      f"Erase {self.dev.desc}?",
                                      f"Do you want to erase the entire device?")
        if result != QMessageBox.StandardButton.Yes:
            return
        result = self.pgm.chip_erase(self.dev)
        if result == 0:
            self.log("Device erased")
            self.flash_size = 0
            self.memories.buffer.setStyleSheet(self.buffer_empty)
        else:
            self.log("Failed to erase device", ad.MSG_WARNING)

    def save_logfile(self):
        fname = QFileDialog.getSaveFileName(caption = "Save logfile to",
                                            filter = "Text files (*.txt *.log);; All Files (*)")
        if fname:
            fname = fname[0]  # [1] is filter used
            if fname.rfind(".") == -1:
                # no suffix given
                fname += ".log"
            try:
                f = open(fname, "w")
            except Exception as e:
                self.log(f"Cannot create log file: {str(e)}", ad.LOG_EXT_ERROR)
                return
            try:
                f.write(self.debuglog)
                self.debuglog = ""
            except Exception as e:
                self.log(f"Cannot write log file: {str(e)}", ad.LOG_WXT_ERROR)

    def ask_eeprom_file(self):
        dlg = QFileDialog(caption = "Select file",
                          filter = "Load files (*.elf *.hex *.eep *.srec *.bin);; All Files (*)")
        if dlg.exec():
            self.memories.ee_filename.setText(dlg.selectedFiles()[0])
            self.detect_eeprom_file()

    def detect_eeprom_file(self):
        # If file exists, try finding out real format. If file doesn't
        # exist, try guessing the intended file format based on the
        # suffix.
        fname = self.memories.ee_filename.text()
        if len(fname) > 0:
            self.eepromname = fname
            self.memories.ee_load.setEnabled(True)
            self.memories.ee_save.setEnabled(True)
        else:
            # no filename, disable load/save buttons
            self.eepromname = None
            self.memories.ee_load.setEnabled(False)
            self.memories.ee_save.setEnabled(False)
            return
        p = pathlib.Path(fname)
        if p.is_file():
            fmt = ad.fileio_fmt_autodetect(fname)
            if fmt == ad.FMT_ELF:
                self.memories.ee_ffELF.setChecked(True)
            elif fmt == ad.FMT_IHEX:
                self.memories.ee_ffIhex.setChecked(True)
            elif fmt == ad.FMT_SREC:
                self.memories.ee_ffSrec.setChecked(True)
        else:
            if fname.endswith('.hex') or fname.endswith('.ihex') \
               or fname.endswith('.eep'): # common name for EEPROM Intel hex files
                self.memories.ee_ffIhex.setChecked(True)
            elif fname.endswith('.srec'):
                self.memories.ee_ffSrec.setChecked(True)
            elif fname.endswith('.bin'):
                self.memories.ee_ffRbin.setChecked(True)

    def eeprom_read(self):
        self.adgui.progressBar.setEnabled(True)
        m = ad.avr_locate_mem(self.dev, 'eeprom')
        if not m:
            self.log("Could not find 'eeprom' memory", ad.MSG_ERROR)
            return
        amnt = ad.avr_read_mem(self.pgm, self.dev, m)
        self.eeprom_size = amnt
        self.log(f"Read {amnt} bytes")
        if amnt > 0:
            self.memories.ee_buffer.setStyleSheet(self.buffer_full)

    def eeprom_write(self):
        self.adgui.progressBar.setEnabled(True)
        m = ad.avr_locate_mem(self.dev, 'eeprom')
        if not m:
            self.log("Could not find 'eeprom' memory", ad.MSG_ERROR)
            return
        if self.eeprom_size == 0:
            self.log("No data to write into 'eeprom' memory", ad.MSG_WARNING)
            return
        amnt = ad.avr_write_mem(self.pgm, self.dev, m, self.eeprom_size)
        self.log(f"Programmed {amnt} bytes")

    def clear_buffer(self):
        m = ad.avr_locate_mem(self.dev, 'eeprom')
        if not m:
            self.log("Could not find 'eeprom' memory", ad.MSG_ERROR)
            return
        m.clear(m.size)
        self.log(f"Cleared {m.size} bytes of buffer, and allocation flags")
        self.eeprom_size = 0
        self.memories.ee_buffer.setStyleSheet(self.buffer_empty)

    def eeprom_save(self):
        if self.memories.ee_ffAuto.isChecked() or \
           self.memories.ee_ffELF.isChecked():
            self.log("Auto or ELF are not valid for saving files", ad.MSG_ERROR)
            return
        if self.memories.ee_ffIhex.isChecked():
            fmt = ad.FMT_IHEX
        elif self.memories.ee_ffSrec.isChecked():
            fmt = ad.FMT_SREC
        elif self.memories.ee_ffRbin.isChecked():
            fmt = ad.FMT_RBIN
        else:
            self.log("Internal error: cannot determine file format", ad.MSG_ERROR)
            return
        fname = self.eepromname
        p = pathlib.Path(fname)
        if p.is_file():
            result = QMessageBox.question(self.memories,
                                          f"Overwrite {fname}?",
                                          f"Do you want to overwrite {fname}?")
            if result != QMessageBox.StandardButton.Yes:
                return
        if self.eeprom_size != 0:
            amnt = self.eeprom_size
        else:
            amnt = -1
        amnt = ad.fileio(ad.FIO_WRITE, self.eepromname, fmt, self.dev, "eeprom", amnt)
        self.log(f"Wrote {amnt} bytes to {self.eepromname}")

    def eeprom_load(self):
        if self.memories.ee_ffAuto.isChecked():
            fmt = ad.FMT_AUTO
        elif self.memories.ee_ffELF.isChecked():
            fmt = ad.FMT_ELF
        elif self.memories.ee_ffIhex.isChecked():
            fmt = ad.FMT_IHEX
        elif self.memories.ee_ffSrec.isChecked():
            fmt = ad.FMT_SREC
        elif self.memories.ee_ffRbin.isChecked():
            fmt = ad.FMT_RBIN
        else:
            self.log("Internal error: cannot determine file format", ad.MSG_ERROR)
            return
        amnt = ad.fileio(ad.FIO_READ, self.eepromname, fmt, self.dev, "eeprom", -1)
        self.log(f"Read {amnt} bytes from {self.eepromname}")
        self.eeprom_size = amnt
        if amnt > 0:
            self.memories.ee_buffer.setStyleSheet(self.buffer_full)

    def disable_fuses(self):
        # make all fuse labels and entries invisible
        for w in self.memories.groupBox_13.children():
            if w.objectName().find('Layout') != -1:
                continue
            w.setVisible(False)
            w.clear()

    def fuse_names(self):
        # return name of fuse memories for self.dev
        res = []
        m = ad.lfirst(self.dev.mem)
        while m:
            mm = ad.ldata_avrmem((m))
            name = mm.desc
            if name != 'fuses' and name.find('fuse') != -1:
                res.append(name)
            m = ad.lnext(m)
        return res

    def enable_fuses(self):
        # update fuse labels, and make them visible
        if 'fuselabels' in self.__dict__.keys():
            self.disable_fuses()
        fuses = self.fuse_names()
        if len(fuses) == 0:
            return
        idx = 0
        self.fuselabels = {}
        for name in fuses:
            # self.fuselabels has the fuse name as key, and a list of
            # [idx, allocated] as data. 'idx' is the GUI index for the
            # fuseN and fvalN label and lineedit, respectively.
            # 'allocated' is a flag indicating that there is
            # "interesting" data in the lineedit entry.
            self.fuselabels[name] = [idx, False]
            name = find_mem_alias(self.dev, name)
            eval(f"self.memories.fuse{idx}.setText({'name'})")
            eval(f"self.memories.fuse{idx}.setEnabled(True)")
            eval(f"self.memories.fuse{idx}.setVisible(True)")
            eval(f"self.memories.fval{idx}.setVisible(True)")
            idx += 1

    def ask_fuses_file(self):
        dlg = QFileDialog(caption = "Select file",
                          filter = "Load files (*.elf *.hex *.eep *.srec *.bin);; All Files (*)")
        if dlg.exec():
            self.memories.fuse_filename.setText(dlg.selectedFiles()[0])
            self.detect_fuses_file()

    def detect_fuses_file(self):
        # If file exists, try finding out real format. If file doesn't
        # exist, try guessing the intended file format based on the
        # suffix.
        fname = self.memories.fuse_filename.text()
        if len(fname) > 0:
            self.fusename = fname
            self.memories.fuse_load.setEnabled(True)
            self.memories.fuse_save.setEnabled(True)
        else:
            # no filename, disable load/save buttons
            self.fusename = None
            self.memories.fuse_load.setEnabled(False)
            self.memories.fuse_save.setEnabled(False)
            return
        for fuse in self.fuselabels.keys():
            m = ad.avr_locate_mem(self.dev, fuse)
            if not m:
                self.log(f"Could not find {fuse} memory", ad.MSG_ERROR)
                continue
            fname = self.fuse_filename(fuse)
            p = pathlib.Path(fname)
            if p.is_file():
                fmt = ad.fileio_fmt_autodetect(fname)
                if fmt == ad.FMT_ELF:
                    self.memories.fuse_ffELF.setChecked(True)
                elif fmt == ad.FMT_IHEX:
                    self.memories.fuse_ffIhex.setChecked(True)
                elif fmt == ad.FMT_SREC:
                    self.memories.fuse_ffSrec.setChecked(True)
            else:
                if fname.endswith('.hex') or fname.endswith('.ihex') \
                   or fname.endswith('.eep'): # common name for EEPROM Intel hex files
                    self.memories.fuse_ffIhex.setChecked(True)
                elif fname.endswith('.srec'):
                    self.memories.fuse_ffSrec.setChecked(True)
                elif fname.endswith('.bin'):
                    self.memories.fuse_ffRbin.setChecked(True)

    def read_fuses(self):
        for fuse in self.fuselabels.keys():
            m = ad.avr_locate_mem(self.dev, fuse)
            if not m:
                self.log(f"Could not find {fuse} memory", ad.MSG_ERROR)
                continue
            amnt = ad.avr_read_mem(self.pgm, self.dev, m)
            self.log(f"Read {amnt} bytes of {fuse}")
            if amnt != m.size:
                self.log(f"Read only {amnt} out of {m.size} bytes", ad.MSG_WARNING)
            val = int(m.get(1)[0])
            s = f"{val:02X}"
            (idx, allocated) = self.fuselabels[fuse]
            eval(f"self.memories.fval{idx}.clear()")
            eval(f"self.memories.fval{idx}.insert('{s}')")

    @Slot(QLineEdit)
    def fuseval_changed(self, le):
        slotnumber = int(le.objectName()[-1])
        for fuse in self.fuselabels.keys():
            if self.fuselabels[fuse][0] == slotnumber:
                m = ad.avr_locate_mem(self.dev, fuse)
                if not m:
                    self.log(f"Could not find {fuse} memory", ad.MSG_ERROR)
                    return
                t = le.text()
                if t == "" or t.isspace():
                    m.clear(m.size)
                    self.fuselabels[fuse][1] = False
                    self.log(f"Cleared {fuse} field", ad.MSG_DEBUG)
                    return
                try:
                    val = int(t, 16)
                except ValueError as e:
                    self.log(str(e), ad.MSG_WARNING)
                    return
                self.check_fuse(fuse, val)
                m.put(val.to_bytes(1, 'little'))
                self.fuselabels[fuse][1] = True
                self.log(f"Updated {fuse} memory")
                return
        self.log(f"Could not find a fuse for slot {slotnmber}", ad.MSG_ERROR)

    def program_fuses(self):
        for fuse in self.fuselabels.keys():
            m = ad.avr_locate_mem(self.dev, fuse)
            if not m:
                self.log(f"Could not find {fuse} memory", ad.MSG_ERROR)
                continue
            slotnumber = self.fuselabels[fuse][0]
            val = eval(f"self.memories.fval{slotnumber}.text()")
            if val.isspace():
                self.log(f"Not programming {fuse} memory: no value", ad.MSG_DEBUG)
                continue
            if not self.fuselabels[fuse][1]:
                self.log(f"Not programming {fuse} memory: not changed", ad.MSG_DEBUG)
                continue
            amnt = ad.avr_write_mem(self.pgm, self.dev, m, 1)
            if amnt > 0:
                self.log(f"Wrote {amnt} bytes of {fuse}")
            else:
                self.log(f"Failed to write {fuse}", ad.MSG_WARNING)
            self.fuselabels[fuse][1] = False

    def fuse_filename(self, fuse):
        fname = self.fusename
        if not fname:
            return ''
        if (percentpos := fname.find('%')) != -1:
            fname = fname[:percentpos] + fuse + fname[(percentpos + 1):]
        return fname

    def fuses_save(self):
        if self.memories.fuse_ffAuto.isChecked() or \
           self.memories.fuse_ffELF.isChecked():
            self.log("Auto or ELF are not valid for saving files", ad.MSG_ERROR)
            return
        if self.memories.fuse_ffIhex.isChecked():
            fmt = ad.FMT_IHEX
        elif self.memories.fuse_ffSrec.isChecked():
            fmt = ad.FMT_SREC
        elif self.memories.fuse_ffRbin.isChecked():
            fmt = ad.FMT_RBIN
        else:
            self.log("Internal error: cannot determine file format", ad.MSG_ERROR)
            return
        for fuse in self.fuselabels.keys():
            if not self.fuselabels[fuse][1]:
                # empty field, skip
                continue
            m = ad.avr_locate_mem(self.dev, fuse)
            if not m:
                self.log(f"Could not find {fuse} memory", ad.MSG_ERROR)
                continue
            fname = self.fuse_filename(fuse)
            p = pathlib.Path(fname)
            if p.is_file():
                result = QMessageBox.question(self.memories,
                                              f"Overwrite {fname}?",
                                              f"Do you want to overwrite {fname}?")
                if result != QMessageBox.StandardButton.Yes:
                    continue
            amnt = ad.fileio(ad.FIO_WRITE, fname, fmt, self.dev, fuse, m.size)
            self.log(f"Wrote {amnt} bytes of {fuse} to {fname}")

    def fuse_ask(self):
        self.askfuse.fuse_selection.clear()
        for fuse in self.fuselabels.keys():
            self.askfuse.fuse_selection.addItem(fuse)
        result = self.askfuse.exec()
        if result == QDialog.Accepted:
            return self.askfuse.fuse_selection.currentText()
        return None

    def fuses_load(self):
        if self.memories.fuse_ffAuto.isChecked():
            fmt = ad.FMT_AUTO
        elif self.memories.fuse_ffELF.isChecked():
            fmt = ad.FMT_ELF
        elif self.memories.fuse_ffIhex.isChecked():
            fmt = ad.FMT_IHEX
        elif self.memories.fuse_ffSrec.isChecked():
            fmt = ad.FMT_SREC
        elif self.memories.fuse_ffRbin.isChecked():
            fmt = ad.FMT_RBIN
        else:
            self.log("Internal error: cannot determine file format", ad.MSG_ERROR)
            return
        if self.fusename.find('%') == -1:
            # If absolute name, see whether it is ELF or something
            # else.  ELF can contain data for all fuses, all other
            # file formats only contain data for a single fuse, so we
            # must ask for which one it is intended.
            p = pathlib.Path(self.fusename)
            if p.is_file():
                if fmt == ad.FMT_AUTO:
                    fmt = ad.fileio_fmt_autodetect(fname)
                if fmt == ad.FMT_ELF:
                    # OK, assume multi-fuse
                    pass
                else:
                    # Ask
                    fname = self.fusename
                    fuse = self.fuse_ask()
                    if fuse:
                        m = ad.avr_locate_mem(self.dev, fuse)
                        if not m:
                            self.log(f"Could not find {fuse} memory", ad.MSG_ERROR)
                            return
                        amnt = ad.fileio(ad.FIO_READ, fname, fmt, self.dev, fuse, -1)
                        self.log(f"Read {amnt} bytes of {fuse} from {fname}")
                        if amnt > 0:
                            val = int(m.get(1)[0])
                            s = f"{val:02X}"
                            (idx, allocated) = self.fuselabels[fuse]
                            eval(f"self.memories.fval{idx}.clear()")
                            eval(f"self.memories.fval{idx}.insert('{s}')")
                            self.fuselabels[fuse][1] = True
                    return
        # proceed with attempting to load all fuses here
        for fuse in self.fuselabels.keys():
            m = ad.avr_locate_mem(self.dev, fuse)
            if not m:
                self.log(f"Could not find {fuse} memory", ad.MSG_ERROR)
                continue
            fname = self.fuse_filename(fuse)
            amnt = ad.fileio(ad.FIO_READ, fname, fmt, self.dev, fuse, -1)
            self.log(f"Read {amnt} bytes of {fuse} from {fname}")
            if amnt > 0:
                val = int(m.get(1)[0])
                s = f"{val:02X}"
                (idx, allocated) = self.fuselabels[fuse]
                eval(f"self.memories.fval{idx}.clear()")
                eval(f"self.memories.fval{idx}.insert('{s}')")
                self.fuselabels[fuse][1] = True

    def fuse_popup(self, widget):
        # if there is no config table, stop here
        if not self.devcfg:
            return
        # widget is supposed to be "fval<N>"
        fuse_id = int(widget.objectName()[-1])
        found = False
        # find out which fuse we are talking about
        for fuse in self.fuselabels.keys():
            idx = self.fuselabels[fuse][0]
            if idx == fuse_id:
                found = True
                break
        if not found:
            self.log(f"Internal error: cannot find fuse for {widget.objectName()}",
                     ad.MSG_ERROR)
            return
        self.fpop = FusePopup(widget, self.devcfg, fuse)
        self.fpop.dialog.accepted.connect(widget.editingFinished)

    def check_fuse(self, fuse, val):
        '''Check fuse for certain "dangerous" values'''
        dangerous = {
            'rstdisbl': ('.*gpio.*', 'Configuring /RESET as GPIO can be difficult to recover'),
            'spien': ('isp_disabled', 'Disabling ISP programming can be difficult to recover'),
            'dwen': ('dw_enabled', 'Enabling debugWIRE requires a debugWIRE-capable programmer and circuitry on /RESET pin'),
        }
        remarks = ''
        vallist = dissect_fuse(self.devcfg, fuse, val)
        count = 0
        for ele in vallist:
            name = ele[0]
            value = ele[1]
            for (tag, content) in dangerous.items():
                if name != tag:
                    continue
                if re.match(content[0], value):
                    if not value in self.fuses_warned:
                        remarks += content[1] + '\n'
                        self.fuses_warned[value] = True
                        count += 1
        if count > 0:
            if count == 1:
                remarks += '\n(This warning will not be displayed again.)'
            else:
                remarks += '\n(These warnings will not be displayed again.)'
            QMessageBox.warning(self.memories,
                                f"Some fuse values might be dangerous",
                                remarks)


    def helptext(self):
        self.helptext='''
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">
<html>
<head><title>Usage instructions</title></head>
<body>
<h1>Usage instructions</h1>

<h2>Preface</h2>

<p>Keep in mind that this is a GUI demonstrator only.</p>

<h2>GUI layout</h2>

<p>The GUI consists of three major parts: a menu bar to select the
various actions; a central part for operating messages, warnings and
errors; and interactive windows for individual tasks like programming
an AVR.</p>

<p>The center area is reserved for logging. Different log levels are
marked in different colors to allow for an easy optical
differentiation.</p>

<p>The log level shown can be selected using Settings → Log level …  Note
that log entries above "Debug" are not displayed in the log window but
only stored internally.</p>

<p>The internal log data can be written into a file using File → Save
log …</p>

<p>The selected log level is remembered in the configuration data
mentioned below.</p>

<h2>Operating instructions</h2>

<p>First pick the AVR using the File → Device dialog. Once selected,
Device → Info … shows more information about the part.</p>

<p>Then, use File → Programmer to select the programming hardware or
bootloader. Be sure to set the Port entry. For programmers that only
make sense on a particular port (e.g. "usb"), an attempt is made to
pre-fill the port.</p>

<p>All these values are saved in a platform-dependent configuration
database, and loaded from that place at next start. Thus, if operating
again on the same platform next time, they are already pre-selected.</p>

<p>In the next step select File → Attach Programmer to start talking to
the device. Once the programmer has started up successfully, the
Device → Programming … menu is enabled which pops up a window to
handle the various persistent device memories (flash, EEPROM,
fuses).</p>

<p>Before anything else can proceed, the signature must be read from
the device on the Signature tab. If the read signature does not
match that of the selected device the entry is marked red and a
candidate device is suggested from the database that matches the
signature.</p>

<h3>Flash, EEPROM</h3>

<p>Flash and EEPROM tabs are laid out similarly.</p>

<p>Internally, all data are stored in a buffer for the correspondig
memory area. The buffer can be filled either by reading it from the
device, or by loading it from a file. For file operations, the desired
file format can be selected using radion button entries. For reading
files, it is possible to use ELF files (that could contain information
for more than one memory area), as well as use an autodection logic.
For writing files, the file format must be selected explicitly, to
either "Intel Hex", "Motorola S-Record", or "Raw binary".</p>

<p>The buffer can be saved to a file, or programmed into the device.</p>

<p>The "Clear" button clears all internally remembered contents of the
buffer, as well as the "cell has been loaded" flags.</p>

<p>On the "Flash" memory tab, the "Erase" button (marked red) can be used
to perform a chip erase. Depending on the fuse selections for the
device, this might also erase the EEPROM memory area.</p>

<h3>Fuses</h3>

<p>The memory tab for fuse memories is laid out similar to the other
tabs, yet on the right-hand side, it contains entry fields for the
individual fuse values. The entries are displayed in hexadecimal
notation. Provided the internal <tt>libavrdude</tt> database contains
configuration information for the selected device, right-clicking on
each entry pops up menues that allow selecting the individual fuse
bitfields based on their purpose. If the respective entry field
already contains a value (either, from reading the device, or loaded
from a file), that value is used to pre-select the respective entries
of the comboboxes. If there was no value present, the default value
(according to the datasheet) is used as a starting point.</p>

<p>Only ELF files can contain values for multiple fuses. All other file
formats can only contain a single fuse value. To allow loading or
saving multiple fuses, a percent sign (<tt>%</tt>) in the file name entry is
considered a pattern that will be replaced by the <tt>fuse</tt><strong>N</strong> name.</p>
</body>
</html>
'''

    def avrlogo(self):
        # source: https://github.com/avrdudes/avrdude/discussions/841
        self.avrlogo = b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00`'+ \
            b'\x00\x00\x00`\x08\x06\x00\x00\x00\xe2\x98w8\x00\x00\x00\tpHY'+ \
            b's\x00\x00\x0b\x12\x00\x00\x0b\x12\x01\xd2\xdd~\xfc\x00\x00\x05iID'+ \
            b'ATx\x9c\xed\x9d\xffU\x1c7\x10\xc7\x87\xbc\xfc\x8fS\x81\x9d\n'+ \
            b'N\xa9\x00w`:\x00wp\xae \xa2\x82\\\x076\x15\x84\x0e\xe0'+ \
            b'*\xc8\\\x05\x86\n\x12* O\xce,\xec-\xd2\xccH\xab\xbd\x81'+ \
            b'0\x9f\xf7\xee\xbdc\x7fh%}5#i\xa4c\x8f\x1e\x1e\x1e\xc0'+ \
            b"\xb1\xe3'\xaf{[\\\x00c\\\x00c\\\x00c~\xd6>>\xc6"+ \
            b'x\x0e\x00_G\x87>\xc7\x18\xbf\xf5\xca~\x8c\xf1\x1d\x00\xdc\x00\xc0'+ \
            b'\x8a\x0e\xed\x00\xe0c\x8c\xf1\x9f\x97\x98\xee$\xef\xcduSc\x01_'+ \
            b'\x85\xbf\xe7\xb2\x1eU\x12\xd0\xf7\xf5\x0bN\xb7K\xdd\xb8\x0b2F='+ \
            b'\x0f\x881j.\xacvK\x19\xf3\xfd?\xa4\x9b\xae;\xd2\xa4\xd7\xdb'+ \
            b'\x02Z\xdc\x92\xe6\x9e\xd7\x96\xae\x1awA\xc6\xb8\x00\xc6\xb8\x00\xc6\xb8'+ \
            b'\x00\xc6\xb8\x00\xc6\x1c\xadV\xab\x08\x00\xbf\xbf\xe9Z\xb0\xe3"\t\xe0'+ \
            b'\x0b\x02\x86\xb8\x0b2\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x05'+ \
            b'0\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6'+ \
            b'\x050\xc6\x050F\xbd3\xae\x92/\x00p\x0b\x00\x7f*n\xdb"'+ \
            b'\xe2\xc7\xe9\xc1\x10B\xda\xd1v\x05\x00\'\x8dy\xd8\x02\xc0)">'+ \
            b'\xdb\x01\x17B\xb8\xc9\xa4{\x89\x88\xe75\x0f\x08!\x0c\xf9\x0e\xe9Y'+ \
            b'-y]b=\xe0\xb1BC\x08i\xcf\xcd\x99p\xfd\x17D\xdc\xe4'+ \
            b'N\x84\x10R\xc1\xfej\xcc\xc7o\x88\x88\x85t\xd3\xce\xb8?\xc6\xc7'+ \
            b'\x10Q\xb5\x8f\x87\x83\xf2\xbb\xa9\x11b\t\x17\x14G\xdf5\x9b\x9e\x8a'+ \
            b'\xdb\x04\xa9\x02\xb7\ry\xd8\x96*_z\xe6\x1c\xf0?R\xe3\xbb\xd4'+ \
            b'&\xd3[\x80\x1d"\xde\x0c\x7f\xd0w\xa9\x02\xdf\x87\x108\xd3\x8f\xcc'+ \
            b'\xb9\xea{\xe8Y\xef\x1b\xd2TC\xaeL%Bo\x01r\xaeDS'+ \
            b'\x81\xc5kH\xc4]E\x1e\xf6\x1aAc~fC"\x88\xf9\xee)'+ \
            b'\xc0\x1d">s9\x9d\xac \xdbG\xd4^{\x88\xd6?Atu'+ \
            b'=GA\\\xcb\xd2tL\xebR\x9f\x91\x84\r!DE\xe5e\x1b'+ \
            b'\x812\x8fYh\xc4T"\x8d\xb0\xbe!\xe2U!\xdf7!\x84\xdd'+ \
            b'd{\xfc\x1e\xbd,\x80-8e\xf0NHc5\x1a\xd6\xe5\x98\xe5'+ \
            b'\xca(\xed\x96\xd6\x7f\xc2|>\xa5\xa1\xb6`\xbdYq\x06z\t\xa0'+ \
            b'q\x11\xb3*\x90\nr\xcf\x9c\xbf\x17\n\xbb\xa4\xef\xe7\xca\xcf\x8d\xc6'+ \
            b'\xba\x08p\xaf\x19n\x92\x85HVpR\xb2\x02\x9aPq\x05\xdd\xe4'+ \
            b"&]\xf0\xd4\xfa['t\x1a\x8ei\x0e\x90\x83\xfd)T\x0f\x01\x8a"+ \
            b'\x05\xcf0\xd7\n6\x05+\xb8\x17\xc49\xc4\xc8\xe7\xb6\xe5\xa6\xb9\x02'+ \
            b'H\x05\x9f"\xb9\x11 +\xf8\x90;AB\xe7\xdc\xcc\x95a\xeb\x07'+ \
            b'\x9a\xf8\x95\x1a!\xd7\xaf\xcd\x16\xa0\xa6\xf5k\xdc\xc8\x00\xd7bs\xe7'+ \
            b'\xb8\xeb\xab\xe2;\r\xec(\x0eT\x82;7;\x16\xf4K\x8d\x00\xf0'+ \
            b'\x14dK\xe6z,\\\xfa+"f\xcdz\x12c*\x06\xd1\xc8\x92'+ \
            b'\xbek\xf2U\x8a\x05\t#3\xe0&}t\xef5w\xff\x9cy\xc0'+ \
            b'%c\xf6\x01\x9eb9{\xa4{B\x08\x1b\xc5\x8e\xec\xc8\xb4\xde\xcd'+ \
            b'H\x80E}\xbf0\xab\x96\x10\xad}\x8e\x0b\x92:Kv\xc4\xa2H'+ \
            b'\xff\x8c\xac\xe5\x19\xa3 ]1\xe8F\xad_\x8a\xc4.\x06Yiq'+ \
            b'\x026\xd0*\xc0%\xe3\x1e\xc20q)\r\xcd\xc8r4\xc1*n'+ \
            b'*\xbf\x16\xce\x1f$\xe63%5\x1ae\x18\xfe\x07\xad\x02p\x85['+ \
            b'\x17\xbe\xd7\xa4\xf1\x98\x16g\x05L\xeb\x7fg\xd1\xfa\xc9\xe7c\xcd\xb3'+ \
            b'[\x04\xe0Z\xff\xd4\xec\xcf\x98!eJC\xb2\x82\xe3\xc6\xd8}\xb7'+ \
            b"x\x7f\xaa\xd4\xd1'\xdb\x18\xe0\xc9\xe5\\\xd7\x86;Z\x04\xa8\rv"+ \
            b'\xd5\x0e)\xa7\x14\xad \x07]\xdbs\xc1\xe5z\xf4\xe1\xfa.6\xe6'+ \
            b'S\xa2V\x80miT@\x05\xcf\x8dyO\x197r\xab\x08U\x1f'+ \
            b'Kc\xe9\tk\xc5\x10\xb7\x15\xce\xa2\xafZV\xefj\x05\x90|\x7f'+ \
            b'\xae\xe0\x92\x1b\x99\x1b\x9exd\x81\xd6_\x9b\x97\xea\x8e\xbfF\x00\xa9'+ \
            b'\xf5\xb3#\x16\xc6\nz,\xd8\x0c\x9c/\xd8\xfa\x078+\xd0\x94e'+ \
            b'\x8f\x1a\x018uO\x85\x82Kn\xa4\x97\x15,\xdd\xfa\x07\xba\x85>'+ \
            b'\xb4\x02\xdcuXg\x9d\xbb\xee\x9b\xac\xa0(\xe2\x81\x97\x1b\xe7\x8e\xee'+ \
            b'\x1e\xd1\n\xd0c\x97A\x8fu\xdf\x974\xf1\x92\xfa\x02)\xea\xfb\x03'+ \
            b'\x8d\x00\xd2:k\x8d\xd9s{\x80\x9a\x17l\x0c\x16\xdbAa\x05\xaa'+ \
            b'0\xbdF\x00i\x9dU\x8cw\x8cXj\xdd\xf7P\xbe\x7fJ\xcb\xe2'+ \
            b'\xd1\x1e\x92\x00\xddw\x19\x08}A\xb5\x1544\x82\x9epV\xa0Z'+ \
            b'\xfb\x90\x04(V\xfe(\xe8VK1HGhLw\xdc\x97\x98\x04'+ \
            b'\xdd4\xcfGD\xb1/\x90\x04\xc8\xba\x0b\x1a\xd3\xd7,EN\xd90'+ \
            b'\xe1\x05\xcd\xac7\xb5\xbcS\xdad\xdbe\xb9\x91b9\xd3c\x9a!'+ \
            b'\xe5YitF\xd6\xc9\xceK\xfc\xbf\xa5\x18\xe3?\xd00\xc6\x050'+ \
            b'\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x05'+ \
            b'0\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6\x050\xc6'+ \
            b'\x050\xc6\x050&\tp\xf1\xa6k\xc0\x96\x0b\xd5\x8b\xdc\xe8}\x8c'+ \
            b'\x7fk\xb2\xaa}\x81\xd9\xe8z\xd5\x92\xe8kK7\xfd\x80Q\xf3\xbe'+ \
            b'J\x95\x0b\xea\xf9\xe2\xcb\xb7\x82\xb6\xce\xbc\x0f0\xc6\x050\xc6\x050'+ \
            b'\xc6\x050\xa6\xb7\x00\x9f\x17\xba\xe7\xb5\xa5\xabf\xd6\xfb\x84k\x87p'+ \
            b'B\xfa\xb9\x17\xca]\xd0\xf1\x17\x97\xee$\xef\xcdu\xe3.\xc8\x98\x1a'+ \
            b'\x01\xa6\xa6\xd7\xd5\x14i\xb3\xef\xf8gJ\xbb\x99\x1b\x80\x97NwL'+ \
            b's\xdd\xa8]\x90\xb3\x0c\xee\x82\x8cq\x01\x8cq\x01\x8cq\x01,\x01'+ \
            b'\x80\x7f\x01\xd4\xa0\x1d\x07\x10\x94\xee\x92\x00\x00\x00\x00IEND\xae'+ \
            b'B`\x82'

def main():
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    gui = adgui(sys.argv)

    global pyside
    if pyside == 6:
        # PySide6 deprecated the exec_ method
        sys.exit(gui.app.exec())
    else:
        sys.exit(gui.app.exec_())

if __name__ == "__main__":
    main()

