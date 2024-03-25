#!/usr/bin/env python

import sys
import os
import pathlib
import re

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
    sys.path.append(builddir + '/python')

import swig_avrdude as ad



def avrdude_init():
    ad.init_config()

    found = False
    for d in [builddir, "/etc", "/usr/local/etc"]:
        p = pathlib.Path(d + "/avrdude.conf")
        if p.is_file():
            ad.read_config(d + "/avrdude.conf")
            return (True, f"Found avrdude.conf in {d}\n")

    return (False, "Sorry, no avrdude.conf could be found.\n")

def classify_devices():
    result = {
        'at90': [],
        'attiny': [],
        'atmega': [],
        'atxmega': [],
        'avr_de': [],
        'other': []
    }
    avr_de_re = re.compile('AVR\d+[DE][A-Z]\d+')
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

from PySide2.QtWidgets import *
from PySide2.QtGui import *
from PySide2.QtCore import *
from PySide2.QtUiTools import QUiLoader

class adgui(QObject):
    def __init__(self, argv):
        super().__init__()
        self.logstring = "<font color='#000060'><strong>Welcome to AVRDUDE!</strong></font><br>\n"
        self.app = QApplication(sys.argv)

        p = pathlib.Path(argv[0])
        srcdir = str(p.parent)
        for f in [ "adgui.ui", "about.ui", "device.ui",
                   "devinfo.ui", "loglevel.ui" ]:
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

        self.adgui.show()

        self.adgui.actionAbout.triggered.connect(self.about.show)
        self.adgui.actionDevice.triggered.connect(self.device.show)
        self.adgui.loggingArea.setHtml(self.logstring)

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
            self.devices = classify_devices()
            self.update_device_cb()
            self.device.at90.stateChanged.connect(self.update_device_cb)
            self.device.attiny.stateChanged.connect(self.update_device_cb)
            self.device.atmega.stateChanged.connect(self.update_device_cb)
            self.device.atxmega.stateChanged.connect(self.update_device_cb)
            self.device.avr_de.stateChanged.connect(self.update_device_cb)
            self.device.other.stateChanged.connect(self.update_device_cb)
            self.device.buttonBox.accepted.connect(self.device_selected)
            self.adgui.actionDevice_Info.triggered.connect(self.devinfo.show)

    def log(self, s: str, level: int = ad.MSG_INFO):
        # level to color mapping
        colors = [
            '#804040', # MSG_EXT_ERROR
            '#603030', # MSG_ERROR
            '#605000', # MSG_WARNING
            '#000000', # MSG_INFO
            '#006000', # MSG_NOTICE
            '#005030', # MSG_NOTICE2
            '#808080', # MSG_DEBUG
            '#60A060', # MSG_TRACE
            '#6060A0', # MSG_TRACE2
        ]
        color = colors[level - ad.MSG_EXT_ERROR]
        if level <= ad.MSG_WARNING:
            html = f"<font color={color}><strong>{s}</strong></font><br>\n"
        else:
            html = f"<font color={color}>{s}</font><br>\n"
        self.logstring += html
        self.adgui.loggingArea.setHtml(self.logstring)

    # rough equivalent of avrdude_message2()
    # first argument is either "stdout" or "stderr"
    #
    # install callback with ad.set_msg_callback(msg_callback)
    def msg_callback(self, target: str, lno: int, fname: str, func: str,
                     msgmode: int, msglvl: int, msg: str):
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
            self.log(s, msglvl)

    def update_device_cb(self):
        fams = list(self.devices.keys())
        #fams.sort()
        self.device.devices.clear()
        for f in fams:
            obj = eval('self.device.' + f + '.isChecked()')
            if obj:
                for d in self.devices[f]:
                    self.device.devices.addItem(d)

    def update_device_info(self):
        p = ad.locate_part(ad.cvar.part_list, self.dev_selected)
        if not p:
            log(f"Could not find {self.dev_selected} again, confused\n")
            return
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

    def device_selected(self):
        self.dev_selected = self.device.devices.currentText()
        self.log(f"Selected device: {self.dev_selected}")
        self.update_device_info()
        self.adgui.actionDevice_Info.setEnabled(True)

    def loglevel_changed(self, checked: bool):
        btn = self.sender()
        if checked:
            # we abuse the tooltip for the verbosity value
            val = int(btn.toolTip())
            ad.cvar.verbose = val


def main():
    gui = adgui(sys.argv)

    sys.exit(gui.app.exec_())

if __name__ == "__main__":
    main()
