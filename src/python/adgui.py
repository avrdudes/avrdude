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

from PySide2.QtWidgets import *
from PySide2.QtCore import QFile, QIODevice

from ui_adgui import Ui_MainWindow
from ui_about import Ui_About
from ui_device import Ui_Device

class About(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_About()
        self.ui.setupUi(self)

class Device(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Device()
        self.ui.setupUi(self)

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

class adgui():
    def __init__(self, argv):
        self.logstring = "Welcome to AVRDUDE!\n"
        self.app = QApplication(sys.argv)

        self.window = MainWindow()
        self.window.show()
        self.about = About()
        self.device = Device()

        self.window.ui.actionAbout.triggered.connect(self.about.show)
        self.window.ui.actionDevice.triggered.connect(self.device.show)
        self.window.ui.loggingArea.setPlainText(self.logstring)

        (success, message) = avrdude_init()
        self.initialized = success
        self.log(message)
        if not success:
            self.window.ui.actionDevice.setEnabled(False)
            self.window.ui.actionProgrammer.setEnabled(False)
            # essentially, only Exit and Help work anymore
        else:
            self.devices = classify_devices()
            self.update_device_cb()
            self.device.ui.at90.stateChanged.connect(self.update_device_cb)
            self.device.ui.attiny.stateChanged.connect(self.update_device_cb)
            self.device.ui.atmega.stateChanged.connect(self.update_device_cb)
            self.device.ui.atxmega.stateChanged.connect(self.update_device_cb)
            self.device.ui.avr_de.stateChanged.connect(self.update_device_cb)
            self.device.ui.other.stateChanged.connect(self.update_device_cb)
            self.device.ui.buttonBox.accepted.connect(self.device_selected)

    def log(self, s: str):
        self.logstring += s
        self.window.ui.loggingArea.setPlainText(self.logstring)

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
            self.log(s)

    def update_device_cb(self):
        fams = list(self.devices.keys())
        #fams.sort()
        self.device.ui.devices.clear()
        for f in fams:
            obj = eval('self.device.ui.' + f + '.isChecked()')
            if obj:
                for d in self.devices[f]:
                    self.device.ui.devices.addItem(d)

    def device_selected(self):
        self.dev_selected = self.device.ui.devices.currentText()
        self.log(f"Selected device: {self.dev_selected}")
        self.window.ui.actionDevice_Memories.setEnabled(True)


def main():
    gui = adgui(sys.argv)

    sys.exit(gui.app.exec_())

if __name__ == "__main__":
    main()
