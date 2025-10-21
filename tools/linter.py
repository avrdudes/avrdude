#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# avrdude - A Downloader/Uploader for AVR device programmers
# Copyright (C) 2025 MX682X
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

# this python skript provides a basic linting of the recently
# modified files (1 week). The rules are adjusted for avrdude specifically.
# The results are ot absolute, it just shows where to check things


import os
import pathlib
import time

# List of words that don't fllow normal writing conventions
c_keywords = ("if", "else", "do", "while", "for", "void", "unsigned", "char", "int", "return", "switch", "case", "default", )
other_keywords = ("debugWIRE", "debugWire", "UPDI", "MPLAB", "AVR", "PICkit")
chip_keywords = ("xMega", "CS", "RC", "HV")

def get_file_list(path):
    return [f.path for f in os.scandir(path)
                if f.name.endswith((".c", ".h"))    # limit c and h files, 1 week old modification
                if os.path.getmtime(f.path) > (time.time() - 7*24*3600)]

def check_slc(line, linenum, pos):
    if line[0] != " " and line[0] != "/":
        print(f"{linenum}:{pos+1} - missing space after '//'")
    found_printable = False
    for i in range(len(line)):
        if line[i:i+2] == "//" and i > 2:
            check_slc(line[i+2:], linenum, i + pos)
        elif line[i] == " ":
            continue
        elif not found_printable:
            if line[i].isprintable():
                found_printable = True
                if line[i].islower():
                    word = line[i:-1].split(" ", 1)[0]
                    if not (word in c_keywords or word in other_keywords):
                        if not any (x in word for x in ("->", ".", "_")):
                            print(f"{linenum+1}:{i+pos+1} - lower case '{word}' at beginning of '//'")
        else:
            continue

def check_console_output(line, linenumber):
    try:
        type, string = line.split("(")
        word = string.split('"')[1].split(" ")[0]
        if word[0].isupper():       # First letter is capital
            if not word.isupper():  # Whole word capital is allowed, e.g. NOT
                if not (word in other_keywords or word in chip_keywords):  # reduce false positives 
                    print(f"{linenumber+1} - {type} messages should start lower case: '{word}'")
    except:
        pass

def check_lines(lines):
    mlc = 0 # multi-line comment
    string = 0
    for l, line in enumerate(lines):
        for i in range(len(line)):
            if mlc:
                if line [i:i+2] == "*/":    # ignore everything in multi-line comment
                    mlc = 0
                continue

            elif string:                    # ignore C strings
                if line[i] == '"' and line[i-1] != "\\":
                    string = False

            elif line [i] == "#":
                break   # skip preprocessor line 

            elif line [i:i+2] == "/*":
                mlc = 1

            elif line[i:i+2] == "//":
                check_slc(line[i+2:], l, 2)
                break   # skip the rest of the single line comment

            elif line[i] == '"':
                string = True

            elif line[i:i+4] == "if (":
                print(f"{l+1}:{i+3} - found if space")
            elif line[i:i+7] == "while (":
                print(f"{l+1}:{i+6} - found while space")
            elif line[i:i+5] == "for (":
                print(f"{l+1}:{i+4} - found for space at")
            
            elif line[i:i+4] == "char":
                if line[i+4] == "*":
                    print(f"{l+1}:{i+5} - missing space between char and * ")
                elif line[i+4:i+6] == " *":
                    if line[i+6] == " ":
                        if line[i+7:i+12] != "const":
                            print(f"{l+1}:{i+7} - space found between * and function/variable name")
            
            elif line[i:i+3] == "int":
                if line[i+3] == "*":
                    print(f"{l+1}:{i+4} - missing space between int and * ")
                elif line[i+3:i+5] == " *":
                    if line[i+5] == " ":
                        if line[i+6:i+11] != "const":
                            print(f"{l+1}:{i+6} - space found between * and function/variable name")
            
            elif line[i:i+4] == "msg_":  # wildcarding pmsg/imsg, might pick up others though, careful there
                check_console_output(line[i+4:], l)
        if len(line) > 1 and line[-2:] == " \n":
            print(f"{l+1} - Trailing whitespace found")


def main():
    working_dir = pathlib.Path(__file__).parent.parent.resolve()
    working_dir = os.path.join(working_dir, "src")
    print("working dir:", working_dir)
    files = get_file_list(working_dir)

    for file in files:
        print(file)
        with open(file, "r") as f:
            lines = f.readlines()
            check_lines(lines)

main()