#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# this is a small python sketch that tries to find the PICkit5_TP folder
# created by MPLAB X that contains the scripts.xml file.
# Works on Windows only, until someone (probably me) adds an OS recognition
# (All that is needed for that is to find out where the scripts.xml lies on Linux)   
# The scripts.xml contains an assmbler-like syntax that tells the Debugger what to do
# That file is pretty big, so first, we pull out all commands that end with UPDI
# and create a smaller, cached XML file
# From that XML file we extract all useful information and create an equivalent
# in python using Dictionaries.
# From this Dictionaries we group together the same byte sequences and replace them with "links"
# shring down the whole ordeal to a managable size.
# This program creates 3 different files because it's easier to debug this way
# P.S. Opening the original scripts.xml in ElementTree takes about 6GB of RAM, so... 16GB RAM recomended

import os, fnmatch, mmap
from pathlib import Path
from xml.etree import ElementTree as ET

dict_header = \
'''
func_dict =  {
    "EnterProgMode" : [],
    "EnterProgModeHvSp" : [],
    "EnterProgModeHvSpRst" : [],
    "EnterProgModeHvUpt" : [],
    "ExitProgMode" : [],
    "SetSpeed" : [],
    "GetDeviceID" : [],
    "EraseChip" : [],
    "WriteProgmem" : [],
    "ReadProgmem" : [],
    "WriteDataEEmem" : [],
    "ReadDataEEmem" : [],
    "WriteConfigmem" : [],
    "WriteConfigmemFuse" : [],
    "WriteConfigmemLock" : [],
    "ReadConfigmem" : [],
    "ReadConfigmemFuse" : [],
    "ReadConfigmemLock" : [],
    "WriteIDmem" : [],
    "ReadIDmem" : [],
    "WriteCSreg" : [],
    "ReadCSreg" : [],
    "WriteMem8" : [],
    "WriteMem16" : [],
    "ReadMem8" : [],
    "ReadMem16" : [],
    "ReadSIB" : [],
    "HoldInReset" : [],
    "ReleaseFromReset" : [],
    "EnterDebugMode" : [],
    "EnterDebugModeHvSp" : [],
    "EnterDebugModeHvSpRst" : [],
    "EnterDebugModeHvUpt" : [],
    "ExitDebugMode" : [],
    "SetPC" : [],
    "GetPC" : [],
    "Run" : [],
    "Halt" : [],
    "DebugReset" : [],
    "GetHaltStatus" : [],
    "SingleStep" : [],
    "SetHWBP" : [],
    "ClearHWBP" : [],
}\n
'''


import platform

work_dir = os.path.abspath(os.getcwd())
cache_dir = os.path.join(work_dir, "scripts_cache")

print(work_dir)
print(cache_dir)

def find_xml():
    home_dir = str(Path.home())
    print("Home Path: {0}".format(home_dir))
    if home_dir == None:
        return
    home_dir = os.path.join(home_dir, ".mchp_packs", "Microchip")
    home_dir_A = os.path.join(home_dir, "PICkit5_TP")
    result = []
    for root, dirs, files in os.walk(home_dir_A):
        for name in files:
            if fnmatch.fnmatch(name, "scripts.xml"):
                result.append(os.path.join(root, name))
    
    # home_dir_A = os.path.join(home_dir, "PICkit4_TP")     # maybe in the future, if needed
    print("List of scripts.xml files:")
    print(result)
    return result[-1]
    # EOF

def cache_xml(file):
    if file == None:
        return
    print("Opening file {0}".format(file))
    global cache_dir
    origin_tree = ET.parse(file)
    origin_root = origin_tree.getroot()
    work_root = ET.Element("scripts")
    counter = 0
    old_chip_name = ""
    print("List of UPDI MCUs:")
    for script in origin_root.findall('script'):
        function_name = script[0].text      # the function name is always on the first place
        if (function_name.endswith("UPDI")):
            chip_name = script[1].text
            if (old_chip_name != chip_name):
                print(chip_name)
                old_chip_name = chip_name
            work_root.append(script)         # copy UPDI entries to our working element
        counter += 1
        #if counter > 25:
        #    return
    
    work_tree = ET.ElementTree(work_root)
    work_tree.write(os.path.join(cache_dir, "scripts_updi.xml"))
    # EOF


def decode_xml_cache(xml_path):
    global cache_dir
    dict_path = os.path.join(cache_dir, "scripts_dict.py")

    xml_tree = ET.parse(xml_path) 
    xml_root = xml_tree.getroot()

    if (os.path.exists(dict_path)):
        os.remove(dict_path)

    with open(dict_path, 'w') as dict_file:
        dict_list = []
        dict_iterator = -1
        old_chip_name = ""
        old_function_name = ""
        for script in xml_root:
            function_string = ""
            function_name = script[0].text
            chip_name = script[1].text
            if (old_chip_name != chip_name):
                if (dict_iterator >= 0):
                    dict_list[dict_iterator] += "    },\n"   # trailing bracket
                    print("{0} generated".format(old_chip_name))

                dict_iterator += 1
                dict_list.append("")
                dict_list[dict_iterator] = "   \"{0}\" : ".format(chip_name)  # thisdict : {
                dict_list[dict_iterator] += "{\n"
                old_chip_name = chip_name

            if (old_function_name != function_name):
                dict_list[dict_iterator] += "       \"{0}\" : ".format(function_name[0:-5]) #    "function_name" : 
                old_function_name = function_name

            scrbytes = script[3]
            for bytes in scrbytes:
                function_string += bytes.text
                function_string += ", "
            
            dict_list[dict_iterator] += "[{0}],\n".format(function_string[0:-2])    # "function string", 
        
        dict_file.write(dict_header)
        dict_file.write("scripts = {\n")
        for x in range (dict_iterator):
            dict_file.write(dict_list[x])   # store decoded dictionary
        dict_file.write("}")
       

import scripts_cache.scripts_dict as dict


def optimize_dict():
    global cache_dir
    for chip_name in dict.scripts:                  # Go through every chip
        for func_name in dict.scripts[chip_name]:   # Go through every function for each chip
            if func_name in dict.func_dict:         # Only handle the function if we care
                func_array = dict.func_dict[func_name]          # Use the array
                position = -1
                for x in range (len(func_array)):               # go through
                    if func_array[x] == dict.scripts[chip_name][func_name]:
                        position = x
                        break
                if position >= 0:
                    dict.scripts[chip_name][func_name] = position
                else:
                    func_array.append(dict.scripts[chip_name][func_name])
                    dict.scripts[chip_name][func_name] = position + 1
    
    lut_path = os.path.join(cache_dir, "scripts_lut.py")
    if (os.path.exists(lut_path)):
        os.remove(lut_path)
    
    with open(lut_path, 'w') as lut_file:                   # Create file
        lut_file.write("func_dict = {\n")                   # start with function look-up Table
        for func in dict.func_dict:
            lut_file.write("    \"{0}\" : [\n".format(func))    # function start
            func_array = dict.func_dict[func]
            
            for array_elem in func_array:                       # function list start
                func_string = "        ["                               
                for i in array_elem:                            # iterate through the sub-function elements
                    func_string += "0x{0:02X}, ".format(i)      # format for readability
                func_string += "],\n"                           # end of sub-function
                lut_file.write(func_string)                     # write to file
            lut_file.write("    ],\n")                          # end of function
        lut_file.write("}\n\n\n")                               # end of lut

        lut_file.write("scripts = {\n")
        for chip_name in dict.scripts:                  # Go through every chip
            lut_file.write("    \""+ chip_name + "\" : {\n")
            for func_name in dict.scripts[chip_name]:   # Go through every function for each chip
                lut_file.write("        \"{0}\" : {1},\n".format(func_name, dict.scripts[chip_name][func_name]))
            lut_file.write("    },\n")
        lut_file.write("}")                             # close dict
        print("done")
    #EOF


import scripts_cache.scripts_lut as lut

def generate_h_file(c_dict):
    h_header = \
'''
/* This file was auto-generated by scripts_decoder.py, changes may be overwritten */
#ifndef scripts_lut_h
#define scripts_lut_h

#ifdef __cplusplus
  extern "C" {
#endif

struct avr_script_lut {
'''

    h_trailer = \
'''
};

struct avr_script_lut* get_pickit_script(const char *partdesc);

#ifdef __cplusplus
}
#endif

#endif // scripts_lut_h
'''

    global cache_dir
    if cache_dir is None:
        return

    h_lut_path = os.path.join(cache_dir, "scripts_lut.h")  # first - handle defining the structure
    if (os.path.exists(h_lut_path)):
        os.remove(h_lut_path)
    with open(h_lut_path, 'w') as h_file:
        h_file.write(h_header)
        for func_name in c_dict:
            h_file.write("  const unsigned char *{0};\n         unsigned int  {0}_len;\n".format(func_name))
        h_file.write(h_trailer)
        print("h-File generated")
    #EOF




def generate_c_file(c_dict):
    global cache_dir
    if cache_dir is None:
        return
    
    c_lut_path = os.path.join(cache_dir, "scripts_lut.c")
    if (os.path.exists(c_lut_path)):
        os.remove(c_lut_path)
    with open(c_lut_path, 'w') as c_file:
        non_unique_func = []
        c_file.write("/* this file was auto-generated */\n")
        c_file.write("#include <ac_cfg.h>\n")
        c_file.write("#include <stddef.h>\n")
        c_file.write("#include <string.h>\n")
        c_file.write("#include \"scripts_lut.h\"\n\n\n")
        struct_init_func = ""
        struct_init_len = ""
        for func_name in lut.func_dict:                             # for each function in our database
            if func_name in c_dict:                                 # if the function exists in our c-list
                function = lut.func_dict[func_name]                 # load data associated with the function
                array_iterator = 0
                for array in function:                              # for each array in function
                    array_str  = "const unsigned char {0}_{1}[{2}]".format(func_name, array_iterator, len(array))
                    array_str += " = {\n  "                         # begin array
                    for i in range (len(array)):                    # go through every byte
                        array_str += "0x{0:02x}, ".format(array[i]) # and generate String
                    array_str += "\n};\n"                           # complete array
                    array_iterator += 1
                    c_file.write(array_str)

                if array_iterator == 1:
                    struct_init_func += "  .{0} = {0}_0,\n".format(func_name)
                    struct_init_len  += "  .{0}_len = sizeof({0}_0),\n".format(func_name)
                else:
                    struct_init_func += "  .{0} = NULL,\n".format(func_name)
                    struct_init_len  += "  .{0}_len = 0,\n".format(func_name)
                    non_unique_func.append(func_name)


        c_file.write("\n\n\nstruct avr_script_lut avr_scripts = {\n")
        c_file.write(struct_init_func)
        c_file.write(struct_init_len)
        c_file.write("};\n\n\n")

        chip_lut_str = "char *chip_lut[] = {\n  "
        chip_name_iterator = 0
        for chip_name in lut.scripts:
            chip_lut_str += "\""
            chip_lut_str += chip_name
            chip_lut_str += "\", "
            chip_name_iterator += 1
            if chip_name_iterator % 8 == 0:
                chip_lut_str += "\n  "
        chip_lut_str += "\n};\n\n\n"
        c_file.write(chip_lut_str)

        c_func_str = "struct avr_script_lut* get_pickit_script(const char* partdesc) { \n"
        c_func_str += "  int namepos = -1;\n"
        c_func_str += "  for (int i = 0; i < {0}; i++)".format(chip_name_iterator)
        c_func_str += " {\n"
        c_func_str += "    if (strncmp(chip_lut[i], partdesc, 10) == 0) {\n"
        c_func_str += "      namepos = i;\n      break;\n    }\n  }\n"
        c_func_str += "  if (namepos == -1) {\n    return NULL;\n  }\n"
        c_file.write(c_func_str)

        switch_iterator = 0
        c_file.write("  switch (namepos) {\n")
        for chip_name in lut.scripts:
            case_str = "    case {0}:\n".format(switch_iterator)
            chip_func = lut.scripts[chip_name]
            for func_lut in chip_func:
                if func_lut in non_unique_func:
                    case_str += "      avr_scripts.{0} = {0}_{1};\n".format(func_lut, chip_func[func_lut])
                    case_str += "      avr_scripts.{0}_len = sizeof({0}_{1});\n".format(func_lut, chip_func[func_lut])
            case_str += "      break;\n"
            switch_iterator += 1
            c_file.write(case_str)
        c_file.write("    }\n    return &avr_scripts;\n  }")
    print("c-File generated")




def convert_to_c():
    c_dict = {                          # for avrdude, that's all the functions we'll need, no need to overdo it...
        "EnterProgMode" : [],
        "EnterProgModeHvSp" : [],
        "EnterProgModeHvSpRst" : [],
        "EnterProgModeHvUpt" : [],
        "ExitProgMode" : [],
        "SetSpeed" : [],
        "GetDeviceID" : [],
        "EraseChip" : [],
        "ReadProgmem" : [],
        "WriteDataEEmem" : [],
        "ReadDataEEmem" : [],
        "WriteCSreg" : [],
        "ReadCSreg" : [],
        "WriteMem8" : [],
        "WriteMem16" : [],
        "ReadMem8" : [],
        "ReadMem16" : [],
        "ReadSIB" : [],
        "HoldInReset" : [],
        "ReleaseFromReset" : [],
    }
    generate_h_file(c_dict)
    generate_c_file(c_dict)
    #EOF

while True:
    user_in = input(">")

    if (user_in == "cache"):
        xml_path = find_xml()
        cache_xml(xml_path)
    elif (user_in == "decode"):
        xml_path = os.path.join(cache_dir, "scripts_updi.xml")
        decode_xml_cache(xml_path)
    elif (user_in == "optimize"):
        optimize_dict()
    elif (user_in == "cfy"):
        convert_to_c()
    elif (user_in == "dude"):
        pass

