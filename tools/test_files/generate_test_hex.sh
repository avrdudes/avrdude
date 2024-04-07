#!/bin/bash

# published under GNU General Public License, version 3 (GPL-3.0)
# authors Hans Eirik Bull and Stefan Rueger, 2024

flashsizes=(512 1024 2048 4096 8192 10240 16384 20480 32768 36864 40960 49152 65536 69632 131072 139264 204800 262144 270336 401408 524288)
eepromsizes=(64 128 256 512 1024 2048 4096 8192)
usersigsizes=(32 64 128 256 512 768)

###
# Files for EEPROM testing
#
for i in ${eepromsizes[@]}; do
  srec_cat -generate 0x00 $i -repeat-string "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG. " -o the_quick_brown_fox_${i}B.hex -Intel
  # An srec file
  srec_cat -generate 0x00 $i -repeat-string "LOREM IPSUM DOLOR SIT AMET. " -o lorem_ipsum_${i}B.srec
  echo "S9030000FC" >> lorem_ipsum_${i}B.srec
  # Difficult files that start at i/8+2, end at i-i/8-2 and remove the space from i/4 to i-i/4 as a hole
  srec_cat -generate $((i/8+2)) $((i-i/8-2)) -repeat-string "PACK MY BOX WITH FIVE DOZEN LIQUOR JUGS" -exclude $((i/4)) $((i-i/4)) -o holes_pack_my_box_${i}B.hex -Intel
  srec_cat -generate $((i/8+2)) $((i-i/8-2)) -repeat-string "THE FIVE BOXING WIZARDS JUMP QUICKLY" -exclude $((i/4)) $((i-i/4)) -o holes_the_five_boxing_wizards_${i}B.hex -Intel
  # A partial file to spot check a chip erase
  srec_cat -generate $((i/8+2)) $((i-i/8-2)) -repeat-data 0xff -exclude $((i/4)) $((i-i/4)) -o holes_eeprom_0xff_${i}B.hex -Intel
done

###
# Files suitable for flash containing code with endless loops
#
for i in ${flashsizes[@]}; do
  # Almost full flash leaving space for bootloaders either end
  srec_cat -generate $((i/4)) $((i-i/4)) -repeat-data $(for i in {255..0}; do printf "0x%02x 0xcf " $i; done) \
    -o rjmp_loops_for_bootloaders_${i}B.hex -Intel
  # A "difficult" sketch file with two code blocks and a data block of one byte with holes in between
  srec_cat -generate $((i/4)) $((i/3)) -repeat-data $(for i in {255..0}; do printf "0x%02x 0xcf " $i; done) \
    -generate $((i-i/3)) $((i-i/4-2)) -repeat-data $(for i in {255..0}; do printf "0x%02x 0xcf " $i; done) \
    -generate $((i-i/4-1)) $((i-i/4)) -repeat-data 0xcf \
    -o holes_rjmp_loops_${i}B.hex -Intel
  # A partial file to spot check a chip erase
  srec_cat -generate $((i/4)) $((i/3)) -repeat-data 0xff \
    -generate $((i-i/3)) $((i-i/4-2)) -repeat-data 0xff \
    -generate $((i-i/4-1)) $((i-i/4)) -repeat-data 0xff \
    -o holes_flash_0xff_${i}B.hex -Intel
done

###
# Files for usersig testing
#
for i in ${usersigsizes[@]}; do
  # Random data
  dd if=/dev/urandom of=random_data_${i}B.bin bs=$i count=1 >& /dev/null
  # Empty memory
  srec_cat -generate 0x00 $i -repeat-data 0xff -o 0xff_${i}B.hex -Intel
done
