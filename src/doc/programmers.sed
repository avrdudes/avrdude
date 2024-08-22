# Transform the avrdude -p \? output into a texinfo table
s/^ *\([^ ]*\) *= \([^(]*\).(/@cindex @code{\1}\n@cindex \2\n@item @code{\1} @tab \2 (/
s/2232hio.based on FT2232H with buffer and LEDs/FT2232H with buffer and LEDs/
s/FT4232H.based generic programmer/FT4232H programmer/
s/ABCmini Board, aka Dick Smith HOTCHIP/ABCmini Board\n@cindex Dick Smith HOTCHIP/
s/Trinket Gemma bootloader disguised as USBtiny/Trinket Gemma bootloader/
s/Nightshade ALF-PgmAVR via PC parallel port/Nightshade ALF-PgmAVR/
s/Arduino bootloader using STK500 v1 protocol/Arduino bootloader/
s/Arduino: FT232R connected to ISP/Arduino: FT232R to ISP/
s/AVR as programmer with Arduino-as-ISP FW/AVR as programmer/
s/Arduino Gemma bootloader disguised as USBtiny/Arduino Gemma bootloader/
s/Arduino-branded USBtiny ISP Programmer/Arduino-branded USBtiny/
s/AT-ISP v1.1 programming cable for AVR-SDK1/AT-ISP v1.1 cable/
s/Atmel Low Cost Serial Programmer/Atmel Low Cost Programmer/
s/FT2232H\/D.based generic programmer/FT2232H\/D programmer/
s/Serial Atmel AVR ISP using STK500/Serial Atmel AVR ISP/
s/Serial Atmel AVR ISP using STK500v2/Serial Atmel AVR ISP/
s/Bascom SAMPLE programming cable/Bascom SAMPLE cable/
s/Brian S. Dean's parallel programmer/Brian S. Dean's programmer/
s/The Bus Pirate in AVR programming mode/The Bus Pirate/
s/The Bus Pirate in bitbang mode/The Bus Pirate/
s/Mikrokopter.de Butterfly bootloader/Mikrokopter.de Butterfly/
s/BitWizard ftdi_atmega builtin programmer/BitWizard ftdi_atmega/
s/serial port: reset=dtr sck=!rts sdo=!txd sdi=!cts/Serial port as programmer/
s/CH341A programmer: note AVR F_CPU > 6.8 MHz/CH341A programmer/
s/Direct AVR Parallel Access cable/Direct AVR Parallel cable/
s/serial port: reset=rts sck=dtr sdo=txd sdi=cts/Serial port as programmer/
s/serial port: reset=!dtr sck=rts sdo=txd sdi=cts/Serial port as programmer/
s/Emulates bootloader programming without the part/Emulating a bootloader (dryboot)/
s/Emulates programming without a programmer/Emulating a HW programmer (dryrun)/
s/AVR ISP programmer from eHaJo.de/AVR ISP programmer/
s/FLIP bootloader using USB DFU v1/FLIP bootloader/
s/FLIP bootloader using USB DFU v2/FLIP bootloader/
s/FT2232H\/D.based generic programmer/FT2232H\/D programmer/
s/FT2232H.based generic JTAG programmer/FT2232H JTAG programmer/
s/FT232H.based generic programmer/FT232H programmer/
s/FT232H.based generic JTAG programmer/FT232H JTAG programmer/
s/FT232R.based generic programmer/FT232R programmer/
s/FT245R.based generic programmer/FT245R programmer/
s/FT4232H.based generic programmer/FT4232H programmer/
s/Futurlec.com programming cable/Futurlec.com cable/
s/AVR ISP programmer from iascaled.com/AVR ISP programmer/
s/Amontec JTAGKey\/JTAGKey-Tiny\/JTAGKey2/Amontec JTAGKey/
s/KT-LINK FT2232H: IO switching, voltage buffers/KT-LINK FT2232H/
s/Use Linux SPI device in \/dev\/spidev\*/Linux \/dev\/spidev\* programmer/
s/Luminary Micro LM3S811 Eval Board/Luminary Micro LM3S811/
s/Crossbow MIB510 programming board/Crossbow MIB510/
s/Jason Kyle's pAVR Serial Programmer/Jason Kyle's pAVR/
s/ponyprog serial: reset=!txd sck=rts sdo=dtr sdi=cts/Serial port programmer/
s/Raspberry Pi GPIO via sysfs\/libgpiod/RPi GPIO programmer/
s/Program via the Serprog protocol from Flashrom/Flashcom serprog protocol/
s/FTDI TTL232R-5V with ICSP adapter/FTDI TTL232R-5V/
s/TIAO USB Multi-Protocol Adapter/TIAO USB programmer/
s/uncompatino with all pairs of pins shorted/Uncompatino programmer/
s/Urboot bootloaders using urprotocol/Urboot bootloader/
s/Any usbasp clone with correct VID\/PID/Usbasp clones/
s/Wiring bootloader using STK500 v2 protocol/Wiring bootloader/
s/XBeeBoot Over-The-Air bootloader/XBeeBoot OTA bootloader/
