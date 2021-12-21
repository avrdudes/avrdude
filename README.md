# AVRDUDE

AVRDUDE - AVR Downloader Uploader - is a program for downloading and uploading
the on-chip memories of Microchip’s [AVR microcontrollers](https://en.wikipedia.org/wiki/AVR_microcontrollers).
It can program the Flash and EEPROM, and where supported by the programming
protocol, it can program fuse and lock bits.
AVRDUDE also supplies a direct instruction mode allowing one to issue any
programming instruction to the AVR chip regardless of whether AVRDUDE
implements that specific feature of a particular chip.

AVRDUDE was originally written in 2003 by Brian S. Dean. Since 2006, AVRDUDE has been maintained by Jörg Wunsch,
with the help of [various contributors](./AUTHORS).

The latest version of AVRDUDE is always available here:\
<https://github.com/avrdudes/avrdude>

## Getting AVRDUDE for Windows

To get AVRDUDE for Windows, install the latest version from the [Releases](http://download.savannah.gnu.org/releases/avrdude/) page.

Alternatively, you may [build AVRDUDE](#building-avrdude-for-windows) yourself from source.

## Getting AVRDUDE for Linux

To install AVRDUDE for Linux, install the package `avrdude` by running the following commands:

```console
sudo apt-get install avrdude
```

Alternatively, you may [build AVRDUDE](#building-avrdude-for-linux) yourself from source.

## Getting AVRDUDE for MacOS

On MacOS, AVRDUDE can be installed through Mac Ports.

Alternatively, you may [build AVRDUDE](#building-avrdude-for-macos) yourself from source.

## Using AVRDUDE

AVRDUDE is a command-line application. Run the command `avrdude` without any arguments for a list of options.

A typical command to program your HEX file into your AVR microcontroller looks like this:

```console
avrdude -c <programmer> -p <part> -U flash:w:<file>:i
```

For instance, to program an **Arduino Uno** connected to the serial port **COM1** with a HEX file called `blink.hex`,
you would run the following command:

```console
avrdude -c arduino -P COM1 -b 115200 -p atmega328p -D -U flash:w:objs/blink.hex:i
```

There are many different programmers and options that may be required for the programming to succeed.
For more information, refer to the [AVRDUDE documentation](#todo).

## General build instructions

### Prerequisites

Depending on your requirements, the following prerequisites are
needed:

* libelf including header files (for directly reading ELF files)
* libusb 0.1 or 1.0 (or compatible), including header files
* libftdi or libftdi1 (for direct access to FTDI devices)
* libhidapi or libhid (for access to recent Atmel/Microchip dongles)

### Building

All source code is located in the `src/` subdirectory. Thus all
instructions are relative to that directory.

Source-code releases contain an up-to-date configure script that
can be run to generate the required Makefiles:

```console
cd src && ./configure && make && sudo make install
```
At the end of the configure script, a configuration summary is issued,
like this:

```console
Configuration summary:
----------------------
DO HAVE    libelf
DO HAVE    libusb
DO HAVE    libusb_1_0
DO HAVE    libftdi1
DON'T HAVE libftdi
DON'T HAVE libhid
DO HAVE    libhidapi
DO HAVE    pthread
DISABLED   doc
DISABLED   parport
DISABLED   linuxgpio
DISABLED   linuxspi
```

Make sure all the features you are interested in have been found.

Building the development source tree might possibly require to
re-generate the configure script using the autoconf/automake
tools. This can be done using the `bootstrap` script:

```console
cd src && ./bootstrap
```

## Building AVRDUDE for Windows

### Windows Prerequisites

TODO.

### Windows Build Instructions

TODO.

## Building AVRDUDE for Linux

### Linux Prerequisites

To build AVRDUDE for Linux, you need to install the following packages:

```console
sudo apt-get install build-essential git automake libtool flex bison libelf-dev libusb-dev libftdi1-dev libhidapi-dev
```

To build the documentation, you need to install the following packages:

```console
sudo apt-get install texlive texi2html
```

### Linux Build Instructions

To build AVRDUDE for Linux, run the following commands:

```console
git clone https://github.com/avrdudes/avrdude
cd avrdude
./bootstrap
./configure
make
```

To build the documentation for AVRDUDE, run the following commands:

```console
cd doc
make all
```

## Building AVRDUDE for MacOS

### Prerequisites

The following things are needed to build AVRDUDE on MacOS:

* a C compiler; either full XCode, or the XCode Command Line tools
* autoconf, automake, libtool, hidapi, libftdi1, libusb, libelf;
  they can be installed e.g. from Mac Ports using
```console
port install autoconf automake \
  libtool hidapi libftdi1 libusb libelf
```

### Compilation

Depending on the location of the prerequisites, the `CPPFLAGS` and
`LDFLAGS` variables need to be set accordingly. Mac Ports installs
everything under `/opt/local`, so use

```console
./configure CPPFLAGS=-I/opt/local/include LDFLAGS=-L/opt/local/lib
```

MacOS Brew requires

```console
./configure CPPFLAGS=-I/usr/local/include LDFLAGS=-L/usr/local/Cellar
```

## License

AVRDUDE is licensed under the [GNU GPLv2](./COPYING).
