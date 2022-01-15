
# AVRDUDE [![Build Badge]][Build Status]

***AVR*** ***D*** *ownloader* ***U*** *ploa* ***DE*** *er*

This is a program for **Downloading** and **Uploading** <br>
on-chip memories of **[Microchip’s AVR Microcontrollers]**.

---

**⸢ [Installation] ⸥ ⸢ [Usage] ⸥ ⸢ [Wiki] ⸥ ⸢ [Documentation] ⸥**

---

## Features

- Can program `Flash` & `EEPROM`

- Is able to program `Fuse` & `Lock` bits.

- **Direct Instruction Mode**

  ***AVR*** *chips can be issues any programming* <br>
  *instructions regardless of* ***AVRDUDEs*** *support* <br>
  *for that specific feature of the particular chip.*

---

## History

#### 2003

**AVRDUDE** was originally written by <kbd>Brian S. Dean</kbd> .

#### 2006

The project is being maintained by <kbd>**[Jörg Wunsch]**</kbd> , <br>
as well the various **[Authors]** / **[Contributors]** .

---


## Installation

In the following you will find system specific installation <br>
instructions, however you can always **[Build It Yourself]**.

<br>

### Windows

**AVRDUDE** can be downloaded from the **[Releases]** page.

<br>

### Linux

**AVRDUDE** can be installed by adding <br>
the `avrdude` package to your system.

##### Using APT

```sh
sudo apt install avrdude
```

<br>

### MacOS

**AVRDUDE** can be installed via `Mac Ports`.

---

## Usage

**AVRDUDE** is a command-line application.

*Run the `avrdude` command without* <br>
*any arguments for a list of options.*

<br>

### Example

A typical command to program your **HEX** file <br>
into your `AVR Microcontroller` looks like this:

```sh
avrdude -c <Programmer> -p <Part> -U flash:w:<File>:i
```

<br>

##### Arduino Uno

For instance, to program an `Arduino Uno` <br>
connected to the serial port `COM1` with a <br>
**HEX** file called `blink.hex`, you would use:

```sh
avrdude -c arduino -P COM1 -b 115200 -p atmega328p -D -U flash:w:objs/blink.hex:i
```

<br>

### Arguments

There are many different programmers and options <br>
that may be required for the programming to succeed.

*For this, please refer to* ***[AVRDUDEs Documentation][Documentation]*** *.*


<!----------------------------------------------------------------------------->

[Installation]: docs/Installation.md
[Usage]: docs/Usage.md
[Wiki]: https://github.com/avrdudes/avrdude/wiki
[Documentation]: http://download.savannah.gnu.org/releases/avrdude/avrdude-doc-6.4.pdf


[Authors]: AUTHORS
[Contributors]: https://github.com/avrdudes/avrdude/graphs/contributors
[Jörg Wunsch]: https://github.com/dl8dtl

[Releases]: http://download.savannah.gnu.org/releases/avrdude/
[Build It Yourself]: https://github.com/avrdudes/avrdude/wiki


[Microchip’s AVR Microcontrollers]: https://en.wikipedia.org/wiki/AVR_microcontrollers

[Build Badge]: https://github.com/avrdudes/avrdude/actions/workflows/build.yml/badge.svg

[Build Status]: https://github.com/avrdudes/avrdude/actions/workflows/build.yml
