
# Usage

**AVRDUDE** is a command-line application.

*Run the `avrdude` command without* <br>
*any arguments for a list of options.*

<br>

---

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

---

<br>

### Arguments

There are many different programmers and options <br>
that may be required for the programming to succeed.

*For this, please refer to* ***[AVRDUDEs Documentation]*** *.*


<!----------------------------------------------------------------------------->

[AVRDUDEs Documentation]: http://download.savannah.gnu.org/releases/avrdude/avrdude-doc-6.4.pdf
