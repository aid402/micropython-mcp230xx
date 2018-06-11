# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola, ported for Micropython ESP8266 by Cefn Hoile
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from machine import Pin, I2C
from time import sleep

OUT     = 0
IN      = 1
HIGH    = True
LOW     = False

RISING  = 1
FALLING = 2
BOTH    = 3

PUD_OFF = 0
PUD_DOWN= 1
PUD_UP  = 2

class MCP():
    """Base class to represent an MCP230xx series GPIO extender.  Is compatible
    with the Adafruit_GPIO BaseGPIO class so it can be used as a custom GPIO
    class for interacting with device.
    """

    def __init__(self, address=0x20, gpioScl=5, gpioSda=4, pinReset=0):
        """Initialize MCP230xx at specified I2C address and bus number.  If bus
        is not specified it will default to the appropriate platform detected bus.
        """
        self.address = address
        self.pinReset = Pin(pinReset, Pin.OUT, Pin.PULL_UP, value = 1)
        self.i2c = I2C(scl=Pin(gpioScl),sda=Pin(gpioSda))
        # Assume starting in ICON.BANK = 0 mode (sequential access).
        # Compute how many bytes are needed to store count of GPIO.
        self.gpio_bytes = self.NUM_GPIO//8
        # Buffer register values so they can be changed without reading.
        self.iodir = bytearray(self.gpio_bytes)  # Default direction to all inputs.
        self.gppu = bytearray(self.gpio_bytes)  # Default to pullups disabled.
        self.gpio = bytearray(self.gpio_bytes)  # Write current direction and pullup buffer state.
        self.ipol = bytearray(self.gpio_bytes)
        self.gpinten = bytearray(self.gpio_bytes)
        self.defval = bytearray(self.gpio_bytes)
        self.intcon = bytearray(self.gpio_bytes)
        self.iocon = bytearray(self.gpio_bytes)
        self.intf = bytearray(self.gpio_bytes)
        self.intcap = bytearray(self.gpio_bytes)
        self.olat = bytearray(self.gpio_bytes)

    def _validate_pin(self, pin):
        # Raise an exception if pin is outside the range of allowed values.
        if pin < 0 or pin >= self.NUM_GPIO:
            raise ValueError('Invalid GPIO value, must be between 0 and {0}.'.format(self.NUM_GPIO))

    def writeList(self, register, data):
        return self.i2c.writeto_mem(self.address, register, data)

    def readList(self, register, length):
        return self.i2c.readfrom_mem(self.address, register, length)

    def write_bit(self, bit, value, reg, addr):
        """Set the specified pin the provided high/low value.  Value should be
        either HIGH/LOW or a boolean (True = HIGH).
        """
        self.write_bits({bit: value}, reg, addr)

    def write_bits(self, bits, reg, addr):
        """Set multiple pins high or low at once.  Pins should be a dict of pin
        name to pin value (HIGH/True for 1, LOW/False for 0).  All provided pins
        will be set to the given values.
        """
        [self._validate_pin(bit) for bit in bits.keys()]

        for bit, value in iter(bits.items()):
            if value:
                reg[int(pin/8)] |= 1 << (int(pin%8))
            else:
                reg[int(pin/8)] &= ~(1 << (int(pin%8)))
        self.writeList(addr, reg)

    def read_bit(self, bit, reg, addr, read=True):
        """Read the specified pin and return HIGH/True if the pin is pulled
        high, or LOW/False if pulled low.
        """
        return self.read_bits([bit], reg, addr, read)[0]

    def read_bits(self, bits, reg, addr, read=True):
        """Read multiple pins specified in the given list and return list of pin values
        HIGH/True if the pin is pulled high, or LOW/False if pulled low.
        """
        [self._validate_pin(bit) for bit in bits]

        if read:
            reg = self.readList(addr, self.gpio_bytes)
        return [(reg[int(bit/8)] & 1 << (int(bit%8))) > 0 for bit in bits]

    def _init(self, pin):
        self._validate_pin(pin)

    def reset():
        self.pinReset.value(0)
        sleep(0.5)
        self.pinReset.value(1)

    def iomode(self, pin, _mode):
        if _mode is not None:
            if _mode == IN or _mode == OUT:
                write_bit(pin, _mode, self.iodir, self.IODIR)
            else:
                raise ValueError('Unexpected value.  Must be IN or OUT.')
        else:
            return read_bit(pin, self.iodir, self.IODIR)

    def pullup(self, pin, enabled):
        write_bit(pin, enabled, self.gppu, self.GPPU)

    def write(self, pin, value):
        write_bit(pin, value, self.gpio, self.GPIO)

    def read(self, pin):
        return read_bit(pin, self.gpio, self.GPIO)

    def

class MCP23017(MCP):
    """MCP23017-based GPIO class with 16 GPIO pins."""
    # Define number of pins and registor addresses.
    NUM_GPIO = 16
    IODIR    = 0x00
    IPOL     = 0x02
    GPINTEN  = 0x04
    DEFVAL   = 0x06
    INTCON   = 0x08
    IOCON    = 0x0A
    GPPU     = 0x0C
    INTF     = 0x0E
    INTCAP   = 0x10
    GPIO     = 0x12
    OLAT     = 0x14

class MCP23008(MCP):
    """MCP23008-based GPIO class with 8 GPIO pins."""
    # Define number of pins and registor addresses.
    NUM_GPIO = 8
    IODIR    = 0x00
    IPOL     = 0x01
    GPINTEN  = 0x02
    DEFVAL   = 0x03
    INTCON   = 0x04
    IOCON    = 0x05
    GPPU     = 0x06
    INTF     = 0x07
    INTCAP   = 0x08
    GPIO     = 0x09
    OLAT     = 0x0A
