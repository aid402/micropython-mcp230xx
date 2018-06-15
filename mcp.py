# Credit https://github.com/ShrimpingIt/micropython-mcp230xx
#        https://github.com/adafruit/Adafruit_Python_GPIO/blob/master/Adafruit_GPIO/MCP230xx.py
#        https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
# Author: Suttipong Wongkheaw, Thailand

from machine import Pin, I2C
from time import sleep

OUT     = 0
IN      = 1

RISING  = 1
FALLING = 2
CHANGE  = 3

class MCP():

    def __init__(self, i2c, address=0x20, pinReset=None):
        self.i2c = i2c
        self.address = address
        if pinReset is not None:
            self.pinReset = Pin(pinReset, Pin.OUT, Pin.PULL_UP, value = 1)

        self.gpio_bytes = self.NUM_GPIO//8
        self.iodir = bytearray(self.gpio_bytes)
        self.gppu = bytearray(self.gpio_bytes)
        self.gpio = bytearray(self.gpio_bytes)
        self.ipol = bytearray(self.gpio_bytes)
        self.gpinten = bytearray(self.gpio_bytes)
        self.defval = bytearray(self.gpio_bytes)
        self.intcon = bytearray(self.gpio_bytes)
        self.iocon = bytearray(self.gpio_bytes)
        self.intf = bytearray(self.gpio_bytes)
        self.intcap = bytearray(self.gpio_bytes)
        self.olat = bytearray(self.gpio_bytes)

    def _validate_pin(self, pin):
        if pin < 0 or pin >= self.NUM_GPIO:
            raise ValueError('Invalid GPIO value, must be between 0 and {0}.'.format(self.NUM_GPIO))

    def writeRegister(self, register, data):
        return self.i2c.writeto_mem(self.address, register, data)

    def readRegister(self, register, length):
        return self.i2c.readfrom_mem(self.address, register, length)

    def set_bit(self, bit, value, reg, addr):
        self.set_bits({bit: value}, reg, addr)

    def set_bits(self, bits, reg, addr):
        [self._validate_pin(bit) for bit in bits.keys()]

        for bit, value in iter(bits.items()):
            if value:
                reg[int(pin/8)] |= 1 << (int(pin%8))
            else:
                reg[int(pin/8)] &= ~(1 << (int(pin%8)))
        self.writeRegister(addr, reg)

    def get_bit(self, bit, reg, addr, read=True):
        return self.get_bits([bit], reg, addr, read)[0]

    def get_bits(self, bits, reg, addr, read=True):
        [self._validate_pin(bit) for bit in bits]

        if read:
            reg = self.readRegister(addr, self.gpio_bytes)
        return [(reg[int(bit/8)] & 1 << (int(bit%8))) > 0 for bit in bits]

    def _init(self, pin):
        self._validate_pin(pin)

    def reset(): # Reset mcp
        self.pinReset.value(0)
        sleep(0.5)
        self.pinReset.value(1)

        self.iodir[] = 1
        self.gppu[] = 0
        self.gpio[] = 0
        self.ipol[] = 0
        self.gpinten[] = 0
        self.defval[] = 0
        self.intcon[] = 0
        self.iocon[] = 0
        self.intf[] = 0
        self.intcap[] = 0
        self.olat[] = 0

    def iomode(self, pin, _mode): # Set GPIO direction IN/OUT
        if _mode is not None:
            if _mode in [IN, OUT]:
                self.set_bit(pin, _mode, self.iodir, self.IODIR)
            else:
                raise ValueError('Unexpected value.  Must be IN or OUT.')
            return None
        else:
            return self.get_bit(pin, self.iodir, self.IODIR)

    def pullup(self, pin, enabled=True): # Set GPIO pull_up resistor
        self.set_bit(pin, enabled, self.gppu, self.GPPU)

    def write(self, pin, value): # Set GPIO state HIGH/LOW
        self.set_bit(pin, value, self.gpio, self.GPIO)

    def read(self, pin): # Get GPIO state
        return self.get_bit(pin, self.gpio, self.GPIO)

    def interrupt(self, pin, trigger): # Set Interrupt
        if trigger:
            self.set_bit(pin, (trigger != CHANGE), self.intcon, self.INTCON)
            self.set_bit(pin, (trigger == FALLING), self.defval, self.DEFVAL)
            self.set_bit(pin, 1, self.gpinten, self.GPINTEN)
        else:
            self.set_bit(pin, 0, self.gpinten, self.GPINTEN)

    def int_flag(self, pin):
        return self.get_bit(pin, self.intf, self.INTF)

    def int_captured(self, pin):
        if int_flag(pin):
            return self.get_bit(pin, self.intcap, self.INTCAP)

class MCP23017(MCP):
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
