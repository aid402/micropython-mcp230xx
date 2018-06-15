'''mcp --> machine.Pin format'''
from machine import Pin,I2C
import mcp

i2c = I2C(scl=Pin(5), sda=Pin(4))
io = mcp.MCP23017(i2c)
pinIRQ = 15         # connect INTA or INTB

class PinX():
    OUT = mcp.OUT
    IN = mcp.IN
    PULL_UP = True
    IRQ_RISING = mcp.RISING
    IRQ_FALLING = mcp.FALLING
    IRQ_CHANGE = mcp.CHANGE

    def __init__(self, id, mode = None, pull = None, value = None):
        self.id = id
        io._init(self.id)
        self.init(mode, pull, value)
        self.pinIRQ = pinIRQ
        self.handler = None

    def init(self,_mode = None, _pull = None, _value = None):
        if mode is not None:
            self.mode(_mode)
            if _mode == PinX.OUT:
                if _value is not None:
                    self.value(_value)
            if _pull is not None:
                self.pull(_pull)

    def value(self, X = None):
        if X is not None:
            io.write(self.id, X)
        else:
            return io.read(self.id)

    def on(self):
        io.write(self.id, 1)

    def off(self):
        io.write(self.id, 0)

    def mode(self, direction = None):
        io.iomode(self.id, direction)

    def pull(self, enabled):
        io.pullup(self.id, enabled)

    def irq(self, handler=None, trigger=PinX.IRQ_RISING):
        if trigger:
            Irq = Pin(self.pinIRQ, Pin.IN)
            Irq.irq(self.interrupted, trigger)
            if handler is not None:
                self.handler = handler
        io.interrupt(self.id, trigger)

    def interrupted(self):
        if io.int_flag(self.id):
            if self.handler is not None:
                self.handler()
