'''mcp --> machine.Pin format'''
import mcp
io = mcp.MCP23017()
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

    def mode(self, m = None):
        io.iomode(self.id, m)

    def pull(self, p):
        io.pullup(self.id, p)

    def irq(handler=None, trigger=PinX.IRQ_RISING):
        pass
