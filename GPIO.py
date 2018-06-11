'''mcp --> machine.Pin format'''
import mcp
io = mcp.MCP23017()
class PinX():
    OUT = mcp.OUT
    IN = mcp.IN
    PULL_UP = True
    IRQ_RISING = mcp.RISING
    IRQ_FALLING = mcp.FALLING
    IRQ_CHANGE = mcp.BOTH

    def __init__(self, id, mode = None, pull = None, value = None):
        self.id = id
        io._init(self.id)
        init(mode, pull, value)

    def init(mode = None, pull = None, value = None):
        if mode is not None:
            self.mode(mode)
            if mode == PinX.OUT:
                if value is not None:
                    self.value(value)
            if pull is not None:
                self.pull(pull)

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
