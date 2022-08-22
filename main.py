"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

try:
    import asyncio
except ImportError:
    import uasyncio as asyncio  # noqa
from src.mavlite import test
from src.uart import UART


try:
    import board  # noqa
    _uart = UART(tx=board.TX, rx=board.RX, baudrate=115200)
except ImportError:
    _uart = UART(tx=17, rx=16, baudrate=115200)


asyncio.run(test(_uart))
