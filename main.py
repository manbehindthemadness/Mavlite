"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

try:
    import asyncio
except ImportError:
    import uasyncio as asyncio  # noqa
from micropymavlink.mavlite import test
from micropymavlink.uart import UART


try:
    import board  # noqa
    _uart = UART(tx=board.TX, rx=board.RX, baudrate=115200)
except ImportError:
    _uart = UART(1, baudrate=115200)


asyncio.run(test(_uart))
