"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

import asyncio
from micropymavlink.mavlite import test


asyncio.run(test())
