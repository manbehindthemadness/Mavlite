"""
This is a simple test method that sends restart commands.
"""
from mavlite import MavLink, UART
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio  # noqa
try:
    import board  # noqa
    _uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)
except ImportError:
    _uart = UART(tx=17, rx=16, baudrate=230400)


m_id = 246
m = MavLink([m_id, 253])


async def send():
    """Send test command"""
    await m.heartbeat_wait(True)
    await m.send_command(
        command_id=246,
        target_system=0,
        target_component=0,
        params=[1, 0, 0, 0, 0, 0, 0],
        debug=True
    )


async def main(uart_):
    """
    Test mainloop.
    """
    tasks = await m.io_buffers(uart_, debug=True)
    snd_cmd = asyncio.create_task(send())
    await asyncio.gather(snd_cmd, *tasks)

asyncio.run(main(_uart))
