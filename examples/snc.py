"""
This is an attempt to send some dummy sensor readings.
"""
import random
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio  # noqa
from mavlite import MavLink, UART

from time import monotonic


try:
    import board  # noqa
    _uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)
except ImportError:
    _uart = UART(tx=17, rx=16, baudrate=230400)


ml = MavLink(
    message_ids=[  # Message includes.
        132,  # DISTANCE_SENSOR
    ],
)


async def distance_sensor():
    """
    This will send the distance sensor messages to the flight controller.
    """
    while True:  # Transmit loop
        try:
            await ml.send_message(
                message_id=132,  # DISTANCE_SENSOR
                payload=[
                    int(monotonic()),  # boot_time
                    1,  # min_distance
                    100,  # max_distance
                    random.randint(1, 100),  # current_distance
                    0,  # type
                    0,  # id
                    25,  # orientation  # Make fucking sure this matched the value in mission planner...
                    255,  # covariance
                    20,  # horizontal_fov
                    20,  # vertical_fov
                    0,  # quaternion
                    0,  # signal_quality
                ],
                debug=True
            )
        except KeyboardInterrupt:
            break


async def main(uart_):
    """
    Test mainloop.
    """

    tasks = await ml.io_buffers(uart_, debug=True)  # [write_loop, read_loop, heartbeat_loop, command_listener]
    await asyncio.gather(distance_sensor(), *tasks[0:3])


asyncio.run(main(_uart))
