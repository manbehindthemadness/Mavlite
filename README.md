![Mavlite](https://mavlite.readthedocs.io/en/latest/_images/mavlite_sphinx.png)

![Mavlite](https://mavlite.readthedocs.io/en/latest/_static/mavlite.png)

## About

Mavlite is an attempt to make a fully elective, asynchronous, and light-weight Mavlink module for MicroPython and CircuitPython.

## System Requirements

As of the time of this writing, Mavlink uses roughly 45kb of flash and as little as 30kb of memory. Note: memory consumption is subject to change based on the number of message includes that are in use.

## Tested Hardware

- XIAO rp2040
- ESP32 Standard

## Prerequisites

### MicroPython

- [uasyncio](https://github.com/peterhinch/micropython-async)

### CircuitPython

- [busio](https://github.com/adafruit/Adafruit_BusIO)
- [board](https://docs.circuitpython.org/en/latest/shared-bindings/support_matrix.html)
- [asyncio](https://github.com/adafruit/Adafruit_CircuitPython_asyncio)

### Examples

- [adafruit_tca9548a](https://github.com/adafruit/Adafruit_CircuitPython_TCA9548A)
- [adafruit_vl53l1x](https://github.com/adafruit/Adafruit_CircuitPython_VL53L1X)

### Documentation

- sphinx>=2.4
- sphinx-copybutton
- sphinx-issues>=3.0.1
- sphinx-removed-in
- sphinx-rtd-theme>=1.0
- sphinxext-opengraph

## Usage

### General

**Message Includes**

As the Mavlink dialect files are far too large for general microcontrollers to use, Mavlite includes them based on message_ids that are specified in a list object passed during init. Message definitions can be found in the Mavlink documentation (see links section).

```python
from mavlite import MavLink

ml = MavLink(
    message_ids=[
        132,  # DISTANCE_SENSOR
    ],
)
```

**Command Callbacks**

Incoming commands are handled in the form of callback executions and are defined in a dict object passed during init. Commands are required to accept exactly seven arguments, the definition of these arguments is specific to the command (see links section).

```python
async def reboot(*args):
    """
    This is a simple example of an incoming command callback function.
    """
    import microcontroller
    microcontroller.reset()

ml = MavLink(
    callbacks={
        246: reboot
    }
)
```

**Task Loops**

Selective task loops can be called from `mavlite.Mavlink.io_buffers`. This returns a tuple of asynchronous functions that can be fed directly into tasks:

- `write loop`: Provides outgoing packet transmission buffer
- `read_loop`: Provides incoming packet reception buffer
- `heartbeat_loop`: Sends a heartbeat packet at 1hz into the write buffer
- `command_listener`: Checks the read buffer for specific commands and executes their respective callback functions

```python
from mavlite import MavLink, UART

_uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)

ml = MavLink(
    message_ids=[
        132,  # DISTANCE_SENSOR
    ],
)

async def main(read):
    """
    Test mainloop.
    """
    tasks = await ml.io_buffers(_uart, debug=False)
    await asyncio.gather(read, tasks[0], tasks[2])

lidar = LiDAR(debug=True, test=True)

asyncio.run(main(lidar.read()))
```

### Command Syntax

Sending a command can be achieved using `mavlite.MavLink.send_command`. This will transmit the command to the write buffer and attempt to return the ACK, providing it's available and has not timed out. Each Mavlink command has seven unique params that must be transmitted, the definitions of these can be found in the documentation provided in the links section.

```python
await m.send_command(
    command_id=246,
    target_system=0,
    target_component=0,
    params=[1, 0, 0, 0, 0, 0, 0],
    debug=True
)
```

## Examples

### Restart Flight Controller

*examples/restart.py*

```python
"""
This is a simple test method that sends restart commands.
"""
from mavlite import MavLink, UART
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio

try:
    import board
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
```

### Rangefinder

*examples/snc.py*

```python
"""
This is an attempt to send some dummy sensor readings.
"""
import random
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio
from mavlite import MavLink, UART

from time import monotonic

try:
    import board
    _uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)
except ImportError:
    _uart = UART(tx=17, rx=16, baudrate=230400)

ml = MavLink(
    message_ids=[
        132,  # DISTANCE_SENSOR
    ],
)

async def

 distance_sensor():
    """
    This will send the distance sensor messages to the flight controller.
    """
    while True:
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
                    25,  # orientation  # Make sure this matched the value in mission planner...
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
    tasks = await ml.io_buffers(uart_, debug=True)
    await asyncio.gather(distance_sensor(), *tasks[0:3])

asyncio.run(main(_uart))
```

### I2C Multiplexor

*examples/mplex.py*

```python
"""
This is an example of how to use multiple vl53l1x sensors over a tca9548a multiplexor
"""
import random
import time
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio
from mavlite import MavLink, UART

from time import monotonic

try:
    import board
    _uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)
except ImportError:
    _uart = UART(tx=17, rx=16, baudrate=230400)

TEST = True

i2c = None
tca = None
if not TEST:
    import adafruit_tca9548a
    import adafruit_vl53l1x
    i2c = board.I2C()
    tca = tca9548a.TCA9548A(i2c)

ml = MavLink(
    message_ids=[
        132,  # DISTANCE_SENSOR
    ],
)

SENSORS = {
    0: 25,  # down
    1: 24,  # up
    2: 2,   # right
    3: 6,   # left
    4: 4,   # back
    5: 0,   # front
    6: -1,
    7: -1
}

def test_scan():
    """
    This will fire up the multiplexer and lidar sensors to prove everything is talking the way it should.
    """
    total = 0
    for channel in range(8):
        needs_unlock = True
        if tca[channel].try_lock():
            print("Channel {}:".format(channel), end="")
            addresses = tca[channel].scan()
            for address in addresses:
                if address == 0x29:
                    print('found lidar____________________________________________________________________________________')
                    total += 1
                    print('unlocking channel')
                    tca[channel].unlock()
                    needs_unlock = False
                    print('grabbing sensor')
                    sensor_i2c = adafruit_vl53l1x.VL53L1X(tca[channel], 0x29)
                    print('start measuring')
                    sensor_i2c.start_ranging()
                    print('record data')
                    # while True:
                    for i in range(1):
                        print('check sensor ready')
                        if sensor_i2c.data_ready:
                            print('get distance')
                            distance = sensor_i2c.distance
                            if distance:
                                print('reading', distance)
                            print('clearing interrupt')
                            sensor_i2c.clear_interrupt()
                        time.sleep(0.1)
        if needs_unlock:
            print('unlocking channel')
            tca[channel].unlock()
    print('total modules found:', total)

class LiDAR:
    """
    One day this will handle the reading of the LiDAR array.
    """
    rangefinders = channel_nums = range(0, 7)

    def __init__(self, debug: bool = False, test: bool = False):
        """
        Here we will fire up all of our sensors and tell them to start taking samples.
        """
        self.test = test
        self.debug = debug
        if not self.test:
            self.channels = [i2c]
            for channel in self.channel_nums:
                self.channels.append(tca[channel])
            self.rangefinders = list()
            for channel in self.channels:  # Confirm all our sensors are healthy
                if SENSORS[channel] >= 0:  # Skip disabled channels.
                    if channel.try_lock():
                        channel.unlock()
                    rangefinder = adafruit_vl53l1x.VL53L1X(channel, 0x29)
                    rangefinder.start_ranging()
                    self.rangefinders.append(rangefinder)
                    channel.unlock()

    async def mavlink_send(self, chan: int, value: int):
        """
        This will send the distance sensor messages to the flight controller.
        """

        await ml.send_message(
            message_id=132,  # DISTANCE_SENSOR
            payload=[
                int(monotonic()),  # boot_time
                2,  # min_distance
                400,  # max_distance
                value,  # current_distance
                0,  # type
                chan,  # id  # This isn't a requirement, but it's tidy.
                SENSORS[chan],  # orientation  # Make sure this matches the value in mission planner...
                255,  # covariance
                20,  # horizontal_fov
                20,  # vertical_fov
                0,  # quaternion
                0,  # signal_quality
            ],
            debug=self.debug
        )
        # print('sensorID', chan, 'orient', SENSORS[chan], 'value', value)

    @staticmethod
    async def read_sensor(
            channel: [tca9548a.TCA9548A, board.I2C],
            rangefinder: adafruit_vl53l1x.VL53L1X
    ) -> [int, float]:
        """
        This will read an individual rangefinder and return the data.
        """
        if channel.try_lock():
            channel.unlock()
        while not rangefinder.data_ready:
            await asyncio.sleep(0.01)
        result = rangefinder.distance
        """
        Here we can add dynamic range mode and FOV changes should we want in the future.
        """
        rangefinder.clear_interrupt()
        channel.unlock()
        return result

    async def read(self) -> list:
        """
        This will take a reading from the various sensors and passes them into the mavlink_send method.
        """
        while True:
            for channel, rangefinder in zip(self.channel_nums, self.rangefinders):
                if SENSORS[channel] >= 0:  # Skip disabled channels.
                    if self.test:
                        value = random.randint(1, 100)
                    else:
                        value = await self.read_sensor(channel,

 rangefinder)
                    await self.mavlink_send(channel, value)

async def main(read):
    """
    Test mainloop.
    """
    tasks = await ml.io_buffers(_uart, debug=False)  # [write_loop, read_loop, heartbeat_loop, command_listener]
    await asyncio.gather(read, tasks[0], tasks[2])

lidar = LiDAR(debug=True, test=True)

asyncio.run(main(lidar.read()))
```

## Links

- [Full Documentation](https://mavlite.readthedocs.io/en/latest/)
- [GitHub Repository](https://github.com/manbehindthemadness/Mavlite)
- [Mavlink Documentation](https://mavlink.io/en/messages/common.html)
- [ArduPilotMega Documentation](https://mavlink.io/en/messages/ardupilotmega.html)
- [Mavlink Commands](https://ardupilot.org/dev/docs/mavlink-commands.html)

## Notes

### Known Issues

- Read loop needs serious optimization.
- A request-command/receive method has not yet been implemented.

### Future Plans

- Optimize read performance.
- Accelerate code using C-extensions where possible.
- Add custom XML dialog functionality.
- Add request-command/receive functionality.

### Caveats

- At this time this module only supports UART communication, as such packet signing has not been included:
  - Serves no function for hard-wired communication.
  - Performance would diminish without hardware SHA acceleration.
- Currently only the following dialects have been included in the MSCFormats dictionary:
  - minimal
  - ardupilotmega
  - common
  - uAvionix
