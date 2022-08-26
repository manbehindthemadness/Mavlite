"""
This is an example of how to use multiple vl53l1x sensors over a tca9548a multiplexor

ArduPilot NOTES:

* Rangefinder configurations must match transmission data in orientation.
* RNGFNDn_TYPE must be set to MavLink (10)
* PRX_TYPE Must be set to Mavlink (2)

It seems that only the orientation is evaluated, not the sensor id or originating component id.


"""
import random
import time
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
    _uart = UART(tx=17, rx=16, baudrate=230400)  # For micropython, adjust pins as needed.


TEST = True  # Change this if you just want to send random values instead of actually reading the multiplexor.

i2c = None
tca = None
if not TEST:
    import adafruit_tca9548a
    import adafruit_vl53l1x
    # Create I2C bus as normal
    i2c = board.I2C()  # uses board.SCL and board.SDA
    # TODO: We need to add compat for micropython's I2C.
    # Create the TCA9548A object and give it the I2C bus
    tca = tca9548a.TCA9548A(i2c)

# Create the mavlink object and include the definitions of the DISTANCE_SENSOR message
ml = MavLink(
    message_ids=[  # Message includes.
        132,  # DISTANCE_SENSOR
    ],
)


"""
I2c channel : orientation (-1 for disabled).

0	Forward
1	Forward-Right
2	Right
3	Back-Right
4	Back
5	Back-Left
6	Left
7	Forward-Left
24	Up
25	Down

NOTE: Ensure these orientations meet those configured on the flight controller.
"""
SENSORS = {
    0: 25,  # down
    1: 24,  # up
    2: 2,  # right
    3: 6,  # left
    4: 4,  # back
    5: 0,  # front
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
                    sensor_i2c = adafruit_vl53l1x.VL53L1X(tca[channel], 0x29)  # Pass child bus instead of parent.
                    print('start measuring')
                    sensor_i2c.start_ranging()
                    print('record data')
                    # while True:
                    for i in range(1):  # Place holder to get multiple samples if desired.
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
    One day this will handle the reading of the LiDAR array. Eight sensors will be read from the TCA9548A and one will be
    read from the native I2C bus all at address 0x29 / 41 This will allow us to sample a total of nine rangefinders
    through the microcontroller and a tenth via the flight controller's on-board I2C bus bringing us up to a total
    of ten.

    This will of course allow us to extend this module in the future to accommodate for other types of sensors.
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
            # c_id=chan + 25,  # This isn't a requirement, but it's tidy.
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
                        value = await self.read_sensor(channel, rangefinder)
                    await self.mavlink_send(channel, value)


async def main(read):
    """
    Test mainloop.
    """
    tasks = await ml.io_buffers(_uart, debug=False)  # [write_loop, read_loop, heartbeat_loop, command_listener]
    await asyncio.gather(read, tasks[0], tasks[2])


lidar = LiDAR(debug=True, test=True)

asyncio.run(main(lidar.read()))
