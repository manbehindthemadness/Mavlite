.. documentation master file, created by
   sphinx-quickstart on Mon Aug 22 08:07:29 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. image:: /_static/mavlite_sphinx.png

.. toctree::
   :maxdepth: 4
   :caption: Contents:

About
=====

Mavlite is an attempt to make a fully elective, asynchronous, and light-weight Mavlink module for micropython and circuitpython.

System requirements
===================

As of the time of this writing Mavlink uses roughly 45kb of flash and as little as 30kb of memory.
*Note: memory consumption is subject to change based on the number of message includes that are in use.*

Tested hardware
---------------

* XIAO rp2040
* ESP32 Standard

Prerequisites
=============

micropython
-----------

* uasyncio
   * https://github.com/peterhinch/micropython-async

circuitpython
-------------

* busio
   * https://github.com/adafruit/Adafruit_BusIO
* board
   * https://docs.circuitpython.org/en/latest/shared-bindings/support_matrix.html
* asyncio
   * https://github.com/adafruit/Adafruit_CircuitPython_asyncio

examples
--------

* adafruit_tca9548a
   * https://github.com/adafruit/Adafruit_CircuitPython_TCA9548A
* adafruit_vl53l1x
   * https://github.com/adafruit/Adafruit_CircuitPython_VL53L1X

documentation
-------------

* sphinx>=2.4
* sphinx-copybutton
* sphinx-issues>=3.0.1
* sphinx-removed-in
* sphinx-rtd-theme>=1.0
* sphinxext-opengraph


Usage
=====

General
-------

**Message includes**

As the Mavlink dialect files are far too large for general microcontrollers to use, Mavlite includes them based on
message_ids that are specified in a list object passed during init. Message definitions can be found in the Mavlink
documentation *(see links section)*.


.. code-block:: python

   from mavlite import MavLink

   # Create the mavlink object and include the definitions of the DISTANCE_SENSOR message
   ml = MavLink(
       message_ids=[  # Message includes.
           132,  # DISTANCE_SENSOR
       ],
   )

**Command callbacks**

Incoming commands are handled in the form of callback executions and are defined in a dict object passed during init. Commands
are required to accept exactly seven arguments, the definition of these arguments is specific to the command *(see
links section)*. In addition to specific arguments, commands must return three values: ``result`` | ``progress`` | ``result2``.
The command callback will be executed when the specified command is received by the ``command_listener`` task loop.

.. code-block:: python

   from mavlite import MavLink

   async def reboot(*args):
      """
      This is a simple example of an incoming command callback function.
      """
      # A real command should evaluate the 7 bytes of param data that will be passed as args.
      import microcontroller
      microcontroller.reset()
      # Any other command would need to return status information.
      # return 0, 0, 0

   ml = MavLink(
      callbacks={  # Command callbacks.
         246: reboot
      }
   )


**Task loops**

Selective task loops can be called from ``mavlite.Mavlink.io_buffers`` This returns a tuple of asynchronous functions
that can be fed directly into tasks:

* ``write loop`` Provides outgoing packet transmission buffer
* ``read_loop`` Provides incoming packet reception buffer
* ``heartbeat_loop`` Sends a heartbeat packet at 1hz into the write buffer
* ``command_listener`` Checks the read buffer for specific commands and executes their respective callback functions

.. code-block:: python

   from mavlite import MavLink, UART

   _uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)

   ml = MavLink(
    message_ids=[  # Message includes.
        132,  # DISTANCE_SENSOR
      ],
   )

   async def main(read):
      """
      Test mainloop.
      """
      tasks = await ml.io_buffers(_uart, debug=False)  # [write_loop, read_loop, heartbeat_loop, command_listener]
      await asyncio.gather(read, tasks[0], tasks[2])


   lidar = LiDAR(debug=True, test=True)

   asyncio.run(main(lidar.read()))


**Command syntax**

Sending a command can be achieved using ``mavlite.MavLink.send_command``. This will transmit the command to the write buffer and attempt to return the ACK, providing
it's available and has not timed out. Each mavlink command has seven unique params that must be transmitted, the definitions of these can be found in the documentation
provided in the links section.

.. code-block:: python

      await m.send_command(
           command_id=246,
           target_system=0,
           target_component=0,
           params=[1, 0, 0, 0, 0, 0, 0],
           debug=True
       )

Examples
--------

**Restart flight controller**

*examples/restart.py*

.. code-block:: python

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


**Rangefinder**

*examples/snc.py*

.. code-block:: python

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


**I2C Multiplexor**

*examples/mplex.py*

.. code-block:: python

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


Links
=====

https://github.com/manbehindthemadness/Mavlite |br|
https://mavlink.io/en/messages/common.html |br|
https://mavlink.io/en/messages/ardupilotmega.html |br|
https://ardupilot.org/dev/docs/mavlink-commands.html |br|

Notes
=====

**Known issues**

* Read loop needs serious optimization.
* A request-command/receive method has not yet been implemented.

**Future plans**

* Optimize read performance.
* Accelerate code using C-extensions where possible.
* Add custom XML dialog functionality.
* Add request-command/receive functionality.

**Caveats**

* At this time this module only supports UART communication, as such packet signing has not been included:
   * Serves no function for hard-wired communication.
   * Performance would diminish without hardware SHA acceleration.
* Currently only the following dialects have been included in the MSCFormats dictionary:
   * minimal
   *  ardupilotmega
   *  common
   *  uAvionix

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. |br| raw:: html

     <br>