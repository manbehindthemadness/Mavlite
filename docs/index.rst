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

Mavlite is an attempt to make a fully elective, light-weight Mavlink module for micropython and circuitpython.

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
are required to accept exactly seven arguments, the definition of these arguments is specific to the command* (see
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

**Command syntax**

**Heartbeats**

Examples
--------

**Restart flight controller**

**Rangefinder**

**I2C Multiplexor**

Links
=====

https://github.com/manbehindthemadness/Mavlite |br|
https://mavlink.io/en/messages/common.html |br|
https://mavlink.io/en/messages/ardupilotmega.html |br|
https://ardupilot.org/dev/docs/mavlink-commands.html |br|

Notes
=====

**Known issues**

**Future plans**


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. |br| raw:: html

     <br>