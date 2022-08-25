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