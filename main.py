"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
# Import mavutil
from micropymavlink import mavutil

#mavutil.set_dialect("mavgen_python_dialect")

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600 )

# Make sure the connection is valid
master.wait_heartbeat()

# Get some information !
min_distance = 5
max_distance = 380
# Sensor direcion mapping, Forward, Right, Back, Left, Up, Down
SensorDirections=[0,2,4,6,24,25]

while True:
    # Forward

    for i in range(len(SensorDirections)):
        ExampleDistance = 3 # COLLECT YOUR SENSOR DISTANCE HERE FOR EACH SENSOR (cm) 
        mavutil.distance_sensor_send(min_distance ,max_distance , ExampleDistance, SensorDirections[i], 0, 6)
    time.sleep(0.1)