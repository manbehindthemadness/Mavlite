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

print("Got Heartbeat")

master.reboot_autopilot()

#print("Loaded")
## Get some information !
#while True:
#    try:
#        print(master.recv_match().to_dict())
#    except:
#        pass
#    time.sleep(0.1)