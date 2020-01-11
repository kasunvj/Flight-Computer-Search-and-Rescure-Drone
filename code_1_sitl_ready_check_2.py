import subprocess as sp
import logging 
sp.call('cls',shell=True)
logging.basicConfig(format='%(asctime)s - %(message)s',level=logging.INFO)
logging.info("Flight Checking Procedures")
#info, debug,warning, eror,critical
#Crashing Situations
# 1. Go too far
# 2. Loss GPS
# 3. Loss by Wind 
# 4. Battery
# 5. Unbalanced COG or constant drift
# 6.  


import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative # Import DroneKit-Python
import time 
import os
import numpy as np


#SITL
connection_string = '127.0.0.1:14550'
sitl = dronekit_sitl.start_default()

#Actual Drone over micro USB Direrctly
#connection_string = '/dev/ttyACM0'
#connection_string = '192.168.43.220:14550'

#connection to FC
try:
    logging.info("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("Connection to FC: Go ")
except:
    logging.info("Connection to FC: No go ")

#Radio for emergency control

try:
    # Access channels individually
    t1= time.time()
    while(time.time()-t1 < 10):
        print("Read channels individually:",round(10-(time.time()-t1)))
        print(" Ch1:    Roll :    %s" % vehicle.channels['1'])
        print(" Ch2:    Pitch:    %s" % vehicle.channels['2'])
        print(" Ch3:    Throttle: %s" % vehicle.channels['3'])
        print(" Ch4:    Yaw:      %s" % vehicle.channels['4'])
        print(" Ch5: %s" % vehicle.channels['5'])
        print(" Ch6: %s" % vehicle.channels['6'])
        print(" Ch7: %s" % vehicle.channels['7'])
        print(" Ch8: %s" % vehicle.channels['8'])
        print("Number of channels: %s" % len(vehicle.channels))
        time.sleep(1)
    if(raw_input("See a change in values? y/n\n") == 'y'):
        logging.info("Radio Go")
        
    else:
        logging.info("Radio No Go")

except:
    logging.info("Radio : No Go")
    


#Arming check

try: 
    logging.info("Pre Arm check")
    while not vehicle.is_armable:
        logging.info("check- vehicle booted,good GPS Fix,EKF pre-arm complte")
        time.sleep(1)

    logging.info("Arming Motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        logging.info("Arming")
        time.sleep(1)
    
    t1= time.time()
    while(time.time()-t1 <10):
        print("Armed ",round(10 - (time.time()-t1)))
        time.sleep(0.5)
    vehicle.armed = False


    if (raw_input("Can you see arming? y/n\n") == 'y'):
        logging.info("Arming Go\n")

    else:
        logging.info("Arming No Go\n")

except:
    logging.info("Arming : No Go")

while(True):
    {}
    


