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
#connection_string = '127.0.0.1:14550'
#sitl = dronekit_sitl.start_default()

#Actual Drone over micro USB Direrctly
#connection_string = '/dev/ttyACM0'
connection_string = '192.168.43.220:14550'

#connection to FC
try:
    logging.info("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("Connection to FC: Ok ")
except:
    logging.info("Connection to FC: Fail ")

#Radio for emergency control

try:
    # Access channels individually
    #while(True):
        print("Read channels individually:")
        print(" Ch1: %s" % vehicle.channels['1'])
        print(" Ch2: %s" % vehicle.channels['2'])
        print(" Ch3: %s" % vehicle.channels['3'])
        print(" Ch4: %s" % vehicle.channels['4'])
        print(" Ch5: %s" % vehicle.channels['5'])
        print(" Ch6: %s" % vehicle.channels['6'])
        print(" Ch7: %s" % vehicle.channels['7'])
        print(" Ch8: %s" % vehicle.channels['8'])
        print("Number of channels: %s" % len(vehicle.channels))
        time.sleep(1)

except:
    logging.info("Radio : Fail")
    


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
    
    time.sleep(10) #Arming for 3seconds
    vehicle.armed = False

    arming_verify = True

    while (arming_verify):
        if (raw_input("Can you see arming? y/n") == 'y'):
            arming_verify = True
            logging.info("Arming verified")
        elif (raw_input("Can you see arming? y/n") == 'n'):
            arming_verify = False
            logging.info("Problem With Arming")
            break
        else:
            print("please use only 'y' and 'n'\n")
    
    logging.info("Arming : Success")

except:
    logging.info("Arming : Fail")

while(True):
    {}
    


