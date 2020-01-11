print ("Start simulator (SITL) _ AutoPilot")
import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative # Import DroneKit-Python
import time 
import os
import numpy as np

newDir = os.path.join(os.getcwd(),"Flight_Doc")
if not os.path.exists(newDir):
	os.mkdir(newDir)

connection_string = '127.0.0.1:14550'

sitl = dronekit_sitl.start_default()
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)


def reporter():
	'''reporter
	This is to show and send some useful vehicle attributes (state) to the GCS to watch the behaviour of the Vehicle
	
	#Args: None 
	#Return : None '''
	timestamp = time.ctime()
	print ("---------------------------------", timestamp)
	print ("Altitude : %s" % vehicle.attitude)
	print (" GPS: %s" % vehicle.gps_0)
	print (" Battery: %s" % vehicle.battery)
	print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
	print (" Is Armable?: %s" % vehicle.is_armable)
	print (" System status: %s" % vehicle.system_status.state)
	print (" Mode: %s" % vehicle.mode.name)   # settable
	print ("-----------------------------------------------------------")
def arm_and_takeoff(targetAlt):
	'''arm_and_takeoff
	arming motors and lift off to a given alttude 
	
	#Args: 
		targetAlt : Altitude you want to lift off

	#Return : None '''
	print ("Pre-Arm Check")
	while not vehicle.is_armable: #waiting until the copter armble
		print ("Waiting for vehicle to initialize")
		time.sleep(1)

	print("Arming Motors")
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
		print("Arming")
		time.sleep(1)

	print ("Taking off")
	vehicle.simple_takeoff(targetAlt)

	while  True:
		print ("Altitude: ", vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= targetAlt*0.95:
			print ("Altitude Reached")
			break
		time.sleep(1)
def wayPointGenaration():
	'''
	wayPointGenaration
	this will genarate waypoints in the user selected area and save inside a txt file for reference

	Args:
		OutsidePoint1: user spesified edge gps point 
		OutsidePoint2: user spesified edge gps point 

	return:
		None

		'''
	gps  =np.array( [[7.088252, 80.037605],
                     [7.090202, 80.034575],
	                 [7.093444, 80.036800],
	                 [7.090271, 80.039997]])

	
	file1 = open(os.getcwd() + "\Flight_Doc\wayPointGenaration.txt","w+")
	for i in range(0,4):
		file1.write(str(i)+","+str(gps[i,0])+","+str(gps[i,1])+"\n")

	file1.close()
def wayPointGet(point):
	'''
	wayPointGet
	This will send the next waypoint to the vehical from the list of the waypoints 
	genarated from wayPointGenaration
	Args :
		point : request waypoint number
	Return:
		Nest waypoint
	'''
	file2 = open(os.getcwd() + "\Flight_Doc\wayPointGenaration.txt","r+")
	allGPS = file2.readlines()
	waypointNo,lat,longi = allGPS[point].split(",")
	longi = longi.split("\n")[0]
	file2.close()

	return [lat,longi]



arm_and_takeoff(5)



# wayPointGenaration()

# print(wayPointGet(1))
# vehicle.simple_goto(LocationGlobalRelative(float(wayPointGet(1)[0]),float(wayPointGet(1)[1]),30),groundspeed = 10)

 # while(True):
 # 	print(vehicle.location.global_relative_frame)
 # 	time.sleep(1)

try:
	
except:
	print('Return to launch')
	vehicle.mode = VehicleMode("RTL")
	#Close vehicle object before exiting script
	#print("Close vehicle object")
	#vehicle.close()
	# Shut down simulator if it was started.
	if sitl is not None:
		sitl.stop()

print('Return to launch')
vehicle.mode = VehicleMode("RTL")
#Close vehicle object before exiting script
#print("Close vehicle object")
#vehicle.close()
# Shut down simulator if it was started.
if sitl is not None:
	sitl.stop()






