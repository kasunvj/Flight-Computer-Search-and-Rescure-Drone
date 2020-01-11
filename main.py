print ("Start simulator (SITL)")
import dronekit_sitl
from dronekit import connect, VehicleMode # Import DroneKit-Python
import time 

#setting Connection mecthod
#Linux computer connected to the vehicle via Serial port (RaspberryPi example)	/dev/ttyAMA0 (also set baud=57600)
#SITL connected to the vehicle via UDP	127.0.0.1:14550
connection_string = '127.0.0.1:14550'

sitl = dronekit_sitl.start_default()
# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
def reporter():
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
	print ("Pre-Arm Check")
	while not vehicle.is_armable:
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


def main():
	reporter()
	arm_and_takeoff(80)
	reporter()


if(__name__ == "__main__"):
	main()

