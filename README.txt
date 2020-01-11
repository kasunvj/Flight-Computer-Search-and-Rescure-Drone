1. open 3 tabs 
2. 

*Terminal 1 - Start SITL: 
dronekit-sitl copter --home=7.086271,80.038264,0,0
dronekit-sitl copter --home=6.796470,79.902294,0,0
mora ground
dronekit-sitl copter --home=6.7975379,79.8993748,0,0 

*Terminal 2 - Start MavProxy:
mavproxy --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14552

Connecting with Actual Drone(Pxhawk,Rpi(192.168.43.20) and GCS(Mission Planner)(192.168.43.115)Note: but Connect MP with SW radio  
*Terminal 1 - Start MavProxy:
sudo -s
mavproxy.py --master=/dev/ttyACM0 --out 192.168.43.220:14550 --out 192.168.43.115:14552

*Terminal 2 - Start Python Code(Check Connection String det as connection_string = '192.168.43.220:14550')
python sitl_ready_check_1.py
or
python .\Hover_and_Land.py  --connect 127.0.0.1:14550

Now you can now open mission planner and connect to UDP baus 921600 on port 14552





---------------------------------
Common Errors:
1. 1 link down with unsuccessful barometer calib 

try : dronekit-sitl copter