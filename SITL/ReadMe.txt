1. open 3 tabs 
2. 

Terminal 1 - Start SITL: 
dronekit-sitl copter --home=7.086271,80.038264,0,0

Terminal 2 - Start MavProxy:
mavproxy --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14552

Connecting with Actual Drone(Pxhawk and Rpi) 
mavproxy --master /dev/ttyACM0 --out=udp: 10.0.1.128:14550 --out=udp: 10.0.1.XX:14550
xx - computer which Mission Planner is running

Now you can now opne mission planner and connect to UDP baus 921600 on port 14552

Terminal 3 - Start your code , connect to 127.0.0.1:14550
python .\Hover_and_Land.py  --connect 127.0.0.1:14550



---------------------------------
Common Errors:
1. 1 link down with unsuccessful barometer calib 

try : dronekit-sitl copter