**wire_scan node interface**

Following the step to configure and launch the node:
1) Fill the define in wire_scan.h "PROXIMITY_SENSOR_TOPIC" with the name of the Proximity Sensor Topic
2) Launch the node by executing the following command in the terminal:
rosrun wire_scan wire_scan_node deltaX deltaY deltaZ deltaScan v [x/y]
where
- deltaX: is the area x dimension w.r.t. the current e-e position expressed in base-frame [m]
- deltaY: is the area y dimension w.r.t. the current e-e position expressed in base-frame [m]
- deltaZ: is the offset on Z axis w.r.t. the current e-e position expressed in base-frame [m]
- deltaScan: is the distance between each scan line [m]
- v: is the cruiser speed [m/s]
- x/y: is an optional parameter to determine the scanning direction. Default is x
