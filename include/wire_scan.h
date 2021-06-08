#ifndef _WIRE_SCAN_H_
#define _WIRE_SCAN_H_

// ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "read_sensor/proximity_sensor_data.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"

// Robot library headers
#include "sun_robot_lib/Robot.h"
#include "sun_robot_lib/Robots/UR5e.h"

// C/C++ library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <limits>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// Private headers
#include "bash_color_list.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

/* Proximity Sensor Topic Name */
#define PROXIMITY_SENSOR_TOPIC "/ProximityData_dev_ttyUSB3_P001"

/* Distance between successive trajectory points */
#define TRAJ_MAX_POINT_DISTANCE 0.005 /* [m] */

#endif /* _WIRE_SCAN_H_ */
