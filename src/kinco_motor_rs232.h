#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
//#include "geometry_msgs/Pose.h"


int byte;
int head;
int frame;
ssize_t size;
std_msgs::Float32 yaw;

