 

#include <unistd.h>
#include <curses.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"


#include "std_msgs/Empty.h" 
#include "geometry_msgs/Twist.h"
#include "iostream"

#include "RosControl.h"
using namespace std;

int main(int argc,char**argv){

	ros::init(argc, argv, "dronetest");
	RosControl ardrone;

	ardrone.run();
	ardrone.startControl();
	

	
	return 0;


}
