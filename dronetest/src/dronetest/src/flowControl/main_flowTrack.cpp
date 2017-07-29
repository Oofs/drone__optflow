#include "flowTrack.h"
//-------------------------------Main Function------------------------------------------
//
int main(int argc, char **argv){

ros::init(argc,argv,"flowTrack");
flowTrack planetrack;
ros::Rate rate(10);

while(ros::ok())
{
	
	
	ros::spinOnce();
	
	planetrack.setCmdTwistAvoidance();
	//planetrack.detect_stop();
	planetrack.sendCmdTwist();
	
	rate.sleep();
}

return 0;
}
