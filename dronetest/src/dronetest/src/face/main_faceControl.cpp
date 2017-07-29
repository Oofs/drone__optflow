#include "faceControl.h"
//-------------------------------Main Function------------------------------------------
int main(int argc, char **argv){

ros::init(argc,argv,"faceControl");
faceControl facecontrol;
ros::Rate rate(20);

while(ros::ok())
{
	
	ros::spinOnce();
	facecontrol.setVel();
	facecontrol.sendVel();
	rate.sleep();
}

return 0;
}
