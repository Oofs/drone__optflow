#include "RosControl.h"




//-------------------------------Main Function------------------------------------------
int main(int argc, char **argv){

ros::init(argc,argv,"droneTest");
RosControl flowpub;
ros::spin();
return 0;
}
//imu subrate:117hz
//pic subrate::17hz
