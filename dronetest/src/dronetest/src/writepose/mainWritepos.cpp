#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64MultiArray.h"
#include <iomanip>
using namespace std;

void posCb(const gazebo_msgs::ModelStatesConstPtr model)
{
	geometry_msgs::Pose pos;
	//pos = model->pose[11];//8walls
	pos = model->pose[1];//4walls
	ofstream write_pos("/opt/result/pos4_4.txt",ios::app);
	write_pos<<setiosflags(ios::fixed)<<setprecision(2);	
	write_pos<<pos.position.x<<" "<<pos.position.y<<endl;
	
	
}
void flowCb(const std_msgs::Float64MultiArrayConstPtr flowPtr)
{	
	ofstream write_flow("/opt/result/flowdiff.txt",ios::app);
	write_flow<<setiosflags(ios::fixed)<<setprecision(1);
	write_flow<<flowPtr->data[1]-flowPtr->data[0]<<endl;
}


int main(int argc, char **argv){

	ros::init(argc,argv,"writePos");
	ros::NodeHandle n_write;
	ros::Subscriber write_pos_sub;
	ros::Subscriber write_flow_sub;
	write_pos_sub = n_write.subscribe(n_write.resolveName("gazebo/model_states"),100, posCb);
	write_flow_sub  = n_write.subscribe(n_write.resolveName("optical_flow_net"),100,flowCb);
	ros::spin();


	

	return 0;

}


