
#include <unistd.h>
#include <curses.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"



#include "std_msgs/Empty.h" 
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "iostream"
using namespace std;


struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};

class RosControl
{
private:
	//ros initialize
	
	ros::NodeHandle nh_;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher vel_pub;
	ros::Subscriber imu_sub;
	
	
public:
	RosControl(void);
	~RosControl(void);
	
	ControlCommand forward ;
	ControlCommand backward ;
	ControlCommand right ;
	ControlCommand left ;
	ControlCommand up;
	ControlCommand down;
	ControlCommand hover;
	ControlCommand clockwise;
	ControlCommand anticlockwise;

	void startControl();
	void sendVel(ControlCommand cmd);
	void run();
	void imuCb(const sensor_msgs::ImuConstPtr);
	void track(char* flow_file);
};


