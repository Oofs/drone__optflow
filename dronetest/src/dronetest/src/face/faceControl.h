#include "ros/ros.h"
#include "ros/callback_queue.h"


#include "std_msgs/Empty.h" 
#include "std_msgs/Int64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "iostream"
using namespace std;


const int recordSize = 10;
const double faceSDThreshold = 50;

const	double gain=0.5;
const	double thresholdX=0.45;
const	double thresholdY=0.45;
const	double thresholdMin=0.1 ;
const	double thresholdMax=0.25;
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

class faceControl
{

private:
	ros::NodeHandle n_fC;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	
	
	
	ros::Subscriber face_sub;
	
	
public:
	ros::Publisher cmd_vel_pub;

	ControlCommand forward ;
	ControlCommand backward ;
	ControlCommand right ;
	ControlCommand left ;
	ControlCommand up;
	ControlCommand down;
	ControlCommand hover;
	ControlCommand clockwise;
	ControlCommand anticlockwise;
	
	sensor_msgs::Imu imu;
	ardrone_autonomy::Navdata navdata;
	geometry_msgs::Twist cmd;

	
	int count ;
	
	
	int recordX[recordSize];
	int recordY[recordSize];
	double SDX;
	double SDY;
	double speed;
	int faceX; 
	int faceY;
	int faceWidth;
	int faceHeight;
	int direction;
	void setVel();
	void sendVel();
	void cmdCb(const std_msgs::Int64MultiArrayConstPtr facePtr);
	void calSD();
	faceControl();
	~faceControl();
	

};
	
	
	
