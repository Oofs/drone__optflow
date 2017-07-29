

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#include "std_msgs/Empty.h" 
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "iostream"
using namespace std;

const int recordSize = 10;
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
	
	ros::NodeHandle n_;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher vel_pub;
	ros::Subscriber imu_sub;
	ros::Publisher flow_pub;
	
	
	image_transport::Subscriber image_sub;
	sensor_msgs::Imu imu;
	sensor_msgs::Imu RTimu;
	
public:

	int counter;
	int flowRecordX[recordSize];
	int flowRecordY[recordSize];
	int integraleX;
	int integraleY;
	int times;
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
	//void run();
	void imuCb(const sensor_msgs::ImuConstPtr);
	void pubFlow(int code1 ,int code2);
	void pubFlow(float code1 ,float code2);
	void pykFlow_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void pykFlowAvoidance_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void pykFlowRec_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void pykFlowTTC_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void pykFlowRecTTC_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void pykFlowRecAvg_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void pykFlowRecAvoidance_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	RosControl(void);
	~RosControl(void);
};


