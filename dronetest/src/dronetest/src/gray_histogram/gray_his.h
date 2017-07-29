

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

const int checkSize = 5;
const int checkTime = 6;
const int gray_bins = 10;
const int scale = 5;

class gray_his
{
private:
	//ros initialize
	
	ros::NodeHandle n_his;
	
	
	image_transport::Subscriber image_sub;
	
	
public:
	IplImage frame;
	int checker;
	int timer;
	int counter;
	float lastGrowth;
	bool obstacle_occur;
	float gray_growth[checkTime];
	void image_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void rgbHis_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	
	gray_his(void);

	~gray_his(void);
};


