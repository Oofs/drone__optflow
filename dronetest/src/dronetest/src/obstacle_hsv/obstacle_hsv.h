

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
using namespace cv;


class obstacle_hsv
{
private:
	//ros initialize
	
	ros::NodeHandle n_his;
	
	
	image_transport::Subscriber image_sub;
	
	
public:
	
	
	CvHistogram* hist_h;
        CvHistogram* hist_v;


	
	void myobstacle_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	void rgbHis_Cb(const sensor_msgs::ImageConstPtr& cam_image);
	
	obstacle_hsv(void);

	~obstacle_hsv(void);
};


