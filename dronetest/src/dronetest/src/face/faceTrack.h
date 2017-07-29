#include <unistd.h>
#include <curses.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/opencv.hpp"

#include "std_msgs/Empty.h" 
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "iostream"
using namespace std;
using namespace cv;




class faceTrack
{
private:
	//ros initialize
	
	ros::NodeHandle n_f;
	ros::Publisher face_pub;
	
	
public:
	faceTrack(void);
	~faceTrack(void);
	image_transport::Subscriber image_sub;
	CascadeClassifier faces_cascade;
	Mat img_rgb ;
	Mat img_gray;
	vector<Rect> faces;
	int faceX; 
	int faceY;
	int faceWidth;
	int faceHeight;
	
	void faceDetectCb(const sensor_msgs::ImageConstPtr& cam_image);
	void facePub(int x,int y, int w, int h);
};


