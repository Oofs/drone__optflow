#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

static const char WINDOW[]="RGB Image";
static const char WINDOW2[]="Gray Image";

void process(const sensor_msgs::ImageConstPtr& cam_image){
cv_bridge::CvImagePtr cv_ptr;
try
{
  cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
}

catch (cv_bridge::Exception& e)
{
  ROS_ERROR("cv_bridge exception:%s",e.what());
  return;
}

Mat img_rgb = cv_ptr->image;
Mat img_gray;

cvtColor(img_rgb,img_gray,CV_RGB2GRAY);

imshow(WINDOW,img_rgb);
imshow(WINDOW2,img_gray);
cvWaitKey(1);
}

int main(int argc, char **argv){
ros::init(argc,argv,"droneTest");
ros::NodeHandle n;
image_transport::ImageTransport it(n);
image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw",1,process);

cv::namedWindow(WINDOW);
cv::namedWindow(WINDOW2);
ros::spin();
return 0;
}
