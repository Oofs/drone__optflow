#include <unistd.h>
#include <curses.h>
#include <fstream>
#include <math.h>
#include <iomanip>


#include "std_msgs/Int64MultiArray.h"
#include "faceTrack.h"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

faceTrack::faceTrack(){
	
	
	faceX =0 ;
	faceY =0 ;
	faceWidth = 0;
	faceHeight = 0;
	face_pub = n_f.advertise<std_msgs::Int64MultiArray>("face",100);
	image_transport::ImageTransport it(n_f);
	//the constructor of ImageTransport() doesn't exist... so have to use ImageTransport(node)
        image_sub = it.subscribe("/ardrone/image_raw",1,&faceTrack::faceDetectCb,this);
	
}

faceTrack::~faceTrack(void){}

void faceTrack::facePub(int x,int y,int w,int h)
{
	std_msgs::Int64MultiArray face;
	
	face.data.resize(4);
	face.data[0] = x;
	face.data[1] = y;
	face.data[2] = w;
	face.data[3] = h;
	face_pub.publish(face);
}
void faceTrack::faceDetectCb(const sensor_msgs::ImageConstPtr& cam_image)
{
	
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

	 img_rgb = cv_ptr->image;
         cvtColor(img_rgb,img_gray,COLOR_BGR2GRAY );
         equalizeHist(img_gray,img_gray);
         
	faces_cascade.load("/opt/data/haarcascades/haarcascade_frontalface_alt.xml");
	faces_cascade.detectMultiScale(img_gray,faces,1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
         
	//for(size_t i=0;i<faces.size();i++){
	if(faces.size() == 1)
	{
		Point center(faces[0].x + faces[0].width/2,faces[0].y + faces[0].height/2);
		ellipse(img_rgb,center,Size(faces[0].width/2,faces[0].height/1.5),0,0,360,Scalar(0,255,0),2,8,0);
		faceX = center.x;
		faceY = center.y;
		faceWidth = faces[0].width;
		faceHeight = faces[0].height;
		
	}
	facePub(faceX,faceY,faceWidth,faceHeight);

/*
		Point center(faces[0].x + faces[0].width/2,faces[0].y + faces[0].height/2);
		//cout<<"center.x: "<<center.x<<" center.y: "<<center.y<<endl;

		faceX = center.x;
		faceY = center.y;
		faceWidth = faces[0].width;
		faceHeight = faces[0].height;
		ellipse(img_rgb,center,Size(faces[0].width/2,faces[0].height/1.5),0,0,360,Scalar(0,255,0),2,8,0);
	
	facePub(faceX,faceY,faceWidth,faceHeight);
  */    	
  	namedWindow("face",0);
	imshow("face",img_rgb);
	cvWaitKey(33);


}



