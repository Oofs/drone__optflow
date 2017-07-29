



#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include  "opencv2/opencv.hpp"



#include "std_msgs/Empty.h" 
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "iostream"

using namespace std;
using namespace cv;
bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
CvHistogram* hist_h;
CvHistogram* hist_v;
int vmin = 10,vmax=256,smin=30;
int threshold_h_bins =100;

static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
        
     //   selection &= Rect(0, 0, image.cols, image.rows);
    }
    
    switch( event )
    {
        case EVENT_LBUTTONDOWN:
           
            origin = Point(x,y);
            selection = Rect(x,y,0,0);
            selectObject = true;
            break;
        case EVENT_LBUTTONUP:
            selectObject = false;
            if( selection.width > 0 && selection.height > 0 )
                trackObject = -1;
            break;
    }
}


void myobstacle_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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

	
    Mat _frame = cv_ptr->image;
    IplImage frame = (IplImage)_frame;
    
   
    
   
  	
        
        
      
            
       
        
        IplImage* hsv = cvCreateImage(cvGetSize(&frame), 8, 3);
        IplImage* frame_smooth = cvCreateImage(cvGetSize(&frame), 8, 3);
        IplImage* h_plane = cvCreateImage(cvGetSize(&frame), 8, 1);
        IplImage* s_plane = cvCreateImage(cvGetSize(&frame), 8, 1);
        IplImage* v_plane = cvCreateImage(cvGetSize(&frame), 8, 1);
        IplImage* bi_pic = cvCreateImage(cvGetSize(&frame),8,1);
        IplImage* plane_h[] = {h_plane};
        IplImage* plane_v[] = {v_plane};
        IplImage* hist_img;
        IplImage* mask = cvCreateImage(cvGetSize(&frame),8,1);
	cvSet(bi_pic, Scalar(255,255,255));

	int _vmin = vmin,_vmax = vmax;
	float _threshold_h_bins = (float)threshold_h_bins;
	
	
        cvSmooth(&frame, frame_smooth,CV_GAUSSIAN,5,5);
        
        cvCvtColor(frame_smooth, hsv, CV_BGR2HSV);
        
        cvSplit(hsv, h_plane, s_plane, v_plane, NULL);
        cvInRangeS(hsv,Scalar(0,smin,MIN(_vmin,_vmax)),Scalar(180,256,MAX(_vmin,_vmax)),mask);
        
        
        /* Define and set the ROI
         a-b
         | |
         d-c
         */
       
        CvPoint a = cvPoint  (selection.x,selection.y);
        CvPoint b = cvPoint  (selection.x+selection.width, selection.y);
        CvPoint c = cvPoint  (selection.x+selection.width, selection.y+selection.height);
        CvPoint d = cvPoint  (selection.x  ,   selection.y+selection.height);
        
       
       
        
        //Calcule the histograms in h and v planes
        int h_bins =16;
        int v_bins =16;
        int hist_size_h[]={h_bins};
        int hist_size_v[]={v_bins};
        
        float h_range[] = {0,180};
        float v_range[] = {0,255};
        float* range_h[] = {h_range};
        float* range_v[] = {v_range};
        
        
        if(selectObject){
            cvSetImageROI(h_plane, selection);
            cvSetImageROI(v_plane, selection);
	    cvSetImageROI(mask, selection);
            hist_h = cvCreateHist(1, hist_size_h, CV_HIST_ARRAY,range_h,1);
            hist_v = cvCreateHist(1, hist_size_v, CV_HIST_ARRAY,range_v,1);

            
            
            cvCalcHist(plane_h, hist_h,0,mask);
            cvCalcHist(plane_v, hist_v,0,mask);
        
        
        
        
        //Compare all pixels with the histogram in h and s plane of ROI
        
        
        
        cvResetImageROI(h_plane);
        cvResetImageROI(v_plane);
	cvResetImageROI(mask);
        }
        
        //1 Check h plane
     if(trackObject<0 || selectObject){
        for(int y=0;y<h_plane->height;y++){
            uchar* ptr =(uchar*)(h_plane->imageData + y*h_plane->widthStep);
            uchar* ptr_bi_pic = (uchar*)(bi_pic->imageData+y*bi_pic->widthStep);
            for(int x =0; x<h_plane->width;x++){
                int num = (int)ptr[x];
                // cout<<num<<",";
                // cout<<num*h_bins/180<<",";
                //cout<<hist_h->mat.data.fl[num*h_bins/180]<<endl;
                if(  hist_h->mat.data.fl[ num*h_bins/180 ] < _threshold_h_bins){
                   ptr_bi_pic[x] =0;
                    
                    //CvPoint p =  cvPoint(x, y);
                    // cvCircle(frame, p, 1, cvScalar(255,0,0),1);
                }
            }
        }
        
        //2 Check v plane
        
        /*for(int y=0;y<v_plane->height;y++){
            uchar* ptr =(uchar*)(v_plane->imageData + y*v_plane->widthStep);
            uchar* ptr_bi_pic = (uchar*)(bi_pic->imageData+y*bi_pic->widthStep);
            for(int x =0; x<v_plane->width;x++){
                int num = (int)ptr[x];
                if(  hist_v->mat.data.fl[ num*v_bins/255 ] < threshold_v_bin){
                    ptr_bi_pic[x] = 255;
                    //cout<<hist_v->mat.data.fl[ ptr[x]%v_bins ]<<endl;
                    // CvPoint p =  cvPoint(x, y);
                    // cvCircle(frame, p, 1, cvScalar(255,0,0),1);
                }
            }
        }*/
        }
        
        cvAnd(bi_pic,mask,bi_pic);
        
        //Draw the roi
        if(selectObject && selection.width > 0 && selection.height > 0){
            cvLine(&frame, a, b, cvScalar(255,0,0));
            cvLine(&frame, b, c, cvScalar(255,0,0));
            cvLine(&frame, c, d, cvScalar(255,0,0));
            cvLine(&frame, d, a, cvScalar(255,0,0));
        }
        
        //Show histogram
        
        if(backprojMode){
            cvShowImage("frame", bi_pic);
            
        }else{
        
            cvShowImage("frame", &frame);
        }
        
        cvReleaseData(h_plane);
        cvReleaseData(s_plane);
        cvReleaseData(v_plane);
       // cvReleaseData(&frame);
        cvReleaseData(bi_pic);
        cvReleaseData(frame_smooth);
        cvReleaseData(hsv);
	cvReleaseData(mask);
        
        
        
        
        
        
        char k = cvWaitKey(33);
        
        switch(k)
        {
            case 'b':
                backprojMode = !backprojMode;
                break;
         
            default:
                ;
                
        }
        
       
        
        
      


}





//-------------------------------Main Function------------------------------------------
int main(int argc, char **argv){
	
ros::init(argc,argv,"obstacle_hsv");
ros::NodeHandle n_his;

    
    
 
image_transport::Subscriber image_sub;   	
image_transport::ImageTransport it(n_his);
image_sub = it.subscribe("/ardrone/image_raw",1,myobstacle_Cb);
 cvNamedWindow("frame");
 setMouseCallback( "frame", onMouse, 0 );
 createTrackbar("Vmin","frame",&vmin,256,0);
 createTrackbar("Vmax","frame",&vmax,256,0);
 createTrackbar("Smin","frame",&smin,256,0);
 createTrackbar("thres_h","frame",&threshold_h_bins,200,0);
	   

ros::spin();
return 0;
}

