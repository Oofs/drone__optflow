#include <unistd.h>
#include <curses.h>
#include <fstream>
#include <math.h>
#include <iomanip>


#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "obstacle_hsv.h"

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



obstacle_hsv::obstacle_hsv(){
	 
	

	image_transport::ImageTransport it(n_his);
	//the constructor of ImageTransport() doesn't exist... so have to use ImageTransport(node)
        image_sub = it.subscribe("/ardrone/image_raw",1,&obstacle_hsv::myobstacle_Cb,this);
	
}

obstacle_hsv::~obstacle_hsv(void){}





void obstacle_hsv::myobstacle_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
    
    cvNamedWindow("ground");
    cvNamedWindow("frame");
    setMouseCallback( "frame", onMouse, 0 );
    
 
    
   
   
        
        
      
            
       
        
        IplImage* hsv = cvCreateImage(cvGetSize(&frame), 8, 3);
        IplImage* frame_smooth = cvCreateImage(cvGetSize(&frame), 8, 3);
        IplImage* h_plane = cvCreateImage(cvGetSize(&frame), 8, 1);
        IplImage* s_plane = cvCreateImage(cvGetSize(&frame), 8, 1);
        IplImage* v_plane = cvCreateImage(cvGetSize(&frame), 8, 1);
        IplImage* bi_pic = cvCreateImage(cvGetSize(&frame),8,1);
        IplImage* plane_h[] = {h_plane};
        IplImage* plane_v[] = {v_plane};
        IplImage* hist_img;
       
        cvSmooth(&frame, frame_smooth,CV_GAUSSIAN,5,5);
        
        cvCvtColor(frame_smooth, hsv, CV_BGR2HSV);
        
        cvSplit(hsv, h_plane, s_plane, v_plane, NULL);
        
        
        
        /* Define and set the ROI
         a-b
         | |
         d-c
         */
       
        CvPoint a = cvPoint  (selection.x,selection.y);
        CvPoint b = cvPoint  (selection.x+selection.width, selection.y);
        CvPoint c = cvPoint  (selection.x+selection.width, selection.y+selection.height);
        CvPoint d = cvPoint  (selection.x  ,   selection.y+selection.height);
        
       
        cvSetImageROI(h_plane, selection);
        cvSetImageROI(v_plane, selection);
        
        //Calcule the histograms in h and v planes
        int h_bins =20;
        int v_bins =20;
        int hist_size_h[]={h_bins};
        int hist_size_v[]={v_bins};
        
        float h_range[] = {0,180};
        float v_range[] = {0,255};
        float* range_h[] = {h_range};
        float* range_v[] = {v_range};
        float threshold_h_bin = 60;
        float threshold_v_bin = 80;
        
        
        if(selectObject){
            hist_h = cvCreateHist(1, hist_size_h, CV_HIST_ARRAY,range_h,1);
            hist_v = cvCreateHist(1, hist_size_v, CV_HIST_ARRAY,range_v,1);

            
            
            cvCalcHist(plane_h, hist_h,0,0);
            cvCalcHist(plane_v, hist_v,0,0);
        
        
        
        
        //Compare all pixels with the histogram in h and s plane of ROI
        
        
        
        cvResetImageROI(h_plane);
        cvResetImageROI(v_plane);
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
                if(  hist_h->mat.data.fl[ num*h_bins/180 ] < threshold_h_bin){
                   ptr_bi_pic[x] =255;
                    
                    //CvPoint p =  cvPoint(x, y);
                    // cvCircle(frame, p, 1, cvScalar(255,0,0),1);
                }
            }
        }
        
        //2 Check v plane
        
        for(int y=0;y<v_plane->height;y++){
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
        }
        }
        
        
        
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
        cvReleaseData(&frame);
        cvReleaseData(bi_pic);
        cvReleaseData(frame_smooth);
        cvReleaseData(hsv);
        
        
        
        
        
        
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



