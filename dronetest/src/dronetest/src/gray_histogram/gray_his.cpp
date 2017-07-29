#include <unistd.h>
#include <curses.h>
#include <fstream>
#include <math.h>
#include <iomanip>


#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "gray_his.h"

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

gray_his::gray_his(){
	 
	checker =0;
	timer = 0;
	counter =0;
	image_transport::ImageTransport it(n_his);
	//the constructor of ImageTransport() doesn't exist... so have to use ImageTransport(node)
        image_sub = it.subscribe("/ardrone/image_raw",1,&gray_his::rgbHis_Cb,this);
	
}

gray_his::~gray_his(void){}






void gray_his::image_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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

	
    Mat img_rgb = cv_ptr->image;
    Mat img_gray;
    cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
    IplImage frame = (IplImage)img_rgb;
    

    IplImage* gray;
    IplImage* hist_img;
    
   gray = cvCreateImage(cvGetSize(&frame),8,1);
   cvCvtColor(&frame,gray,CV_RGB2GRAY);
    
    
    

    hist_img = cvCreateImage(cvSize(scale*gray_bins, 300), 8, 3);
    
    CvHistogram* hist;{
        int hist_size[]={gray_bins};
        float g_range[] = {0,255};
        float* ranges[] = {g_range};
        hist = cvCreateHist(1, hist_size, CV_HIST_ARRAY,ranges,1);
    }
    
    cvCalcHist(&gray, hist,0,0);
    
    
    cvRectangle(hist_img, cvPoint(0, 0), cvPoint(hist_img->width, hist_img->height), CV_RGB(255,255,255),CV_FILLED);
    
    float max_value = 0;
    float sum=0;
    int bin_max_index =0;
    cvGetMinMaxHistValue(hist, 0, &max_value,0,0);
    for(int i =0;i<gray_bins;i++){
        sum += hist->mat.data.fl[i];
	if(hist->mat.data.fl[i]==max_value){
		bin_max_index = i;
	}
        
    }
    
    for( int h = 0; h < gray_bins; h++ ) {
        
        float bin_val = hist->mat.data.fl[h];
        
        
        int intensity = cvRound( bin_val*300/sum );
        cvRectangle(
                    hist_img,
                    cvPoint( h*scale, hist_img->height-1),
                    cvPoint( (h+1)*scale - 1, hist_img->height-intensity),
                    cvScalar((int)h*255/gray_bins,(int)h*255/gray_bins,(int)h*255/gray_bins),
                    CV_FILLED
                    );
        
    }
//#################### Record growth ###################

	if(timer%checkTime == 0){
		gray_growth[counter%checkSize] = max_value/sum - lastGrowth;
		counter++;
	//for(int j=0;j<checkSize;j++){cout<<gray_growth[j]<<",";}
	//cout<<endl;
	}

	lastGrowth = max_value/sum;
	for(int i =0;i<checkSize;i++){
		if(gray_growth[i] < 0){
			obstacle_occur = false;
			break;
		}else{obstacle_occur = true;}
	}
	timer++;

	if(obstacle_occur) cout<<"obstacle"<<endl;
	


//#################### Draw obstacle area ################	
 for(int y = 0; y<gray->height;y++){
	uchar* ptr = (uchar*)(gray->imageData + y* gray->widthStep);
	for(int x = 0;x<gray->width;x++){
	    if(ptr[x]< 256*(bin_max_index + 1)/gray_bins && ptr[x]>256*(bin_max_index)/gray_bins){
		CvPoint p = cvPoint(x,y);
		cvCircle(&frame,p,1,cvScalar(255,0,0),1);
		}
	}
    }

  cvNamedWindow("gray_histogram");
 cvShowImage("gray_histogram", hist_img);
    cvNamedWindow("gray");
    cvShowImage("gray", gray);
    cvNamedWindow("color");
    cvShowImage("color", &frame);
      cvWaitKey(33);
      cvReleaseData(gray);
   
        cvReleaseData(hist_img);
    	
	
        
        
        
      


}


void gray_his::rgbHis_Cb(const sensor_msgs::ImageConstPtr& cam_image){

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

  
    frame = (IplImage)img_rgb;
    
  
    

        
        
        
        
        
       // IplImage* hist_img;
        IplImage* r_plane = cvCreateImage(cvGetSize(&frame), IPL_DEPTH_8U, 1);
        IplImage* g_plane = cvCreateImage(cvGetSize(&frame), IPL_DEPTH_8U, 1);
        IplImage* b_plane = cvCreateImage(cvGetSize(&frame), IPL_DEPTH_8U, 1);
        IplImage* planes[] = {r_plane,g_plane,b_plane};
        cvSplit(&frame, b_plane, g_plane, r_plane, NULL);
      
        int r_bins =3;
        int g_bins =3;
        int b_bins =3;
      
        int scale = 10;
       // hist_img = cvCreateImage(cvSize(scale*r_bins*g_bins*b_bins, 300), 8, 3);
        
        CvHistogram* hist;{
            int hist_size[]={r_bins,g_bins,b_bins};
            float r_range[] = {0,255};
            float g_range[] = {0,255};
            float b_range[] = {0,255};
            float* ranges[] = {r_range,g_range,b_range};
            hist = cvCreateHist(3, hist_size, CV_HIST_ARRAY,ranges,1);
        }
        
        cvCalcHist(planes, hist,0,0);
        
        
        //Draw histogram
        //cvRectangle(hist_img, cvPoint(0, 0), cvPoint(hist_img->width, hist_img->height), CV_RGB(255,255,255),CV_FILLED);
        
        float max_value = 0;
        float sum=0;
      int bin_max_index[3]={0,0,0};
        cvGetMinMaxHistValue(hist, 0, &max_value,0,0);
        
        for(int i =0;i<r_bins;i++){
            for(int j =0;j<g_bins;j++){
                for(int k=0;k<b_bins;k++){
                    sum += hist->mat.data.fl[i*g_bins*b_bins+j*b_bins+k];
                    if(hist->mat.data.fl[i*g_bins*b_bins+j*b_bins+k]==max_value){
                        bin_max_index[0] = i;
                        bin_max_index[1] = j;
                        bin_max_index[2] = k;
                    }

                }
            }
            
        }
        
       /* for( int r = 0; r < r_bins; r++ ) {
            for(int g =0;g<g_bins;g++){
                for(int b = 0;b<b_bins;b++){
                    float bin_val = hist->mat.data.fl[r*g_bins*b_bins+g*b_bins+b];
                    
                    
                    int intensity = cvRound( bin_val*6000/sum );
                    cvRectangle(
                                hist_img,
                                cvPoint( (r*g_bins*b_bins+g*b_bins+b)*scale, hist_img->height-1),
                                cvPoint( (r*g_bins*b_bins+g*b_bins+b+1)*scale - 1, hist_img->height-intensity),
                                cvScalar(255,255,255),
                                CV_FILLED
                                );

                    
                }
            }
            
            
        }*/
      
        for(int y=0;y<(&frame)->height;y++){
            uchar* ptr =(uchar*)((&frame)->imageData + y*(&frame)->widthStep);
            for(int x =0; x<(&frame)->widthStep;x++){
                if(
                   
                   (ptr[3*x]< 256*(bin_max_index[0]+1)/r_bins && ptr[3*x] > 256*(bin_max_index[0])/r_bins) &&
                   (ptr[3*x+1]< 256*(bin_max_index[1]+1)/g_bins && ptr[3*x+1] > 256*(bin_max_index[1])/g_bins) &&
                   (ptr[3*x+2]< 256*(bin_max_index[2]+1)/b_bins && ptr[3*x+2] > 256*(bin_max_index[2])/b_bins)
                   
                   ){
                    
                    CvPoint p =  cvPoint(x, y);
                    cvCircle(&frame, p, 1, cvScalar(255,0,0),1);
                }
            }
        }
        //Show histogram
       // cvNamedWindow("gray_histogram");
        //cvShowImage("gray_histogram", hist_img);
        
        cvNamedWindow("frame");
        cvShowImage("frame", &frame);
        
       cvWaitKey(33);
	//cvReleaseData(planes);
        cvReleaseData(r_plane);
        cvReleaseData(g_plane);
        cvReleaseData(b_plane);
        //cvReleaseData(hist_img);
	
        
        
        
        
        
       
        
    
    

}



