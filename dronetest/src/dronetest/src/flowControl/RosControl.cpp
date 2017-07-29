#include <unistd.h>
#include <curses.h>
#include <fstream>
#include <math.h>
#include <iomanip>


#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "RosControl.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/opencv.hpp"

const int MAX_CORNERS = 200;
const double pi = 3.1415926;
const int binsize = 8;
const float rate =17.5;// by test
const float f =0.004/0.00000625; //pix
using namespace std;
using namespace cv;

RosControl::RosControl(){
	 forward = ControlCommand(0,-0.2,0,0);
	 backward = ControlCommand(0,0.2,0,0);
	 right = ControlCommand(0.2,0,0,0);
	 left = ControlCommand(-0.2,0,0,0);
	 up = ControlCommand(0,0,0,1);
	 down = ControlCommand(0,0,0,-1);
	 hover = ControlCommand(0,0,0,0);
	clockwise = ControlCommand(0,0,1,0);
	anticlockwise = ControlCommand(0,0,-1,0);
	
	memset(flowRecordX,0,recordSize);
	memset(flowRecordY,0,recordSize);
	times=0;
	takeoff_pub	   = n_.advertise<std_msgs::Empty>(n_.resolveName("ardrone/takeoff"),1);
	land_pub	   = n_.advertise<std_msgs::Empty>(n_.resolveName("ardrone/land"),1);
	vel_pub	   	   = n_.advertise<geometry_msgs::Twist>(n_.resolveName("cmd_vel"),1);
	imu_sub	   	   = n_.subscribe(n_.resolveName("ardrone/imu"),10, &RosControl::imuCb, this);
	flow_pub	   = n_.advertise<std_msgs::Float64MultiArray>("optical_flow_net",100);
	counter =0;
	integraleX=0;
	integraleY=0;
	image_transport::ImageTransport it(n_);
	//the constructor of ImageTransport() doesn't exist... so have to use ImageTransport(node)
        image_sub = it.subscribe("/ardrone/image_raw",1,&RosControl::pykFlowRecAvoidance_Cb,this);
	
}

RosControl::~RosControl(void){}




void RosControl::imuCb(const sensor_msgs::ImuConstPtr imuPtr)
{

	imu.angular_velocity.x = imuPtr->angular_velocity.x;
	imu.angular_velocity.y = imuPtr->angular_velocity.y;
	imu.angular_velocity.z = imuPtr->angular_velocity.z;
	//cout<<times++<<endl;

}

/*
void RosControl::track(int code1,int code2)
{
	int sleeptime = 800000;// sleep time mus
	if(code1 > 150)
	{
		cout<<"in the control! backword"<<endl;
		sendVel(down);
	
		usleep(sleeptime);
		sendVel(hover);
		//usleep(sleeptime);
	};

	if(code2 > 150)
	{
		cout<<"in the control! forward!"<<endl;
		sendVel(forward);
		
		usleep(sleeptime);
		sendVel(hover);
		//usleep(sleeptime);
	};
	
	//sendVel(hover);
}
*/


void RosControl::pubFlow(int code1,int code2)
{
	
	std_msgs::Int64MultiArray bincode;
	
	bincode.data.resize(2);
	bincode.data[0] = code1;
	bincode.data[1] = code2;
	flow_pub.publish(bincode);

}
void RosControl::pubFlow(float code1,float code2)
{
	
	std_msgs::Float64MultiArray bincode;
	
	bincode.data.resize(2);
	bincode.data[0] = code1;
	bincode.data[1] = code2;
	flow_pub.publish(bincode);

}

void RosControl::pykFlowTTC_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
IplImage imgSave = IplImage(img_rgb);

//Image from ardrone
IplImage imgnew= IplImage(img_gray);


//Image from disk
IplImage* imgOld = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
IplImage* imgC = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);
IplImage* imgD = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);

/*   Define the ROI
         a-b
         | |
         d-c
*/
        int width = imgOld->width;
        int height = imgOld->height;
        CvPoint a = cvPoint  (64    ,   100);
        CvPoint b = cvPoint  (576,   100);
        CvPoint c = cvPoint  (576,   300);
        CvPoint d = cvPoint  (64  ,   300);
        
        CvRect Roi= cvRect(a.x  ,   a.y, b.x-a.x,   c.y-b.y);
        
        
       
        
        cvSetImageROI(imgOld, Roi);
        cvSetImageROI(&imgnew, Roi);
        

	CvSize      img_sz    = cvGetSize( imgOld );
	int         win_size = 10;
	//cout<<"width: "<<img_sz.width<<"length: "<<img_sz.height<<endl;
	
	// The first thing we need to do is get the features
	// we want to track.
	//
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];

	cvGoodFeaturesToTrack(
		imgOld,
		eig_image,
		tmp_image,
		cornersA,
		&corner_count,
		0.01,
		5.0,
		0,
		3,
		0,
		0.04
	);
	cvFindCornerSubPix(
		imgOld,
		cornersA,
		corner_count,
		cvSize(win_size,win_size),
		cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
	);

	// Call the Lucas Kanade algorithm
	
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
 	CvSize pyr_sz = cvSize( b.x-a.x+8, (c.y-b.y)/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];

  	cvCalcOpticalFlowPyrLK(
   	  	imgOld,
    	 	&imgnew,
    		pyrA,
     		pyrB,
     		cornersA,
     		cornersB,
     		corner_count,
     		cvSize( win_size,win_size ),
    		 5,
     		features_found,
    		feature_errors,
     		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     		0
  	);





float dp[corner_count][6];
int ind1=0;

  for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>550 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x + a.x ),
                                 cvRound( cornersA[i].y + a.y)
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x + a.x ),
                                 cvRound( cornersB[i].y + a.y)
                                 );
    

 	if( (pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) < 50|| ( pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) > 3000) continue;
            
 	
	float x =(float)p0.x-320;
	float y = (float)p0.y-180;
	float u = (float)-(p1.x-p0.x)*rate;
	float v = (float)-(p1.y-p0.y)*rate;	  
 	dp[ind1][0] = -x;
	dp[ind1][1] = -y;
	dp[ind1][2] = u + y*(float)RTimu.angular_velocity.x - (1/f)*x*y*(float)RTimu.angular_velocity.y + (f + x*x/f)*(float)RTimu.angular_velocity.z;
	dp[ind1][3] = v - x*(float)RTimu.angular_velocity.x - (f+y*y/f)*(float)RTimu.angular_velocity.y + (1/f)*x*y*(float)RTimu.angular_velocity.z;
	dp[ind1][4] = p1.x-p0.x;
	dp[ind1][5] = p1.y-p0.y;	
	ind1++;
	cout<<"u: "<<u<<" after imu: "<<u + y*(float)RTimu.angular_velocity.x - (1/f)*x*y*(float)RTimu.angular_velocity.y + (f + x*x/f)*(float)RTimu.angular_velocity.z<<endl;	
	//cout<<"u Buchuang: "<<y*(float)RTimu.angular_velocity.x - x*y*(float)RTimu.angular_velocity.y + (1 + x*x)*(float)RTimu.angular_velocity.z<<endl;
	    
  }


if(ind1 < 10){
	//cout<<"non answer"<<endl;
}else{
	int ind_a = ind1*2;
	float A_num[ind_a];
	float B_num[ind1];
	float TTC[ind1];
	float posX[ind1],posY[ind1];

	for(int i =0;i<ind1;i++)
	{	
		posX[i] = -dp[i][0]+320;
		posY[i] = -dp[i][1]+180;
		A_num[i*2] = dp[i][3];
		A_num[i*2+1] = -dp[i][2];
		B_num[i] = dp[i][0]*dp[i][3]-dp[i][1]*dp[i][2];
	}
	CvMat* A = cvCreateMat(ind1,2,CV_32FC1);
	CvMat* B = cvCreateMat(ind1,1,CV_32FC1);
	CvMat* x = cvCreateMat(2,1,CV_32FC1);
	CvMat* y = cvCreateMat(2,1,CV_32FC1);
	CvMat* pos_x = cvCreateMat(ind1,1,CV_32FC1);
	CvMat* pos_y = cvCreateMat(ind1,1,CV_32FC1);
	
	CvScalar avgX,sdvX,avgY,sdvY;

	cvSetData(pos_x,posX,CV_AUTOSTEP);
	cvSetData(pos_y,posY,CV_AUTOSTEP);
	cvSetData(A,A_num,CV_AUTOSTEP);
	cvSetData(B,B_num,CV_AUTOSTEP);
	
	cvSolve(A,B,x,CV_SVD);
	cvAvgSdv(pos_x,&avgX,&sdvX);
	cvAvgSdv(pos_y,&avgY,&sdvY);
	//cvSolve(A,B,y,CV_LU);
	//cout<<"svd: "<<x->data.fl[0]<<" lu: "<<y->data.fl[0]<<endl;
	
	CvPoint center = cvPoint( cvRound((int)avgX.val[0]), cvRound((int)avgY.val[0]) );
	if(sdvX.val[0]+sdvY.val[0] <100){
		cvCircle(imgD,center,(int)(sdvX.val[0]+sdvY.val[0]),CV_RGB(255,0,0),1);
	}
	
	float cy =x->data.fl[0];
	float cz =x->data.fl[1];
	float minTTC=20.0;
	for(int i =0;i<ind1;i++){
		float y=dp[i][0];
		float z=dp[i][1];
		float dy = dp[i][2];
		float dz = dp[i][3];		
		TTC[i] = (float) pow((double)( (y-cy)*(y-cy) + (z-cz)*(z-cz) ) / (dy*dy + dz*dz),0.5);
		if(TTC[i]<minTTC){
			minTTC = TTC[i];
		}
	}
	//cout<<TTC[i]<<" "<<endl;

	for(int i=0;i<ind1;i++){
		CvPoint p0 = cvPoint(
                                 cvRound(-(int)dp[i][0]+320 ),
                                 cvRound(-(int)dp[i][1]+180 )
                                 );
		CvPoint p1 = cvPoint(
				     cvRound( (int)(-dp[i][0]-dp[i][2]/rate)+320 ),
				     cvRound( (int)(-dp[i][1]-dp[i][3]/rate)+180 )
					);
		cvLine(imgD,p0,p1,CV_RGB(255,0,0),1 );

 		double angle;
	    CvPoint p2 = cvPoint(cvRound(0.0),cvRound(0.0));
            angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );//线段pq的坡角大小
            p2.x = (int) (p1.x + 5 * cos(angle + pi/6));
            p2.y = (int) (p1.y + 5* sin( angle + pi/6));//获得短线1起点x,y
            
            cvLine( imgD, p2, p1, CV_RGB(255,0,0),1 );//以q为终点绘制第一条箭头短线
            p2.x = (int) (p1.x + 5  * cos(angle - pi / 6));//箭头下短线2，角度为pq角度减少pi/6,长度为5
            p2.y = (int) (p1.y + 5 * sin(angle - pi / 6));//获得短线2起点x,y
            cvLine( imgD, p2, p1, CV_RGB(255,0,0),1 );//画下第二条箭头短线
		char ttc[16];
		int redness = (int)255*minTTC/TTC[i];
		sprintf(ttc,"%.2f",TTC[i]);
		CvFont font;
		cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1.0,0.5,0,1,8);
		cvPutText(imgC,ttc,p0,&font,CV_RGB(redness,0,0));
	}
		
	   
  }


 	
	

	//Draw che Roi
  	cvLine(imgC, a, b, cvScalar(255,0,0));
        cvLine(imgC, b, c, cvScalar(255,0,0));
        cvLine(imgC, c, d, cvScalar(255,0,0));
        cvLine(imgC, d, a, cvScalar(255,0,0));
 	
  cvNamedWindow("LKpyr_OpticalFlow",0);
  cvShowImage("LKpyr_OpticalFlow",imgD);
   cvNamedWindow("TTC",0); 
  cvShowImage("TTC",imgC);	



//save the frames
        char address[]="/opt/pic/";
        char num[8];
        char suffix[8]=".jpg";
        char name[100];
        int number =1;
	
        
        sprintf(num,"%d",number);
        sprintf(name,"%s%s%s",address,num,suffix);
      
        cvSaveImage(name, &imgSave);//save the rgb_image
	RTimu.angular_velocity.x = imu.angular_velocity.x;
	RTimu.angular_velocity.y = imu.angular_velocity.y;
	RTimu.angular_velocity.z = imu.angular_velocity.z;
//cvShowImage(WINDOW,imgOld);
//cvShowImage(WINDOW2,imgNew);

// Can cause memory leak, 'new','cvload','cvCreate','cvcopy'

	cvReleaseData(imgOld);/// IMPORTANT!!!!!
       //cvReleaseData(imgNew);
	cvReleaseData(imgC);
	cvReleaseData(imgD);
        cvReleaseData(pyrA);
        cvReleaseData(pyrB);
	delete cornersA;
	delete cornersB;
        cvReleaseData(eig_image);
        cvReleaseData(tmp_image);

cvWaitKey(33);


}

void RosControl::pykFlowRecTTC_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
IplImage imgSave = IplImage(img_rgb);

//Image from ardrone
IplImage imgnew= IplImage(img_gray);


//Image from disk

//cout<<times++<<endl;

IplImage* imgOld = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
IplImage* imgC = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);
IplImage* imgD = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);

//########## imu data should be real time with picture #########

/*   Define the ROI
         a-b
         | |
         d-c
*/
        int width = imgOld->width;
        int height = imgOld->height;
        CvPoint a = cvPoint  (0*width/10 +1   ,   1*height/10);
        CvPoint b = cvPoint  (10*width/10-1,   1*height/10);
        CvPoint c = cvPoint  (10*width/10-1,   9*height/10);
        CvPoint d = cvPoint  (0*width/10 +1 ,   9*height/10);
        
        CvRect Roi= cvRect(a.x  ,   a.y, b.x-a.x,   c.y-b.y);
        
        
       
        
        cvSetImageROI(imgOld, Roi);
        cvSetImageROI(&imgnew, Roi);
        

	CvSize      img_sz    = cvGetSize( imgOld );
	int         win_size = 10;
	//cout<<"width: "<<img_sz.width<<"length: "<<img_sz.height<<endl;
	
	// The first thing we need to do is get the features
	// we want to track.
	//
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];
	
	//Set the ROI rectangle
	int roiSize =(b.x-a.x)*(c.y-b.y); 
	int scale = (int) pow(roiSize/MAX_CORNERS,0.5);
	int widthStep = scale+1;
	int hightStep = scale+1;
	int k=0;
	for(int i = hightStep; i<c.y-a.y ;i += hightStep)
	{
		for(int j = widthStep; j<b.x-a.x ; j += widthStep)
		{
			cornersA[k].x = (float)j;
			cornersA[k].y = (float)i;
			k++;
		}
	} 
	//cout<<k<<endl;
	

	// Call the Lucas Kanade algorithm
	
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
 	CvSize pyr_sz = cvSize( b.x-a.x+8, (c.y-b.y)/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];

  	cvCalcOpticalFlowPyrLK(
   	  	imgOld,
    	 	&imgnew,
    		pyrA,
     		pyrB,
     		cornersA,
     		cornersB,
     		corner_count,
     		cvSize( win_size,win_size ),
    		 5,
     		features_found,
    		feature_errors,
     		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     		0
  	);


  // Now make some image of what we are looking at:


float dp[corner_count][6];
int ind1=0;


  for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>400 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x+a.x  ),
                                 cvRound( cornersA[i].y+a.y )
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x+a.x ),
                                 cvRound( cornersB[i].y+a.y )
                                 );
    

 	if( (pow(double(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) < 50|| ( pow(double(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) > 1000) continue;
		
          

	float x =(float)p0.x-320;
	float y = (float)p0.y-180;
	float u = (float)-(p1.x-p0.x)*rate;
	float v = (float)-(p1.y-p0.y)*rate;	  
 	dp[ind1][0] = -x;
	dp[ind1][1] = -y;
	dp[ind1][2] = u + y*(float)RTimu.angular_velocity.x - x*y*(float)RTimu.angular_velocity.y + (1 + x*x)*(float)RTimu.angular_velocity.z;
	dp[ind1][3] = v - x*(float)RTimu.angular_velocity.x - (1+y*y)*(float)RTimu.angular_velocity.y + x*y*(float)RTimu.angular_velocity.z;
	dp[ind1][4] = p1.x-p0.x;
	dp[ind1][5] = p1.y-p0.y;	
	ind1++;

	    
  }


if(ind1 < 10){
	//cout<<"non answer"<<endl;
}else{
	int ind_a = ind1*2;
	float A_num[ind_a];
	float B_num[ind1];
	float TTC[ind1];


	for(int i =0;i<ind1;i++)
	{	
		A_num[i*2] = dp[i][3];
		A_num[i*2+1] = -dp[i][2];
		B_num[i] = dp[i][0]*dp[i][3]-dp[i][1]*dp[i][2];
	}
	CvMat* A = cvCreateMat(ind1,2,CV_32FC1);
	CvMat* B = cvCreateMat(ind1,1,CV_32FC1);
	CvMat* x = cvCreateMat(2,1,CV_32FC1);
	CvMat* y = cvCreateMat(2,1,CV_32FC1);

	cvSetData(A,A_num,CV_AUTOSTEP);
	cvSetData(B,B_num,CV_AUTOSTEP);
	cvSolve(A,B,x,CV_SVD);
	//cvSolve(A,B,y,CV_LU);
	//cout<<"svd: "<<x->data.fl[0]<<" lu: "<<y->data.fl[0]<<endl;
	float cy =x->data.fl[0];
	float cz =x->data.fl[1];
	float minTTC=20.0;
	for(int i =0;i<ind1;i++){
		float y=dp[i][0];
		float z=dp[i][1];
		float dy = dp[i][2];
		float dz = dp[i][3];		
		TTC[i] = (float) pow((double)( (y-cy)*(y-cy) + (z-cz)*(z-cz) ) / (dy*dy + dz*dz),0.5);
		if(TTC[i]<minTTC){
			minTTC = TTC[i];
		}
	}
	//cout<<TTC[i]<<" "<<endl;

	for(int i=0;i<ind1;i++){
		CvPoint p0 = cvPoint(
                                 cvRound(-(int)dp[i][0]+320 ),
                                 cvRound(-(int)dp[i][1]+180 )
                                 );
		CvPoint p1 = cvPoint(
				     cvRound( (int)(-dp[i][0]+dp[i][4])+320 ),
				     cvRound( (int)(-dp[i][1]+dp[i][5])+180 )
					);
		cvLine(imgD,p0,p1,cvScalar(255,0,0));
		char ttc[16];
		int redness = (int)255*minTTC/TTC[i];
		sprintf(ttc,"%.2f",TTC[i]);
		CvFont font;
		cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1.0,0.5,0,1,8);
		cvPutText(imgC,ttc,p0,&font,CV_RGB(redness,0,0));
	}
		


	cvReleaseData(A);
	cvReleaseData(B);
	cvReleaseData(x);
}

 	   


	
	
	

	//Draw che Roi
  	cvLine(imgC, a, b, cvScalar(255,0,0));
        cvLine(imgC, b, c, cvScalar(255,0,0));
        cvLine(imgC, c, d, cvScalar(255,0,0));
        cvLine(imgC, d, a, cvScalar(255,0,0));
 	
  cvNamedWindow("TTC",0);
  cvShowImage("TTC",imgC); 
  cvNamedWindow("LKpyr_OpticalFlow",0);
  cvShowImage("LKpyr_OpticalFlow",imgD);

 




//save the frames
        char address[]="/opt/pic/";
        char num[8];
        char suffix[8]=".jpg";
        char name[100];
        int number =1;
	
        
        sprintf(num,"%d",number);
        sprintf(name,"%s%s%s",address,num,suffix);
      
        cvSaveImage(name, &imgSave);//save the rgb_image
	RTimu.angular_velocity.x = imu.angular_velocity.x;
	RTimu.angular_velocity.y = imu.angular_velocity.y;
	RTimu.angular_velocity.z = imu.angular_velocity.z;

	
//cvShowImage(WINDOW,imgOld);
//cvShowImage(WINDOW2,imgNew);

// Can cause memory leak, 'new','cvload','cvCreate','cvcopy'

	cvReleaseData(imgOld);/// IMPORTANT!!!!!
       //cvReleaseData(imgNew);
	cvReleaseData(imgC);
	cvReleaseData(imgD);
        cvReleaseData(pyrA);
        cvReleaseData(pyrB);
	delete cornersA;
	delete cornersB;
	
        cvReleaseData(eig_image);
        cvReleaseData(tmp_image);

cvWaitKey(33);


}




//####################################### pykFLowAvoidance#################################
void RosControl::pykFlowAvoidance_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
//------Initialize for histogram-----
int his_max[1000];
int bincode;
int his[binsize];
int line =0;
memset(his,0,sizeof(his));
memset(his_max,0,sizeof(his_max));


Mat img_rgb = cv_ptr->image;
Mat img_gray;

cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
IplImage imgSave = IplImage(img_rgb);

//Image from ardrone
IplImage imgnew= IplImage(img_gray);


//Image from disk
IplImage* imgOld = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
IplImage* imgC = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);


/*   Define the ROI
         a-b
         | |
         d-c
*/
        int width = imgOld->width;
        int height = imgOld->height;
        CvPoint a = cvPoint  (64    ,   100);
        CvPoint b = cvPoint  (576,   100);
        CvPoint c = cvPoint  (576,   300);
        CvPoint d = cvPoint  (64  ,   300);
        
        CvRect Roi= cvRect(a.x  ,   a.y, b.x-a.x,   c.y-b.y);
        
        
       
        
        cvSetImageROI(imgOld, Roi);
        cvSetImageROI(&imgnew, Roi);
        

	CvSize      img_sz    = cvGetSize( imgOld );
	int         win_size = 10;
	//cout<<"width: "<<img_sz.width<<"length: "<<img_sz.height<<endl;
	
	// The first thing we need to do is get the features
	// we want to track.
	//
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];

	cvGoodFeaturesToTrack(
		imgOld,
		eig_image,
		tmp_image,
		cornersA,
		&corner_count,
		0.01,
		5.0,
		0,
		3,
		0,
		0.04
	);
	cvFindCornerSubPix(
		imgOld,
		cornersA,
		corner_count,
		cvSize(win_size,win_size),
		cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
	);

	// Call the Lucas Kanade algorithm
	
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
 	CvSize pyr_sz = cvSize( b.x-a.x+8, (c.y-b.y)/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];

  	cvCalcOpticalFlowPyrLK(
   	  	imgOld,
    	 	&imgnew,
    		pyrA,
     		pyrB,
     		cornersA,
     		cornersB,
     		corner_count,
     		cvSize( win_size,win_size ),
    		 5,
     		features_found,
    		feature_errors,
     		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     		0
  	);




  // Now make some image of what we are looking at:
  float meanX_right =0;
 float meanX_left = 0;
 float count_right =0;
 float count_left =0;
  for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>550 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x + a.x ),
                                 cvRound( cornersA[i].y + a.y)
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x + a.x ),
                                 cvRound( cornersB[i].y + a.y)
                                 );
    

 	if( (pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) < 30|| ( pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) > 10000) continue;
            
 	if((p0.x>384 && p0.x<576) && (p0.y>100 && p0.y<300))
	    {
			meanX_right += float (p1.x - p0.x);
			count_right++;
	    }else if((p0.x>64 && p0.x<256) && (p0.y>100 && p0.y<300)){
			meanX_left  += float (p1.x - p0.x);
			count_left++;
		 }else{continue;}
	    //Draw line
	    cvLine( imgC, p0, p1, CV_RGB(255,0,0),1);
      
            //画箭头
            double angle;
	    CvPoint p2 = cvPoint(cvRound(0.0),cvRound(0.0));
            angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );//线段pq的坡角大小
            p2.x = (int) (p1.x + 5 * cos(angle + pi/6));
            p2.y = (int) (p1.y + 5* sin( angle + pi/6));//获得短线1起点x,y
            
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//以q为终点绘制第一条箭头短线
            p2.x = (int) (p1.x + 5  * cos(angle - pi / 6));//箭头下短线2，角度为pq角度减少pi/6,长度为5
            p2.y = (int) (p1.y + 5 * sin(angle - pi / 6));//获得短线2起点x,y
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//画下第二条箭头短线


	   
  }
	

 	if(count_right != 0 && count_right > 10){
	   	 meanX_right = meanX_right / count_right;
	   	 
	       }else{meanX_right = 0;}
		
	    if (count_left != 0 && count_left >10){
		  meanX_left  = meanX_left / count_left;
		}else{meanX_left = 0;}
	
	    pubFlow(meanX_left,meanX_right);
		cout<<"right-left : "<<meanX_right-meanX_left<<endl;

	

	//Draw che Roi
  	cvLine(imgC, a, b, cvScalar(255,0,0));
        cvLine(imgC, b, c, cvScalar(255,0,0));
        cvLine(imgC, c, d, cvScalar(255,0,0));
        cvLine(imgC, d, a, cvScalar(255,0,0));
 	
  cvNamedWindow("LKpyr_OpticalFlow",0);
  cvShowImage("LKpyr_OpticalFlow",imgC);




//save the frames
        char address[]="/opt/pic/";
        char num[8];
        char suffix[8]=".jpg";
        char name[100];
        int number =1;
	
        
        sprintf(num,"%d",number);
        sprintf(name,"%s%s%s",address,num,suffix);
      
        cvSaveImage(name, &imgSave);//save the rgb_image

//cvShowImage(WINDOW,imgOld);
//cvShowImage(WINDOW2,imgNew);

// Can cause memory leak, 'new','cvload','cvCreate','cvcopy'

	cvReleaseData(imgOld);/// IMPORTANT!!!!!
       //cvReleaseData(imgNew);
	cvReleaseData(imgC);
        cvReleaseData(pyrA);
        cvReleaseData(pyrB);
	delete cornersA;
	delete cornersB;
        cvReleaseData(eig_image);
        cvReleaseData(tmp_image);

cvWaitKey(33);


}

void RosControl::pykFlowRecAvoidance_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
//------Initialize for histogram-----
int his_max[1000];
int bincode;
int his[binsize];
int line =0;
memset(his,0,sizeof(his));
memset(his_max,0,sizeof(his_max));


Mat img_rgb = cv_ptr->image;
Mat img_gray;

cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
IplImage imgSave = IplImage(img_rgb);

//Image from ardrone
IplImage imgnew= IplImage(img_gray);


//Image from disk
IplImage* imgOld = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
IplImage* imgC = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);


/*   Define the ROI
         a-b
         | |
         d-c
*/
        int width = imgOld->width;
        int height = imgOld->height;
        CvPoint a = cvPoint  (0*width/10 +1   ,   1*height/10);
        CvPoint b = cvPoint  (10*width/10-1,   1*height/10);
        CvPoint c = cvPoint  (10*width/10-1,   9*height/10);
        CvPoint d = cvPoint  (0*width/10 +1 ,   9*height/10);
        
        CvRect Roi= cvRect(a.x  ,   a.y, b.x-a.x,   c.y-b.y);
        
        
       
        
        cvSetImageROI(imgOld, Roi);
        cvSetImageROI(&imgnew, Roi);
        

	CvSize      img_sz    = cvGetSize( imgOld );
	int         win_size = 10;
	//cout<<"width: "<<img_sz.width<<"length: "<<img_sz.height<<endl;
	
	// The first thing we need to do is get the features
	// we want to track.
	//
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];
	
	//Set the ROI rectangle
	int roiSize =(b.x-a.x)*(c.y-b.y); 
	int scale = (int) pow(roiSize/MAX_CORNERS,0.5);
	int widthStep = scale+1;
	int hightStep = scale+1;
	int k=0;
	for(int i = hightStep; i<c.y-a.y ;i += hightStep)
	{
		for(int j = widthStep; j<b.x-a.x ; j += widthStep)
		{
			cornersA[k].x = (float)j;
			cornersA[k].y = (float)i;
			k++;
		}
	} 
	//cout<<k<<endl;
	

	// Call the Lucas Kanade algorithm
	
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
 	CvSize pyr_sz = cvSize( b.x-a.x+8, (c.y-b.y)/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];

  	cvCalcOpticalFlowPyrLK(
   	  	imgOld,
    	 	&imgnew,
    		pyrA,
     		pyrB,
     		cornersA,
     		cornersB,
     		corner_count,
     		cvSize( win_size,win_size ),
    		 5,
     		features_found,
    		feature_errors,
     		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     		0
  	);


  // Now make some image of what we are looking at:
float meanX_right =0;
 float meanX_left = 0;
 float count_right =0;
 float count_left =0;
int checker=0;
  for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>400 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x+a.x  ),
                                 cvRound( cornersA[i].y+a.y )
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x+a.x ),
                                 cvRound( cornersB[i].y+a.y )
                                 );
    

 	if( (pow(double(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) < 50|| ( pow(double(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) > 1000) continue;
		
            if((p0.x>384 && p0.x<640) && (p0.y>100 && p0.y<260))
	    {
			meanX_right += float (p1.x - p0.x);
			count_right++;
	    }else if((p0.x>0 && p0.x<256) && (p0.y>100 && p0.y<260)){
			meanX_left  += float (p1.x - p0.x);
			count_left++;
		 }else{continue;}
	checker++;
	   

		

	    //Draw line
	    cvLine( imgC, p0, p1, CV_RGB(255,0,0),1);
            
            //画箭头
            double angle;
	    CvPoint p2 = cvPoint(cvRound(0.0),cvRound(0.0));
            angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );//线段pq的坡角大小
            p2.x = (int) (p1.x + 5 * cos(angle + pi/6));
            p2.y = (int) (p1.y + 5* sin( angle + pi/6));//获得短线1起点x,y
            
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//以q为终点绘制第一条箭头短线
            p2.x = (int) (p1.x + 5  * cos(angle - pi / 6));//箭头下短线2，角度为pq角度减少pi/6,长度为5
            p2.y = (int) (p1.y + 5 * sin(angle - pi / 6));//获得短线2起点x,y
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//画下第二条箭头短线


	    //calculate the histogram of optical flow
            
            bincode = (int) binsize/2 + floor(angle/(2*pi/binsize));  //floor(1.4)=1
           
 	    his[bincode]++;
	    
  }
		//cout<<checker<<endl;
 	    if(count_right != 0 && count_right > 15){
	   	 meanX_right = meanX_right / count_right;
	   	 
	       }else{meanX_right = 0;}
		
	    if (count_left != 0 && count_left >15){
		  meanX_left  = meanX_left / count_left;
		}else{meanX_left = 0;}
	
	    pubFlow(meanX_left,meanX_right);
		cout<<"left-right: "<<meanX_left-meanX_right<<endl;



	
	
	

	//Draw che Roi
  	cvLine(imgC, a, b, cvScalar(255,0,0));
        cvLine(imgC, b, c, cvScalar(255,0,0));
        cvLine(imgC, c, d, cvScalar(255,0,0));
        cvLine(imgC, d, a, cvScalar(255,0,0));
 	
  cvNamedWindow("LKpyr_OpticalFlow",0);
  cvShowImage("LKpyr_OpticalFlow",imgC);




//save the frames
        char address[]="/opt/pic/";
        char num[8];
        char suffix[8]=".jpg";
        char name[100];
        int number =1;
	
        
        sprintf(num,"%d",number);
        sprintf(name,"%s%s%s",address,num,suffix);
      
        cvSaveImage(name, &imgSave);//save the rgb_image

//cvShowImage(WINDOW,imgOld);
//cvShowImage(WINDOW2,imgNew);

// Can cause memory leak, 'new','cvload','cvCreate','cvcopy'

	cvReleaseData(imgOld);/// IMPORTANT!!!!!
       //cvReleaseData(imgNew);
	cvReleaseData(imgC);
        cvReleaseData(pyrA);
        cvReleaseData(pyrB);
	delete cornersA;
	delete cornersB;
        cvReleaseData(eig_image);
        cvReleaseData(tmp_image);

cvWaitKey(33);


}
























//-----------------------------------------pykFlowRec_Cb-----------------------------------------
void RosControl::pykFlowRec_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
//------Initialize for histogram-----
int his_max[1000];
int bincode;
int his[binsize];
int line =0;
memset(his,0,sizeof(his));
memset(his_max,0,sizeof(his_max));


Mat img_rgb = cv_ptr->image;
Mat img_gray;

cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
IplImage imgSave = IplImage(img_rgb);

//Image from ardrone
IplImage imgnew= IplImage(img_gray);


//Image from disk
IplImage* imgOld = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
IplImage* imgC = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);


/*   Define the ROI
         a-b
         | |
         d-c
*/
        int width = imgOld->width;
        int height = imgOld->height;
        CvPoint a = cvPoint  (1*width/5    ,   1*height/8);
        CvPoint b = cvPoint  (4*width/5,   1*height/8);
        CvPoint c = cvPoint  (4*width/5,   7*height/8);
        CvPoint d = cvPoint  (1*width/5  ,   7*height/8);
        
        CvRect Roi= cvRect(a.x  ,   a.y, b.x-a.x,   c.y-b.y);
        
        
       
        
        cvSetImageROI(imgOld, Roi);
        cvSetImageROI(&imgnew, Roi);
        

	CvSize      img_sz    = cvGetSize( imgOld );
	int         win_size = 10;
	//cout<<"width: "<<img_sz.width<<"length: "<<img_sz.height<<endl;
	
	// The first thing we need to do is get the features
	// we want to track.
	//
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];
	
	//Set the ROI rectangle
	int roiSize =(b.x-a.x)*(c.y-b.y); 
	int scale = (int) pow(roiSize/MAX_CORNERS,0.5);
	int widthStep = scale+1;
	int hightStep = scale+1;
	int k=0;
	for(int i = hightStep; i<c.y-a.y ;i += hightStep)
	{
		for(int j = widthStep; j<b.x-a.x ; j += widthStep)
		{
			cornersA[k].x = (float)j;
			cornersA[k].y = (float)i;
			k++;
		}
	} 
	//cout<<k<<endl;
	

	// Call the Lucas Kanade algorithm
	
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
 	CvSize pyr_sz = cvSize( b.x-a.x+8, (c.y-b.y)/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];

  	cvCalcOpticalFlowPyrLK(
   	  	imgOld,
    	 	&imgnew,
    		pyrA,
     		pyrB,
     		cornersA,
     		cornersB,
     		corner_count,
     		cvSize( win_size,win_size ),
    		 5,
     		features_found,
    		feature_errors,
     		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     		0
  	);


  // Now make some image of what we are looking at:

  for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>550 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x+a.x  ),
                                 cvRound( cornersA[i].y+a.y )
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x+a.x ),
                                 cvRound( cornersB[i].y+a.y )
                                 );
    

 	if( (pow(double(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) < 50|| ( pow(double(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) > 2000) continue;
            //------------------------Body rotation optical flow compensation----------------------
	/*	int y= p1.y - 320;
		int x = p1.x -160;
		p1.x = p1.x + (int)( -40*imu.angular_velocity.z );
		p1.y = p1.y + (int)( 50*imu.angular_velocity.y  );
	//if( (pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) < 50) continue;
		//cout<<"X compensation "<< imu.angular_velocity.y * focal_length/1000 + imu.angular_velocity.z * y<<" int X "<<(int)( imu.angular_velocity.y * focal_length/1000 + imu.angular_velocity.z * y)<<endl;*/
	    //Draw line
	    cvLine( imgC, p0, p1, CV_RGB(255,0,0),1);
            
            //画箭头
            double angle;
	    CvPoint p2 = cvPoint(cvRound(0.0),cvRound(0.0));
            angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );//线段pq的坡角大小
            p2.x = (int) (p1.x + 5 * cos(angle + pi/6));
            p2.y = (int) (p1.y + 5* sin( angle + pi/6));//获得短线1起点x,y
            
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//以q为终点绘制第一条箭头短线
            p2.x = (int) (p1.x + 5  * cos(angle - pi / 6));//箭头下短线2，角度为pq角度减少pi/6,长度为5
            p2.y = (int) (p1.y + 5 * sin(angle - pi / 6));//获得短线2起点x,y
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//画下第二条箭头短线


	    //calculate the histogram of optical flow
            
            bincode = (int) binsize/2 + floor(angle/(2*pi/binsize));  //floor(1.4)=1
           
 	    his[bincode]++;
	    
  }



//------------------------Find the max bincode---------------------
int maxbincode=0;
for(int i=0;i<binsize;i++){
	if(his[i]>maxbincode) maxbincode = i;
}


//------------------------Caculate Vx and Vy from the optical flow in the max bincode orientation----------------------

int meanCount =0; 
int meanX=0,meanY=0;
for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>550 ) {
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x + a.x ),
                                 cvRound( cornersA[i].y + a.y )
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x + a.x ),
                                 cvRound( cornersB[i].y + a.y )
                                 );
    

 	if( (pow((double)(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) < 50|| ( pow(double(p0.x-p1.x),2)+pow(double(p0.y-p1.y),2) ) > 2000) continue;
	double angle;
	int bincode_c;
	
	angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );
	bincode_c = (int) binsize/2 + floor(angle/(2*pi/binsize)); 

	if(bincode_c == maxbincode){
		meanX += (p0.x-p1.x);
		meanY += (p0.y-p1.y);
		meanCount++;
	}
}


if(meanCount == 0)
{
	//cout<<"can't find maxbinode"<<endl;
}else{
	meanX /= meanCount;
	meanY /= meanCount;
}
		

int numbers = counter%10;
if (meanCount >10)
{
	flowRecordX[numbers] = meanX;
	flowRecordY[numbers] = meanY;
}else{
	flowRecordX[numbers] = 0;
	flowRecordY[numbers] = 0;
     }
counter++;

int sumFlowX =0;
int sumFlowY =0;
for(int i=0;i<recordSize;i++)
{
	sumFlowX += flowRecordX[i];
	sumFlowY += flowRecordY[i];
	
}
sumFlowX /= recordSize;
sumFlowY /= recordSize;


if(abs(sumFlowX) < 2000 && abs(sumFlowY)<2000)
{

integraleX += sumFlowX;
integraleY += sumFlowY;

}
//cout<<"x: "<<integraleX<<" y: "<<integraleY<<endl;


pubFlow(1,-1);
//pubFlow(integraleX,integraleY);

	
	/*
	for(int i =0;i<binsize;i++){
	
	cout<<his[i]<<",";
	}
	cout<<endl;
	*/

	//cout<<"0+1:"<<his[0]+his[1]<<" 1+2:"<<his[1]+his[2]<<" 2+3:"<<his[2]+his[3]<<" 3+0:"<<his[3]+his[0]<<endl;

	//Save opticalflow to desk
	/*
	ofstream f1("/opt/optical_flow/flow.txt");
	if(!f1) cout<<"write file failed!"<<endl;
	f1<<his[0]<<" "<<his[1];
	f1.close();
	*/
	
	//Publish optical flow message

	/*
	if(meanCount>10 )
	{
		this->pubFlow(meanX,meanY);
	}else{
		this->pubFlow(0,0);
	     }
	*/
	

	//Draw che Roi
  	cvLine(imgC, a, b, cvScalar(255,0,0));
        cvLine(imgC, b, c, cvScalar(255,0,0));
        cvLine(imgC, c, d, cvScalar(255,0,0));
        cvLine(imgC, d, a, cvScalar(255,0,0));
 	
  cvNamedWindow("LKpyr_OpticalFlow",0);
  cvShowImage("LKpyr_OpticalFlow",imgC);




//save the frames
        char address[]="/opt/pic/";
        char num[8];
        char suffix[8]=".jpg";
        char name[100];
        int number =1;
	
        
        sprintf(num,"%d",number);
        sprintf(name,"%s%s%s",address,num,suffix);
      
        cvSaveImage(name, &imgSave);//save the rgb_image

//cvShowImage(WINDOW,imgOld);
//cvShowImage(WINDOW2,imgNew);

// Can cause memory leak, 'new','cvload','cvCreate','cvcopy'

	cvReleaseData(imgOld);/// IMPORTANT!!!!!
       //cvReleaseData(imgNew);
	cvReleaseData(imgC);
        cvReleaseData(pyrA);
        cvReleaseData(pyrB);
	delete cornersA;
	delete cornersB;
        cvReleaseData(eig_image);
        cvReleaseData(tmp_image);

cvWaitKey(33);


}



void RosControl::pykFlowRecAvg_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
//------Initialize for histogram-----
int his_max[1000];
int bincode;
int his[binsize];
int line =0;
memset(his,0,sizeof(his));
memset(his_max,0,sizeof(his_max));


Mat img_rgb = cv_ptr->image;
Mat img_gray;

cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
IplImage imgSave = IplImage(img_rgb);

//Image from ardrone
IplImage imgnew= IplImage(img_gray);


//Image from disk
IplImage* imgOld = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
IplImage* imgC = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);


/*   Define the ROI
         a-b
         | |
         d-c
*/
        int width = imgOld->width;
        int height = imgOld->height;
        CvPoint a = cvPoint  (width/3    ,   height/3);
        CvPoint b = cvPoint  (2*width/3,   height/3);
        CvPoint c = cvPoint  (2*width/3,   2*height/3);
        CvPoint d = cvPoint  (width/3  ,   2*height/3);
        
        CvRect Roi= cvRect(a.x  ,   a.y, b.x-a.x,   c.y-b.y);
        
        
       
        
        cvSetImageROI(imgOld, Roi);
        cvSetImageROI(&imgnew, Roi);
        

	CvSize      img_sz    = cvGetSize( imgOld );
	int         win_size = 10;
	//cout<<"width: "<<img_sz.width<<"length: "<<img_sz.height<<endl;
	
	// The first thing we need to do is get the features
	// we want to track.
	//
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];
	
	//Set the ROI rectangle
	int roiSize =(b.x-a.x)*(c.y-b.y); 
	int scale = (int) pow(roiSize/MAX_CORNERS,0.5);
	int widthStep = scale+1;
	int hightStep = scale+1;
	int k=0;
	for(int i = hightStep; i<c.y-a.y ;i += hightStep)
	{
		for(int j = widthStep; j<b.x-a.x ; j += widthStep)
		{
			cornersA[k].x = (float)j;
			cornersA[k].y = (float)i;
			k++;
		}
	} 
	//cout<<k<<endl;
	

	// Call the Lucas Kanade algorithm
	
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
 	CvSize pyr_sz = cvSize( b.x-a.x+8, (c.y-b.y)/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];

  	cvCalcOpticalFlowPyrLK(
   	  	imgOld,
    	 	&imgnew,
    		pyrA,
     		pyrB,
     		cornersA,
     		cornersB,
     		corner_count,
     		cvSize( win_size,win_size ),
    		 5,
     		features_found,
    		feature_errors,
     		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     		0
  	);


  // Now make some image of what we are looking at and calculate the average optical flow
 int meanX = 0;
 int meanY = 0;
 int meanCount = 0;
  for( int i=0; i<corner_count; i++ ) {
	
     if( features_found[i]==0 || feature_errors[i]>550 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x+a.x  ),
                                 cvRound( cornersA[i].y+a.y )
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x+a.x ),
                                 cvRound( cornersB[i].y+a.y )
                                 );
    

 	if( (pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) < 50|| ( pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) > 1000) continue;
            
	    //Draw line
	    cvLine( imgC, p0, p1, CV_RGB(255,0,0),1);
            
            //画箭头
            double angle;
	    CvPoint p2 = cvPoint(cvRound(0.0),cvRound(0.0));
            angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );//线段pq的坡角大小
            p2.x = (int) (p1.x + 5 * cos(angle + pi/6));
            p2.y = (int) (p1.y + 5* sin( angle + pi/6));//获得短线1起点x,y
            
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//以q为终点绘制第一条箭头短线
            p2.x = (int) (p1.x + 5  * cos(angle - pi / 6));//箭头下短线2，角度为pq角度减少pi/6,长度为5
            p2.y = (int) (p1.y + 5 * sin(angle - pi / 6));//获得短线2起点x,y
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//画下第二条箭头短线


	    //calculate the histogram of optical flow
            
            bincode = (int) binsize/2 + floor(angle/(2*pi/binsize));  //floor(1.4)=1
           
 	    his[bincode]++;

	    meanX += p0.x-p1.x;
	    meanY += p0.y-p1.y;
	    meanCount++;
	    
  }



if(meanCount == 0)
{
	//cout<<"Dnt find maxbinode"<<endl;
}else{
	meanX /= meanCount;
	meanY /= meanCount;
}
		
//----------------------------------------Publish the flow-----------------------
	this->pubFlow(meanX,meanY);
	

	//Draw che Roi
  	cvLine(imgC, a, b, cvScalar(255,0,0));
        cvLine(imgC, b, c, cvScalar(255,0,0));
        cvLine(imgC, c, d, cvScalar(255,0,0));
        cvLine(imgC, d, a, cvScalar(255,0,0));
 	
  cvNamedWindow("LKpyr_OpticalFlow",0);
  cvShowImage("LKpyr_OpticalFlow",imgC);




//save the frames
        char address[]="/opt/pic/";
        char num[8];
        char suffix[8]=".jpg";
        char name[100];
        int number =1;
	
        
        sprintf(num,"%d",number);
        sprintf(name,"%s%s%s",address,num,suffix);
      
        cvSaveImage(name, &imgSave);//save the rgb_image

//cvShowImage(WINDOW,imgOld);
//cvShowImage(WINDOW2,imgNew);

// Can cause memory leak, 'new','cvload','cvCreate','cvcopy'

	cvReleaseData(imgOld);/// IMPORTANT!!!!!
       //cvReleaseData(imgNew);
	cvReleaseData(imgC);
        cvReleaseData(pyrA);
        cvReleaseData(pyrB);
	delete cornersA;
	delete cornersB;
        cvReleaseData(eig_image);
        cvReleaseData(tmp_image);

cvWaitKey(33);


}



//---------------------------------------------------pykPlow_Cb-------------------------------------------------------
void RosControl::pykFlow_Cb(const sensor_msgs::ImageConstPtr& cam_image)
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
//------Initialize for histogram-----
int his_max[1000];
int bincode;
int his[binsize];
int line =0;
memset(his,0,sizeof(his));
memset(his_max,0,sizeof(his_max));


Mat img_rgb = cv_ptr->image;
Mat img_gray;

cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
IplImage imgSave = IplImage(img_rgb);

//Image from ardrone
IplImage imgnew= IplImage(img_gray);


//Image from disk
IplImage* imgOld = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
IplImage* imgC = cvLoadImage("/opt/pic/1.jpg",CV_LOAD_IMAGE_UNCHANGED);


/*   Define the ROI
         a-b
         | |
         d-c
*/
        int width = imgOld->width;
        int height = imgOld->height;
        CvPoint a = cvPoint  (0*width/3+1    ,   0*height/3+1);
        CvPoint b = cvPoint  (3*width/3-1,   0*height/3+1);
        CvPoint c = cvPoint  (3*width/3-1,   3*height/3-1);
        CvPoint d = cvPoint  (0*width/3+1  ,   3*height/3-1);
        
        CvRect Roi= cvRect(a.x  ,   a.y, b.x-a.x,   c.y-b.y);
        
        
       
        
        cvSetImageROI(imgOld, Roi);
        cvSetImageROI(&imgnew, Roi);
        

	CvSize      img_sz    = cvGetSize( imgOld );
	int         win_size = 10;
	//cout<<"width: "<<img_sz.width<<"length: "<<img_sz.height<<endl;
	
	// The first thing we need to do is get the features
	// we want to track.
	//
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];

	cvGoodFeaturesToTrack(
		imgOld,
		eig_image,
		tmp_image,
		cornersA,
		&corner_count,
		0.01,
		5.0,
		0,
		3,
		0,
		0.04
	);
	cvFindCornerSubPix(
		imgOld,
		cornersA,
		corner_count,
		cvSize(win_size,win_size),
		cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
	);

	// Call the Lucas Kanade algorithm
	
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];
 	CvSize pyr_sz = cvSize( b.x-a.x+8, (c.y-b.y)/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  	CvPoint2D32f* cornersB        = new CvPoint2D32f[ MAX_CORNERS ];

  	cvCalcOpticalFlowPyrLK(
   	  	imgOld,
    	 	&imgnew,
    		pyrA,
     		pyrB,
     		cornersA,
     		cornersB,
     		corner_count,
     		cvSize( win_size,win_size ),
    		 5,
     		features_found,
    		feature_errors,
     		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
     		0
  	);

//for(int i=0 ;i<corner_count;i++){cout<<i<<": "<<features_found[i]<<endl;}
//cout<<endl;


  // Now make some image of what we are looking at:
  
  for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>550 ) {
 //       printf("Error is %f/n",feature_errors[i]);
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x + a.x ),
                                 cvRound( cornersA[i].y + a.y)
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x + a.x ),
                                 cvRound( cornersB[i].y + a.y)
                                 );
    

 	if( (pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) < 50|| ( pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) > 5000) continue;
            
	    //Draw line
	    cvLine( imgC, p0, p1, CV_RGB(255,0,0),1);
            
            //画箭头
            double angle;
	    CvPoint p2 = cvPoint(cvRound(0.0),cvRound(0.0));
            angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );//线段pq的坡角大小
            p2.x = (int) (p1.x + 5 * cos(angle + pi/6));
            p2.y = (int) (p1.y + 5* sin( angle + pi/6));//获得短线1起点x,y
            
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//以q为终点绘制第一条箭头短线
            p2.x = (int) (p1.x + 5  * cos(angle - pi / 6));//箭头下短线2，角度为pq角度减少pi/6,长度为5
            p2.y = (int) (p1.y + 5 * sin(angle - pi / 6));//获得短线2起点x,y
            cvLine( imgC, p2, p1, CV_RGB(255,0,0),1 );//画下第二条箭头短线


	    //calculate the histogram of optical flow
            
            bincode = (int) binsize/2 + floor(angle/(2*pi/binsize));  //floor(1.4)=1
           
 	    his[bincode]++;
  }



//------------------------Find the max bincode---------------------
int maxbincode=0;
for(int i=0;i<binsize;i++){
	if(his[i]>maxbincode) maxbincode = i;
}


//------------------------Caculate Vx and Vy from the optical flow in the max bincode orientation----------------------
int meanCount =0; 
int meanX=0,meanY=0;
for( int i=0; i<corner_count; i++ ) {
     if( features_found[i]==0 || feature_errors[i]>550 ) {
        continue;
     }

      CvPoint p0 = cvPoint(
                                 cvRound( cornersA[i].x + a.x ),
                                 cvRound( cornersA[i].y + a.y)
                                 );
      CvPoint p1 = cvPoint(
                                 cvRound( cornersB[i].x + a.x ),
                                 cvRound( cornersB[i].y + a.y)
                                 );
    

 	if( (pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) < 50|| ( pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2) ) > 5000) continue;
	double angle;
	int bincode_c;
	
	angle = atan2( (double) p0.y - p1.y, (double) p0.x - p1.x );
	bincode_c = (int) binsize/2 + floor(angle/(2*pi/binsize)); 

	if(bincode_c == maxbincode){
		meanX += abs(p0.x-p1.x);
		meanY += abs(p0.y-p1.y);
		meanCount++;
	}
}

if(meanCount == 0)
{
	//cout<<"Dnt find maxbinode"<<endl;
}else{
	meanX /= meanCount;
	meanY /= meanCount;
}
		







	
	/*
	for(int i =0;i<binsize;i++){
	
	cout<<his[i]<<",";
	}
	cout<<endl;
	*/

	//cout<<"0+1:"<<his[0]+his[1]<<" 1+2:"<<his[1]+his[2]<<" 2+3:"<<his[2]+his[3]<<" 3+0:"<<his[3]+his[0]<<endl;

	//Save opticalflow to desk
	/*
	ofstream f1("/opt/optical_flow/flow.txt");
	if(!f1) cout<<"write file failed!"<<endl;
	f1<<his[0]<<" "<<his[1];
	f1.close();
	*/
	
	//Publish optical flow message

	

	//this->pubFlow(meanX,meanY);
	

	//Draw che Roi
  	cvLine(imgC, a, b, cvScalar(255,0,0));
        cvLine(imgC, b, c, cvScalar(255,0,0));
        cvLine(imgC, c, d, cvScalar(255,0,0));
        cvLine(imgC, d, a, cvScalar(255,0,0));
 	
  cvNamedWindow("LKpyr_OpticalFlow",0);
  cvShowImage("LKpyr_OpticalFlow",imgC);




//save the frames
        char address[]="/opt/pic/";
        char num[8];
        char suffix[8]=".jpg";
        char name[100];
        int number =1;
	
        
        sprintf(num,"%d",number);
        sprintf(name,"%s%s%s",address,num,suffix);
      
        cvSaveImage(name, &imgSave);//save the rgb_image

//cvShowImage(WINDOW,imgOld);
//cvShowImage(WINDOW2,imgNew);

// Can cause memory leak, 'new','cvload','cvCreate','cvcopy'

	cvReleaseData(imgOld);/// IMPORTANT!!!!!
       //cvReleaseData(imgNew);
	cvReleaseData(imgC);
        cvReleaseData(pyrA);
        cvReleaseData(pyrB);
	delete cornersA;
	delete cornersB;
        cvReleaseData(eig_image);
        cvReleaseData(tmp_image);

cvWaitKey(33);


}





