
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <curses.h>
#include <math.h>

#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "faceControl.h"
#include "geometry_msgs/Twist.h"

#include <ros/ros.h>
using namespace std;
	
	

faceControl::faceControl()
{
	 
	count = 0;
	
	memset(recordX,0,recordSize);
	memset(recordY,0,recordSize);
	SDX=0;
	SDY=0;	
	
	takeoff_pub	   = n_fC.advertise<std_msgs::Empty>(n_fC.resolveName("ardrone/takeoff"),1);
	land_pub	   = n_fC.advertise<std_msgs::Empty>(n_fC.resolveName("ardrone/land"),1);
	cmd_vel_pub	   = n_fC.advertise<geometry_msgs::Twist>(n_fC.resolveName("cmd_vel"),1);
	
	face_sub	   = n_fC.subscribe(n_fC.resolveName("face"),1,&faceControl::cmdCb,this);
	
}

faceControl::~faceControl(){}




void faceControl::cmdCb(const std_msgs::Int64MultiArrayConstPtr facePtr)
{
		
	faceX= facePtr->data[0];
     	faceY = facePtr->data[1];
	faceWidth = facePtr->data[2];
	faceHeight = facePtr->data[3];
	
	int i = count%recordSize;
	//cout<<i<<endl;
	recordX[i] =  faceX;
	recordY[i] =  faceY;
	count++;
	//for(int j=0;j<recordSize;j++) {cout<<recordX[j]<<" ";}
	//cout<<endl;
	calSD();
	//cout<<"SDX: "<<SDX<<" SDY: "<<SDY<<endl;
}



void faceControl::setVel()
{
		
		
		double face_offset_x = (double)(faceX -320);
		double face_offset_y = (double)(faceY - 180);
		double percent_offset_x = face_offset_x/320;
		double percent_offset_y = face_offset_y/180;
		double face_distance = (double)(faceWidth + faceHeight );
		double percent_distance = face_distance/1000;
	   	cmd.angular.x=cmd.angular.y=cmd.angular.z=0.0;
		cmd.linear.x=cmd.linear.y=cmd.linear.z=0.0;
	if( SDX+SDY < faceSDThreshold)
	{
		cout<<"distance: "<<percent_distance<<" x: "<<percent_offset_x<<" y: "<<percent_offset_y<<endl;
		if(abs(percent_offset_x) > thresholdX)
		{
			speed = gain*percent_offset_x;
			if( speed<0 ) 
			{
				direction =  1;cout<<"x 1"<<" speed "<<speed<<endl;
			}else{
				direction = -1;cout<<"x -1"<<" speed "<<speed<<endl;
			     }
			cmd.angular.z = direction*max( 0.05, min( 1.0, abs(speed) ) );
			cmd.angular.x=1;
			cmd.angular.y=1;
			
			
		}

		 else if(abs(percent_offset_y) > thresholdY)
		{
			speed = gain*percent_offset_y;
			if( speed<0 ) 
			{
				direction = 1;cout<<"y 1"<<" speed "<<speed<<endl;
			}else{
				direction = -1;cout<<"y -1"<<" speed "<<speed<<endl;
			     }
			cmd.linear.z = direction*max( 0.1, min( 1.0, abs(speed) ) );
			cmd.angular.x=1;
			cmd.angular.y=1;
		}
		     
		else if(abs(percent_distance) < thresholdMin || abs(percent_distance) >thresholdMax )
		{
			speed = gain*percent_distance;
			if( percent_distance < thresholdMin)
			{	
				cout<<"dis 1"<<" speed "<<speed<<endl;
				direction = 1;
			}else if(percent_distance > thresholdMax )
			{
				direction = -1;cout<<"dis -1"<<" speed "<<speed<<endl;
			}
			
			cmd.linear.x = direction*max( 0.1, min( 1.0, abs(speed) ) );
			cmd.angular.x=1;
			cmd.angular.y=1;
			
		}
		
		else{
			cmd.angular.x=cmd.angular.y=cmd.angular.z=0.0;
			cmd.linear.x=cmd.linear.y=cmd.linear.z=0.0;
		     }
	}else{
		cmd.angular.x=cmd.angular.y=cmd.angular.z=0.0;
		cmd.linear.x=cmd.linear.y=cmd.linear.z=0.0;
	     }
	
	
}

void faceControl::sendVel()
{	
	cmd_vel_pub.publish(cmd);
}


void faceControl::calSD()
{	
	int countX = 0;
	int countY = 0;
	for(int i =0;i<recordSize;i++)
	{
		countX += recordX[i];
		countY += recordY[i];
	}
	countX /= recordSize;
	countY /= recordSize;
	
	for(int j = 0;j<recordSize;j++)
	{
		
		SDX += pow(double(recordX[j]-countX),2);
		SDY += pow(double(recordY[j]-countY),2);
	}
	//cout<<SDX<<endl;
	SDX /= (double)recordSize;
	SDY /= (double)recordSize;
	SDX = pow(SDX,0.5);
	SDY = pow(SDY,0.5);

}



