
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <curses.h>
#include <math.h>

#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "flowTrack.h"

#include <ros/ros.h>
using namespace std;
	const int flowThreshold   = 25 ;
	const double imuThreshold = 0.5 ;
	const double orientationThreshold = 0.01;
	const double focal_length = 4;//mm should be mesured!
	const double pix_size =0.00000625 ;//m cmos:4mm*2.25mm(mujidao caide) 4/640 = 0.00000625 m 
	const double fps_bottom = 30.0;
	const double fps_front = 30.0;
	int sleeptime1 = 400000;
	int sleeptime2 = 500000;
	
flowTrack::flowTrack()
{

  	 forward = ControlCommand(0,-0.2,0,0);
	 backward = ControlCommand(0,0.2,0,0);
	 right = ControlCommand(1,0,0,0);
	 left = ControlCommand(-1,0,0,0);
	 up = ControlCommand(0,0,0,1);
	 down = ControlCommand(0,0,0,-1);
	 hover = ControlCommand(0,0,0,0);
	 clockwise = ControlCommand(0,0,1,0);
	 anticlockwise = ControlCommand(0,0,-1,0);
	left_turntime =0;
	right_turntime =0;
	stop_time = 0;
	back_time = 0;
	left_turn_finished = true;
	right_turn_finished = true;
	hover_finished = true;
	back_finished = true;
	
	takeoff_pub	   = n_t.advertise<std_msgs::Empty>(n_t.resolveName("ardrone/takeoff"),1);
	land_pub	   = n_t.advertise<std_msgs::Empty>(n_t.resolveName("ardrone/land"),1);
	cmd_vel_pub	   = n_t.advertise<geometry_msgs::Twist>(n_t.resolveName("cmd_vel"),1);
	imu_sub	   	   = n_t.subscribe(n_t.resolveName("ardrone/imu"),1, &flowTrack::imuCb, this);
	flow_sub	   = n_t.subscribe(n_t.resolveName("optical_flow_net"),1,&flowTrack::flowCbAvoidance,this);
	navdata_sub	   = n_t.subscribe(n_t.resolveName("ardrone/navdata"),1,&flowTrack::navdataCb,this);//for drone in gazebo
		
	//navdataAltitude_sub = n_t.subscribe(n_t.resolveName("ardrone/navdata_altitude"),1,&flowTrack::navdataAltitudeCb,this);//for real drone
	flowVel_pub 	    =n_t.advertise<std_msgs::Float64MultiArray>("optical_flow_vel",100);
}

flowTrack::~flowTrack(){}

void flowTrack::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	navdata.vz = navdataPtr->vz;// mm/s
	navdata.altd = navdataPtr->altd;// mm
}

void flowTrack::navdataAltitudeCb(const ardrone_autonomy::navdata_altitudeConstPtr navdataAltdPtr)
{
	navdata.altd = navdataAltdPtr->altitude_vision;//mm
}


void flowTrack::flowCb(const std_msgs::Int64MultiArrayConstPtr flowPtr)
{
		
	opt_velX = (double)(flowPtr->data[0])*fps_bottom*pix_size;//m/s
        opt_velY = (double)(flowPtr->data[1])*fps_bottom*pix_size;
	est_velX = -(double)(navdata.altd) * opt_velX/focal_length - (double) navdata.altd * imu.angular_velocity.y/1000  ;
	est_velY = -(double)(navdata.altd)* opt_velY/focal_length + (double) navdata.altd * imu.angular_velocity.x/1000 ;
	
}
void flowTrack::flowCbAvoidance(const std_msgs::Float64MultiArrayConstPtr flowPtr)
{
		
	opt_left = flowPtr->data[0];
	opt_right = flowPtr->data[1];

	
	
}



void flowTrack::track()
{
		double i = imu.angular_velocity.x + imu.angular_velocity.y;
		double r = abs(imu.orientation.x) + abs(imu.orientation.y);
		
		
		//cout<<"X: "<<est_velX <<" Y: "<<est_velY<<endl;
	
		
	
	
}

void flowTrack::imuCb(const sensor_msgs::ImuConstPtr imuPtr)
{
	imu.angular_velocity.x = imuPtr->angular_velocity.x;
	imu.angular_velocity.y = imuPtr->angular_velocity.y;
	imu.angular_velocity.z = imuPtr->angular_velocity.z;
}

void flowTrack::sendCmdVel(ControlCommand cmd)
{
	
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;

	cmdT.angular.x = cmdT.angular.y = 1;

	cmd_vel_pub.publish(cmdT);
}

void flowTrack::flowVelPub()
{
	std_msgs::Float64MultiArray flowVel;
	flowVel.data.resize(2);
	flowVel.data[0] = est_velX;
	flowVel.data[1] = est_velY;
	flowVel_pub.publish(flowVel);
}

void flowTrack::setCmdTwist()
{
	double speed;
	double direction;
if( abs(imu.angular_velocity.x) + abs(imu.angular_velocity.y) < thresholdImu /*&& abs(navdata.rotX) + abs(navdata.rotY) < thresholdLevel*/ )
	{
		cout<<" x: "<<opt_velX<<" y: "<<opt_velY<<endl;
		if(abs(opt_velX) > thresholdX)
		{
			speed = gain*opt_velX;
			if( speed<0 ) 
			{
				direction =  1;cout<<"y 1"<<" speed "<<speed<<endl;
			}else{
				direction = -1;cout<<"y -1"<<" speed "<<speed<<endl;
			     }
			cmd.linear.y = direction*max( 0.05, min( 0.3, abs(speed) ) );
			cmd.angular.x=1;
			cmd.angular.y=1;
			
			
		}

		 else if(abs(opt_velY) > thresholdY)
		{
			speed = gain*opt_velY;
			if( speed<0 ) 
			{
				direction = 1;cout<<"x 1"<<" speed "<<speed<<endl;
			}else{
				direction = -1;cout<<"x -1"<<" speed "<<speed<<endl;
			     }
			cmd.linear.x = direction*max( 0.1, min( 0.3, abs(speed) ) );
			cmd.angular.x=1;
			cmd.angular.y=1;
		}
		     
		
		
		else{
			cout<<"keep"<<endl;
		     }
	}else{
		
	     }
	

}
void flowTrack::setCmdTwistAvoidance()
{
	
	if( abs(imu.angular_velocity.x) + abs(imu.angular_velocity.y)+abs(imu.angular_velocity.z)< thresholdImu ||
		!left_turn_finished || !right_turn_finished)
	{
		if( (opt_right - opt_left > thresholdFlow && abs(opt_right)>abs(opt_left) )||(!left_turn_finished && right_turn_finished) )
		{
			cout<<"stop! turn left! : "<<left_turntime<<endl;
			cmd.angular.z = 1.0;
			cmd.angular.x=1;
			cmd.angular.y=1;
			cmd.linear.x=turnSpeed;cmd.linear.y=0;
			if(left_turntime< action_time) 
			{
				left_turn_finished = false;
				left_turntime++;
			}else{
				left_turn_finished = true;
				left_turntime = 0;
			     }
			
		}
		else if ( (opt_right - opt_left > thresholdFlow && abs(opt_left) > abs(opt_right) ) || !right_turn_finished)
		{
			cout<<"stop! turn right! : "<<right_turntime<<endl;
			
			cmd.angular.z = -1.0;
			cmd.angular.x=1;
			cmd.angular.y=1;
			cmd.linear.x=turnSpeed;cmd.linear.y=0;
			if(right_turntime < action_time) 
			{
				right_turn_finished = false;
				right_turntime++;
			}else{
				right_turn_finished = true;
				right_turntime = 0;
			     }
		}
		else
		{
		cmd.linear.x = cruiseSpeed;
		cmd.angular.x=0.3;
		cmd.angular.y=0.3;
		cmd.angular.z=0;
		}
	}else{
		
		cmd.linear.x = cruiseSpeed;
		cmd.angular.x=0.3;
		cmd.angular.y=0.3;
		cmd.angular.z=0;
		}
	
}
void flowTrack::detect_stop()
{
	if(( abs(imu.angular_velocity.x) + abs(imu.angular_velocity.y)+abs(imu.angular_velocity.z)< thresholdImu && navdata.vx>100) || !hover_finished)
	{
		if( abs(opt_right - opt_left) > thresholdStop ||!hover_finished )
		{
			cout<<"stop! Hover! : "<<stop_time<<endl;
			cmd.angular.z = 0.0;
			cmd.angular.x=0;
			cmd.angular.y=0;
			cmd.linear.x=turnSpeed;cmd.linear.y=0;
			cmd_vel_pub.publish(cmd);
			if(stop_time < hover_time) 
			{
				hover_finished = false;
				stop_time++;
			}else{
				hover_finished = true;
				stop_time = 0;
			     }
			
		}
	}else if ( ( abs(imu.angular_velocity.x) + abs(imu.angular_velocity.y)+abs(imu.angular_velocity.z) < thresholdImu && abs(navdata.rotX) + abs(navdata.rotY) < thresholdLevel) || !back_finished )
		{
			if(abs(opt_right - opt_left) > thresholdBack||!back_finished)
			{
				cout<<" back! : "<<back_time<<endl;
				cmd.angular.z = 0.0;
				cmd.angular.x=1;
				cmd.angular.y=0;
				cmd.linear.x=-0.2;cmd.linear.y=0;
				cmd_vel_pub.publish(cmd);
				if(back_time < Back_time) 
				{
					back_finished = false;
					back_time++;
				}else{
					back_finished = true;
					back_time = 0;
			    	     }
			}
		}

	
}

void flowTrack::sendCmdTwist()
{
	cmd_vel_pub.publish(cmd);
}



