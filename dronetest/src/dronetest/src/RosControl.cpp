#include <unistd.h>
#include <curses.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"


#include "std_msgs/Empty.h" 
#include "geometry_msgs/Twist.h"
#include "iostream"
#include "fstream"
#include <stdlib.h>


#include "RosControl.h"

RosControl::RosControl(){
	 forward = ControlCommand(0,-0.5,0,0);
	 backward = ControlCommand(0,0.5,0,0);
	 right = ControlCommand(1,0,0,0);
	 left = ControlCommand(-1,0,0,0);
	 up = ControlCommand(0,0,0,1);
	 down = ControlCommand(0,0,0,-1);
	 hover = ControlCommand(0,0,0,0);
	clockwise = ControlCommand(0,0,1,0);
	anticlockwise = ControlCommand(0,0,-1,0);
}

RosControl::~RosControl(void){}

void RosControl::run()
{
	//Define takoff pub and land pub 
	
	takeoff_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/takeoff"),1);
	land_pub	   = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/land"),1);
	vel_pub	   	   = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);
	imu_sub	   	   = nh_.subscribe(nh_.resolveName("ardrone/imu"),10, &RosControl::imuCb, this);

}
void RosControl::startControl()
{

	char c;
	int sleeptime=200000;//sleep time second
	

	while( (c =getchar()) != 'e'){

		if(c == 'l'){
		cout<<"right"<<endl;
		sendVel(right);
		}
		
		if(c == 'j'){
		cout<<"left"<<endl;
		sendVel(left);
		}

		if(c == 'i'){
		cout<<"forward"<<endl;
		sendVel(forward);
		}
		
		if(c == 'k'){
		cout<<"backward"<<endl;
		sendVel(backward);
		}

		if(c == 'o'){
		cout<<"clockwise"<<endl;
		sendVel(clockwise);
		}

		if(c == 'u'){
		cout<<"anticlockwise"<<endl;
		sendVel(anticlockwise);
		}


		if(c == 'h'){
		cout<<"hover"<<endl;
		sendVel(hover);
		}

		if(c == 'q'){
		cout<<"up"<<endl;
		sendVel(up);
		}

		if(c == 'a'){
		cout<<"down"<<endl;
		sendVel(down);
		}

		if(c == 's')
		{
		cout<<"Takeoff"<<endl;
		takeoff_pub.publish(std_msgs::Empty());
		}

	
		if (c == 'd'){
		cout<<"Land"<<endl;
		land_pub.publish(std_msgs::Empty());
		}

		if(c == '\n'){
		usleep(sleeptime);
		sendVel(hover);
		}
		
		}

		land_pub.publish(std_msgs::Empty());
		cout<<"Land"<<endl;
		
		usleep(sleeptime);

}
void RosControl::sendVel(ControlCommand cmd)
{
	
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;

	cmdT.angular.x = cmdT.angular.y = 1;

	vel_pub.publish(cmdT);
}

void RosControl::imuCb(const sensor_msgs::ImuConstPtr imuPtr)
{
	cout<<imuPtr->angular_velocity.x<<endl;
	

}

void RosControl::track(char* flow_file)
{
	char c;
	while(1)
	{

	ifstream in(flow_file);
	string s;
        int sleeptime1 = 900000;
	int sleeptime2 = 1.5;
	int code[]={0,0,0,0,0};
	int i=0;
	while(in >> s)
	{
		
		code[i++] = atoi(s.c_str());
	}
	in.close();
	
	cout<<code[0]<<" "<<code[1]<<endl;

	if(code[0]>150)
	{
		cout<<"in the control:backward!"<<endl;
		sendVel(backward);
		
	};

	if(code[1]>150)
	{	
		cout<<"in the control:forward!"<<endl;
		sendVel(forward);
	};

	usleep(sleeptime1);
	sendVel(hover);
	sleep(sleeptime2);

	}
}

