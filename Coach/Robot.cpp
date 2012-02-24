#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <netinet/in.h>
#include <resolv.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "netraw.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "Robot.h"
#define PI 3.14159265 

Robot::Robot(){
}
Robot::Robot(int id_in, bool yellow_in){
	id = id_in;
	yellow = yellow_in; 
	velT = 0;
	velN = 0;
	velA = 0;
	orientation = 0;
	x = 0;
	y = 0;
	velK = 0;
}

void Robot::Move(float x1, float y1, float vel){
	
	
	float yDiff= y1-y;
	float xDiff= x1-x;
	float angle = 9999999;
	float R = sqrt((yDiff*yDiff)+(xDiff*xDiff));
	
	if(R > 25){
		angle = acos(xDiff/R);
		if(yDiff < 0)
			angle = 2*PI - angle;
		angle -= orientation;
		if(angle < 0)
			angle = 2*PI + angle;
	
		float new_xDiff = R*cos(angle);
		float new_yDiff = R*sin(angle);

		float e;
		e = std::max(abs(new_xDiff),abs(new_yDiff));

		velT = (new_xDiff/e)*vel;
		velN = (new_yDiff/e)*vel;
	}
	else{
		velT = 0;
		velN = 0;
		R = 9999999;
	}

	printf("\nVelT is %f, VelN is %f, R is %f, angle is %f, x is %f, y is %f",velT,velN,R,angle,x1,y1);

	grSim_Packet packet;	
	
	packet.mutable_commands()->set_isteamyellow(yellow);
    packet.mutable_commands()->set_timestamp(1.0);
	grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
	command->set_id(id);
	command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(velT);
    command->set_velnormal(velN);
    command->set_velangular(velA);
    command->set_kickspeedx(velK);
    command->set_kickspeedz(0);
    command->set_spinner(true);

	sendPacket(packet);
			
}

void Robot::Update(SSL_DetectionFrame* frame){

	SSL_DetectionRobot robot;
	if(yellow)
		robot = frame-> robots_yellow(id);
	else
		robot = frame-> robots_blue(id);

	x= robot.x();
	y= robot.y();
	orientation = robot.orientation();
	if(orientation < 0)
		orientation = 2*PI + orientation;
}

void Robot::Kick(float vel){
	velK = vel;
	grSim_Packet packet;	

	packet.mutable_commands()->set_isteamyellow(yellow);
    packet.mutable_commands()->set_timestamp(0.0);
	grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
	command->set_id(id);
	command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(velT);
    command->set_velnormal(velN);
    command->set_velangular(velA);
    command->set_kickspeedx(velK);
    command->set_kickspeedz(0);
    command->set_spinner(true);

	sendPacket(packet);
}

void Robot::Teleport(float x1, float y1){

	grSim_Packet packet;	

	grSim_RobotReplacement* replace = packet.mutable_replacement()->add_robots();
	
	replace->set_id(id);
	replace->set_x(x1);
	replace->set_y(y1);
	replace->set_dir(3.14);
	replace->set_yellowteam(yellow);

	sendPacket(packet);
}
void Robot::Rotate(float angle){
	if(orientation != angle){
		float dir = 1;
		float checkAngle = angle - orientation;
		if(checkAngle < 0)
			checkAngle += 2*PI;
		if(checkAngle > 0 && checkAngle < PI)
			dir = 1;
		else
			dir = -1;
		if(checkAngle > PI)
			checkAngle -= PI;
		velA = dir * (5.0 * (checkAngle/PI));
	}
	else{
		velA = 0;
		angle = -10;
	}
	grSim_Packet packet;	
	packet.mutable_commands()->set_isteamyellow(yellow);
    packet.mutable_commands()->set_timestamp(0.0);
	grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
	command->set_id(id);
	command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(velT);
    command->set_velnormal(velN);
    command->set_velangular(velA);
    command->set_kickspeedx(velK);
    command->set_kickspeedz(0);
    command->set_spinner(true);

	sendPacket(packet);
}

void Robot::sendPacket(grSim_Packet packet){

	std::string s;
    packet.SerializeToString(&s);
	Net::UDP udpsocket;
	Net::Address _addr;
	_addr.setHost("127.0.0.1",20011);
	udpsocket.open();
    udpsocket.send((void*) s.c_str(), s.length(), _addr);

}

bool Robot::Prepare(float x1, float y1,float vel, float x2, float y2){

	float yDiff= y1-y2;
	float xDiff= x1-x2;
	float angle = 9999999;
	float R = sqrt((yDiff*yDiff)+(xDiff*xDiff));

	if(sqrt(pow((y - y1),2)) < 200 && sqrt(pow((x - x1),2)) > 300){
		Move(x,y+300,1);
		usleep(1000*1000);
	}

	if(R > 50){
		angle = acos(-xDiff/R);
		if(yDiff < 0)
			angle = 2*PI - angle;
	}
	float new_xDiff = -(250)*cos(angle);
	float new_yDiff = (250)*sin(angle);

	float new_x;
	float new_y;
	angle = 2*PI - angle;
	printf("\nAngle to shoot is: %f",angle);
	new_x = new_xDiff + x1;
	new_y = new_yDiff + y1;

	R = sqrt(pow((x - new_x),2)+pow((y-new_y),2));	
	if(abs(new_x - x) < 50){
		Move(x,new_y,1); 
		Rotate(angle);
	}
	else{
		Move(new_x,y, 1);
	}
	float diff = angle - orientation;
	if(sqrt(pow(diff,2)) < 0.1 && velT == 0 && velN == 0)
	{
		Move(x1,y1,1);
		Kick(5);
		usleep(1500*1000);
	}
		
}

	


