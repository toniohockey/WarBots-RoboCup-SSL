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
#include <queue>
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
#define MOVE 1
#define BRAKE 2
#define SHOOT 3

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
	current = NULL;
	shooter = true;
}

void Robot::Move(float angle, float vel){
	float newAngle = angle - orientation;
	velT = cos(newAngle) * vel;
	velN = sin(newAngle) * vel;
	
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

void Robot::Move(float x1, float y1, float vel){
	printf("entered MoveTo %d", id);
	
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
	if(R > 25 && R < 200){
		velT *= R/200.0;
		velN *= R/200.0;
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
	printf("EXITED MOVETO %d", id);
			
}
void Robot::PathTo(float x1, float y1, float vel){

	printf("\nentered pathto %d",id);
	float R = sqrt(pow(x-x1,2)+pow(y-y1,2));
	float x2 = x1;
	float y2 = y1;
	//printf("\nA is %f, B is %f, C is %f, R is %f",a,b,c,R);
	//printf("In the way is %d, Interference is %d",inTheWay(x,x1,-1000,y,y1,-0.001,true),interference(a,b,c,-0.001,-1000,200));



	if(R > 0 && 0< current->robots_yellow_size()  && 0 < current->robots_blue_size()){
	for(int i = 0; i < current->robots_yellow_size(); i++){
		SSL_DetectionRobot *robot = const_cast<SSL_DetectionRobot*>(&(current->robots_yellow(i)));
		if(!(yellow && id==i)){
		if(id == 1){
			inTheWay(x,x1,robot->x(),y,y1,robot->y(),true);
			printf("InTheWay? %d, Intefere? %d",inTheWay(x,x1,robot->x(),y,y1,robot->y(),true),interference(x,y,x1,y1,robot->x(),robot->y(),200));}
		if(interference(x,y,x1,y1,robot->x(),robot->y(),300) && distance(x,robot->x(),y,robot->y()) < R && inTheWay(x,x1,robot->x(),y,y1,robot->y(),false)){
			float angle2 = atan2(y-robot->y(),x-robot->x());
			angle2 += 1.5;
			float xUp = 300*cos(angle2);
			float yUp = 300*sin(angle2);
			xUp = xUp + robot->x();
			yUp = yUp + robot->y();
			x2 = xUp;
			y2 = yUp;
			R = distance(x,robot->x(),y,robot->y());
			printf("\nRobot %d in yellow is in the Way!!",i);
		}}
	}
	for(int i = 0; i < current->robots_blue_size(); i++){
		SSL_DetectionRobot * robot = const_cast<SSL_DetectionRobot*>(&current->robots_blue(i));
		if(!(!yellow && id == i)){
		if(interference(x,y,x1,y1,robot->x(),robot->y(),300) && distance(x,robot->x(),y,robot->y()) < R && inTheWay(x,x1,robot->x(),y,y1,robot->y(),false)){
			float angle2 = atan2(y-robot->y(),x-robot->x());
			angle2 += 1.5;
			float xUp = 300*cos(angle2);
			float yUp = 300*sin(angle2);
			xUp = xUp + robot->x();
			yUp = yUp + robot->y();
			x2 = xUp;
			y2 = yUp;
			R  = distance(x,robot->x(),y,robot->y());
			printf("\nRobot %d in blue is in the Way!!",i);
		}}
	}
	SSL_DetectionBall ball = current->balls(0);
	if(interference(x,y,x1,y1,ball.x(),ball.y(),150) && distance(x,ball.x(),y,ball.y()) < R && inTheWay(x,x1,ball.x(),y,y1,ball.y(),true)){
			float angle2 = atan2(y-ball.y(),x-ball.x());
			angle2 += 1.5;
			float xUp = 200*cos(angle2);
			float yUp = 200*sin(angle2);
			xUp = xUp + ball.x();
			yUp = yUp + ball.y();
			x2 = xUp;
			y2 = yUp;
			R  = distance(x,ball.x(),y,ball.y());
			printf("\nBall is in the WAY!!!");
		}
	//printf("InTheWay? %d, Intefere? %d",inTheWay(x,x1,ball.x(),y,y1,ball.y(),true),interference(x,y,x1,y1,ball.x(),ball.y(),300));
	}
	Move(x2,y2,vel);
	printf("\nX2 is %f, Y2 is %f",x2,y2);
	printf("Exited PATHTO!! %d",id);
}
			
void Robot::Update(SSL_DetectionFrame* frame){

	SSL_DetectionRobot robot;
	if(yellow)
		robot = frame-> robots_yellow(id);
	else
		robot = frame-> robots_blue(id);

	x= robot.x();
	y= robot.y();
	current = frame; 
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

void Robot::Stop(){

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
    command->set_veltangent(0);
    command->set_velnormal(0);
    command->set_velangular(0);
    command->set_kickspeedx(0);
    command->set_kickspeedz(0);
    command->set_spinner(true);

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

void Robot::Pass(int Rid, bool ISyellow, float vel){
	//call prepare to aim at/in front of shooter
	SSL_DetectionBall ball = current->balls(0);
	SSL_DetectionRobot robot;
	if(ISyellow){
		robot = current->robots_yellow(Rid);
	}
	else{
		robot = current->robots_blue(Rid);
	}
	float R = distance(robot.x(),robot.y(),x,y);
	float velocityK = R/1000;
	Prepare(ball.x(),ball.y(),vel,velocityK+vel,robot.x(),robot.y());
}

void Robot::Prepare(float x1, float y1, float vel, float velK, float x2, float y2){

	printf("\nPreparing! IDID %d", id);

	float yDiff= y1-y2;
	float xDiff= x1-x2;
	float angle = 9999999;
	float R = sqrt((yDiff*yDiff)+(xDiff*xDiff));

	float dist = R;
	int standoffDist = (int)(dist/10.0);
	
	

	if(R > 50){
		angle = acos(-xDiff/R);
		if(yDiff < 0)
			angle = 2*PI - angle;
	}
	float new_xDiff = -(standoffDist)*cos(angle);
	float new_yDiff = (standoffDist)*sin(angle);

	float new_x;
	float new_y;
	angle = 2*PI - angle;
	printf("\nAngle to shoot is: %f",angle);
	new_x = new_xDiff + x1;
	new_y = new_yDiff + y1;
	float diff = angle - orientation;

	R = sqrt(pow((x - new_x),2)+pow((y-new_y),2));

	if(sqrt(pow(diff,2)) < 0.05 && velT == 0 && velN == 0 && sqrt(pow(velA,2)) < 0.05)
	{
		velT = 5;
		velN = 0;
		velA = 0;
		Kick(velK);
		printf("\nTIME TO SHOOT");
		usleep(standoffDist*(5.0/4.0)*1000);	//time taken to move in for kick
		velT = -5;
		usleep(standoffDist*(1000.0/dist)*1000);	//time used to pull back / brake
		velT = 0;
	}
	if(velT != 5){
		Rotate(angle);
		printf("%d is WEIRD, %d is WEIRD", id, id);
		
		printf("\nnewX is %f, newY is %f + %d this is in Prepare",new_x,new_y,id);
		PathTo(new_x,new_y,vel);
		
	}	
	printf("Exited Prepare! %d",id);	
}

float Robot::distance(float x1, float x2, float y1, float y2){
	float someR = sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0));
	return someR;
}

bool Robot::interference(float x1, float y1, float x2, float y2, float x3, float y3, float radius){
	float px = x2-x1;
	float py = y2-y1;

	float something = px*px+py*py;
	
	float u = ((x3-x1)*px+(y3-y1)*py)/something;
	
	if(u > 1.0)
		u = 1.0;
	else if (u < 0)
		u= 0;

	float xx = x1 + u * px;
	float yy = y1 + u * py;

	float dx = xx - x3;
	float dy = yy - y3;

	float d = sqrt(dx*dx + dy*dy);
	if(d>radius)
		return false;
	else
		return true;
}

bool Robot::inTheWay(float x1, float x2, float x3, float y1, float y2, float y3, bool v){
	float midX = (x1+x2)/2.0;
	float midY = (y1+y2)/2.0;
	float radius = distance(x1,x2,y1,y2)/2.0;
	if(v)
		printf("\nmidX is %f, midY is %f, radius is %f, distance is %f, x3 is %f, y3 is %f",midX,midY,radius,distance(x3,midX,y3,midY),x3,y3);
	if(distance(x3,midX,y3,midY) > radius)
		return false;
	else
		return true;
}
	

	


