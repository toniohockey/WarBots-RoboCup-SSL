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
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "netraw.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "Robot.h"
#include "Goalie.h"
#define PI 3.14159265 

Goalie::Goalie(int id_in, bool yellow_in):Robot::Robot (id_in, yellow_in){
}

//x1, y1 = coordinate of ball
void Goalie::Defend(float x1, float y1){

	Move(3025,0,20);
	Rotate(3.14);
	float y2 = ((y1)/(x1-3200))*(2800-x1) + y1;
	Kick(3.5);
	

	//Center of goal, deep end (3200, 0)
	
	if (y2 < 375 && y2 > -375)
	{
		float xDiff = x1 - 3000;
		float yDiff = y1 - y2;
		Move(2900,y2,1.2);
		
		//if (yDiff > 0)
			//Move(2900,y2-100,1);
		
		//else
			//Move(2900,y2+50,1);
		
		float angle = atan (yDiff/xDiff);
		if(xDiff < 0)
			angle = PI + angle;
		if(xDiff > 0 && yDiff < 0)
			angle += 2*PI;
		
		Rotate(angle + 0.05);
	}
	
	else
	{
		if (y2 < -375)
		{ 
			Move(2900,-350,1.2);
			Rotate(1.25*PI);
		}
		
		else
		{ 
			Move(2950,350,1);
			Rotate(0.75*PI);
		}
	}
	
}







