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

//This function 


//x1, y1 = coordinate of ball
void Goalie::Defend(float x1, float y1){

	float y2 = ((y1)/(x1-3200))*(2800-x1) + y1;
	Kick(3.5);
	float xDiff = x1 - 3000;
	float yDiff = y1 - y2;
	float angle = atan (yDiff/xDiff);
	if(xDiff < 0)
		angle = PI + angle;
	if(xDiff > 0 && yDiff < 0)
		angle += 2*PI;

	//Center of goal, deep end (3200, 0)
	if(sqrt(pow(angle-orientation,2)) < .05 && velT == 0 && velN == 0 && x1 > 2600 && x1 < 2900 && y1 > -350 && y1 < 350){
		velT = 3;
		velN = 0;
		velA = 0;
		Kick(3);
		usleep(800*1000);
		velT = 0;
	}
	
	if (y2 < 375 && y2 > -375 && velT != 3 && x1 < 3000)
	{
		
		Move(2900,y2,1.2);
		
		//if (yDiff > 0)
			//Move(2900,y2-100,1);
		
		//else
			//Move(2900,y2+50,1);
		
		Rotate(angle);
	}
	
	else if(velT !=3 && x1 < 3000)
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

	if(x1 > 3000){
		Move(2900,0,1.2);
	}
	
}
