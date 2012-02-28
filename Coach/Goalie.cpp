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

	Move(2800,0,1.2);
	float y2 = ((y1)/(x1-3200))*(2900-x1) + y1;
	
	Kick(3.5);
	float xDiff = x1 - 3000;
	float yDiff = y1 - y2;
	float angle = atan (yDiff/xDiff);
	if(xDiff < 0)
		angle = PI + angle;
	if(xDiff > 0 && yDiff < 0)
		angle += 2*PI;

	//Center of goal, deep end (3200, 0)
	// velT == 0 && velN == 0 &&
/*	
	if (x1 > 2600 && x1 < 2950 && y1 > -350 && y1 < 350)
	{
		float angle1 = atan ( (y-y1) / (x-x1) );
		Rotate(angle1);
		if(sqrt(pow(angle-orientation,2)) < .05){
		
			velT = 3;
			velN = 0;
			velA = 0;
			Kick(3);
			usleep(800*1000);
			velT = 0;
		}
	}
*/
		
	if (y1 < 350 && y1 > -350 && velT != 3)
	{
		
		//Move(2900,y2,1.5);
		if (x1 < 2000)
		{
		
			if (y1 > 100)
			{
				Move(2900,y2+50,1);
				Rotate(angle+0.2);
			}
	
			else if (y1 < -100)
			{		
					Move(2900,y2-50,1);	
					Rotate(angle-0.2);
			}	
		
			else if (y1 <= 0 && y1 >= -100)
			{
				Move(2900,y2-25,1);
				Rotate(angle-0.2);
			}

			else if (y1 > 0 && y1 <= 100)
			{
				Move(2900,y2+25,1); 
				Rotate(angle+0.2);
			}
		}		
		
		else if (x1 >= 2000 && x1 <2400)
		{
			if (y1 > 150)
			{
				Move(2950,y1-150,1);
				Rotate(angle+0.2);
			}
	
			else if (y1 < -150)
			{		
					Move(2950,y1+150,1);	
					Rotate(angle-0.2);
			}	
		
			else if (y1 <= 0 && y1 >= -150)
			{
				Move(2950,y1+50,1);
				Rotate(angle-0.2);
			}

			else if (y1 > 0 && y1 <= 150)
			{
				Move(2950,y1-50,1); 
				Rotate(angle+0.2);
			}		

		
		}
	
		else if (x1 >= 2400 && x1 <= 3000){
			if (y1 > 0)
			{
				Move(2950,y1,0.75);
				Rotate(angle+0.5);

			}
	
			else
			{
				Move(2950,y1,0.75);
				Rotate(angle-0.3);

			}
			
		}
			

		if(sqrt(pow(angle-orientation,2)) < .05){
		
		velT = 1;
		velN = 0;
		velA = 0;
		Kick(3);
		usleep(500*1000);
		velT = 0;
		}
			

		

	}

	
	if (y1 > 350 || y1 < -350 && velT != 3)
	{
		if (x1 < 2000)
		{
			if ( y1 > 350 )
			{
				Move (2650,y2+100,0.75);
				Rotate(PI+0.2);
			}
		
			else if ( y1 <= -350)
			{
				Move (2650,y2-100,0.75);
				Rotate(PI-0.2);
			}
		}
		
		else if (x1 >= 2000 && x1 < 2400)
		{
			if ( y1 > 350 )
			{
				Move (2750,y2,0.75);
				Rotate(PI+0.2);
			}
		
			else if ( y1 <= -350)
			{
				Move (2750,y2,0.75);
				Rotate(PI-0.2);
			}
	
		}
		
		else if (x1 >= 2400 && x1 < 3000)
		{
			
			if (y1 < -350 && x1 >= 2650)
			{ 
				Move(2950,-275,0.75);
				Rotate(PI + 0.7);
			}
			
			else if(y1 < -350 && x1 < 2650)
			{
				Move(2900, y2,0.75);
				Rotate(PI + 0.7);
			}
		
			else if (y1 > 350 && x1 >= 2650)
			{ 
				Move(2950,275,0.75);
				Rotate(PI- 0.4);
			}
			
			else if (y1 > 350 && x1 < 2650)
			{
				Move(2900,y2,0.75);
				Rotate (PI + 0.3);
			}
		
		}
	
	} 


	if(x1 > 3000){
		Move(2900,0,1.2);
	}
	
}
