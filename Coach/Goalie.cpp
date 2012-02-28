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
//	Kick(3.5);
	float xDiff = x1 - 3000;
	float yDiff = y1 - y2;
	float angle1 = atan ( (x-x1) / (y-y1) );
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
	if( x <= 2400)
	{
		Move(2950,y2,1.5);
	}
		
	if (y1 <= 350 && y1 >= -350)
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
				Move(2800,y1-150,1);
				Rotate(angle+0.2);
			}
	
			else if (y1 < -150)
			{		
				Move(2800,y1+150,1);	
				Rotate(angle-0.2);
			}	
		
			else if (y1 < 0 && y1 >= -150)
			{
				Move(2800,y1+50,1);
				Rotate(angle-0.2);
			}

			else if (y1 > 0 && y1 <= 150)
			{
				Move(2800,y1-50,1); 
				Rotate(angle+0.2);
			}
			
			else if (y1 == 0)		
			{
				Move(2800,y1,1);
				Rotate(PI);
			}		
		}
	
		else if (x1 >= 2400 && x1 <= 3000  && velK==0){
			
//			Kick(3.0);
			if (y1 > 100)
			{
				Move(2950,y1+50,0.75);
				Rotate(angle+0.5);

			}
	
			
			else if (y1< 100 && y1 >= 0)
			{	
				Move(2950,y1+25,0.75);
				Rotate(angle+0.25);
		
			}

			else if (y1 < 0 && y1 > -100)
			{
				Move(2950,y1-25,0.75);
				Rotate(angle-0.25);
			}

			else
			{
				Move(2950,y1-50,0.75);
				Rotate(angle-0.5);

			}

		}
	
		if ( x >= 2400 && x <= 2950 && sqrt(pow((y-y1),2)) <= 50){
					
			Kick(3.0);
			while (x1 >= 2400 && x1 <= 2950 && x >= 2400)		
			{
				velT = 3;
				velN = 0;
				velA = 0;
				Kick(3);
				usleep(500*1000);	
			}
			velT = 0;
			Kick(0.0);
		}
	

	}

	
	if (y1 > 350 || y1 < -350)
	{
		if (x1 < 2000)
		{
			if ( y1 > 350 )
			{
				Move (2650,y2+100,0.75);
				Rotate(PI+0.2);
			}
		
			else if ( y1 < -350)
			{
				Move (2650,y2-100,0.75);
				Rotate(PI-0.2);
			}
		}
		
		else if (x1 >= 2000 && x1 < 2400)
		{
			if ( y1 > 350 )
			{
				Move (2850,y2,0.75);
				Rotate(PI+0.2);
			}
		
			else if ( y1 < -350)
			{
				Move (2850,y2,0.75);
				Rotate(PI-0.2);
			}
	
		}

		
		
		else if (x1 >= 2400 && x1 < 3000)
		{
	
				if (y1 >= 450)
				{
					if (x1 >= 2650)
					{ 
						Move(2950,275,0.75);
						Rotate(PI- 0.5);
					}
			
					else if (x1 < 2650)
					{
						Move(2800,y2,0.75);
						Rotate (PI + 0.3);
					}

				}
	
				else if (y1 <= -450)
				{
					if (x1 >= 2500)
					{ 
						Move(2950,-275,0.75);
						Rotate(PI + 0.5);
					}
		
					else if(x1 < 2500)
					{
						Move(2800, y2,0.75);
						Rotate(PI + 0.5);
					}
			

				}
			
				else if (y1< 450)
				{	
					Move(2800,y2+25,0.75);
					Rotate(angle+0.25);
		
				}

				else if (y1 > -450)
				{
					Move(2800,y1-25,0.75);
					Rotate(angle-0.25);
				}

			
	
			if ( x >= 2400 && x <= 2950 && sqrt(pow((y-y1),2)) <= 50){
					
				Kick(3.0);
				while (x1 >= 2400 && x1 <= 2950 && x >= 2400)		
				{
					velT = 3;
					velN = 0;
					velA = 0;
					Kick(3);
					usleep(500*1000);	
				}
				velT = 0;
				Kick(0.0);
			}

//			Kick(3);			
/*
			if (y1 < -350 && x1 >= 2650)
			{ 
				Move(2950,-275,0.75);
				Rotate(PI + 0.5);
			}
		
			else if(y1 < -350 && x1 < 2650)
			{
				Move(2800, y2,0.75);
				Rotate(PI + 0.5);
			}

		
			else if (y1 > 350 && x1 >= 2650)
			{ 
				Move(2950,275,0.75);
				Rotate(PI- 0.4);
			}

			
			else if (y1 > 350 && x1 < 2650)
			{
				Move(2800,y2,0.75);
				Rotate (PI + 0.3);
			}
*/
		}
		
		if (y1 >= -450 || y1 <= 450){
			if ( x >= 2400 && x <= 2950 && sqrt(pow((y-y1),2)) <= 100){
					
				Kick(3.0);
				while (x1 >= 2400 && x1 <= 2950 && x >= 2400)		
				{
					velT = 3;
					velN = 0;
					velA = 0;
					Kick(3);
					usleep(500*1000);	
				}
				velT = 0;
				Kick(0.0);
			}
		}
	} 


	if(x1 > 3000){
		Move(2900,0,1.2);
	}
	
}
