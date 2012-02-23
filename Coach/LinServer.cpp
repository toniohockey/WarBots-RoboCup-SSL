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
#define goal_x 3200
#define goal_y 0
#define goalie_x  2900
#define goalie_y1 370
#define goalie_y2 -380

Robot Ybots [5];
Robot Bbots [5];
int count = 0;
int newCount = 0;
bool ready;


float distance(float x1, float x2, float y1, float y2){
	return sqrt((pow((x2-x1),2)-pow((y2-y1),2)));
}

float ballToGoal(SSL_DetectionBall ball, float x0){
	float x = ball.x();
	float y = ball.y();
	float slope = (y - goal_y)/(x - goal_x);
	float y0 = slope*(x0 - x) + y;
	return y0;
}

	
void sendPacket(grSim_Packet packet)
{
	/*
    grSim_Packet packet;
    bool yellow = false;
	bool blue = false;

    //if (cmbTeam->currentText()=="Yellow") yellow = true;
	

    packet.mutable_commands()->set_isteamyellow(i == 0);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(id);

    command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(-8);
    command->set_velnormal(2);
    command->set_velangular(0);

    command->set_kickspeedx(20);
    command->set_kickspeedz(20);
    command->set_spinner(true);

	*/
    std::string s;
    packet.SerializeToString(&s);
	Net::UDP udpsocket;
	Net::Address _addr;
	_addr.setHost("127.0.0.1",20011);
	udpsocket.open();
    udpsocket.send((void*) s.c_str(), s.length(), _addr);

}
void chaseBall(SSL_DetectionFrame* frame){

	SSL_DetectionBall ball = frame->balls(0);
	SSL_DetectionRobot robot = frame->robots_blue(4);

	double yDiff = ball.y() - robot.y();
	double xDiff = ball.x() - robot.x();
	double angleToBall = atan(yDiff/xDiff);
	if(xDiff < 0)
		angleToBall = PI + angleToBall;
	if(xDiff > 0 && yDiff < 0)
		angleToBall += 2*PI;
	printf("\nAngle to ball is: %f",angleToBall);

	double robotAngle = robot.orientation();
	if(robotAngle < 0)
		robotAngle = 2*PI + robotAngle;
	printf("\nRobot angle is: %f",robotAngle);
	
	double checkAngle = angleToBall - robotAngle;
	if(checkAngle < 0)
		checkAngle += 2*PI;
	printf("\ncheck is: %f",checkAngle);
	double velT = 0;
	double velA = 0;
	double angleL = angleToBall + .4;
	double angleS = angleToBall - .4;
	
	if(checkAngle > 0 && checkAngle < PI)
		checkAngle = 1;
	else 
		checkAngle = -1;

	if(robotAngle < angleL && robotAngle > angleS){
		velT = 2;
		velA = 0;
	}
	else{
		velA = (int)checkAngle*2;
		velT = 0;
	}
	
	grSim_Packet packet;



    packet.mutable_commands()->set_isteamyellow(false);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();

	//robot.command = command
    command->set_id(4);
	
	

    command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(velT);
    command->set_velnormal(0);
    command->set_velangular(velA);

    command->set_kickspeedx(5);
    command->set_kickspeedz(0);
    command->set_spinner(true);
	
	sendPacket(packet);


}

void scoring(SSL_DetectionFrame* frame){
	SSL_DetectionBall ball = frame->balls(0);
	SSL_DetectionRobot robot = frame->robots_blue(4);

	float x0 = ball.x() - 200;
	float y0 = ballToGoal(ball,x0);

	if(x0 > 3000 || x0 < -3000 || y0 > 2000 || y0 < -2000){
		chaseBall(frame);
		return;
	}	

	double yDiff0 = y0 - robot.y();
	double xDiff0 = x0 - robot.x();
	double angleToSpot = atan(yDiff0/xDiff0);
	if(xDiff0 < 0)
		angleToSpot = PI + angleToSpot;
	if(xDiff0 > 0 && yDiff0 < 0)
		angleToSpot += 2*PI;

	double yDiff = ball.y() - robot.y();
	double xDiff = ball.x() - robot.x();
	double angleToGoal = atan(yDiff/xDiff);
	if(xDiff < 0)
		angleToGoal = PI + angleToGoal;
	if(xDiff > 0 && yDiff < 0)
		angleToGoal += 2*PI;
 
	double robotAngle = robot.orientation();
	if(robotAngle < 0)
		robotAngle = 2*PI + robotAngle;

	double checkAngle0 = angleToSpot - robotAngle;
	if(checkAngle0 < 0)
		checkAngle0 += 2*PI;
	double velT = 0;
	double velA = 0;
	double kickVel = 0;

	if(checkAngle0 > 0 && checkAngle0 < PI)
		checkAngle0 = 1;
	else 
		checkAngle0 = -1;

	if(robotAngle < angleToSpot + .4 && robotAngle > angleToSpot - .4){
		velT = 2;
		velA = 0;
	}
	else{
		velA = (int)checkAngle0*2;
		velT = 0;
	}
	
	if(distance(robot.x(),x0,robot.y(),y0) < 200) {
		
		velT = 0;
		double checkAngle = angleToGoal - robotAngle;
		if(checkAngle < 0)
			checkAngle += 2*PI;
		if(checkAngle > 0 && checkAngle < PI)
			checkAngle = 1;
		else
			checkAngle = -1;
		if(robotAngle < angleToGoal + .2 && robotAngle > angleToGoal - .2){
			velT = 2;
			velA = 0;
			kickVel = 5;
		}
		else{
			velA = (int)checkAngle*1;
			velT = 0;
		}
		
	}
		
	grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(false);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();



    command->set_id(4);
    command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(velT);
    command->set_velnormal(0);
    command->set_velangular(velA);

    command->set_kickspeedx(kickVel);
    command->set_kickspeedz(0);
    command->set_spinner(true);
	
	sendPacket(packet);
}
		

void goalie(SSL_DetectionFrame* frame){

	SSL_DetectionBall ball = frame->balls(0);
	SSL_DetectionRobot robot = frame->robots_yellow(4);
	int velN = 0;
	int velT = 0;
	int velA = 0;
	if(robot.x() > goalie_x + 200)
		velT = 1;
	else if(robot.x() < goalie_x - 200)
		velT = -1;
	if(robot.x() <  goalie_x + 200 && robot.x() > goalie_x - 200){
		float y0 = ballToGoal(ball,goalie_x);
		printf("\ny0 is %f",y0);
		if(y0 - 50 > robot.y())
			velN = -5;
		else if(y0 + 50 < robot.y())
			velN = 5;
		if(velN > 0 && robot.y() < goalie_y2)
			velN = 0;
		if(velN < 0 && robot.y() > goalie_y1)
			velN = 0;
		
	}
	if(robot.orientation() > 0){
		if(robot.orientation() < PI - .2){
			velA = 1;
			velN = 0;
			velT = 0;
		}
	}
	else if(robot.orientation() < 0){
		if(robot.orientation() > -PI + .2){
			velA = -1;
			velN = 0;
			velT = 0;
		}
	}
				
	
	
	grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(true);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(4);
	
	

    command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(velT);
    command->set_velnormal(velN);
    command->set_velangular(velA);

    command->set_kickspeedx(10);
    command->set_kickspeedz(0);
    command->set_spinner(true);
	
	sendPacket(packet);

	
}

void start1v1(){

for(int id = 0; id < 4; id++){
 grSim_Packet packet;
	
	for(int i = 0; i < 2; i++)
{
    packet.mutable_commands()->set_isteamyellow(i == 0);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(id);

    command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(-8);
    command->set_velnormal(2);
    command->set_velangular(0);

    command->set_kickspeedx(20);
    command->set_kickspeedz(20);
    command->set_spinner(true);

	grSim_RobotReplacement* replace = packet.mutable_replacement()->add_robots();

	replace->set_id(id);
	replace->set_x(10000);
	replace->set_y(10000);
	replace->set_dir(3.14);
	replace->set_yellowteam(i==0);

    std::string s;
    packet.SerializeToString(&s);
	Net::UDP udpsocket;
	Net::Address _addr;
	_addr.setHost("127.0.0.1",20011);
	udpsocket.open();
    udpsocket.send((void*) s.c_str(), s.length(), _addr);
}
}


}

void* SocketHandler(void*);

int main(int argv, char** argc){

	for(int i = 0; i < 5; i++){
		Ybots[i] = Robot(i, true);
		Bbots[i] = Robot(i, false);
	}	
	ready = false;

    int host_port= 30015;

    struct sockaddr_in my_addr;

    int hsock;
    int * p_int ;
    int err;

    socklen_t addr_size = 0;
    int* csock;
    sockaddr_in sadr;
    pthread_t thread_id=0;


    hsock = socket(AF_INET, SOCK_STREAM, 0);
    if(hsock == -1){
        printf("Error initializing socket %d\n", errno);
        goto FINISH;
    }
    
    p_int = (int*)malloc(sizeof(int));
    *p_int = 1;
        
    if( (setsockopt(hsock, SOL_SOCKET, SO_REUSEADDR, (char*)p_int, sizeof(int)) == -1 )||
        (setsockopt(hsock, SOL_SOCKET, SO_KEEPALIVE, (char*)p_int, sizeof(int)) == -1 ) ){
        printf("Error setting options %d\n", errno);
        free(p_int);
        goto FINISH;
    }
    free(p_int);

    my_addr.sin_family = AF_INET ;
    my_addr.sin_port = htons(host_port);
    
    memset(&(my_addr.sin_zero), 0, 8);
    my_addr.sin_addr.s_addr = INADDR_ANY ;
    
    if( bind( hsock, (sockaddr*)&my_addr, sizeof(my_addr)) == -1 ){
        fprintf(stderr,"Error binding to socket, make sure nothing else is listening on this port %d\n",errno);
        goto FINISH;
    }
    if(listen( hsock, 10) == -1 ){
        fprintf(stderr, "Error listening %d\n",errno);
        goto FINISH;
    }

    //Now lets do the server stuff

    addr_size = sizeof(sockaddr_in);
    
    while(true){
        printf("waiting for a connection\n");
        csock = (int*)malloc(sizeof(int));
        if((*csock = accept( hsock, (sockaddr*)&sadr, &addr_size))!= -1){
            printf("---------------------\nReceived connection from %s\n",inet_ntoa(sadr.sin_addr));
            pthread_create(&thread_id,0,&SocketHandler, (void*)csock );
            pthread_detach(thread_id);
        }
        else{
            fprintf(stderr, "Error accepting %d\n", errno);
        }
    }
    
FINISH:
;
}

void* SocketHandler(void* lp){
    int *csock = (int*)lp;

    char buffer[1024];
    int buffer_len = 1024;
    int bytecount;

    memset(buffer, 0, buffer_len);
    if((bytecount = recv(*csock, buffer, buffer_len, 0))== -1){
       // fprintf(stderr, "Error receiving data %d\n", errno);
        //goto FINISH
    }
    printf("Received bytes %d\nReceived string \"%s\"\n", bytecount,buffer);
    
	
    SSL_WrapperPacket packet;
	
	std::string message = std::string(buffer, bytecount);

    int mode = packet.ParseFromString(message);

    SSL_DetectionFrame packetFrame;

    packetFrame = packet.detection();
    SSL_GeometryData packetGeo = packet.geometry();
    SSL_GeometryFieldSize packetField = packetGeo.field();

	printf("\nParse status: %d",mode);


    //printf("\n%d",packetField.field_length());
    //printf("\n%d",packetField.field_width());
	
    printf("\nNumber of b: %d",packet.detection().balls_size());
    printf("\nNumber of r: %d",packet.detection().robots_yellow_size());
    printf("\nTimestamp: %f\n",packetFrame.t_sent());
	SSL_DetectionBall ball;
    if(packetFrame.balls_size() > 0){
    	
    	ball = packetFrame.balls(0);
    	float x = ball.x();
    	float y = ball.y();
    	float z = ball.z();
    	printf("x: %f, y: %f, z: %f",x,y,z);
    }
    for(int i = 0; i < packet.detection().robots_yellow_size(); i++){
	SSL_DetectionRobot packetR;
	packetR = packet.detection().robots_yellow(i);
	float x = packetR.x();
	float y = packetR.y();
	float orientation = packetR.orientation();
	printf("\nRobot id %d, x: %f, y: %f, orientation: %f",i,x,y,orientation);
    }


	

	//updates info for all bots
	for(int i = 0; i < 5; i++){
		Ybots[i].Update(&packetFrame);
		Bbots[i].Update(&packetFrame);
	}

	for(int i = 0; i < 4; i++){
		Ybots[i].Teleport(10,10);
		Bbots[i].Teleport(10,10);
	}

	//Ybots[4].Move(1000,1000,1);
	//Ybots[4].Rotate(1.57);
	printf("\nCount is %d",count++);
	//Ybots[4].Kick(5);

	

	Ybots[4].Prepare(ball.x(),ball.y(),1,3025,0);






	//start1v1();
/*	
if(packetBall.x() > -3000 && packetBall.x() < 3000 && packetBall.y() > -2000 && packetBall.y() < 2000)
		scoring(&packetFrame);
	else
		chaseBall(&packetFrame);

	goalie(&packetFrame);
*/

	
	
	close(*csock);
	/*    
	strcat(buffer, " SERVER ECHO");

    if((bytecount = send(*csock, buffer, strlen(buffer), 0))== -1){
        fprintf(stderr, "Error sending data %d\n", errno);
        goto FINISH;
    }
    
    printf("Sent bytes %d\n", bytecount);
	*/


FINISH:
    free(csock);
    return 0;
}
