#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iostream>
#include <termios.h>
#include <unistd.h>

#define PI 3.14159265

using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int mygetch(void)
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

void send_msg(char* host, char* port, char* msg){
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    portno = atoi(port);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(host);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    n = write(sockfd,msg,strlen(msg));
    if (n < 0) 
         error("ERROR writing to socket");
  
	bzero(buffer,256);
    n = read(sockfd,buffer,255);
    if (n < 0) 
         error("ERROR reading from socket");
    printf("%s\n",buffer);

    close(sockfd);
}

int main(int argc, char *argv[])
{
	char *host = (char*)"192.168.1.149";
	char *port = (char*)"1000";	
	char *msg;
	//send_msg(host, port, msg);

 	int input;
	bool valid = true;

  	while(1){
		input = mygetch();
		valid = true;

		if(input == 27 && mygetch() == 91){
			input = mygetch();
			switch(input){
			case 65:
				//msg = (char*)"up\n";
				msg = (char*)"1 1.0 0 0\n";
				break;
			case 66:
				//msg = (char*)"down\n";
				msg = (char*)"1 1.0 3.141592654 0\n";
				break;
			case 68:
				//msg = (char*)"left\n";
				msg = (char*)"1 1.0 1.570796327 0\n";
				break;
			case 67:
				//msg = (char*)"right\n";
				msg = (char*)"1 1.0 4.712388980 0\n";
				break;
			default:
				valid = false;
				break;
			}
		}
		else{
			switch(input){
			case 115:
				//msg = (char*)"s\n";
				msg = (char*)"0 0\n";
				break;
			case 122:
				//msg = (char*)"z\n";
				msg = (char*)"2 1.0 1\n";
				break;
			case 120:
				//msg = (char*)"x\n";
				msg = (char*)"2 -1.0 1\n";
				break;
			case 32:
				//msg = (char*)"space\n";

				//msg = (char*)"1 1 1 1 1\n";
				//usleep(100000);
				
				break;
			default:
				valid = false;
				break;
			}
		}
		if(valid) send_msg(host, port, msg);
 	}

    return 0;
}
