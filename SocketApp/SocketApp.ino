/*
 * Socket App
 *
 * A simple socket application example using the WiShield 1.0
 */

#include <WiShield.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <TimerOne.h>

#define PI 3.141592654

#define WIRELESS_MODE_INFRA	1
#define WIRELESS_MODE_ADHOC	2

//-- Update the TICKER_MAX and _STEP to 16 16 or 32 8 or 64 4 for performance vs colors available
#define TICKER_MAX 64
#define TICKER_STEP 4

int prevFL = 0;
int prevFR = 0;
int prevBK = 0;

//-- May have to tweak the timer
int timerDelay = 300;

//Pin connected to ST_CP (pin 12) of 74HC595
int latchPin = 3;
//Pin connected to SH_CP (pin 11) of 74HC595
int clockPin = 4;
//Pin connected to DS (pin14) of 74HC595
int dataPin = 5;
//======================

//number of Shift Registers - set to 1 for this example
int srcount = 1;
byte srval = 0;
int ticker = 0;

byte srPins[8] = {
  0,0,0,0,0,0,0,0
};

void pwmWrite(int port, byte val){
  srPins[port] = val;  
}

void pwmProcess(){
  ticker = (ticker+1)%TICKER_MAX;
  int myPos = ticker * TICKER_STEP;
  int myLev = 0;
  byte currVal = 0;

  for( int i = 0 ; i < 8 ; i++ ){
    myLev = 0;
    if (srPins[i] > myPos)
      myLev = 1;
    bitWrite(currVal,i,myLev );
  }

  srval = currVal;

  shiftRegisterWrite();
}

void shiftRegisterWrite()
{
// bit operations seems to be faster than digital write
  bitClear(PORTD, latchPin);
  shiftOut(dataPin, clockPin, MSBFIRST, srval);
  bitSet(PORTD, latchPin);
}

void allTo(int val, int delayval){
  for (int i = 0 ; i < 8 ; i++){
    pwmWrite(i,val);
    if( delayval > 0 )
      delay(delayval);
  }
}


void allOff(){
  allTo(0,0);
}
void allOn(){
  allTo(255,0);
}

void setMotors(int motor1, int motor2, int motor3)
{
  prevFL = motor1;
  prevFR = motor2;
  prevBK = motor3;
  
  pwmWrite(1,255);
  pwmWrite(3,255);
  pwmWrite(5,255);
  
  if (motor1 < 0)
    pwmWrite(1,0);  
  if (motor2 < 0)
    pwmWrite(3,0);
  if (motor3 < 0)
    pwmWrite(5,0);
    
  pwmWrite(2,abs(motor1));
  pwmWrite(4,abs(motor2));
  pwmWrite(6,abs(motor3));
}

class instruction{
  public:
    bool compound;
    double FL1;
    double FR1;
    double BK1;
    double FL2;
    double FR2;
    double BK2;
    
    instruction(double a1, double a2, double a3){
       compound = false;
       FL1 = 255 * a1;
       FR1 = 255 * a2;
       BK1 = 255 * a3;
    }
    
    instruction(double a11, double a21, double a31, double a12, double a22, double a32){
       compound = true;
       FL1 = 255 * a11;
       FR1 = 255 * a21;
       BK1 = 255 * a31;
       FL2 = 255 * a12;
       FR2 = 255 * a22;
       BK2 = 255 * a32;
    }
    
    instruction(){
    }
};

instruction INSTR;
bool useInstr = false;

char *cmd_message = "0 0                                            \0";

// Wireless configuration parameters ----------------------------------------
unsigned char local_ip[] = {192,168,1,147};	// IP address of WiShield
unsigned char gateway_ip[] = {192,168,1,1};	// router or gateway IP address
unsigned char subnet_mask[] = {255,255,255,0};	// subnet mask for the local network
const prog_char ssid[] PROGMEM = {"WarBots RoboCup"};		// max 32 bytes

unsigned char security_type = 0;	// 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2

// WPA/WPA2 passphrase
const prog_char security_passphrase[] PROGMEM = {"password"};	// max 64 characters

// WEP 128-bit keys
// sample HEX keys
prog_uchar wep_keys[] PROGMEM = {	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,	// Key 0
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00,	// Key 1
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00,	// Key 2
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00	// Key 3
								};

void stopMovement(){
   //reverse direction to stop stray wheels
   setMotors(-prevFL, -prevFR, -prevBK);
   delay(10);  //might want to replace this delay
   setMotors(0, 0, 0);
}

void moveMotors(int m1, int m2, int m3){
                        Serial.print("motor1: ");
                        Serial.println(m1);
                        
                        Serial.print("motor2: ");
                        Serial.println(m2);
                        
                        Serial.print("motor3: ");
                        Serial.println(m3);
                        setMotors(m1,m2,m3);
}

void rotate(double spd){
      int power = (int)(255.0 * spd);
      setMotors(power, power, power);
}

void runInstr(){
    if(!INSTR.compound){
        setMotors(INSTR.FL1, INSTR.FR1, INSTR.BK1);
    }
    else{
        setMotors(INSTR.FL1, INSTR.FR1, INSTR.BK1);
        delay(100);
        setMotors(INSTR.FL2, INSTR.FR2, INSTR.BK2);
        delay(100);
        stopMovement();
    }
}

float cross_product(float*vec1,float*vec2)
{
  return vec1[0]*vec2[1] - vec1[1]*vec2[0];
}


void process_message(){
               
        if(strlen(cmd_message) < 2){
              return;
        }
        
    	char * pch;
    	int command = 0;
    	double arguments[4] = {0.0};
    
        pch = strtok(cmd_message," ,\n");
    
    	//command type
    	command = atoi(pch);
    
    	pch = strtok (NULL, " ,\n");
    	int counter = 0;
        while (pch != NULL)
        {
    		arguments[counter] = atof(pch);		
    		printf("%f\n", arguments[counter]);
    		counter++;
            pch = strtok (NULL, " ,\n");
    
        }
    
        useInstr = false;
        double v;
        double dir, theta;
        double v1;
        double v2;
        
    	//execute
    	switch(command){
                
    		case 0:
    			//stop
    			//arguments = {roller, 0, 0, 0}
    			Serial.print("stop, with roller speed = ");
                        Serial.println(arguments[0]);
                        stopMovement();
    			break;
    		case 1:
    			//move: send motor speeds
    			//arguments = {v, v_dir (radians), roller, 0}
                        stopMovement();
                        double v, dir;
                        v = arguments[0];
                        dir = arguments[1];
                        
                        float posM1[2]; posM1[0] = 0.5; posM1[1] = 0.866;
                        float posM2[2]; posM2[0] = -1; posM2[1] = 0;
                        float posM3[2]; posM3[0] = 0.5; posM3[1] = -0.866;
                        
                        float newVel[2]; newVel[0] = v*cos(dir); newVel[1] = v*sin(dir);
                        
                        int tempScalar; tempScalar = 255;
                        
                        int motor1; motor1 = int(tempScalar*v*cross_product(posM1,newVel));
                        int motor2; motor2 = int(tempScalar*v*cross_product(posM2,newVel));
                        int motor3; motor3 = int(tempScalar*v*cross_product(posM3,newVel));
                        
                        Serial.print("motor1: ");
                        Serial.println(motor1);
                        
                        Serial.print("motor2: ");
                        Serial.println(motor2);
                        
                        Serial.print("motor3: ");
                        Serial.println(motor3);
                        
                        Serial.print("v: ");
                        Serial.print(v);
                        Serial.print("\t dir: ");
                        Serial.println(dir);
                        
                        moveMotors(motor1,motor2,motor3);
                        
    			break;
    		case 2:
    			//rotate: +ve is counterclockwise
    			//arguments = {v_ROT, roller, 0, 0}    
                        Serial.print("rotate: v_ROT = ");
                        Serial.print(arguments[0]);
                        Serial.print(" roller speed = ");
                        Serial.println(arguments[1]);
                        
                        rotate(arguments[0]);
    			break;
    		case 3:
    			//kick
    			//arguments = {roller, 0, 0, 0}
    			Serial.print("kick, with roller speed = ");
                        Serial.println(arguments[3]);
                        //stopMovement();
    			break;
    	}
}


// setup the wireless mode
// infrastructure - connect to AP
// adhoc - connect to another WiFi device
unsigned char wireless_mode = WIRELESS_MODE_INFRA;

unsigned char ssid_len;
unsigned char security_passphrase_len;
//---------------------------------------------------------------------------

void setup()
{
        Serial.begin(9600);
	WiFi.init();
        Serial.print("---Program");
        Serial.println(" Begins---");
        pinMode(latchPin, OUTPUT);
        pinMode(clockPin, OUTPUT);
        pinMode(dataPin, OUTPUT);
       
        digitalWrite(latchPin,LOW);
        digitalWrite(dataPin,LOW);
        digitalWrite(clockPin,LOW);
      
        Timer1.initialize(timerDelay); // Timer for updating pwm pins
        Timer1.attachInterrupt(pwmProcess);
}

void loop()
{
        process_message();
	WiFi.run();
}
