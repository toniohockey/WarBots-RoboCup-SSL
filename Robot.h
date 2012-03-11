#ifndef ROBOT_H
#define ROBOT_H

#include "Roles.h"
#include "BasicTypes.h"

class SSL_DetectionFrame;
class grSim_Packet;

struct RobotState{
    double x, y, orientation;
    Vector2D speed;
};

class Robot{
  public:
    grSim_Packet command;                   //the command to be sent
    SSL_DetectionFrame *field;              //pointer, since the field status is universal
    
    //command to be executed
    Role *role;                             //has built-in run function
    Task *task;                             //higher priority than role
    
    //Robot info
    int id;
    bool yellow;
    RobotState state;
    
    //constructors
    Robot();
    Robot(int id, bool yellow);
    
    //prepares the command packet with task or role then sends it
    void makeMove();    
    
    //Robot update/teleport is left to the coach
    
    //robot movements (should only modify command packet)
    void Kick(double kickSpeed);                //turns on the kicker with kickSpeed
    
    void Rotate(double delta);                  //rotate robot for delta radians
    void Rotate(double angle);                  //rotate robot to angle orientation
	
    //move to target coordinate with speed
	void Move(double targetX, double targetY, double speed);
	void Move(Point2D target, double speed);    //calls above fn with fields in target
	void Move(double angle, double speed);
	void Move(Vector2D vec);                    //calls above fn with fields in vec
	
	//pass to target coordinates with speed
	void Pass(double targetX, double targetY, double speed);
	void Pass(Point2D target, double speed);    //calls above fn with fields in target

};

#endif
