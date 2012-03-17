#ifndef ROLES_H
#define ROLES_H

//Require headers from grSim packet, SSL_Dectection packet in imeplementation
#include "Robot.h"

//abstract class
class Role{
    Role();                //constructor
    
    //modifies the grSim packet within curBot with intended commands
    //this method should only modify the grSim packet, not send it
    virtual void Run(Robot *curBot);
};

//a type of Role (defender)
class DefenderRole : public Role{
    //Require an additional field to specify defending parameters
        // an area to cover?
        // a enemy player to cover?
        // block call path to goal?
    //void *behavior;
        
    //Run should implement the behavior specified
    void Run(Robot *curBot);
};

#endif
