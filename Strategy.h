#ifndef STRATEGY_H
#define STRATEGY_H

#include "Robot.h"

//abstract class
class Strategy{

    vector<Robot*> team;
    
    Strategy();                             //empty constructor
    Strategy(vector<Robot*> team);          //constructor that assigns team directly
    
    //returns false if failed
    virtual bool Plan(SSL_DetectionFrame *field);
};

#endif

