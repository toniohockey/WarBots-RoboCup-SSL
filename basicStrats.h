#ifndef BASICSTRATEGY_H
#define BASICSTRATEGY_H

//for basics strategies
//strategies should be grouped in different files and included

#include "Robot.h"
#include "Strategy.h"

class basicDefStrat : public Strategy{
    
    bool Plan(SSL_DetectionFrame *field);           //Simple 1v1 defence
};

class basicAtkStrat : public Strategy{
    
    bool Plan(SSL_DetectionFrame *field);           //uhh....think of some plan
};

#endif
