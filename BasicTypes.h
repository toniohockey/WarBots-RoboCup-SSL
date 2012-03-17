#ifndef BASICTYPES_H
#define BASICTYPES_H

#include "Robot.h"

struct Vector2D{
    double mag;                 //magitude
    double direction;           //direction in radians
};

struct Vector3D{
    double mag;                 //magitude
    double alpha, beta, gamma;
};

struct Point2D{
    double x, y;                //position
};

struct Point3D{
    double x, y, z;             //position
};

Enum taskType{MOVE, KICK, FOLLOW, AIM};

class Task{
    bool blocking;
    taskType type;
    vector<void*> param;                //vector of anything
    
    Task();                             //blocking false by default
    void Run(Robot *curBot);            //prepares command in curbot based on the task
};

#endif
