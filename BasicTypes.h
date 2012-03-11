#ifndef BASICTYPES_H
#define BASICTYPES_H

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

class Task{
    //design for task
        //char array (string) and parse?
        //higher order functions?
};

#endif
