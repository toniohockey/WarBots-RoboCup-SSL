
#include "Robot.h"
#ifndef GOALIE_H
#define GOALIE_H

class SSL_DetectionFrame;
class grSim_Packet;
class Goalie: public Robot {
	public:
		Goalie (int id_in, bool yellow_in);
		void Defend(float x1, float y1);
};


#endif
