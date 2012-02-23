#ifndef ROBOT_H
#define ROBOT_H

class SSL_DetectionFrame;
class grSim_Packet;
class Robot {
	public:
		float x,y,orientation,velT,velA,velN,velK;
		bool yellow;
		int id;
	public:
		Robot();
		Robot(int id, bool yellow);
		void Update(SSL_DetectionFrame* frame);
		void Kick(float vel);
		void Move(float x1, float y1, float vel);
		void Teleport(float x1, float y1);
		void Rotate(float angle);
		bool Prepare(float x1, float y1, float vel, float x2, float y2);
		//void ChaseBall(float x1, float y1, float vel);
	protected:
		void sendPacket(grSim_Packet packet);
};
		
#endif
