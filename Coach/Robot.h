#ifndef ROBOT_H
#define ROBOT_H

class SSL_DetectionFrame;
class grSim_Packet;
class Robot {
	protected:
		int state;
		SSL_DetectionFrame* current;
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
		void Move(float angle, float vel);
		void PathTo(float x1, float y1, float vel);
		void Teleport(float x1, float y1);
		void Rotate(float angle);
		void Prepare(float x1, float y1, float vel, float x2, float y2);
		//void ChaseBall(float x1, float y1, float vel);
	protected:
		void sendPacket(grSim_Packet packet);
		bool interference(float x1, float y1, float x2, float y2, float x3, float y3, float radius);
		bool inTheWay(float x1, float x2, float x3, float y1, float y2, float y3, bool v);
		float distance(float x1, float x2, float y1, float y2);
};
		
#endif
