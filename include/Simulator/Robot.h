
#ifndef ROBOT_H
#define ROBOT_H

using namespace std;

class Robot: public Cell
{
    public:
		Robot();
		virtual ~Robot(){};

		int ID;
		int leftNeighborId;
		int rightNeighborId;
		PhysicsVector actualPosition;
		PhysicsVector desiredPosition;
		State currentState;

		void findNeighbors();
		void gotUpdate();
		bool move(PhysicsVector desiredPosition, PhysicsVector actualPosition);
		PhysicsVector getPosition();

		void updateFormation();
		void updateDesiredPosition();


};


#endif
