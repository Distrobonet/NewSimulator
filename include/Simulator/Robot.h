
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
		Vector actualPosition;
		Vector desiredPosition;
		State currentState;

		void findNeighbors();
		void gotUpdate();
		bool move(Vector desiredPosition, Vector actualPosition);
		Vector getPosition();

		void updateFormation();
		void updateDesiredPosition();


};


#endif
