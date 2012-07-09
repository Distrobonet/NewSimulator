
#ifndef STATE_H_
#define STATE_H_

#include <Simulator/Formation.h>
#include <Simulator/Relationship.h>
#include <Simulator/Vector.h>

using namespace std;

class State
{
	public:
		State();
		virtual ~State(){};

		vector<Relationship> desiredRelationship;
		vector<Relationship> actualRelationship;
		Vector formationRelativePosition;
		Vector translationalError;
		float rotationalError;
		int tStep;
};

State::State()
{
//	desiredRelationship = new vector<Relationship>;
}




#endif
