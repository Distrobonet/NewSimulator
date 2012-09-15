
#ifndef STATE_H_
#define STATE_H_

#include <Simulator/Formation.h>
#include <Simulator/Relationship.h>
#include <Simulator/PhysicsVector.h>

using namespace std;

class State
{
	public:
		State(){};
		virtual ~State(){};

		vector<Relationship> desiredRelationships;		// R^2 = (Vx - Pix)^2 + (f(Vx) - Piy)^2   page 15-16 of thesis
		vector<Relationship> actualRelationships;		// k | Pk ’ = min({||Pi-1  – Pseed ||, ||Pi+1  – Pseed ||})  page 16-17 of thesis
		PhysicsVector formationRelativePosition;
		PhysicsVector translationalError;				// γi  = Γk  + Rk→i,des ' – Rk→i,act
		float rotationalError;
		int timeStep;
};



#endif
