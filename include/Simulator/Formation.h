
#ifndef FORMATION_H_
#define FORMATION_H_
#include <Simulator/Vector.h>

using namespace std;

typedef float (*Function)(const float);

class Formation {
	public:
		Formation(){};
		virtual ~Formation(){};

		Formation fx(int formationId);
		int radius;
		float formationRelativeOrientation;
		Vector seedFormationRelativePosition;
};

#endif
