
#ifndef FORMATION_H_
#define FORMATION_H_
#include <Simulator/Vector.h>

using namespace std;

class Formation {
	public:
		Formation();
		virtual ~Formation();

		int getFormation();
		void setForamation(int formation);
		int getRadius();
		void setRadius(int radius);
		float getformationRelativeOrientation();
		void setformationRelativeOrientation(float fro);
		Vector getseedFormationRelativePosition();
		void setseedFormationRelativePosition(Vector frp);

//		Formation fx(int formationId);
		int formationID;
		int radius;
		float formationRelativeOrientation;
		Vector seedFormationRelativePosition;
};

#endif
