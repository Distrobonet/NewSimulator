
#include <Simulator/Formation.h>

using namespace std;


Formation::Formation()
{
	setRadius(1);
	setFormationID(0);
	setFormationRelativeOrientation(0);

	seedFormationRelativePosition.x = 0.0f;
	seedFormationRelativePosition.y = 0.0f;
	seedFormationRelativePosition.z = 0.0f;
}

Formation::Formation(const float radius, const PhysicsVector frp,
		const int formationID, const float formationRelativeOrientation)
{
	setFormationID(formationID);
	setRadius(radius);
	setSeedFormationRelativePosition(frp);
	setFormationID(formationID);
	setFormationRelativeOrientation(formationRelativeOrientation);
}

Formation::Formation(const Formation &f) {
	*this = f;
}

Formation& Formation::operator=(const Formation &f) {
	radius = f.radius;
	formationID = f.formationID;
	formationRelativeOrientation = f.formationRelativeOrientation;
	setSeedFormationRelativePosition(f.seedFormationRelativePosition);

	return *this;
}

Formation::~Formation()
{

}

int Formation::getFormation()
{
	return formationID;
}

void Formation::setFormationID(int formationID)
{
	this->formationID = formationID;
}

int Formation::getRadius()
{
	return radius;
}

void Formation::setRadius(int newRadius)
{
	radius = newRadius;
}

float Formation::getFormationRelativeOrientation()
{
	return formationRelativeOrientation;
}

void Formation::setFormationRelativeOrientation(float fro)
{
	formationRelativeOrientation = fro;
}
PhysicsVector Formation::getSeedFormationRelativePosition()
{
	return seedFormationRelativePosition;
}

void Formation::setSeedFormationRelativePosition(PhysicsVector frp)
{
	seedFormationRelativePosition.x = frp.x;
	seedFormationRelativePosition.y = frp.y;
	seedFormationRelativePosition.z = frp.z;

    for (int i = 0; i < 3; ++i)
    {
    	seedFormationRelativePosition.translate[i]  = frp.translate[i];
    	seedFormationRelativePosition.rotate[i]     = frp.rotate[i];
    	seedFormationRelativePosition. scale[i]     = frp.scale[i];
    }
    seedFormationRelativePosition. showLine         = frp.showLine;
    seedFormationRelativePosition.showHead          = frp.showHead;
}
