#include <Simulator/Formation.h>

using namespace std;

Formation::Formation() {
	setRadius(1);
	setFormation(0);
	setFormationRelativeOrientation(0);

}

Formation::~Formation() {

}

int Formation::getFormation() {
	return formationID;
}

void Formation::setFormation(int formation) {
	formationID = formation;
}

int Formation::getRadius() {
	return radius;
}

void Formation::setRadius(int newRadius) {
	radius = newRadius;
}

float Formation::getFormationRelativeOrientation() {
	return formationRelativeOrientation;
}

void Formation::setFormationRelativeOrientation(float fro) {
	formationRelativeOrientation = fro;
}
Vector Formation::getSeedFormationRelativePosition() {
	return seedFormationRelativePosition;
}

void Formation::setSeedFormationRelativePosition(Vector frp) {
	seedFormationRelativePosition = frp;
}
