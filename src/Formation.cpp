#include <Simulator/Formation.h>

using namespace std;

Formation::Formation() {

}

Formation::~Formation() {

}

int Formation::getFormation() {
	return formationID;
}

void Formation::setForamation(int formation) {
	formationID = formation;
}

int Formation::getRadius() {
	return radius;
}

void Formation::setRadius(int newRadius) {
	radius = newRadius;
}

float Formation::getformationRelativeOrientation() {
	return formationRelativeOrientation;
}

void Formation::setformationRelativeOrientation(float fro) {
	formationRelativeOrientation = fro;
}
Vector Formation::getseedFormationRelativePosition() {
	return seedFormationRelativePosition;
}

void Formation::setseedFormationRelativePosition(Vector frp) {
	seedFormationRelativePosition = frp;
}
