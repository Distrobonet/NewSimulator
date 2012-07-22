#include <iostream>
#include <Simulator/Cell.h>
#include <Simulator/Formation.h>
#include <Simulator/State.h>
#include <vector>

using namespace std;

Cell::Cell(const int ID)
{
	cellID = ID;
	cellState = State();
	cellFormation = Formation();
}

Cell::~Cell()
{
	// TODO Auto-generated destructor stub
}

// This is where most of the magic happens
void Cell::update()
{
	getFormation();
}

int Cell::getCellID() {
	return cellID;
}

void Cell::setCellID(int newID) {
	cellID = newID;
}

Formation Cell::getFormation() {
	return cellFormation;
}

void Cell::setFormation(Formation formation) {
	cellFormation = formation;
}

vector<int> Cell::getNeighborhood() {
	return neighborhoodList;
}
void Cell::setNeighborhood(vector<int> neighborhood) {
	neighborhoodList = neighborhood;
}

State Cell::getState() {
 return cellState;
}

void Cell::setState(State state) {
 cellState = state;
}

// Translates the robot relative to itself based on the parameterized translation vector.
void Cell::translateRelative(float dx , float dy)
{
//	  vector.rotateRelative(getHeading());
//    x += vector.x;
//    y += vector.y;
}

void Cell::rotateRelative(float theta)
{
	theta = degreesToRadians(theta);
	x = x * cos(theta)- y * sin(theta);
	y = x * sin(theta) + y * cos(theta);
	//z = z;

}







