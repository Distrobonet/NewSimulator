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
