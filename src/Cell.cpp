#include <iostream>
#include <Simulator/Cell.h>
#include <vector>

using namespace std;

Cell::Cell(const int ID)
{
	cellID = ID;
	update();
}

Cell::~Cell()
{
	// TODO Auto-generated destructor stub
}

bool Cell::update()
{
	getFormation();
	return true;
}

int Cell::getCellID() {
	return cellID;
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
