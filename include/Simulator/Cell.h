
#ifndef CELL_H_
#define CELL_H_

#include "Simulator/State.h"
#include "Simulator/Formation.h"
#include <vector>

using namespace std;

class Cell: public Formation, public State
{
	public:
		Cell(const int cellId);
		virtual ~Cell();

		void update();
		int getCellID();
		void setCellID(int ID);
		Formation getFormation();
		void setFormation(Formation formation);
		vector<int> getNeighborhood();
		void setNeighborhood(vector<int> neighborhood);
		State getState();
		void setState(State state);

	protected:
		State cellState;
		Formation cellFormation;
		vector<int> neighborhoodList;
		int cellID;
};

#endif
