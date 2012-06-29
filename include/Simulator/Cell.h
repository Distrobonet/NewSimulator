
#ifndef CELL_H_
#define CELL_H_

#include <vector>

#include "Simulator/State.h"
#include "Simulator/Formation.h"

using namespace std;

class Cell
{
	public:
		Cell(const int cellId);
		virtual ~Cell();

	protected:
		State cellState;
		Formation formation;
		vector< pair<int,int> > neighborhoodList;
		bool update();
		int cellId;
};

#endif
