
#ifndef STATE_H_
#define STATE_H_
#include <Simulator/Formation.h>
using namespace std;

struct State
{
    Formation formation;

    State(const Formation formation = Formation())
          : formation(formation){}

};






#endif
