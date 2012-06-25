////
//// Filename:        "State.h"
////
//// Description:     This structure defines a robot cell state.
////
//
//// preprocessor directives
//#ifndef STATE_H
//#define STATE_H
//#include <Simulator/Formation.h>
//#include <Simulator/Vector.h>
//#include <ros/serialization.h>
//#include <../msg_gen/cpp/include/Simulator/StateMessage.h>
//
//using namespace std;
//
//
//
//// defines a robot cell state
//struct State
//{
//
//	public:
//		// <data members>
//		Formation            formation;     // the current formation
//		Vector               frp;     		// the formation relative position
//		vector<Relationship> rels;  		// the formation relationships
//		Vector               transError;    // the summed translational error
//		float                rotError;      // the summed rotational error
//		int                  tStep;         // the time step in the formation
//		int                  refID;         // the ID of the reference neighbor
//		float                temperature;   // the current temperature
//		float                heat;          // the current heat
//		bool 				 inPosition;
//
//		// Default constructor that initializes
//		// this state to the parameterized values.
//		State(const Formation            f      = Formation(),
//			  const Vector               formationrelativepos   = Vector(),
//			  const vector<Relationship> rel      = vector<Relationship>(),
//			  const Vector               tError = Vector(),
//			  const float                rError = 0.0f,
//			  const int                  ts     = 0,
//			  const int                  rID    = -1,
//			  const float                temp   = 0.0f,
//			  const float                h      = 0.0f)
//			  : formation(f),       frp(formationrelativepos),   rels(rel),
//				transError(tError), rotError(rError), tStep(ts), refID(rID),
//				temperature(temp),  heat(h), inPosition(false)
//		{
//		}   // State(const..{Formation, Vector, LL<Relationship>, Vector, int,int})
//};
//
//
//
//
//
//
//#endif
