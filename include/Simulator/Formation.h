
#ifndef FORMATION_H_
#define FORMATION_H_
#include <vector>
#include "Simulator/PhysicsVector.h"

using namespace std;

typedef float (*Function)(const float);

// global constants
static const int      DEFAULT_SEED_ID = 3;		// Hard-coded seed ID because it is not selectable

static const Function DEFAULT_FORMATION_FUNCTION = NULL;
static const float    DEFAULT_FORMATION_RELATIVE_ORIENTATION = 90.0f;
static const float    DEFAULT_SENSOR_ERROR = 0;
static const float    DEFAULT_COMMUNICATION_ERROR = 0;
static const int      NO_FUNCTION_FORMATION_ID = -1;
static const float    DEFAULT_FORMATION_RADIUS   = 1.0f;
static const double   X_ROOT_THRESHOLD           = 5E-7;
static const int      X_N_ITERATIONS             = 100;


//class Formation : protected vector<Function>			// Multi-function formations will use this line eventually
class Formation
{
	public:
		Formation();
		Formation(const float radius, const PhysicsVector frp,
				  const vector<int> formationID, const float formationRelativeOrientation);
		Formation(const Formation &f);
		virtual ~Formation();
		Formation& operator=(const Formation &newFormation);

		// Mutators
		void setFunction(const Function newFunction = DEFAULT_FORMATION_FUNCTION);
		void addFunction(const Function additionalFunction);
		void clearFunctions();
		void setFunctionFromFormationID(vector<int> newFormationId);
		void setFormationIDs(int newFormationID);
		void setFormationIDs(vector<int> newFormationIDs);
		void setRadius(float newRadius);
		void setFormationRelativeOrientation(float newFormationRelativeOrientation);
		void setSeedFormationRelativePosition(PhysicsVector newFormationRelativePosition);
		void setSensorError(float newSensorError);
		void setCommunicationError(float newCommunicationError);


		// Accessors
		vector<Function> getFunctions();
		vector<int> getFormationIDs();
		float getRadius();
		float getFormationRelativeOrientation();
		PhysicsVector getSeedFormationRelativePosition();
		int getSeedID();
		vector<PhysicsVector> getRelationships(const PhysicsVector someVector);
		bool isValid();
		float getSensorError();
		float getCommunicationError();

		vector<PhysicsVector> getDesiredRelationships( const vector<Function> intersectingFunctions,
		                               const float intersectingCircleRadius = DEFAULT_FORMATION_RADIUS,
		                               const PhysicsVector centerPosition = PhysicsVector(),
		                               const float  rotationOfRelationship = 0.0f);

//		PhysicsVector getDesiredRelationship( const int   positionOfDesiredFunction   = 0,
//		                       	   	   const float intersectingCircleRadius     = DEFAULT_FORMATION_RADIUS,
//		                       	   	   const PhysicsVector  centerPosition     = PhysicsVector(),
//		                       	   	   const float rotationOfRelationship = 0.0f);

		float getFunctionIntersection(const Function formationToCopy = DEFAULT_FORMATION_FUNCTION,
		                 	 	 	  const float  intersectingCircleRadius = DEFAULT_FORMATION_RADIUS,
		                 	 	 	  const PhysicsVector centerPosition = PhysicsVector(),
		                 	 	 	  const float offset = 0.0f);



		// Member variables
		int seedID;
		Formation fx(int formationId);
		vector<Function> currentFunctions;
		vector<int> formationIDs;									// The identifier for the current formation function
		float radius;										// The desired distance between neighbors
		float sensorError;
		float communicationError;
		float formationRelativeOrientation;					// Orientation of each robot relative to the formation
		PhysicsVector seedFormationRelativePosition;		// FRP that serves as starting point from which formation and relationships will propagate
		PhysicsVector cellFormationRelativePosition;		// p_i in the thesis

};

#endif
