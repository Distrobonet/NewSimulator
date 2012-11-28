
#include <Simulator/Formation.h>
#include "Simulator/functions.h"

using namespace std;

Formation::Formation()
{
	seedID = DEFAULT_SEED_ID;
	setRadius(DEFAULT_FORMATION_RADIUS);
	setFormationIDs(NO_FUNCTION_FORMATION_ID);
	setFormationRelativeOrientation(DEFAULT_FORMATION_RELATIVE_ORIENTATION);

	seedFormationRelativePosition.x = 0.0f;
	seedFormationRelativePosition.y = 0.0f;
	seedFormationRelativePosition.z = 0.0f;
}

Formation::Formation(const float radius, const PhysicsVector frp, const vector<int> formationIDs, const float formationRelativeOrientation)
{
	seedID = DEFAULT_SEED_ID;
	setRadius(radius);
	setFormationIDs(formationIDs);
	setSeedFormationRelativePosition(frp);
	setFormationRelativeOrientation(formationRelativeOrientation);
}

// Copy constructor
Formation::Formation(const Formation &newFormation)
{
	*this = newFormation;
}

Formation& Formation::operator=(const Formation &newFormation)
{
	radius = newFormation.radius;
	setFormationIDs(newFormation.formationIDs);
	formationRelativeOrientation = newFormation.formationRelativeOrientation;
	setSeedFormationRelativePosition(newFormation.seedFormationRelativePosition);

	return *this;
}

Formation::~Formation()
{
}

void Formation::setFunction(const Function newFunction)
{
	currentFunctions.clear();
	currentFunctions.push_back(newFunction);
}

void Formation::addFunction(const Function additionalFunction)
{
	currentFunctions.push_back(additionalFunction);
}

void Formation::clearFunctions() {
	currentFunctions.clear();
}

void Formation::setFunctionFromFormationID(vector<int> newFormationId)
{
	// TODO: fix the vector coming in...
	int tempNewFormationID = newFormationId.at(0);
	clearFunctions();

	switch(tempNewFormationID)
	{
		case 0:
			setFunction(line);
			break;
		case 1:
			setFunction(x);
			break;
		case 2:
			setFunction(absX);
			break;
		case 3:
			setFunction(negHalfX);
			break;
		case 4:
			setFunction(negAbsHalfX);
			break;
		case 5:
			setFunction(negAbsX);
			break;
		case 6:
			setFunction(parabola);
			break;
		case 7:
			setFunction(cubic);
			break;
		case 8:
			setFunction(condSqrt);
			break;
		case 9:
			setFunction(sine);
			break;
		case 10:
			setFunction(x);
			addFunction(negX);
			break;
		case 11:
			setFunction(xRoot3);
			break;
		case 12:
			setFunction(negXRoot3);
			break;
		default:
			setFunction(line);
			break;
	}
}

vector<Function> Formation::getFunctions()
{
	return currentFunctions;
}

vector<int> Formation::getFormationIDs()
{
	return formationIDs;
}

void Formation::setFormationIDs(int newFormationID)
{
	formationIDs.clear();
	formationIDs.push_back(newFormationID);
}

void Formation::setFormationIDs(vector<int> newFormationIDs)
{
	formationIDs.clear();
	for(uint i = 0; i < newFormationIDs.size(); i++)
	{
		formationIDs.push_back(newFormationIDs.at(i));
	}
}

float Formation::getRadius()
{
	return radius;
}

void Formation::setRadius(float newRadius)
{
	radius = newRadius;
}

float Formation::getFormationRelativeOrientation()
{
	return formationRelativeOrientation;
}

void Formation::setFormationRelativeOrientation(float newFormationRelativeOrientation)
{
	formationRelativeOrientation = newFormationRelativeOrientation;
}
PhysicsVector Formation::getSeedFormationRelativePosition()
{
	return seedFormationRelativePosition;
}

void Formation::setSeedFormationRelativePosition(PhysicsVector newFormationRelativePosition)
{
	seedFormationRelativePosition.x = newFormationRelativePosition.x;
	seedFormationRelativePosition.y = newFormationRelativePosition.y;
	seedFormationRelativePosition.z = newFormationRelativePosition.z;

    for (int i = 0; i < 3; ++i)
    {
    	seedFormationRelativePosition.translate[i]  = newFormationRelativePosition.translate[i];
    	seedFormationRelativePosition.rotate[i]     = newFormationRelativePosition.rotate[i];
    	seedFormationRelativePosition. scale[i]     = newFormationRelativePosition.scale[i];
    }
}

int Formation::getSeedID()
{
	return seedID;
}

//// This function is useful when multi-function is implemented
//vector<PhysicsVector> Formation::getRelationships(const PhysicsVector c)
//{
//	vector<PhysicsVector> temp;	// See below \/
//	return temp;				// Temporary, just to remove warning.  Remove when implementing multi-function
//
////    if (isEmpty())
////    	return vector<PhysicsVector>();
////
////    vector<PhysicsVector> rels;
////    Function           curr = NULL;
////    for (int i = 0; i < getSize(); ++i)
////    {
////        if (!getHead(curr))
////        	break;
////
////        rels.insertTail(getRelationship(curr, -radius, c, heading));
////        rels.insertTail(getRelationship(curr,  radius, c, heading));
////        ++(*this);
////    }
////    return rels;
//}   // getRelationships(const Vector)


// Uses the secant method to calculate the intersection of the function and a circle centered at
// the parameterized vector position centerPosition with the appropriate radius, returning a vector
// from centerPosition to this intersection.
// Uses the secant method:    x_(n + 1) = x_n - f(x_n) * (x_n - x_(n - 1)) / (f(x_n) - f(x_(n - 1)))
vector<PhysicsVector> Formation::getDesiredRelationships(const vector<Function> intersectingFunctions, const float intersectingCircleRadius,
		const PhysicsVector centerPosition, const float rotationOfRelationship)
{
    if (intersectingFunctions.empty()) {
    	vector<PhysicsVector> temp;
    	temp.push_back(PhysicsVector());
    	return temp;
    }

    vector<PhysicsVector> desiredRelationships;

    for(uint fx = 0; fx < intersectingFunctions.size(); fx++) {
		float xn = centerPosition.x + intersectingCircleRadius + X_ROOT_THRESHOLD;
		float xn_1 = centerPosition.x + intersectingCircleRadius - X_ROOT_THRESHOLD;
		float intersect = 0.0f;
		float error = 0.0f;

		for (int i = 0; i < X_N_ITERATIONS; ++i)
		{
			intersect     = getFunctionIntersection(intersectingFunctions.at(fx), intersectingCircleRadius, centerPosition, xn);
			error         = intersect * (xn - xn_1) /
						   (intersect - getFunctionIntersection(intersectingFunctions.at(fx), intersectingCircleRadius, centerPosition, xn_1));
			if(error > 0)
				error = 0;
			if (abs(error) <= X_ROOT_THRESHOLD){
				break;
			}
			xn_1          = xn;
			xn           -= error;
		}

		Function intFunction = intersectingFunctions.at(fx);
		PhysicsVector tempDesiredRelationship = (PhysicsVector(xn, intFunction(xn)) - centerPosition);
		tempDesiredRelationship.rotateRelative(-rotationOfRelationship);

		desiredRelationships.push_back(tempDesiredRelationship);
    }
    return desiredRelationships;
}


//// Useful when we implement multi-function
//PhysicsVector Formation::getDesiredRelationship(const int positionOfDesiredFunction, const float intersectingCircleRadius,
//		const PhysicsVector  centerPosition, const float rotationOfRelationship)
//{
//	PhysicsVector temp;	// See below \/
//	return temp;		// Temporary, just to remove warning.  Remove when implementing multi-function
//
//	// This will be used when multi-function is implemented
//	//return getRelationship(getFunction(positionOfDesiredFunction), intersectingCircleRadius, centerPosition, rotationOfRelationship);
//}


float Formation::getFunctionIntersection(const Function formationToCopy, const float  intersectingCircleRadius,
										 const PhysicsVector centerPosition, const float offset)
{
	return pow(offset - centerPosition.x, 2.0f) + pow(formationToCopy(offset) - centerPosition.y, 2.0f) - pow(intersectingCircleRadius, 2.0f);
}

bool Formation::isValid()
{
	// if the first formationID is empty, they all should be
	if(formationIDs.at(0) != NO_FUNCTION_FORMATION_ID)
		return true;
	return false;
}

float Formation::getSensorError()
{
	return sensorError;
}

void Formation::setSensorError(float newSensorError)
{
	sensorError = newSensorError;
}

float Formation::getCommunicationError()
{
	return communicationError;
}

void Formation::setCommunicationError(float newCommunicationError)
{
	communicationError = newCommunicationError;
}
