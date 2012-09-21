
#include <Simulator/Formation.h>

using namespace std;


Formation::Formation()
{
	seedID = DEFAULT_SEED_ID;
	setRadius(1);
	setFormationID(0);
//	setSeedFormationRelativePosition();
	setFormationRelativeOrientation(0);

	seedFormationRelativePosition.x = 0.0f;
	seedFormationRelativePosition.y = 0.0f;
	seedFormationRelativePosition.z = 0.0f;
}

Formation::Formation(const float radius, const PhysicsVector frp, const int formationID, const float formationRelativeOrientation)
{
	seedID = DEFAULT_SEED_ID;
	setRadius(radius);
	setFormationID(formationID);
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
	formationID = newFormation.formationID;
	formationRelativeOrientation = newFormation.formationRelativeOrientation;
	setSeedFormationRelativePosition(newFormation.seedFormationRelativePosition);

	return *this;
}

Formation::~Formation()
{
}

void Formation::setFunction(const Function newFunction)
{
	//clear();						// Clears this vector if functions, should be used when we switch to multifunction.
	//this->push_back(newFunction);	// Adds the new function to this vector.  Will replace the next line.

	currentFunction = newFunction;
}

Function Formation::getFunction()
{
	return currentFunction;
}

int Formation::getFormationID()
{
	return formationID;
}

void Formation::setFormationID(int newFormationID)
{
	formationID = newFormationID;
}

int Formation::getRadius()
{
	return radius;
}

void Formation::setRadius(int newRadius)
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
    seedFormationRelativePosition. showLine         = newFormationRelativePosition.showLine;
    seedFormationRelativePosition.showHead          = newFormationRelativePosition.showHead;
}

int Formation::getSeedID()
{
	return seedID;
}

// This function is useful when multi-function is implemented
vector<PhysicsVector> Formation::getRelationships(const PhysicsVector c)
{
	vector<PhysicsVector> temp;	// See below \/
	return temp;				// Temporary, just to remove warning.  Remove when implementing multi-function

//    if (isEmpty())
//    	return vector<PhysicsVector>();
//
//    vector<PhysicsVector> rels;
//    Function           curr = NULL;
//    for (int i = 0; i < getSize(); ++i)
//    {
//        if (!getHead(curr))
//        	break;
//
//        rels.insertTail(getRelationship(curr, -radius, c, heading));
//        rels.insertTail(getRelationship(curr,  radius, c, heading));
//        ++(*this);
//    }
//    return rels;
}   // getRelationships(const Vector)


// Uses the secant method to calculate the intersection of the function and a circle centered at
// the parameterized vector position centerPosition with the appropriate radius, returning a vector
// from centerPosition to this intersection.
// Uses the secant method:    x_(n + 1) = x_n - f(x_n) * (x_n - x_(n - 1)) / (f(x_n) - f(x_(n - 1))).
PhysicsVector Formation::getRelationship(const Function intersectingFunction, const float intersectingCircleRadius,
		                      	  	  	 const PhysicsVector centerPosition, const float rotationOfRelationship)
{
    if (intersectingFunction == NULL)
    	return PhysicsVector();

    float xn = centerPosition.x + intersectingCircleRadius + X_ROOT_THRESHOLD;
    float xn_1 = centerPosition.x + intersectingCircleRadius - X_ROOT_THRESHOLD;
    float intersect = 0.0f;
    float error = 0.0f;

    for (int i = 0; i < X_N_ITERATIONS; ++i)
    {
        intersect     = getFunctionIntersection(intersectingFunction, intersectingCircleRadius, centerPosition, xn);
        error         = intersect * (xn - xn_1) /
                       (intersect - getFunctionIntersection(intersectingFunction, intersectingCircleRadius, centerPosition, xn_1));
        if (abs(error) <= X_ROOT_THRESHOLD) break;
        xn_1          = xn;
        xn           -= error;
    }

    PhysicsVector relationship;

    relationship =  PhysicsVector(xn, intersectingFunction(xn)) - centerPosition;
    relationship.rotateRelative(-rotationOfRelationship);
    return relationship;
}


// Useful when we implement multi-function
PhysicsVector Formation::getRelationship(const int positionOfDesiredFunction, const float intersectingCircleRadius,
		                      const PhysicsVector  centerPosition, const float rotationOfRelationship)
{
	PhysicsVector temp;	// See below \/
	return temp;		// Temporary, just to remove warning.  Remove when implementing multi-function

	// This will be used when multi-function is implemented
	//return getRelationship(getFunction(positionOfDesiredFunction), intersectingCircleRadius, centerPosition, rotationOfRelationship);
}


float Formation::getFunctionIntersection(const Function formationToCopy, const float  intersectingCircleRadius,
										 const PhysicsVector centerPosition, const float offset)
{
	return pow(offset - centerPosition.x, 2.0f) + pow(formationToCopy(offset) - centerPosition.y, 2.0f) - pow(intersectingCircleRadius, 2.0f);
}

bool Formation::isValid() {
	if(formationID != -1)
		return true;
	return false;
}
