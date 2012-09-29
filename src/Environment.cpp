//
// Describes an environment through which robots figure out their actual and desired positioning.
//

#include <stdio.h>
#include <Simulator/Environment.h>
#include <time.h>
#include <ctime>
#include <stdlib.h>
#include <angles/angles.h>


Environment::Environment()
{}

// Default constructor that initializes this environment to the parameterized values.
Environment::Environment(int numRobots)
{
	numOfRobots = numRobots;
}

// Copy constructor that copies the contents of  the parameterized environment into this environment.
  Environment::Environment(const Environment &env)
{}

Environment::~Environment()
{}

// Everything that the environment continuously does goes here.
void Environment::update(bool doSpin)
{
	ros::Rate loop_rate(10);	// This should be 10 in production code.  A value of 1 is useful for debugging.

	while(ros::ok())
	{
	    loop_rate.sleep();
		ros::spinOnce();
	}
}


string Environment::generateSubMessage(int cellID)
{
	stringstream ss;						//create a stringstream
	ss << (cellID);							//add number to the stream
	string  nbrID = ss.str();
	string subString = "/robot_/state";

	subString.insert(7, nbrID);
	return subString;
}


// Sets the response(relationship vector) based on the requests(IDs).  This is a callback.
bool Environment::setActualRelationshipMessage(NewSimulator::Relationship::Request &request, NewSimulator::Relationship::Response &response)
{
	// Get the relationship between the origin cell and the target cell from rviz

	string requestingCell = "/sphero/base_link";
	string targetCell = "/sphero/base_link";

	stringstream originStringStream;
	originStringStream << (request.OriginID);
	string  originID = originStringStream.str();
	requestingCell.insert(7, originID);

	stringstream targetStringStream;
	targetStringStream << (request.TargetID);
	string targetID = targetStringStream.str();
	targetCell.insert(7, targetID);

	// This will return the ACTUAL RELATIVE position (actual relationship) between 2 frames/cells
	PhysicsVector relationshipVector = getTransform(targetCell, requestingCell);
	response.theRelationship.actual_relationship.x = relationshipVector.x;
	response.theRelationship.actual_relationship.y = relationshipVector.y;
	response.theRelationship.actual_relationship.z = relationshipVector.z;


	// This will return the ACTUAL POSITION of the requesting cell (for debugging purposes)
//	PhysicsVector actualPosition = getActualPosition(requestingCell);
//	response.theRelationship.actual_relationship.x = actualPosition.x;
//	response.theRelationship.actual_relationship.y = actualPosition.y;
//	response.theRelationship.actual_relationship.z = actualPosition.z;
	return true;
}

// Get the actual position of a cell relative to the world frame.
PhysicsVector Environment::getActualPosition(string tfOriginName)
{
	return getTransform(tfOriginName, "/world");
}

// Get the relative pose (x, y, theta) of origin cell to target cell.  Names should be in the format "/sphero1/base_link"
// This is effectively the distance between cells
PhysicsVector Environment::getTransform(string tfOriginName, string tfTargetName)
{
	ros::Time time = ros::Time(0);
	double waitTime = 0.0;

	PhysicsVector transformValues(0,0,0);

	if (tfTargetName.empty() || tfOriginName.empty())
		return transformValues;

	tf::StampedTransform transform;

	try
	{
		if (waitTime > 0.00l)
			spheroTransformListener.waitForTransform(tfOriginName, tfTargetName, time, ros::Duration(waitTime));

		spheroTransformListener.lookupTransform(tfOriginName, tfTargetName, time, transform);
	}
	catch (tf::TransformException &ex)
	{
		ROS_DEBUG("%s", ex.what());
		return transformValues;
	}

	// Get position (x, y, z)
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();


	// Get orientation (roll, pitch, yaw)
	btScalar roll = 0.0, pitch = 0.0, yaw = 0.0;
	btMatrix3x3 rotationMatrix(transform.getRotation().asBt());
	rotationMatrix.getEulerYPR(yaw, pitch, roll);

	transformValues.x = x;
	transformValues.y = y;
	transformValues.z = yaw;

	return transformValues;
}

// Starts the environment's relationship service server
void Environment::startActualRelationshipServiceServer()
{
	int argc = 0;
	char **argv = 0;
	ros::init(argc, argv, "relationship_server");

	relationshipService = RelationshipServerNode.advertiseService("relationship", &Environment::setActualRelationshipMessage, this);
	cout << "\n*** Now serving the " << relationshipService.getService() << " service from the environment ***\n\n";

	ros::spinOnce();
}

