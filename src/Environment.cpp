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
	startNeighborhoodServiceServer();
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

	// This will return the ACTUAL RELATIVE position to the SEED cell, which is used to get the FRP
	string seedCell = "/sphero3/base_link";			// Change this once the seed is configurable
	relationshipVector = getTransform(requestingCell, seedCell);
	response.theRelationship.actual_position.x = relationshipVector.x;
	response.theRelationship.actual_position.y = relationshipVector.y;
	response.theRelationship.actual_position.z = relationshipVector.z;


//	if(request.OriginID == 2)
//	{
//		// This will output the ACTUAL POSITION of the requesting cell (for debugging purposes)
//		PhysicsVector actualPosition = getActualPosition(requestingCell);
//		cout << "\nCell " << request.OriginID << "'s ACTUAL POSITION: " << actualPosition.x << ", "
//				<< actualPosition.y << ", " << actualPosition.z;
//	}
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
//	btMatrix3x3 rotationMatrix(transform.getRotation());
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
//	cout << "\n*** Now serving the " << relationshipService.getService() << " service from the environment ***\n\n";

	ros::spinOnce();
}

// Sets the response(neighborhood id vector) based on the requests(IDs) and number of functions.  This is a callback.
bool Environment::setNeighborhoodMessage(NewSimulator::Neighborhood::Request &request, NewSimulator::Neighborhood::Response &response)
{
	string requestingCell = createTargetIdString(request.OriginID);
	PhysicsVector relationshipVector;

	// add an iterator value, otherwise vector will crash the service message
	vector<int>::iterator iter = response.neighborIds.end();
	response.neighborIds.insert(iter,99);

    vector<int> closestNeighbors = findClosestNeighbors(requestingCell, request.OriginID);
    copy(closestNeighbors.begin(),closestNeighbors.end(),back_inserter(response.neighborIds));

	if(response.neighborIds.end() != find(response.neighborIds.begin(), response.neighborIds.end(), 99)){
		response.neighborIds.erase(find(response.neighborIds.begin(), response.neighborIds.end(), 99));
	}

	return true;
}

vector<int> Environment::findClosestNeighbors(const string requestingCell, const int cellID) {
	vector<neighborMagnitudes> closestNeighbors;
	string targetCell = "/sphero/base_link";

	// finds magnitude of all neighbors and adds them to vector
	for(int j = 0; j<numOfRobots; j++)
	{
		neighborMagnitudes temp;
		temp.cellID = j;
		targetCell = createTargetIdString(j);
	    temp.magnitude = getTransform(requestingCell, targetCell).magnitude();

	    if(temp.cellID != cellID) {
	    	closestNeighbors.push_back(temp);
	    }
	}

	// sorts vector by magnitude size
	sort(closestNeighbors.begin(), closestNeighbors.end());

	vector<int> neighbors;
	for(int i = 0; i < closestNeighbors.size(); i++) {
		neighbors.push_back(closestNeighbors.at(i).cellID);
//		cout << "Cell's neighbor: " << closestNeighbors.at(i).cellID << endl;
	}

	return neighbors;
}

// Starts the environment's neighborhood service server
void Environment::startNeighborhoodServiceServer()
{
	int argc = 0;
	char **argv = 0;
	ros::init(argc, argv, "neighborhood_server");

	neighborhoodService = NeighborhoodServerNode.advertiseService("neighborhood", &Environment::setNeighborhoodMessage, this);
//	cout << "\n*** Now serving the " << relationshipService.getService() << " service from the environment ***\n\n";

	ros::spinOnce();
}

string Environment::createTargetIdString(int idNumber)
{
	string targetCell = "/sphero/base_link";
	stringstream targetStringStream;
	targetStringStream << (idNumber);
	string  targetID = targetStringStream.str();
	targetCell.insert(7, targetID);
	return targetCell;
}

