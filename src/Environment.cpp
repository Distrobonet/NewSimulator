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
	//startRelationshipServiceServer();

	numOfRobots = numRobots;
	initEnvironmentSubscribers();
}

// Copy constructor that copies the contents of  the parameterized environment into this environment.
  Environment::Environment(const Environment &env)
{}

Environment::~Environment()
{}

// Everything that the environment continuously does goes here.
void Environment::update(bool doSpin)
{
	ros::NodeHandle rosNode;
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
	}
}

// Set up the environment's vector of cell actual positions as received from Stage (base_pose_ground_truth)
void Environment::initEnvironmentSubscribers()
{
	// Create a dummy robot Velocity to fill the subRobotPoses vector with
	vector<double> tempSubRobotVelocity;
	tempSubRobotVelocity.push_back(0);	// x
	tempSubRobotVelocity.push_back(0);	// y
	tempSubRobotVelocity.push_back(0);	// z, orientation

	// Push numOfRobots robot locations into the subRobotSubscribers vector
	for(int k = 0; k < numOfRobots; k++)
		cellActualPositions.push_back(tempSubRobotVelocity);


	// Sets all the robot subscribers to base_pose_ground_truth
	ros::Subscriber robot0 = environmentNode.subscribe("/robot_0/base_pose_ground_truth", 1000, &Environment::ReceiveOdometry, this);
	cellActualPositionSubscribers.push_back(robot0);
	ros::Subscriber robot1 = environmentNode.subscribe("/robot_1/base_pose_ground_truth", 1000, &Environment::ReceiveOdometry, this);
	cellActualPositionSubscribers.push_back(robot1);
	ros::Subscriber robot2 = environmentNode.subscribe("/robot_2/base_pose_ground_truth", 1000, &Environment::ReceiveOdometry, this);
	cellActualPositionSubscribers.push_back(robot2);
	ros::Subscriber robot3 = environmentNode.subscribe("/robot_3/base_pose_ground_truth", 1000, &Environment::ReceiveOdometry, this);
	cellActualPositionSubscribers.push_back(robot3);
	ros::Subscriber robot4 = environmentNode.subscribe("/robot_4/base_pose_ground_truth", 1000, &Environment::ReceiveOdometry, this);
	cellActualPositionSubscribers.push_back(robot4);
	ros::Subscriber robot5 = environmentNode.subscribe("/robot_5/base_pose_ground_truth", 1000, &Environment::ReceiveOdometry, this);
	cellActualPositionSubscribers.push_back(robot5);
	ros::Subscriber robot6 = environmentNode.subscribe("/robot_6/base_pose_ground_truth", 1000, &Environment::ReceiveOdometry, this);
	cellActualPositionSubscribers.push_back(robot6);
}

// Used for getting the actual positions of the cells
void Environment::ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& odometryMessage)
{
	// the quaternion is used to find the yaw, for calculating z (theta)
	btQuaternion quaternion(odometryMessage->pose.pose.orientation.x,
			odometryMessage->pose.pose.orientation.y,
			odometryMessage->pose.pose.orientation.z,
			odometryMessage->pose.pose.orientation.w);

	// Set the stage ID for the cell whose actual position we're checking
	string ID = odometryMessage->header.frame_id.substr(7,1);
	int IDNumber = atoi(ID.c_str());

	cellActualPositions.at(IDNumber).at(0) = -odometryMessage->pose.pose.position.y;						// x
	cellActualPositions.at(IDNumber).at(1) = odometryMessage->pose.pose.position.x;							// y
	cellActualPositions.at(IDNumber).at(2) = angles::normalize_angle(tf::getYaw(quaternion) + M_PI / 2.0l);	// z, theta

	ros::spinOnce();
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
bool Environment::setRelationshipMessage(NewSimulator::Relationship::Request &request, NewSimulator::Relationship::Response &response)
{
//	cout << "\nsetRelationshipMessage has been called. Origin ID = " << request.OriginID << "\n\n";

//	temp.rotateRelative(-fromCell->getHeading());

	// Target - Origin
//	PhysicsVector tempVector;
//	tempVector.x = cellActualPositions[request.TargetID][0] - cellActualPositions[request.OriginID][0];
//	tempVector.y = cellActualPositions[request.TargetID][1] - cellActualPositions[request.OriginID][1];

//	tempVector.rotateRelative(-(subRobotPoses[req.OriginID][2]));
//	tempVector.rotateRelative(angles::to_degrees(-(cellActualPositions[request.OriginID][2])));

//	//set(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta), z);
//	tempVector.x = tempVector.x * cos(tempVector.z) - tempVector.y * sin(tempVector.z);
//	tempVector.y = tempVector.x * sin(tempVector.z) + tempVector.y * cos(tempVector.z);

//	response.theRelationship.actual_relationship.x = tempVector.x;
//	response.theRelationship.actual_relationship.y = tempVector.y;


	// Test values to make sure relationship service is working
	response.theRelationship.actual_relationship.x = 1;
	response.theRelationship.actual_relationship.y = 2;

	return true;
}


// Starts the environment's relationship service server
void Environment::startRelationshipServiceServer()
{
	int argc = 0;
	char **argv = 0;
	ros::init(argc, argv, "relationship_server");

	relationshipService = RelationshipServerNode.advertiseService("relationship", &Environment::setRelationshipMessage, this);
	cout << "\n*** Now serving the " << relationshipService.getService() << " service from the environment ***\n\n";

	ros::spinOnce();
}

