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
	ros::NodeHandle environmentNode;

	// Create a dummy robot Velocity to fill the subRobotPoses vector with
	vector<double> tempSubRobotVelocity;
	tempSubRobotVelocity.push_back(0);	// x
	tempSubRobotVelocity.push_back(0);	// y
	tempSubRobotVelocity.push_back(0);	// z, orientation

	// Push numOfRobots robot locations into the subRobotSubscribers vector
	for(int k = 0; k < numOfRobots; k++)
		cellActualPositions.push_back(tempSubRobotVelocity);


	// Sets all the robot subscribers to base_pose_ground_truth
	ros::Subscriber subRobot0 = environmentNode.subscribe("/robot_0/base_pose_ground_truth", 1000, &Environment::callBackRobot, this);
	subRobotSubscribers.push_back(subRobot0);
	ros::Subscriber subRobot1 = environmentNode.subscribe("/robot_1/base_pose_ground_truth", 1000, &Environment::callBackRobot, this);
	subRobotSubscribers.push_back(subRobot1);
	ros::Subscriber subRobot2 = environmentNode.subscribe("/robot_2/base_pose_ground_truth", 1000, &Environment::callBackRobot, this);
	subRobotSubscribers.push_back(subRobot2);
	ros::Subscriber subRobot3 = environmentNode.subscribe("/robot_3/base_pose_ground_truth", 1000, &Environment::callBackRobot, this);
	subRobotSubscribers.push_back(subRobot3);
	ros::Subscriber subRobot4 = environmentNode.subscribe("/robot_4/base_pose_ground_truth", 1000, &Environment::callBackRobot, this);
	subRobotSubscribers.push_back(subRobot4);
	ros::Subscriber subRobot5 = environmentNode.subscribe("/robot_5/base_pose_ground_truth", 1000, &Environment::callBackRobot, this);
	subRobotSubscribers.push_back(subRobot5);
	ros::Subscriber subRobot6 = environmentNode.subscribe("/robot_6/base_pose_ground_truth", 1000, &Environment::callBackRobot, this);
	subRobotSubscribers.push_back(subRobot6);
}

void Environment::callBackRobot(const nav_msgs::Odometry::ConstPtr& odom)
{
	btScalar yaw = 0.0l;
	btScalar roll = 0.0l;
	btScalar pitch = 0.0l;

	double currentY = odom-> pose.pose.position.x;
	double currentX = -odom-> pose.pose.position.y;

//	btQuaternion q(odom->pose.pose.orientation.x,
//				 odom->pose.pose.orientation.y,
//				 odom->pose.pose.orientation.z,
//				 odom->pose.pose.orientation.w);
//	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	double currentTheta = angles::normalize_angle(yaw + M_PI / 2.0l);
	string ID = odom->header.frame_id.substr(7,1);
	int IDNumber = atoi(ID.c_str());

	cellActualPositions.at(IDNumber).at(0) = currentX;
	cellActualPositions.at(IDNumber).at(1) = currentY;
	cellActualPositions.at(IDNumber).at(2) = currentTheta;

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
	//cout << "\nsetRelationshipMessage has been called\n";
//	temp.rotateRelative(-fromCell->getHeading());

	//target - origin
	PhysicsVector tempVector;
	tempVector.x = cellActualPositions[request.TargetID][0] - cellActualPositions[request.OriginID][0];
	tempVector.y = cellActualPositions[request.TargetID][1] - cellActualPositions[request.OriginID][1];

//	tempVector.rotateRelative(-(subRobotPoses[req.OriginID][2]));
	tempVector.rotateRelative(angles::to_degrees(-(cellActualPositions[request.OriginID][2])));

//	//set(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta), z);
//	tempVector.x = tempVector.x * cos(tempVector.z) - tempVector.y * sin(tempVector.z);
//	tempVector.y = tempVector.x * sin(tempVector.z) + tempVector.y * cos(tempVector.z);

	response.theRelationship.actual_relationship.x = tempVector.x;
	response.theRelationship.actual_relationship.y = tempVector.y;
	return true;
}


// Starts the environment's relationship service server
void Environment::startRelationshipServiceServer()
{
	int argc = 0;
	char **argv = 0;
	ros::init(argc, argv, "relationship_server");

	ros::NodeHandle RelationshipServerNode;

	relationshipService = RelationshipServerNode.advertiseService("relationship", &Environment::setRelationshipMessage, this);
	cout << "Now serving the " << relationshipService.getService() << " service from the environment\n";

	ros::spinOnce();

	//RelationshipServerNode.shutdown();
}

