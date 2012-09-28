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
	ros::Rate loop_rate(1);

	while(ros::ok())
	{
	    loop_rate.sleep();
		ros::spinOnce();
	}
}

// Set up the environment's vector of cell actual positions as received from Stage (base_pose_ground_truth)
void Environment::initEnvironmentSubscribers()
{
//	// Create a dummy robot Velocity to fill the subRobotPoses vector with
//	vector<double> tempSubRobotVelocity;
//	tempSubRobotVelocity.push_back(0);	// x
//	tempSubRobotVelocity.push_back(0);	// y
//	tempSubRobotVelocity.push_back(0);	// z, orientation
//
//	// Push numOfRobots robot locations into the subRobotSubscribers vector
//	for(int k = 0; k < numOfRobots; k++)
//		cellActualPositions.push_back(tempSubRobotVelocity);
//
//
//	// Sets all the robot subscribers to base_pose_ground_truth
//	ros::Subscriber robot0 = environmentBasePoseGroundTruthNode.subscribe("/sphero0/base_link", 1000, &Environment::ReceiveOdometry, this);
//	cellActualPositionSubscribers.push_back(robot0);
//	ros::Subscriber robot1 = environmentBasePoseGroundTruthNode.subscribe("/sphero1/base_link", 1000, &Environment::ReceiveOdometry, this);
//	cellActualPositionSubscribers.push_back(robot1);
//	ros::Subscriber robot2 = environmentBasePoseGroundTruthNode.subscribe("/sphero2/base_link", 1000, &Environment::ReceiveOdometry, this);
//	cellActualPositionSubscribers.push_back(robot2);
//	ros::Subscriber robot3 = environmentBasePoseGroundTruthNode.subscribe("/sphero3/base_link", 1000, &Environment::ReceiveOdometry, this);
//	cellActualPositionSubscribers.push_back(robot3);
//	ros::Subscriber robot4 = environmentBasePoseGroundTruthNode.subscribe("/sphero4/base_link", 1000, &Environment::ReceiveOdometry, this);
//	cellActualPositionSubscribers.push_back(robot4);
//	ros::Subscriber robot5 = environmentBasePoseGroundTruthNode.subscribe("/sphero5/base_link", 1000, &Environment::ReceiveOdometry, this);
//	cellActualPositionSubscribers.push_back(robot5);
//	ros::Subscriber robot6 = environmentBasePoseGroundTruthNode.subscribe("/sphero6/base_link", 1000, &Environment::ReceiveOdometry, this);
//	cellActualPositionSubscribers.push_back(robot6);
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




	// Get the relationship between the origin cell and the target cell from rviz

	string requestingCell = "/sphero/base_link";
	string targetCell = "/sphero/base_link";

	stringstream originStringStream;					//create a stringstream
	originStringStream << (request.OriginID);			//add number to the stream
	string  originID = originStringStream.str();
	requestingCell.insert(7, originID);


	stringstream targetStringStream;
	targetStringStream << (request.TargetID);
	string targetID = targetStringStream.str();
	targetCell.insert(7, targetID);

//	cout << "\n*** Requesting cell: " << requestingCell << " - Target cell: " << targetCell << endl;
	try	{
		// Get the transform from one frame to a second and store it in a StampedTransform object
		spheroTransformListener.lookupTransform(requestingCell, targetCell, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
	}

	// Set the values to the response
	response.theRelationship.actual_relationship.x = transform.getOrigin().x();
	response.theRelationship.actual_relationship.y = transform.getOrigin().y();
	response.theRelationship.actual_relationship.z = transform.getRotation().getAngle();

	return true;
}

// Gets the relative pose (x, y, theta) of target cell to origin cell.  Names should be in the format "/sphero1/base_link"
void Environment::getTransform(string tfTargetName, string tfOriginName, ros::Time time = ros::Time(0), double waitTime = 0.0)
{
	if (tfTargetName.empty() || tfOriginName.empty())
		return;

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
		return;
	}

	// get position (x, y, z)
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = transform.getOrigin().z();


	// get orientation (roll, pitch, yaw)
	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	btMatrix3x3 const rotationMatrix(transform.getRotation());
	rotationMatrix.getEulerYPR(yaw, pitch, roll);
	ROS_INFO("(x, y, th) = (%.2f, %.2f, %.2f)", x, y, yaw);

	return;
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

