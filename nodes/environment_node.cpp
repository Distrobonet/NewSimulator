
#include<Simulator/Environment.h>

int main(int argc, char **argv)
{
	// Initialize ROS stuff
	ros::init(argc, argv, "environment");
	ros::NodeHandle base_pose;

	// Initialize Environment
	int numOfRobots = atoi(argv[1]);
	Environment environmentNode(numOfRobots);

	// Relationship service server
	environmentNode.startActualRelationshipServiceServer();

	environmentNode.update(argv[2]);

	return 0;
}
