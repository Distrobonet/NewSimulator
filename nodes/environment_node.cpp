#include<Simulator/Environment.h>

int main(int argc, char **argv)
{
	// initialize ROS stuff
	ros::init(argc, argv, "environment");
	ros::NodeHandle base_pose;

	// Initialize Environment
	int numOfRobots = atoi(argv[1]);
	Environment environmentNode(numOfRobots);

	// Relationship service server
	environmentNode.startRelationshipServiceServer();

	environmentNode.update(argv[2]);

	return 0;
}


//void environmentCallBack(const nav_msgs::OdometryConstPtr &odom)
//{
//  g_by = odom->pose.pose.position.x;
//  g_bx = -odom->pose.pose.position.y;
//
//  double roll = 0.0l;
//  double pitch = 0.0l;
//  double yaw = 0.0l;
//  btQuaternion q(odom->pose.pose.orientation.x,
//                 odom->pose.pose.orientation.y,
//                 odom->pose.pose.orientation.z,
//                 odom->pose.pose.orientation.w);
//  btMatrix3x3(q).getRPY(roll, pitch, yaw);
//  g_bth = angles::normalize_angle(yaw + M_PI / 2.0l);
//}
