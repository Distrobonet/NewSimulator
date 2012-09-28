//
// Filename:        "Environment.h"
//
// Description:     This class describes a cell environment.
//

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

#include <queue>
#include <vector>
#include "Simulator/PhysicsVector.h"

#include "../srv_gen/cpp/include/NewSimulator/Relationship.h"

using namespace std;


// Describes an environment through which robots figure out their actual and desired positioning.
class Environment
{
    public:
		Environment();
        Environment(int numRobots);
        Environment(const Environment &e);

        virtual ~Environment();

        void initEnvironmentSubscribers();
        void update(bool doSpin);


        // Functions for the subscribers for all robots
        string generateSubMessage(int cellID);
		void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& odometryMessage);


        PhysicsVector getRelationship(const int toID, const int fromID);


        // Relationship service server
        ros::NodeHandle RelationshipServerNode;
		ros::ServiceServer relationshipService;
		bool setRelationshipMessage(NewSimulator::Relationship::Request &request, NewSimulator::Relationship::Response &response);
		void startRelationshipServiceServer();

		ros::NodeHandle environmentBasePoseGroundTruthNode;
		vector<ros::Subscriber> cellActualPositionSubscribers;
		vector< vector<double> > cellActualPositions;
		void getTransform(string tf_dst_name, string tf_src_name, ros::Time t, double wait_for);


    protected:
        int                numOfRobots;
        tf::TransformListener spheroTransformListener;		// Listens to sphero positions in rviz
        tf::StampedTransform transform;						// Stores the sphero position info

};  // Environment

#endif
