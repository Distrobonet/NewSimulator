//
// Filename:        "Environment.h"
//
// Description:     This class describes a robot cell environment.
//

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#define SUBSCRIBER 0
#define ROBOT_LABEL 1

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

#include <queue>
#include <vector>
#include "Simulator/PhysicsVector.h"

using namespace std;

#include "../msg_gen/cpp/include/NewSimulator/RelationshipMessage.h"
#include "../srv_gen/cpp/include/NewSimulator/Relationship.h"



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
		void callBackRobot(const nav_msgs::Odometry::ConstPtr& odom);

		nav_msgs::Odometry odomMsg;
		vector<ros::Subscriber> subRobotSubscribers;
		vector< vector<double> > cellActualPositions;

        PhysicsVector getRelationship(const int toID, const int fromID);


        // Relationship service server
		ros::ServiceServer relationshipService;
		bool setRelationshipMessage(NewSimulator::Relationship::Request &request, NewSimulator::Relationship::Response &response);
		void startRelationshipServiceServer();

    protected:
        //Formation          formation;
        int                numOfRobots;

};  // Environment

#endif
