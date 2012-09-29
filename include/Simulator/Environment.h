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
		bool setActualRelationshipMessage(NewSimulator::Relationship::Request &request, NewSimulator::Relationship::Response &response);
		void startActualRelationshipServiceServer();

		PhysicsVector getTransform(string tfOriginName, string tfTargetName);
		PhysicsVector getActualPosition(string tfOriginName);


    protected:
        int numOfRobots;
        tf::TransformListener spheroTransformListener;		// Listens to sphero positions in rviz

};  // Environment

#endif
