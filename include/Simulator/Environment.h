//
// Filename:        "Environment.h"
//
// Description:     This class describes a robot cell environment.
//

// preprocessor directives
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

using namespace std;

//#include "../msg_gen/cpp/include/NewSimulator/RelationshipMessage.h"
//#include "../srv_gen/cpp/include/NewSimulator/Relationship.h"



// Describes a robot cell environment
class Environment
{
    public:

        // <public data members>
        double robotX;
		double robotY;
		double robotTheta;
		nav_msgs::Odometry odomMsg;
		vector<ros::Subscriber> subRobots;
		vector< vector<double> > subRobotPoses;

        // <constructors>
		Environment();
        Environment(int numRobots);
        Environment(const Environment &e);

        // <destructors>
        virtual ~Environment();

        void initOverlordSubscribers();
        void update(bool doSpin);


        // Functions for the subscribers for all robots
        string generateSubMessage(int cellID);
		void callBackRobot(const nav_msgs::Odometry::ConstPtr& odom);

        // <public utility functions>
        //Vector  getRelationship(const int toID, const int fromID);


        // Relationship service server
//		ros::ServiceServer relationshipService;
//		bool setRelationshipMessage(NewSimulator::Relationship::Request  &req, NewSimulator::Relationship::Response &res );
//      void startRelationshipServiceServer();

    protected:

        // <protected data members>
        //Formation          formation;
        int                numOfRobots;

};  // Environment

#endif
