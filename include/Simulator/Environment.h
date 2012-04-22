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
#include <Simulator/Cell.h>
#include <Simulator/Object.h>

using namespace std;

#define VERBOSE (0)

#include "../msg_gen/cpp/include/Simulator/RelationshipMessage.h"
#include "../srv_gen/cpp/include/Simulator/Relationship.h"



// describes a robot cell environment
class Environment
{
    public:

        // <public data members>
        double robotX;
		double robotY;
		double robotTheta;
		nav_msgs::Odometry odomMsg;
		vector<ros::Subscriber> subRobots;
		vector< vector<double> > subRobotVels;

        // <constructors>
		Environment();
        Environment(int numRobots);
        Environment(const Environment &e);

        // <destructors>
        virtual ~Environment();

        void initOverloardSubscribers(Environment *e);
        void update(bool doSpin);


        //functions for the subscribers for all robots
        string generateSubMessage(int cellID);
		void callBackRobot(const nav_msgs::Odometry::ConstPtr& odom);

		float getDistanceTo(const int fromID, const int toID) const;
		float getAngleTo(const int fromID, const int toID) const;

        // <public utility functions>
        Vector  getRelationship(const int toID, const int fromID);
        float   getDistanceTo(const int id)   const;
        float   getAngleTo(const int id)      const;


        // Relationship service server
		ros::ServiceServer relationshipService;
		bool setRelationshipMessage(Simulator::Relationship::Request  &req, Simulator::Relationship::Response &res );
        void startRelationshipServiceServer();

    protected:

        // <protected data members>
        vector<Cell *>     cells;
		vector<Object *>   objects;
        queue<Packet>      msgQueue;
        Cell              *newestCell;
        Formation          formation;
        int                numOfRobots;

};  // Environment

#endif
