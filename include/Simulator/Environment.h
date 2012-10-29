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
#include "../srv_gen/cpp/include/NewSimulator/Neighborhood.h"

using namespace std;

struct neighborMagnitudes {
	public:
	int cellID;
	int magnitude;

	bool operator< (neighborMagnitudes const& rhs) const {
        return magnitude < rhs.magnitude;
	}
};

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

        // Neighborhood service server
        ros::NodeHandle NeighborhoodServerNode;
		ros::ServiceServer neighborhoodService;
		bool setNeighborhoodMessage(NewSimulator::Neighborhood::Request &request, NewSimulator::Neighborhood::Response &response);
		vector<int> findClosestNeighbors(const int numberOfNeighbors, const string requestingCell, const int cellID);
		bool smallestMagnitudeSorter(neighborMagnitudes const& lhs, neighborMagnitudes const& rhs);
		void startNeighborhoodServiceServer();

		PhysicsVector getTransform(string tfOriginName, string tfTargetName);
		PhysicsVector getActualPosition(string tfOriginName);

		string createTargetIdString(int idNumber);


    protected:
        int numOfRobots;
        tf::TransformListener spheroTransformListener;		// Listens to sphero positions in rviz

};  // Environment

#endif
