
#ifndef CELL_H_
#define CELL_H_

#include "Simulator/State.h"
#include "Simulator/Formation.h"
#include <vector>
#include "Simulator/utils.h"

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

// Formation service
#include "../msg_gen/cpp/include/NewSimulator/FormationMessage.h"
#include "../srv_gen/cpp/include/NewSimulator/CurrentFormation.h"

// State service
#include "../msg_gen/cpp/include/NewSimulator/StateMessage.h"
#include "../srv_gen/cpp/include/NewSimulator/State.h"

// Relationship service
#include "../msg_gen/cpp/include/NewSimulator/RelationshipMessage.h"
#include "../srv_gen/cpp/include/NewSimulator/Relationship.h"

using namespace std;

class Cell//: public Formation, public State
{
	public:
		Cell(const int cellId);
		virtual ~Cell();

		void update();
		int getCellID();
		void setCellID(int ID);
		Formation getFormation();
		void setFormation(Formation formation);
		vector<int> getNeighborhood();
		void setNeighborhood();
		void setLeftNeighbor(const int nbr);
		void setRightNeighbor(const int nbr);
		void establishNeighborhoodCom();
		State getState();
		void setState(State state);
		void translateRelative(float dx = 0.0f, float dy = 0.0f);
		void rotateRelative(float theta);

		ros::NodeHandle stateNode;
		ros::Publisher state_pub;
		geometry_msgs::Twist commandVelocity;
		ros::Publisher cmd_velPub;
		ros::Subscriber leftNeighborStateSubscriber;
		ros::Subscriber rightNeighborStateSubscriber;




		void stateCallback(const NewSimulator::StateMessage & state);
		string generateSubMessage(int cellID);
		string generatePubMessage(int cellID);

		// State service client
		bool getNeighborState();
		ros::ServiceClient stateClient;
		State stateSrv;

		// State service server
		ros::ServiceServer stateService;
		bool setStateMessage(NewSimulator::State::Request & req, NewSimulator::State::Response & res);
		void startStateServiceServer();


	protected:
		State cellState;
		Formation cellFormation;
		vector<int> neighborhoodList;
		int cellID;
		float x, y, z;		// Not yet sure exactly what these are for
};

#endif
