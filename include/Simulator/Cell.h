
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
#include "../srv_gen/cpp/include/NewSimulator/CurrentFormation.h"

// State service
#include "../msg_gen/cpp/include/NewSimulator/StateMessage.h"
#include "../srv_gen/cpp/include/NewSimulator/State.h"

// Relationship service
#include "../msg_gen/cpp/include/NewSimulator/RelationshipMessage.h"
#include "../srv_gen/cpp/include/NewSimulator/Relationship.h"

using namespace std;

enum Status{
	WAITING_FOR_FORMATION,
	WAITING_TO_UPDATE,
	UPDATING,
	WAITING_TO_MOVE,
	MOVING
};

class Cell
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
		void updateState(const NewSimulator::State::Response &incomingState);
		int getCurrentStatus();
		void setCurrentStatus(int newStatus);

		ros::NodeHandle stateNode;
		ros::Publisher state_pub;
		geometry_msgs::Twist commandVelocity;
		ros::Publisher cmd_velPub;
		ros::Subscriber leftNeighborStateSubscriber;
		ros::Subscriber rightNeighborStateSubscriber;


		void stateCallback(const NewSimulator::StateMessage & state);
		string generateSubMessage(int cellID);
		string generatePubMessage(int cellID);



		// Formation service client - only used by seed cell
		ros::NodeHandle formationNodeHandle;
		ros::ServiceClient formationClient;
		NewSimulator::CurrentFormation currentFormationService;
		void receiveFormationFromSimulator();




		// State service client
		void receiveNeighborState();
		ros::NodeHandle stateNodeHandle;
		ros::ServiceClient stateClient;
		NewSimulator::State incomingStateService;
		State stateSrv;
		void makeStateClientCall(string neighbor);

		// State service server
		ros::ServiceServer stateService;
		bool setStateMessage(NewSimulator::State::Request & req, NewSimulator::State::Response & res);
		void startStateServiceServer();


	protected:
//		PhysicsVector actualPosition;
//		PhysicsVector desiredPosition;	// todo: These came from Robot.  Are they needed?
		State cellState;
		Formation cellFormation;
		vector<int> neighborhoodList;
		int cellID;
		float x, y, z;		// Not yet sure exactly what these are for
		int currentStatus;

    private:
		void publishState();
};

#endif
