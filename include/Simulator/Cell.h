
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

// Formation publisher/subscriber
#include "../msg_gen/cpp/include/NewSimulator/FormationMessage.h"

// State service
#include "../srv_gen/cpp/include/NewSimulator/State.h"

// Relationship service
#include "../srv_gen/cpp/include/NewSimulator/Relationship.h"

using namespace std;

enum Status{
	// Ross' simulator just uses ACTIVE, INACTIVE, and DONE, which is not like the thesis
	WAITING_FOR_FORMATION,
	WAITING_TO_UPDATE,
	UPDATING,
	WAITING_TO_MOVE,
	MOVING
};

const int NO_NEIGHBOR = -1;

class Cell
{
	public:
		Cell(const int cellId);
		virtual ~Cell();

		void update();
		void updateCurrentStatus();
		int getNumberOfNeighbors();

		bool calculateMovement();
		void move(int neighborIndex);

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
		void updateState();
		void updateState(const NewSimulator::State::Response &incomingState);
		int getCurrentStatus();
		void setCurrentStatus(int newStatus);
		void checkNeighborStatus();
		void calculateDesiredRelationship(int neighborIndex);
		void applySensorAndCommError(int neighborIndex);

		ros::NodeHandle stateNode;
		ros::Publisher state_pub;
		geometry_msgs::Twist commandVelocity;
		ros::Publisher cmd_velPub;
		ros::Subscriber leftNeighborStateSubscriber;
		ros::Subscriber rightNeighborStateSubscriber;


		void stateCallback(const NewSimulator::StateMessage & state);
		string generateStateSubMessage(int cellID);
		string generateCommandVelocityPubMessage(int cellID);


		// Relationship service client
		ros::NodeHandle relationshipNodeHandle;
		ros::ServiceClient relationshipClient;
		NewSimulator::Relationship relationshipService;
		void receiveRelationshipFromEnvironment(int neighborIndex);


		// Simulator formation subscriber - only used by seed cell to get formation from Simulator
		ros::NodeHandle simulatorFormationNodeHandle;
		ros::Subscriber simulatorFormationSubscriber;
		void receiveFormationFromSimulator(const NewSimulator::FormationMessage::ConstPtr &formationMessage);

		// Neighbor formation change subscriber
		ros::NodeHandle formationChangeSubscriberNode;
		ros::Subscriber formationChangeSubscriber;
		void receiveFormationFromNeighbor(const NewSimulator::FormationMessage::ConstPtr &formationMessage);
		NewSimulator::FormationMessage createFormationChangeMessage();

		// Formation change publisher to neighbors
		string generateFormationPubName(int cellID);
		ros::NodeHandle formationChangePublisherNode;
		ros::Publisher formationChangePublisher;



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
		State cellState;
		Formation cellFormation;
		int formationCount;
		vector<int> neighborhoodList;
		int cellID;
		int currentStatus;

    private:
		bool isFormationChanged;
		void publishState();
		void outputCellInfo();	// This is useful for debugging
};

#endif
