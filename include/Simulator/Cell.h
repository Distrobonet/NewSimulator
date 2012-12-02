
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
const float FLOAT_ZERO_APPROXIMATION = 0.001f;
const float MAX_TRANSLATIONAL_VELOCITY = 0.7f;
const float MAX_ROTATIONAL_VELOCITY = 0.5f;

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
		void moveMultiFunction();
		void moveFunction();

		int getCellID();
		void setCellID(int ID);
		Formation getFormation();
		void setFormation(Formation formation);

		vector<int> updateNeighborhood();
		void createNeighborhood();
		void setLeftNeighbor(const int nbr);
		void setRightNeighbor(const int nbr);
		void establishNeighborhoodCom();

		State getState();
		void setState(State state);
		void updateState();
		void updateState(const NewSimulator::State::Response &incomingState);

		int getCurrentStatus();
		void setCurrentStatus(int newStatus);
		void checkNeighborStatus();

		void calculateDesiredRelationship(int neighborIndex);
		void applySensorError(int neighborIndex);
		bool getCommunicationLostBasedOnError();
		void limitAndScaleVelocities(PhysicsVector &velocities);

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
		void receiveActualRelationshipFromEnvironment(int neighborIndex);


		// Simulator formation subscriber - only used by seed cell to get formation from Simulator
		ros::NodeHandle simulatorFormationNodeHandle;
		ros::Subscriber simulatorFormationSubscriber;
		void receiveFormation(const NewSimulator::FormationMessage::ConstPtr &formationMessage);

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
		bool isMultiFunction;
		void outputCellInfo();	// This is useful for debugging
};

#endif
