#include <iostream>
#include <Simulator/Cell.h>
#include <Simulator/Formation.h>
#include <Simulator/State.h>
#include <vector>

using namespace std;

Cell::Cell(const int ID)
{
	cellID = ID;
	cellState = State();
	cellFormation = Formation();
}

Cell::~Cell()
{}

// This is where most of the magic happens
void Cell::update()
{
	getFormation();

//	cmd_vel.linear.x = getTransVel().x;
//	cmd_vel.linear.y = getTransVel().y;
//	cmd_vel.angular.z = getAngVel().z;

}

int Cell::getCellID() {
	return cellID;
}

void Cell::setCellID(int newID) {
	cellID = newID;
}

Formation Cell::getFormation() {
	return cellFormation;
}

void Cell::setFormation(Formation formation) {
	cellFormation = formation;
}

vector<int> Cell::getNeighborhood() {
	return neighborhoodList;
}
void Cell::setNeighborhood(vector<int> neighborhood) {
	neighborhoodList = neighborhood;
}

State Cell::getState() {
 return cellState;
}

void Cell::setState(State state) {
 cellState = state;
}

// Translates the robot relative to itself based on the parameterized translation vector.
void Cell::translateRelative(float dx , float dy)
{
//	  vector.rotateRelative(getHeading());
//    x += vector.x;
//    y += vector.y;
}

void Cell::rotateRelative(float theta)
{
	theta = degreesToRadians(theta);
	x = x * cos(theta)- y * sin(theta);
	y = x * sin(theta) + y * cos(theta);
	//z = z;

}

// create correct string for state subscriber
string Cell::generateSubMessage(int cellID)
{
	stringstream ss;//create a stringstream
	ss << (cellID);//add number to the stream
	string  nbrID = ss.str();
	string subString = "/robot_/state";

	subString.insert(7, nbrID);
	return subString;
}


string Cell::generatePubMessage(int cellID)
{
	stringstream ss;//create a stringstream
	ss << (cellID);//add number to the stream
	string  nbrID = ss.str();
	string subString = "/robot_/cmd_vel";

	subString.insert(7, nbrID);
	return subString;
}

// Get a neighbor's state from the State service
bool Cell::getNeighborState()
{
	if(cellID == 0)
		return true;
	if(cellID % 2 == 1)
	{
		cellFormation.formationID = neighborhoodList[0];
		return true;
	}
	else if(cellID % 2 == 0)
	{
		cellFormation.formationID = neighborhoodList[1];
		return true;

	}
	else
		return false;

}



// Starts the cell's state service server
void Cell::startStateServiceServer()
{
	ros::NodeHandle StateServerNode;
	string name = "cell_state_";
	name = name + boost::lexical_cast<std::string>(cellID);	// add the index to the name string

	stateService = StateServerNode.advertiseService(name, &Cell::setStateMessage, this);
	//cout << "Now serving the " << stateService.getService() << " service!\n";

	ros::spinOnce();

	//StateServerNode.shutdown();
}


// Sets the state message to this state's info.  This is the callback for the state service.
bool Cell::setStateMessage(NewSimulator::State::Request  &req, NewSimulator::State::Response &res )
{
  	res.state.formation.radius = cellFormation.radius;
  	//res.state.formation.heading = cellFormation.heading;
  	res.state.formation.seed_frp.x = cellFormation.seedFormationRelativePosition.x;
  	res.state.formation.seed_frp.y = cellFormation.seedFormationRelativePosition.y;
  	//res.state.formation.seed_id = cellFormation.seedID;
  	res.state.formation.formation_id = cellFormation.formationID;
  	//res.state.in_position = inPosition;

//	res.state.frp.x = frp.x;
// 	res.state.frp.y = frp.y;
//
// 	for(uint i = 0; i < rels.size(); i++)
// 	{
//        res.state.actual_relationships[i].id = rels[i].ID;
//        res.state.actual_relationships[i].actual.x = rels[i].relActual.x;
//        res.state.actual_relationships[i].actual.y = rels[i].relActual.y;
//        res.state.desired_relationships[i].desired.x = rels[i].relDesired.x;
//        res.state.desired_relationships[i].desired.y = rels[i].relDesired.y;
// 	}
//
//	res.state.linear_error.x = transError.x;
//	res.state.linear_error.y = transError.y;
//	res.state.angular_error = rotError;
//	res.state.timestep = tStep;
//	res.state.reference_id = refID;
//	res.state.temperature = temperature;
//	res.state.heat = heat;

	ROS_INFO("sending back response with state info");
	return true;
}





