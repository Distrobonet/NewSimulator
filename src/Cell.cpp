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
	currentStatus = WAITING_FOR_FORMATION;

	commandVelocity.linear.x = 0;
	commandVelocity.linear.y = 0;
	commandVelocity.linear.z = 0;

	commandVelocity.angular.x = 0;
	commandVelocity.angular.y = 0;
	commandVelocity.angular.z = 0;

	// For 1-dimensional formations, just set 2 neighbor relationships for each cell
	cellState.actualRelationships.resize(2, PhysicsVector());
	cellState.desiredRelationships.resize(2, PhysicsVector());
}

Cell::~Cell()
{}

// This is where most of the magic happens
void Cell::update()
{

	ros::Rate loop_rate(10);
	setNeighborhood();
	updateCurrentStatus(WAITING_FOR_FORMATION);

	while(ros::ok)
	{
		checkNeighrborStatus();



		// If the cell's ID is the seedID set in Formation, then get the formation ID set by Simulator
		if(cellID == cellFormation.seedID)
		{
			receiveFormationFromSimulator();
		}

		updateCurrentStatus(WAITING_TO_UPDATE);

		// Send requests to Environment for the cell's relationships with its neighbors
		for(uint i = 0; i < getNumberOfNeighbors(); i++)
		{
			// When we do multi-function formations and dynamic neighborhoods, these indices will be changed
			receiveRelationshipFromEnvironment(neighborhoodList[i]);
		}

		receiveNeighborState();
		calculateDesiredPosition();
		moveToDesiredFromActualPosition();
		updateCurrentStatus();

//		publishState();
//		cmd_velPub.publish(commandVelocity);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

vector<Status> Cell::getNeighrborStatus()
{
	for(uint i = i; i < getNumberOfNeighbors(); i++) {

	}
}

bool Cell::figureCurrentStatus(int currentNeighborCellID, vector<Status> neighrborsStatus)
{
		string status = neighrborsStatus.at(currentNeighborCellID);
		switch (status) {
			case WAITING_FOR_FORMATION:
				if(figureCurrentStatus(neighborhoodList.at(currentNeighborCellID + 1), neighrborsStatus)) {

					break;
				}
				else {
					return false;
				}

			case WAITING_TO_UPDATE:
				figureCurrentStatus(currentNeighbor, neighrborsStatus);
				break;

			case UPDATING:
				figureCurrentStatus(currentNeighbor, neighrborsStatus);
				break;

			case WAITING_TO_MOVE:
				figureCurrentStatus(currentNeighbor, neighrborsStatus);
				break;

			case MOVING:
				figureCurrentStatus(currentNeighbor, neighrborsStatus);
				break;

			default:
				break;
		}
}

void Cell::calculateDesiredPosition()
{

}

void Cell::moveToDesiredFromActualPosition()
{
	calculateMovement();
	move();
}

void Cell::updateCurrentStatus()
{
	// Stuff from Ross' simulator to mimic eventually:
	if (currentStatus == 1) // This should be whatever Status means that the cell should figure out its movement
	{
//		commandVelocity.linear.x = getTranslationalVelocity().x;
//		commandVelocity.linear.y = getTranslationalVelocity().y;
//		commandVelocity.angular.z = getAngularVelocity().z;
	}

//		commandVelocity.linear.x = 1;	// moves forward
//		commandVelocity.angular.z = 1;	// moves counter-clockwise
}

void Cell::calculateMovement()
{
	// getActualPosition();
	// getDesiredPosition();
	// getFormationRelativePosition();

	// doMath();
}

void Cell::move()
{

}


// Uses a service client to get the relationship from Environment
void Cell::receiveRelationshipFromEnvironment(int neighborIndex)
{
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	relationshipClient = relationshipNodeHandle.serviceClient<NewSimulator::Relationship>("relationship");

	// Set the request values here
	relationshipService.request.OriginID = cellID;
	relationshipService.request.TargetID = neighborhoodList[neighborIndex];

	if (relationshipClient.call(relationshipService))
	{
		cellState.actualRelationships[neighborIndex].x = relationshipService.response.theRelationship.actual_relationship.x;
		cellState.actualRelationships[neighborIndex].y = relationshipService.response.theRelationship.actual_relationship.y;
		cellState.actualRelationships[neighborIndex].z = relationshipService.response.theRelationship.actual_relationship.z;

		relationshipNodeHandle.shutdown();
		spinner.stop();
		return;
	}

	relationshipNodeHandle.shutdown();
	spinner.stop();
	return;
}

// Uses a service client to get the formation from Simulator
void Cell::receiveFormationFromSimulator()
{
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	formationClient = formationNodeHandle.serviceClient<NewSimulator::CurrentFormation>("formation");

	if (formationClient.call(currentFormationService))
	{
//		cout << "*** Success - Formation ID is " << currentFormationService.response.formation.formation_id << " ***\n";

		cellFormation.setFormationID(currentFormationService.response.formation.formation_id);

		cellFormation.radius = currentFormationService.response.formation.radius;
		cellFormation.seedID = currentFormationService.response.formation.seed_id;

		formationNodeHandle.shutdown();
		spinner.stop();
		return;
	}
	formationNodeHandle.shutdown();
	spinner.stop();
	return;
}

int Cell::getCellID()
{
	return cellID;
}

void Cell::setCellID(int newID)
{
	cellID = newID;
}

Formation Cell::getFormation()
{
	return cellFormation;
}

void Cell::setFormation(Formation formation)
{
	cellFormation = formation;
}

vector<int> Cell::getNeighborhood()
{
	return neighborhoodList;
}

// temp setNeighbohood - HARDCODED for 1-dimensional formations
void Cell::setNeighborhood()
{
	setLeftNeighbor(cellID - 1);
	setRightNeighbor(cellID + 1);
}

// temp setLeftNeighbor - HARDCODED for 1-dimensional formations
void Cell::setLeftNeighbor(const int nbr)
{
	if (cellID == 0)
		neighborhoodList.insert(neighborhoodList.begin() + 0, -1);
	else {
		neighborhoodList.insert(neighborhoodList.begin() + 0, nbr);
//		leftNeighborStateSubscriber = stateNode.subscribe(generateSubMessage(nbr), 1000, &Cell::stateCallback, this);
	}
}

// temp setRightNeighbor - HARDCODED for 1-dimensional formations
void Cell::setRightNeighbor(const int nbr)
{
	// Temp max number of cells (6)
	if (cellID == 6)
		neighborhoodList.insert(neighborhoodList.begin() + 1, -1);
	else
	{
		neighborhoodList.insert(neighborhoodList.begin() + 1, nbr);
//		rightNeighborStateSubscriber = stateNode.subscribe(generateSubMessage(nbr), 1000, &Cell::stateCallback, this);
	}
}

//void Cell::establishNeighborhoodCom()
//{
//	for (int i = 0; i < (int)neighborhoodList.size(); i++)
//	{
//		stateNode.subscribe(generateSubMessage(neighborhoodList.at(i)), 1000, &Cell::stateCallback, this);
//	}
//}

State Cell::getState()
{
	return cellState;
}

void Cell::setState(State state)
{
	cellState = state;
}

// Translates the robot relative to itself based on the parameterized translation vector.
void Cell::translateRelative(float dx , float dy)
{
//	rotateRelative(0);
//    x += dx;
//    y += dy;
}

void Cell::rotateRelative(float theta)
{
//	theta = degreesToRadians(theta);
//	x = x * cos(theta)- y * sin(theta);
//	y = x * sin(theta) + y * cos(theta);
	//z = z;

}

// create correct string for state subscriber
string Cell::generateSubMessage(int cellID)
{
	stringstream ss;					//create a stringstream
	ss << (cellID);						//add number to the stream
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
// Gets the state of the neighbor that is closest to the seed cell
// This should prevent cells from getting conflicting states
void Cell::receiveNeighborState()
{

	stringstream leftSS;					//create a stringstream
	leftSS << (neighborhoodList[0]);		//add number to the stream
	string  leftNeighborID = leftSS.str();

	stringstream rightSS;					//create a stringstream
	rightSS << (neighborhoodList[1]);		//add number to the stream
	string  rightNeighborID = rightSS.str();


	if(cellID > cellFormation.getSeedID())
	{
		makeStateClientCall(leftNeighborID);
	}
	else if(cellID < cellFormation.getSeedID())
	{
		makeStateClientCall(rightNeighborID);
	}
}

void Cell::updateState()
{
  if ((getNumberOfRobots() == 0) || (formation.getSeedID() != ID))
		  return;

  // update actual relationships to neighbors
  Neighbor *currentNeighbor = NULL;
  for (uint i = 0; i < getNumberOfNeighbors(); ++i)
  {
    currentNeighbor = neighborhoodList.at(i);
    if (currentNeighbor == -1) break;

    // change formation if a neighbor has changed formation
    if (currentNeighbor->formation.getFormationID() > formation.getFormationID())
      changeFormation(currentNeighbor->formation, *currentNeighbor);

    currentNeighbor->relActual = getRelationship(currentNeighbor->ID);
  }

  rels = getRelationships();

  // reference the neighbor with the minimum gradient
  // to establish the correct position in formation
  if (getNNbrs() > 0)
  {
    Neighbor *neighborReference = nbrWithMinGradient(formation.getSeedGradient());
    Relationship *nbrRelToMe = relWithID(neighborReference->rels, ID);

    if ((formation.getSeedID() != ID) && (neighborReference != NULL) && (nbrRelToMe != NULL))
    {
      // error (state) is based upon the
      // accumulated error in the formation
      Vector  nbrRelToMeDesired = nbrRelToMe->relDesired;
      nbrRelToMeDesired.rotateRelative(-neighborReference->rotError);
      float theta = scaleDegrees(nbrRelToMe->relActual.angle() - (-neighborReference->relActual).angle());
      rotError = scaleDegrees(theta + neighborReference->rotError);
      transError = nbrRelToMeDesired - nbrRelToMe->relActual + neighborReference->transError;
      transError.rotateRelative(-theta);

      //set the state variable of refID  = ID of the reference nbr.
      refID = neighborReference->ID;
    }
  }
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
bool Cell::setStateMessage(NewSimulator::State::Request &req, NewSimulator::State::Response &res )
{
  	//res.state.formation.heading = cellFormation.heading;
	res.state.frp.x = cellFormation.seedFormationRelativePosition.x;
	res.state.frp.y = cellFormation.seedFormationRelativePosition.y;
  	//res.state.formation.seed_id = cellFormation.seedID;
  	res.state.formation_id = cellFormation.formationID;
  	//res.state.in_position = inPosition;

//	res.state.frp.x = frp.x;
// 	res.state.frp.y = frp.y;

// 	for(uint i = 0; i < rels.size(); i++)
// 	{
//        res.state.actual_relationships[i].id = rels[i].ID;
//        res.state.actual_relationships[i].actual.x = rels[i].relActual.x;
//        res.state.actual_relationships[i].actual.y = rels[i].relActual.y;
//        res.state.desired_relationships[i].desired.x = rels[i].relDesired.x;
//        res.state.desired_relationships[i].desired.y = rels[i].relDesired.y;
// 	}

//	res.state.linear_error.x = transError.x;
//	res.state.linear_error.y = transError.y;
//	res.state.angular_error = rotError;
//	res.state.timestep = tStep;
//	res.state.reference_id = refID;
//	res.state.temperature = temperature;
//	res.state.heat = heat;

	//ROS_INFO("sending back response with state info");
	return true;
}

void Cell::stateCallback(const NewSimulator::StateMessage &incomingState)
{
	//Proof of concept for cell communication
//	cout << "Cell " << cellID << " hears Cell " << incomingState.formation.seed_id << " as is its neighbor" << endl;

//		res.state.formation.radius = incomingState.formation.radius;
//		res.state.formation.heading = incomingState.formation.heading;
//		res.state.formation.seedFrp.x = incomingState.formation.seed_frp.x;
//		res.state.formation.seedFrp.y = incomingState.formation.seed_frp.y;
//		res.state.formation.seedID = incomingState.formation.seed_id;
//		res.state.formation.formationID = incomingState.formation.formation_id;
//		rightNbr.formation.radius = incomingState.formation.radius;
//		rightNbr.formation.heading = incomingState.formation.heading;
//		rightNbr.formation.seedFrp.x = incomingState.formation.seed_frp.x;
//		rightNbr.formation.seedFrp.y = incomingState.formation.seed_frp.y;
//		rightNbr.formation.seedID = incomingState.formation.seed_id;
//		rightNbr.formation.formationID = incomingState.formation.formation_id;

//
//			changeFormation(formations[formation.formationID], rels[0].ID);
//			Vector r = formation.calculateDesiredRelationship(formations[formation.formationID], 1.0f, frp, 0.0f);
//			rels[0].relDesired.x = r.x;
//			rels[0].relDesired.y = r.y;
//			rels[0].relDesired.z = r.z;
//			cout<<r.x<<endl;
//			cout<<r.y<<endl;
//		if((incomingState.in_position == true) && (inPosition == false) && (startMoving != true) && formation.formationID != -1)
//		{
//			startMoving = true;
//			stateChanged = true;
//		}
//	}
//	updateState();
}

void Cell::updateState(const NewSimulator::State::Response &incomingState)
{
//	cellFormation.formationID = incomingState.state.formation_id;
}

//void Cell::publishState()
//{
//	NewSimulator::StateMessage state;
////    state.formation.radius = formation.heading;
////    state.formation.heading = formation.heading;
////    state.formation.seed_frp.x = formation.seedFrp.x;
////    state.formation.seed_frp.y = formation.seedFrp.y;
////    state.formation.seed_id = formation.seedID;
////    state.formation.formation_id = formation.formationID;
////    state.in_position = inPosition;
//
//	//Proof of Concept
////	state.formation.seed_id = cellID;		// statemessage doesn't contain this anymore
//
//    state_pub.publish(state);
////    stateChanged = false;
//}

void Cell::makeStateClientCall(string neighbor)
{
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	//ros::NodeHandle clientNode;
	stateClient = stateNodeHandle.serviceClient<NewSimulator::State>("cell_state_"+neighbor);

	if (stateClient.call(incomingStateService) && incomingStateService.response.state.formation_id != cellFormation.formationID)
	{
	//	cout << "For cell " << cellID << " old formation id getting " << neighbor << "s formation id " << cellFormation.formationID << endl;
		cellFormation.formationID = incomingStateService.response.state.formation_id;
	//	cout << "For cell " << cellID << " new formation id " << cellFormation.formationID << endl << endl;

		stateNodeHandle.shutdown();
		spinner.stop();
		return;
	}

//	ROS_INFO("Shutting down client node for Formation service...");
	stateClient.shutdown();
	spinner.stop();
	return;
}

void Cell::setCurrentStatus(int newStatus)
{
	currentStatus = newStatus;
}

int Cell::getCurrentStatus()
{
	return currentStatus;
}

void Cell::checkNeighborStatus()
{
	// The following comments are what we wrote on the board for this method at 9/5 meeting

	// If neighbor state is (in position)
		// Change current state to (moving)
		// Move (step)
	// Change state to (in position)
}

int Cell::getNumberOfNeighbors()
{
	return neighborhoodList.size();
}
}
