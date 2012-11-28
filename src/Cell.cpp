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
	formationCount = 0;
	isFormationChanged = false;

	commandVelocity.linear.x = 0;
	commandVelocity.linear.y = 0;
	commandVelocity.linear.z = 0;

	commandVelocity.angular.x = 0;
	commandVelocity.angular.y = 0;
	commandVelocity.angular.z = 0;

	updateNeighborhood();

	// Set the seed cell to subscribe to the Simulator's formation messages
	if(cellID == cellFormation.getSeedID())
	{
		simulatorFormationSubscriber = simulatorFormationNodeHandle.subscribe("seedFormationMessage", 100, &Cell::receiveFormation, this);
	}
}

Cell::~Cell()
{
	simulatorFormationNodeHandle.shutdown();
	relationshipNodeHandle.shutdown();
	stateNodeHandle.shutdown();
	stateService.shutdown();
	stateClient.shutdown();
	simulatorFormationSubscriber.shutdown();
	relationshipClient.shutdown();
	stateNode.shutdown();
	state_pub.shutdown();
	cmd_velPub.shutdown();
	leftNeighborStateSubscriber.shutdown();
	rightNeighborStateSubscriber.shutdown();
}

// This is where most of the magic happens
void Cell::update()
{
	ros::Rate loop_rate(10);// This should be 10 in production code

	while(ros::ok)
	{
		updateNeighborhood();

		// If a neighbor published a message to this cell that the formation changed
		if(isFormationChanged)
		{
			// Publish the formation change to this cell's non-reference neighbor (cell farthest from seed)
			formationChangePublisher.publish(createFormationChangeMessage());
			isFormationChanged = false;
			//cout << "\n**** " << "cell" << cellID << "'s Formation ID: " << cellFormation.formationID << " - Formation count: " << formationCount << " ****\n";
		}

		//cout << "\n**** " << "cell" << cellID << "'s Formation ID: " << cellFormation.formationID << " - Formation count: " << formationCount << " ****\n";
		receiveNeighborState();

//		if(isMultiFunction)
			moveFunction();
//		else
//			moveSingleFunction();


		updateCurrentStatus();

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void Cell::moveFunction() {
	for(uint i = 0; i < neighborhoodList.size(); i++)
	{
		int neighbor = neighborhoodList.at(i);
		if(neighbor != NO_NEIGHBOR)
		{
			receiveActualRelationshipFromEnvironment(neighbor);
			applySensorError(neighbor);
			calculateDesiredRelationship(neighbor);
			move(neighbor);
		}
	}
}

//void Cell::moveSingleFunction() {
//	// Send request to Environment for the cell's relationship with its reference neighbor, calculate the desired relationship, and do the movement.
//	if(cellID > cellFormation.getSeedID() && updateNeighborhood()[0] != NO_NEIGHBOR)
//	{
//		// This cell is to the right of the seed.  Get relationship to our left neighbor.
//		receiveActualRelationshipFromEnvironment(0);
//		applySensorError(0);
//		calculateDesiredRelationship(0);
//		move(0); 		// Move relative to this cell's left neighbor
//	}
//	if(cellID < cellFormation.getSeedID() && updateNeighborhood()[1] != NO_NEIGHBOR)
//	{
//		// This cell is to the left of the seed.  Get relationship to our right neighbor.
//		receiveActualRelationshipFromEnvironment(1);
//		applySensorError(1);
//		calculateDesiredRelationship(1);
//		move(1); 		// Move relative to this cell's right neighbor
//	}
//}

void Cell::updateCurrentStatus() {

	int neighborhoodStatuses = 0;
	for(int i = 0; i < getNumberOfNeighbors(); i++) {
		neighborhoodStatuses *= neighborhoodList.at(i);
	}

	switch (currentStatus) {
		case WAITING_FOR_FORMATION:
			if(cellFormation.isValid()) {
				currentStatus = WAITING_TO_UPDATE;
				break;
			}
			break;

		case WAITING_TO_UPDATE:
			if((neighborhoodStatuses % 3 != 0) ||
			   (neighborhoodStatuses % 11 != 0) ||
			   (neighborhoodStatuses % 13 != 0)) {
				currentStatus = UPDATING;
				break;
			}
			break;

		case UPDATING:
//			if(calculateMovement()) {
				currentStatus = WAITING_TO_UPDATE;
				break;
//			}
			break;

		case WAITING_TO_MOVE:
			if(neighborhoodStatuses % 7 != 0) {
				currentStatus = MOVING;
				break;
			}
			break;

		case MOVING:
//			if(move()) {
				currentStatus = WAITING_TO_UPDATE;
				break;
//			}
			break;

		default:
			break;
	}
}

// Movement =  desired relationship - actual relationship.  Movement is relative to the parameterized neighbor
void Cell::move(int neighborIndex)
{
	// At this point, we should know the cell's actual and desired relationship to its reference neighbor.

	// If a formation hasn't been set, don't move
	// if first formationID is nothing, the rest will be nothing
	if(cellFormation.formationIDs.at(0) == NO_FUNCTION_FORMATION_ID || getCommunicationLostBasedOnError())
		return;

	PhysicsVector movementPhysicsVector;

	movementPhysicsVector.x = cellState.desiredRelationships[neighborIndex].x - cellState.actualRelationships[neighborIndex].x;
	movementPhysicsVector.y = cellState.desiredRelationships[neighborIndex].y - cellState.actualRelationships[neighborIndex].y;
	movementPhysicsVector.z = cellState.desiredRelationships[neighborIndex].z - cellState.actualRelationships[neighborIndex].z;

	// Limit (and scale) the translational and rotational velocities so that the cells move realistically.
	limitAndScaleVelocities(movementPhysicsVector);

	// Set the velocities and publish to the cells.
	commandVelocity.linear.x = movementPhysicsVector.x;
	commandVelocity.linear.y = movementPhysicsVector.y;
	commandVelocity.angular.z = movementPhysicsVector.z;
	cmd_velPub.publish(commandVelocity);

	return;
}

void Cell::limitAndScaleVelocities(PhysicsVector &velocities)
{
	// Scale the translational x and y velocities to be less than the MAX_TRANSLATIONAL_VELOCITY
	float magnitude = sqrt(velocities.x * velocities.x + velocities.y * velocities.y);

	if (magnitude > MAX_TRANSLATIONAL_VELOCITY)
	{
		velocities.x /= magnitude;
		velocities.y /= magnitude;
		velocities.x *= MAX_TRANSLATIONAL_VELOCITY;
		velocities.y *= MAX_TRANSLATIONAL_VELOCITY;
	}

	// Rotational velocity just needs to be less than the maximum
	if(velocities.z > 0 && velocities.z > MAX_ROTATIONAL_VELOCITY)
		velocities.z = MAX_ROTATIONAL_VELOCITY;
	else if(velocities.z < 0 && velocities.z < -MAX_ROTATIONAL_VELOCITY)
		velocities.z = -MAX_ROTATIONAL_VELOCITY;

}

// Uses a service client to get the relationship to your reference neighbor from Environment
void Cell::receiveActualRelationshipFromEnvironment(int neighborIndex)
{
	if(neighborIndex == NO_NEIGHBOR || getCommunicationLostBasedOnError())
		return;

	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	relationshipClient = relationshipNodeHandle.serviceClient<NewSimulator::Relationship>("relationship");

	// Set the request values here
	relationshipService.request.OriginID = cellID;
	relationshipService.request.TargetID = neighborhoodList[neighborIndex];

	if (relationshipClient.call(relationshipService))
	{
		// Actual relationship to neighbor
		cellState.actualRelationships[neighborIndex].x = relationshipService.response.theRelationship.actual_relationship.x;
		cellState.actualRelationships[neighborIndex].y = relationshipService.response.theRelationship.actual_relationship.y;
		cellState.actualRelationships[neighborIndex].z = relationshipService.response.theRelationship.actual_relationship.z;

		// Actual relationship to seed (frp)
		cellFormation.cellFormationRelativePosition.x = relationshipService.response.theRelationship.actual_position.x;
		cellFormation.cellFormationRelativePosition.y = relationshipService.response.theRelationship.actual_position.y;
		cellFormation.cellFormationRelativePosition.z = relationshipService.response.theRelationship.actual_position.z;

//		if(cellID == 0)
//		{
//			cout << "\nFor origin cell " << cellID << " and target cell " << relationshipService.request.TargetID
//					<< "\n       the environment relationship service returned ACTUAL relationship:\n"
//				<< "x: " << relationshipService.response.theRelationship.actual_relationship.x << endl
//				<< "y: " << relationshipService.response.theRelationship.actual_relationship.y << endl
//				<< "z: " << relationshipService.response.theRelationship.actual_relationship.z << endl << endl;
//		}

		relationshipNodeHandle.shutdown();
		spinner.stop();
		return;
	}

	relationshipNodeHandle.shutdown();
	spinner.stop();
	return;
}



// Uses a service client to get the relationship to your reference neighbor from Environment
void Cell::receiveNeighborhoodIdsFromEnvironment(int originId)
{
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	neighborhoodClient = neighborhoodNodeHandle.serviceClient<NewSimulator::Neighborhood>("neighborhood");

	// Set the request values here
	neighborhoodService.request.OriginID = cellID;

	// TODO: create method to return number of formations
	neighborhoodService.request.NumberOfFormations = 1;

	if (neighborhoodClient.call(neighborhoodService))
	{
		neighborhoodIds.swap(neighborhoodService.response.neighborIds);
//		for(int i = 0; i < neighborhoodIds.size(); i++)
//		{
//			cout<<"Neighborhood Ids for cell: "<<cellID<<" - "<<neighborhoodIds[i]<<endl;
//		}

		neighborhoodNodeHandle.shutdown();
		spinner.stop();
		return;
	}

	neighborhoodNodeHandle.shutdown();
	spinner.stop();
	return;
}

// Calculate this cell's desired relationship to its parameterized neighbor
void Cell::calculateDesiredRelationship(int neighborIndex)
{
	// if first formationID is nothing, the rest will be nothing
	if(cellFormation.formationIDs.at(0) == NO_FUNCTION_FORMATION_ID)
		return;


	// For cells to the left of the seed, flip their radius to use their other desired relationship intersection
	float radius = cellFormation.getRadius();
	if(cellID < cellFormation.getSeedID())
		radius *= -1;


	vector<PhysicsVector> desiredRelationship = cellFormation.getDesiredRelationships(cellFormation.getFunctions(), radius,
			cellFormation.cellFormationRelativePosition, cellFormation.getFormationRelativeOrientation());


	for(uint i = 0; i < desiredRelationship.size(); i++) {
		// According to thesis, the desired relationship gets rotated about the negation of the formation relative orientation
		desiredRelationship.at(i).rotateRelative(-1 * cellFormation.getFormationRelativeOrientation());

		cellState.desiredRelationships[neighborIndex].x = desiredRelationship.at(i).x;
		cellState.desiredRelationships[neighborIndex].y = desiredRelationship.at(i).y;
		cellState.desiredRelationships[neighborIndex].z = desiredRelationship.at(i).z;
	}

//	if(cellID == 4)
//	{
//		cout << "\nFor origin cell " << cellID << " and target cell " << neighborhoodList[neighborIndex]
//				<< "\n       the calculated DESIRED relationship is:\n"
//			<< "x: " << cellState.desiredRelationships[neighborIndex].x << endl
//			<< "y: " << cellState.desiredRelationships[neighborIndex].y << endl
//			<< "z: " << cellState.desiredRelationships[neighborIndex].z << endl << endl;
//	}
}

// Creates a message to send to neighhbor cells informing them of the new formation
NewSimulator::FormationMessage Cell::createFormationChangeMessage()
{
	NewSimulator::FormationMessage formationChangeMessage;

	formationChangeMessage.radius = cellFormation.radius;
	formationChangeMessage.formation_count = formationCount;
	formationChangeMessage.seed_id = cellFormation.seedID;
	formationChangeMessage.sensor_error = cellFormation.getSensorError();
	formationChangeMessage.communication_error = cellFormation.getCommunicationError();

	vector<int> formationList = cellFormation.formationIDs;
	for(uint i = 0; i < formationList.size(); i++)
	{
		formationChangeMessage.formation_ids.push_back(formationList.at(i));
	}

	return formationChangeMessage;
}

// This cell uses this to get the new formation from a neighbor who is publishing it (this is the callback)
void Cell::receiveFormation(const NewSimulator::FormationMessage::ConstPtr &formationMessage)
{
	// If the formation count is higher than ours, then this is a newer formation than we currently have stored
	if(formationMessage->formation_count > formationCount)
	{
		cellFormation.setCommunicationError(formationMessage->communication_error);
		if(getCommunicationLostBasedOnError())
			return;

		isFormationChanged = true;
		formationCount = formationMessage->formation_count;
		cellFormation.radius = formationMessage->radius;
		cellFormation.setFunctionFromFormationID(cellFormation.formationIDs);
		cellFormation.seedID = formationMessage->seed_id;
		cellFormation.sensorError = formationMessage->sensor_error;
		cellFormation.setCommunicationError(formationMessage->communication_error);

		vector<int> formationList = formationMessage->formation_ids;
		for(uint i = 0; i < formationList.size(); i++)
		{
			cellFormation.formationIDs.push_back(formationList.at(i));
		}

		return;
	}
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

vector<int> Cell::updateNeighborhood()
{
	//This should be were we make a service request to environment to get the neighbors
	receiveNeighborhoodIdsFromEnvironment(cellID);
	return neighborhoodList;
}

//// temp setNeighbohood - HARDCODED for 1-dimensional formations
//void Cell::setNeighborhood()
//{
//	if(!isMultiFunction) {
//		// For 1-dimensional formations, just set 2 neighbor relationships for each cell
//		cellState.actualRelationships.resize(2, PhysicsVector());
//		cellState.desiredRelationships.resize(2, PhysicsVector());
//
//		setLeftNeighbor(cellID - 1);
//		setRightNeighbor(cellID + 1);
//	} else {
//		updateNeighborhood();
//	}
//}
//
//// temp setLeftNeighbor - HARDCODED for 1-dimensional formations
//void Cell::setLeftNeighbor(const int nbr)
//{
//	if (cellID == 0)
//		neighborhoodList.insert(neighborhoodList.begin() + 0, NO_NEIGHBOR);
//	else {
//		neighborhoodList.insert(neighborhoodList.begin() + 0, nbr);
//
//		// Set up the subscriber callback for new formations between neighbors.  ONLY SUBSCRIBE TO YOUR REFERENCE NEIGHBOR
//		if(cellID > cellFormation.getSeedID() && updateNeighborhood()[0] != NO_NEIGHBOR)
//		{
//			// This cell is to the right of the seed.  Subscribe to our left neighbor.
//			formationChangeSubscriber = formationChangeSubscriberNode.subscribe(generateFormationPubName(updateNeighborhood()[0]), 100, &Cell::receiveFormation, this);
//		}
//	}
//}
//
//// temp setRightNeighbor - HARDCODED for 1-dimensional formations
//void Cell::setRightNeighbor(const int nbr)
//{
//	// Temp max number of cells (6)
//	if (cellID == 6)
//		neighborhoodList.insert(neighborhoodList.begin() + 1, -1);
//	else
//	{
//		neighborhoodList.insert(neighborhoodList.begin() + 1, nbr);
//
//		if(cellID < cellFormation.getSeedID() && updateNeighborhood()[1] != NO_NEIGHBOR)
//		{
//			// This cell is to the left of the seed.  Subscribe to our right neighbor.
//			formationChangeSubscriber = formationChangeSubscriberNode.subscribe(generateFormationPubName(updateNeighborhood()[1]), 100, &Cell::receiveFormation, this);
//		}
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


string Cell::generateFormationPubName(int cellID)
{
	stringstream ss;					//create a stringstream
	ss << (cellID);						//add number to the stream
	string  nbrID = ss.str();
	string subString = "/cell_/formation";

	subString.insert(6, nbrID);
	return subString;
}

// create correct string for state subscriber
string Cell::generateStateSubMessage(int cellID)
{
	stringstream ss;					//create a stringstream
	ss << (cellID);						//add number to the stream
	string  neighborID = ss.str();
	string subString = "/cell_/state";

	subString.insert(6, neighborID);
	return subString;
}

string Cell::generateCommandVelocityPubMessage(int cellID)
{
	stringstream ss;//create a stringstream
	ss << (cellID);//add number to the stream
	string  neighborID = ss.str();

	// We publish to the sphero subscriber that runs with rviz via Ross' code
	string subString = "/sphero/cmd_vel";

	subString.insert(7, neighborID);
	return subString;
}

// Get a neighbor's state from the State service
// Gets the state of the neighbor that is closest to the seed cell (reference cell)
// This should prevent cells from getting conflicting states
void Cell::receiveNeighborState()
{
	stringstream leftSS;					//create a stringstream
	leftSS << (neighborhoodList[0]);		//add number to the stream
	string leftNeighborID = leftSS.str();

	stringstream rightSS;					//create a stringstream
	rightSS << (neighborhoodList[1]);		//add number to the stream
	string rightNeighborID = rightSS.str();


	// If this cell is to the right of the seed, get our left neighbor's state (it is our reference cell)
	if(cellID > cellFormation.getSeedID() && updateNeighborhood()[0] != NO_NEIGHBOR)
	{
		makeStateClientCall(leftNeighborID);
	}
	// If this cell is to the left of the seed, get our right neighbor's state (it is our reference cell)
	if(cellID < cellFormation.getSeedID() && updateNeighborhood()[1] != NO_NEIGHBOR)
	{
		makeStateClientCall(rightNeighborID);
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
}


// Sets the state message to this state's info.  This is the callback for the state service.
bool Cell::setStateMessage(NewSimulator::State::Request &req, NewSimulator::State::Response &res )
{
	res.state.frp.x = cellFormation.seedFormationRelativePosition.x;
	res.state.frp.y = cellFormation.seedFormationRelativePosition.y;

	return true;
}

void Cell::stateCallback(const NewSimulator::StateMessage &incomingState)
{

}

// This cell checks the state service of parameterized neighbor
void Cell::makeStateClientCall(string neighbor)
{
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	stateClient = stateNodeHandle.serviceClient<NewSimulator::State>("cell_state_" + neighbor);

	if (stateClient.call(incomingStateService))
	{
		cellState.timeStep = incomingStateService.response.state.timestep;
		//cout << "Cell " << cellID << " got time step " << cellState.timeStep << " from neighbor " << neighbor << endl;
		stateNodeHandle.shutdown();
		spinner.stop();
		return;
	}
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

// Applies the sensor error set by the user and transmitted to this cell in the formation message
void Cell::applySensorError(int neighborIndex)
{
	if(cellFormation.getSensorError() > FLOAT_ZERO_APPROXIMATION)
	{
		// Generate a random multipler for the error between 0 and 2
		float randomizer = rand() % 200;
		randomizer /= 100;
		cellState.actualRelationships[neighborIndex].x += cellFormation.getSensorError() * randomizer;

		randomizer = rand() % 200;
		randomizer /= 100;
		cellState.actualRelationships[neighborIndex].y += cellFormation.getSensorError() * randomizer;
	}
}

// Use communication error likelihood to determine if the communication is lost
bool Cell::getCommunicationLostBasedOnError()
{
	if(cellFormation.getCommunicationError() > FLOAT_ZERO_APPROXIMATION)
	{
		// Get a random number between 0 and 100.  If it is less than or equal to the error percentage, the messge is lost.
		// This effectively makes x% of every message get lost.
		if((rand() % 100) <= cellFormation.getCommunicationError() || cellFormation.getCommunicationError() > 99.0f)
			return true;
	}

	return false;
}

// Outputs all the useful info about this cell for debugging purposes
void Cell::outputCellInfo()
{
//
//	cout << "\n\nCell stuff:\n"
//		<< "   cell ID: " << cellID << endl
//		<< "   isFormationChanged: " << isFormationChanged << endl
//		<< "   formationCount: " << formationCount << endl
//		<< "   cell state time step: " << cellState.timeStep << endl
//		<< "   getNumberOfNeighbors(): " << getNumberOfNeighbors() << endl
//		<< "Formation stuff:\n"
//		<< "   cellFormation.formationID: " << cellFormation.formationID << endl
//		<< "   cellFormation.currentFunction: " << cellFormation.currentFunctions.at(0) << endl;

//	if(isMultiFunction)
//		cout << "   cellFormation.currentFunction: " << cellFormation.currentFunctions.at(1) << endl;
//
//	cout << "   cellFormation.getRadius: " << cellFormation.getRadius() << endl
//		<< "   seed ID: " << cellFormation.seedID << endl
//		<< "   sensorError: " << cellFormation.getSensorError() << endl
//		<< "   communicationError: " << cellFormation.getCommunicationError() << endl
//		<< "Left neighbor: " << neighborhoodList[0] << endl
//		<< "   Actual relationship: " << cellState.actualRelationships[0].x << ", "
//				<< cellState.actualRelationships[0].y << ", "
//				<< cellState.actualRelationships[0].z << ", " << endl
//		<< "   Desired relationship: " << cellState.desiredRelationships[0].x << ", "
//				<< cellState.desiredRelationships[0].y << ", "
//				<< cellState.desiredRelationships[0].z << ", " << endl
//		<< "Right neighbor: " << neighborhoodList[1] << endl
//		<< "   Actual relationship: " << cellState.actualRelationships[1].x << ", "
//				<< cellState.actualRelationships[1].y << ", "
//				<< cellState.actualRelationships[1].z << ", " << endl
//		<< "   Desired relationship: " << cellState.desiredRelationships[1].x << ", "
//				<< cellState.desiredRelationships[1].y << ", "
//				<< cellState.desiredRelationships[1].z << ", " << endl << endl;

}
