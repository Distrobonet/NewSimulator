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
	cellFormation.seedID = 3;
	isFormationChanged = false;
	isMultiFunction = false;
	referencePosition = NO_NEIGHBOR;

	commandVelocity.linear.x = 0;
	commandVelocity.linear.y = 0;
	commandVelocity.linear.z = 0;

	commandVelocity.angular.x = 0;
	commandVelocity.angular.y = 0;
	commandVelocity.angular.z = 0;

	// For 1-dimensional formations, just set 2 neighbor relationships for each cell
	cellState.actualRelationships.resize(2, PhysicsVector());
	cellState.desiredRelationships.resize(2, PhysicsVector());

	neighborhoodList.resize(2, NO_NEIGHBOR);

//	updateNeighborhood();

	// Set the seed cell to subscribe to the Simulator's formation messages
	if(cellID == cellFormation.getSeedID())
	{
		simulatorFormationSubscriber = simulatorFormationNodeHandle.subscribe("seedFormationMessage", 100, &Cell::receiveFormationFromSimulator, this);
	}
	startAuctionServiceServer();
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
	ros::Rate loop_rate(10);	// This should be 10 in production code

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
		if(cellID != cellFormation.seedID && (referencePosition == 0 || referencePosition == 1))
		{
//			cout << "\n ***** Cell " << cellID << " is trying to move...";
			receiveNeighborState();
			moveFunction();
		}


//		if(cellID == 2 || cellID == 4)
//		{
//			for(uint i = 0; i <  neighborhoodList.size(); i++)
//			{
//				cout << "\n *****" << cellID << " neighborList " << i << ": " << neighborhoodList[i] << endl;
//			}
//		}

		if(cellID == 3)
		{
//			cout << "\nCell " << cellID << " OFFICIALLY has left neighbor " << neighborhoodList[0] << " and right neighbor " << neighborhoodList[1];
		}



		ros::spinOnce();
		loop_rate.sleep();
	}
}

void Cell::updateNeighborhood()
{
	/* 1. seed gets proximity list
	 * 2. seed sets left to closest cell
	 * 3. seed sets right to 2nd closest cell
	 * 4. seed sends auction message to closest(left) cell containing LEFT_NEIGHBOR value
	 * 5. seed sends auction message to second closest(right) cell containing RIGHT_NEIGHBOR value
	 *
	 * 1. cell waits till the formation message sent before establishing neighborhood
	 * 2. cell sets closest nbr (except referenceNbr) to appropriate side (given from last auction message)
	 */


	receiveNeighborhoodIdsFromEnvironment(cellID);


	if(possibleNeighborList.size() < 6)
	{
		cout << "\nCell " << cellID << " has possible neighborhood list of size " << possibleNeighborList.size();
		return;
	}


	if(cellID == cellFormation.seedID)
	{
		bool leftNeighborSet = false;
		bool rightNeighborSet = false;
		for(uint i = 0; i < possibleNeighborList.size(); i++)
		{
//			cout << "\nCell " << cellID << " has POSSIBLE neighbors " << possibleNeighborList[i];
			if(!leftNeighborSet)
			{
				if(makeAuctionConnectionCall(possibleNeighborList[i], LEFT_POSITION))
				{
					leftNeighborSet = true;
					neighborhoodList[0] = possibleNeighborList[i];
//					cout << "\nCell " << cellID << " has left neighbor " << possibleNeighborList[i];
					continue;
				}
			}
			else if(!rightNeighborSet && leftNeighborSet)
			{
				if(makeAuctionConnectionCall(possibleNeighborList[i], RIGHT_POSITION))
				{
					rightNeighborSet = true;
					neighborhoodList[1] = possibleNeighborList[i];
//					cout << "\nCell " << cellID << " has right neighbor " << possibleNeighborList[i];
				}
			}
		}

		// Output whether the seed set its neighbors for debugging
//		if(leftNeighborSet && rightNeighborSet)
//			cout << "\nCell " << cellID << " successfully set its neighbors\n";
	}
	else
	{
		bool neighborSet = false;
		// We now have the list of neighbors, iterate through it to set our closest non-reference neighbor
		for(uint i = 0; i < possibleNeighborList.size(); i++)
		{
			if(possibleNeighborList[i] != cellFormation.referenceNbrID && !neighborSet)
			{
				if(referencePosition == LEFT_POSITION)
				{
					if(makeAuctionConnectionCall(possibleNeighborList[i], RIGHT_POSITION))
					{
						neighborhoodList[0] = possibleNeighborList[i];
						neighborSet = true;
					}
				}
				else {
					if(makeAuctionConnectionCall(possibleNeighborList[i], LEFT_POSITION))
					{
						neighborhoodList[1] = possibleNeighborList[i];
						neighborSet = true;
					}
				}
			}
		}
	}
}

void Cell::moveFunction() {
	int referenceIndex = NO_NEIGHBOR;

	for(uint i = 0; i < neighborhoodList.size(); i++) {
		if(neighborhoodList[i] == cellFormation.referenceNbrID)
			referenceIndex = i;
	}

	if(referenceIndex != NO_NEIGHBOR) {
		receiveActualRelationshipFromEnvironment(referenceIndex);
		applySensorError(referenceIndex);
		calculateDesiredRelationship(referenceIndex);
		move(referenceIndex);
	}
}

// Movement =  desired relationship - actual relationship.  Movement is relative to the parameterized neighbor
void Cell::move(int neighborIndex)
{
	// At this point, we should know the cell's actual and desired relationship to its reference neighbor.

	// If a formation hasn't been set, don't move
	if(cellFormation.formationID == NO_FUNCTION_FORMATION_ID || getCommunicationLostBasedOnError())
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

	if(velocities.x < FLOAT_ZERO_APPROXIMATION && velocities.x > -FLOAT_ZERO_APPROXIMATION) {
		velocities.x = 0;
	} if(velocities.y < FLOAT_ZERO_APPROXIMATION && velocities.y > -FLOAT_ZERO_APPROXIMATION) {
		velocities.y = 0;
	} if(velocities.z < FLOAT_ZERO_APPROXIMATION && velocities.z > -FLOAT_ZERO_APPROXIMATION) {
		velocities.z = 0;
	}

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

// Calculate this cell's desired relationship to its parameterized neighbor
void Cell::calculateDesiredRelationship(int neighborIndex)
{
	if(cellFormation.formationID == NO_FUNCTION_FORMATION_ID)
		return;

	// For cells to the left of the seed, flip their radius to use their other desired relationship intersection
	float radius = cellFormation.getRadius();
	if(!referencePosition)
		radius *= -1;

	PhysicsVector desiredRelationship = cellFormation.getDesiredRelationship(cellFormation.getFunctions()[0], radius,
			cellFormation.cellFormationRelativePosition, cellFormation.getFormationRelativeOrientation());


	// According to thesis, the desired relationship gets rotated about the negation of the formation relative orientation
	desiredRelationship.rotateRelative(-1 * cellFormation.getFormationRelativeOrientation());

	cellState.desiredRelationships[neighborIndex].x = desiredRelationship.x;
	cellState.desiredRelationships[neighborIndex].y = desiredRelationship.y;
	cellState.desiredRelationships[neighborIndex].z = desiredRelationship.z;


//	if(cellID == 4)
//	{
//		cout << "\nFor origin cell " << cellID << " and target cell " << neighborhoodList[neighborIndex]
//				<< "\n       the calculated DESIRED relationship is:\n"
//			<< "x: " << cellState.desiredRelationships[neighborIndex].x << endl
//			<< "y: " << cellState.desiredRelationships[neighborIndex].y << endl
//			<< "z: " << cellState.desiredRelationships[neighborIndex].z << endl << endl;
//	}
}

// Creates a message to send to neighbor cells informing them of the new formation
NewSimulator::FormationMessage Cell::createFormationChangeMessage()
{
	NewSimulator::FormationMessage formationChangeMessage;

	formationChangeMessage.radius = cellFormation.radius;
	formationChangeMessage.formation_id = cellFormation.formationID;
	formationChangeMessage.formation_count = formationCount;
	formationChangeMessage.seed_id = cellFormation.seedID;
	formationChangeMessage.reference_nbr_id = cellID;
	formationChangeMessage.sensor_error = cellFormation.getSensorError();
	formationChangeMessage.communication_error = cellFormation.getCommunicationError();
	return formationChangeMessage;
}

// Uses a subscriber to get the formation from Simulator (this is the callback). Only the seed does this.
void Cell::receiveFormationFromSimulator(const NewSimulator::FormationMessage::ConstPtr &formationMessage)
{
	// If the formation count is higher than ours, then this is a newer formation than we currently have stored
	if(formationMessage->formation_count > formationCount)
	{
		cellFormation.setCommunicationError(formationMessage->communication_error);

		if(getCommunicationLostBasedOnError())
		return;

		isFormationChanged = true;
		cellFormation.radius = formationMessage->radius;
		cellFormation.formationID = formationMessage->formation_id;
		cellFormation.setFunctionFromFormationID(cellFormation.formationID);
		formationCount = formationMessage->formation_count;
		cellFormation.seedID = formationMessage->seed_id;
		cellFormation.setSensorError(formationMessage->sensor_error);
		cellFormation.setCommunicationError(formationMessage->communication_error);

//		 cout << "\nSeed Cell " << cellID << " got new formation: " << formationMessage->formation_id << " from Simulator.\n";
		return;
	}
}

// This cell uses this to get the new formation from a neighbor who is publishing it (this is the callback)
void Cell::receiveFormationFromNeighbor(const NewSimulator::FormationMessage::ConstPtr &formationMessage)
{
	// If the formation count is higher than ours, then this is a newer formation than we currently have stored
	if(formationMessage->formation_count > formationCount)
	{
		cellFormation.communicationError = formationMessage->communication_error;

		if(getCommunicationLostBasedOnError())
		return;

		isFormationChanged = true;

		cellFormation.radius = formationMessage->radius;
		cellFormation.formationID = formationMessage->formation_id;
		cellFormation.setFunctionFromFormationID(cellFormation.formationID);
		formationCount = formationMessage->formation_count;
		cellFormation.seedID = formationMessage->seed_id;
		cellFormation.sensorError = formationMessage->sensor_error;

		 cout << "\nCell " << cellID << " got new formation: " << cellFormation.formationID << " from reference neighbor " << cellFormation.referenceNbrID;
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

void Cell::setFormation(Formation formation)
{
	cellFormation = formation;
}

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
	if(cellID > cellFormation.getSeedID() && neighborhoodList[0] != NO_NEIGHBOR)
	{
		makeStateClientCall(leftNeighborID);
	}
	// If this cell is to the left of the seed, get our right neighbor's state (it is our reference cell)
	if(cellID < cellFormation.getSeedID() && neighborhoodList[1] != NO_NEIGHBOR)
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
//	if(cellID == 4)
	{
		cout << "\n\nCell stuff:\n"
				<< "   cell ID: " << cellID << endl
				<< "   isFormationChanged: " << isFormationChanged << endl
				<< "   formationCount: " << formationCount << endl
				<< "   cell state time step: " << cellState.timeStep << endl
				<< "   getNumberOfNeighbors(): " << getNumberOfNeighbors() << endl
				<< "Formation stuff:\n"
				<< "   cellFormation.formationID: " << cellFormation.formationID << endl;
		for(uint i = 0; i < cellFormation.currentFunctions.size(); i++)
		{
			cout << "   cellFormation.currentFunctions: " << cellFormation.currentFunctions[i] << endl;
		}
		cout	<< "   cellFormation.getRadius: " << cellFormation.getRadius() << endl
				<< "   seed ID: " << cellFormation.seedID << endl
				<< "   sensorError: " << cellFormation.getSensorError() << endl
				<< "   communicationError: " << cellFormation.getCommunicationError() << endl
				<< "Left neighbor: " << neighborhoodList[0] << endl
				<< "   Actual relationship: " << cellState.actualRelationships[0].x << ", "
						<< cellState.actualRelationships[0].y << ", "
						<< cellState.actualRelationships[0].z << ", " << endl
				<< "   Desired relationship: " << cellState.desiredRelationships[0].x << ", "
						<< cellState.desiredRelationships[0].y << ", "
						<< cellState.desiredRelationships[0].z << ", " << endl
				<< "Right neighbor: " << neighborhoodList[1] << endl
				<< "   Actual relationship: " << cellState.actualRelationships[1].x << ", "
						<< cellState.actualRelationships[1].y << ", "
						<< cellState.actualRelationships[1].z << ", " << endl
				<< "   Desired relationship: " << cellState.desiredRelationships[1].x << ", "
						<< cellState.desiredRelationships[1].y << ", "
						<< cellState.desiredRelationships[1].z << ", " << endl << endl;
	}
}

// Uses a service client to get the relationship to your reference neighbor from Environment
void Cell::receiveNeighborhoodIdsFromEnvironment(int originId)
{
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();

	neighborhoodClient = neighborhoodNodeHandle.serviceClient<NewSimulator::Neighborhood>("neighborhood");

	// Set the request values here
	neighborhoodService.request.OriginID = cellID;

	if (neighborhoodClient.call(neighborhoodService))
	{
		possibleNeighborList = neighborhoodService.response.neighborIds;
//		for(int i = 0; i < possibleNeighborList.size(); i++)
//		{
//			cout<<"\nNeighborhood Ids for cell: "<< cellID << " - "<<possibleNeighborList[i] << endl;
//		}

		neighborhoodNodeHandle.shutdown();
		spinner.stop();
		return;
	}

	neighborhoodNodeHandle.shutdown();
	spinner.stop();
}

bool Cell::makeAuctionConnectionCall(int targetCellId, int newReferencePosition)
{
//	cout << "\nAbout to attempt auction call...";
	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
	spinner.start();
	if(targetCellId == NO_NEIGHBOR){
		return false;
	}
	string name = "cell_auctioning_";
	name = name + boost::lexical_cast<std::string>(targetCellId);	// add the index to the name string
	auctionService.request.OriginID = cellID;
	auctionService.request.referencePosition = newReferencePosition;

	auctionClient = auctionNodeHandle.serviceClient<NewSimulator::Auctioning>(name);

	if (auctionClient.call(auctionService))
	{
		// **********************************************************************************
		string responseOutput;
		if(auctionService.response.acceptOrDeny)
			responseOutput = "true";
		else
			responseOutput = "false";
//		cout << "\nauction call FROM cell " << cellID << " TO cell " << toCallCellId << " succeeded.  Response: " << responseOutput;
		// **********************************************************************************


		bool response = auctionService.response.acceptOrDeny;
		if(response)
		{
			if(referencePosition == LEFT_POSITION)
			{
				neighborhoodList[0] = targetCellId;
			}
			else
			{
				neighborhoodList[1] = targetCellId;
			}
		}

		auctionNodeHandle.shutdown();
		spinner.stop();
		return response;
	}
	auctionClient.shutdown();
	spinner.stop();
	return false;
}

// Auctioning server
void Cell::startAuctionServiceServer()
{
	ros::NodeHandle AuctionServerNode;
	string name = "cell_auctioning_";
	name = name + boost::lexical_cast<std::string>(cellID);	// add the index to the name string

	auctionServer = AuctionServerNode.advertiseService(name, &Cell::setAuctionResponse, this);

	ros::spinOnce();
}

bool Cell::setAuctionResponse(NewSimulator::Auctioning::Request &request, NewSimulator::Auctioning::Response &response)
{
//	// If the cell already contains the requesting cell as a neighbor, respond false
//	if(count(neighborhoodList.begin(), neighborhoodList.end(), request.OriginID) != 0){
//		response.acceptOrDeny = false;
//		return true;
//	}



	// If I'm on the requester's left and I've got an empty right neighbor spot,
	// I'll set him to my right neighbor and subscribe to his formation change requests and set him as my reference neighbor
	int leftOrRight;
	if(request.referencePosition == 0)
		leftOrRight = 1;
	else
		leftOrRight = 0;

	if(neighborhoodList[leftOrRight] != NO_NEIGHBOR)
	{	response.acceptOrDeny = false;
		return true;
	}
	referencePosition = request.referencePosition;
	neighborhoodList[leftOrRight] = request.OriginID;

//	cout << "\nCell " << cellID << " got an auction request from cell " << request.OriginID << " with referencePosition " << request.referencePosition
//			<< ".  \n           The requesting cell is now my neighborhoodList[" << leftOrRight << "] neighbor.";

	formationChangeSubscriber = formationChangeSubscriberNode.subscribe(generateFormationPubName(neighborhoodList[leftOrRight]), 100, &Cell::receiveFormationFromNeighbor, this);
	cellFormation.referenceNbrID = request.OriginID;
	response.acceptOrDeny = true;
	return true;
}
