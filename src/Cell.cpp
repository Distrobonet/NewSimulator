////
//// Description:     This class implements a robot cell.
////
//
//#define VERBOSE            (0)
//#define AUTONOMOUS_INIT    (1)
//#define ALLOW_CELL_BIDS    (0)
//#define CELL_INFO_VIEW     (0)
//#define AUCTION_STEP_COUNT (3)
//
//// created for future
//#define RADIUS			   (1.0)
//#define Z_VALUE 		   (0)
//
//#include<iostream>
//// preprocessor directives
//#include <stdlib.h>
//#include <stdio.h>
//#include <Simulator/Cell.h>
//#include <Simulator/Environment.h>
//#include <Simulator/UserFunctions.h>
//
//using namespace std;
//
//int NUM_OF_CELLS = 7;
//
//Function formations[] = {line,        x,       absX,     negHalfX,
//  negAbsHalfX, negAbsX, parabola, cubic,
//  condSqrt,    sine};
//
//// Default constructor that initializes
//// this cell to the parameterized values.
//Cell::Cell(const int cellID) : State(), Neighborhood()
//{
//	leftNbr = rightNbr = -1;
//	ID  			   = cellID;
//	behavior           = DEFAULT_ROBOT_BEHAVIOR;
//	stateChanged 	   = false;
//	startMoving = false;
//
//	if(ID == 0)
//		inPosition = true;
//
//	initNbrs();
//}
//
//// Copy constructor that copies the contents of
//// the parameterized cell into this cell.
//Cell::Cell(const Cell &c) :	State(c), Neighborhood(c)
//{
//	leftNbr = c.leftNbr;
//	rightNbr = c.rightNbr;
//}
//
//// Destructor that clears this cell.
//Cell::~Cell() {}
//
//void Cell::publishState()
//{
//    Simulator::StateMessage state;
//    state.formation.radius = formation.radius;
//    state.formation.heading = formation.heading;
//    state.formation.seed_frp.x = formation.seedFrp.x;
//    state.formation.seed_frp.y = formation.seedFrp.y;
//    state.formation.seed_id = formation.seedID;
//    state.formation.formation_id = formation.formationID;
//    state.in_position = inPosition;
//
////    state.frp.x = frp.x;
////    state.frp.y = frp.y;
////
////    for(uint i = 0;i < rels.size();i++)
////    {
////        state.actual_relationships[i].id = rels[i].ID;
////        state.actual_relationships[i].actual.x = rels[i].relActual.x;
////        state.actual_relationships[i].actual.y = rels[i].relActual.y;
////        state.desired_relationships[i].desired.x = rels[i].relDesired.x;
////        state.desired_relationships[i].desired.y = rels[i].relDesired.y;
////    }
////
////    state.linear_error.x = transError.x;
////    state.linear_error.y = transError.y;
////    state.angular_error = rotError;
////    state.timestep = tStep;
////    state.reference_id = refID;
////    state.temperature = temperature;
////    state.heat = heat;
//
//    state_pub.publish(state);
//    stateChanged = false;
//}
//
//// Updates the cell
//void Cell::update(bool doSpin)
//{
//	ros::Rate loop_rate(10);
//
//	while(ros::ok())
//	{
//		setFormationFromService();
////		if(startMoving)
////		{
////			// Testing relationship service from environment
////			if(ID % 2 == 0)
////				getRelationship(rightNbr.ID);
////			if (ID % 2 == 1)
////				getRelationship(leftNbr.ID);
////
////			Vector movement = Vector(rels[0].relDesired.x - rels[0].relActual.x, rels[0].relDesired.y - rels[0].relActual.y, rels[0].relDesired.z - rels[0].relActual.z);
////			behavior = move(movement);
////
////			translateRelative(behavior.getTransVel());
////			rotateRelative(behavior.getRotVel());
////
////			// publish cmd_vel
//
////		}
////
////		// publish state
//		updateState();
//	    publishState();
//
//		commandVelocity.linear.x = transError.x;
//		commandVelocity.angular.z = rotError;
//		cmd_velPub.publish(commandVelocity);
////
//////	    cout << "Cell ID: "<< ID << endl;
//////	    cout << "formationID: " << formation.formationID << endl;
//////	    cout << "inPosition state: " << inPosition << endl;
//////	    cout << "startMoving state: " << startMoving << endl << endl;
////
////
////
//
//<<<<<<< .mine
////	    if(ID == 1 || ID == 0)
////		{
////			RossMove();
////		}
//=======
//	    if(ID == 1 || ID == 2)
//		{
//			RossMove();
//		}
//>>>>>>> .r393
//
//
//		//if(doSpin)
////			ros::spinOnce();
////
////		//Only updates formation if seed node, takes it from the nbr's state otherwise
////		setFormationFromService();
//	}
//}
//
//
//// Initializes the neighborhood of each cell,
//// returning true if successful, false otherwise.
//bool Cell::initNbrs()
//{
//	// Initializing the relationship vector in state to skeleton information.
//	// Needs size of two for a left and a right neighbor.  This gets populated with data in getNeighborState
//	rels.reserve(2);
//
//
//	string cellSubName;
//	std::stringstream converter;
//
//		int leftNbrID, rightNbrID;
//
//		// Sets what your neighbor's ID should be
//		switch (ID)
//		{
//			case 0:
//				leftNbrID = ID + 2;
//				rightNbrID = ID + 1;
//				break;
//			case 1:
//				leftNbrID = ID - 1;
//				rightNbrID = ID + 2;
//				break;
//			case 2:
//			case 4:
//			case 6:
//				leftNbrID = ID + 2;
//				rightNbrID = ID - 2;
//				break;
//			default:
//				leftNbrID = ID - 2;
//				rightNbrID = ID + 2;
//				break;
//		}
//
//		if(leftNbrID > NUM_OF_CELLS)
//			leftNbr.ID = -1;
//		else
//		{
//			if(addNbr(leftNbrID))
//			{
//				Neighbor *nbrWithId = nbrWithID(leftNbrID);
//				leftNbr = *nbrWithId;
//				leftNeighborStateSubscriber = stateNode.subscribe(generateSubMessage(leftNbrID), 1000, &Cell::stateCallback, this);
//			}
//		}
//
//		if(rightNbrID > NUM_OF_CELLS)
//			rightNbr.ID = -1;
//		else
//		{
//			if(addNbr(rightNbrID))
//			{
//				Neighbor *nbrWithId = nbrWithID(rightNbrID);
//		    	rightNbr = *nbrWithId;
//		    	rightNeighborStateSubscriber = stateNode.subscribe(generateSubMessage(rightNbrID), 1000, &Cell::stateCallback, this);
//			}
//		}
//
//	if(VERBOSE)
//		printf("finished initNbrs()\n");
//	return true;
//}
//
//
//// create correct string for state subscriber
//string Cell::generateSubMessage(int cellID)
//{
//	stringstream ss;//create a stringstream
//	ss << (cellID);//add number to the stream
//	string  nbrID = ss.str();
//	string subString = "/robot_/state";
//
//	subString.insert(7, nbrID);
//	return subString;
//}
//
//
//string Cell::generatePubMessage(int cellID)
//{
//	stringstream ss;//create a stringstream
//	ss << (cellID);//add number to the stream
//	string  nbrID = ss.str();
//	string subString = "/robot_/cmd_vel";
//
//	subString.insert(7, nbrID);
//	return subString;
//}
//
//
//// Attempts to set the state to the parameterized state,
//// returning true if successful, false otherwise.
//bool Cell::setState(const State &s)
//{
//	*this = s;
//	return true;
//}
//
//
//// Attempts to set the neighborhood to the parameterized neighborhood,
//// returning true if successful, false otherwise.
//bool Cell::setNbrs(Neighborhood &nh)
//{
//	*this = nh;
//	return true;
//}
//
//
//// Returns the state of this cell.
//State Cell::getState() const
//{
//	return (State) *this;
//}
//
//
//// Returns the neighborhood of this cell.
//Neighborhood Cell::getNbrs() const
//{
//	return (Neighborhood) *this;
//}
//
//
//// Returns the ID of this robot.
//int Cell::getID() const
//{
//    return ID;
//}
//
//
//// Sets the ID of this robot.
//void Cell::setID(int cellID)
//{
//    ID = cellID;
//}
//
//
//void Cell::stateCallback(const Simulator::StateMessage &incomingState)
//{
////	if(ID > incomingState.reference_id && formation.formationID != incomingState.formation.formation_id && incomingState.formation.formation_id != -1)
////	{
//	if(ID % 2 == 1 && formation.formationID != incomingState.formation.formation_id)
//	{
//		leftNbr.formation.radius = incomingState.formation.radius;
//		leftNbr.formation.heading = incomingState.formation.heading;
//		leftNbr.formation.seedFrp.x = incomingState.formation.seed_frp.x;
//		leftNbr.formation.seedFrp.y = incomingState.formation.seed_frp.y;
//		leftNbr.formation.seedID = incomingState.formation.seed_id;
//		leftNbr.formation.formationID = incomingState.formation.formation_id;
//	}else if(ID % 2 == 0 && formation.formationID != incomingState.formation.formation_id)
//	{
//		rightNbr.formation.radius = incomingState.formation.radius;
//		rightNbr.formation.heading = incomingState.formation.heading;
//		rightNbr.formation.seedFrp.x = incomingState.formation.seed_frp.x;
//		rightNbr.formation.seedFrp.y = incomingState.formation.seed_frp.y;
//		rightNbr.formation.seedID = incomingState.formation.seed_id;
//		rightNbr.formation.formationID = incomingState.formation.formation_id;
//	}else{
//		leftNbr.formation.radius = incomingState.formation.radius;
//		leftNbr.formation.heading = incomingState.formation.heading;
//		leftNbr.formation.seedFrp.x = incomingState.formation.seed_frp.x;
//		leftNbr.formation.seedFrp.y = incomingState.formation.seed_frp.y;
//		leftNbr.formation.seedID = incomingState.formation.seed_id;
//		leftNbr.formation.formationID = incomingState.formation.formation_id;
//		rightNbr.formation.radius = incomingState.formation.radius;
//		rightNbr.formation.heading = incomingState.formation.heading;
//		rightNbr.formation.seedFrp.x = incomingState.formation.seed_frp.x;
//		rightNbr.formation.seedFrp.y = incomingState.formation.seed_frp.y;
//		rightNbr.formation.seedID = incomingState.formation.seed_id;
//		rightNbr.formation.formationID = incomingState.formation.formation_id;
//	}
//
////
////			changeFormation(formations[formation.formationID], rels[0].ID);
//			Vector r = formation.calculateDesiredRelationship(formations[formation.formationID], 1.0f, frp, 0.0f);
////			rels[0].relDesired.x = r.x;
////			rels[0].relDesired.y = r.y;
////			rels[0].relDesired.z = r.z;
////			cout<<r.x<<endl;
////			cout<<r.y<<endl;
////		if((incomingState.in_position == true) && (inPosition == false) && (startMoving != true) && formation.formationID != -1)
////		{
////			startMoving = true;
////			stateChanged = true;
////		}
////	}
//	updateState();
//}
//
//
//// Attempts to change the formation of the cell,
//// returning true if successful, false otherwise.
//bool Cell::changeFormation(const Formation &f, Neighbor seedingNbr)
//{
//	formation = f;
//
//	if (formation.getSeedID() == ID) {
//		frp = formation.getSeedFrp();
//		transError = Vector();
//		rotError = 0.0f;
//	}
//
//	else {
//		Relationship *nbrRelToMe = relWithID(seedingNbr.rels, ID);
//		if (nbrRelToMe == NULL)
//			return false;
//
//		nbrRelToMe->relDesired.rotateRelative(seedingNbr.formation.getHeading());
//		frp = seedingNbr.frp + nbrRelToMe->relDesired;
//		transError = Vector();
//		rotError = 0.0f;
//		delete nbrRelToMe;
//	}
//
//	vector<Vector> desiredRelBasedOffFrp = formation.getRelationships(frp);
//
//
//	if (leftNbr.ID != -1)
//		leftNbr.relDesired = desiredRelBasedOffFrp[LEFT_NBR_INDEX];
//
//	if (rightNbr.ID != -1)
//		rightNbr.relDesired = desiredRelBasedOffFrp[RIGHT_NBR_INDEX];
//
//	return true;
//}
//
//
//// Moves the robot cell using the current translational and
//// rotational errors, activating and returning the appropriate
//// robot behavior.
//Behavior Cell::moveError()
//{
//	return moveErrorBehavior(transError, rotError);
//}
//
//
//// Moves the robot cell using the parameterized translational and
//// rotational errors, activating and returning the appropriate
//// robot behavior.
//Behavior Cell::moveError(const Vector tError, const float rError)
//{
//	return moveErrorBehavior(tError, rError);
//}
//
//
//// Moves the robot using the parameterized translational and
//// rotational errors, returning the appropriate robot behavior.
//Behavior Cell::moveErrorBehavior(const Vector tError, const float rError)
//{
//	if (transError.magnitude() > threshold())
//		return move(transError);
//
//	else if (abs(rotError) > angThreshold())
//		return move(0.0, degreesToRadians(-rotError));
//
//	return moveStop();
//}
//
//void Cell::updateState()
//{
////  if ((getNNbrs()               == 0)     ||
////      (nbrWithMinStep()->tStep  <  tStep) ||
////      ((formation.getSeedID()   != ID)    &&
////       (nbrWithMaxStep()->tStep == tStep))) return;
//
//  // update actual relationships to neighbors
////  Neighbor *currNbr = NULL;
////  for int i = 0; i < size(); ++i)
////  {
////    currNbr = getNbr(i);
////    if (currNbr == NULL) break;
////
////    // change formation if a neighbor has changed formation
//  cout<<ID<<endl;
//  cout<<"form this"<<formation.formationID<<endl;
//  cout<<"left form"<<leftNbr.formation.formationID<<endl;
//	if (ID % 2 == 1)
//	{
//		if (leftNbr.formation.formationID != formation.formationID){
//			changeFormation(leftNbr.formation, leftNbr);
//			cout<<"fuck"<<endl;
//		}
//		getRelationship(leftNbr.ID);
//	}
//
//	if(ID % 2 == 0)
//	{
//		if (rightNbr.formation.formationID != formation.formationID){
//			changeFormation(rightNbr.formation, rightNbr);
//			cout<<"hell yeah"<<endl;
//		}
//		getRelationship(rightNbr.ID);
//	}
////  }
////  rels = getRelationships();
//
//  // reference the neighbor with the minimum gradient
//  // to establish the correct position in formation
//    if (ID == 0)
//    {
//    	rotError = 0;
//    	transError = Vector();
//    }
//
//
//	if(ID % 2 == 0)
//	{
//		Neighbor     refNbr     = rightNbr;
//		Relationship *nbrRelToMe = relWithID(refNbr.rels, ID);
//		if ((formation.getSeedID() != ID)   &&
//				(nbrRelToMe            != NULL))
//		{
//
//			// error (state) is based upon the
//			// accumulated error in the formation
//			cout<<"here left"<<endl;
//			Vector  nbrRelToMeDesired = nbrRelToMe->relDesired;
//			nbrRelToMeDesired.rotateRelative(-refNbr.rotError);
//			float theta = scaleDegrees(nbrRelToMe->relActual.angle() -
//			  (-refNbr.relActual).angle());
//			rotError      = scaleDegrees(theta + refNbr.rotError);
//			transError    = nbrRelToMeDesired - nbrRelToMe->relActual +
//			refNbr.transError;
//			transError.rotateRelative(-theta);
//			//set the state variable of refID  = ID of the reference nbr.
//			refID = refNbr.ID;
//			cout<<"out left"<<endl;
//		}
//	}
//	if (ID % 2 == 1)
//	{
//		getRelationship(leftNbr.ID);
//		Neighbor     refNbr     = leftNbr;
//		Relationship *nbrRelToMe = relWithID(refNbr.rels, ID);
//		if ((formation.getSeedID() != ID)   &&
//				(nbrRelToMe            != NULL))
//		{
//
//			// error (state) is based upon the
//			// accumulated error in the formation
//			cout<<"here right"<<endl;
//			Vector  nbrRelToMeDesired = nbrRelToMe->relDesired;
//			nbrRelToMeDesired.rotateRelative(-refNbr.rotError);
//			float theta = scaleDegrees(nbrRelToMe->relActual.angle() -
//			  (-refNbr.relActual).angle());
//			rotError      = scaleDegrees(theta + refNbr.rotError);
//			transError    = nbrRelToMeDesired - nbrRelToMe->relActual +
//			refNbr.transError;
//			transError.rotateRelative(-theta);
//			//set the state variable of refID  = ID of the reference nbr.
//			refID = refNbr.ID;
//			cout<<"out right"<<endl;
//		}
//	}
//
//	tStep = max(tStep + 1, nbrWithMaxStep()->tStep);
//}   // updateState()
//
//
//// Moves the robot using the parameterized movement vector,
//// returning the appropriate robot behavior.
//Behavior Cell::move(const Vector &target)
//{
//	float theta    		= target.angle();
//	float phi     		= heading.angle();
//	float delta   		= degreesToRadians(theta);
//	float cosDelta 		= cos(delta);
//	float sinDelta 		= sin(delta);
//	float t       		= cosDelta * cosDelta * sign(cosDelta);
//	float r       		= sinDelta * sinDelta * sign(sinDelta);
////	cout << "theta : " << theta << endl
////		 <<	"phi : " << phi << endl
////		 << "delta: " << delta << endl
////		 << "cosdelta: " << cosDelta << endl
////		 << "sindelta: " << sinDelta << endl
////		 << "t: " << t << endl
////		 << "r: " << r << endl << endl;
//	Behavior behavior	= Behavior(t, r, maxSpeed());
//
//	if (abs(theta) < 90.0f)
//	      behavior.setDiffVel(maxSpeed() * (t + r), maxSpeed() * (t - r));
//    else
//        behavior.setDiffVel(maxSpeed() * (t - r), maxSpeed() * (t + r));
//
//	return behavior;
///*
//    float r     = target.magnitude();
//    if (r <= threshold()) return moveStop();
//    float theta = degreesToRadians(target.angle());
//    if (theta == 0.0f)    return moveForwardBehavior(r);
//    else return moveArcBehavior((abs(theta) >
//                                degreesToRadians(angThreshold())) ?
//                                0.0f :
//                                r * velocityTheta / sin(theta), getDiameter() * theta);
//*/
//}
//
//
//// Returns the minimum angular movement threshold of this robot.
//float Cell::angThreshold() const
//{
//    return 0.5f * FACTOR_THRESHOLD * maxAngSpeed();
//}
//
//
//// Returns the minimum movement threshold of this robot.
//float Cell::threshold() const
//{
//    return FACTOR_THRESHOLD * maxSpeed();
//}
//
//
//// Returns the max angular speed of this robot.
//float Cell::maxAngSpeed() const
//{
//    return radiansToDegrees(maxSpeed() / RADIUS);
//}
//
//
//// Stops the robot from moving,
//// returning the appropriate robot behavior.
//Behavior Cell::moveStop()
//{
//	inPosition = true;
//	startMoving = false;
//    return move(0.0f, 0.0f);
//}
//
//
//// Moves the robot using the parameterized translational
//// and rotational velocities, returning the appropriate robot behavior.
//Behavior Cell::move(const float t, const float r)
//{
//    return Behavior(t, r, maxSpeed());
//}
//
//
//// Returns the max speed of this robot.
//float Cell::maxSpeed() const
//{
//    return FACTOR_MAX_SPEED * RADIUS;
//}
//
//float Cell::getHeading() const
//{
//    return heading.angle();
//}
//
//// Attempts to set the heading to the parameterized heading,
//// returning true if successful, false otherwise.
//bool Cell::setHeading(const float theta)
//{
//    return heading.setPolar(RADIUS + VECTOR_HEAD_HEIGHT, theta, Z_VALUE);
//}
//
//
//// Copies the contents of the parameterized state into this cell.
//Cell& Cell::operator =(const State &s) {
//	return *this = s;
//}
//
//// Copies the contents of the parameterized neighborhood into this cell.
//Cell& Cell::operator =(const Neighborhood &nh) {
//	return *this = nh;
//}
//
//
//// Sets this cell's formation object from the response from the Formation service
//bool Cell::setFormationFromService()
//{
//	if (ID == 0)
//	{
//		//ROS_INFO("Trying to access the formation message");
//		ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
//		spinner.start();
//
//		ros::NodeHandle clientNode;
//		formationClient = clientNode.serviceClient<Simulator::CurrentFormation>("formation");
//
//		if (formationClient.call(formationSrv))
//		{
//
//			formation.formationID = formationSrv.response.formation.formation_id;
////			formation.heading = formationSrv.response.formation.heading;
////			formation.radius = formationSrv.response.formation.radius;
////			formation.seedFrp.x = formationSrv.response.formation.seed_frp.x;
////			formation.seedFrp.y = formationSrv.response.formation.seed_frp.y;
////			formation.seedID = formationSrv.response.formation.seed_id;
////			stateChanged = true;
//
//			clientNode.shutdown();
//			spinner.stop();
//			return true;
//		}
//
//		clientNode.shutdown();
//		spinner.stop();
//		return false;
//
//	}
//	else
//		return false;
//}
//
//
//// Starts the cell's state service server
//void Cell::startStateServiceServer()
//{
//	ros::NodeHandle StateServerNode;
//	string name = "cell_state_";
//	name = name + boost::lexical_cast<std::string>(ID);	// add the index to the name string
//
//	stateService = StateServerNode.advertiseService(name, &Cell::setStateMessage, this);
//	//cout << "Now serving the " << stateService.getService() << " service!\n";
//
//	ros::spinOnce();
//
//	//StateServerNode.shutdown();
//}
//
//
//// Sets the state message to this state's info.  This is the callback for the state service.
//bool Cell::setStateMessage(Simulator::State::Request  &req, Simulator::State::Response &res )
//{
//  	res.state.formation.radius = formation.radius;
//  	res.state.formation.heading = formation.heading;
//  	res.state.formation.seed_frp.x = formation.seedFrp.x;
//  	res.state.formation.seed_frp.y = formation.seedFrp.y;
//  	res.state.formation.seed_id = formation.seedID;
//  	res.state.formation.formation_id = formation.formationID;
//  	res.state.in_position = inPosition;
//
////	res.state.frp.x = frp.x;
//// 	res.state.frp.y = frp.y;
////
//// 	for(uint i = 0; i < rels.size(); i++)
//// 	{
////        res.state.actual_relationships[i].id = rels[i].ID;
////        res.state.actual_relationships[i].actual.x = rels[i].relActual.x;
////        res.state.actual_relationships[i].actual.y = rels[i].relActual.y;
////        res.state.desired_relationships[i].desired.x = rels[i].relDesired.x;
////        res.state.desired_relationships[i].desired.y = rels[i].relDesired.y;
//// 	}
////
////	res.state.linear_error.x = transError.x;
////	res.state.linear_error.y = transError.y;
////	res.state.angular_error = rotError;
////	res.state.timestep = tStep;
////	res.state.reference_id = refID;
////	res.state.temperature = temperature;
////	res.state.heat = heat;
//
//	ROS_INFO("sending back response with state info");
//	return true;
//}
//
//
//// Relationship client, sends its ID and a target ID as requests and gets a relationship vector response
//bool Cell::getRelationship(int targetID)
//{
//    //cout << "Relationship client called for cell " << ID << " targeting cell " << targetID <<  endl;
//	ros::AsyncSpinner spinner(1);	// Uses an asynchronous spinner to account for the blocking service client call
//	spinner.start();
//
//	ros::NodeHandle clientNode;
//	relationshipClient = clientNode.serviceClient<Simulator::Relationship>("relationship");
//
//
//	//Set the request values to the service
//	relationshipSrv.request.OriginID = ID;
//	relationshipSrv.request.TargetID = targetID;
//
//
//	// Here is where neighbor states get set from the relationship service
//	if (relationshipClient.call(relationshipSrv))
//	{
//		//cout << "The relationship call for cell " << ID << " targeting cell " << targetID << " was successful!\n";
//
//		// Set the returned relationship to the stored neighbor state in the relationship vector
//		if(targetID == leftNbr.ID)		// This is the cell's relationship to its left neighbor
//		{
//			rels[0].ID = targetID;
//			rels[0].relActual.x = relationshipSrv.response.theRelationship.actual.x;
//			rels[0].relActual.y = relationshipSrv.response.theRelationship.actual.y;
//			rels[0].relActual.z = 0;
//
//		}
//		else if(targetID == rightNbr.ID)	// right neighbor relationship
//		{
//			rels[0].ID = targetID;
//			rels[1].relActual.x = relationshipSrv.response.theRelationship.actual.x;
//			rels[1].relActual.y = relationshipSrv.response.theRelationship.actual.y;
//
//			rels[1].relDesired.x = relationshipSrv.response.theRelationship.desired.x;
//			rels[1].relDesired.y = relationshipSrv.response.theRelationship.desired.y;
//
//			// Test stuff
////			cout << "\nCell " << ID << " has ACTUAL relationship with cell " << targetID << " of: " << rels[1].relActual.x << ", " << rels[1].relActual.y << endl;
////			cout << "Cell " << ID << " has DESIRED relationship with cell " << targetID << " of: " << rels[1].relDesired.x << ", " << rels[1].relDesired.y << endl;
//		}
////
////		else
////			cout << "\nSomething is wrong in getRelationship.  Neighbor is neither left nor right?\n";
//
//		//clientNode.shutdown();
//		spinner.stop();
//		return true;
//	}
//	else
//	{
//		//ROS_ERROR("Failed to call relationship service");
//		cout << "failed to call " << relationshipClient.getService() << " service\n";
//		//clientNode.shutdown();
//		spinner.stop();
//		return false;
//	}
//}
//
//// Get a neighbor's state from the State service
//bool Cell::getNeighborState()
//{
////	if(ID == 0)
////		return true;
////	if(ID % 2 == 1)
////	{
////		formation.formationID = leftNbr.formation.formationID;
////		return true;
////	}
////	else if(ID % 2 == 0)
////	{
////		formation.formationID = rightNbr.formation.formationID;
////		return true;
////
////	}
////	else
////		return false;
//}
//
//int Cell::RossMove()
//{
//	// determine actual relationships
//	getRelationship(rightNbr.ID);
//	getRelationship(leftNbr.ID);
//
//
//
//	//The difference in desired and actual relationship will generate translational (a vector with x and y components)
//	//and rotational (an angle in radians) error values, which can be translated to linear (x, y) and angular (yaw) command velocities.
//
//	// Desired relationship must be calculated somewhere
//
//
//
//
//	// calculate translational/rotational error
//	if(ID == 1)
//	{
//		transError.x = rels[0].relDesired.x - rels[0].relActual.x;
//		transError.y = rels[0].relDesired.y - rels[0].relActual.y;
//		rotError = rels[0].relDesired.z - rels[0].relActual.z;
//	}
//	else if(ID == 2)
//	{
//		transError.x = rels[1].relDesired.x - rels[1].relActual.x;
//		transError.y = rels[1].relDesired.y - rels[1].relActual.y;
//		rotError = rels[1].relDesired.z - rels[1].relActual.z;
//	}
//
//
//
//
//
//	// "gains" for proportional motor control
//	// - I made these values up, so they need to be tuned
//	// - often small positive values (likely between 0.0 and 2.0)
//	// - should be set somewhere else, not in this function
//	double gain_x = 0.4;
//	double gain_y = 0.4;
//	double gain_th = 1.0;
//
//	// minimum speed limits
//	// - in meters (or radians) per second
//	// - should be positive values
//	// - should be set somewhere else, not in this function
//	double speed_min_x = 0.0;
//	double speed_min_y = 0.0;
//	double speed_min_th = 0.0;
//
//	// maximum speed limits
//	// - in meters (or radians) per second
//	// - should be positive values
//	// - should be set somewhere else, not in this function
//	double speed_max_x = 1.0;
//	double speed_max_y = 1.0;
//	double speed_max_th = 0.5 * M_PI;
//
//	// calculate motor command velocities based on
//	// translational and rotational error
//	double v_x = gain_x * transError.x;
//	double v_y = gain_y * transError.y;
//	double v_th = gain_th * rotError;
//
//	// scale linear x speed to [speed_min_x, speed_max_x]
//	if (abs(v_x) < speed_min_x)
//	v_x = speed_min_x * sign(v_x);
//	else if (abs(v_x) > speed_max_x)
//	v_x = speed_max_x * sign(v_x);
//
//	// scale linear y speed to [speed_min_y, speed_max_y]
//	if (abs(v_y) < speed_min_y)
//	v_y = speed_min_y * sign(v_y);
//	else if (abs(v_y) > speed_max_y)
//	v_y = speed_max_y * sign(v_y);
//
//
//	// scale linear theta speed to [speed_min_th, speed_max_th]
//	if (abs(v_th) < speed_min_th)
//	v_th = speed_min_th * sign(v_th);
//	else if (abs(v_th) > speed_max_th)
//	v_th = speed_max_th * sign(v_th);
//
//	// create cmd_vel (a Twist message)
//	geometry_msgs::Twist cmd_vel;
//	cmd_vel.linear.x = v_x;
//	cmd_vel.linear.y = v_y;
//	cmd_vel.angular.z = v_th;
//
//	// assume pub_cmd_vel already exists somewhere
//	cmd_velPub.publish(cmd_vel);
//
//	return ++tStep;
//}    // end step() function
//
//// Translates the robot relative to itself based
//// on the parameterized translation vector.
//void Cell::translateRelative(Vector v)
//{
//    v.rotateRelative(getHeading());
////    x += v.x;
////    y += v.y;
//}
//
//
//// Translates the robot relative to itself based
//// on the parameterized x-/y-coordinate translations.
//void Cell::translateRelative(const float dx, const float dy)
//{
//    translateRelative(Vector(dx, dy));
//}
//
//
//// Rotates the robot about itself (in 2-dimensions)
//// based on the parameterized rotation angle.
//void Cell::rotateRelative(float theta)
//{
//    heading.rotateRelative(theta);
//}
//
