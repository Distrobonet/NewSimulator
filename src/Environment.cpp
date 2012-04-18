//
// Description:     This class implements a robot cell environment.
//

// preprocessor directives
#include <stdio.h>
#include <Simulator/Environment.h>
#include <time.h>
#include <ctime>
#include <stdlib.h>

#define AUCTION_STEP_COUNT (3)



// Default constructor that initializes
// this environment to the parameterized values.
Environment::Environment(const int numRobots, const Formation formation)
{
  if (!init(numRobots, formation))
	  clear();
}   // Environment(const int, const Formation, const Color)


// Copy constructor that copies the contents of
// the parameterized environment into this environment.
  Environment::Environment(const Environment &env)
: cells(env.cells), robots(env.robots), objects(env.objects),  // BAD!!!
  msgQueue(env.msgQueue)
{
}


// Destructor that clears this environment.
Environment::~Environment()
{
  clear();
}


// Clears this environment
void Environment::clear()
{
   while (removeCell());
   while (removeRobot());
//   while (removeObject());
}


// Executes the next step in each cell in the environment
// and forwards all sent packets to their destinations.
bool Environment::step()
{
//  vector<Cell*> auctionCalls;
//  Cell *auctionCall = NULL;
//  Cell *currCell = NULL;
//  Robot *r = NULL;

//  for (int i = 0; i < getNumberOfCells(); i++)
//  {
//    auctionCall = cells[i]->cStep();
//    if((auctionCall != NULL)&&(startFormation))
//      auctionCalls.push_back(auctionCall);
//  }

//  if(startFormation)
//  {
//    for(unsigned i = 0; i < auctionCalls.size(); i++)
//    {
//      Cell* a = auctionCalls[i];
//      State s = a->getState() ;
//      Formation f = formation;
//      bool dir;
//
//      if (a->rightNbr == NULL)
//    	  dir = true;
//      else
//    	  dir = false;
//
//      Auction_Announcement* aa = new Auction_Announcement(a->getState().gradient, s, dir);
//      sendMsg((Message)aa, ID_BROADCAST, a->getID(), AUCTION_ANNOUNCEMENT);
//      a->setAuctionStepCount(1);
//    }

    for(unsigned i = 0; i < robots.size(); i++)
    {
//      robots[i]->processPackets();
      robots[i]->step();
    }
    forwardPackets();
//    auctionCalls.clear();
//    for(int i=0; i<getNumberOfCells(); i++)
//    {
//      if(cells[i]->getAuctionStepCount() >= AUCTION_STEP_COUNT)
//    	  cells[i]->settleAuction();
//    }
//  }
  return true;
}


// Initializes the environment to the parameterized values,
// returning true if successful, false otherwise.
bool Environment::init(const int numberOfRobots, const Formation f)
{
  srand(time(NULL));

  numOfRobots  = numberOfRobots;

  formation    = f;
  formation.setFormationID(0);
  formationID  = 0;

  bool result = true;
  startFormation = false;
  initCells(numberOfRobots, f);
  //system("PAUSE");
  initRobots();

  if (VERBOSE)
	  cout << "finished initCells()\n";
  return result;
}   // init(const int, const Formation)


bool Environment::initRobots()
{
  for (int i = 0; i < numOfRobots; i++)
    addRobot(randSign() * frand(),
        randSign() * frand(),
        0.0f,
        randSign() * frand(0.0f, 180.0f));
  return true;
}


// Initializes each cell to the parameterized values,
// returning true if successful, false otherwise.
bool Environment::initCells(const int numberOfRobots, const Formation f)
{
	srand(time(NULL));
	for (int i = 0; i < numberOfRobots; i++)
		if (!addCell())
			return false;

	// initialize each robot's neighborhood
	if (!initNbrs())
		return false;

	// organizes the cells into an initial formation (default line)
	Cell *c  = NULL;
	for (int i = 0; i < numberOfRobots; i++)
	{
//		if (!cells.getHead(c))
//			return false;
		c->x = formation.getRadius() * ((float)i - (float)(getNumberOfCells() - 1) / 2.0f);
		c->y = 0.0f;
		c->setHeading(formation.getHeading());
		cells.push_back(c);

		if(VERBOSE)
			printf("iterating through cells...\n");
	}

	newestCell = c;
	//printf("newestCell == %d\n",c->getID());
	return (getNumberOfCells() == numberOfRobots)
			&& sendMsg(new Formation(formation), formation.getSeedID(), ID_OPERATOR, CHANGE_FORMATION);
}


// Initializes the neighborhood of each cell,
// returning true if successful, false otherwise.
bool Environment::initNbrs(const int nNbrs)
{
	Cell *c = NULL;
	for (int i = 0; i < getNumberOfCells(); i++)
	{
//		if (!cells.getHead(c))
//			return false;

		c->clearNbrs();
		if ((i > 0) && (!c->addNbr(i - 1)))
			return false;

		else
			c->leftNbr  = c->nbrWithID(i - 1);

		if ((i < getNumberOfCells() - 1) && (!c->addNbr(i + 1)))
			return false;
		else
			c->rightNbr = c->nbrWithID(i + 1);

		// TODO: fix this neighborhood
		//neighborhood.push_back(c);
	}

	if(VERBOSE)
		printf("finished initNbrs()\n");
	return true;
}


// Attempts to add a cell to the environment,
// returning true if successful, false otherwise.
bool Environment::addCell(Cell *cell)
{
  if ((cell == NULL) && ((cell = new Cell()) == NULL))
	  return false;

  cell-> setEnvironment(this);

  // attempt to add this cell to the cell list
  cells.push_back(cell);
  return true;
}


// Attempts to remove a cell from the environment,
// returning true if successful, false otherwise.

bool Environment::removeCell()
{
  if (cells.size() == 0)
	  return false;

  Cell *cell = cells[cells.size() - 1];
  if (!removeCell(cell))
	  return false;

  delete cell;
  return true;
}


// Attempts to remove a cell from the environment,
// storing the address of the removed cell and
// returning true if successful, false otherwise.
bool Environment::removeCell(Cell* cell)
{
  if(cell==NULL)
    return false;

  bool answer = false;

  for(unsigned i = 0; i < cells.size(); i++)
  {
    if(cell == cells[i])
    {
      cells.erase(cells.begin() + i);
      answer = true;
      break;
    }
  }
  return answer;
}


// Attempts to add an object to the environment
// at the parameterized position, returning
// true if successful, false otherwise.
//bool Environment::addObject(float dx, float dy, float dz)
//{
//   Object* o = new Object(dx, dy, dz);
//   if (!addObject(o))
//   {
//      if (o != NULL) delete o;
//      return false;
//   }
//   return true;
//}  // addObject(float, float, float)


// Attempts to add an object to the environment,
// returning true if successful, false otherwise.
//bool Environment::addObject(Object *o)
//{
//   if ((o == NULL) && ((o = new Object()) == NULL)) return false;
//   o->setRadius(DEFAULT_ROBOT_RADIUS);
//   o->showFilled = true;
//   objects.push_back(o);   // attempt to add this object to the object list
//   return true;
//}   // addObject(Object *)


// Attempts to remove an object from the environment,
// returning true if successful, false otherwise.
//bool Environment::removeObject()
//{
//   if (objects.size() == 0) return false;
//   Object* o = objects[objects.size() - 1];
//   if (!removeObject(o)) return false;
//   delete o;
//   return true;
//}   // removeObject()


// Attempts to remove an object from the environment,
// storing the address of the removed object and
// returning true if successful, false otherwise.
//bool Environment::removeObject(Object *o)
//{
//   if (o == NULL) return false;
//   for (unsigned i = 0; i < objects.size(); i++)
//   {
//      if (o == objects[i])
//      {
//         objects.erase(objects.begin() + i);
//         return true;
//      }
//   }
//   return false;
//}  // removeObject(Object *)


// Returns the cell at the parameterized position.
Cell* Environment::getCell(int position) const
{
  Cell *cell = NULL;
  for(unsigned i = 0; i < cells.size(); i++)
  {
    if(position == cells[i]->getID())
    {
      cell = cells[i];
      break;
    }
  }
  return cell;
}


// Returns the object at the parameterized index.
//Object* Environment::getObject(int index) const
//{
//   if ((index < 0) || (index >= objects.size())) return NULL;
//   return objects[index];
//}  // getObject(int) const


// Returns all of the cells in the environment.
vector<Cell *> Environment::getCells()
{
  return cells;
} 


// Returns all of the [free] robots in the environment.
vector<Robot *> Environment::getRobots()
{
  return robots;
}

// Returns all of the objects in the environment.
//vector<Object *> Environment::getObjects()
//{
//  return objects;
//}   // getObjects()


// Returns the number of cells in the environment.
int Environment::getNumberOfCells() const
{
  return cells.size();
}


// Returns the number of [free] robots in the environment.
int Environment::getNumberOfFreeRobots() const
{
  return robots.size();
}


// Returns the number of objects in the environment.
int Environment::getNObjects() const
{
  return objects.size();
}


// Returns the relationship between the two cells
// with the parameterized ID's.
Vector Environment::getRelationship(const int toID, const int fromID)
{
  Cell  *toCell = getCell(toID), *fromCell = getCell(fromID);

  if ((toCell == NULL) || (fromCell == NULL))
	  return Vector();

  Vector temp   = *toCell - *fromCell;
  temp.rotateRelative(-fromCell->getHeading());

  return temp;
}


void Environment::getCentroid(Vector v)
{
  centroid = v;
}//getCentroid( Vector v)

// void getRadius(int rad)
// last modified: 07-01-10

void Environment::getRadius(float rad)
{
  radius = rad;
}

void Environment::getDistance(Vector dist)
{
  distance = dist;

  //send a formation change message for the cell at dist+centroid to be seed
  for (int i = 0; i < getNumberOfCells(); i++)
  {
    if( ((distance.magnitude()) - (cells[i]->gradient).magnitude()) < 0.01)
    {
      cout << "cell id: " << cells[i]->ID << " is the new seed" << endl;
      Formation formationCopy = formation;
      formationCopy.setSeedID(cells[i]->ID);
      formationCopy.setFormationID(++formationID);
      changeFormation(formationCopy);
      break;
    }
  }
}


// Attempts to send a packet to its destination
// based upon the given parameters, returning
// true if successful, false otherwise.
bool Environment::sendMsg(const Message &msg,
    const int    toID,
    const int    fromID,
    const int    type)
{
  Message msg_c = msg;
  bool answer = sendPacket(Packet(msg_c, toID, fromID, type));
  //if(VERBOSE)printf("Received sendPacket answer\n");
  return answer;
}


// Attempts to send a packet to its destination,
// returning true if successful, false otherwise.
bool Environment::sendPacket(const Packet &packet)
{
  // discrete message passing
  //if (msgQueue.enqueue(p)) return true;

  // continuous message passing
  //if(p.type==AUCTION_ANNOUNCEMENT)printf("auctionannouncement in sendPacket()\n");
  if (forwardPacket(packet)) return true;
  //if(VERBOSE)printf("just before delete p.msg;\n");
  //(cast as message type -- delete (state *)
  if(packet.msg != NULL)
  {
    if(packet.type==STATE)
    {
      //if(VERBOSE) printf("attempting to delete STATE message\n");
      delete (State *)packet.msg;
      //if(VERBOSE) printf("successfully deleted STATE message\n");
    }else if(packet.type==CHANGE_FORMATION)
    {
      //if(VERBOSE) printf("attempting to delete CHANGE_FORMATION message\n");
      delete (Formation *)packet.msg;
    }
  }
  //p.msg = NULL;
  if(VERBOSE)
	  cout << "finished sendPacket()\n";
  return false;
}


// Attempts to forward a packet to its destination,
// returning true if successful, false otherwise.
bool Environment::forwardPacket(const Packet &packet)
{
  Cell *cell;
  if(!packet.fromBroadcast())
	  cell = getCell(packet.toID);

  int to = packet.toID;
  if (cell != NULL)
  {
    cell->msgQueue.push(packet);
    return true;
  }
  //if (c == NULL) printf("CELL is NULL!\n");
  //if p.msg != NULL
  //if(p.msg != NULL) delete p.msg;
  if(packet.msg != NULL)
  {
    if(packet.type==STATE)
    	delete (State *)packet.msg;

    else if(packet.type==CHANGE_FORMATION)
    	delete (Formation *)packet.msg;

    else if(packet.type == AUCTION_ANNOUNCEMENT)
    {
      //Robot* r;
      for(unsigned i = 0; i < robots.size(); i++)
    	  robots[i]->msgQueue.push(packet);
    }

    else if(packet.type == NCELL_REQUEST)
    {
    	// WTF?
    }
  }

  if(VERBOSE)
	  cout << "finished forwarding Packet to %d\n" << to;
  return false;
}


// Attempts to forward all packets to their destinations,
// returning true if successful, false otherwise.
bool Environment::forwardPackets()
{
  Packet packet;
  while (!msgQueue.empty())
  {
    packet = msgQueue.front();
    msgQueue.pop();
    if (!forwardPacket(packet)) return false;
  }
  return true;
}


// Returns the relationships between the robot with
// parameterized ID and all objects in the environment.
vector<Vector> Environment::getObjectRelationships(const int   fromID,
                                                   const float maxDistance)
{
  Cell *fromCell = getCell(fromID);
  int numberOfObjects = getNObjects();
  if ((fromCell == NULL) || (numberOfObjects == 0)) return vector<Vector>();

  vector<Vector> relationships;
  for (int i = 0; i < numberOfObjects; i++)
  {
	Vector relationship = *objects[i] - *fromCell;
	if (relationship.magnitude() <= maxDistance)
		relationships.push_back(relationship);
  }
  return relationships;
}


// Attempts to display the line of the heading vector of each cell,
// returning true if successful, false otherwise.
bool Environment::showLine(const bool show)
{
  for (int i = 0; i < getNumberOfCells(); i++)
	  cells[i]->heading.showLine = show;

  return true;
}


// Attempts to display the head of the heading vector of each cell,
// returning true if successful, false otherwise.
bool Environment::showHead(const bool show)
{
  for (int i = 0; i < getNumberOfCells(); i++)
	  cells[i]->heading.showHead = show;

  return true;
}


// Attempts to display the position vector of each cell,
// returning true if successful, false otherwise.
bool Environment::showPos(const bool show)
{
  for (int i = 0; i < getNumberOfCells(); i++)
	  cells[i]->showPos = show;

  return true;
}


// Attempts to display the heading vector of each cell,
// returning true if successful, false otherwise.
bool Environment::showHeading(const bool show)
{
  for (int i = 0; i < getNumberOfCells(); i++)
	  cells[i]->showHeading = show;

  return true;
}


bool Environment::addRobot(float x, float y, float z, float theta)
{
  if (VERBOSE)
	  cout << "new Robot(x = %.2f, y = %.2f, z = %.2f, theta = %.2f)\n" <<  x << y << z << theta;
  Robot *robot = new Robot(x, y, z, theta);
  robot-> setEnvironment(this);

  robots.push_back(robot);
  return true;
}


Robot * Environment::getNearestRobot(Cell *cell)
{
	Robot *robot = robots[0];
	float minDistance = distanceToRobot(cell,robots[0]), distance;

	for (int i = 0; i < getNumberOfFreeRobots(); i++)
	{
		distance = distanceToRobot(cell,robots[i]);
		if(minDistance > distance)
		{
		  robot = robots[i];
		  minDistance = distance;
		}
	}
	//printf("minDistance = %f\n",minDistance);
	return robot;
}

Robot * Environment::getNearestRobot(float x, float y)
{
	Robot *robot = robots[0];
	float minDistance = distanceToRobot(x,y,robots[0]), distance;
	distance = minDistance;

	for (int i = 0; i < getNumberOfFreeRobots(); i++)
	{
		distance = distanceToRobot(x,y,robots[i]);
		if(minDistance> distance)
		{
		  robot = robots[i];
		  minDistance = distance;
		}
	}
	//printf("minDistance = %f\n",minDistance);
	return robot;
}


float Environment::distanceToRobot(Cell *cell,Robot *robot)
{
  float x = 0, y = 0;
  x = fabs(robot-> x - cell-> x);
  y = fabs(robot-> y - cell-> y);
  return sqrt(pow(x, 2) + pow(y, 2));
}

float Environment::distanceToRobot(float xx,float yy, Robot* robot)
{
  float x = 0, y = 0;
  x = fabs(robot-> x - xx);
  y = fabs(robot-> y - yy);
  return sqrt(pow(x, 2) + pow(y, 2));
}

/*bool Environment::auctionPosition(Cell * a)
  {
  if(robots.getSize()>0)
  {
  Cell* c;
  c = new Cell();
  addCell(c);
  Robot *r = getNearestRobot(a);
  Robot *rr;
  c->x = r->x;
  c->y = r->y;
  for(int ii=0;ii< robots.getSize();ii++)
  {
  robots.getHead(rr);
  if(r==rr)
  {
  robots.removeHead();
  break;
  }else{
  ++robots;
  }
  }
  c->setColor(255,0,0);
  c->clearNbrs();
  c->addNbr(a->getID());
  a->addNbr(c->getID());

  if(a->rightNbr == NULL)
  {
  c->leftNbr  = c->nbrWithID(a->getID());
  a->rightNbr = a->nbrWithID(c->getID());
  newestCell  = c;
  }
  else if(a->leftNbr == NULL)
  {
  c->rightNbr = c->nbrWithID(a->getID());
  a->leftNbr  = a->nbrWithID(c->getID());
  newestCell  = c;
  }
  formation.setFormationID(++formationID);
  sendMsg(new Formation(formation),
  formation.getSeedID(),
  ID_OPERATOR,
  CHANGE_FORMATION);
  }
  return true;
  }*/

void Environment::formUp()
{

}

//get the nearest robot, turn it into a cell, and start the formation
void Environment::formFromClick(float x, float y)
{
  addCell();
  Cell *cell  = cells[0];
  Robot* robot = getNearestRobot(x,y);
  cell-> x = robot-> x;
  cell-> y = robot-> y;

  for(unsigned i = 0; i < robots.size(); i++)
  {
    if(robot == robots[i])
    {
      robots.erase(robots.begin() + i);
      break;
    }
  }

  //c->setColor(MAGENTA);
  cell->setHeading(formation.getHeading());
  newestCell = cell;
  sendMsg(new Formation(formation), formation.getSeedID(),
      ID_OPERATOR,      CHANGE_FORMATION);
}

//used by Simulator object to pass user formation change commands to the formation
bool Environment::changeFormation(Formation &f)
{
  formation = f;
  return sendMsg(&formation, formation.getSeedID(), ID_OPERATOR, CHANGE_FORMATION);
}
bool Environment::changeFormationSeed(Formation &f, int id)
{
  formation = f;
  return sendMsg(&formation, id, ID_OPERATOR,CHANGE_FORMATION);
}

//return a pointer to the robot with the matching id
Robot* Environment::getRobot(int id)
{
	// TODO catch possible null pointer
  Robot* robot;
  int rID=id;

  for(unsigned i = 0; i < robots.size(); i++)
  {
    if(robots[i]->getID()== rID)
      robot = robots[i];
  }
  return robot;
}

void Environment::settleAuction(Cell* auctionCell,int rID)
{
  //cout << "entering env->settleAuction()" << endl;

  if(robots.size() > 0)
  {
    Cell* cell;
    cell = new Cell();
    if(!addCell(cell))
    {
      cout << "addCell() failed!" << endl;
      system("PAUSE");
    }

    Robot *robot = getRobot(rID);
    if (robot == NULL)
    {
      cout << ">> ERROR: Robot[" << rID << "] not found!\n\n";
      return;
    }
    //cout <<"Robot should be "<< rID << " but env->settleAuction() is # "<< r->getID()<<endl;

    cell->x = robot->x;
    cell->y = robot->y;
    for(unsigned i = 0; i < robots.size(); i++)
    {
      if(robot->getID() == robots[i]->getID())
      {
        robots.erase(robots.begin() + i);
        break;
      }
    }
    //c->setColor(MAGENTA);
    cell->clearNbrs();
    cell->leftNbr = cell->rightNbr = NULL;
    cell->addNbr(auctionCell->getID());
    auctionCell->addNbr(cell->getID());

    if(auctionCell->rightNbr == NULL)
    {
      cell->leftNbr  = cell->nbrWithID(auctionCell->getID());
      auctionCell->rightNbr = auctionCell->nbrWithID(cell->getID());
      //cout << "a->rightNbr = " << a->rightNbr->ID << endl;
      newestCell  = cell;
    }
    else if(auctionCell->leftNbr == NULL)
    {
      cell->rightNbr = cell->nbrWithID(auctionCell->getID());
      auctionCell->leftNbr  = auctionCell->nbrWithID(cell->getID());
      //cout << "a->leftNbr = " << a->leftNbr->ID << endl;
      newestCell  = cell;
    }
    formation.setFormationID(++formationID);
    sendMsg(new Formation(formation),
        formation.getSeedID(),
        ID_OPERATOR,
        CHANGE_FORMATION);
    //getCell(formation.getSeedID())->sendStateToNbrs();
    //system("PAUSE");
  }
}


// Attempts to remove a robot from the environment,
// returning true if successful, false otherwise.
//
bool Environment::removeRobot()
{
  if (robots.size() == 0)
	  return false;

  Robot *robot = robots[robots.size()-1];
  if (!removeRobot(robot)) return false;
  	  delete robot;

  return true;
}   // removeRobot()



// remove a robot form the environment
bool Environment::removeRobot(Robot *robot)
{
  for(unsigned i = 0; i < robots.size(); i++)
  {
    if(robot == robots[i])
    {
      robots.erase(robots.begin() + i);
      return true;
    }
  }	
  return false;
}   // removeRobot(Robot *)



//
vector<Cell *> Environment::getCellVector()
{
  return cells;
}   // getCellVector()



//
vector<Robot *> Environment::getRobotVector()
{
  return robots;
}   // getRobotVector()

