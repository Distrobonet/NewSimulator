//------------------------------------------------------------------
// Description:     This program tests the robot cell simulator.
//------------------------------------------------------------------

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

// Used for non-blocking user input
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include "Simulator/Formation.h"
#include "Simulator/PhysicsVector.h"

// Formation publisher - Simulator publishes the formation to the seed cell, the ID of which is
// determined by Formation.seedID (set to global variable in Formation for now)
#include "../msg_gen/cpp/include/NewSimulator/FormationMessage.h"

using namespace std;

// define SIGPIPE if not defined (compatibility for win32)
#ifndef SIGPIPE
#define SIGPIPE 13
#endif

// Prototypes
void terminate(int retVal);
void displayMenu();
void keyboardInput();
void clearScreen();

const char CHAR_ESCAPE = char(27);    			// 'ESCAPE' character key
int LAST_SELECTION = -1;
int CURRENT_SELECTION = -1;
int FORMATION_COUNT = 0;

PhysicsVector SEED_FRP(0,0,0);
float CELL_RADIUS = 1.0f;
float CELL_RADIUS_MAXIMUM = 16.0f;
float CELL_RADIUS_INCREMENT = 0.2f;
bool IS_RADIUS_CHANGED = false;
float SENSOR_ERROR = 0.0f;
bool IS_SENSOR_ERROR_CHANGED = false;
float SENSOR_ERROR_INCREMENT = 0.01f;
float COMMUNICATION_ERROR = 0.0f;				// Communication error means that x% of all ROS messages are lost
bool IS_COMMUNICATION_ERROR_CHANGED = false;
float COMMUNICATION_ERROR_INCREMENT = 1.0f;
int SEED_ID = 3;								// Can use this to change which cell is the seed



// Service utility function to set the formation being served based on CURRENT_SELECTION
NewSimulator::FormationMessage setFormationMessage()
{
	NewSimulator::FormationMessage formationMessage;

	formationMessage.seed_frp.x = SEED_FRP.x;
	formationMessage.seed_frp.y = SEED_FRP.y;
	formationMessage.seed_frp.z = SEED_FRP.z;
	formationMessage.seed_fro = 0.0f;
	formationMessage.radius = CELL_RADIUS;
	formationMessage.sensor_error = SENSOR_ERROR;
	formationMessage.communication_error = COMMUNICATION_ERROR;
	formationMessage.seed_id = SEED_ID;
	formationMessage.formation_id = CURRENT_SELECTION;
	formationMessage.formation_count = FORMATION_COUNT;
//	ROS_INFO("Setting the formation message");
	return formationMessage;
}

int main(int argc, char **argv)
{
	clearScreen();
	displayMenu();

	// Formation publisher to the seed cell
	ros::init(argc, argv, "formation_publisher");
	ros::NodeHandle formationPublisherNode;
	ros::Publisher formationPublisher = formationPublisherNode.advertise<NewSimulator::FormationMessage>("seedFormationMessage", 1000);

	ros::spinOnce();

	// Create handler for interrupts (i.e., ^C)
	if (signal(SIGINT, SIG_IGN) != SIG_IGN) signal(SIGINT, terminate);
	signal(SIGPIPE, SIG_IGN);

	// Simulator infinite loop.
	while(ros::ok)
	{
		ros::spinOnce();

		// Selection or radius or error has changed, push this new formation to the seed cell
		if(CURRENT_SELECTION != LAST_SELECTION || IS_RADIUS_CHANGED || IS_SENSOR_ERROR_CHANGED || IS_COMMUNICATION_ERROR_CHANGED)
		{
			FORMATION_COUNT += 1;
			LAST_SELECTION = CURRENT_SELECTION;
			IS_RADIUS_CHANGED = false;
			IS_SENSOR_ERROR_CHANGED = false;
			IS_COMMUNICATION_ERROR_CHANGED = false;

			formationPublisher.publish(setFormationMessage());
		}

		keyboardInput();
	}

  return 0;
}


// Used by keyboardInput() to catch keystrokes without blocking
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

// Catches keyboard input and sets CURRENT_SELECTION based on user input, redisplays the menu
void keyboardInput()
{
	char keyPressed;

	if(kbhit())
	{
		keyPressed=getchar();

		clearScreen();
		cout << "\nKey pressed: " << keyPressed;

		if(keyPressed >= '0' && keyPressed <= '9')
		{
			CURRENT_SELECTION = keyPressed-48;	// convert from ascii char to int
			cout << "   Setting CURRENT_SELECTION to " << CURRENT_SELECTION << endl;
		}
		else if(keyPressed == '+')
		{
			if(CELL_RADIUS < CELL_RADIUS_MAXIMUM - CELL_RADIUS_INCREMENT)
			{
				CELL_RADIUS += CELL_RADIUS_INCREMENT;
				IS_RADIUS_CHANGED = true;
				cout << "   Increasing cell radius to " << CELL_RADIUS << endl;
			}
			else
			{
				cout << "   Cannot increase cell radius over maximum value " << CELL_RADIUS_MAXIMUM << endl;
			}
		}
		else if(keyPressed == '-')
		{
			if(CELL_RADIUS >= 0.4f)
			{
				CELL_RADIUS -= CELL_RADIUS_INCREMENT;
				IS_RADIUS_CHANGED = true;
				cout << "   Decreasing cell radius to " << CELL_RADIUS << endl;
			}
			else
				cout << "   Can not decrease the radius any more!\n";
		}
		else if(keyPressed == 'd')
		{
			SENSOR_ERROR += SENSOR_ERROR_INCREMENT;
			IS_SENSOR_ERROR_CHANGED = true;
			cout << "   Increasing sensor error to " << SENSOR_ERROR << endl;
		}
		else if(keyPressed == 's')
		{
			if(SENSOR_ERROR >= SENSOR_ERROR_INCREMENT)
			{
				SENSOR_ERROR -= SENSOR_ERROR_INCREMENT;
				IS_SENSOR_ERROR_CHANGED = true;
				cout << "   Decreasing sensor error to " << SENSOR_ERROR << endl;
			}
			else
			{
				SENSOR_ERROR = 0.0f;
				cout << "   Can not decrease the sensor error any more!\n";
			}
		}
		else if(keyPressed == 'v')
		{
			if(COMMUNICATION_ERROR == 100.0f)
				cout << "   Can not increase the communication error over 100%!\n";
			else
			{
				COMMUNICATION_ERROR += COMMUNICATION_ERROR_INCREMENT;
				IS_COMMUNICATION_ERROR_CHANGED = true;
				cout << "   Increasing communication error to " << COMMUNICATION_ERROR << "%" << endl;
			}
		}
		else if(keyPressed == 'c')
		{
			if(COMMUNICATION_ERROR >= COMMUNICATION_ERROR_INCREMENT)
			{
				COMMUNICATION_ERROR -= COMMUNICATION_ERROR_INCREMENT;
				IS_COMMUNICATION_ERROR_CHANGED = true;
				cout << "   Decreasing communication error to " << COMMUNICATION_ERROR << "%" << endl;
			}
			else
			{
				COMMUNICATION_ERROR = 0.0f;
				cout << "   Can not decrease the communication error any more!\n";
			}
		}
		else
			cout << "   Not a valid input.";

		displayMenu();
	}
}

// Displays the selection menu to the screen
void displayMenu()
{
	cout << endl << "Use the '0-9' keys to "
		<< "change to a formation seeded at the selected robot."
		<< endl << endl
		<< "PRESET FORMATIONS\n-----------------"            << endl
		<< "0) f(x) = 0"                                     << endl
		<< "1) f(x) = x"                                     << endl
		<< "2) f(x) = |x|"                                   << endl
		<< "3) f(x) = -0.5 x"                                << endl
		<< "4) f(x) = -|0.5 x|"                              << endl
		<< "5) f(x) = -|x|"                                  << endl
		<< "6) f(x) = 5(x^2)"                                   << endl
		<< "7) f(x) = x^3"                                   << endl
		<< "8) f(x) = {sqrt(x),  x >= 0 | -sqrt|x|, x < 0}"  << endl
		<< "9) f(x) = sin(x)"                        		 << endl << endl
		<< "Use - and + to adjust the cell radius"    		 << endl
		<< "Use s and d to adjust the sensor error"    		 << endl
		<< "Use c and v to adjust the communication error (0 - 100% message loss)"	 << endl
		<< "Use ctrl+C to exit."                             << endl << endl
		<< "Please enter your selection: ";
}

// A simple and basic way to clear the screen for the menu refresh
void clearScreen()
{
	std::cout << "\n\n\n\n\n\n\n";
}

// Terminates the program on interrupt (i.e., ^C).
void terminate(int retVal)
{
  signal(SIGINT, SIG_IGN);
  signal(SIGINT, SIG_DFL);
  exit(retVal);
}
