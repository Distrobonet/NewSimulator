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

const char  CHAR_ESCAPE = char(27);    // 'ESCAPE' character key
int LAST_SELECTION = -1;
int CURRENT_SELECTION = -1;
int formationCount = 0;

NewSimulator::FormationMessage formationMessage;

// A formation is a vector of Functions, which are functions that take floats and return floats
Formation DEFAULT_FORMATION = Formation();// = Formation(line, 1, PhysicsVector(), MIDDLE_CELL, 0,  90.0f);

// Service utility function to set the formation being served based on CURRENT_SELECTION
void setFormationMessage()
{
	formationMessage.seed_id = DEFAULT_FORMATION.getSeedID();
	formationMessage.formation_id = CURRENT_SELECTION;
	formationMessage.formation_count = formationCount;
//	ROS_INFO("sending back response with formation info");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simulator");

	displayMenu();

	// Only continue program once a valid selection has been made
	while(CURRENT_SELECTION < 0 || CURRENT_SELECTION > 9)
	{
		keyboardInput();
	}

	// Formation publisher to the seed cell
	ros::init(argc, argv, "formation_publisher");
	ros::NodeHandle formationPublisherNode;
	ros::Publisher formationPublisher = formationPublisherNode.advertise<NewSimulator::FormationMessage>("seedFormationMessage", 1000);

	cout << "\nNow publishing formations.  Current formation = " << CURRENT_SELECTION << endl;
	ros::spinOnce();


	// Create handler for interrupts (i.e., ^C)
	if (signal(SIGINT, SIG_IGN) != SIG_IGN) signal(SIGINT, terminate);
	signal(SIGPIPE, SIG_IGN);

	// Simulator infinite loop.
	while(ros::ok)
	{
		ros::spinOnce();

		keyboardInput();

		// Selection has changed, push this new formation to the seed cell
		if(CURRENT_SELECTION != LAST_SELECTION)
		{
			formationCount += 1;
//			cout << "\nformationCount: " << formationCount << " - New formation: " << CURRENT_SELECTION << endl;
			LAST_SELECTION = CURRENT_SELECTION;
			setFormationMessage();
			formationPublisher.publish(formationMessage);
		}
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

		//int keyNum = atoi(&keyPressed);
		cout << "\nKey pressed: " << keyPressed;

		if(keyPressed >= '0' && keyPressed <= '9')
		{
			CURRENT_SELECTION = keyPressed-48;	// convert from ascii char to int
			cout << " - Setting CURRENT_SELECTION to " << CURRENT_SELECTION <<endl;
		}
		else
			cout << " - Not a valid input.";

		displayMenu();
	}

}


// Displays the selection menu to the screen
void displayMenu()
{
	clearScreen();

	cout << endl << endl << "Use the '0-9' keys to "
		<< "change to a formation seeded at the selected robot."
		<< endl << endl
		<< "PRESET FORMATIONS\n-----------------"            << endl
		<< "0) f(x) = 0"                                     << endl
		<< "1) f(x) = x"                                     << endl
		<< "2) f(x) = |x|"                                   << endl
		<< "3) f(x) = -0.5 x"                                << endl
		<< "4) f(x) = -|0.5 x|"                              << endl
		<< "5) f(x) = -|x|"                                  << endl
		<< "6) f(x) = x^2"                                   << endl
		<< "7) f(x) = x^3"                                   << endl
		<< "8) f(x) = {sqrt(x),  x >= 0 | -sqrt|x|, x < 0}"  << endl
		<< "9) f(x) = 0.05 sin(10 x)"                        << endl << endl
		<< "Use ctrl+C to exit."                                << endl << endl
		<< "Please enter your selection: ";
}

// A simple and basic way to clear the screen for the menu refresh
void clearScreen()
{
	std::cout << "\n\n\n\n\n";
}

// Terminates the program on interrupt (i.e., ^C).
void terminate(int retVal)
{
  signal(SIGINT, SIG_IGN);
  signal(SIGINT, SIG_DFL);
  exit(retVal);
}

