#include<Simulator/Cell.h>

int main(int argc, char **argv)
{
	string ros_name;

	// Formation Service Client
	ros_name = "formation_client_cell_";
	ros_name.append(argv[1]);
    ros::init(argc, argv, ros_name);

    // State Service Client
    ros_name = "state_client_cell_";
    ros_name.append(argv[1]);
    ros::init(argc, argv, ros_name);

	// Relationship Service Client
    ros_name = "relationship_client_cell_";
    ros_name.append(argv[1]);
    ros::init(argc, argv, ros_name);

    // State publisher
    ros::init(argc, argv, "state");
    ros::NodeHandle state_pub;


    // Initilize cell, giving it the ID of the parameterized value
    Cell thisCell = Cell(atoi(argv[1]));

    thisCell.state_pub = thisCell.stateNode.advertise<NewSimulator::StateMessage>(thisCell.generateStateSubMessage(thisCell.getCellID()), 1);
    thisCell.cmd_velPub = thisCell.stateNode.advertise<geometry_msgs::Twist>(thisCell.generateCommandVelocityPubMessage(thisCell.getCellID()), 1);

	// Start the state service server for this cell
	ros_name = "state_server_";
	ros_name += thisCell.getCellID();
	ros::init(argc, argv, ros_name);
	thisCell.startStateServiceServer();

	// Update this cell
	//thisCell.update(argv[4]);
	thisCell.update();

	return 0;
}
