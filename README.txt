To run the NewSimulator, do the following in this order:

roslaunch NewSimulator simulation.launch
rosrun NewSimulator Simulator
rosrun stage stageros `rospack find stage_controllers`/world/distrobo-world-7-robots.world

*If running ROS Fuerte
	rosrun NewSimulator Simulator
	roslaunch NewSimulator newSimulation.launch
	rosrun stage stageros `rospack find stage_controllers`/world/NewWorld.world
	

*This has to have been built before it will work or there are no binaries to run against


Make sure you do not commit the msg_gen/lisp and srv_gen/lisp folders created when you rosmake


To get the testing running
1)Download gtest
2)Unpack gtest
3)Run through the readme in gtest and setup gtest
4)To run the tests roscd into robot_driver and run 'make test'