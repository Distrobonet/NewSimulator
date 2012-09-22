To run the NewSimulator, do the following in this order:

roscore											
rosrun rviz rviz								// This runs the rviz environment
roslaunch fake_sphero multi_sphero.launch		// This creates all the spheros with which we interface
roslaunch NewSimulator simulation.launch		// This creates our Cell nodes, which interface with the spheros
rosrun NewSimulator Simulator					// This is our ui with which we change the formation



*This has to have been built before it will work or there are no binaries to run against


Make sure you do not commit the msg_gen/lisp and srv_gen/lisp folders created when you rosmake


To get the testing running
1)Download gtest
2)Unpack gtest
3)Run through the readme in gtest and setup gtest
4)To run the tests roscd into robot_driver and run 'make test'