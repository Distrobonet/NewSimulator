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




It is suggested that you replace your multi-sphero.launch file (contained at ROS_DIRECTORY/sphero/fake_sphero/launch) with this for easier testing...

<launch>

    <!-- Namespace -->
    <arg name="ns" default="sphero" />

    <!-- Sphero #0 -->
    <include file="$(find fake_sphero)/launch/sphero.launch">
      <arg name="name" value="$(arg ns)0" />
      <arg name="px" value="-8.0" />
      <arg name="py" value="-5.0" />
      <arg name="r" value="1.0" />
      <arg name="g" value="1.0" />
      <arg name="b" value="1.0" />
    </include>

    <!-- Sphero #1 -->
    <include file="$(find fake_sphero)/launch/sphero.launch">
      <arg name="name" value="$(arg ns)1" />
      <arg name="px" value="-2.0" />
      <arg name="py" value="-5.0" />
      <arg name="r" value="1.0" />
      <arg name="g" value="1.0" />
      <arg name="b" value="0.0" />
    </include>

    <!-- Sphero #2 -->
    <include file="$(find fake_sphero)/launch/sphero.launch">
      <arg name="name" value="$(arg ns)2" />
      <arg name="px" value="0.0" />
      <arg name="py" value="-5.0" />
      <arg name="r" value="1.0" />
      <arg name="g" value="0.0" />
      <arg name="b" value="1.0" />
    </include>

    <!-- Sphero #3 -->
    <include file="$(find fake_sphero)/launch/sphero.launch">
      <arg name="name" value="$(arg ns)3" />
      <arg name="px" value="0.0" />
      <arg name="py" value="0.0" />
      <arg name="r" value="1.0" />
      <arg name="g" value="0.0" />
      <arg name="b" value="0.0" />
    </include>

    <!-- Sphero #4 -->
    <include file="$(find fake_sphero)/launch/sphero.launch">
      <arg name="name" value="$(arg ns)4" />
      <arg name="px" value="4.0" />
      <arg name="py" value="-8.0" />
      <arg name="r" value="0.0" />
      <arg name="g" value="1.0" />
      <arg name="b" value="1.0" />
    </include>

    <!-- Sphero #5 -->
    <include file="$(find fake_sphero)/launch/sphero.launch">
      <arg name="name" value="$(arg ns)5" />
      <arg name="px" value="5.0" />
      <arg name="py" value="-2.0" />
      <arg name="r" value="0.0" />
      <arg name="g" value="1.0" />
      <arg name="b" value="0.0" />
    </include>

    <!-- Sphero #6 -->
    <include file="$(find fake_sphero)/launch/sphero.launch">
      <arg name="name" value="$(arg ns)6" />
      <arg name="px" value="8.0" />
      <arg name="py" value="-2.0" />
      <arg name="r" value="0.0" />
      <arg name="g" value="0.0" />
      <arg name="b" value="1.0" />
    </include>
</launch>
