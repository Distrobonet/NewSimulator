//#include <gtest/gtest.h>
//#include <iostream>
//#include <stdio.h>
//#include <stdlib.h>
//#include <angles/angles.h>
//#include <nav_msgs/Odometry.h>
//#include "../src/RobotDriver.h"
//using namespace std;
//
//TEST(RobotDriver, initialTest){
//	int actual = 1;
//	EXPECT_EQ(1, actual);
//}
//
//TEST(RobotDriver, getDistance){
//	double target_x = 1;
//	double target_y = 1;
//	double origin_x = 0;
//	double origin_y = 0;
//	double actual = getDistance(target_x, target_y, origin_x, origin_y);
//	EXPECT_EQ(1.4142135623730950488016, actual);
//}
//
//TEST(RobotDriver, getDistanceWithNegNumbers){
//	double target_x = -1;
//	double target_y = -1;
//	double origin_x = -2;
//	double origin_y = -2;
//	double actual = getDistance(target_x, target_y, origin_x, origin_y);
//	EXPECT_EQ(1.4142135623730950488016, actual);
//}
//
//TEST(RobotDriver, getDistanceWithSameTagetAndOrigin){
//	double target_x = 1;
//	double target_y = 1;
//	double origin_x = 1;
//	double origin_y = 1;
//	double actual = getDistance(target_x, target_y, origin_x, origin_y);
//	EXPECT_EQ(0, actual);
//}
//
//TEST(RobotDriver, getAngle){
//	double target_x = 0;
//	double target_y = 0;
//	double origin_x = 1;
//	double origin_y = 1;
//	double theta = 1;
//	double actual = getAngle(target_x, target_y, origin_x, origin_y, theta);
//	EXPECT_EQ(2.9269908169872415480, actual);
//	//atan((0 - 1)/ (0- 1)) + pi - 1
//}
//
//TEST(RobotDriver, getAngleWithNegNumbers){
//	double target_x = -1;
//	double target_y = -1;
//	double origin_x = -2;
//	double origin_y = -2;
//	double theta = -1;
//	double actual = getAngle(target_x, target_y, origin_x, origin_y, theta);
//	EXPECT_EQ(1.785398163397448309615, actual);
//	//atan((-1 + 2)/ (-1 + 2)) + 1
//}
//
//TEST(RobotDriver, getYValue){
//	double xValue = 1;
//	string input = "1";
//	setFunctionSelection(input);
//	double actual = getYValue(xValue);
//	EXPECT_EQ(0.0l, actual);
//}
//
//TEST(RobotDriver, getYValueCase2){
//	double xValue = 1;
//	string input = "2";
//	setFunctionSelection(input);
//	double actual = getYValue(xValue);
//	EXPECT_EQ(1.0l, actual);
//}
//
//TEST(RobotDriver, getYValueCase3){
//	double xValue = -1;
//	string input = "3";
//	setFunctionSelection(input);
//	double actual = getYValue(xValue);
//	EXPECT_EQ(1.0l, actual);
//	xValue = 1;
//	actual = getYValue(xValue);
//	EXPECT_EQ(1.0l, actual);
//}
//
//TEST(RobotDriver, getYValueCase4){
//	double xValue = 1;
//	string input = "4";
//	setFunctionSelection(input);
//	double actual = getYValue(xValue);
//	EXPECT_EQ(-0.5l, actual);
//}
//
//TEST(RobotDriver, getYValueCase5){
//	double xValue = -1;
//	string input = "5";
//	setFunctionSelection(input);
//	double actual = getYValue(xValue);
//	EXPECT_EQ(-0.5l, actual);
//	xValue = 1;
//	actual = getYValue(xValue);
//	EXPECT_EQ(-0.5l, actual);
//}
//
//TEST(RobotDriver, getYValueCase6){
//	double xValue = -1;
//	string input = "6";
//	setFunctionSelection(input);
//	double actual = getYValue(xValue);
//	EXPECT_EQ(-1.0l, actual);
//	xValue = 1;
//	actual = getYValue(xValue);
//	EXPECT_EQ(-1.0l, actual);
//}
//
//TEST(RobotDriver, setFunctionSelection){
//	string input = "3";
//	setFunctionSelection(input);
//	double actual = CURRENT_SELECTION;
//	EXPECT_EQ(3, actual);
//}
//
//TEST(RobotDriver, setFunctionSelectionWithNegNumber){
//	string input = "-3";
//	setFunctionSelection(input);
//	double actual = CURRENT_SELECTION;
//	EXPECT_EQ(-3, actual);
//}
//
//TEST(RobotDriver, setFunctionSelectionWithNoValueDoesNotChangeSelection){
//	string input = "1";
//	setFunctionSelection(input);
//	input = "";
//	setFunctionSelection(input);
//	double actual = CURRENT_SELECTION;
//	EXPECT_EQ(1, actual);
//}
//
//
//
//int main(int argc, char **argv){
//	testing::InitGoogleTest(&argc, argv);
//	return RUN_ALL_TESTS();
//}
//
