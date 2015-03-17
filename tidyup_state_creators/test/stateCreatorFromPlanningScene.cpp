#include <gtest/gtest.h>
#include <climits>
#include <ros/ros.h>
#include "tidyup_state_creators/stateCreatorFromPlanningScene.h"
#include <shape_msgs/SolidPrimitive.h>
#include <shape_tools/solid_primitive_dims.h>

namespace tidyup_state_creators {

class stateCreatorFromPlanningSceneTest : public ::testing::Test {
	protected:
	stateCreatorFromPlanningSceneTest() :
		tableName_("table1_room1")
	{
		ros::NodeHandle nh;
		pub_co_ = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

		state_.addObject(tableName_, "table");
		state_.setNumericalFluent("timestamp", tableName_, ros::Time::now().toSec());
	    state_.addObject("/map", "frameid");
	    state_.setObjectFluent("frame-id", tableName_, "/map");
	    state_.setNumericalFluent("x", tableName_, 0.50);
	    state_.setNumericalFluent("y", tableName_, -0.75);
	    state_.setNumericalFluent("z", tableName_, 0.58);
	    state_.setNumericalFluent("qx", tableName_, 0);
	    state_.setNumericalFluent("qy", tableName_, 0.28);
	    state_.setNumericalFluent("qz", tableName_, 0);
	    state_.setNumericalFluent("qw", tableName_, 1);

		coTable_.header.frame_id = "/map";
		header_.stamp = ros::Time::now();
		coTable_.header.stamp = header_.stamp;
		coTable_.id = tableName_;
		coTable_.primitives.resize(1);
		coTable_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		coTable_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		coTable_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
		coTable_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.0;
		coTable_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
		geometry_msgs::Pose p;
		p.position.x = 0.50;
		p.position.y = -0.75;
		p.position.z = 0.58;
		p.orientation.x = 0;
		p.orientation.y = 0.28;
		p.orientation.z = 0;
		p.orientation.w = 1;
		coTable_.primitive_poses.push_back(p);
//		coTable_.operation = coTable_.ADD;
//		pub_co_.publish(coTable_);
	}

	// virtual void SetUp() {}
	// virtual void TearDown() {}
	SymbolicState state_;
	const std::string tableName_;
	moveit_msgs::CollisionObject coTable_;
	std_msgs::Header header_;
	moveit_msgs::CollisionObject coObject_;
	ros::Publisher pub_co_;
};

TEST_F(stateCreatorFromPlanningSceneTest, addObjectToState){

	StateCreatorFromPlanningScene scfps;

    std::string objectType = "table";

	// Test 1: if object is added to state
    {
    	state_.clear();
		scfps.addObjectToState(state_, coTable_, objectType);
		Predicate pred;
		pred.parameters.push_back(coTable_.id);
		pred.name = "x";
		double val;
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(0.50, val);
		pred.name = "y";
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_NE(-0.74, val);
		pred.name = "qy";
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(0.28, val);
		pred.name = "qw";
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(1, val);
    }

    // Test 2: object containing wrong frame should not be added to state
    //			Aborted by ROS_ASSERT
//    {
//    	state_.clear();
//    	coTable_.header.frame_id = "/base_link";
//    	//pub_co_.publish(coTable_);
//    	::testing::FLAGS_gtest_death_test_style = "threadsafe";
//    	EXPECT_DEATH_IF_SUPPORTED(scfps.addObjectToState(state_, coTable_, objectType), "");
//    }

    // Test 3: add object without pose
    {
    	state_.clear();
    	moveit_msgs::CollisionObject co = coTable_;
    	co.primitive_poses.clear();
    	co.header.frame_id = "/map";
    	//pub_co_.publish(coTable_);
		Predicate pred;
		pred.parameters.push_back(co.id);
		pred.name = "x";
		double val = -1;
		scfps.addObjectToState(state_, co, objectType);
		EXPECT_FALSE(state_.hasNumericalFluent(pred, &val));
		// verify that value has not been changed
		EXPECT_EQ(-1, val);
    }

    // Test 4: add object with different pose - test if state is updated
    {
    	state_.clear();
    	scfps.addObjectToState(state_, coTable_, objectType);
    	moveit_msgs::CollisionObject co = coTable_;
    	co.primitive_poses[0].position.x = -2.0;
    	scfps.addObjectToState(state_, co, objectType);
		Predicate pred;
		pred.parameters.push_back(co.id);
		pred.name = "x";
		double val;
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(-2.0, val);

    }
}

TEST_F(stateCreatorFromPlanningSceneTest, checkIfTableInState)
{
	StateCreatorFromPlanningScene scfps;
	// test return value if tableName is found in state
	EXPECT_TRUE(scfps.checkIfTableInState(state_, tableName_));
	// test return value if tableName does not exist in state
	EXPECT_FALSE(scfps.checkIfTableInState(state_, "notTable"));
	// test return value when tableName is empty
	EXPECT_FALSE(scfps.checkIfTableInState(state_, ""));
	// test return value when state is empty
	state_.clear();
	EXPECT_FALSE(scfps.checkIfTableInState(state_, tableName_));
}

TEST_F(stateCreatorFromPlanningSceneTest, extractPoseStampedFromSymbolicState)
{
	StateCreatorFromPlanningScene scfps;
	// Test 1: verify that the correct pose is returned
	geometry_msgs::PoseStamped pose;
	EXPECT_TRUE(scfps.extractPoseStampedFromSymbolicState(state_, coTable_.id, pose));
	EXPECT_EQ(header_.stamp.sec, pose.header.stamp.sec);
	EXPECT_EQ(0.5, pose.pose.position.x);
	EXPECT_EQ(-0.75, pose.pose.position.y);
	EXPECT_EQ(0.58, pose.pose.position.z);
	EXPECT_EQ(0, pose.pose.orientation.x);
	EXPECT_EQ(0.28, pose.pose.orientation.y);
	EXPECT_EQ(0, pose.pose.orientation.z);
	EXPECT_EQ(1, pose.pose.orientation.w);

	// Test 2: verify return value if object does not exist in state
	EXPECT_FALSE(scfps.extractPoseStampedFromSymbolicState(state_, "notExisting", pose));
}

TEST_F(stateCreatorFromPlanningSceneTest, extractPoseStampedFromCollisionObject)
{
	StateCreatorFromPlanningScene scfps;
	// Test 1: verify that the correct pose is returned
	geometry_msgs::PoseStamped pose;
	EXPECT_TRUE(scfps.extractPoseStampedFromCollisionObject(coTable_, pose));
	EXPECT_EQ(0.5, pose.pose.position.x);
	EXPECT_EQ(-0.75, pose.pose.position.y);
	EXPECT_EQ(0.58, pose.pose.position.z);
	EXPECT_EQ(0, pose.pose.orientation.x);
	EXPECT_EQ(0.28, pose.pose.orientation.y);
	EXPECT_EQ(0, pose.pose.orientation.z);
	EXPECT_EQ(1, pose.pose.orientation.w);

	// Test 2: verify that false is returned when co does not have a pose
	moveit_msgs::CollisionObject co = coTable_;
	co.primitive_poses.clear();
	EXPECT_FALSE(scfps.extractPoseStampedFromCollisionObject(co, pose));
}

TEST_F(stateCreatorFromPlanningSceneTest, distanceBetweenTwoPoses)
{
	StateCreatorFromPlanningScene scfps;
	geometry_msgs::PoseStamped a, b;
	a.header.frame_id = "/map";
	a.header.stamp = ros::Time::now();
	geometry_msgs::Pose pose;
	pose.position.x = 1.0;
	pose.position.y = 1.0;
	pose.position.z = 1.0;
	pose.orientation.w = 1.0;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	a.pose = pose;

	b.header.frame_id = "/map";
	b.header.stamp = ros::Time::now();
	pose.position.x = 2.0;
	pose.position.y = 1.0;
	pose.position.z = 0;
	b.pose = pose;

	// Test 1: simple distance calc where the distance between the points is 1
	std::pair<double, double> result;
	result = scfps.distanceBetweenTwoPoses(a, b);
	double dist, height;
	dist = result.first;
	height = result.second;
	// Computed distance between 2 points: A (1, 1, 1) and B(2, 1, 0)
	EXPECT_EQ(1, dist);
	EXPECT_EQ(1, height);

	// Test 2: distance gives no exact result (sqrt(5))
	b.pose.position.y = 3.0;
	b.pose.position.z = 3.0;
	result = scfps.distanceBetweenTwoPoses(a, b);
	dist = result.first;
	height = result.second;
	// Computed distance between 2 points: A (1, 1, 1) and B(2, 3, 0)
	// SQRT( (xB - xA)^2 + (yB - yA)^2) = SQRT( (2-1)^2 + (3-1)^2 )
	EXPECT_EQ(std::sqrt(5), dist);
	EXPECT_EQ(2, height);

	// TODO: Test 3: orientation of poses changes, compute dist
//	a.pose.orientation.x = 0.33;
//	a.pose.orientation.y = 0.67;
//	a.pose.orientation.z = 0.73;
//
//	b.pose.orientation.x = 0.45;
//	b.pose.orientation.y = 0.82;
//	b.pose.orientation.z = 0.12;
//	result = scfps.distanceBetweenTwoPoses(a, b);
//	dist = result.first;
//	height = result.second;
//	// Computed distance between 2 points: A (1, 1, 1) and B(2, 3, 0)
//	// SQRT( (xB - xA)^2 + (yB - yA)^2) = SQRT( (2-1)^2 + (3-1)^2 )
//	EXPECT_EQ(std::sqrt(5), dist);
//	EXPECT_EQ(2, height);
}

};

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // do not forget to init ros because this is also a node
    ros::init(argc, argv, "stateCreatorFromPlanningSceneTest");
    return RUN_ALL_TESTS();
}
