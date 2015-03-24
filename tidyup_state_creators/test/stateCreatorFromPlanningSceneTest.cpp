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

		state_.addObject("table2_room1", "table");
		state_.setNumericalFluent("timestamp", tableName_, ros::Time::now().toSec());
	    state_.addObject("/map", "frameid");
	    state_.setObjectFluent("frame-id", tableName_, "/map");
	    state_.setNumericalFluent("x", tableName_, 2.50);
	    state_.setNumericalFluent("y", tableName_, -2.75);
	    state_.setNumericalFluent("z", tableName_, 0.58);
	    state_.setNumericalFluent("qx", tableName_, 0);
	    state_.setNumericalFluent("qy", tableName_, 0.28);
	    state_.setNumericalFluent("qz", tableName_, 0);
	    state_.setNumericalFluent("qw", tableName_, 1);

	    std::string location = "table1_loc1_room1";
	    state_.setBooleanPredicate("location-near-table", location + " " + tableName_, true);
        state_.setNumericalFluent("timestamp", location, ros::Time::now().toSec());
        state_.addObject("/map", "frameid");
        state_.setObjectFluent("frame-id", location, "/map");
        state_.setNumericalFluent("x", location, 0);
        state_.setNumericalFluent("y", location, -0.75);
        state_.setNumericalFluent("z", location, 0);
        state_.setNumericalFluent("qx", location, 0);
        state_.setNumericalFluent("qy", location, 0.28);
        state_.setNumericalFluent("qz", location, 0);
        state_.setNumericalFluent("qw", location, 0);

		coTable1_.header.frame_id = "/map";
		header_.stamp = ros::Time::now();
		coTable1_.header.stamp = header_.stamp;
		coTable1_.id = tableName_;
		coTable1_.primitives.resize(1);
		coTable1_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		coTable1_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		coTable1_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
		coTable1_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.0;
		coTable1_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
		geometry_msgs::Pose p;
		p.position.x = 0.50;
		p.position.y = -0.75;
		p.position.z = 0.58;
		p.orientation.x = 0;
		p.orientation.y = 0.28;
		p.orientation.z = 0;
		p.orientation.w = 1;
		coTable1_.primitive_poses.push_back(p);

		coTable2_.header.frame_id = "/map";
		header_.stamp = ros::Time::now();
		coTable2_.header.stamp = header_.stamp;
		coTable2_.id = "table2_room1";
		coTable2_.primitives.resize(1);
		coTable2_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		coTable2_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		coTable2_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
		coTable2_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.0;
		coTable2_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
		p.position.x = 2.50;
		p.position.y = -2.75;
		p.position.z = 0.58;
		p.orientation.x = 0;
		p.orientation.y = 0.28;
		p.orientation.z = 0;
		p.orientation.w = 1;
		coTable2_.primitive_poses.push_back(p);
//		coTable1_.operation = coTable1_.ADD;
//		pub_co_.publish(coTable1_);

		coObject_.header.frame_id = "/head_mount_kinect";
		coObject_.header.stamp = header_.stamp;
		coObject_.id = "cokeTest";
		coObject_.primitives.resize(1);
		coObject_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		coObject_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		coObject_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.067;
		coObject_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.067;
		coObject_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.12;
		p.position.x = 0.50;
		p.position.y = 0.25;
		p.position.z = 0.58;
		p.orientation.x = 0.12;
		p.orientation.y = 0.28;
		p.orientation.z = 0.89;
		p.orientation.w = 1;
		coObject_.primitive_poses.push_back(p);

		allCos.push_back(coTable1_);
		allCos.push_back(coTable2_);
		allCos.push_back(coObject_);
	}

	// virtual void SetUp() {}
	// virtual void TearDown() {}
	SymbolicState state_;
	const std::string tableName_;
	std_msgs::Header header_;
	moveit_msgs::CollisionObject coTable1_;
	moveit_msgs::CollisionObject coTable2_;
	moveit_msgs::CollisionObject coObject_;
	std::vector<moveit_msgs::CollisionObject> allCos;
	ros::Publisher pub_co_;
};

TEST_F(stateCreatorFromPlanningSceneTest, initializeTables)
{
	StateCreatorFromPlanningScene scfps;
	scfps.initializeTables(state_);

	// testing tables_
	EXPECT_EQ(2, scfps.tables_.size());
	EXPECT_TRUE(scfps.tables_.find("table1_room1") != scfps.tables_.end());
	EXPECT_TRUE(scfps.tables_.find("table2_room1") != scfps.tables_.end());

	EXPECT_TRUE(scfps.tables_.find("table3_room1") == scfps.tables_.end());

	// testing tableLocations_
//	EXPECT_EQ(1, scfps.tableLocations_.size());
//	EXPECT_TRUE(scfps.tableLocations_.find(std::make_pair("table1_room1", "table1_loc1_room1")) !=
//			scfps.tableLocations_.end());
//	EXPECT_TRUE(scfps.tableLocations_.find(std::make_pair("table1_room1", "table1_loc2")) ==
//			scfps.tableLocations_.end());

	// testing state
	// TODO: only if tableLocations is used
}

TEST_F(stateCreatorFromPlanningSceneTest, doesObjectTypeExist)
{
	StateCreatorFromPlanningScene scfps;

	EXPECT_TRUE(scfps.doesObjectTypeExist("pose"));
	EXPECT_TRUE(scfps.doesObjectTypeExist("frameid"));
	EXPECT_TRUE(scfps.doesObjectTypeExist("location"));
	EXPECT_TRUE(scfps.doesObjectTypeExist("manipulation_location"));
	EXPECT_TRUE(scfps.doesObjectTypeExist("table"));
	EXPECT_TRUE(scfps.doesObjectTypeExist("movable_object"));
	EXPECT_TRUE(scfps.doesObjectTypeExist("arm"));
	EXPECT_TRUE(scfps.doesObjectTypeExist("arm_state"));

	EXPECT_FALSE(scfps.doesObjectTypeExist("abc"));
	EXPECT_FALSE(scfps.doesObjectTypeExist(""));
}

TEST_F(stateCreatorFromPlanningSceneTest, addObjectToSymbolicState){

	StateCreatorFromPlanningScene scfps;

    std::string objectType = "table";

	// Test 1: if object is added to state
    {
    	state_.clear();
		scfps.addObjectToSymbolicState(state_, coTable1_, objectType);
		Predicate pred;
		pred.parameters.push_back(coTable1_.id);
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
//    	coTable1_.header.frame_id = "/base_link";
//    	//pub_co_.publish(coTable1_);
//    	::testing::FLAGS_gtest_death_test_style = "threadsafe";
//    	EXPECT_DEATH_IF_SUPPORTED(scfps.addObjectToSymbolicState(state_, coTable1_, objectType), "");
//    }

    // Test 3: add object without pose
    {
    	state_.clear();
    	moveit_msgs::CollisionObject co = coTable1_;
    	co.primitive_poses.clear();
    	co.header.frame_id = "/map";
    	//pub_co_.publish(coTable1_);
		Predicate pred;
		pred.parameters.push_back(co.id);
		pred.name = "x";
		double val = -1;
		scfps.addObjectToSymbolicState(state_, co, objectType);
		EXPECT_FALSE(state_.hasNumericalFluent(pred, &val));
		// verify that value has not been changed
		EXPECT_EQ(-1, val);
    }

    // Test 4: add object with different pose - test if state is updated
    {
    	state_.clear();
    	// first add object to state
    	scfps.addObjectToSymbolicState(state_, coTable1_, objectType);
    	// modify object
    	moveit_msgs::CollisionObject co = coTable1_;
    	co.primitive_poses[0].position.x = -2.0;
    	// re-add it
    	scfps.addObjectToSymbolicState(state_, co, objectType);
		Predicate pred;
		pred.parameters.push_back(co.id);
		pred.name = "x";
		double val;
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(-2.0, val);
    }

    // Test 5: add object coke
    {
    	state_.clear();
    	scfps.addObjectToSymbolicState(state_, coObject_, "movable_object");
		Predicate pred;
		pred.parameters.push_back(coObject_.id);
		pred.name = "x";
		double val;
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].position.x, val);
		pred.name = "y";
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].position.y, val);
		pred.name = "qy";
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].orientation.y, val);
		pred.name = "qw";
		EXPECT_TRUE(state_.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].orientation.w, val);
    }
}
TEST_F(stateCreatorFromPlanningSceneTest, findMatchingTable)
{
	StateCreatorFromPlanningScene scfps;
	// needs to be called first, to set membervariable tables_
	scfps.initializeTables(state_);

	scfps.findMatchingTable(state_, allCos, coObject_);
	Predicate p;
	p.name = "object-on";
	p.parameters.push_back(coObject_.id);
	p.parameters.push_back(coTable1_.id);
	bool value = false;
	EXPECT_TRUE(state_.hasBooleanPredicate(p, &value));

	p.parameters[0] = "NonExistingObject";
	EXPECT_FALSE(state_.hasBooleanPredicate(p, &value));
}


TEST_F(stateCreatorFromPlanningSceneTest, extractPoseStampedFromCollisionObject)
{
	StateCreatorFromPlanningScene scfps;
	// Test 1: verify that the correct pose is returned
	geometry_msgs::PoseStamped pose;
	EXPECT_TRUE(scfps.extractPoseStampedFromCollisionObject(coTable1_, pose));
	EXPECT_EQ(0.5, pose.pose.position.x);
	EXPECT_EQ(-0.75, pose.pose.position.y);
	EXPECT_EQ(0.58, pose.pose.position.z);
	EXPECT_EQ(0, pose.pose.orientation.x);
	EXPECT_EQ(0.28, pose.pose.orientation.y);
	EXPECT_EQ(0, pose.pose.orientation.z);
	EXPECT_EQ(1, pose.pose.orientation.w);

	// Test 2: verify that false is returned when co does not have a pose
	moveit_msgs::CollisionObject co = coTable1_;
	co.primitive_poses.clear();
	EXPECT_FALSE(scfps.extractPoseStampedFromCollisionObject(co, pose));
}

//TEST_F(stateCreatorFromPlanningSceneTest, distanceBetweenTwoPoses)
//{
//	StateCreatorFromPlanningScene scfps;
//	geometry_msgs::PoseStamped a, b;
//	a.header.frame_id = "/map";
//	a.header.stamp = ros::Time::now();
//	geometry_msgs::Pose pose;
//	pose.position.x = 1.0;
//	pose.position.y = 1.0;
//	pose.position.z = 1.0;
//	pose.orientation.w = 1.0;
//	pose.orientation.x = 0;
//	pose.orientation.y = 0;
//	pose.orientation.z = 0;
//	a.pose = pose;
//
//	b.header.frame_id = "/map";
//	b.header.stamp = ros::Time::now();
//	pose.position.x = 2.0;
//	pose.position.y = 1.0;
//	pose.position.z = 0;
//	b.pose = pose;
//
//	// Test 1: simple distance calc where the distance between the points is 1
//	std::pair<double, double> result;
//	result = scfps.distanceBetweenTwoPoses(a, b);
//	double dist, height;
//	dist = result.first;
//	height = result.second;
//	// Computed distance between 2 points: A (1, 1, 1) and B(2, 1, 0)
//	EXPECT_EQ(1, dist);
//	EXPECT_EQ(1, height);
//
//	// Test 2: distance gives no exact result (sqrt(5))
//	b.pose.position.y = 3.0;
//	b.pose.position.z = 3.0;
//	result = scfps.distanceBetweenTwoPoses(a, b);
//	dist = result.first;
//	height = result.second;
//	// Computed distance between 2 points: A (1, 1, 1) and B(2, 3, 0)
//	// SQRT( (xB - xA)^2 + (yB - yA)^2) = SQRT( (2-1)^2 + (3-1)^2 )
//	EXPECT_EQ(std::sqrt(5), dist);
//	EXPECT_EQ(2, height);
//
//	// TODO: Test 3: orientation of poses changes, compute dist
////	a.pose.orientation.x = 0.33;
////	a.pose.orientation.y = 0.67;
////	a.pose.orientation.z = 0.73;
////
////	b.pose.orientation.x = 0.45;
////	b.pose.orientation.y = 0.82;
////	b.pose.orientation.z = 0.12;
////	result = scfps.distanceBetweenTwoPoses(a, b);
////	dist = result.first;
////	height = result.second;
////	// Computed distance between 2 points: A (1, 1, 1) and B(2, 3, 0)
////	// SQRT( (xB - xA)^2 + (yB - yA)^2) = SQRT( (2-1)^2 + (3-1)^2 )
////	EXPECT_EQ(std::sqrt(5), dist);
////	EXPECT_EQ(2, height);
//}

};

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // do not forget to init ros because this is also a node
    ros::init(argc, argv, "stateCreatorFromPlanningSceneTest");
    return RUN_ALL_TESTS();
}
