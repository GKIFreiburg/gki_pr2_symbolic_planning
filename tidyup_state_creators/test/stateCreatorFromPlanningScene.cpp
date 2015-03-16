#include <gtest/gtest.h>
#include <climits>
#include <ros/ros.h>
#include "tidyup_state_creators/stateCreatorFromPlanningScene.h"
#include <shape_msgs/SolidPrimitive.h>
#include <shape_tools/solid_primitive_dims.h>

namespace tidyup_state_creators {
TEST(stateCreatorFromPlanningSceneTest, addObjectToState){

	StateCreatorFromPlanningScene scfps;
	SymbolicState state;
	moveit_msgs::CollisionObject co;
	co.header.frame_id = "/map";
	co.header.stamp = ros::Time::now();
	co.id = "table1";
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.0;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
	geometry_msgs::Pose p;
    p.position.x = 0.50;
    p.position.y = -0.75;
    p.position.z = 0.58;
    p.orientation.x = 0;
    p.orientation.y = 0.28;
    p.orientation.z = 0;
    p.orientation.w = 1;
	co.primitive_poses.push_back(p);

	co.operation = co.ADD;
	ros::NodeHandle nh;
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	pub_co.publish(co);

    std::string objectType = "table";

	// Test 1: if object is added to state
    {
		scfps.addObjectToState(state, co, objectType);
		Predicate pred;
		pred.parameters.push_back(co.id);
		pred.name = "x";
		double val;
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(0.50, val);
		pred.name = "y";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_NE(-0.74, val);
		pred.name = "qy";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(0.28, val);
		pred.name = "qw";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(1, val);
    }

    // Test 2: object containing wrong frame should not be added to state
    //			Aborted by ROS_ASSERT
    {
    	co.header.frame_id = "/base_link";
    	pub_co.publish(co);
    	SymbolicState state;
    	::testing::FLAGS_gtest_death_test_style = "threadsafe";
    	EXPECT_DEATH_IF_SUPPORTED(scfps.addObjectToState(state, co, objectType), "");
    }

    // Test 3: add object without pose
    {
    	co.primitive_poses.clear();
    	co.header.frame_id = "/map";
    	pub_co.publish(co);
    	SymbolicState state;
		Predicate pred;
		pred.parameters.push_back(co.id);
		pred.name = "x";
		double val = -1;
		scfps.addObjectToState(state, co, objectType);
		EXPECT_FALSE(state.hasNumericalFluent(pred, &val));
		// verify that value has not been changed
		EXPECT_EQ(-1, val);
    }
}

TEST(stateCreatorFromPlanningSceneTest, checkIfTableInState){

	EXPECT_TRUE(true);
	//bool checkIfTableInState(const SymbolicState& state, const std::string& table);

}

TEST(stateCreatorFromPlanningSceneTest, extractPoseStampedFromSymbolicState)
{
//bool extractPoseStampedFromSymbolicState(const SymbolicState & state, const string & object,
//        geometry_msgs::PoseStamped & pose) const;
}


TEST(stateCreatorFromPlanningSceneTest, extractPoseStampedFromCollisionObject)
{
//bool extractPoseStampedFromCollisionObject(const moveit_msgs::CollisionObject &co,
//		geometry_msgs::PoseStamped & pose) const;
}

TEST(stateCreatorFromPlanningSceneTest, distanceBetweenTwoPoses)
{
//std::pair<double, double> distanceBetweenTwoPoses(const geometry_msgs::PoseStamped & posePS,
//        const geometry_msgs::PoseStamped & poseState);
}

};

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // do not forget to init ros because this is also a node
    ros::init(argc, argv, "stateCreatorFromPlanningSceneTest");
    return RUN_ALL_TESTS();
}
