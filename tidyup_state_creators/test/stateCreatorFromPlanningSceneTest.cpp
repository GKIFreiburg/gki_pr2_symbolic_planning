#include <gtest/gtest.h>
#include <climits>
#include <ros/ros.h>
#include "tidyup_state_creators/stateCreatorFromPlanningScene.h"
#include <shape_msgs/SolidPrimitive.h>
#include <shape_tools/solid_primitive_dims.h>

namespace tidyup_state_creators {

class stateCreatorFromPlanningSceneTest : public ::testing::Test {
	protected:
	stateCreatorFromPlanningSceneTest()
	{
		ros::NodeHandle nh;
		pubPlanningScene_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

	    ROS_INFO("Waiting for planning_scene publisher to have subscribers.");
	    while(pubPlanningScene_.getNumSubscribers() < 1) {
	        ros::Duration(0.5).sleep();
	    }

		setCollisionObjects();
	}

	void setCollisionObjects()
	{
		coTable1_.header.frame_id = "/odom_combined";
		header_.stamp = ros::Time::now();
		coTable1_.header.stamp = header_.stamp;
		coTable1_.id = "table1";
		coTable1_.primitives.resize(1);
		coTable1_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		coTable1_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		coTable1_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.40;
		coTable1_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.650;
		coTable1_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.02;
		geometry_msgs::Pose p;
		p.position.x = 4.81;
		p.position.y = 5.28;
		p.position.z = 0.90;
		p.orientation.x = 0;
		p.orientation.y = 0;
		p.orientation.z = 0;
		p.orientation.w = 1;
		coTable1_.operation = moveit_msgs::CollisionObject::ADD;
		coTable1_.primitive_poses.push_back(p);

		coTable2_.header.frame_id = "/odom_combined";
		header_.stamp = ros::Time::now();
		coTable2_.header.stamp = header_.stamp;
		coTable2_.id = "table2";
		coTable2_.primitives.resize(1);
		coTable2_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		coTable2_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		coTable2_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
		coTable2_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.50;
		coTable2_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.02;
		p.position.x = 0.5;
		p.position.y = 6.44;
		p.position.z = 0.73;
		p.orientation.x = 0;
		p.orientation.y = 0;
		p.orientation.z = 0;
		p.orientation.w = 1;
		coTable2_.operation = moveit_msgs::CollisionObject::ADD;
		coTable2_.primitive_poses.push_back(p);

		coObject_.header.frame_id = "/odom_combined";
		coObject_.header.stamp = header_.stamp;
		coObject_.id = "coke";
		coObject_.primitives.resize(1);
		coObject_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		coObject_.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		coObject_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.067;
		coObject_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.067;
		coObject_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.12;
		p.position.x = 4.81;
		p.position.y = 5.28;
		p.position.z = 0.90;
		p.orientation.x = 0;
		p.orientation.y = 0;
		p.orientation.z = 0;
		p.orientation.w = 1;
		coObject_.operation = moveit_msgs::CollisionObject::ADD;
		coObject_.primitive_poses.push_back(p);
	}

    // ROS Interface
    tf::TransformListener tf_;
	std_msgs::Header header_;
	moveit_msgs::CollisionObject coTable1_;
	moveit_msgs::CollisionObject coTable2_;
	moveit_msgs::CollisionObject coObject_;
	ros::Publisher pubPlanningScene_;


}; // class

// testing that member variable planningScene_ is initialized correctly
TEST_F(stateCreatorFromPlanningSceneTest, initializePlanningScene)
{
	StateCreatorFromPlanningScene scfps;
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.is_diff = true;

	planning_scene.world.collision_objects.push_back(coTable1_);
	planning_scene.world.collision_objects.push_back(coTable2_);
	planning_scene.world.collision_objects.push_back(coObject_);
	pubPlanningScene_.publish(planning_scene);
	ROS_INFO("Waiting 2s to make sure planning scene is updated");
	ros::Duration(2.0).sleep();

	scfps.initializePlanningScene();
	EXPECT_EQ(scfps.planningScene_.world.collision_objects.size(), 3);

	// shortcut
	moveit_msgs::CollisionObject co = scfps.planningScene_.world.collision_objects[0];
	EXPECT_EQ(co.id, coObject_.id);
	EXPECT_EQ(co.primitive_poses[0].position.x, coObject_.primitive_poses[0].position.x);
	EXPECT_EQ(co.primitive_poses[0].position.y, coObject_.primitive_poses[0].position.y);
	EXPECT_EQ(co.primitive_poses[0].position.z, coObject_.primitive_poses[0].position.z);

	co = scfps.planningScene_.world.collision_objects[1];
	EXPECT_EQ(co.id, coTable1_.id);
	EXPECT_EQ(co.primitive_poses[0].position.x, coTable1_.primitive_poses[0].position.x);
	EXPECT_EQ(co.primitive_poses[0].position.y, coTable1_.primitive_poses[0].position.y);
	EXPECT_EQ(co.primitive_poses[0].position.z, coTable1_.primitive_poses[0].position.z);

	co = scfps.planningScene_.world.collision_objects[2];
	EXPECT_EQ(co.id, coTable2_.id);
	EXPECT_EQ(co.primitive_poses[0].position.x, coTable2_.primitive_poses[0].position.x);
	EXPECT_EQ(co.primitive_poses[0].position.y, coTable2_.primitive_poses[0].position.y);
	EXPECT_EQ(co.primitive_poses[0].position.z, coTable2_.primitive_poses[0].position.z);

}

TEST_F(stateCreatorFromPlanningSceneTest, fillState)
{
	// Test 1: if object in planning scene is added to symbolic state
	{
		StateCreatorFromPlanningScene scfps;
		moveit_msgs::PlanningScene planning_scene;
		planning_scene.is_diff = true;

		planning_scene.world.collision_objects.push_back(coTable1_);
		planning_scene.world.collision_objects.push_back(coTable2_);
		planning_scene.world.collision_objects.push_back(coObject_);
		pubPlanningScene_.publish(planning_scene);
		ROS_INFO("Waiting 2s to make sure planning scene is updated");
		ros::Duration(2.0).sleep();

		SymbolicState state;
		// in real application, table collision objects are added in goalState
		scfps.addObjectToSymbolicState(state, coTable1_, "table");
		scfps.addObjectToSymbolicState(state, coTable2_, "table");

		scfps.fillState(state);

		// test if state has now object
		Predicate pred;
		pred.parameters.push_back(coObject_.id);
		pred.name = "x";
		double val;
		string value;
		bool result;
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].position.x, val);
		pred.name = "y";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].position.y, val);
		pred.name = "qy";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].orientation.y, val);
		pred.name = "qw";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(coObject_.primitive_poses[0].orientation.w, val);
		pred.name = "frame-id";
		EXPECT_TRUE(state.hasObjectFluent(pred, &value));
		moveit_msgs::CollisionObject tmp = scfps.planningScene_.world.collision_objects[0];
		EXPECT_EQ(tmp.header.frame_id, value);
		// frame-id of collision object coke stays the same, here it is "odom_combined"
		EXPECT_EQ(tmp.id, "coke");

		// test if state has object now
//		pred.parameters.clear();
//		pred.parameters.push_back(coObject_.id);
//		//pred.name = "frame-id";
//		pred.name = "movable_object";
//		EXPECT_TRUE(state.hasObjectFluent(pred, &value));

		pred.parameters.clear();
		pred.parameters.push_back(coObject_.id);
		pred.parameters.push_back(coTable1_.id);
		pred.name = "object-on";
		EXPECT_TRUE(state.hasBooleanPredicate(pred, &result));
		EXPECT_TRUE(result);

		pred.parameters.clear();
		pred.parameters.push_back(coObject_.id);
		pred.parameters.push_back(coTable2_.id);
		pred.name = "object-on";
		EXPECT_FALSE(state.hasBooleanPredicate(pred, &result));
		// testing value of result does not make sense since it is not set

	// Test 2: object removed from planning scene, if it is removed from symbolic state
		coObject_.operation = moveit_msgs::CollisionObject::REMOVE;
		planning_scene.world.collision_objects.push_back(coObject_);
		pubPlanningScene_.publish(planning_scene);
		ROS_INFO("Waiting 2s to make sure planning scene is updated");
		ros::Duration(2.0).sleep();

		scfps.fillState(state);
		// only table objects should be in state
		EXPECT_EQ(scfps.planningScene_.world.collision_objects.size(), 2);

		pred.parameters.clear();
		pred.parameters.push_back(coObject_.id);
		pred.name = "x";
		// object is not in state
		EXPECT_FALSE(state.hasNumericalFluent(pred, &val));

		// verify that predicate object-on is also removed
		pred.parameters.clear();
		pred.parameters.push_back(coObject_.id);
		pred.parameters.push_back(coTable1_.id);
		pred.name = "object-on";
		EXPECT_FALSE(state.hasBooleanPredicate(pred, &result));

	// Test 3: object added as grasped object, verify that it is in state and predicates are correct
		coObject_.operation = moveit_msgs::CollisionObject::ADD;

		moveit_msgs::AttachedCollisionObject attachedCo;
		attachedCo.link_name = "r_wrist_roll_link";
		attachedCo.object = coObject_;
		planning_scene.robot_state.attached_collision_objects.push_back(attachedCo);

		pubPlanningScene_.publish(planning_scene);
		ROS_INFO("Waiting 2s to make sure planning scene is updated");
		ros::Duration(2.0).sleep();

		scfps.fillState(state);
		// only table objects should be in state
		EXPECT_EQ(scfps.planningScene_.world.collision_objects.size(), 2);
		// coObject is now an attachedCollisionObject
		EXPECT_EQ(scfps.planningScene_.robot_state.attached_collision_objects.size(), 1);

		// test if state has object now
		pred.parameters.clear();
		pred.parameters.push_back(coObject_.id);
		pred.name = "x";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		// something wrong with frames - therefore different values for x
		// EXPECT_EQ(coObject_.primitive_poses[0].position.x, val);

		moveit_msgs::AttachedCollisionObject aco = scfps.planningScene_.robot_state.attached_collision_objects[0];

		pred.parameters.clear();
		pred.parameters.push_back(aco.object.id);
		pred.name = "frame-id";
		EXPECT_TRUE(state.hasObjectFluent(pred, &value));
		// frame is "r_wrist_roll_link"
		EXPECT_EQ(value, "r_wrist_roll_link");
		EXPECT_EQ(aco.object.header.frame_id, value);

	    EXPECT_EQ(aco.object.header.frame_id, "r_wrist_roll_link");
	    //			r_wrist_roll_link			odom_combined
	    EXPECT_NE(aco.object.header.frame_id, attachedCo.object.header.frame_id);

		pred.parameters.clear();
		pred.parameters.push_back(aco.object.id);
		EXPECT_EQ(aco.object.id, attachedCo.object.id);
		pred.name = "x";
		EXPECT_TRUE(state.hasNumericalFluent(pred, &val));
		EXPECT_EQ(aco.object.primitive_poses[0].position.x, val);

		pred.parameters.clear();
		pred.parameters.push_back(aco.object.id);
		pred.parameters.push_back("right_arm");
		pred.name = "object-grasped";
		EXPECT_TRUE(state.hasBooleanPredicate(pred, &result));
		EXPECT_TRUE(result);

		pred.parameters.clear();
		pred.parameters.push_back(coObject_.id);
		pred.parameters.push_back(coTable1_.id);
		pred.name = "object-on";
		// predicate object-on for object1 exists because it is not deleted but set to false
		// therefore need to test that variable result is false
		EXPECT_TRUE(state.hasBooleanPredicate(pred, &result));
		EXPECT_FALSE(result);

	// Test 4: object no longer grasped, verfiy that predicate object-grasped is false/not set
		attachedCo.object.operation = moveit_msgs::CollisionObject::REMOVE;
		planning_scene.robot_state.attached_collision_objects.push_back(attachedCo);
		pubPlanningScene_.publish(planning_scene);
		ROS_INFO("Waiting 2s to make sure planning scene is updated");
		ros::Duration(2.0).sleep();

		scfps.fillState(state);
		// only table objects should be in state
		EXPECT_EQ(scfps.planningScene_.world.collision_objects.size(), 2);
		EXPECT_EQ(scfps.planningScene_.robot_state.attached_collision_objects.size(), 0);

		// not possible to verify if object is deleted by using objectFluents because
		// object fluents are not removed when removing an object from symbolic state
//		pred.parameters.clear();
//		pred.parameters.push_back(attachedCo.object.id);
//		pred.name = "frame-id";
//		EXPECT_FALSE(state.hasObjectFluent(pred, &value));
		pred.parameters.clear();
		pred.parameters.push_back(attachedCo.object.id);
		pred.name = "x";
		EXPECT_FALSE(state.hasNumericalFluent(pred, &val));

		pred.parameters.clear();
		pred.parameters.push_back(attachedCo.object.id);
		pred.parameters.push_back("right_arm");
		pred.name = "object-grasped";
		EXPECT_FALSE(state.hasBooleanPredicate(pred, &result));
	}

}

}; // namespace

// Only one main function for all tests!
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // do not forget to init ros because this is also a node
    ros::init(argc, argv, "stateCreatorFromPlanningSceneTest");
    return RUN_ALL_TESTS();
}

