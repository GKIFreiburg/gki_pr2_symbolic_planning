#include "symbolic_planning_utils/moveGroupInterface.h"

#include <gtest/gtest.h>
#include <climits>
#include <ros/ros.h>

namespace symbolic_planning_utils {

	struct testStruct{
		int a;
		int b;
	};

	TEST(moveGroupInterfaceTest, MoveGroupInterface)
	{
		// Verify that 2 moveGroupInterface objects have the same address
		MoveGroupInterface* mg = MoveGroupInterface::getInstance();
		MoveGroupInterface* mg2 = MoveGroupInterface::getInstance();
		EXPECT_EQ(mg, mg2);

		// Verify that 2 objects created with new have different addresses
		testStruct* a = new testStruct;
		testStruct* b = new testStruct;
		EXPECT_NE(a, b);

		// Verify that object is initialized correctly
		moveit::planning_interface::MoveGroup* group = NULL;
		group = MoveGroupInterface::getInstance()->getRightArmGroup();
		EXPECT_TRUE(group != NULL);
		EXPECT_EQ("right_arm", group->getName());
	}

	TEST(moveGroupInterfaceTest, copyConstructor)
	{
		MoveGroupInterface* mg = MoveGroupInterface::getInstance();

		// Test copy constructor
		MoveGroupInterface* mg2(mg);
		EXPECT_EQ(mg, mg2);
	}

	TEST(moveGroupInterfaceTest, assignmentOperator)
	{
		MoveGroupInterface* mg = MoveGroupInterface::getInstance();

		// Test copy constructor
		MoveGroupInterface* mg2 = mg;
		EXPECT_EQ(mg, mg2);
	}

}; // namespace

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // do not forget to init ros because this is also a node
    ros::init(argc, argv, "moveGroupInterfaceTest");
    return RUN_ALL_TESTS();
}
