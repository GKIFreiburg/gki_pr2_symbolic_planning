#ifndef MOVEGROUPINTERFACE_H_
#define MOVEGROUPINTERFACE_H_

#include <moveit/move_group_interface/move_group.h>
#include <gtest/gtest.h>

/* THIS IMPLENTATION IS NOT THREAD SAFE */
namespace symbolic_planning_utils
{
	class MoveGroupInterface
	{
	private:
		static MoveGroupInterface* instance_;
		moveit::planning_interface::MoveGroup* right_arm_group_;
		moveit::planning_interface::MoveGroup* left_arm_group_;
		moveit::planning_interface::MoveGroup* arms_group_;
		moveit::planning_interface::MoveGroup* head_group_;
		moveit::planning_interface::MoveGroup* torso_group_;

		// Constructor: creating all moveit::planning_interfaces
//		FRIEND_TEST(moveGroupInterfaceTest, MoveGroupInterface);
		MoveGroupInterface();

		// Copy constructor must be private to prevent any additional creation of the object
		FRIEND_TEST(moveGroupInterfaceTest, copyConstructor);
		MoveGroupInterface(const MoveGroupInterface* mgi);

		// Assignment operator must also be private to prevent any additional creation of the object
		FRIEND_TEST(moveGroupInterfaceTest, assignmentOperator);
		MoveGroupInterface* operator= (const MoveGroupInterface* mgi);

		// The destructor is private in order to prevent clients that hold a pointer to the
		// Singleton object from deleting it accidentally.
		// Is not implemented, since the object should exist until the end of execution of the
		// entire program
		virtual ~MoveGroupInterface();

	public:
		static MoveGroupInterface* getInstance();
		moveit::planning_interface::MoveGroup* getRightArmGroup();
		moveit::planning_interface::MoveGroup* getLeftArmGroup();
		moveit::planning_interface::MoveGroup* getArmsGroup();
		moveit::planning_interface::MoveGroup* getHeadGroup();
		moveit::planning_interface::MoveGroup* getTorsoGroup();
	};
};

#endif /* MOVEGROUPINTERFACE_H_ */
