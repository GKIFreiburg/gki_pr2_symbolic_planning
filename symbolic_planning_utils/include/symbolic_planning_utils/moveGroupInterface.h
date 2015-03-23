#ifndef MOVEGROUPINTERFACE_H_
#define MOVEGROUPINTERFACE_H_

#include <moveit/move_group_interface/move_group.h>

/* A thread-safe eagerly created instance for moveGroupInterface
 * Using an eagerly method, because it is thread safe, easy to implement
 * and the most important the interface is being used in almost every
 * actionExecutor.
 * Other possibility would be to use a double-checked locking - requires
 * a mutex variable
 *
 * http://silviuardelean.ro/category/software/programming/c/c-11/
 */

namespace symbolic_planning_utils
{
	class MoveGroupInterface
	{
	private:
		static MoveGroupInterface* instance;
		moveit::planning_interface::MoveGroup* right_arm_group_;
		moveit::planning_interface::MoveGroup* left_arm_group_;
		moveit::planning_interface::MoveGroup* arms_group_;
		moveit::planning_interface::MoveGroup* head_group_;

		// Constructor: creating all moveit::planning_interfaces
		MoveGroupInterface();

		// Copy constructor must be private to prevent any additional creation of the object
		MoveGroupInterface(const MoveGroupInterface* mgi);

		// Assignment operator must also be private to prevent any additional creation of the object
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

	};
};

#endif /* MOVEGROUPINTERFACE_H_ */
