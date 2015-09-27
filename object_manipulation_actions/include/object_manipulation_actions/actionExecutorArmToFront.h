#ifndef ACTION_EXECUTOR_ARM_TO_FRONT_H
#define ACTION_EXECUTOR_ARM_TO_FRONT_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <moveit/move_group_interface/move_group.h>

namespace object_manipulation_actions
{
    class ActionExecutorArmToFront : public continual_planning_executive::ActionExecutorInterface
    {
        public:

			virtual void initialize(const std::deque<std::string> & arguments);

			virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

			virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

			virtual void cancelAction();

        private:

			std::string actionName_;
			std::string rosparam_right_arm_to_front_;
			std::string rosparam_left_arm_to_front_;
			ros::Publisher pub_pose_;

			moveit::planning_interface::MoveGroup* left_arm_;
			moveit::planning_interface::MoveGroup* right_arm_;

    		moveit::planning_interface::MoveItErrorCode executeArmToFront(
    				moveit::planning_interface::MoveGroup* group,
    				const geometry_msgs::PoseStamped& pose);

    };

};

#endif

