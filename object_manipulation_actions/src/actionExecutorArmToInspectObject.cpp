#include "object_manipulation_actions/actionExecutorArmToInspectObject.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_utils/planning_scene_interface.h"

//PLUGINLIB_DECLARE_CLASS(object_manipulation_actions, action_executor_arm_to_inspect_object,
//        object_manipulation_actions::ActionExecutorArmToInspectObject,
//        continual_planning_executive::ActionExecutorInterface)
PLUGINLIB_EXPORT_CLASS(object_manipulation_actions::ActionExecutorArmToInspectObject, continual_planning_executive::ActionExecutorInterface)

namespace object_manipulation_actions
{

    void ActionExecutorArmToInspectObject::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorActionlib<tidyup_msgs::ArmToInspectObjectAction, tidyup_msgs::ArmToInspectObjectGoal,
            tidyup_msgs::ArmToInspectObjectResult>::initialize(arguments);

        _armStatePredicateName = "arm-state";
        _armAtInspectObjectConstantName = "arm_unknown";

        if(arguments.size() >= 3) {     // 3rd param: arm-state predicate name
            _armStatePredicateName = arguments[2];
        }
        if(arguments.size() >= 4) {     // 4th param: arm_at_InspectObject constant name
            _armAtInspectObjectConstantName = arguments[3];
        }
    }

    bool ActionExecutorArmToInspectObject::fillGoal(tidyup_msgs::ArmToInspectObjectGoal & goal,
                        const DurativeAction & a, const SymbolicState & current)
     {
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

        ROS_ASSERT(a.parameters.size() == 2);
        goal.object_id = a.parameters[0];
        goal.arm = a.parameters[1];

        return true;
    }

    void ActionExecutorArmToInspectObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
    		const tidyup_msgs::ArmToInspectObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("ArmToInspectObject returned result: %d", result.error_code.val);
        ROS_ASSERT(a.parameters.size() == 2);
        string object_id = a.parameters[0];
        string arm = a.parameters[1];
        // arm at InspectObject state estimator now sets this predicate
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arm Inspect Position succeeded.");
            //current.setObjectFluent(_armStatePredicateName, arm, _armAtInspectObjectConstantName);
            current.setBooleanPredicate("object-inspectable", object_id, true);
        } else {
            // current.setObjectFluent(_armStatePredicateName, arm, "arm_unknown");
        }
    }

};

