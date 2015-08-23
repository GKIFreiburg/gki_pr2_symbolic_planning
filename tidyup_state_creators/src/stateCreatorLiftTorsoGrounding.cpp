#include "tidyup_state_creators/stateCreatorLiftTorsoGrounding.h"
#include <pluginlib/class_list_macros.h>
//#include <symbolic_planning_utils/moveGroupInterface.h>
//#include <tidyup_utils/stringutil.h>
//#include <symbolic_planning_utils/joint_limits.h>
//#include <symbolic_planning_utils/extractPose.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorLiftTorsoGrounding, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    StateCreatorLiftTorsoGrounding::StateCreatorLiftTorsoGrounding()
    {
//    	torso_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();

	    ros::NodeHandle nhPriv("~");
	    // Namespace is "/continual_planning_executive"(/vdist_head_to_table)
	    nhPriv.param("vdist_threshold", vdist_threshold_, 0.02);
    }

    StateCreatorLiftTorsoGrounding::~StateCreatorLiftTorsoGrounding()
    {
    }

    void StateCreatorLiftTorsoGrounding::initialize(const std::deque<std::string> & arguments)
    {
    	ROS_ASSERT(arguments.size() == 3);
    	current_torso_height_ 	   = arguments[0];
    	sampled_torso_height_	   = arguments[1];
    	predicate_torso_lifted_	   = arguments[2];
    }

    bool StateCreatorLiftTorsoGrounding::fillState(SymbolicState & state)
    {
	    // get current torso height
		tf::StampedTransform transform;
		try
		{
			tf_.lookupTransform("/map", "/torso_lift_link", ros::Time(0), transform);
		} catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			return false;
		}

		const double& current_torso_height = transform.getOrigin().z();

    	// update current torso height in symbolic state
		state.setNumericalFluent(current_torso_height_, "", current_torso_height);

		// fetch sampled torso height from symbolic state
        Predicate p;
        p.name = sampled_torso_height_;
        double sampled_torso_height;
        if(!state.hasNumericalFluent(p, &sampled_torso_height))
        {
        	ROS_ERROR("StateCreatorLiftTorsoGrounding::%s: Could not fetch '%s' from symbolic state!",
        			__func__, sampled_torso_height_.c_str());
            return false;
        }

        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> table_range =
                state.getTypedObjects().equal_range("table");

        for (SymbolicState::TypedObjectConstIterator it = table_range.first;
                it != table_range.second; it++)
        {
        	// verify that robot is near table
        	Predicate p;
        	// Note: robot-near-table predicate has to be determined before this state creator!
        	p.name = "robot-near-table";
        	p.parameters.push_back(it->second); // table
        	bool val;
        	if (!state.hasBooleanPredicate(p, &val))
        	{
        		ROS_ERROR("StateCreatorLiftTorsoGrounding::%s: predicate: (%s %s) does not exist!", __func__,
        				p.name.c_str(), p.parameters[0].c_str());
        		return false;
        	}

        	// robot is near a table
        	if (val)
        	{
				// reset predicates torso-lifted to false
				state.setAllBooleanPredicates(predicate_torso_lifted_, false);

		        if (sampled_torso_height < 0)
		        {
		        	ROS_WARN("StateCreatorLiftTorsoGrounding::%s: No position for torso was sampled! Skip action.", __func__);
		        	state.setBooleanPredicate(predicate_torso_lifted_, it->second, true);
		        	return true;
		        }

		        // Check if current torso height and sampled torso height are nearly the same
		        if (fabs(current_torso_height - sampled_torso_height) < vdist_threshold_)
		        {
		        	ROS_INFO("StateCreatorLiftTorsoGrounding::%s: Torso height is lifted!", __func__);
		        	state.setBooleanPredicate(predicate_torso_lifted_, it->second, true);
		        }
		        else
		        {
		        	ROS_WARN("StateCreatorLiftTorsoGrounding::%s: Need to lift TORSO!", __func__);
		        	state.setBooleanPredicate(predicate_torso_lifted_, it->second, false);
		        }
        	}

        }
        return true;
    }
};

