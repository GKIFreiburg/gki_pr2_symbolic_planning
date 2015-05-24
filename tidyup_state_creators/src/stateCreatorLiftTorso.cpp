#include "tidyup_state_creators/stateCreatorLiftTorso.h"
#include <pluginlib/class_list_macros.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <tidyup_utils/stringutil.h>
#include <symbolic_planning_utils/joint_limits.h>
#include <symbolic_planning_utils/extractPose.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorLiftTorso, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    StateCreatorLiftTorso::StateCreatorLiftTorso()
    {
    	torso_group_ = symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();

	    ros::NodeHandle nhPriv("~");
	    // Namespace is "/continual_planning_executive"(/vdist_head_to_table)
	    nhPriv.param("vdist_head_to_table", vdist_head_to_table_, 0.60);
	    nhPriv.param("vdist_threshold", vdist_threshold_, 0.002);
    }

    StateCreatorLiftTorso::~StateCreatorLiftTorso()
    {
    }

    void StateCreatorLiftTorso::initialize(const std::deque<std::string> & arguments)
    {
    	ROS_ASSERT(arguments.size() == 2);
    	torso_position_ 		= arguments[0];			// torso-position
    	predicate_torso_lifted_ = arguments[1];			// torso-lifted
    }

    bool StateCreatorLiftTorso::fillState(SymbolicState & state)
    {
    	// get current torso position
    	std::vector<double> current_joint_values = torso_group_->getCurrentJointValues();
    	ROS_ASSERT(current_joint_values.size() == 1);

    	state.setNumericalFluent(torso_position_, "", current_joint_values[0]);

    	// TODO: mani_loc as argument in init
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> loc_range =
                state.getTypedObjects().equal_range("manipulation_location");

        for (SymbolicState::TypedObjectConstIterator it = loc_range.first;
                it != loc_range.second; it++)
        {
        	// verify that robot is in current location
        	Predicate p;
        	// TODO: robot as argument in init
        	p.name = "robot-at";
        	p.parameters.push_back(it->second); // mani_loc
        	bool val;
        	if (!state.hasBooleanPredicate(p, &val))
        	{
        		ROS_ERROR("StateCreatorLiftTorso::%s: predicate: (%s %s) does not exist!", __func__,
        				p.name.c_str(), p.parameters[0].c_str());
        		return false;
        	}

        	// robot is at a manipulation_location
        	if (val)
        	{
				// reset predicates torso-lifted to false
				state.setAllBooleanPredicates(predicate_torso_lifted_, false);

            	std::vector<std::string> params = StringUtil::split(it->second, "_");
            	ROS_ASSERT(StringUtil::startsWith(params[0], "table"));
            	std::string table = params[0];

            	// Head is vdist_head_to_table away from table
        		if (verifyHeadTableDistance(state, table))
        			state.setBooleanPredicate(predicate_torso_lifted_, table, true);
        		// Head is less than vdist_h_t away from table, but torso is lifted to maximum
        		else if (verifyTorsoMaximum())
        			state.setBooleanPredicate(predicate_torso_lifted_, table, true);
        	}

        }
        return true;
    }

    bool StateCreatorLiftTorso::verifyHeadTableDistance(SymbolicState & state, const std::string& table)
    {
    	// get height of table from symbolic state
    	geometry_msgs::PoseStamped tablePose;
    	if (!symbolic_planning_utils::extractPoseStampedFromSymbolicState(state, table, tablePose))
    	{
    		ROS_ERROR("StateCreatorLiftTorso::%s: Could not extract %s pose from symbolic state",
    				__func__, table.c_str());
    		return false;
    	}
		ROS_ASSERT(tablePose.header.frame_id == "/map");
		ROS_DEBUG_STREAM("table pose" << tablePose);

		// transform headPose into frame of /map
		geometry_msgs::PoseStamped headPose;
		headPose.header.frame_id = "/head_pan_link";
		headPose.header.stamp = ros::Time::now();
		geometry_msgs::Quaternion q;
		q.x = 0;
		q.y = 0;
		q.z = 0;
		q.w = 1;
		headPose.pose.orientation = q;
		geometry_msgs::Point p;
		p.x = 0;
		p.y = 0;
		p.z = 0;
		headPose.pose.position = p;

		geometry_msgs::PoseStamped head_transformed;
		try {
			tf_.waitForTransform("/map", headPose.header.frame_id, headPose.header.stamp,
					ros::Duration(0.5));
			tf_.transformPose("/map", headPose, head_transformed);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
			return false;
		}
		ROS_DEBUG_STREAM("head pose transformed" << head_transformed);

		double distance = fabs(head_transformed.pose.position.z - tablePose.pose.position.z);
		ROS_DEBUG("Distance: %lf, head: %lf, table: %lf", distance, head_transformed.pose.position.z, tablePose.pose.position.z);
		if (vdist_head_to_table_ - vdist_threshold_ < distance &&
			distance < vdist_head_to_table_ + vdist_threshold_)
			// head_mount_link is about vdist_head_to_table_ above table, no need to lift torso
			return true;
		else
			return false;
    }

    bool StateCreatorLiftTorso::verifyTorsoMaximum()
    {
    	std::vector<std::string> joint_names = torso_group_->getActiveJoints();
    	ROS_ASSERT(joint_names.size() == 1);

    	// get joint limits
    	symbolic_planning_utils::limits limits =
    			symbolic_planning_utils::JointLimits::getJointLimit(torso_group_, joint_names[0]);

    	// get current joint values
    	std::vector<double> current_joint_values = torso_group_->getCurrentJointValues();
    	ROS_ASSERT(current_joint_values.size() == 1);

    	if (fabs(current_joint_values[0] - limits.max_position) < vdist_threshold_)
    		return true;
    	else
    		return false;
    }
};

