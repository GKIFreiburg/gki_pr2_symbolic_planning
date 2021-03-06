#include "tidyup_state_creators/stateCreatorObjectReachable.h"
#include "tidyup_state_creators/StateAccessException.h"
#include <pluginlib/class_list_macros.h>
#include <symbolic_planning_utils/extractPose.h>
//#include <sstream>

//PLUGINLIB_DECLARE_CLASS(tidyup_state_creators, state_creator_object_reachable,
//        tidyup_state_creators::StateCreatorObjectReachable, continual_planning_executive::StateCreator)
PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorObjectReachable, continual_planning_executive::StateCreator)


namespace tidyup_state_creators
{
    StateCreatorObjectReachable::StateCreatorObjectReachable()
    {
        reachDistance = 0.0;
        onlyForCurrentLocation = false;
    }
    StateCreatorObjectReachable::~StateCreatorObjectReachable()
    {
    }
    void StateCreatorObjectReachable::initialize(const std::deque<std::string> & arguments)
    {
        ROS_ASSERT(arguments.size() == 6);
        reachablePredicate = arguments[0];
        atPredicate = arguments[1];
        locationType = arguments[2];
        objectType  = arguments[3];
        reachDistance  = boost::lexical_cast<double>(arguments[4]);
        if (arguments[5] == "only_current_location")
        {
            onlyForCurrentLocation = true;
        }
    }

    bool StateCreatorObjectReachable::fillState(SymbolicState & state)
    {
        try
        {
            vector<tidyup_msgs::GraspableObjectPtr> objects;
            getMovableObjects(state, objects);
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets =
                    state.getTypedObjects().equal_range(locationType);
            Predicate at;
            at.name = atPredicate;
            at.parameters.push_back("location");
            for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++)
            {
                const string& location = it->second;
                if (onlyForCurrentLocation)
                {
                    at.parameters[0] = location;
                    bool value = false;
                    if (! state.hasBooleanPredicate(at, &value))
                    {
                        std::stringstream buff;
                        buff << __func__ << ": predicate: (" << at.name << " " << location << ") not found in state.";
                        throw StateAccessException(buff.str());
                    }
                    if (value)
                    {
                        processLocation(state, location, objects);
                    }
                }
                else
                {
                    processLocation(state, location, objects);
                }
            }
            return true;
        }
        catch (StateAccessException& ex)
        {
            ROS_ERROR_STREAM(ex.what());
            return false;
        }
    }

    void StateCreatorObjectReachable::getMovableObjects(const SymbolicState& state, vector<tidyup_msgs::GraspableObjectPtr>& objects) const
    {
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets =
            state.getTypedObjects().equal_range(objectType);
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++)
        {
            tidyup_msgs::GraspableObjectPtr object(new tidyup_msgs::GraspableObject);
            object->name = it->second;
            symbolic_planning_utils::extractPoseStampedFromSymbolicState(state, object->name, object->pose);
            objects.push_back(object);
        }
    }

    void StateCreatorObjectReachable::processLocation(SymbolicState& state, const string& location, const vector<tidyup_msgs::GraspableObjectPtr>& objects) const
    {
        geometry_msgs::PoseStamped robotPose;
        symbolic_planning_utils::extractPoseStampedFromSymbolicState(state, location, robotPose);
        Predicate reachable;
        reachable.name = reachablePredicate;
        reachable.parameters.push_back("object");
        reachable.parameters.push_back(location);
        forEach (tidyup_msgs::GraspableObjectConstPtr object, objects)
        {
            if (hypot(robotPose.pose.position.x - object->pose.pose.position.x, robotPose.pose.position.x-object->pose.pose.position.x) < reachDistance)
            {
                reachable.parameters[0] = object->name;
                state.setBooleanPredicate(reachablePredicate, reachable.parameters, true);
            }
        }
    }
};

