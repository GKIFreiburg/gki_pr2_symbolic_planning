#ifndef STATE_CREATOR_OBJECT_REACHABLE_H
#define STATE_CREATOR_OBJECT_REACHABLE_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <tidyup_msgs/GraspableObject.h>

namespace tidyup_state_creators
{

/// This state creator adds the reachable predicate for objects at the current manipulation location
class StateCreatorObjectReachable: public continual_planning_executive::StateCreator
{
public:
    StateCreatorObjectReachable();
    ~StateCreatorObjectReachable();

    /// Initialize the state creator parameters.
    /**
     * args: service_name predicate_name predicate_value
     *
     * The service name of the arms-at-side service.
     */
    virtual void initialize(const std::deque<std::string> & arguments);

    virtual bool fillState(SymbolicState & state);

protected:
    void getMovableObjects(const SymbolicState& state, vector<tidyup_msgs::GraspableObjectPtr>& objects) const;
    void processLocation(SymbolicState& state, const string& location, const vector<tidyup_msgs::GraspableObjectPtr>& objects) const;
    void extractPoseStamped(const SymbolicState & state, const string & object, geometry_msgs::PoseStamped & pose) const;

    string reachablePredicate;
    string atPredicate;
    string locationType;
    string objectType;
    double reachDistance;
    bool onlyForCurrentLocation;
};

}
;

#endif
