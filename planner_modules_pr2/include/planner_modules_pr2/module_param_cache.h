/*
 * module_param_cache.h
 *
 *  Created on: 19 Jul 2012
 *      Author: andreas
 */

#ifndef MODULE_PARAM_CACHE_H_
#define MODULE_PARAM_CACHE_H_

#include <string>
#include <map>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include "tfd_modules/module_api/pddlModuleTypes.h"

/// Module cache that allows to cache entries locally (for one run) or additionally
/// on the param server between multiple runs. ValueType needs to be able to be written
/// as a ROS param.
/**
 * It is assumed that during a planner run only this class accesses the
 * module cache stored on the param server, i.e. nobody interferes.
 */
template <class ValueType>
class ModuleParamCache
{
public:
    ModuleParamCache();
    ~ModuleParamCache();

    /// Clear all cache entries for this modules namespace (especially on the param server)
    void clearAll();

    /// Initialize the module param cache for param lookup.
    void initialize(const std::string& moduleNamespace, ros::NodeHandle* node);

    /// Add a new cache entry.
    /**
     * \param [in] key the key for the entry
     * \param [in] value the value to cache for key
     * \param [in] allowCacheAsParam if true, cache entries are entered on the
     * \param [in] time time in seconds to calculate this value, unknown values (< 0) are not cached
     * param server and thus are available between multiple planner calls.
     * Therefore their value should obviously not change in the world
     * throughout multiple calls.
     */
    void set(const std::string& key, ValueType value, bool allowCacheAsParam, double time = -1);

    /// Retrieve a cached value for key.
    /**
     * \param [in] key the key for the entry
     * \param [out] value the retrieved value, if found.
     * \param [out] wasLocal if not NULL, set to true, if the value was already in the local cache
     * \param [in] storeGlobalLocally for hits from param server, store them in local cache
     * \returns true, if the value was found in the cache.
     */
    bool get(const std::string& key, ValueType& value, bool* wasLocal = NULL, bool storeGlobalLocally = true);

    /// Retrieve a cached value for key.
    /**
     * \param [in] key the key for the entry
     * \param [out] time the time in seconds to originally calculate the value
     * \returns true, if the value was found in the cache.
     */
    bool getTime(const std::string& key, double& time);

    void dump() const;
private:
    static std::string baseNamespace;

    /// Param cache across multiple runs.
    ros::NodeHandle* node;
    std::string keyPrefix;

    std::map<std::string, ValueType> _localCache;  ///< local cache only for this planner run
    std::map<std::string, double> _localTimeCache; ///< local cache for the time it took for computing this keys value.

    bool s_Debug;
};

/// Create a string from a Pose that is unique and can be stored in the param daemon.
std::string createPoseParamString(const geometry_msgs::Pose & pose, double precPose = 0.01, double precQuat = 0.01);
/// Create a string from a Pose that is unique and can be stored in the param daemon.
std::string createPoseParamString(const geometry_msgs::Pose2D & pose, double precPose = 0.01, double precTheta = 0.01);

std::string computeFullStateCacheKey(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback);

/// Split a named id (e.g. robot0) in name and id (robot, 0).
bool splitNamedId(const string & namedId, string & name, int & id);

#include "module_param_cache.hpp"

typedef ModuleParamCache<double> ModuleParamCacheDouble;
typedef ModuleParamCache<std::string> ModuleParamCacheString;

#endif /* MODULE_PARAM_CACHE_H_ */

