/*
 * inverse_reachability_maps.h
 *
 *  Created on: Aug 27, 2015
 *      Author: andreas
 */

#ifndef INVERSE_REACHABILITY_MAPS_H_
#define INVERSE_REACHABILITY_MAPS_H_

#include <boost/shared_ptr.hpp>
#include <map>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <inverse_capability_map/InverseCapabilityOcTree.h>
#include <inverse_capability_map/Visualization.h>
#include <ros/ros.h>

namespace planner_modules_pr2
{
class InverseReachabilityMaps;
typedef boost::shared_ptr<InverseReachabilityMaps> InverseReachabilityMapsPtr;
struct TableInverseReachability
{
	geometry_msgs::PoseStamped pose;
	boost::shared_ptr<InverseCapabilityOcTree> inverse_reachability;
	TableInverseReachability(const geometry_msgs::PoseStamped& pose, boost::shared_ptr<InverseCapabilityOcTree> inverse_reachability) :
		pose(pose), inverse_reachability(inverse_reachability)
	{
	}
};
typedef boost::shared_ptr<const TableInverseReachability> TableInverseReachabilityConstPtr;
/// table_name -> inverse_reachablility_info
typedef std::map<const std::string, TableInverseReachabilityConstPtr> InverseReachabilityMap;

class InverseReachabilityMaps
{
public:
	static InverseReachabilityMapsPtr instance();
	virtual ~InverseReachabilityMaps();

	InverseReachabilityMap::const_iterator find(const std::string& table) const
	{
		return map.find(table);
	}
	InverseReachabilityMap::const_iterator begin() const
	{
		return map.begin();
	}
	InverseReachabilityMap::const_iterator end() const
	{
		return map.end();
	}

private:
	static InverseReachabilityMapsPtr instance_;
	InverseReachabilityMaps();

	InverseReachabilityMap map;
	ros::Publisher vis_publisher;
	visualization_msgs::MarkerArray markers;
};

} /* namespace planner_modules_pr2 */

#endif /* INVERSE_REACHABILITY_MAPS_H_ */
