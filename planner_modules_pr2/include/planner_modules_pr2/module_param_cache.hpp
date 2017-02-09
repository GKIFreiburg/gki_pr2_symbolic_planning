/*
 * module_param_cache.cpp
 *
 *  Created on: 19 Jul 2012
 *      Author: andreas
 */

template<class ValueType>
std::string ModuleParamCache<ValueType>::baseNamespace = "/tfd_modules/module_cache";

template<class ValueType>
ModuleParamCache<ValueType>::ModuleParamCache(
		const std::string& moduleNamespace,
		bool allowCachingToParamServer,
		bool storeParamHitsLocally)
{
	s_Debug = false;
	this->allowCachingToParamServer = allowCachingToParamServer;
	this->storeParamHitsLocally = storeParamHitsLocally;
	node.reset(new ros::NodeHandle());
	if (this->node->hasParam("/tfd_modules/run_name"))
	{
		std::string runName;
		this->node->getParam("/tfd_modules/run_name", runName);
		ModuleParamCache<ValueType>::baseNamespace += "/" + runName;
	}
	keyPrefix = ModuleParamCache<ValueType>::baseNamespace + "/" + moduleNamespace + "/";
}

template<class ValueType>
ModuleParamCache<ValueType>::~ModuleParamCache()
{
}

template<class ValueType>
void ModuleParamCache<ValueType>::clearAll()
{
	if (node->hasParam(keyPrefix))
	{
		node->deleteParam(keyPrefix);
	}
	_localCache.clear();
	_localTimeCache.clear();
}

template<class ValueType>
void ModuleParamCache<ValueType>::set(const std::string& key, ValueType value, double time)
{
//    ROS_INFO("[cache]: writing to cache: %s -> %f", key.c_str(), value);
	if (s_Debug)
	{
//		std::cout << "ModuleParamCache: adding " << key << " -> " << value << std::endl;
	}
	if (allowCachingToParamServer)
	{
		typename std::map<std::string, ValueType>::iterator it = _localCache.find(key);
		// if we found the same key,value in the local cache, it was inserted by this function and thus
		// is already on the param server, no need to make an extra setParam call here.
//		if (it == _localCache.end() || it->second != value)
		if (it == _localCache.end())
		{
			if (s_Debug)
			{
				std::cout << "Miss/Mismatch in local cache, adding as param" << std::endl;
			}
			node->setParam(keyPrefix + key, value);
			if (time >= 0)
				node->setParam(keyPrefix + key + "___time", time);
		}
	}
	_localCache.insert(std::make_pair(key, value));
	if (time >= 0)
	{
		_localTimeCache.insert(std::make_pair(key, time));
	}
}

template<class ValueType>
bool ModuleParamCache<ValueType>::get(const std::string& key, ValueType& value)
{
//	ROS_INFO("[cache]: lookup in cache: %s", key.c_str());
	typename std::map<std::string, ValueType>::iterator it = _localCache.find(key);
	if (it != _localCache.end())
	{
		value = it->second;
		return true;
	}

	// local miss, look on param server
	bool found = node->getParamCached(keyPrefix + key, value);
	if (found)
	{ // we found this on the param server, insert locally to prevent additional getParam calls
		if (storeParamHitsLocally)
		{
			_localCache.insert(std::make_pair(key, value));
		}
	}
	else
	{
		//ROS_INFO("[cache] GLOBAL MISS");
	}
	return found;
}

template<class ValueType>
bool ModuleParamCache<ValueType>::getTime(const std::string& key, double& time)
{
	typename std::map<std::string, double>::iterator it = _localTimeCache.find(key);
	if (it != _localTimeCache.end())
	{       // local cache hit
		time = it->second;
		return true;
	}
	// local miss, look on param server
	bool found = node->getParamCached(keyPrefix + key + "___time", time);
	if (found)
	{ // we found this on the param server, insert locally to prevent additional getParam calls
		_localTimeCache.insert(std::make_pair(key, time));
	}
	return found;

}

template<class ValueType>
void ModuleParamCache<ValueType>::dump() const
{
	std::cout << "ModuleParamCache at " << keyPrefix << std::endl;
	for (typename std::map<std::string, ValueType>::const_iterator it = _localCache.begin(); it != _localCache.end(); it++)
	{
		std::cout << it->first << " -> " << it->second << std::endl;
	}
	std::cout << std::endl;
}

