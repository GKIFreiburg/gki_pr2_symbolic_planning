#ifndef SUBSUMPTION_CACHING_H
#define SUBSUMPTION_CACHING_H

#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <iostream>

/// Subsumption caching for putdown modules


/// A putdown request (all that is needed to specify this) with a possible answer.
class PutdownRequest
{
    public:
        /// A movable object relevant for this request.
        struct MovableObject {
            std::string id;
            std::string type;
            geometry_msgs::Pose pose;
        };

    public:
        PutdownRequest();
        PutdownRequest(const std::string & paramKey, const std::string & paramValue);

        std::string toParamKey() const;
        std::string toParamValue() const;

    public:
        std::string putdownObject;
        std::string putdownObjectType;
        std::string arm;
        std::string staticObject;
        std::vector<MovableObject> objectsOnStatic;

        bool computed;  ///< has this been computed yet or is it just the request
        double computationTime; ///< Time it took to compute this request.
        bool success;   ///< if it has been computed was the request successfull (is putdownPose valid)

        geometry_msgs::Pose putdownPose;
};

class SubsumptionCache
{
    public:
        SubsumptionCache(const std::string & moduleNamespace = std::string());

        /// Record a computed PutdownRequest in the cache.
        void put(const PutdownRequest & pdr);

        /// Query the cache for the request pdr.
        /**
         * \returns true if a match was found.
         * \param [out] match the matching PutdownRequest that already answers pdr (neednt be the same)
         */
        bool query(const PutdownRequest & pdr, PutdownRequest & match);

        /// Output verbose debugging information for this query.
        void debugQuery(const PutdownRequest & pdr) const;

        /// Determines if lhs is more constrained than rhs.
        /**
         * This is not a total order sorting!
         * if lhs isMoreConstrained than rhs, then rhs isLessConstrained than lhs,
         * but if lhs !isMoreConstrained than rhs, then rhs !isMoreConstrained lhs
         *
         * \returns 1 if lhs isMoreConstrained then rhs, -1 if rhs isMoreConstrained then lhs,
         *          0 if the request match, -100 if they cannot be compared.
         */
        int isMoreConstrained(const PutdownRequest & lhs, const PutdownRequest & rhs) const;

        void writeToParams();
        bool readFromParams();

        void dump() const;
    protected:
        bool objectMatches(const PutdownRequest::MovableObject & mol, const PutdownRequest::MovableObject & mor) const;

        /// Determines the path for param caching.
        std::string getParamKeyPrefix();

    protected:
        /// The actual cache is a list of successfull and failed requests.
        std::vector<PutdownRequest> successfullRequests;
        std::vector<PutdownRequest> failedRequests;

        std::string moduleNamespace;
};

std::ostream & operator<<(std::ostream & os, const PutdownRequest::MovableObject & mo);
std::ostream & operator<<(std::ostream & os, const PutdownRequest & pdr);

#endif

