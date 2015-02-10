#ifndef CACHING_EVALUATION_H
#define CACHING_EVALUATION_H

#include <string>
#include <set>
#include <map>
#include <fstream>

/// CachingEvaluation for one single type of caching and computed value.
/**
 * There might be multiple requests on this same thing if there is more than one
 * single key matching this value (depending on the key)
 */
class CachingEntry
{
    public:
        CachingEntry(double time);
        CachingEntry(bool hit, double time);

        double computationTime;     ///< time to compute this entry
        unsigned int hits;          ///< Cache hit this often
        unsigned int misses;        ///< Cache missing this often (i.e. value would have to be computed)
};

class CachingEvaluation
{
    public:
        CachingEvaluation();

        void recordNoCacheQuery(std::string & bestCacheKey, double time);

        /// Record a query for full state caching
        /**
         * \param [in] bestCacheKey the best key that could be found for caching (i.e. one that hits the most stuff)
         * \param [in] hit was this a cache hit
         * \param [in] time the time to compute such a query
         */
        void recordFullCacheQueryGlobal(const std::string & bestCacheKey,
                bool hit, double time);
        void recordPartialCacheQueryGlobal(const std::string & bestCacheKey,
                bool hit, double time);

        void recordFullCacheQueryLocal(const std::string & bestCacheKey,
                bool hit, double time);
        void recordPartialCacheQueryLocal(const std::string & bestCacheKey,
                bool hit, double time);

        void recordSubsumedCacheQueryGlobal(const std::string & bestCacheKey,
                bool hit, double time);
        void recordSubsumedCacheQueryLocal(const std::string & bestCacheKey,
                bool hit, double time);

        void writeResults();

        void setOutputFileName(const std::string & fn) { outputFilename = fn; }

    protected:
        std::string outputFilename;

        /// Best state key -> caching entry for no caching
        std::map<std::string, CachingEntry> noCacheCachings;

        /// Best state key -> caching entry for full state
        std::map<std::string, CachingEntry> fullStateCachings;
        /// Best state key -> caching entry for partial state
        std::map<std::string, CachingEntry> partialStateCachings;

        std::map<std::string, CachingEntry> fullStateLocalCachings;
        std::map<std::string, CachingEntry> partialStateLocalCachings;

        std::map<std::string, CachingEntry> subsumedStateCachings;
        std::map<std::string, CachingEntry> subsumedStateLocalCachings;
};

#endif

