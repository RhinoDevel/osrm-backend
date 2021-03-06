#ifndef QUERY_NODE_HPP
#define QUERY_NODE_HPP

#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include "osrm/coordinate.hpp"

#include <limits>

struct QueryNode
{
    using key_type = OSMNodeID; // type of NodeID
    using value_type = int;     // type of lat,lons

    explicit QueryNode(int lat, int lon, OSMNodeID node_id) : lat(lat), lon(lon), node_id(node_id)
    {
    }
    QueryNode()
        : lat(std::numeric_limits<int>::max()), lon(std::numeric_limits<int>::max()),
          node_id(SPECIAL_OSM_NODEID)
    {
    }

    int lat;
    int lon;
    OSMNodeID node_id;

    static QueryNode min_value()
    {
        return QueryNode(static_cast<int>(-90 * COORDINATE_PRECISION),
                         static_cast<int>(-180 * COORDINATE_PRECISION), MIN_OSM_NODEID);
    }

    static QueryNode max_value()
    {
        return QueryNode(static_cast<int>(90 * COORDINATE_PRECISION),
                         static_cast<int>(180 * COORDINATE_PRECISION), MAX_OSM_NODEID);
    }

    value_type operator[](const std::size_t n) const
    {
        switch (n)
        {
        case 1:
            return lat;
        case 0:
            return lon;
        default:
            break;
        }
        BOOST_ASSERT_MSG(false, "should not happen");
        return std::numeric_limits<int>::lowest();
    }
};

#endif // QUERY_NODE_HPP
