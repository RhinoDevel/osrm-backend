#ifndef GPX_DESCRIPTOR_HPP
#define GPX_DESCRIPTOR_HPP

#include "engine/descriptors/descriptor_base.hpp"
#include "util/xml_renderer.hpp"
#include "util/string_util.hpp"

#include "osrm/json_container.hpp"

#include <iostream>

namespace osrm
{
namespace engine
{
namespace descriptors
{



template <class DataFacadeT> class GPXDescriptor final : public BaseDescriptor<DataFacadeT>
{
  private:
    DescriptorConfig config;
    DataFacadeT *facade;

    template<std::size_t digits>
    void fixedIntToString(const int value, std::string &output)
    {
        char buffer[digits];
        buffer[digits-1] = 0; // zero termination
        output = util::printInt<11, 6>(buffer, value);
    }

    void AddRoutePoint(const util::FixedPointCoordinate &coordinate, util::json::Array &json_route)
    {
        util::json::Object json_lat;
        util::json::Object json_lon;
        util::json::Array json_row;

        std::string tmp;

        fixedIntToString<12>(coordinate.lat, tmp);
        json_lat.values["_lat"] = tmp;

        fixedIntToString<12>(coordinate.lon, tmp);
        json_lon.values["_lon"] = tmp;

        json_row.values.push_back(json_lat);
        json_row.values.push_back(json_lon);
        util::json::Object entry;
        entry.values["rtept"] = json_row;
        json_route.values.push_back(entry);
    }

  public:
    explicit GPXDescriptor(DataFacadeT *facade) : facade(facade) {}

    virtual void SetConfig(const DescriptorConfig &c) final { config = c; }

    virtual void Run(const InternalRouteResult &raw_route, util::json::Object &json_result) final
    {
        util::json::Array json_route;
        if (raw_route.shortest_path_length != INVALID_EDGE_WEIGHT)
        {
            AddRoutePoint(raw_route.segment_end_coordinates.front().source_phantom.location,
                          json_route);

            for (const std::vector<PathData> &path_data_vector : raw_route.unpacked_path_segments)
            {
                for (const PathData &path_data : path_data_vector)
                {
                    const util::FixedPointCoordinate current_coordinate =
                        facade->GetCoordinateOfNode(path_data.node);
                    AddRoutePoint(current_coordinate, json_route);
                }
            }
            AddRoutePoint(raw_route.segment_end_coordinates.back().target_phantom.location,
                          json_route);
        }
        // util::json::gpx_render(reply.content, json_route);
        json_result.values["route"] = json_route;
    }
};
}
}
}

#endif // GPX_DESCRIPTOR_HPP
