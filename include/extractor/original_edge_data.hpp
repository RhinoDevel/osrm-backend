#ifndef ORIGINAL_EDGE_DATA_HPP
#define ORIGINAL_EDGE_DATA_HPP

#include "extractor/travel_mode.hpp"
#include "extractor/turn_instructions.hpp"
#include "util/typedefs.hpp"

#include <limits>

struct OriginalEdgeData
{
    explicit OriginalEdgeData(NodeID via_node,
                              unsigned name_id,
                              TurnInstruction turn_instruction,
                              bool compressed_geometry,
                              TravelMode travel_mode)
        : via_node(via_node), name_id(name_id), turn_instruction(turn_instruction),
          compressed_geometry(compressed_geometry), travel_mode(travel_mode)
    {
    }

    OriginalEdgeData()
        : via_node(std::numeric_limits<unsigned>::max()),
          name_id(std::numeric_limits<unsigned>::max()), turn_instruction(TurnInstruction::NoTurn),
          compressed_geometry(false), travel_mode(TRAVEL_MODE_INACCESSIBLE)
    {
    }

    NodeID via_node;
    unsigned name_id;
    TurnInstruction turn_instruction;
    bool compressed_geometry;
    TravelMode travel_mode;
};

#endif // ORIGINAL_EDGE_DATA_HPP
