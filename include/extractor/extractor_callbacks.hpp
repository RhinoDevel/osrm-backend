#ifndef EXTRACTOR_CALLBACKS_HPP
#define EXTRACTOR_CALLBACKS_HPP

#include "util/typedefs.hpp"
#include <boost/optional/optional_fwd.hpp>

#include <string>
#include <unordered_map>

struct ExternalMemoryNode;
class ExtractionContainers;
struct InputRestrictionContainer;
struct ExtractionNode;
struct ExtractionWay;
namespace osmium
{
class Node;
class Way;
}

/**
 * This class is uses by the extractor with the results of the
 * osmium based parsing and the customization through the lua profile.
 *
 * It mediates between the multi-threaded extraction process and the external memory containers.
 * Thus the synchronization is handled inside of the extractor.
 */
class ExtractorCallbacks
{
  private:
    // used to deduplicate street names: actually maps to name ids
    std::unordered_map<std::string, NodeID> string_map;
    ExtractionContainers &external_memory;

  public:
    ExtractorCallbacks() = delete;
    ExtractorCallbacks(const ExtractorCallbacks &) = delete;
    explicit ExtractorCallbacks(ExtractionContainers &extraction_containers);

    // warning: caller needs to take care of synchronization!
    void ProcessNode(const osmium::Node &current_node, const ExtractionNode &result_node);

    // warning: caller needs to take care of synchronization!
    void ProcessRestriction(const boost::optional<InputRestrictionContainer> &restriction);

    // warning: caller needs to take care of synchronization!
    void ProcessWay(const osmium::Way &current_way, const ExtractionWay &result_way);
};

#endif /* EXTRACTOR_CALLBACKS_HPP */
