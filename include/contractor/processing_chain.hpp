#ifndef PROCESSING_CHAIN_HPP
#define PROCESSING_CHAIN_HPP

#include "contractor/contractor.hpp"
#include "contractor/contractor_options.hpp"
#include "contractor/query_edge.hpp"
#include "util/static_graph.hpp"
#include "util/deallocating_vector.hpp"
#include "util/node_based_graph.hpp"

struct SpeedProfileProperties;
struct EdgeBasedNode;
struct lua_State;

#include <boost/filesystem.hpp>

#include <vector>

/**
    \brief class of 'prepare' utility.
 */
class Prepare
{
  public:
    using EdgeData = QueryEdge::EdgeData;

    explicit Prepare(ContractorConfig contractor_config) : config(std::move(contractor_config)) {}
    Prepare(const Prepare &) = delete;
    ~Prepare();

    int Run();

  protected:
    void ContractGraph(const unsigned max_edge_id,
                       DeallocatingVector<EdgeBasedEdge> &edge_based_edge_list,
                       DeallocatingVector<QueryEdge> &contracted_edge_list,
                       std::vector<bool> &is_core_node,
                       std::vector<float> &node_levels) const;
    void WriteCoreNodeMarker(std::vector<bool> &&is_core_node) const;
    void WriteNodeLevels(std::vector<float> &&node_levels) const;
    void ReadNodeLevels(std::vector<float> &contraction_order) const;
    std::size_t WriteContractedGraph(unsigned number_of_edge_based_nodes,
                                     const DeallocatingVector<QueryEdge> &contracted_edge_list);
    void FindComponents(unsigned max_edge_id,
                        const DeallocatingVector<EdgeBasedEdge> &edges,
                        std::vector<EdgeBasedNode> &nodes) const;

  private:
    ContractorConfig config;
    std::size_t LoadEdgeExpandedGraph(const std::string &edge_based_graph_path,
                                      DeallocatingVector<EdgeBasedEdge> &edge_based_edge_list,
                                      const std::string &edge_segment_lookup_path,
                                      const std::string &edge_penalty_path,
                                      const std::string &segment_speed_path);
};

#endif // PROCESSING_CHAIN_HPP
