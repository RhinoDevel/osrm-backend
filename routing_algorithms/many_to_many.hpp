/*

Copyright (c) 2014, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MANY_TO_MANY_ROUTING_HPP
#define MANY_TO_MANY_ROUTING_HPP

#include "routing_base.hpp"
#include "../data_structures/search_engine_data.hpp"
#include "../typedefs.h"

#include <boost/assert.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

template <class DataFacadeT>
class ManyToManyRouting final
    : public BasicRoutingInterface<DataFacadeT, ManyToManyRouting<DataFacadeT>>
{
    using super = BasicRoutingInterface<DataFacadeT, ManyToManyRouting<DataFacadeT>>;
    using QueryHeap = SearchEngineData::QueryHeap;
    SearchEngineData &engine_working_data;

    struct NodeBucket
    {
        unsigned target_id; // essentially a row in the distance matrix
        EdgeWeight distance;
        const EdgeMeters meters;
        NodeBucket(const unsigned target_id, const EdgeWeight distance, const EdgeMeters meters)
            : target_id(target_id), distance(distance), meters(meters)
        {
        }
    };
    using SearchSpaceWithBuckets = std::unordered_map<NodeID, std::vector<NodeBucket>>;

  public:
    ManyToManyRouting(DataFacadeT *facade, SearchEngineData &engine_working_data)
        : super(facade), engine_working_data(engine_working_data)
    {
    }

    ~ManyToManyRouting() {}

    std::shared_ptr<std::vector<std::pair<EdgeWeight, EdgeMeters>>>
    operator()(const std::vector<PhantomNode> &phantom_sources_array, const std::vector<PhantomNode> &phantom_targets_array) const
    {
        const auto number_of_sources = phantom_sources_array.size();
        const auto number_of_targets = phantom_targets_array.size();
        std::shared_ptr<std::vector<std::pair<EdgeWeight, EdgeMeters>>> result_table =
            std::make_shared<std::vector<std::pair<EdgeWeight, EdgeMeters>>>(number_of_targets * number_of_sources,
                                                      std::make_pair(std::numeric_limits<EdgeWeight>::max(), std::numeric_limits<EdgeMeters>::max()));

        engine_working_data.InitializeOrClearFirstThreadLocalStorage(
            super::facade->GetNumberOfNodes());

        QueryHeap &query_heap = *(engine_working_data.forward_heap_1);

        SearchSpaceWithBuckets search_space_with_buckets;

        std::unordered_map<NodeID, EdgeMeters> node_meters;

        unsigned target_id = 0;
        for (const auto &phantom : phantom_targets_array)
        {
            query_heap.Clear();
            node_meters.clear();
            // insert target(s) at distance 0

            if (SPECIAL_NODEID != phantom.forward_node_id)
            {
                query_heap.Insert(phantom.forward_node_id,
                                  phantom.GetForwardWeightPlusOffset(),
                                  phantom.forward_node_id);

                node_meters[phantom.forward_node_id] = 0;
            }
            if (SPECIAL_NODEID != phantom.reverse_node_id)
            {
                query_heap.Insert(phantom.reverse_node_id,
                                  phantom.GetReverseWeightPlusOffset(),
                                  phantom.reverse_node_id);

                node_meters[phantom.reverse_node_id] = 0;
            }

            // explore search space
            while (!query_heap.Empty())
            {
                BackwardRoutingStep(target_id, query_heap, search_space_with_buckets, node_meters);
            }
            BOOST_ASSERT(node_meters.empty());

            ++target_id;
        }

        // for each source do forward search
        unsigned source_id = 0;
        for (const auto &phantom : phantom_sources_array)
        {
            query_heap.Clear();
            node_meters.clear();
            // insert target(s) at distance 0

            if (SPECIAL_NODEID != phantom.forward_node_id)
            {
                query_heap.Insert(phantom.forward_node_id,
                                 -phantom.GetForwardWeightPlusOffset(),
                                  phantom.forward_node_id);

                node_meters[phantom.forward_node_id] = 0;
            }
            if (SPECIAL_NODEID != phantom.reverse_node_id)
            {
                query_heap.Insert(phantom.reverse_node_id,
                                 -phantom.GetReverseWeightPlusOffset(),
                                  phantom.reverse_node_id);

                node_meters[phantom.reverse_node_id] = 0;
            }

            // explore search space
            while (!query_heap.Empty())
            {
                ForwardRoutingStep(source_id, number_of_targets, query_heap,
                                   search_space_with_buckets, node_meters, result_table);
            }
            BOOST_ASSERT(node_meters.empty());

            ++source_id;
        }
        // BOOST_ASSERT(source_id == target_id);
        return result_table;
    }

    void ForwardRoutingStep(const unsigned source_id,
                            const unsigned number_of_targets,
                            QueryHeap &query_heap,
                            const SearchSpaceWithBuckets &search_space_with_buckets,
                            std::unordered_map<NodeID, EdgeMeters> &node_meters,
                            std::shared_ptr<std::vector<std::pair<EdgeWeight, EdgeMeters>>> result_table) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int source_distance = query_heap.GetKey(node);
        const EdgeMeters source_meters = node_meters[node];
        node_meters.erase(node);

        // check if each encountered node has an entry
        const auto bucket_iterator = search_space_with_buckets.find(node);
        // iterate bucket if there exists one
        if (bucket_iterator != search_space_with_buckets.end())
        {
            const std::vector<NodeBucket> &bucket_list = bucket_iterator->second;
            for (const NodeBucket &current_bucket : bucket_list)
            {
                // get target id from bucket entry
                const unsigned target_id = current_bucket.target_id;
                const int target_distance = current_bucket.distance;
                const EdgeMeters target_meters = current_bucket.meters;
                const EdgeWeight current_distance =
                    ((*result_table)[source_id * number_of_targets + target_id]).first;
                // check if new distance is better
                const EdgeWeight new_distance = source_distance + target_distance;
                if (new_distance >= 0 && new_distance < current_distance)
                {
                    ((*result_table)[source_id * number_of_targets + target_id]).first =
                       (source_distance + target_distance);

                    ((*result_table)[source_id * number_of_targets + target_id]).second =
                        source_meters + target_meters;
                }
            }
        }
        if (StallAtNode<true>(node, source_distance, query_heap))
        {
            return;
        }
        RelaxOutgoingEdges<true>(node, source_distance, source_meters, query_heap, node_meters);
    }

    void BackwardRoutingStep(const unsigned target_id,
                             QueryHeap &query_heap,
                             SearchSpaceWithBuckets &search_space_with_buckets,
                             std::unordered_map<NodeID, EdgeMeters> &node_meters) const
    {
        const NodeID node = query_heap.DeleteMin();
        const int target_distance = query_heap.GetKey(node);
        const EdgeMeters target_meters = node_meters[node];
        node_meters.erase(node);

        // store settled nodes in search space bucket
        search_space_with_buckets[node].emplace_back(target_id, target_distance, target_meters);

        if (StallAtNode<false>(node, target_distance, query_heap))
        {
            return;
        }

        RelaxOutgoingEdges<false>(node, target_distance, target_meters, query_heap, node_meters);
    }

    template <bool forward_direction>
    inline void
    RelaxOutgoingEdges(const NodeID node, const EdgeWeight distance, const EdgeMeters meters, QueryHeap &query_heap, std::unordered_map<NodeID, EdgeMeters> &node_meters) const
    {
        for (auto edge : super::facade->GetAdjacentEdgeRange(node))
        {
            const auto &data = super::facade->GetEdgeData(edge);
            const bool direction_flag = (forward_direction ? data.forward : data.backward);
            if (direction_flag)
            {
                const NodeID to = super::facade->GetTarget(edge);
                const int edge_weight = data.distance;

                const EdgeMeters edge_meters = data.meters;

                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                const int to_distance = distance + edge_weight;
                const EdgeMeters to_meters = meters + edge_meters;

                BOOST_ASSERT(to_distance>=0);
                BOOST_ASSERT(to_meters>=0);

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    query_heap.Insert(to, to_distance, node);
                    BOOST_ASSERT(node_meters.count(to)==0);
                    node_meters[to] = to_meters;
                }
                // Found a shorter Path -> Update distance
                else if (to_distance < query_heap.GetKey(to))
                {
                    // new parent
                    query_heap.GetData(to).parent = node;
                    query_heap.DecreaseKey(to, to_distance);
                    BOOST_ASSERT(node_meters.count(to)==1);
                    node_meters[to] = to_meters;
                }
            }
        }
    }

    // Stalling
    template <bool forward_direction>
    inline bool
    StallAtNode(const NodeID node, const EdgeWeight distance, QueryHeap &query_heap) const
    {
        for (auto edge : super::facade->GetAdjacentEdgeRange(node))
        {
            const auto &data = super::facade->GetEdgeData(edge);
            const bool reverse_flag = ((!forward_direction) ? data.forward : data.backward);
            if (reverse_flag)
            {
                const NodeID to = super::facade->GetTarget(edge);
                const int edge_weight = data.distance;
                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                if (query_heap.WasInserted(to))
                {
                    if (query_heap.GetKey(to) + edge_weight < distance)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
};
#endif
