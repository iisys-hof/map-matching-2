// Copyright (C) 2023-2025 Adrian Wöltche
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see https://www.gnu.org/licenses/.

#ifndef MAP_MATCHING_2_GRAPH_ALGORITHM_DIJKSTRA_SHORTEST_PATH_HPP
#define MAP_MATCHING_2_GRAPH_ALGORITHM_DIJKSTRA_SHORTEST_PATH_HPP

#include <queue>

#include <boost/heap/d_ary_heap.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

#include "graph/common.hpp"
#include "graph/concepts.hpp"

#include "visitor.hpp"
#include "priority_type_comparator.hpp"

namespace map_matching_2::graph {

    template<typename Weight, typename VertexDescriptor>
    using priority_type = std::pair<Weight, VertexDescriptor>;

    using predecessors_type = boost::unordered_flat_map<std::size_t, std::size_t>;

    template<typename Weight>
    using distances_type = boost::unordered_flat_map<std::size_t, Weight>;

    using colors_type = boost::unordered_flat_map<std::size_t, graph::color>;

    template<typename VertexDescriptor>
    using queue_type = std::queue<VertexDescriptor>;

    // template<typename Priority>
    // using priority_queue_type = std::priority_queue<Priority,
    //     std::vector<Priority>, priority_type_comparator<Priority>>;

    template<typename Priority>
    using priority_queue_type = boost::heap::d_ary_heap<Priority, boost::heap::arity<4>,
        boost::heap::compare<priority_type_comparator<Priority>>>;

    template<typename Graph, typename Predecessors, typename Distances, typename Colors, typename Queue,
        typename Visitor, typename WeightFunc>
    void dijkstra_shortest_path(const Graph &graph, typename Graph::vertex_descriptor start,
            Predecessors &predecessors, Distances &distances, Colors &colors, Queue &queue, Visitor &visitor,
            const WeightFunc &weight_func) requires
        (is_adjacency_list<Graph> or is_compressed_sparse_row<Graph>) {
        const auto start_index = graph.vertex_index(start);
        predecessors[start_index] = start_index;
        distances[start_index] = 0;
        colors[start_index] = color::GRAY;
        visitor.discover_vertex(graph, start);
        queue.push({0, start});

        while (not queue.empty() and not visitor.stopped()) {
            const auto [distance, vertex_desc] = queue.top();
            queue.pop();
            const auto vertex_index = graph.vertex_index(vertex_desc);

            visitor.examine_vertex(graph, vertex_desc);

            const auto out_edges_v = graph.out_edges_view(vertex_desc);
            for (auto oe_it = out_edges_v.begin(); oe_it != out_edges_v.end(); oe_it = out_edges_v.next(oe_it)) {
                const typename Graph::edge_descriptor *edge_desc;
                if constexpr (is_adjacency_list<Graph>) {
                    edge_desc = &(out_edges_v[oe_it]);
                } else {
                    edge_desc = &oe_it;
                }

                visitor.examine_edge(graph, *edge_desc);

                const auto &target_desc = graph.target(*edge_desc);
                const auto target_index = graph.vertex_index(target_desc);

                if (not colors.contains(target_index) or colors[target_index] != color::BLACK) {
                    const auto edge_distance = weight_func(graph.get_edge_data(*edge_desc));
                    const auto target_distance = distance + edge_distance;

                    if (not distances.contains(target_index) or distances[target_index] > target_distance) {
                        predecessors[target_index] = vertex_index;
                        distances[target_index] = target_distance;
                        colors[target_index] = color::GRAY;
                        visitor.discover_vertex(graph, target_desc);
                        queue.push({target_distance, target_desc});
                    }
                }
            }

            colors[vertex_index] = color::BLACK;
            visitor.finish_vertex(graph, vertex_desc);
        }
    }

}

#endif //MAP_MATCHING_2_GRAPH_ALGORITHM_DIJKSTRA_SHORTEST_PATH_HPP
