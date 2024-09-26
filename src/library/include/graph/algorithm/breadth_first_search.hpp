// Copyright (C) 2023-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GRAPH_ALGORITHM_BREADTH_FIRST_SEARCH_HPP
#define MAP_MATCHING_2_GRAPH_ALGORITHM_BREADTH_FIRST_SEARCH_HPP

#include <queue>

#include "graph/concepts.hpp"

#include "visitor.hpp"

namespace map_matching_2::graph {

    template<typename VertexDescriptor>
    using queue_type = std::queue<VertexDescriptor>;

    template<typename Graph, typename Visited, typename Queue, typename Visitor>
    void breadth_first_search(const Graph &graph, typename Graph::vertex_descriptor start,
            Visited &visited, Queue &queue, Visitor &visitor) requires
        (is_adjacency_list<Graph> or is_compressed_sparse_row<Graph>) {
        const auto start_index = graph.vertex_index(start);
        visited.emplace(start_index);
        visitor.discover_vertex(graph, start);
        queue.push(start);

        while (not queue.empty() and not visitor.stopped()) {
            const auto vertex_desc = queue.front();
            queue.pop();
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

                if (not visited.contains(target_index)) {
                    visited.emplace(target_index);
                    visitor.discover_vertex(graph, target_desc);
                    queue.push(target_desc);
                }
            }

            visitor.finish_vertex(graph, vertex_desc);
        }
    }

}

#endif //MAP_MATCHING_2_GRAPH_ALGORITHM_BREADTH_FIRST_SEARCH_HPP
