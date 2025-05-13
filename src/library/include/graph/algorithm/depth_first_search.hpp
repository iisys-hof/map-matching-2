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

#ifndef MAP_MATCHING_2_GRAPH_ALGORITHM_DEPTH_FIRST_SEARCH_HPP
#define MAP_MATCHING_2_GRAPH_ALGORITHM_DEPTH_FIRST_SEARCH_HPP

#include <stack>

#include "graph/concepts.hpp"

#include "visitor.hpp"

namespace map_matching_2::graph {

    template<typename VertexDescriptor>
    using stack_type = std::stack<VertexDescriptor>;

    template<typename Graph, typename Visited, typename Stack, typename Visitor>
    void depth_first_search(const Graph &graph, typename Graph::vertex_descriptor start,
            Visited &visited, Stack &stack, Visitor &visitor) requires
        (is_adjacency_list<Graph> or is_compressed_sparse_row<Graph>) {
        const auto start_index = graph.vertex_index(start);
        visited.emplace(start_index);
        visitor.discover_vertex(graph, start);
        stack.push(start);

        while (not stack.empty() and not visitor.stopped()) {
            const auto vertex_desc = stack.top();
            stack.pop();
            visitor.examine_vertex(graph, vertex_desc);

            const auto out_edges_v = graph.out_edges_view(vertex_desc);
            if (not out_edges_v.empty()) {
                auto oe_it = out_edges_v.end();
                do {
                    oe_it = out_edges_v.prev(oe_it);

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
                        stack.push(target_desc);
                    }
                } while (oe_it != out_edges_v.begin());
            }

            visitor.finish_vertex(graph, vertex_desc);
        }
    }

}

#endif //MAP_MATCHING_2_GRAPH_ALGORITHM_DEPTH_FIRST_SEARCH_HPP
