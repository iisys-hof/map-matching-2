// Copyright (C) 2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GRAPH_CONCEPTS_HPP
#define MAP_MATCHING_2_GRAPH_CONCEPTS_HPP

namespace map_matching_2::graph {

    template<typename Graph>
    concept is_adjacency_list = requires(Graph &graph) {
        typename Graph::vertex_container;
        typename Graph::edge_container;
        typename Graph::out_edge_container;
    };

    template<typename Graph>
    concept is_compressed_sparse_row = not is_adjacency_list<Graph> and requires(Graph &graph) {
        typename Graph::vertex_container;
        typename Graph::edge_container;
    };

}

#endif //MAP_MATCHING_2_GRAPH_CONCEPTS_HPP
