// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GRAPH_TRAITS_GRAPH_HPP
#define MAP_MATCHING_2_GRAPH_TRAITS_GRAPH_HPP

#include "graph/concepts.hpp"

namespace map_matching_2::graph {

    template<typename Graph> requires
        (is_adjacency_list<Graph> or is_compressed_sparse_row<Graph>)
    struct graph_traits {
        using graph_type = Graph;

        using allocator_type = typename graph_type::allocator_type;

        template<typename Type>
        using allocator_traits_type = typename graph_type::template allocator_traits_type<Type>;

        using direction_type = typename graph_type::direction_type;

        using vertex_data_type = typename graph_type::vertex_data_type;
        using vertex_type = typename graph_type::vertex_type;
        using vertex_container = typename graph_type::vertex_container;
        using vertex_size_type = typename graph_type::vertex_size_type;
        using vertex_difference_type = typename graph_type::vertex_difference_type;
        using vertex_descriptor = typename graph_type::vertex_descriptor;

        using edge_data_type = typename graph_type::edge_data_type;
        using edge_type = typename graph_type::edge_type;
        using edge_container = typename graph_type::edge_container;
        using edge_size_type = typename graph_type::edge_size_type;
        using edge_difference_type = typename graph_type::edge_difference_type;
        using edge_descriptor = typename graph_type::edge_descriptor;
    };

}

#endif //MAP_MATCHING_2_GRAPH_TRAITS_GRAPH_HPP
