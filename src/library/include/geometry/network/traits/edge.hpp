// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TRAITS_EDGE_HPP
#define MAP_MATCHING_2_TRAITS_EDGE_HPP

#include <concepts>
#include <osmium/osm/types.hpp>

namespace map_matching_2::geometry::network {

    template<typename Edge>
    concept is_edge = requires(Edge edge) {
        requires std::same_as<decltype(edge.id), osmium::object_id_type>;
        requires std::same_as<decltype(edge.rich_line), typename Edge::rich_line_type>;
        requires std::same_as<decltype(edge.tags), typename Edge::tags_container_type>;
    };

    template<is_edge Edge>
    struct edge_traits {
        using edge_type = Edge;
        using rich_line_type = typename edge_type::rich_line_type;
        using tag_type = typename edge_type::tag_type;
        using allocator_type = typename edge_type::allocator_type;
        using tags_container_type = typename edge_type::tags_container_type;
    };

}

#endif //MAP_MATCHING_2_TRAITS_EDGE_HPP
