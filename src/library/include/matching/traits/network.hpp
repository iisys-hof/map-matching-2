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

#ifndef MAP_MATCHING_2_MATCHING_TRAITS_NETWORK_HPP
#define MAP_MATCHING_2_MATCHING_TRAITS_NETWORK_HPP

#include <cstddef>
#include <utility>
#include <vector>

#include "types/geometry/network/route/lazy_route.hpp"
#include "types/geometry/rich_type/lazy_multi_rich_line.hpp"
#include "types/geometry/track/multi_track.hpp"
#include "types/matching/candidate/candidate.hpp"

#include "geometry/network/traits/network.hpp"

namespace map_matching_2::matching {

    template<typename Network>
    struct network_traits {
        using network_type = Network;

        using vertex_descriptor = typename geometry::network::network_traits<network_type>::vertex_descriptor;
        using edge_descriptor = typename geometry::network::network_traits<network_type>::edge_descriptor;

        using vertex_data_type = typename geometry::network::network_traits<network_type>::vertex_data_type;
        using edge_data_type = typename geometry::network::network_traits<network_type>::edge_data_type;

        using coordinate_system_type = typename geometry::network::network_traits<network_type>::coordinate_system_type;
        using point_type = typename geometry::network::network_traits<network_type>::point_type;
        using time_point_type = typename geometry::models<point_type>::explicit_time_point_type;

        using multi_track_type = geometry::track::multi_track_type<time_point_type>;
        using multi_track_multi_rich_line_type = typename multi_track_type::multi_rich_line_type;
        using track_type = typename multi_track_type::track_type;
        using track_rich_line_type = typename track_type::rich_line_type;

        using candidate_type = candidate<vertex_descriptor, edge_descriptor, track_type>;
        using candidate_node_type = typename candidate_type::candidate_node_type;
        using candidate_edge_type = typename candidate_type::candidate_edge_type;

        using distance_type = typename geometry::data<point_type>::distance_type;
        using length_type = typename geometry::data<point_type>::length_type;
        using angle_type = typename geometry::data<point_type>::angle_type;

        using route_type = geometry::network::lazy_route_type<coordinate_system_type>;

        using lazy_multi_rich_line_type = geometry::lazy_multi_rich_line_type<point_type>;
        using lazy_rich_line_type = typename geometry::multi_rich_line_traits<
            lazy_multi_rich_line_type>::rich_line_type;

        using policy_element_type = std::pair<std::size_t, std::size_t>;
        using policy_type = std::vector<policy_element_type>;
    };

}

#endif //MAP_MATCHING_2_MATCHING_TRAITS_NETWORK_HPP
