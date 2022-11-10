// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_CANDIDATE_NODE_HPP
#define MAP_MATCHING_2_CANDIDATE_NODE_HPP

#include <geometry/network/network.hpp>

namespace map_matching_2::matching {

    template<typename Network>
    struct candidate_node {

        using vertex_descriptor_type = typename boost::graph_traits<typename Network::graph_type>::vertex_descriptor;
        using distance_type = typename boost::geometry::default_distance_result<typename Network::point_type, typename Network::point_type>::type;

        vertex_descriptor_type vertex_descriptor;
        distance_type distance;

        candidate_node(vertex_descriptor_type vertex_descriptor, distance_type distance)
                : vertex_descriptor{vertex_descriptor}, distance{distance} {}

        ~candidate_node() = default;

        candidate_node(const candidate_node &other) = default;

        candidate_node(candidate_node &&other) noexcept = default;

        candidate_node &operator=(const candidate_node &other) = default;

        candidate_node &operator=(candidate_node &&other) noexcept = default;

        [[nodiscard]] static bool distance_comparator(const candidate_node &left, const candidate_node &right) {
            return left.distance < right.distance;
        }

    };

}

#endif //MAP_MATCHING_2_CANDIDATE_NODE_HPP
