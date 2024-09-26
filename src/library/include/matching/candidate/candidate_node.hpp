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

#ifndef MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_NODE_HPP
#define MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_NODE_HPP

#include <utility>

namespace map_matching_2::matching {

    template<typename VertexDescriptor, typename Distance>
    struct candidate_node {

        using vertex_descriptor = VertexDescriptor;
        using distance_type = Distance;

        vertex_descriptor vertex_desc;
        distance_type distance;

        constexpr candidate_node(vertex_descriptor vertex_desc, distance_type distance)
            : vertex_desc{std::move(vertex_desc)}, distance{distance} {}

        constexpr candidate_node(const candidate_node &other) = default;

        constexpr candidate_node(candidate_node &&other) noexcept = default;

        constexpr candidate_node &operator=(const candidate_node &other) = default;

        constexpr candidate_node &operator=(candidate_node &&other) noexcept = default;

        constexpr ~candidate_node() = default;

        friend constexpr bool operator==(const candidate_node &left, const candidate_node &right) {
            return left.vertex_desc == right.vertex_desc and left.distance == right.distance;
        }

        friend constexpr bool operator!=(const candidate_node &left, const candidate_node &right) {
            return not(left == right);
        }

        friend constexpr bool operator<(const candidate_node &left, const candidate_node &right) {
            return left.distance < right.distance;
        }

        friend constexpr bool operator<=(const candidate_node &left, const candidate_node &right) {
            return not(right < left);
        }

        friend constexpr bool operator>(const candidate_node &left, const candidate_node &right) {
            return right < left;
        }

        friend constexpr bool operator>=(const candidate_node &left, const candidate_node &right) {
            return not(left < right);
        }

    };

}

#endif //MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_NODE_HPP
