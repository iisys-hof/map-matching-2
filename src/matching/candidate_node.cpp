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

#include "candidate_node.hpp"

#include <geometry/network/types.hpp>

namespace map_matching_2::matching {

    template<typename Network>
    candidate_node<Network>::candidate_node(vertex_descriptor_type vertex_descriptor, distance_type distance)
            : vertex_descriptor{vertex_descriptor}, distance{distance} {}

    template<typename Network>
    bool candidate_node<Network>::distance_comparator(const candidate_node &left, const candidate_node &right) {
        return left.distance < right.distance;
    }

    template
    class candidate_node<geometry::network::types_geographic::network_static>;

    template
    class candidate_node<geometry::network::types_cartesian::network_static>;

}
