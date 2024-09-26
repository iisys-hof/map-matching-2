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

#ifndef MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_NODE_HPP
#define MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_NODE_HPP

#include <cstddef>

#include "types/extern_define.hpp"

#include "matching/candidate/candidate_node.hpp"

#define MM2_CANDIDATE_NODE(VERTEX_DESCRIPTOR_TYPE, DISTANCE_TYPE) \
    map_matching_2::matching::candidate_node<VERTEX_DESCRIPTOR_TYPE, DISTANCE_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_CANDIDATE_NODE_TEMPLATE(VERTEX_DESCRIPTOR_TYPE, DISTANCE_TYPE) \
    MM2_EXTERN template class MM2_CANDIDATE_NODE(VERTEX_DESCRIPTOR_TYPE, DISTANCE_TYPE);

#define MM2_CANDIDATE_NODE_TEMPLATE_DISTANCE_TYPE(DISTANCE_TYPE) \
    MM2_CANDIDATE_NODE_TEMPLATE(std::size_t, DISTANCE_TYPE)

MM2_CANDIDATE_NODE_TEMPLATE_DISTANCE_TYPE(float)
MM2_CANDIDATE_NODE_TEMPLATE_DISTANCE_TYPE(double)
MM2_CANDIDATE_NODE_TEMPLATE_DISTANCE_TYPE(long double)

#endif

#endif //MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_NODE_HPP
