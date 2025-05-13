// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_EDGE_HPP
#define MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_EDGE_HPP

#include <cstddef>

#include "types/extern_define.hpp"

#include "matching/candidate/candidate_edge.hpp"

#include "util/macros.hpp"

#define MM2_CANDIDATE_EDGE(EDGE_DESCRIPTOR_TYPE, POINT_TYPE) \
    map_matching_2::matching::candidate_edge<EDGE_DESCRIPTOR_TYPE, POINT_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_CANDIDATE_EDGE_TEMPLATE(EDGE_DESCRIPTOR_TYPE, POINT_TYPE) \
    MM2_EXTERN template class MM2_CANDIDATE_EDGE(EDGE_DESCRIPTOR_TYPE, UNPACK POINT_TYPE);

#define MM2_CANDIDATE_EDGE_TEMPLATE_CS(CS) \
    MM2_CANDIDATE_EDGE_TEMPLATE(std::size_t, (MM2_POINT(CS)))

MM2_CANDIDATE_EDGE_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_CANDIDATE_EDGE_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_CANDIDATE_EDGE_TEMPLATE_CS(MM2_CARTESIAN)

#endif

#endif //MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_EDGE_HPP
