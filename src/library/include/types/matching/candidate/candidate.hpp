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

#ifndef MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_HPP
#define MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_HPP

#include <cstddef>

#include "types/extern_define.hpp"

#include "types/geometry/track/track.hpp"

#include "matching/candidate/candidate.hpp"

#define MM2_CANDIDATE(VERTEX_DESCRIPTOR_TYPE, EDGE_DESCRIPTOR_TYPE, TRACK_TYPE) \
    map_matching_2::matching::candidate<VERTEX_DESCRIPTOR_TYPE, EDGE_DESCRIPTOR_TYPE, TRACK_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_CANDIDATE_TEMPLATE(VERTEX_DESCRIPTOR_TYPE, EDGE_DESCRIPTOR_TYPE, TRACK_TYPE) \
    MM2_EXTERN template class MM2_CANDIDATE(VERTEX_DESCRIPTOR_TYPE, EDGE_DESCRIPTOR_TYPE, UNPACK TRACK_TYPE);

#define MM2_CANDIDATE_TEMPLATE_CS(CS) \
    MM2_CANDIDATE_TEMPLATE(std::size_t, std::size_t, (MM2_TRACK(CS)))

MM2_CANDIDATE_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_CANDIDATE_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_CANDIDATE_TEMPLATE_CS(MM2_CARTESIAN)

#endif

#endif //MAP_MATCHING_2_TYPES_MATCHING_CANDIDATE_CANDIDATE_HPP
