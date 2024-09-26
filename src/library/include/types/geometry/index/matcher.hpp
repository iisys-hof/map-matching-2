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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_MATCHER_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_MATCHER_HPP

#include <functional>

#include "types/extern_define.hpp"

#include "types/matching/candidate/candidate_edge.hpp"

#include "index.hpp"

#include "util/macros.hpp"

namespace map_matching_2::geometry {

    template<typename CandidateEdge>
    using matcher_rtree_value_type = std::pair<typename models<typename CandidateEdge::point_type>::segment_type,
        std::pair<std::size_t, std::reference_wrapper<const CandidateEdge>>>;

}

#define MM2_MATCHER_RTREE(CANDIDATE_EDGE_TYPE) \
    MM2_RTREE((map_matching_2::geometry::matcher_rtree_value_type<UNPACK CANDIDATE_EDGE_TYPE>), MM2_MEMORY_TYPES)

#ifdef EXPLICIT_TEMPLATES

#define MM2_MATCHER_RTREE_TEMPLATE(CANDIDATE_EDGE_TYPE) \
    MM2_EXTERN template class MM2_MATCHER_RTREE(CANDIDATE_EDGE_TYPE);

#define MM2_MATCHER_RTREE_TEMPLATE_CS(CS) \
    MM2_MATCHER_RTREE_TEMPLATE((MM2_CANDIDATE_EDGE(std::size_t, MM2_POINT(CS))))

MM2_MATCHER_RTREE_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_MATCHER_RTREE_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_MATCHER_RTREE_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    template<typename CandidateEdge>
    using matcher_rtree_type = rtree_type<matcher_rtree_value_type<CandidateEdge>, io::memory_mapped::memory_types>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_MATCHER_HPP
