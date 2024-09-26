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

#ifndef MAP_MATCHING_2_TYPES_MATCHING_MATCHER_HPP
#define MAP_MATCHING_2_TYPES_MATCHING_MATCHER_HPP

#include "types/extern_define.hpp"

#include "types/geometry/network/network.hpp"

#include "matching/matcher.hpp"

#define MM2_MATCHER(NETWORK_TYPE) map_matching_2::matching::matcher<NETWORK_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_MATCHER_TEMPLATE(NETWORK_TYPE) \
    MM2_EXTERN template class MM2_MATCHER(UNPACK NETWORK_TYPE);

#define MM2_MATCHER_TEMPLATE_CS_NETWORK_TYPE(GRAPH_HELPER_TYPE, INDEX_HELPER_TYPES) \
    MM2_MATCHER_TEMPLATE((MM2_NETWORK(UNPACK GRAPH_HELPER_TYPE, UNPACK INDEX_HELPER_TYPES)))

#define MM2_MATCHER_TEMPLATE_CS_INDEX_TYPES(CS, GRAPH_HELPER_TYPE, INDEX_TYPES) \
    MM2_MATCHER_TEMPLATE_CS_NETWORK_TYPE(GRAPH_HELPER_TYPE, (MM2_INDEX_HELPER_READ_ONLY(CS, INDEX_TYPES)))

#define MM2_MATCHER_TEMPLATE_CS_GRAPH_HELPER_TYPE(CS, GRAPH_HELPER_TYPE) \
    MM2_MATCHER_TEMPLATE_CS_INDEX_TYPES(CS, GRAPH_HELPER_TYPE, MM2_MEMORY_TYPES) \
    MM2_MATCHER_TEMPLATE_CS_INDEX_TYPES(CS, GRAPH_HELPER_TYPE, MM2_MMAP_TYPES)

#define MM2_MATCHER_TEMPLATE_CS_GRAPH_TYPES(CS, GRAPH_TYPES) \
    MM2_MATCHER_TEMPLATE_CS_GRAPH_HELPER_TYPE(CS, (MM2_GRAPH_HELPER(CS, GRAPH_TYPES, MM2_TAG_HELPER(MM2_MEMORY_TYPES)))) \
    MM2_MATCHER_TEMPLATE_CS_GRAPH_HELPER_TYPE(CS, (MM2_GRAPH_HELPER(CS, GRAPH_TYPES, MM2_TAG_HELPER(MM2_MMAP_TYPES))))

#define MM2_MATCHER_TEMPLATE_CS(CS) \
    MM2_MATCHER_TEMPLATE_CS_GRAPH_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_MATCHER_TEMPLATE_CS_GRAPH_TYPES(CS, MM2_MMAP_TYPES)

MM2_MATCHER_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_MATCHER_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_MATCHER_TEMPLATE_CS(MM2_CARTESIAN)

#endif

#endif //MAP_MATCHING_2_TYPES_MATCHING_MATCHER_HPP
