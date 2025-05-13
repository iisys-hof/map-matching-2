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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_INDEX_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_INDEX_HPP

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
#define BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
#endif

#include <cstddef>
#include <utility>

#include <boost/serialization/serialization.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"

#include "io/memory_mapped/concepts.hpp"

#include "util/macros.hpp"

#define MM2_RTREE_RSTAR_PARAMETERS(ELEMENTS) boost::geometry::index::rstar<ELEMENTS>
#define MM2_RTREE_INDEXABLE(TYPE) boost::geometry::index::indexable<TYPE>
#define MM2_RTREE_EQUAL_TO(TYPE) boost::geometry::index::equal_to<TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_RTREE_RSTAR_PARAMETERS_TEMPLATE(ELEMENTS) \
    MM2_EXTERN template class MM2_RTREE_RSTAR_PARAMETERS(ELEMENTS);

MM2_RTREE_RSTAR_PARAMETERS_TEMPLATE(16)

#endif

namespace map_matching_2::geometry {

    template<std::size_t Size>
    using rtree_rstar_parameters = MM2_RTREE_RSTAR_PARAMETERS(Size);

    template<typename Type>
    using rtree_indexable = MM2_RTREE_INDEXABLE(Type);

    template<typename Type>
    using rtree_equal_to = MM2_RTREE_EQUAL_TO(Type);

}

#define MM2_RTREE(TYPE, TYPES) boost::geometry::index::rtree< \
    UNPACK TYPE, \
    map_matching_2::geometry::rtree_rstar_parameters<16>, \
    map_matching_2::geometry::rtree_indexable<UNPACK TYPE>, \
    map_matching_2::geometry::rtree_equal_to<UNPACK TYPE>, \
    typename MM2_ALLOCATOR(TYPES)<UNPACK TYPE>>

namespace map_matching_2::geometry {

    template<typename Type, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using rtree_type = MM2_RTREE((Type), Types);

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_INDEX_HPP
