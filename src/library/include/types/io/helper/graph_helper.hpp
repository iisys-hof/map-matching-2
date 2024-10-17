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

#ifndef MAP_MATCHING_2_TYPES_IO_HELPER_GRAPH_HELPER_HPP
#define MAP_MATCHING_2_TYPES_IO_HELPER_GRAPH_HELPER_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/storage/graph_storage.hpp"
#include "types/io/helper/tag_helper.hpp"

#include "io/helper/graph_helper.hpp"

#define MM2_GRAPH_HELPER_IMPORT(CS, TYPES, TAG_HELPER_TYPE) map_matching_2::io::helper::graph_helper< \
    MM2_GRAPH_STORAGE_IMPORT(CS, TYPES), TAG_HELPER_TYPE>

#define MM2_GRAPH_HELPER(CS, TYPES, TAG_HELPER_TYPE) map_matching_2::io::helper::graph_helper< \
    MM2_GRAPH_STORAGE(CS, TYPES), TAG_HELPER_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_GRAPH_HELPER_IMPORT_TEMPLATE(CS, TYPES, TAG_HELPER_TYPE) \
    MM2_EXTERN template class MM2_GRAPH_HELPER_IMPORT(CS, TYPES, UNPACK TAG_HELPER_TYPE);

#define MM2_GRAPH_HELPER_IMPORT_TEMPLATE_CS_TYPES(CS, TYPES) \
    MM2_GRAPH_HELPER_IMPORT_TEMPLATE(CS, TYPES, (MM2_IMPORT_TAG_HELPER(MM2_MEMORY_TYPES))) \
    MM2_GRAPH_HELPER_IMPORT_TEMPLATE(CS, TYPES, (MM2_IMPORT_TAG_HELPER(MM2_MMAP_TYPES)))

#define MM2_GRAPH_HELPER_IMPORT_TEMPLATE_CS(CS) \
    MM2_GRAPH_HELPER_IMPORT_TEMPLATE_CS_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_GRAPH_HELPER_IMPORT_TEMPLATE_CS_TYPES(CS, MM2_MMAP_TYPES)

MM2_GRAPH_HELPER_IMPORT_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_GRAPH_HELPER_IMPORT_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_GRAPH_HELPER_IMPORT_TEMPLATE_CS(MM2_CARTESIAN)

#define MM2_GRAPH_HELPER_TEMPLATE(CS, TYPES, TAG_HELPER_TYPE) \
    MM2_EXTERN template class MM2_GRAPH_HELPER(CS, TYPES, UNPACK TAG_HELPER_TYPE);

#define MM2_GRAPH_HELPER_TEMPLATE_CS_TYPES(CS, TYPES) \
    MM2_GRAPH_HELPER_TEMPLATE(CS, TYPES, (MM2_TAG_HELPER(MM2_MEMORY_TYPES))) \
    MM2_GRAPH_HELPER_TEMPLATE(CS, TYPES, (MM2_TAG_HELPER(MM2_MMAP_TYPES)))

#define MM2_GRAPH_HELPER_TEMPLATE_CS(CS) \
    MM2_GRAPH_HELPER_TEMPLATE_CS_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_GRAPH_HELPER_TEMPLATE_CS_TYPES(CS, MM2_MMAP_TYPES)

MM2_GRAPH_HELPER_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_GRAPH_HELPER_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_GRAPH_HELPER_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::io::helper {

    template<geometry::is_point Point,
        memory_mapped::is_types Types = memory_mapped::memory_types,
        typename TagHelper = import_tag_helper_type<>>
    using graph_helper_import_type = graph_helper<
        memory_mapped::storage::graph_storage_import_type<Point, Types>, TagHelper>;

    template<geometry::is_point Point,
        memory_mapped::is_types Types = memory_mapped::memory_types,
        typename TagHelper = tag_helper_type<>>
    using graph_helper_type = graph_helper<
        memory_mapped::storage::graph_storage_type<Point, Types>, TagHelper>;

}

#endif //MAP_MATCHING_2_TYPES_IO_HELPER_GRAPH_HELPER_HPP