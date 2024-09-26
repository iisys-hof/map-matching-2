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

#ifndef MAP_MATCHING_2_TYPES_IO_HELPER_INDEX_HELPER_HPP
#define MAP_MATCHING_2_TYPES_IO_HELPER_INDEX_HELPER_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/storage/index_storage.hpp"
#include "types/io/helper/index_build_helper.hpp"

#include "io/helper/index_helper.hpp"

#define MM2_INDEX_HELPER(CS, STORAGE_TYPES, BUILD_TYPES) map_matching_2::io::helper::index_helper< \
    MM2_INDEX_STORAGE(CS, STORAGE_TYPES), MM2_INDEX_BUILD_HELPER(CS, BUILD_TYPES)>

#define MM2_INDEX_HELPER_READ_ONLY(CS, STORAGE_TYPES) map_matching_2::io::helper::index_helper< \
    MM2_INDEX_STORAGE(CS, STORAGE_TYPES), MM2_INDEX_BUILD_HELPER_DUMMY>

#ifdef EXPLICIT_TEMPLATES

#define MM2_INDEX_HELPER_TEMPLATE(CS, TYPES, BUILD_TYPES) \
    MM2_EXTERN template class MM2_INDEX_HELPER(CS, TYPES, BUILD_TYPES);

#define MM2_INDEX_HELPER_TEMPLATE_CS_TYPES(CS, TYPES) \
    MM2_INDEX_HELPER_TEMPLATE(CS, TYPES, MM2_MEMORY_TYPES) \
    MM2_INDEX_HELPER_TEMPLATE(CS, TYPES, MM2_MMAP_TYPES)

#define MM2_INDEX_HELPER_TEMPLATE_CS(CS) \
    MM2_INDEX_HELPER_TEMPLATE_CS_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_INDEX_HELPER_TEMPLATE_CS_TYPES(CS, MM2_MMAP_TYPES)

MM2_INDEX_HELPER_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_INDEX_HELPER_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_INDEX_HELPER_TEMPLATE_CS(MM2_CARTESIAN)

#define MM2_INDEX_HELPER_READ_ONLY_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_INDEX_HELPER_READ_ONLY(CS, TYPES);

#define MM2_INDEX_HELPER_READ_ONLY_TEMPLATE_CS(CS) \
    MM2_INDEX_HELPER_READ_ONLY_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_INDEX_HELPER_READ_ONLY_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_INDEX_HELPER_READ_ONLY_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_INDEX_HELPER_READ_ONLY_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_INDEX_HELPER_READ_ONLY_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::io::helper {

    template<geometry::is_point Point,
        memory_mapped::is_types StorageTypes = memory_mapped::memory_types,
        memory_mapped::is_types BuildTypes = memory_mapped::memory_types>
    using index_helper_type = index_helper<
        memory_mapped::storage::index_storage<Point, StorageTypes>, index_build_helper_type<Point, BuildTypes>>;

    template<geometry::is_point Point, memory_mapped::is_types Types = memory_mapped::memory_types>
    using index_helper_read_only_type = index_helper<
        memory_mapped::storage::index_storage<Point, Types>, index_build_helper_dummy>;

}

#endif //MAP_MATCHING_2_TYPES_IO_HELPER_INDEX_HELPER_HPP
