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

#ifndef MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_INDEX_STORAGE_HPP
#define MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_INDEX_STORAGE_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"
#include "types/geometry/point.hpp"

#include "io/memory_mapped/storage/index_storage.hpp"

#define MM2_INDEX_STORAGE(CS, TYPES) map_matching_2::io::memory_mapped::storage::index_storage<MM2_POINT(CS), TYPES>

#ifdef EXPLICIT_TEMPLATES

#define MM2_INDEX_STORAGE_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_INDEX_STORAGE(CS, TYPES);

#define MM2_INDEX_STORAGE_TEMPLATE_CS(CS) \
    MM2_INDEX_STORAGE_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_INDEX_STORAGE_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_INDEX_STORAGE_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_INDEX_STORAGE_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_INDEX_STORAGE_TEMPLATE_CS(MM2_CARTESIAN)

#endif

#endif //MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_INDEX_STORAGE_HPP
