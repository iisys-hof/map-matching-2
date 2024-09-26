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

#ifndef MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_BASE_STORAGE_HPP
#define MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_BASE_STORAGE_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"

#include "io/memory_mapped/storage/base_storage.hpp"

#define MM2_MMAP_BASE_CONTAINER(TYPES) map_matching_2::io::memory_mapped::storage::mmap_base_container<TYPES>
#define MM2_MMAP_CONTAINER(TYPES) map_matching_2::io::memory_mapped::storage::mmap_container<TYPES>
#define MM2_MMAP_BASE_DATA(TYPES) map_matching_2::io::memory_mapped::storage::mmap_base_data<TYPES>
#define MM2_MMAP_BASE(TYPES) map_matching_2::io::memory_mapped::storage::mmap_base<TYPES>

#ifdef EXPLICIT_TEMPLATES

#define MM2_MMAP_BASE_CONTAINER_TEMPLATE(TYPES) \
    MM2_EXTERN template class MM2_MMAP_BASE_CONTAINER(TYPES);

#define MM2_MMAP_CONTAINER_TEMPLATE(TYPES) \
    MM2_EXTERN template class MM2_MMAP_CONTAINER(TYPES);

#define MM2_MMAP_BASE_DATA_TEMPLATE(TYPES) \
    MM2_EXTERN template class MM2_MMAP_BASE_DATA(TYPES);

#define MM2_MMAP_BASE_TEMPLATE(TYPES) \
    MM2_EXTERN template class MM2_MMAP_BASE(TYPES);

MM2_MMAP_BASE_CONTAINER_TEMPLATE(MM2_MMAP_TYPES)
MM2_MMAP_CONTAINER_TEMPLATE(MM2_MMAP_TYPES)
MM2_MMAP_BASE_DATA_TEMPLATE(MM2_MMAP_TYPES)
MM2_MMAP_BASE_TEMPLATE(MM2_MMAP_TYPES)

#endif

#endif //MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_BASE_STORAGE_HPP
