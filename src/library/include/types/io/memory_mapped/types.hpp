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

#ifndef MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_TYPES_HPP
#define MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_TYPES_HPP

#include "types/extern_define.hpp"

#include "io/memory_mapped/types.hpp"

#ifdef EXPLICIT_TEMPLATES

namespace boost::interprocess {

#define MM2_INTERPROCESS_TEMPLATES \
    MM2_EXTERN template class rbtree_best_fit<mutex_family>; \
    MM2_EXTERN template class rbtree_best_fit<null_mutex_family>; \
    MM2_EXTERN template class basic_managed_mapped_file<char, rbtree_best_fit<mutex_family>, flat_map_index>; \
    MM2_EXTERN template class basic_managed_mapped_file<char, rbtree_best_fit<null_mutex_family>, flat_map_index>;

    MM2_INTERPROCESS_TEMPLATES

}

#endif

#define MM2_MEMORY_TYPES map_matching_2::io::memory_mapped::memory_types
#define MM2_MMAP_TYPES map_matching_2::io::memory_mapped::file_types

#define MM2_VECTOR(TYPES) TYPES::template vector_type
#define MM2_LIST(TYPES) TYPES::template list_type
#define MM2_ALLOCATOR(TYPES) TYPES::template allocator_type
#define MM2_COUNTING_ALLOCATOR(TYPES) TYPES::template counting_allocator_type
#define MM2_FAST_COUNTING_ALLOCATOR(TYPES) TYPES::template fast_counting_allocator_type

#endif //MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_TYPES_HPP
