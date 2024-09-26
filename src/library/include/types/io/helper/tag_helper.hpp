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

#ifndef MAP_MATCHING_2_TYPES_IO_HELPER_TAG_HELPER_HPP
#define MAP_MATCHING_2_TYPES_IO_HELPER_TAG_HELPER_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/storage/tag_storage.hpp"

#include "io/helper/tag_helper.hpp"

#define MM2_TAG_HELPER(TYPES) map_matching_2::io::helper::tag_helper<MM2_TAG_STORAGE(TYPES)>
#define MM2_IMPORT_TAG_HELPER(TYPES) map_matching_2::io::helper::tag_helper<MM2_IMPORT_TAG_STORAGE(TYPES)>
#define MM2_TAG_HELPER_DUMMY map_matching_2::io::helper::tag_helper_dummy

#ifdef EXPLICIT_TEMPLATES

#define MM2_EXTERNAL_TAG_HELPER_TEMPLATE(TYPES) \
    MM2_EXTERN template class MM2_TAG_HELPER(TYPES); \
    MM2_EXTERN template class MM2_IMPORT_TAG_HELPER(TYPES);

MM2_EXTERNAL_TAG_HELPER_TEMPLATE(MM2_MMAP_TYPES)

#endif

namespace map_matching_2::io::helper {

    template<memory_mapped::is_types Types = memory_mapped::memory_types>
    using tag_helper_type = tag_helper<memory_mapped::storage::tag_storage<Types,
        typename Types::template allocator_type<void>>>;

    template<memory_mapped::is_types Types = memory_mapped::memory_types>
    using import_tag_helper_type = tag_helper<memory_mapped::storage::tag_storage<Types,
        typename Types::template fast_counting_allocator_type<void>>>;

}

#endif //MAP_MATCHING_2_TYPES_IO_HELPER_TAG_HELPER_HPP
