// Copyright (C) 2023-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_HELPER_CONCEPTS_HPP
#define MAP_MATCHING_2_IO_HELPER_CONCEPTS_HPP

#include "io/memory_mapped/concepts.hpp"

namespace map_matching_2::io::helper {

    template<typename Types>
    concept has_mmap_types = requires {
        typename Types::mmap_types;
    };

    template<typename Types>
    concept has_memory_types = requires {
        typename Types::memory_types;
    };

    template<typename Types>
    concept is_memory_types = has_memory_types<Types> and
            io::memory_mapped::is_memory_types<typename Types::memory_types>;

    template<typename Types>
    concept is_mmap_types = has_mmap_types<Types> and
            io::memory_mapped::is_mmap_types<typename Types::mmap_types>;

}

#endif //MAP_MATCHING_2_IO_HELPER_CONCEPTS_HPP
