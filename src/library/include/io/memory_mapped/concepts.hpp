// Copyright (C) 2023-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_CONCEPTS_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_CONCEPTS_HPP

#include <concepts>

#include "types.hpp"

namespace map_matching_2::io::memory_mapped {

    template<typename Types>
    concept has_mmap_type = requires {
        typename Types::mmap_type;
    };

    template<typename Types>
    concept has_declarations = requires {
        typename Types::template allocator_type<void>;
        typename Types::template fast_allocator_type<void>;
        typename Types::template counting_allocator_type<void>;
        typename Types::template fast_counting_allocator_type<void>;
        typename Types::template allocator_traits_type<typename Types::template allocator_type<void>>;
        typename Types::template vector_type<char>;
        typename Types::template list_type<char>;
    };

    template<typename Types>
    concept is_managed_shared_memory = has_mmap_type<Types> and has_declarations<Types> and
            std::same_as<boost::interprocess::managed_shared_memory, typename Types::mmap_type>;

    template<typename Types>
    concept is_managed_mapped_file = has_mmap_type<Types> and has_declarations<Types> and
            std::same_as<managed_mapped_file, typename Types::mmap_type>;

    template<typename Types>
    concept is_memory_types = has_declarations<Types> and
            std::same_as<Types, memory_types>;

    template<typename Types>
    concept is_mmap_types = has_mmap_type<Types> and has_declarations<Types> and
            std::same_as<Types, mmap_types<typename Types::mmap_type>>;

    template<typename Types>
    concept is_types = is_memory_types<Types> or is_mmap_types<Types>;

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_CONCEPTS_HPP
