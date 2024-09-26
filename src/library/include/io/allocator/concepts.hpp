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

#ifndef MAP_MATCHING_2_IO_ALLOCATOR_CONCEPTS_HPP
#define MAP_MATCHING_2_IO_ALLOCATOR_CONCEPTS_HPP

#include <concepts>
#include <type_traits>
#include <memory_resource>

namespace map_matching_2::io::allocator {

    template<typename Allocator>
    concept is_counting_allocator = requires(Allocator &a) {
        typename Allocator::allocation_counter_type;
        { Allocator{std::declval<typename Allocator::allocation_counter_type>()} } -> std::same_as<Allocator>;
    };

    template<typename Allocator>
    concept is_counting_polymorphic_allocator = requires(Allocator &a) {
        requires is_counting_allocator<Allocator>;
        {
            Allocator{
                    std::declval<std::pmr::memory_resource *>(),
                    std::declval<typename Allocator::allocation_counter_type>()
            }
        } -> std::same_as<Allocator>;
    };

    template<typename Allocator>
    concept is_counting_interprocess_allocator = requires(Allocator &a) {
        typename Allocator::allocation_counter_type;
        typename Allocator::segment_manager;
        {
            Allocator{
                    std::declval<typename Allocator::segment_manager *>(),
                    std::declval<typename Allocator::allocation_counter_type>()
            }
        } -> std::same_as<Allocator>;
    };

}

#endif //MAP_MATCHING_2_IO_ALLOCATOR_CONCEPTS_HPP
