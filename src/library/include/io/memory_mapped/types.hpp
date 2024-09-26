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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_TYPES_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_TYPES_HPP

#include <vector>
#include <list>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/interprocess/indexes/flat_map_index.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/list.hpp>

#include "io/allocator/allocation_counter.hpp"
#include "io/allocator/counting_allocator.hpp"
#include "allocator/interprocess_allocator.hpp"

namespace map_matching_2::io::memory_mapped {

    struct memory_types {

        template<typename Allocator>
        using allocator_traits_type = std::allocator_traits<Allocator>;

        using allocation_counter_type = io::allocator::allocation_counter;

        template<typename Type>
        using allocator_type = std::allocator<Type>;

        // same as normal allocator, polymorphic allocator becomes slow with large data
        template<typename Type>
        using fast_allocator_type = std::allocator<Type>;

        template<typename Type>
        using counting_allocator_type = io::allocator::counting_allocator<Type, allocation_counter_type>;

        // same as normal allocator, polymorphic allocator becomes slow with large data
        template<typename Type>
        using fast_counting_allocator_type = io::allocator::counting_allocator<Type, allocation_counter_type>;

        template<typename Type,
            typename Allocator = allocator_type<Type>>
        using vector_type = std::vector<Type, Allocator>;

        template<typename Type,
            typename Allocator = allocator_type<Type>>
        using list_type = std::list<Type, Allocator>;

    };

    template<typename MMap>
    struct mmap_types {

        using mmap_type = MMap;
        using mmap_segment_manager_type = typename mmap_type::segment_manager;

        template<typename Allocator>
        using allocator_traits_type = std::allocator_traits<Allocator>;

        using allocation_counter_type = io::allocator::allocation_counter;

        template<typename Type>
        using allocator_type = allocator::interprocess_allocator<Type, mmap_segment_manager_type>;

        template<typename Type>
        using fast_allocator_type = allocator::interprocess_allocator<Type, mmap_segment_manager_type, false>;

        // counting does not correctly work on interprocess files, use regular allocator
        // is no problem, because file has a size that can be used
        template<typename Type>
        using counting_allocator_type = allocator::interprocess_allocator<Type, mmap_segment_manager_type>;

        // counting does not correctly work on interprocess files, use regular allocator
        // is no problem, because file has a size that can be used
        template<typename Type>
        using fast_counting_allocator_type = allocator::interprocess_allocator<Type, mmap_segment_manager_type, false>;

        template<typename Type,
            typename Allocator = allocator_type<Type>>
        using vector_type = boost::interprocess::vector<Type, Allocator>;

        template<typename Type,
            typename Allocator = allocator_type<Type>>
        using list_type = boost::interprocess::list<Type, Allocator>;

    };

    using managed_mapped_file = boost::interprocess::basic_managed_mapped_file<char,
        boost::interprocess::rbtree_best_fit<boost::interprocess::null_mutex_family>,
        boost::interprocess::flat_map_index>;

    using file_types = mmap_types<managed_mapped_file>;

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_TYPES_HPP
