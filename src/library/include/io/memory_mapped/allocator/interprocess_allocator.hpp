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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_ALLOCATOR_INTERPROCESS_ALLOCATOR_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_ALLOCATOR_INTERPROCESS_ALLOCATOR_HPP

#include <boost/interprocess/allocators/allocator.hpp>

namespace map_matching_2::io::memory_mapped::allocator {

    template<typename T, typename SegmentManager, bool Deallocate = true>
    class interprocess_allocator : public boost::interprocess::allocator<T, SegmentManager> {

    public:
        using base_type = boost::interprocess::allocator<T, SegmentManager>;

        using segment_manager = typename base_type::segment_manager;
        using void_pointer = typename base_type::void_pointer;

        using value_type = typename base_type::value_type;
        using pointer = typename base_type::pointer;
        using const_pointer = typename base_type::const_pointer;
        using reference = typename base_type::reference;
        using const_reference = typename base_type::const_reference;
        using size_type = typename base_type::size_type;
        using difference_type = typename base_type::difference_type;
        using version = typename base_type::version;

        using multiallocation_chain = typename base_type::multiallocation_chain;

    protected:
        using aux_pointer_t = typename segment_manager::void_pointer;
        using cvoid_ptr = typename boost::intrusive::pointer_traits<
            aux_pointer_t>::template rebind_pointer<const void>::type;

    public:
        explicit interprocess_allocator(segment_manager *segment_mngr) noexcept
            : base_type{segment_mngr} {}

        interprocess_allocator(const interprocess_allocator &other) noexcept
            : base_type{other} {}

        template<typename U>
        interprocess_allocator(
                const interprocess_allocator<U, SegmentManager, Deallocate> &other) noexcept
            : base_type{other} {}

        ~interprocess_allocator() = default;

        template<class U>
        struct rebind {
            typedef interprocess_allocator<U, SegmentManager, Deallocate> other;
        };

        [[nodiscard]] pointer allocate(size_type n, cvoid_ptr hint = 0) {
            return base_type::allocate(n, hint);
        }

        void deallocate(const pointer &p, size_type n) {
            if constexpr (Deallocate) {
                base_type::deallocate(p, n);
            }
        }

        [[nodiscard]] pointer allocation_command(boost::interprocess::allocation_type command,
                size_type limit_size, size_type &prefer_in_recvd_out_size, pointer &reuse) {
            return base_type::allocation_command(command, limit_size, prefer_in_recvd_out_size, reuse);
        }

        void allocate_many(size_type elem_size, size_type num_elements, multiallocation_chain &chain) {
            base_type::allocate_many(elem_size, num_elements, chain);
        }

        void allocate_many(const size_type *elem_sizes, size_type n_elements, multiallocation_chain &chain) {
            base_type::allocate_many(elem_sizes, n_elements, chain);
        }

        void deallocate_many(multiallocation_chain &chain) {
            if constexpr (Deallocate) {
                base_type::deallocate_many(chain);
            }
        }

        [[nodiscard]] pointer allocate_one() {
            return base_type::allocate_one();
        }

        void allocate_individual(size_type num_elements, multiallocation_chain &chain) {
            base_type::allocate_individual(num_elements, chain);
        }

        void deallocate_one(const pointer &p) {
            if constexpr (Deallocate) {
                base_type::deallocate_one(p);
            }
        }

        void deallocate_individual(multiallocation_chain &chain) {
            if constexpr (Deallocate) {
                base_type::deallocate_individual(chain);
            }
        }

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_ALLOCATOR_INTERPROCESS_ALLOCATOR_HPP
