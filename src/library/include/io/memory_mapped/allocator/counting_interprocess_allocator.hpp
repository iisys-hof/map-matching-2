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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_ALLOCATOR_COUNTING_INTERPROCESS_ALLOCATOR_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_ALLOCATOR_COUNTING_INTERPROCESS_ALLOCATOR_HPP

#include <boost/interprocess/allocators/allocator.hpp>

namespace map_matching_2::io::memory_mapped::allocator {

    template<typename T, typename SegmentManager, typename AllocationCounter, bool Deallocate = true>
    class counting_interprocess_allocator : public boost::interprocess::allocator<T, SegmentManager> {

    public:
        using base_type = boost::interprocess::allocator<T, SegmentManager>;
        using allocation_counter_type = AllocationCounter;

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
        explicit counting_interprocess_allocator(segment_manager *segment_mngr) noexcept
            : base_type{segment_mngr}, _allocation_counter{} {}

        counting_interprocess_allocator(segment_manager *segment_mngr,
                const allocation_counter_type &allocation_counter) noexcept
            : base_type{segment_mngr}, _allocation_counter{allocation_counter} {}

        counting_interprocess_allocator(const counting_interprocess_allocator &other) noexcept
            : base_type{other}, _allocation_counter{other._allocation_counter} {}

        template<typename U>
        counting_interprocess_allocator(
                const counting_interprocess_allocator<U, SegmentManager, AllocationCounter, Deallocate> &other) noexcept
            : base_type{other}, _allocation_counter{other.allocation_counter()} {}

        ~counting_interprocess_allocator() = default;

        template<class U>
        struct rebind {
            typedef counting_interprocess_allocator<U, SegmentManager, AllocationCounter, Deallocate> other;
        };

        [[nodiscard]] pointer allocate(size_type n, cvoid_ptr hint = 0) {
            _allocation_counter.add(n * sizeof(value_type));
            return base_type::allocate(n, hint);
        }

        void deallocate(const pointer &p, size_type n) {
            _allocation_counter.sub(n * sizeof(value_type));
            if constexpr (Deallocate) {
                base_type::deallocate(p, n);
            }
        }

        [[nodiscard]] pointer allocation_command(boost::interprocess::allocation_type command,
                size_type limit_size, size_type &prefer_in_recvd_out_size, pointer &reuse) {
            // WARNING
            // this method when called cannot correctly maintain the allocation counter
            // due to the original size of the reuse pointer not known at this state
            // so the difference that is put into prefer_in_recvd_out_size cannot be calculated
            // when this method is called, it makes the whole counter useless
            return base_type::allocation_command(command, limit_size, prefer_in_recvd_out_size, reuse);
        }

        void allocate_many(size_type elem_size, size_type num_elements, multiallocation_chain &chain) {
            _allocation_counter.add(elem_size * sizeof(value_type) * num_elements);
            base_type::allocate_many(elem_size, num_elements, chain);
        }

        void allocate_many(const size_type *elem_sizes, size_type n_elements, multiallocation_chain &chain) {
            for (std::size_t i = 0; i < n_elements; ++i) {
                _allocation_counter.add(elem_sizes[i] * sizeof(value_type));
            }
            base_type::allocate_many(elem_sizes, n_elements, chain);
        }

        void deallocate_many(multiallocation_chain &chain) {
            for (const auto &elem : chain) {
                _allocation_counter.sub(sizeof(value_type));
            }
            if constexpr (Deallocate) {
                base_type::deallocate_many(chain);
            }
        }

        [[nodiscard]] pointer allocate_one() {
            _allocation_counter.add(sizeof(value_type));
            return base_type::allocate_one();
        }

        void allocate_individual(size_type num_elements, multiallocation_chain &chain) {
            _allocation_counter.add(sizeof(value_type) * num_elements);
            base_type::allocate_individual(num_elements, chain);
        }

        void deallocate_one(const pointer &p) {
            _allocation_counter.sub(sizeof(value_type));
            if constexpr (Deallocate) {
                base_type::deallocate_one(p);
            }
        }

        void deallocate_individual(multiallocation_chain &chain) {
            for (const auto &elem : chain) {
                _allocation_counter.sub(sizeof(value_type));
            }
            if constexpr (Deallocate) {
                base_type::deallocate_individual(chain);
            }
        }

        [[nodiscard]] constexpr const allocation_counter_type &allocation_counter() const {
            return _allocation_counter;
        }

        [[nodiscard]] constexpr std::size_t bytes() const {
            return _allocation_counter.bytes();
        }

    protected:
        allocation_counter_type _allocation_counter;

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_ALLOCATOR_COUNTING_INTERPROCESS_ALLOCATOR_HPP
