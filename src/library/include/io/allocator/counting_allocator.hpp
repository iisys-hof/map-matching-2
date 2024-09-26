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

#ifndef MAP_MATCHING_2_IO_ALLOCATOR_COUNTING_ALLOCATOR_HPP
#define MAP_MATCHING_2_IO_ALLOCATOR_COUNTING_ALLOCATOR_HPP

#include <memory>

namespace map_matching_2::io::allocator {

    template<typename T, typename AllocationCounter>
    class counting_allocator : public std::allocator<T> {

    public:
        using base_type = std::allocator<T>;
        using allocation_counter_type = AllocationCounter;

        using value_type = typename base_type::value_type;
        using size_type = typename base_type::size_type;
        using difference_type = typename base_type::difference_type;
        using is_always_equal = typename base_type::is_always_equal;
        using propagate_on_container_move_assignment = typename base_type::propagate_on_container_move_assignment;

        constexpr counting_allocator() noexcept = default;

        constexpr explicit counting_allocator(const allocation_counter_type &allocation_counter) noexcept
            : base_type{}, _allocation_counter{allocation_counter} {}

        constexpr counting_allocator(const counting_allocator &other) noexcept
            : base_type{other}, _allocation_counter{other._allocation_counter} {}

        template<typename U>
        constexpr counting_allocator(const counting_allocator<U, AllocationCounter> &other) noexcept
            : base_type{other}, _allocation_counter{other.allocation_counter()} {}

        constexpr ~counting_allocator() = default;

        [[nodiscard]] constexpr value_type *allocate(size_type n) {
            _allocation_counter.add(n * sizeof(value_type));
            return base_type::allocate(n);
        }

        constexpr void deallocate(value_type *p, size_type n) {
            _allocation_counter.sub(n * sizeof(value_type));
            base_type::deallocate(p, n);
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

#endif //MAP_MATCHING_2_IO_ALLOCATOR_COUNTING_ALLOCATOR_HPP
