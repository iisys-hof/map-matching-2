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

#ifndef MAP_MATCHING_2_IO_ALLOCATOR_ALLOCATION_COUNTER_HPP
#define MAP_MATCHING_2_IO_ALLOCATOR_ALLOCATION_COUNTER_HPP

#include <memory>

namespace map_matching_2::io::allocator {

    class allocation_counter {

    public:
        allocation_counter(const std::size_t n = 0)
            : _bytes{std::make_shared<std::size_t>(n)} {}

        allocation_counter(const allocation_counter &other)
            : _bytes{other._bytes} {}

        allocation_counter(allocation_counter &&other) noexcept
            : _bytes{std::move(other._bytes)} {}

        allocation_counter &operator=(const allocation_counter &other) {
            if (this != &other) {
                _bytes = other._bytes;
            }
            return *this;
        }

        allocation_counter &operator=(allocation_counter &&other) noexcept {
            if (this != &other) {
                _bytes = std::move(other._bytes);
            }
            return *this;
        }

        ~allocation_counter() = default;

        void add(const std::size_t n) {
            *_bytes += n;
        }

        void sub(const std::size_t n) {
            *_bytes -= n;
        }

        void reset(const std::size_t n = 0) {
            *_bytes = n;
        }

        [[nodiscard]] std::size_t bytes() const {
            return *_bytes;
        }

    protected:
        std::shared_ptr<std::size_t> _bytes;

    };

}

#endif //MAP_MATCHING_2_IO_ALLOCATOR_ALLOCATION_COUNTER_HPP
