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

#ifndef MAP_MATCHING_2_GRAPH_CONTAINER_VIEW_HPP
#define MAP_MATCHING_2_GRAPH_CONTAINER_VIEW_HPP

#include <format>

#include "util/concepts.hpp"

namespace map_matching_2::graph {

    template<typename Container>
    class container_view {

    public:
        using container_type = Container;
        using value_type = typename container_type::value_type;
        using size_type = typename container_type::size_type;
        using difference_type = typename container_type::difference_type;
        using iterator = typename container_type::iterator;
        using const_iterator = typename container_type::const_iterator;
        using descriptor = std::conditional_t<util::is_random_access_v<container_type>,
            size_type, std::conditional_t<std::is_const_v<container_type>, const_iterator, iterator>>;

        constexpr explicit container_view(container_type &ref)
            : _ref{ref} {}

        constexpr container_view(const container_view &other) = default;

        constexpr container_view(container_view &&other) noexcept = default;

        constexpr container_view &operator=(const container_view &other) = default;

        constexpr container_view &operator=(container_view &&other) noexcept = default;

        constexpr ~container_view() = default;

        [[nodiscard]] constexpr const value_type &operator[](std::same_as<size_type> auto desc) const {
            return _ref[desc];
        }

        [[nodiscard]] constexpr value_type &operator[](std::same_as<size_type> auto desc) {
            return _ref[desc];
        }

        [[nodiscard]] constexpr const value_type &operator[](std::same_as<const_iterator> auto desc) const {
            return *desc;
        }

        [[nodiscard]] constexpr const value_type &operator[](std::same_as<iterator> auto desc) const {
            return *desc;
        }

        [[nodiscard]] constexpr value_type &operator[](std::same_as<iterator> auto desc) {
            return *desc;
        }

        [[nodiscard]] constexpr const value_type &at(difference_type pos) const {
            if (pos > 0) {
                if (pos >= size()) {
                    throw std::invalid_argument{std::format("pos {} out of range, size: {}", pos, size())};
                }

                return operator[](next(begin(), pos));
            } else if (pos < 0) {
                if (std::abs(pos) >= size()) {
                    throw std::invalid_argument{std::format("reverse pos {} out of range, size: {}", pos, size())};
                }

                return operator[](prev(end(), std::abs(pos)));
            } else {
                return operator[](begin());
            }
        }

        [[nodiscard]] constexpr value_type &at(difference_type pos) {
            return const_cast<value_type &>(const_cast<const container_view *>(this)->at(pos));
        }

        [[nodiscard]] constexpr const value_type &front() const {
            return _ref.front();
        }

        [[nodiscard]] constexpr value_type &front() {
            return _ref.front();
        }

        [[nodiscard]] constexpr const value_type &back() const {
            return _ref.back();
        }

        [[nodiscard]] constexpr value_type &back() {
            return _ref.back();
        }

        [[nodiscard]] constexpr descriptor begin() const requires std::is_same_v<size_type, descriptor> {
            return 0;
        }

        [[nodiscard]] constexpr descriptor begin() const requires std::is_same_v<const_iterator, descriptor> {
            return std::cbegin(_ref);
        }

        [[nodiscard]] constexpr descriptor begin() const requires std::is_same_v<iterator, descriptor> {
            return std::begin(_ref);
        }

        [[nodiscard]] constexpr descriptor end() const requires std::is_same_v<size_type, descriptor> {
            return size();
        }

        [[nodiscard]] constexpr descriptor end() const requires std::is_same_v<const_iterator, descriptor> {
            return std::cend(_ref);
        }

        [[nodiscard]] constexpr descriptor end() const requires std::is_same_v<iterator, descriptor> {
            return std::end(_ref);
        }

        [[nodiscard]] constexpr descriptor next(descriptor desc, difference_type count = 1) const requires
            std::is_same_v<size_type, descriptor> {
            return desc + count;
        }

        [[nodiscard]] constexpr descriptor next(descriptor desc, difference_type count = 1) const requires
            std::is_same_v<const_iterator, descriptor> {
            return std::next(desc, count);
        }

        [[nodiscard]] constexpr descriptor next(descriptor desc, difference_type count = 1) const requires
            std::is_same_v<iterator, descriptor> {
            return std::next(desc, count);
        }

        [[nodiscard]] constexpr descriptor prev(descriptor desc, difference_type count = 1) const requires
            std::is_same_v<size_type, descriptor> {
            return desc - count;
        }

        [[nodiscard]] constexpr descriptor prev(descriptor desc, difference_type count = 1) const requires
            std::is_same_v<const_iterator, descriptor> {
            return std::prev(desc, count);
        }

        [[nodiscard]] constexpr descriptor prev(descriptor desc, difference_type count = 1) const requires
            std::is_same_v<iterator, descriptor> {
            return std::prev(desc, count);
        }

        [[nodiscard]] constexpr size_type size() const {
            return _ref.size();
        }

        [[nodiscard]] constexpr size_type capacity() const requires util::has_capacity<container_type> {
            return _ref.capacity();
        }

        [[nodiscard]] constexpr bool empty() const {
            return _ref.empty();
        }

    protected:
        container_type &_ref;

    };

}

#endif //MAP_MATCHING_2_GRAPH_CONTAINER_VIEW_HPP
