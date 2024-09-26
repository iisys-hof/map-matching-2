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

#ifndef MAP_MATCHING_2_GRAPH_CONTAINER_SUBVIEW_HPP
#define MAP_MATCHING_2_GRAPH_CONTAINER_SUBVIEW_HPP

#include <stdexcept>
#include <format>

#include "util/concepts.hpp"

#include "container_view.hpp"

namespace map_matching_2::graph {

    template<typename Container>
    class container_subview : public container_view<Container> {

    public:
        using base_type = container_view<Container>;
        using container_type = typename base_type::container_type;
        using value_type = typename base_type::value_type;
        using size_type = typename base_type::size_type;
        using difference_type = typename base_type::difference_type;
        using iterator = typename base_type::iterator;
        using const_iterator = typename base_type::const_iterator;
        using descriptor = typename base_type::descriptor;

        constexpr container_subview(container_type &ref, const size_type start, const size_type stop)
            : base_type{ref}, _start{start}, _stop{stop} {
            if (_start > base_type::size() or _stop > base_type::size() or _start > _stop) {
                throw std::invalid_argument{
                        std::format("start {} or stop {} are out of range {}", _start, _stop, base_type::size())
                };
            }
        }

        constexpr container_subview(const container_subview &other) = default;

        constexpr container_subview(container_subview &&other) noexcept = default;

        constexpr container_subview &operator=(const container_subview &other) = default;

        constexpr container_subview &operator=(container_subview &&other) noexcept = default;

        constexpr ~container_subview() = default;

        [[nodiscard]] constexpr const value_type &at(difference_type pos) const {
            if (pos > 0) {
                return base_type::operator[](next(begin(), pos));
            } else {
                return base_type::operator[](begin());
            }
        }

        [[nodiscard]] constexpr value_type &at(difference_type pos) {
            return const_cast<value_type &>(const_cast<const container_subview *>(this)->at(pos));
        }

        [[nodiscard]] constexpr const value_type &front() const {
            if (empty()) {
                throw std::out_of_range{std::format("out of range access to front with {}", _start)};
            }
            return base_type::operator[](begin());
        }

        [[nodiscard]] constexpr value_type &front() {
            return const_cast<value_type &>(const_cast<const container_subview *>(this)->front());
        }

        [[nodiscard]] constexpr const value_type &back() const {
            if (empty() or _stop == 0) {
                throw std::out_of_range{std::format("out of range access to back with {}", _stop)};
            }
            return base_type::operator[](prev(end()));
        }

        [[nodiscard]] constexpr value_type &back() {
            return const_cast<value_type &>(const_cast<const container_subview *>(this)->back());
        }

        [[nodiscard]] constexpr descriptor begin() const requires std::is_same_v<size_type, descriptor> {
            return _start;
        }

        [[nodiscard]] constexpr descriptor begin() const requires std::is_same_v<const_iterator, descriptor> {
            return std::next(std::cbegin(this->_ref), _start);
        }

        [[nodiscard]] constexpr descriptor begin() const requires std::is_same_v<iterator, descriptor> {
            return std::next(std::begin(this->_ref), _start);
        }

        [[nodiscard]] constexpr descriptor end() const requires std::is_same_v<size_type, descriptor> {
            return _stop;
        }

        [[nodiscard]] constexpr descriptor end() const requires std::is_same_v<const_iterator, descriptor> {
            return next(_stop >= this->_ref.size()) ? std::cend(this->_ref) : std::next(std::cbegin(this->_ref), _stop);
        }

        [[nodiscard]] constexpr descriptor end() const requires std::is_same_v<iterator, descriptor> {
            return next(_stop >= this->_ref.size()) ? std::end(this->_ref) : std::next(std::begin(this->_ref), _stop);
        }

        [[nodiscard]] constexpr size_type size() const {
            return _stop - _start;
        }

        [[nodiscard]] constexpr size_type capacity() const requires util::has_capacity<container_type> {
            return size();
        }

        [[nodiscard]] constexpr bool empty() const {
            return base_type::empty() or _stop == _start;
        }

    protected:
        const size_type _start;
        const size_type _stop;

    };

}

#endif //MAP_MATCHING_2_GRAPH_CONTAINER_SUBVIEW_HPP
