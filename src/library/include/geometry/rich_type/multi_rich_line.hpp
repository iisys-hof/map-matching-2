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

#ifndef MAP_MATCHING_2_RICH_TYPE_MULTI_RICH_LINE_HPP
#define MAP_MATCHING_2_RICH_TYPE_MULTI_RICH_LINE_HPP

#include <format>
#include <vector>

#include <boost/serialization/serialization.hpp>

#include "geometry/traits.hpp"

#include "geometry/algorithm/convert.hpp"

#include "exception.hpp"

#include "traits/rich_line.hpp"

namespace map_matching_2::geometry {

    template<typename RichLine,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator> requires
        (util::is_random_access_v<Container<RichLine, Allocator<RichLine>>>)
    class multi_rich_line {

    public:
        using rich_line_type = RichLine;

        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using point_type = point_type_t<line_type>;
        using length_type = typename data<line_type>::length_type;
        using angle_type = typename data<line_type>::angle_type;

        using allocator_type = Allocator<void>;

        using rich_lines_container_type = Container<rich_line_type, Allocator<rich_line_type>>;

        using multi_line_type = typename models<point_type>::template multi_line_type<line_type>;

        using memory_line_type = typename rich_line_type::memory_line_type;
        using memory_multi_line_type = typename models<point_type>::template multi_line_type<memory_line_type>;

        constexpr multi_rich_line() requires
            (std::default_initializable<rich_line_type> and std::default_initializable<allocator_type>)
            : _rich_lines{} {}

        explicit multi_rich_line(const multi_line_type &multi_line) requires
            (std::default_initializable<allocator_type>)
            : multi_rich_line{convert_container_types<rich_lines_container_type>(multi_line)} {}

        multi_rich_line(const multi_line_type &multi_line, const allocator_type &allocator)
            : multi_rich_line{
                    convert_container_types<rich_lines_container_type>(multi_line, allocator),
                    allocator
            } {}

        // explicit multi_rich_line(const std::vector<line_type> &lines)
        //         : multi_rich_line{lines2multi_line<multi_rich_line>(lines)} {}

        constexpr explicit multi_rich_line(rich_line_type rich_line) requires
            (std::default_initializable<allocator_type>)
            : multi_rich_line{rich_lines_container_type{std::move(rich_line)}} {}

        constexpr multi_rich_line(rich_line_type rich_line, const allocator_type &allocator)
            : multi_rich_line{
                    rich_lines_container_type{
                            std::initializer_list<rich_line_type>{std::move(rich_line)}, allocator
                    },
                    allocator
            } {}

        constexpr multi_rich_line(rich_lines_container_type rich_lines) requires
            (std::default_initializable<allocator_type>)
            : _rich_lines{std::move(rich_lines)} {}

        constexpr multi_rich_line(rich_lines_container_type rich_lines, const allocator_type &allocator)
            : _rich_lines{std::move(rich_lines), allocator} {}

        // template<typename RichLineT>
        // explicit multi_rich_line(RichLineT rich_line)
        //         : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_line))} {}
        //
        // template<typename RichLineT>
        // explicit multi_rich_line(std::vector<RichLineT> rich_lines)
        //         : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_lines))} {}

        constexpr multi_rich_line(const multi_rich_line &other)
            : _rich_lines{other._rich_lines} {}

        template<typename RichLineT>
        multi_rich_line(const multi_rich_line<RichLineT> &other) requires
            (not std::same_as<RichLineT, rich_line_type>)
            : _rich_lines{convert_container_types<rich_lines_container_type>(other.rich_lines())} {}

        constexpr multi_rich_line(multi_rich_line &&other) noexcept
            : _rich_lines{std::move(other._rich_lines)} {}

        constexpr multi_rich_line &operator=(const multi_rich_line &other) {
            if (this != &other) {
                _rich_lines = other._rich_lines;
            }
            return *this;
        }

        template<typename RichLineT>
        multi_rich_line &operator=(const multi_rich_line<RichLineT> &other) requires
            (not std::same_as<RichLineT, rich_line_type>) {
            _rich_lines = convert_container_types<rich_lines_container_type>(other.rich_lines());
            return *this;
        }

        constexpr multi_rich_line &operator=(multi_rich_line &&other) noexcept {
            if (this != &other) {
                _rich_lines = std::move(other._rich_lines);
            }
            return *this;
        }

        constexpr ~multi_rich_line() = default;

        [[nodiscard]] bool has_length() const {
            for (const auto &rich_line : rich_lines()) {
                if (rich_line.has_length()) {
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] length_type length() const {
            bool has_length = false;
            length_type length = default_float_type<length_type>::v0;

            for (const auto &rich_line : rich_lines()) {
                if (rich_line.has_length()) {
                    length += rich_line.length();
                    has_length = true;
                }
            }

            if (has_length) {
                return length;
            }

            throw length_computation_exception{std::format("cannot compute length for multi_line: {}", wkt())};
        }

        [[nodiscard]] bool has_azimuth() const {
            if (not empty()) {
                if (size() == 1) {
                    const auto &rich_line = front();
                    return rich_line.has_azimuth();
                } else {
                    const auto &start = front().front();
                    const auto &end = back().back();
                    return not geometry::equals_points(start, end);
                }
            }

            return false;
        }

        [[nodiscard]] angle_type azimuth() const {
            if (not empty()) {
                if (size() == 1) {
                    const auto &rich_line = front();
                    if (rich_line.has_azimuth()) {
                        return rich_line.azimuth();
                    }
                } else {
                    const auto &start = front().front();
                    const auto &end = back().back();
                    if (not geometry::equals_points(start, end)) {
                        return geometry::azimuth_deg(start, end);
                    }
                }
            }

            throw azimuth_computation_exception{std::format("cannot compute azimuth for multi_line: {}", wkt())};
        }

        [[nodiscard]] bool has_directions() const {
            if (not empty()) {
                if (size() == 1) {
                    const auto &rich_line = front();
                    return rich_line.has_directions();
                } else if (size() >= 2) {
                    for (std::size_t i = 0; i < size(); ++i) {
                        const auto &rich_line = operator[](i);
                        if (rich_line.has_directions()) {
                            return true;
                        }
                        if (i + 1 < size()) {
                            const auto &next = operator[](i + 1);
                            if (geometry::equals_points(rich_line.back(), next.front()) and
                                rich_line.size() >= 2 and rich_line.has_azimuth(rich_line.size() - 2) and
                                next.size() >= 2 and next.has_azimuth(0)) {
                                return true;
                            }
                        }
                    }
                }
            }

            return false;
        }

        [[nodiscard]] angle_type directions() const {
            if (not empty()) {
                if (size() == 1) {
                    const auto &rich_line = front();
                    if (rich_line.has_directions()) {
                        return rich_line.directions();
                    }
                } else if (size() >= 2) {
                    bool has_directions = false;
                    angle_type directions = default_float_type<angle_type>::v0;

                    for (std::size_t i = 0; i < size(); ++i) {
                        const auto &rich_line = operator[](i);
                        if (rich_line.has_directions()) {
                            directions += rich_line.directions();
                            has_directions = true;
                        }
                        if (i + 1 < size()) {
                            const auto &next = operator[](i + 1);
                            if (geometry::equals_points(rich_line.back(), next.front()) and
                                rich_line.size() >= 2 and rich_line.has_azimuth(rich_line.size() - 2) and
                                next.size() >= 2 and next.has_azimuth(0)) {
                                angle_type direction = geometry::direction_deg(
                                        rich_line.azimuth(rich_line.size() - 2), next.azimuth(0));
                                directions += direction;
                                has_directions = true;
                            }
                        }
                    }

                    if (has_directions) {
                        return directions;
                    }
                }
            }

            throw directions_computation_exception{std::format("cannot compute directions for multi_line: {}", wkt())};
        }

        [[nodiscard]] angle_type absolute_directions() const {
            if (not empty()) {
                if (size() == 1) {
                    const auto &rich_line = front();
                    if (rich_line.has_directions()) {
                        return rich_line.absolute_directions();
                    }
                } else if (size() >= 2) {
                    bool has_directions = false;
                    angle_type directions = default_float_type<angle_type>::v0;

                    for (std::size_t i = 0; i < size(); ++i) {
                        const auto &rich_line = operator[](i);
                        if (rich_line.has_directions()) {
                            directions += rich_line.absolute_directions();
                            has_directions = true;
                        }
                        if (i + 1 < size()) {
                            const auto &next = operator[](i + 1);
                            if (geometry::equals_points(rich_line.back(), next.front()) and
                                rich_line.size() >= 2 and rich_line.has_azimuth(rich_line.size() - 2) and
                                next.size() >= 2 and next.has_azimuth(0)) {
                                angle_type direction = geometry::direction_deg(
                                        rich_line.azimuth(rich_line.size() - 2), next.azimuth(0));
                                directions += std::fabs(direction);
                                has_directions = true;
                            }
                        }
                    }

                    if (has_directions) {
                        return directions;
                    }
                }
            }

            throw directions_computation_exception{
                    std::format("cannot compute absolute directions for multi_line: {}", wkt())
            };
        }

        [[nodiscard]] constexpr const rich_lines_container_type &rich_lines() const {
            return _rich_lines;
        }

        [[nodiscard]] constexpr rich_lines_container_type &rich_lines() {
            return _rich_lines;
        }

        [[nodiscard]] constexpr const rich_line_type &operator[](std::size_t pos) const {
            return _rich_lines[pos];
        }

        [[nodiscard]] constexpr rich_line_type &operator[](std::size_t pos) {
            return _rich_lines[pos];
        }

        [[nodiscard]] constexpr const rich_line_type &at(std::size_t pos) const {
            return _rich_lines.at(pos);
        }

        [[nodiscard]] constexpr rich_line_type &at(std::size_t pos) {
            return _rich_lines.at(pos);
        }

        [[nodiscard]] constexpr const rich_line_type &front() const {
            return _rich_lines.front();
        }

        [[nodiscard]] constexpr rich_line_type &front() {
            return _rich_lines.front();
        }

        [[nodiscard]] constexpr const rich_line_type &back() const {
            return _rich_lines.back();
        }

        [[nodiscard]] constexpr rich_line_type &back() {
            return _rich_lines.back();
        }

        [[nodiscard]] memory_multi_line_type multi_line() const {
            memory_multi_line_type multi_line;
            multi_line.reserve(_rich_lines.size());
            for (const auto &rich_line : _rich_lines) {
                if (not rich_line.empty()) {
                    multi_line.emplace_back(memory_line_type{rich_line.line()});
                }
            }
            return multi_line;
        }

        [[nodiscard]] bool empty() const {
            if (_rich_lines.empty()) {
                return true;
            }

            for (const rich_line_type &rich_line : _rich_lines) {
                if (not rich_line.empty()) {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] constexpr std::size_t size() const {
            return _rich_lines.size();
        }

        [[nodiscard]] std::size_t sizes() const {
            std::size_t size = 0;
            for (const rich_line_type &rich_line : _rich_lines) {
                size += rich_line.size();
            }
            return size;
        }

        // void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
        //     for (auto &rich_line: _rich_lines) {
        //         rich_line.simplify(retain_reversals, tolerance, reverse_tolerance);
        //     }
        // }
        //
        // template<typename Network>
        // void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr) requires
        // std::default_initializable<rich_line_type> {
        //     for (auto &rich_line: _rich_lines) {
        //         rich_line.median_merge(tolerance, adaptive, network);
        //     }
        // }

        template<is_rich_line RichLineT>
        multi_rich_line &merge(std::vector<RichLineT> &&rich_lines) requires
            (std::is_base_of_v<rich_line_type, RichLine>) {
            using _rich_line_type = RichLineT;

            auto it = rich_lines.begin();
            auto start_it = it;
            while (it != rich_lines.end()) {
                if (it != rich_lines.begin()) {
                    const _rich_line_type &prev_rich_line = *std::prev(it);
                    const _rich_line_type &next_rich_line = *it;
                    if (not prev_rich_line.empty() and not next_rich_line.empty() and
                        not geometry::equals_points(prev_rich_line.back(), next_rich_line.front())) {
                        _rich_lines.emplace_back(rich_line_type{}.merge(std::vector<RichLineT>{
                                std::move_iterator(start_it), std::move_iterator(it)
                        }));
                        start_it = it;
                    }
                }
                ++it;
            }

            _rich_lines.emplace_back(rich_line_type{}.merge(std::vector<RichLineT>{
                    std::move_iterator(start_it), std::move_iterator(it)
            }));

            return *this;
        }

        [[nodiscard]] std::string wkt() const {
            if (empty()) {
                return geometry::to_wkt(typename models<point_type>::template line_type<>{});
            } else if (_rich_lines.size() <= 1) {
                return geometry::to_wkt(_rich_lines.front().line());
            } else {
                return geometry::to_wkt(multi_line());
            }
        }

        [[nodiscard]] std::string str() const {
            std::string str = "MultiRichLine(";
            str.append(wkt());
            if (has_length()) {
                str.append(",");
                str.append(std::to_string(length()));
            }
            if (has_azimuth()) {
                str.append(",");
                str.append(std::to_string(azimuth()));
            }
            if (has_directions()) {
                str.append(",");
                str.append(std::to_string(directions()));
                str.append(",");
                str.append(std::to_string(absolute_directions()));
            }
            str.append(")");
            return str;
        }

        friend bool operator==(const multi_rich_line &left, const multi_rich_line &right) {
            // if size of lines is not equal, multi_lines cannot be equal
            if (left._rich_lines.size() != right._rich_lines.size()) {
                return false;
            }

            // if any lines is not equal, multi_lines cannot be equal
            for (std::size_t i = 0; i < left._rich_lines.size(); ++i) {
                if (left._rich_lines[i] != right._rich_lines[i]) {
                    return false;
                }
            }

            // if all lines are equal, multi_lines are equal
            return true;
        }

        friend bool operator!=(const multi_rich_line &left, const multi_rich_line &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const multi_rich_line &multi_rich_line) {
            return out << multi_rich_line.str();
        }

    protected:
        rich_lines_container_type _rich_lines;

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_MULTI_RICH_LINE_HPP
