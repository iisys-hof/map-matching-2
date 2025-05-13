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

#ifndef MAP_MATCHING_2_RICH_TYPE_RICH_LINE_HPP
#define MAP_MATCHING_2_RICH_TYPE_RICH_LINE_HPP

#include <format>
#include <vector>

#include <boost/serialization/serialization.hpp>

#include "types/geometry/rich_type/rich_segment.hpp"

#include "geometry/models.hpp"
#include "geometry/traits.hpp"

#include "geometry/algorithm/direction.hpp"
#include "geometry/algorithm/buffer.hpp"
#include "geometry/algorithm/median.hpp"

#include "common.hpp"
#include "concepts.hpp"
#include "exception.hpp"

namespace map_matching_2::geometry {

    template<typename Point,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator>
    class rich_line {

    public:
        using point_type = Point;
        using line_type = linestring<Point, Container, Allocator>;

        using segment_type = typename models<point_type>::segment_type;
        using referring_segment_type = typename models<point_type>::referring_segment_type;
        using coordinate_system_type = typename data<point_type>::coordinate_system_type;
        using rich_segment_type = geometry::rich_segment_type<point_type>;
        using rich_referring_segment_type = geometry::rich_referring_segment_type<point_type>;

        using distance_type = typename data<line_type>::distance_type;
        using length_type = typename data<line_type>::length_type;
        using angle_type = typename data<line_type>::angle_type;

        using allocator_type = Allocator<void>;

        using memory_rich_line_type = rich_line<point_type>;
        using memory_line_type = linestring<point_type>;

        constexpr rich_line() requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>)
            : _line{} {}

        constexpr explicit rich_line(line_type line) requires
            (std::default_initializable<allocator_type>)
            : _line{std::move(line)} {}

        constexpr rich_line(line_type line, const allocator_type &allocator)
            : _line{std::move(line), allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr explicit rich_line(const linestring<point_type, ContainerT, AllocatorT> &line) requires
            (std::default_initializable<allocator_type>)
            : _line{std::cbegin(line), std::cend(line)} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr rich_line(const linestring<point_type, ContainerT, AllocatorT> &line,
                const allocator_type &allocator)
            : _line{std::cbegin(line), std::cend(line), allocator} {}

        constexpr rich_line(const rich_line &other)
            : _line{other._line} {}

        constexpr rich_line(const rich_line &other, const allocator_type &allocator)
            : _line{other._line, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr rich_line(const rich_line<point_type, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : _line{std::cbegin(other.line()), std::cend(other.line())} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr rich_line(const rich_line<point_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : _line{std::cbegin(other.line()), std::cend(other.line()), allocator} {}

        constexpr rich_line(rich_line &&other) noexcept
            : _line{std::move(other._line)} {}

        constexpr rich_line(rich_line &&other, const allocator_type &allocator) noexcept
            : _line{std::move(other._line), allocator} {}

        constexpr rich_line &operator=(const rich_line &other) {
            if (this != &other) {
                _line = other._line;
            }
            return *this;
        }

        constexpr rich_line &operator=(line_type &&line) requires
            (std::default_initializable<allocator_type>) {
            _line = std::move(line);
            return *this;
        }

        constexpr rich_line &operator=(rich_line &&other) noexcept {
            if (this != &other) {
                _line = std::move(other._line);
            }
            return *this;
        }

        constexpr ~rich_line() = default;

        [[nodiscard]] bool has_length() const {
            if (size() >= 2) {
                for (std::size_t i = 0; i < size() - 1; ++i) {
                    const referring_segment_type segment = referring_segment(i);
                    if (not geometry::equals_points_segment(segment)) {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] length_type length() const {
            if (size() >= 2) {
                bool has_length = false;
                length_type length = default_float_type<length_type>::v0;

                for (std::size_t i = 0; i < size() - 1; ++i) {
                    const referring_segment_type segment = referring_segment(i);
                    if (not geometry::equals_points_segment(segment)) {
                        length += geometry::length_segment(segment);
                        has_length = true;
                    }
                }

                if (has_length) {
                    return length;
                }
            }

            throw length_computation_exception{std::format("cannot compute length for line: {}", wkt())};
        }

        [[nodiscard]] bool has_azimuth() const {
            if (size() >= 2) {
                return not geometry::equals_points(front(), back());
            }

            return false;
        }

        [[nodiscard]] angle_type azimuth() const {
            if (size() >= 2) {
                const point_type &start = front();
                const point_type &end = back();
                if (not geometry::equals_points(start, end)) {
                    return geometry::azimuth_deg(start, end);
                }
            }

            throw azimuth_computation_exception{std::format("cannot compute azimuth for line: {}", wkt())};
        }

        [[nodiscard]] bool has_directions() const {
            if (size() >= 3) {
                for (std::size_t i = 0; i < size() - 2; ++i) {
                    if (has_direction(i)) {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] angle_type directions() const {
            if (size() >= 3) {
                bool has_directions = false;
                angle_type directions = default_float_type<angle_type>::v0;

                for (std::size_t i = 0; i < size() - 2; ++i) {
                    angle_type segment_direction = direction(i);
                    if (valid(segment_direction)) {
                        directions += segment_direction;
                        has_directions = true;
                    }
                }

                if (has_directions) {
                    return directions;
                }
            }

            throw directions_computation_exception{std::format("cannot compute directions for line: {}", wkt())};
        }

        [[nodiscard]] angle_type absolute_directions() const {
            if (size() >= 3) {
                bool has_directions = false;
                angle_type absolute_directions = default_float_type<angle_type>::v0;

                for (std::size_t i = 0; i < size() - 2; ++i) {
                    angle_type segment_absolute_direction = absolute_direction(i);
                    if (valid(segment_absolute_direction)) {
                        absolute_directions += segment_absolute_direction;
                        has_directions = true;
                    }
                }

                if (has_directions) {
                    return absolute_directions;
                }
            }

            throw directions_computation_exception{
                    std::format("cannot compute absolute directions for line: {}", wkt())
            };
        }

        [[nodiscard]] bool has_length(std::size_t pos) const {
            return not geometry::equals_points_segment(referring_segment(pos));
        }

        [[nodiscard]] length_type length(std::size_t pos) const {
            const referring_segment_type segment = referring_segment(pos);
            if (not geometry::equals_points_segment(segment)) {
                return geometry::length_segment(segment);
            }

            throw length_computation_exception{
                    std::format("cannot compute length for segment: {}", geometry::to_wkt(segment))
            };
        }

        [[nodiscard]] bool has_azimuth(std::size_t pos) const {
            return not geometry::equals_points_segment(referring_segment(pos));
        }

        [[nodiscard]] angle_type azimuth(std::size_t pos) const {
            const referring_segment_type segment = referring_segment(pos);
            if (not geometry::equals_points_segment(segment)) {
                return geometry::azimuth_segment_deg(segment);
            }

            throw azimuth_computation_exception{
                    std::format("cannot compute azimuth for segment: {}", geometry::to_wkt(segment))
            };
        }

        [[nodiscard]] bool has_direction(std::size_t pos) const {
            if (pos + 2 < size()) {
                const referring_segment_type segment_a = referring_segment(pos);
                const referring_segment_type segment_b = referring_segment(pos + 1);

                return not geometry::equals_points_segment(segment_a) and
                        not geometry::equals_points_segment(segment_b);
            }

            throw std::out_of_range(std::format("rich_line::has_direction(pos: {}): out of range, size: {}",
                    pos, size()));
        }

        [[nodiscard]] angle_type direction(std::size_t pos) const {
            if (pos + 2 < size()) {
                const referring_segment_type segment_a = referring_segment(pos);
                const referring_segment_type segment_b = referring_segment(pos + 1);

                if (not geometry::equals_points_segment(segment_a) and
                    not geometry::equals_points_segment(segment_b)) {
                    return geometry::direction_deg(geometry::azimuth_segment_deg(segment_a),
                            geometry::azimuth_segment_deg(segment_b));
                }

                return not_valid<angle_type>;
            }

            throw std::out_of_range(std::format("rich_line::direction(pos: {}): out of range, size: {}",
                    pos, size()));
        }

        [[nodiscard]] angle_type absolute_direction(std::size_t pos) const {
            return std::fabs(direction(pos));
        }

        [[nodiscard]] constexpr const line_type &line() const {
            return _line;
        }

        [[nodiscard]] constexpr line_type &line() {
            return _line;
        }

        [[nodiscard]] constexpr segment_type segment(std::size_t pos) const {
            if (pos + 1 < size()) {
                return segment_type{operator[](pos), operator[](pos + 1)};
            }

            throw std::out_of_range(std::format("rich_line::segment(pos: {}): out of range, size: {}",
                    pos, size()));
        }

        [[nodiscard]] constexpr segment_type segment(typename line_type::const_iterator it) const {
            return segment(std::distance(std::cbegin(_line), it));
        }

        [[nodiscard]] constexpr rich_segment_type rich_segment(std::size_t pos) const {
            return rich_segment_type{segment(pos)};
        }

        [[nodiscard]] constexpr rich_segment_type rich_segment(typename line_type::const_iterator it) const {
            return rich_segment_type{segment(it)};
        }

        [[nodiscard]] constexpr referring_segment_type referring_segment(std::size_t pos) const {
            if (pos + 1 < size()) {
                return referring_segment_type{operator[](pos), operator[](pos + 1)};
            }

            throw std::out_of_range(std::format("rich_line::referring_segment(pos: {}): out of range, size: {}",
                    pos, size()));
        }

        [[nodiscard]] constexpr referring_segment_type referring_segment(typename line_type::const_iterator it) const {
            return referring_segment(std::distance(std::cbegin(_line), it));
        }

        [[nodiscard]] constexpr rich_referring_segment_type rich_referring_segment(std::size_t pos) const {
            return rich_referring_segment_type{referring_segment(pos)};
        }

        [[nodiscard]] constexpr rich_referring_segment_type rich_referring_segment(
                typename line_type::const_iterator it) const {
            return rich_referring_segment_type{referring_segment(it)};
        }

        [[nodiscard]] constexpr const point_type &operator[](std::size_t pos) const {
            return _line[pos];
        }

        [[nodiscard]] constexpr point_type &operator[](std::size_t pos) {
            return _line[pos];
        }

        [[nodiscard]] constexpr const point_type &at(std::size_t pos) const {
            return _line.at(pos);
        }

        [[nodiscard]] constexpr point_type &at(std::size_t pos) {
            return _line.at(pos);
        }

        [[nodiscard]] constexpr const point_type &front() const {
            return _line.front();
        }

        [[nodiscard]] constexpr point_type &front() {
            return _line.front();
        }

        [[nodiscard]] constexpr const point_type &back() const {
            return _line.back();
        }

        [[nodiscard]] constexpr point_type &back() {
            return _line.back();
        }

        [[nodiscard]] constexpr bool empty() const {
            return _line.empty();
        }

        [[nodiscard]] constexpr std::size_t size() const {
            return _line.size();
        }

        [[nodiscard]] constexpr bool attaches(const is_rich_line auto &rich_line) const {
            if (not empty() and not rich_line.empty()) {
                return geometry::equals_points(back(), rich_line.front());
            }
            return false;
        }

        [[nodiscard]] line_type sub_line(std::size_t from, std::size_t to) const requires
            (std::default_initializable<allocator_type>) {
            if (from <= to and from < size() and to <= size()) {
                return line_type{_line.cbegin() + from, _line.cbegin() + to};
            }

            throw std::out_of_range(std::format("rich_line::sub_line(from: {}, to: {}): out of range, size: {}",
                    from, to, size()));
        }

        [[nodiscard]] line_type sub_line(std::size_t from, std::size_t to,
                const allocator_type &allocator) const {
            if (from <= to and from < size() and to <= size()) {
                return line_type{_line.cbegin() + from, _line.cbegin() + to, allocator};
            }

            throw std::out_of_range(std::format("rich_line::sub_line(from: {}, to: {}): out of range, size: {}",
                    from, to, size()));
        }

        [[nodiscard]] rich_line sub_rich_line(std::size_t from, std::size_t to) const requires
            (std::default_initializable<allocator_type>) {
            return rich_line{sub_line(from, to)};
        }

        [[nodiscard]] rich_line sub_rich_line(std::size_t from, std::size_t to,
                const allocator_type &allocator) const {
            return rich_line{sub_line(from, to, allocator), allocator};
        }

        void erase(std::size_t index) {
            if (index < _line.size()) {
                _line.erase(std::next(_line.begin(), index));
            }
        }

        void erase(const std::vector<std::size_t> &indices) {
            if (not indices.empty()) {
                for (std::size_t i = indices.size(); i-- > 0;) {
                    erase(indices[i]);
                }
            }
        }

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(line());
        }

        [[nodiscard]] std::string str() const {
            std::string str = "RichLine(";
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

        template<is_rich_line... RichLines>
        [[nodiscard]] std::size_t calculate_new_size(const RichLines &... rich_lines) const requires
            (... && std::is_base_of_v<rich_line, RichLines>) {
            std::size_t new_size = size();
            (_add_size(new_size, rich_lines), ...);
            return new_size;
        }

        template<is_rich_line RichLine>
        [[nodiscard]] std::size_t calculate_new_size(const std::vector<RichLine> &rich_lines) const requires
            (std::is_base_of_v<rich_line, RichLine>) {
            std::size_t new_size = size();
            for (const rich_line &next_rich_line : rich_lines) {
                _add_size(new_size, next_rich_line);
            }
            return new_size;
        }

        template<is_rich_line... RichLines>
        rich_line &merge(RichLines &&... rich_lines) requires
            (... && std::is_base_of_v<rich_line, RichLines>) {
            std::size_t new_size = calculate_new_size(rich_lines...);
            _line.reserve(new_size);

            (_merge_single(std::move(rich_lines)), ...);

            return *this;
        }

        template<is_rich_line RichLine>
        rich_line &merge(std::vector<RichLine> &&rich_lines) requires
            (std::is_base_of_v<rich_line, RichLine>) {
            std::size_t new_size = calculate_new_size(rich_lines);
            _line.reserve(new_size);

            for (rich_line &next_rich_line : rich_lines) {
                _merge_single(std::move(next_rich_line));
            }

            return *this;
        }

        friend bool operator==(const rich_line &left, const rich_line &right) {
            // if size of segments is not equal, lines cannot be equal
            if (left.size() != right.size()) {
                return false;
            }

            // if any point is not equal, lines cannot be equal
            for (std::size_t i = 0; i < left.size(); ++i) {
                if (not geometry::equals_points(left[i], right[i])) {
                    return false;
                }
            }

            // if all segments are equal, lines are equal
            return true;
        }

        friend bool operator!=(const rich_line &left, const rich_line &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const rich_line &rich_line) {
            return out << rich_line.str();
        }

        friend std::size_t hash_value(const rich_line &data) {
            std::size_t seed = 0;
            for (const auto &point : data.line()) {
                boost::hash_combine(seed, point);
            }
            return seed;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>) {
            ar & _line;
        }

    protected:
        line_type _line;

        template<is_rich_line RichLine>
        void _add_size(std::size_t &new_size, const RichLine &next_rich_line) const requires
            (std::is_base_of_v<rich_line, RichLine>) {
            if (next_rich_line.size() >= 1) {
                if (new_size == 0) {
                    ++new_size;
                }
                new_size += next_rich_line.size() - 1;
            }
        }

        template<is_rich_line RichLine>
        void _merge_single(RichLine &&next_rich_line) requires
            (std::is_base_of_v<rich_line, RichLine>) {
            if (not next_rich_line.empty()) {
                line_type &next_line = next_rich_line.line();
                if (_line.empty()) {
                    std::move(std::begin(next_line), std::end(next_line), std::back_inserter(_line));
                } else {
                    if (not geometry::equals_points(_line.back(), next_line.front())) {
                        throw rich_line_merge_exception(
                                std::format("last and first points of line\n{}\nand line\n{}\n do not match.",
                                        geometry::to_wkt(_line), geometry::to_wkt(next_line)));
                    }
                    std::move(std::next(std::begin(next_line)), std::end(next_line), std::back_inserter(_line));
                }
            }
        }

    };

}

namespace std {

    template<typename Point,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    struct hash<map_matching_2::geometry::rich_line<Point, Container, Allocator>> {
        using type = map_matching_2::geometry::rich_line<Point, Container, Allocator>;

        std::size_t operator()(const type &data) const noexcept {
            return hash_value(data);
        }
    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_RICH_LINE_DATA_HPP
