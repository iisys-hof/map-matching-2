// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_ROUTE_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_ROUTE_HPP

#include <format>
#include <list>
#include <vector>

#include "geometry/rich_type/multi_rich_line.hpp"
#include "geometry/rich_type/traits/rich_line.hpp"
#include "geometry/rich_type/traits/multi_rich_line.hpp"

#include "geometry/algorithm/equals.hpp"

#include "common.hpp"
#include "concepts.hpp"
#include "exception.hpp"

#include "route_data.hpp"

namespace map_matching_2::geometry::network {

    template<typename RichLineVariant>
    class route {

    public:
        using rich_line_variant_type = RichLineVariant;
        using rich_line_reference_variant_type = util::variant_wrapper_t<
            std::reference_wrapper, rich_line_variant_type, true>;

        template<is_rich_line RichLine>
        using rich_line_reference_type = std::reference_wrapper<RichLine>;

        template<is_rich_line RichLine>
        using rich_line_const_reference_type = std::reference_wrapper<const RichLine>;

        using rich_line_alternative_type = std::variant_alternative_t<0, rich_line_variant_type>;
        // using line_type = typename rich_line_traits<rich_line_type>::line_type;
        // using point_type = typename rich_line_traits<rich_line_type>::point_type;
        // using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        // using segment_type = typename rich_line_traits<rich_line_alternative_type>::segment_type;
        using point_type = typename rich_line_traits<rich_line_alternative_type>::point_type;
        using length_type = typename rich_line_traits<rich_line_alternative_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_alternative_type>::angle_type;

        constexpr route() = default;

        constexpr explicit route(bool is_invalid)
            : _is_invalid{is_invalid} {}

        template<is_rich_line RichLine>
        explicit route(RichLine rich_line) requires
            (util::variant_alternative_t<rich_line_variant_type, RichLine>)
            : route{std::list<rich_line_variant_type>{std::move(rich_line)}} {}

        explicit route(std::list<rich_line_variant_type> own_rich_lines)
            : _own_rich_lines{std::move(own_rich_lines)}, _is_invalid{false} {
            _set_references_from_own();
        }

        template<is_rich_line RichLine>
        constexpr explicit route(rich_line_const_reference_type<RichLine> rich_line_reference) requires
            (util::variant_alternative_t<rich_line_reference_variant_type, rich_line_const_reference_type<RichLine>>)
            : route{std::vector<rich_line_reference_variant_type>{std::move(rich_line_reference)}} {}

        constexpr route(std::vector<rich_line_reference_variant_type> rich_lines_references)
            : _rich_lines{std::move(rich_lines_references)}, _is_invalid{false} {}

        route(std::list<rich_line_variant_type> rich_lines,
                std::vector<rich_line_reference_variant_type> rich_lines_references)
            : _own_rich_lines{std::move(rich_lines)}, _rich_lines{std::move(rich_lines_references)},
            _is_invalid{false} {
            _check_references();
        }

        route(const route &other)
            : _is_invalid{other._is_invalid} {
            _copy_rich_lines(other);
        }

        route(route &&other) noexcept
            : _is_invalid{std::move(other._is_invalid)} {
            _move_rich_lines(std::move(other));
        }

        route &operator=(const route &other) {
            if (this != &other) {
                _is_invalid = other._is_invalid;
                _copy_rich_lines(other);
            }
            return *this;
        }

        route &operator=(route &&other) noexcept {
            if (this != &other) {
                _is_invalid = std::move(other._is_invalid);
                _move_rich_lines(std::move(other));
            }
            return *this;
        }

        constexpr ~route() = default;

        [[nodiscard]] constexpr bool is_invalid() const {
            return _is_invalid;
        }

        [[nodiscard]] constexpr bool empty() const {
            return _rich_lines.empty();
        }

        [[nodiscard]] constexpr std::size_t size() const {
            return _rich_lines.size();
        }

        [[nodiscard]] bool is_connected() const {
            if (empty()) {
                return true;
            } else if (size() == 1) {
                return true;
            } else {
                angle_type directions = default_float_type<angle_type>::v0;
                angle_type absolute_directions = default_float_type<angle_type>::v0;

                for (std::size_t i = 0, j = 1; j < size(); ++i, ++j) {
                    const auto &curr_rich_line = _rich_lines[i];
                    const auto &next_rich_line = _rich_lines[j];
                    if (not _empty(curr_rich_line) and not _empty(next_rich_line)) {
                        // check if rich lines are attached
                        if (not _attaches(curr_rich_line, next_rich_line)) {
                            return false;
                        }
                    }
                }

                return true;
            }
        }

        [[nodiscard]] bool has_length() const {
            if (empty()) {
                return false;
            } else if (size() == 1) {
                const auto &rich_line = _rich_lines.front();
                return _has_length(rich_line);
            } else {
                for (std::size_t i = 0, j = 1; j < size(); ++i, ++j) {
                    const auto &curr_rich_line = _rich_lines[i];

                    // check current length
                    if (_has_length(curr_rich_line)) {
                        return true;
                    }

                    const auto &next_rich_line = _rich_lines[j];
                    // in last loop, also check next length
                    if (j + 1 >= size() and _has_length(next_rich_line)) {
                        return true;
                    }
                }
                return false;
            }
        }

        [[nodiscard]] length_type length() const {
            if (empty()) {
                // nothing to do, throw
            } else if (size() == 1) {
                const auto &rich_line = _rich_lines.front();
                if (_has_length(rich_line)) {
                    return _get_length(rich_line);
                }
            } else {
                bool has_length = false;
                length_type length = default_float_type<length_type>::v0;

                for (std::size_t i = 0, j = 1; j < size(); ++i, ++j) {
                    const auto &curr_rich_line = _rich_lines[i];
                    const auto &next_rich_line = _rich_lines[j];

                    // add current length
                    if (_has_length(curr_rich_line)) {
                        length += _get_length(curr_rich_line);
                        has_length = true;
                    }
                    // in last loop, also add next length
                    if (j + 1 >= size() and _has_length(next_rich_line)) {
                        length += _get_length(next_rich_line);
                        has_length = true;
                    }
                }

                if (has_length) {
                    return length;
                }
            }

            throw length_computation_exception{std::format("cannot compute length for route: {}", wkt())};
        }

        [[nodiscard]] bool has_azimuth() const {
            if (empty()) {
                return false;
            } else if (size() == 1) {
                const auto &rich_line = _rich_lines.front();
                return _has_azimuth(rich_line);
            } else {
                for (auto first_it = _rich_lines.cbegin();
                     first_it != _rich_lines.cend(); ++first_it) {
                    const auto &first_rich_line = *first_it;

                    if (not _empty(first_rich_line)) {
                        for (auto last_it = _rich_lines.crbegin();
                             last_it != _rich_lines.crend(); ++last_it) {
                            const auto &last_rich_line = *last_it;

                            if (not _empty(last_rich_line)) {
                                const auto &first_point = _front(first_rich_line);
                                const auto &last_point = _back(last_rich_line);

                                return not geometry::equals_points(first_point, last_point);
                            }
                        }
                        break;
                    }
                }
                return false;
            }
        }

        [[nodiscard]] angle_type azimuth() const {
            if (empty()) {
                // nothing to do, throw
            } else if (size() == 1) {
                const auto &rich_line = _rich_lines.front();
                if (_has_azimuth(rich_line)) {
                    return _get_azimuth(rich_line);
                }
            } else {
                for (auto first_it = _rich_lines.cbegin();
                     first_it != _rich_lines.cend(); ++first_it) {
                    const auto &first_rich_line = *first_it;

                    if (not _empty(first_rich_line)) {
                        for (auto last_it = _rich_lines.crbegin();
                             last_it != _rich_lines.crend(); ++last_it) {
                            const auto &last_rich_line = *last_it;

                            if (not _empty(last_rich_line)) {
                                const auto &first_point = _front(first_rich_line);
                                const auto &last_point = _back(last_rich_line);

                                if (not geometry::equals_points(first_point, last_point)) {
                                    return geometry::azimuth_deg(first_point, last_point);
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
            }

            throw azimuth_computation_exception{std::format("cannot compute azimuth for route: {}", wkt())};
        }

        [[nodiscard]] bool has_directions() const {
            if (empty()) {
                return false;
            } else if (size() == 1) {
                const auto &rich_line = _rich_lines.front();
                return _has_directions(rich_line);
            } else {
                for (std::size_t i = 0, j = 1; j < size(); ++i, ++j) {
                    const auto &curr_rich_line = _rich_lines[i];
                    const auto &next_rich_line = _rich_lines[j];
                    if (not _empty(curr_rich_line) and not _empty(next_rich_line)) {
                        // check if rich lines are attached
                        if (_attaches(curr_rich_line, next_rich_line)) {
                            // has direction only when attached
                            if (_size(curr_rich_line) >= 2 and
                                _has_azimuth(curr_rich_line, _size(curr_rich_line) - 2) and
                                _has_azimuth(next_rich_line, 0)) {
                                return true;
                            }
                        }
                    }

                    // add remaining directions
                    if (_has_directions(curr_rich_line)) {
                        return true;
                    }
                    // in last loop, also add next directions
                    if (j + 1 >= size() and _has_directions(next_rich_line)) {
                        return true;
                    }
                }
                return false;
            }
        }

        [[nodiscard]] angle_type directions() const {
            if (empty()) {
                // nothing to do, throw
            } else if (size() == 1) {
                const auto &rich_line = _rich_lines.front();
                if (_has_directions(rich_line)) {
                    return _get_directions(rich_line);
                }
            } else {
                bool has_directions = false;
                angle_type directions = default_float_type<angle_type>::v0;

                for (std::size_t i = 0, j = 1; j < size(); ++i, ++j) {
                    const auto &curr_rich_line = _rich_lines[i];
                    const auto &next_rich_line = _rich_lines[j];
                    if (not _empty(curr_rich_line) and not _empty(next_rich_line)) {
                        // check if rich lines are attached
                        if (_attaches(curr_rich_line, next_rich_line)) {
                            // add direction inbetween only when attached
                            if (_size(curr_rich_line) >= 2 and
                                _has_azimuth(curr_rich_line, _size(curr_rich_line) - 2) and
                                _has_azimuth(next_rich_line, 0)) {
                                const auto direction = geometry::direction_deg(
                                        _get_azimuth(curr_rich_line, _size(curr_rich_line) - 2),
                                        _get_azimuth(next_rich_line, 0));
                                directions += direction;
                                has_directions = true;
                            }
                        }
                    }

                    // add remaining directions
                    if (_has_directions(curr_rich_line)) {
                        directions += _get_directions(curr_rich_line);
                        has_directions = true;
                    }
                    // in last loop, also add next directions
                    if (j + 1 >= size() and _has_directions(next_rich_line)) {
                        directions += _get_directions(next_rich_line);
                        has_directions = true;
                    }
                }

                if (has_directions) {
                    return directions;
                }
            }

            throw directions_computation_exception{std::format("cannot compute directions for route: {}", wkt())};
        }

        [[nodiscard]] angle_type absolute_directions() const {
            if (empty()) {
                // nothing to do, throw
            } else if (size() == 1) {
                const auto &rich_line = _rich_lines.front();
                if (_has_directions(rich_line)) {
                    return _get_absolute_directions(rich_line);
                }
            } else {
                bool has_directions = false;
                angle_type absolute_directions = default_float_type<angle_type>::v0;

                for (std::size_t i = 0, j = 1; j < size(); ++i, ++j) {
                    const auto &curr_rich_line = _rich_lines[i];
                    const auto &next_rich_line = _rich_lines[j];
                    if (not _empty(curr_rich_line) and not _empty(next_rich_line)) {
                        // check if rich lines are attached
                        if (_attaches(curr_rich_line, next_rich_line)) {
                            // add direction inbetween only when attached
                            if (_size(curr_rich_line) >= 2 and
                                _has_azimuth(curr_rich_line, _size(curr_rich_line) - 2) and
                                _has_azimuth(next_rich_line, 0)) {
                                const auto direction = geometry::direction_deg(
                                        _get_azimuth(curr_rich_line, _size(curr_rich_line) - 2),
                                        _get_azimuth(next_rich_line, 0));
                                absolute_directions += std::fabs(direction);
                                has_directions = true;
                            }
                        }
                    }

                    // add remaining directions
                    if (_has_directions(curr_rich_line)) {
                        absolute_directions += _get_absolute_directions(curr_rich_line);
                        has_directions = true;
                    }
                    // in last loop, also add next directions
                    if (j + 1 >= size() and _has_directions(next_rich_line)) {
                        absolute_directions += _get_absolute_directions(next_rich_line);
                        has_directions = true;
                    }
                }

                if (has_directions) {
                    return absolute_directions;
                }
            }

            throw directions_computation_exception{std::format("cannot compute directions for route: {}", wkt())};
        }

        [[nodiscard]] constexpr const std::vector<rich_line_reference_variant_type> &rich_lines() const {
            return _rich_lines;
        }

        [[nodiscard]] constexpr std::vector<rich_line_reference_variant_type> &rich_lines() {
            return _rich_lines;
        }

        [[nodiscard]] bool attaches(const route &other) const {
            if (is_invalid() or other.is_invalid()) [[unlikely]] {
                return false;
            }

            if (empty() or other.empty()) [[unlikely]] {
                return true;
            } else [[likely]] {
                const auto &prev_rich_line = _rich_lines.back();
                const auto &next_rich_line = other._rich_lines.front();

                if (_empty(prev_rich_line) or _empty(next_rich_line)) [[unlikely]] {
                    return true;
                } else {
                    const auto &prev_point = _back(prev_rich_line);
                    const auto &next_point = _front(next_rich_line);
                    return geometry::equals_points(prev_point, next_point);
                }
            }
        }

        [[nodiscard]] bool mergeable() const {
            if (is_invalid() or empty()) [[unlikely]] {
                return false;
            }

            if (std::all_of(_rich_lines.cbegin(), _rich_lines.cend(), [](const auto &rich_line) {
                return _size(rich_line) < 2;
            })) [[unlikely]] {
                return false;
            }

            for (const auto &rich_line : _rich_lines) [[likely]] {
                for (std::size_t i = 0, j = 1; j < _size(rich_line); ++i, ++j) [[likely]] {
                    const auto &_first = _at(rich_line, i);
                    const auto &_second = _at(rich_line, j);
                    if (not geometry::equals_points(_first, _second)) [[likely]] {
                        return true;
                    }
                }
            }

            return false;
        }

        template<is_route RouteT = route>
        [[nodiscard]] RouteT sub_route(std::size_t from, std::size_t to) const {
            using _route_type = RouteT;

            if (from <= to and from < size() and to <= size()) {
                std::list<rich_line_variant_type> new_rich_lines;
                std::vector<rich_line_reference_variant_type> new_references;
                new_references.reserve(to - from);

                // rich line copyer
                auto copy_rich_line = [&](const rich_line_reference_variant_type &rich_line_ref,
                        auto &next_rich_it, const auto &end_rich_it) {
                    bool data_moved = false;
                    while (next_rich_it != end_rich_it) {
                        const rich_line_variant_type &next_rich_line = *next_rich_it;
                        if (_equal_reference(next_rich_line, rich_line_ref)) {
                            // reference has local data, move it
                            const auto &_own_rich_line = new_rich_lines.emplace_back(next_rich_line);
                            std::visit([&new_references](auto &&_rich_line) {
                                new_references.emplace_back(std::cref(_rich_line));
                            }, _own_rich_line);
                            ++next_rich_it;
                            data_moved = true;
                            break;
                        }
                        ++next_rich_it;
                    }

                    // only reference movement
                    if (not data_moved) {
                        new_references.emplace_back(rich_line_ref);
                    }
                };

                // copy complete refs from self up to beginning
                auto next_own_rich_it = _own_rich_lines.begin();
                for (std::size_t i = from; i < to; ++i) {
                    copy_rich_line(_rich_lines[i], next_own_rich_it, _own_rich_lines.end());
                }

                return _route_type{std::move(new_rich_lines), std::move(new_references)};
            }

            throw std::out_of_range(std::format("route::sub_route(from: {}, to: {}): out of range, size: {}",
                    from, to, size()));
        }

        template<is_rich_line RichLineT, is_route RouteT = route>
        [[nodiscard]] RouteT sub_route(std::size_t from, std::size_t from_pos,
                std::size_t to, std::size_t to_pos) const {
            using _route_type = RouteT;
            using _rich_line_type = RichLineT;

            if (from <= to and from < size() and to <= size() and
                not _empty(_rich_lines[from]) and from_pos < _size(_rich_lines[from]) and
                (to == 0 or (not _empty(_rich_lines[to - 1]) and to_pos <= _size(_rich_lines[to - 1])))) {
                std::vector<_route_type> routes{};
                routes.reserve(to - from + 2); // plus two for possible from_pos and to_pos sub_rich_lines

                if (from_pos > 0) {
                    routes.emplace_back(_route_type{
                            _as<_rich_line_type>(_rich_lines[from]).
                            sub_rich_line(from_pos, _size(_rich_lines[from]))
                    });
                    from++;
                }

                if (to > 0 and to_pos < _size(_rich_lines[to - 1])) {
                    routes.emplace_back(sub_route<_route_type>(from, to - 1));
                    routes.emplace_back(_route_type{
                            _as<_rich_line_type>(_rich_lines[to - 1]).
                            sub_rich_line(0, to_pos)
                    });
                } else if (from < size()) {
                    routes.emplace_back(sub_route<_route_type>(from, to));
                }

                return merge<_rich_line_type, _route_type>(std::move(routes));
            }

            throw std::out_of_range(std::format("route::sub_route(from: {}, from_pos: {}, to: {}, to_pos: {}): "
                    "out of range, size: {}, size_from: {}, size_to: {}",
                    from, from_pos, to, to_pos, size(),
                    from < size() and not _empty(_rich_lines[from])
                    ? _size(_rich_lines[from])
                    : -1,
                    to <= size() and not _empty(_rich_lines[to - 1])
                    ? _size(_rich_lines[to - 1])
                    : -1));
        }

        [[nodiscard]] route_return_line_parts find_return_line(const route &other) const {
            route_return_line_parts result;

            if (empty() or other.empty() or not attaches(other)) {
                return result;
            }

            std::size_t i = size() - 1;
            std::size_t j = std::numeric_limits<std::size_t>::max();
            std::size_t k = 0;
            std::size_t l = 0;
            if (i >= 0 and k < other.size()) {
                // find first valid position on both lines
                while (i > 0 and _empty(_rich_lines[i])) {
                    --i;
                }
                if (i == 0 and _empty(_rich_lines[i])) {
                    return result;
                }
                j = _size(_rich_lines[i]) - 1;

                while (k < other.size() and _empty(other._rich_lines[k])) {
                    ++k;
                }
                if (k == other.size()) {
                    return result;
                }
                l = 0;

                result.self.end.ref = i;
                result.self.end.index = j;
                result.other.start.ref = k;
                result.other.start.index = l;

                const auto *point_a = &_at(_rich_lines[i], j);
                const auto *point_b = &_at(other._rich_lines[k], l);

                // traverse equal part
                while (geometry::equals_points(*point_a, *point_b)) {
                    result.self.start.ref = i;
                    result.self.start.index = j;
                    result.other.end.ref = k;
                    result.other.end.index = l;

                    // compute new bounds
                    const auto *new_point_a = point_a;
                    do {
                        result.self.start.ref = i;
                        result.self.start.index = j;

                        if (j > 0) {
                            --j;
                        } else {
                            if (i > 0) {
                                --i;
                            } else {
                                return result;
                            }
                            while (i > 0 and _empty(_rich_lines[i])) {
                                --i;
                            }
                            if (i == 0 and _empty(_rich_lines[i])) {
                                return result;
                            }
                            j = _size(_rich_lines[i]) - 1;
                            //--l;
                        }

                        new_point_a = &_at(_rich_lines[i], j);
                    } while (geometry::equals_points(*new_point_a, *point_a));
                    point_a = new_point_a;

                    const auto *new_point_b = point_b;
                    do {
                        result.other.end.ref = k;
                        result.other.end.index = l;

                        if (l < _size(other._rich_lines[k]) - 1) {
                            ++l;
                        } else {
                            ++k;
                            while (k < other.size() and _empty(other._rich_lines[k])) {
                                ++k;
                            }
                            if (k == other.size()) {
                                return result;
                            }
                            l = 0;
                            //++j;
                        }

                        new_point_b = &_at(other._rich_lines[k], l);
                    } while (geometry::equals_points(*new_point_b, *point_b));
                    point_b = new_point_b;
                }
            }

            return result;
        }

        template<is_rich_line RichLineT>
        [[nodiscard]] RichLineT extract_return_line(const route &other, bool from_self = true) const {
            using _rich_line_type = RichLineT;

            route_return_line_parts search = find_return_line(other);
            if (search != route_return_line_parts{}) {
                if (from_self) {
                    return sub_route<_rich_line_type>(
                                    search.self.start.ref, search.self.start.index,
                                    search.self.end.ref + 1, search.self.end.index + 1).
                            template get_rich_line<_rich_line_type>();
                } else {
                    return other.template sub_route<_rich_line_type>(
                                         search.other.start.ref, search.other.start.index,
                                         search.other.end.ref + 1, search.other.end.index + 1).
                                 template get_rich_line<_rich_line_type>();
                }
            }

            return _rich_line_type{};
        }

        template<is_rich_line RichLineT, is_route RouteT = route>
        [[nodiscard]] RouteT trim_merge(const RouteT &other) const {
            using _route_type = RouteT;
            using _rich_line_type = RichLineT;

            route_return_line_parts search = find_return_line(other);
            if (search != route_return_line_parts{}) {
                auto a = sub_route<_rich_line_type, _route_type>(
                        0, 0, search.self.start.ref + 1, search.self.start.index + 1);
                auto b = other.template sub_route<_rich_line_type, _route_type>(
                        search.other.end.ref, search.other.end.index,
                        other.size(), _size(other._rich_lines[other.size() - 1]));

                if (not a.empty() and not b.empty() and
                    _reverse_attaches(a._rich_lines.back(), b._rich_lines.front())) {
                    // reversal detected, trim reverse part by joining
                    join<_rich_line_type, _route_type>(a, b);
                }

                return merge<_rich_line_type, _route_type>(std::vector<_route_type>{std::move(a), std::move(b)});
            }

            return _route_type{true};
        }

        template<is_rich_line RichLineT>
        [[nodiscard]] RichLineT get_rich_line() const {
            using _rich_line_type = RichLineT;

            return _rich_line_type{}.merge(_get_rich_lines<_rich_line_type>(true));
        }

        template<is_multi_rich_line MultiRichLineT>
        [[nodiscard]] MultiRichLineT get_multi_rich_line() const {
            using _multi_rich_line_type = MultiRichLineT;
            using _rich_line_type = typename multi_rich_line_traits<_multi_rich_line_type>::rich_line_type;
            using _rich_lines_container_type = typename multi_rich_line_traits<
                _multi_rich_line_type>::rich_lines_container_type;

            auto multi_rich_lines = _get_multi_rich_lines<_rich_line_type>();
            _rich_lines_container_type rich_lines;
            rich_lines.reserve(multi_rich_lines.size());
            for (auto &rich_lines_collection : multi_rich_lines) {
                rich_lines.emplace_back(_rich_line_type{}.merge(std::move(rich_lines_collection)));
            }

            return _multi_rich_line_type{std::move(rich_lines)};
        }

        template<is_rich_line RichLineT, is_route RouteT = route>
        [[nodiscard]] static RouteT merge(
                std::vector<RouteT> &&routes, bool connect = false, bool non_connect_except = false) {
            using _route_type = RouteT;
            using _rich_line_type = RichLineT;
            using _line_type = typename rich_line_traits<_rich_line_type>::line_type;

            // if no routes given, return invalid route
            if (routes.empty()) {
                return _route_type{true};
            }

            // if all routes are invalid, return invalid route
            if (std::all_of(routes.cbegin(), routes.cend(), [](const auto &route) {
                return route.is_invalid();
            })) {
                return _route_type{true};
            }

            // calculate new size, might be a bit too large due to joints
            std::vector<bool> mergeables(routes.size(), false);
            std::size_t new_references_size = routes.size() - 1;
            for (std::size_t i = 0; i < routes.size(); ++i) {
                const auto &route = routes[i];
                mergeables[i] = route.mergeable();
                if (mergeables[i]) {
                    new_references_size += route.size();
                }
            }

            // prepare new storages
            std::list<rich_line_variant_type> new_rich_lines;
            std::vector<rich_line_reference_variant_type> new_references;
            new_references.reserve(new_references_size);

            // merge routes
            for (std::size_t i = 0; i < routes.size(); ++i) {
                auto &route = routes[i];
                if (mergeables[i]) {
                    // test if routes are attached
                    if (not new_references.empty()) {
                        const auto &prev = new_references.back();
                        if (not _empty(prev)) {
                            const auto &next = route._rich_lines.front();
                            if (not _empty(next)) {
                                const auto &prev_point = _back(prev);
                                const auto &next_point = _front(next);
                                if (not geometry::equals_points(prev_point, next_point)) {
                                    if (connect) {
                                        // connect both routes with a new segment inbetween
                                        _rich_line_type connection{_line_type{prev_point, next_point}};
                                        const auto &_own_rich_line = new_rich_lines.emplace_back(std::move(connection));
                                        std::visit([&new_references](auto &&_rich_line) {
                                            new_references.emplace_back(std::cref(_rich_line));
                                        }, _own_rich_line);
                                    } else if (non_connect_except) {
                                        throw route_merge_exception{};
                                    }
                                }
                            }
                        }
                    }

                    auto next_rich_it = route._own_rich_lines.begin();
                    for (auto next_it = route._rich_lines.begin();
                         next_it != route._rich_lines.end(); ++next_it) {
                        const auto &next = *next_it;

                        // local data movement
                        bool data_moved = false;
                        if (next_rich_it != route._own_rich_lines.end()) {
                            auto &next_rich_line = *next_rich_it;
                            if (_equal_reference(next_rich_line, next)) {
                                // reference has local data, move it
                                const auto &_own_rich_line = new_rich_lines.emplace_back(std::move(next_rich_line));
                                std::visit([&new_references](auto &&_rich_line) {
                                    new_references.emplace_back(std::cref(_rich_line));
                                }, _own_rich_line);
                                ++next_rich_it;
                                data_moved = true;
                            }
                        }

                        // only reference movement
                        if (not data_moved) {
                            new_references.emplace_back(std::move(*next_it));
                        }
                    }
                }
            }

            assert(new_references_size >= new_references.size());

            return _route_type{std::move(new_rich_lines), std::move(new_references)};
        }

        template<is_rich_line RichLineT, is_route RouteT = route>
        static void join(RouteT &a, RouteT &b, bool retain_reversal = false) {
            using _route_type = RouteT;
            using _rich_line_type = RichLineT;
            using _line_type = typename rich_line_traits<_rich_line_type>::line_type;

            if (a.is_invalid() or b.is_invalid() or a.empty() or b.empty()) {
                // cannot join invalid or empty routes
                return;
            }

            const auto &a_back_ref = a._rich_lines.back();
            const auto &b_front_ref = b._rich_lines.front();

            if (_empty(a_back_ref) or _empty(b_front_ref) or _size(a_back_ref) < 2 or _size(b_front_ref) < 2) {
                // cannot join empty lines or lines with only one point
                return;
            }

            if (retain_reversal and _reverse_attaches(a_back_ref, b_front_ref)) {
                // reversal detected, shall be retained, so stop join
                return;
            }

            std::visit([&a, &b, &a_back_ref, &b_front_ref](auto &&a_back) {
                std::visit([&a, &b, &a_back_ref, &b_front_ref, &a_back](auto &&b_front) {
                    _line_type a_back_line{
                            std::cbegin(a_back.get().line()),
                            std::prev(std::cend(a_back.get().line()))
                    };
                    _line_type b_front_line{
                            std::next(std::cbegin(b_front.get().line())),
                            std::cend(b_front.get().line())
                    };
                    a_back_line.emplace_back(b_front_line.front());

                    if (a_back_line.size() < 2) {
                        a_back_line.clear();
                    }
                    if (b_front_line.size() < 2) {
                        b_front_line.clear();
                    }

                    _rich_line_type new_a_back{std::move(a_back_line)};
                    _rich_line_type new_b_front{std::move(b_front_line)};

                    if (not a._own_rich_lines.empty() and
                        _equal_reference(a._own_rich_lines.back(), a_back_ref)) {
                        a._own_rich_lines.pop_back();
                    }

                    if (not b._own_rich_lines.empty() and
                        _equal_reference(b._own_rich_lines.front(), b_front_ref)) {
                        b._own_rich_lines.pop_front();
                    }

                    if (new_a_back.empty()) {
                        a._rich_lines.pop_back();
                    } else {
                        const auto &new_a_back_ref = a._own_rich_lines.emplace_back(std::move(new_a_back));
                        std::visit([&a](auto &&new_a_back) {
                            a._rich_lines.back() = std::cref(new_a_back);
                        }, new_a_back_ref);
                    }

                    if (new_b_front.empty()) {
                        b._rich_lines.erase(std::begin(b._rich_lines));
                    } else {
                        const auto &new_b_front_ref = b._own_rich_lines.emplace_front(std::move(new_b_front));
                        std::visit([&b](auto &&new_b_front) {
                            b._rich_lines.front() = std::cref(new_b_front);
                        }, new_b_front_ref);
                    }
                }, b_front_ref);
            }, a_back_ref);
        }

        [[nodiscard]] std::string wkt() const {
            return get_multi_rich_line<multi_rich_line<rich_line<point_type>>>().wkt();
        }

        [[nodiscard]] constexpr static std::vector<std::string> header() {
            return {"geometry", "invalid", "connected", "length", "azimuth", "directions", "absolute-directions"};
        }

        [[nodiscard]] std::vector<std::string> row() const {
            std::vector<std::string> row;
            row.reserve(7);
            row.emplace_back(wkt());
            row.emplace_back(std::to_string(is_invalid()));
            row.emplace_back(std::to_string(is_connected()));
            if (has_length()) {
                row.emplace_back(std::to_string(length()));
            }
            if (has_azimuth()) {
                row.emplace_back(std::to_string(azimuth()));
            }
            if (has_directions()) {
                row.emplace_back(std::to_string(directions()));
                row.emplace_back(std::to_string(absolute_directions()));
            }
            return row;
        }

        [[nodiscard]] std::string str() const {
            const auto &fields = row();
            std::string str = "Route(";
            bool first = true;
            for (const auto &field : fields) {
                if (!first) {
                    str.append(",");
                }
                first = false;
                str.append(field);
            }
            str.append(")");
            return str;
        }

        friend std::ostream &operator<<(std::ostream &out, const route &route) {
            return out << route.str();
        }

    protected:
        std::list<rich_line_variant_type> _own_rich_lines{};
        std::vector<rich_line_reference_variant_type> _rich_lines{};
        bool _is_invalid{true};

        friend route_data<length_type, angle_type>;

        void _move_rich_lines(route &&other) {
            _own_rich_lines = std::move(other._own_rich_lines);
            _rich_lines = std::move(other._rich_lines);
        }

        void _copy_rich_lines(const route &other) {
            _own_rich_lines.clear();
            _rich_lines.clear();
            _rich_lines.reserve(other.size());
            auto rich_it = other._own_rich_lines.cbegin();
            for (auto reference_it = other._rich_lines.cbegin();
                 reference_it != other._rich_lines.cend(); ++reference_it) {
                const auto &reference = *reference_it;
                if (rich_it != other._own_rich_lines.cend()) {
                    const auto &rich_line = *rich_it;
                    if (_equal_reference(rich_line, reference)) {
                        // reference has local data
                        const auto &own_rich_line_variant = _own_rich_lines.emplace_back(rich_line);
                        std::visit([this](auto &&own_rich_line) {
                            _rich_lines.emplace_back(std::cref(own_rich_line));
                        }, own_rich_line_variant);
                        ++rich_it;
                        continue;
                    }
                }

                _rich_lines.emplace_back(std::cref(reference));
            }
        }

        void _set_references_from_own() {
            _rich_lines.reserve(_own_rich_lines.size());
            for (const auto &rich_line_variant : _own_rich_lines) {
                std::visit([this](auto &&rich_line) {
                    _rich_lines.emplace_back(std::cref(rich_line));
                }, rich_line_variant);
            }
        }

        void _check_references() const {
            // check references being correct
            std::size_t found = 0;
            auto reference_it = _rich_lines.cbegin();
            for (auto rich_it = _own_rich_lines.cbegin(); rich_it != _own_rich_lines.cend(); ++rich_it) {
                const auto &rich_line = *rich_it;
                while (reference_it != _rich_lines.cend()) {
                    const auto &reference = *reference_it++;
                    if (_equal_reference(rich_line, reference)) {
                        found++;
                        break;
                    }
                }
            }

            if (found != _own_rich_lines.size()) {
                throw std::logic_error("route references are incorrect");
            }
        }

        template<is_rich_line RichLineT>
        [[nodiscard]] std::vector<RichLineT> _get_rich_lines(bool attach_gaps = false) const {
            using _rich_line_type = RichLineT;
            using _line_type = typename rich_line_traits<_rich_line_type>::line_type;

            std::vector<_rich_line_type> rich_lines;
            rich_lines.reserve(size());

            for (const auto &rich_line_variant : _rich_lines) {
                if (not _empty(rich_line_variant)) {
                    _rich_line_type rich_line = _as<_rich_line_type>(rich_line_variant);
                    if (attach_gaps and not rich_lines.empty() and not rich_lines.back().attaches(rich_line)) {
                        rich_lines.emplace_back(_rich_line_type{
                                _line_type{
                                        rich_lines.back().back(), rich_line.front()
                                }
                        });
                    }
                    rich_lines.emplace_back(std::move(rich_line));
                }
            }

            return rich_lines;
        }

        template<is_rich_line RichLineT>
        [[nodiscard]] std::vector<std::vector<RichLineT>> _get_multi_rich_lines() const {
            using _rich_line_type = RichLineT;

            std::vector<std::vector<_rich_line_type>> multi_rich_lines;
            multi_rich_lines.reserve(size());

            std::vector<_rich_line_type> rich_lines;
            rich_lines.reserve(size());

            for (const auto &rich_line_variant : _rich_lines) {
                if (not _empty(rich_line_variant)) {
                    _rich_line_type rich_line = _as<_rich_line_type>(rich_line_variant);
                    if (not rich_lines.empty() and not rich_lines.back().attaches(rich_line)) {
                        // gap collect
                        multi_rich_lines.emplace_back(std::move(rich_lines));
                        rich_lines.clear();
                    }
                    rich_lines.emplace_back(std::move(rich_line));
                }
            }

            // final collect
            multi_rich_lines.emplace_back(std::move(rich_lines));

            return multi_rich_lines;
        }

        [[nodiscard]] static bool _reverse_attaches(const rich_line_reference_variant_type &rich_line_variant_a,
                const rich_line_reference_variant_type &rich_line_variant_b) {
            if (_size(rich_line_variant_a) >= 2 and _size(rich_line_variant_b) >= 2 and
                _has_azimuth(rich_line_variant_a, _size(rich_line_variant_a) - 2) and
                _has_azimuth(rich_line_variant_b, 0)) {
                const auto direction = std::fabs(geometry::direction_deg(
                        _get_azimuth(rich_line_variant_a, _size(rich_line_variant_a) - 2),
                        _get_azimuth(rich_line_variant_b, 0))) / 180.0;
                return direction > 0.99;
            }
            return false;
        }

        [[nodiscard]] static bool _empty(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> bool {
                return rich_line.get().empty();
            }, rich_line_variant);
        }

        [[nodiscard]] static std::size_t _size(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> std::size_t {
                return rich_line.get().size();
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _has_length(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> bool {
                return rich_line.get().has_length();
            }, rich_line_variant);
        }

        [[nodiscard]] static length_type _get_length(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> length_type {
                return rich_line.get().length();
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _has_azimuth(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> bool {
                return rich_line.get().has_azimuth();
            }, rich_line_variant);
        }

        [[nodiscard]] static angle_type _get_azimuth(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> angle_type {
                return rich_line.get().azimuth();
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _has_directions(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> bool {
                return rich_line.get().has_directions();
            }, rich_line_variant);
        }

        [[nodiscard]] static angle_type _get_directions(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> angle_type {
                return rich_line.get().directions();
            }, rich_line_variant);
        }

        [[nodiscard]] static angle_type _get_absolute_directions(
                const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> angle_type {
                return rich_line.get().absolute_directions();
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _has_length(const rich_line_reference_variant_type &rich_line_variant,
                std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> bool {
                return rich_line.get().has_length(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static length_type _get_length(const rich_line_reference_variant_type &rich_line_variant,
                std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> length_type {
                return rich_line.get().length(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _has_azimuth(const rich_line_reference_variant_type &rich_line_variant,
                std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> bool {
                return rich_line.get().has_azimuth(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static angle_type _get_azimuth(const rich_line_reference_variant_type &rich_line_variant,
                std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> angle_type {
                return rich_line.get().azimuth(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _has_direction(const rich_line_reference_variant_type &rich_line_variant,
                std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> bool {
                return rich_line.get().has_direction(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static angle_type _get_direction(const rich_line_reference_variant_type &rich_line_variant,
                std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> angle_type {
                return rich_line.get().direction(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static angle_type _get_absolute_direction(
                const rich_line_reference_variant_type &rich_line_variant, std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> angle_type {
                return rich_line.get().absolute_direction(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _attaches(const rich_line_reference_variant_type &rich_line_variant_a,
                const rich_line_reference_variant_type &rich_line_variant_b) {
            return std::visit([&rich_line_variant_b](auto &&rich_line_a) -> bool {
                return std::visit([&rich_line_a](auto &&rich_line_b) -> bool {
                    return rich_line_a.get().attaches(rich_line_b.get());
                }, rich_line_variant_b);
            }, rich_line_variant_a);
        }

        [[nodiscard]] static const point_type &_at(const rich_line_reference_variant_type &rich_line_variant,
                std::size_t pos) {
            return std::visit([pos](auto &&rich_line) -> const point_type &{
                return rich_line.get().at(pos);
            }, rich_line_variant);
        }

        [[nodiscard]] static const point_type &_front(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> const point_type &{
                return rich_line.get().front();
            }, rich_line_variant);
        }

        [[nodiscard]] static const point_type &_back(const rich_line_reference_variant_type &rich_line_variant) {
            return std::visit([](auto &&rich_line) -> const point_type &{
                return rich_line.get().back();
            }, rich_line_variant);
        }

        template<is_rich_line RichLineT>
        [[nodiscard]] static RichLineT _as(const rich_line_reference_variant_type &rich_line_variant) {
            using _rich_line_type = RichLineT;
            return std::visit([](auto &&rich_line) -> _rich_line_type {
                return _rich_line_type{rich_line.get()};
            }, rich_line_variant);
        }

        template<is_rich_line RichLineT>
        [[nodiscard]] static rich_line_reference_type<RichLineT> _reference(
                const rich_line_reference_variant_type &rich_line_variant) {
            using _rich_line_type = RichLineT;
            using _rich_line_reference_type = rich_line_reference_type<_rich_line_type>;
            return std::visit([](auto &&rich_line) -> _rich_line_reference_type {
                return _rich_line_reference_type{std::ref(rich_line.get())};
            }, rich_line_variant);
        }

        template<is_rich_line RichLineT>
        [[nodiscard]] static rich_line_const_reference_type<RichLineT> _const_reference(
                const rich_line_reference_variant_type &rich_line_variant) {
            using _rich_line_type = RichLineT;
            using rich_line_const_reference_type = rich_line_const_reference_type<_rich_line_type>;
            return std::visit([](auto &&rich_line) -> rich_line_const_reference_type {
                return rich_line_const_reference_type{std::cref(rich_line.get())};
            }, rich_line_variant);
        }

        [[nodiscard]] static bool _equal_reference(const rich_line_variant_type &rich_line_variant,
                const rich_line_reference_variant_type &rich_line_reference_variant) {
            return std::visit([&rich_line_reference_variant](auto &&rich_line) -> bool {
                return std::visit([&rich_line](auto &&rich_line_reference) -> bool {
                    const auto &reference = rich_line_reference.get();
                    if constexpr (std::is_same_v<decltype(&rich_line), decltype(&reference)>) {
                        return &rich_line == &reference;
                    } else {
                        return false;
                    }
                }, rich_line_reference_variant);
            }, rich_line_variant);
        }

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_ROUTE_HPP
