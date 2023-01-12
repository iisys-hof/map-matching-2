// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_ROUTE_HPP
#define MAP_MATCHING_2_ROUTE_HPP

#include <deque>

#include "../rich_line.hpp"
#include "../multi_rich_line.hpp"
#include "edge.hpp"

namespace map_matching_2::geometry::network {

    struct route_merge_exception : public std::exception {
    };

    template<typename RichLine>
    class route {

    public:
        using rich_line_type = RichLine;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;

        route()
                : route{true} {}

        explicit route(bool is_invalid)
                : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false},
                  _is_invalid{is_invalid} {}

        explicit route(line_type line)
                : route{rich_line_type{std::move(line)}} {}

        explicit route(rich_line_type rich_line)
                : route{std::deque<rich_line_type>{std::move(rich_line)}} {}

        explicit route(std::deque<rich_line_type> own_rich_lines)
                : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false}, _is_invalid{false},
                  _own_rich_lines{std::move(own_rich_lines)} {
            this->_rich_lines.reserve(_own_rich_lines.size());
            for (const auto &_rich_line: _own_rich_lines) {
                this->_rich_lines.emplace_back(std::cref(_rich_line));
            }
        }

        explicit route(std::reference_wrapper<const rich_line_type> rich_line_reference)
                : route{std::vector<std::reference_wrapper<const rich_line_type>>{std::move(rich_line_reference)}} {}

        explicit route(std::vector<std::reference_wrapper<const rich_line_type>> rich_lines_references)
                : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false}, _is_invalid{false},
                  _rich_lines{std::move(rich_lines_references)} {}

        route(std::deque<rich_line_type> rich_lines,
              std::vector<std::reference_wrapper<const rich_line_type>> rich_lines_references)
                : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false}, _is_invalid{false},
                  _own_rich_lines{std::move(rich_lines)}, _rich_lines{std::move(rich_lines_references)} {
            _check_references();
        }

        ~route() = default;

        route(const route &other) {
            _copy(other);
        }

        route(route &&other) noexcept {
            _move(std::move(other));
        }

        route &operator=(const route &other) {
            _copy(other);
            return *this;
        }

        route &operator=(route &&other) noexcept {
            _move(std::move(other));
            return *this;
        }

        [[nodiscard]] bool is_invalid() const {
            return _is_invalid;
        }

        [[nodiscard]] bool is_connected() const {
            _compute_directions();
            return _is_connected;
        }

        [[nodiscard]] bool has_length() const {
            _compute_length();
            return _has_length;
        }

        [[nodiscard]] length_type length() const {
            _compute_length();
            return _length;
        }

        [[nodiscard]] bool has_azimuth() const {
            _compute_azimuth();
            return _has_azimuth;
        }

        [[nodiscard]] angle_type azimuth() const {
            _compute_azimuth();
            return _azimuth;
        }

        [[nodiscard]] bool has_directions() const {
            _compute_directions();
            return _has_directions;
        }

        [[nodiscard]] angle_type directions() const {
            _compute_directions();
            return _directions;
        }

        [[nodiscard]] angle_type absolute_directions() const {
            _compute_directions();
            return _absolute_directions;
        }

        [[nodiscard]] const std::vector<std::reference_wrapper<const rich_line_type>> &rich_lines() const {
            return _rich_lines;
        }

        [[nodiscard]] bool attaches(const route &other) const {
            if (this->is_invalid() or other.is_invalid()) {
                return false;
            }

            if (this->_rich_lines.empty() or other._rich_lines.empty()) {
                return true;
            } else {
                const rich_line_type &prev_rich_line = this->_rich_lines.back();
                const rich_line_type &next_rich_line = other._rich_lines.front();

                if (prev_rich_line.rich_segments().empty() or next_rich_line.rich_segments().empty()) {
                    return true;
                } else {
                    const auto &prev_point = prev_rich_line.rich_segments().back().second();
                    const auto &next_point = next_rich_line.rich_segments().front().first();
                    return geometry::equals_points(prev_point, next_point);
                }
            }
        }

        [[nodiscard]] std::array<std::int64_t, 8> find_return_line(const route &other) const {
            // self.begin.rich_line, self.begin.rich_line.point, self.end.rich_line, self.end.rich_line.point,
            // other.begin.rich_line, other.begin.rich_line.point, other.end.rich_line, other.end.rich_line.point
            std::array<std::int64_t, 8> result{-1, -1, -1, -1, -1, -1, -1, -1};

            if (_rich_lines.empty() or other._rich_lines.empty() or not attaches(other)) {
                return result;
            }

            std::int64_t i = _rich_lines.size() - 1;
            std::int64_t j = 0;
            if (i >= 0 and j < other._rich_lines.size()) {
                // find first valid position on both lines
                std::int64_t k = ((std::int64_t) _rich_lines[i].get().size()) - 1;
                while (k < 0) {
                    i--;
                    if (i < 0) {
                        return result;
                    }
                    k = ((std::int64_t) _rich_lines[i].get().size()) - 1;
                }
                std::int64_t l = 0;
                while (l >= other._rich_lines[j].get().size()) {
                    j++;
                    if (j >= other._rich_lines.size()) {
                        return result;
                    }
                    l = 0;
                }
                result[2] = i;
                result[3] = k;
                result[4] = j;
                result[5] = l;

                const auto *point_a = &_rich_lines[i].get().at(k);
                const auto *point_b = &other._rich_lines[j].get().at(l);

                // traverse equal part
                bool start_saved = false;
                while (geometry::equals_points(*point_a, *point_b)) {
                    result[0] = i;
                    result[1] = k;
                    result[6] = j;
                    result[7] = l;

                    // compute new bounds
                    k--;
                    l++;

                    while (k < 0) {
                        i--;
                        if (i < 0) {
                            return result;
                        }
                        k = ((std::int64_t) _rich_lines[i].get().size()) - 1;
                        l--;
                    }
                    while (l >= other._rich_lines[j].get().size()) {
                        j++;
                        if (j >= other._rich_lines.size()) {
                            return result;
                        }
                        l = 0;
                        k++;
                    }

                    point_a = &_rich_lines[i].get().at(k);
                    point_b = &other._rich_lines[j].get().at(l);
                }
            }

            return result;
        }

        [[nodiscard]] rich_line_type extract_return_line(const route &other, bool from_self = true) const {
            std::vector<rich_segment_type> new_rich_segments;

            std::array<std::int64_t, 8> search = find_return_line(other);
            if (search != std::array<std::int64_t, 8>{-1, -1, -1, -1, -1, -1, -1, -1}) {
                if (from_self) {
                    std::int64_t i = search[0];
                    std::int64_t k = search[1];
                    while (i < search[2] or (i == search[2] and k <= search[3])) {
                        if ((i == search[0] and k > search[1]) or (i > search[0] and k > 0)) {
                            new_rich_segments.emplace_back(_rich_lines[i].get().rich_segments()[k - 1]);
                        }

                        k++;
                        while (k >= _rich_lines[i].get().size()) {
                            i++;
                            if (i >= _rich_lines.size()) {
                                return rich_line_type{std::move(new_rich_segments)};
                            }
                            k = 1;
                        }
                    }
                } else {
                    std::int64_t j = search[4];
                    std::int64_t l = search[5];
                    while (j < search[6] or (j == search[6] and l <= search[7])) {
                        if ((j == search[4] and l > search[5]) or (j > search[4] and l > 0)) {
                            new_rich_segments.emplace_back(other._rich_lines[j].get().rich_segments()[l - 1]);
                        }

                        l++;
                        while (l >= other._rich_lines[j].get().size()) {
                            j++;
                            if (j >= other._rich_lines.size()) {
                                return rich_line_type{std::move(new_rich_segments)};
                            }
                            l = 1;
                        }
                    }
                }
            }

            return rich_line_type{std::move(new_rich_segments)};
        }

        [[nodiscard]] route trim_merge(const route &other) const {
            std::array<std::int64_t, 8> search = find_return_line(other);
            if (search != std::array<std::int64_t, 8>{-1, -1, -1, -1, -1, -1, -1, -1}) {
                // calculate new size for trim
                std::size_t new_references_size = 1;
                if (search[0] >= 0) {
                    new_references_size += search[0];
                }
                if (search[6] >= 0) {
                    new_references_size += other._rich_lines.size() - (search[6] + 1);
                }

                // prepare new storages
                std::deque<rich_line_type> new_rich_lines;
                std::vector<std::reference_wrapper<const rich_line_type>> new_references;
                new_references.reserve(new_references_size);

                // rich line copyer
                const auto copy_rich_line = [&](const rich_line_type &next, auto &next_rich_it,
                                                const auto &end_rich_it) {
                    bool data_moved = false;
                    while (next_rich_it != end_rich_it) {
                        const rich_line_type &next_rich_line = *next_rich_it;
                        if (&next_rich_line == &next) {
                            // reference has local data, move it
                            const auto &_rich_line = new_rich_lines.emplace_back(next_rich_line);
                            new_references.emplace_back(_rich_line);
                            next_rich_it++;
                            data_moved = true;
                            break;
                        }
                        next_rich_it++;
                    }

                    // only reference movement
                    if (not data_moved) {
                        new_references.emplace_back(next);
                    }
                };

                // copy complete refs from self up to beginning
                auto next_self_rich_it = _own_rich_lines.begin();
                for (std::int64_t i = 0; i < search[0]; ++i) {
                    copy_rich_line(_rich_lines[i].get(), next_self_rich_it, _own_rich_lines.end());
                }

                // prepare middle part
                std::size_t new_segments_size = 1;
                if (search[1] >= 0) {
                    new_segments_size += search[1];
                }
                if (search[6] < other._rich_lines.size() and search[7] >= 0) {
                    new_segments_size += other._rich_lines[search[6]].get().size() - (search[7] + 1);
                }

                std::vector<rich_segment_type> new_rich_segments;
                new_rich_segments.reserve(new_segments_size);

                // copy remaining segments from self up to beginning
                if (search[1] > 0) {
                    // first segment is later attached, if there are more, copy them
                    for (std::int64_t k = 1; k < search[1]; ++k) {
                        new_rich_segments.emplace_back(_rich_lines[search[0]].get().rich_segments()[k - 1]);
                    }
                }

                // attach beginning from self to end of other in copy
                {
                    std::int64_t i = search[0];
                    std::int64_t k = search[1];
                    while (i >= 0 and k <= 0) {
                        i--;
                        if (i >= 0) {
                            k = ((std::int64_t) _rich_lines[i].get().size()) - 1;
                        }
                    }
                    const rich_segment_type *a_segment = nullptr;
                    if (i >= 0 and k >= 0) {
                        a_segment = &_rich_lines[i].get().rich_segments()[k - 1];
                    }

                    std::int64_t j = search[6];
                    std::int64_t l = search[7];
                    while (j < other._rich_lines.size() and l + 1 >= other._rich_lines[j].get().size()) {
                        j++;
                        if (j < other._rich_lines.size()) {
                            l = 0;
                        }
                    }
                    const rich_segment_type *b_segment = nullptr;
                    if (j < other._rich_lines.size() and l + 1 < other._rich_lines[j].get().size()) {
                        b_segment = &other._rich_lines[j].get().rich_segments()[l];
                    }

                    if (a_segment != nullptr and b_segment != nullptr) {
                        const auto direction =
                                std::fabs(geometry::direction_deg(a_segment->azimuth(), b_segment->azimuth())) / 180.0;
                        if (direction > 0.99) {
                            using segment_type = typename geometry::rich_segment_traits<rich_segment_type>::segment_type;
                            // reverse detected, attach to second point
                            new_rich_segments.emplace_back(
                                    rich_segment_type{segment_type{a_segment->first(), b_segment->second()}});
                        } else {
                            if (i >= search[0]) {
                                new_rich_segments.emplace_back(*a_segment);
                            }
                            if (j <= search[6]) {
                                new_rich_segments.emplace_back(*b_segment);
                            }
                        }
                    } else if (a_segment != nullptr and b_segment == nullptr and i >= search[0]) {
                        new_rich_segments.emplace_back(*a_segment);
                    } else if (a_segment == nullptr and b_segment != nullptr and j <= search[6]) {
                        new_rich_segments.emplace_back(*b_segment);
                    }
                }

                // copy remaining segments from other up to remaining complete refs
                if (search[7] + 1 < other._rich_lines[search[6]].get().size()) {
                    // first segment was earlier attached, if there are more, copy them
                    for (std::int64_t l = search[7] + 1;
                         l + 1 < other._rich_lines[search[6]].get().size(); ++l) {
                        new_rich_segments.emplace_back(other._rich_lines[search[6]].get().rich_segments()[l]);
                    }
                }

                assert(new_segments_size >= new_rich_segments.size());

                if (not new_rich_segments.empty()) {
                    const auto &_rich_line = new_rich_lines.emplace_back(rich_line_type{std::move(new_rich_segments)});
                    new_references.emplace_back(_rich_line);
                }

                // copy complete rest from other
                auto next_other_rich_it = other._own_rich_lines.begin();
                for (std::int64_t j = search[6] + 1; j < other._rich_lines.size(); ++j) {
                    copy_rich_line(other._rich_lines[j].get(), next_other_rich_it, other._own_rich_lines.end());
                }

                assert(new_references_size >= new_references.size());

                return route{std::move(new_rich_lines), std::move(new_references)};
            }

            return route{true};
        }

        [[nodiscard]] line_type get_line() const {
            std::size_t size = 0;
            for (const rich_line_type &rich_line: _rich_lines) {
                if (not rich_line.empty()) {
                    const auto points_size = rich_line.size();
                    if (size == 0) {
                        size += points_size;
                    } else {
                        size += points_size > 1 ? points_size - 1 : points_size;
                    }
                }
            }

            line_type line;
            line.reserve(size);
            for (const rich_line_type &rich_line: _rich_lines) {
                if (not rich_line.empty()) {
                    auto line_it_begin = rich_line.cbegin();
                    if (not line.empty() and geometry::equals_points(line.back(), *line_it_begin)) {
                        line_it_begin++;
                    }
                    line.insert(line.end(), line_it_begin, rich_line.cend());
                }
            }

            assert(size >= line.size());

            return line;
        }

        [[nodiscard]] rich_line_type get_rich_line() const {
            std::size_t size = 0;
            for (auto it = _rich_lines.cbegin(); it != _rich_lines.cend(); it++) {
                const rich_line_type &rich_line = *it;
                if (not rich_line.rich_segments().empty()) {
                    if (it != _rich_lines.cbegin()) {
                        const rich_line_type &prev_rich_line = *std::prev(it);
                        const point_type &prev = prev_rich_line.rich_segments().back().segment().second;
                        const point_type &next = rich_line.rich_segments().front().segment().first;
                        if (not geometry::equals_points(prev, next)) {
                            size++;
                        }
                    }
                    size += rich_line.rich_segments().size();
                }
            }

            std::vector<rich_segment_type> rich_segments;
            rich_segments.reserve(size);
            for (const rich_line_type &rich_line: _rich_lines) {
                if (not rich_line.rich_segments().empty()) {
                    if (not rich_segments.empty()) {
                        const point_type &prev = rich_segments.back().segment().second;
                        const point_type &next = rich_line.rich_segments().front().segment().first;
                        if (not geometry::equals_points(prev, next)) {
                            using segment_type = typename geometry::rich_segment_traits<rich_segment_type>::segment_type;
                            rich_segments.emplace_back(rich_segment_type{segment_type{prev, next}});
                        }
                    }
                    rich_segments.insert(rich_segments.end(),
                                         rich_line.rich_segments().cbegin(), rich_line.rich_segments().cend());
                }
            }

            assert(size >= rich_segments.size());

            return rich_line_type{std::move(rich_segments)};
        }

        template<typename MultiRichLine = multi_rich_line <rich_line_type>>
        [[nodiscard]] MultiRichLine get_multi_rich_line() const {
            using _rich_line_type = typename geometry::multi_rich_line_traits<MultiRichLine>::rich_line_type;
            using _rich_segment_type = typename geometry::multi_rich_line_traits<MultiRichLine>::rich_segment_type;
            std::vector<_rich_line_type> multi_rich_lines;

            std::size_t size = 0;
            auto it = _rich_lines.cbegin();
            auto start_it = it;
            while (it != _rich_lines.cend()) {
                const rich_line_type &rich_line = *it;
                if (not rich_line.rich_segments().empty()) {
                    if (it != _rich_lines.cbegin()) {
                        const rich_line_type &prev_rich_line = *std::prev(it);
                        const point_type &prev = prev_rich_line.rich_segments().back().second();
                        const point_type &next = rich_line.rich_segments().front().first();
                        if (not geometry::equals_points(prev, next)) {
                            // found gap, collect
                            std::vector<_rich_segment_type> rich_segments;
                            rich_segments.reserve(size);
                            for (auto copy_it = start_it; copy_it != it; copy_it++) {
                                const rich_line_type &copy_rich_line = *copy_it;
                                for (const auto &rich_segment: copy_rich_line.rich_segments()) {
                                    rich_segments.emplace_back(rich_segment);
                                }
                            }
                            multi_rich_lines.emplace_back(_rich_line_type{std::move(rich_segments)});
                            start_it = it;
                            size = 0;
                        }
                    }
                    size += rich_line.rich_segments().size();
                }
                it++;
            }

            std::vector<_rich_segment_type> rich_segments;
            rich_segments.reserve(size);
            for (auto copy_it = start_it; copy_it != it; copy_it++) {
                const rich_line_type &copy_rich_line = *copy_it;
                for (const auto &rich_segment: copy_rich_line.rich_segments()) {
                    rich_segments.emplace_back(rich_segment);
                }
            }

            assert(size >= rich_segments.size());

            multi_rich_lines.emplace_back(_rich_line_type{std::move(rich_segments)});

            return MultiRichLine{std::move(multi_rich_lines)};
        }

        [[nodiscard]] static route merge(const std::vector<std::reference_wrapper<route>> &routes,
                                         bool connect = false, bool non_connect_except = false) {
            // if no routes given, return invalid route
            if (routes.empty()) {
                return route{true};
            }

            // if all routes are invalid, return invalid route
            if (std::all_of(routes.cbegin(), routes.cend(), [](const route &route) {
                return route.is_invalid();
            })) {
                return route{true};
            }

            // calculate new size, might be a bit too large due to joints
            std::size_t new_references_size = 0;
            for (auto route_it = routes.cbegin(); route_it != routes.cend(); route_it++) {
                const route &route = *route_it;
                if (not route.is_invalid() and route.has_length()) {
                    new_references_size += route._rich_lines.size();
                }
            }

            // prepare new storages
            std::deque<rich_line_type> new_rich_lines;
            std::vector<std::reference_wrapper<const rich_line_type>> new_references;
            new_references.reserve(new_references_size);

            // merge routes
            for (route &next_route: routes) {
                if (not next_route.is_invalid() and next_route.has_length()) {
                    // test if routes are attached
                    if (not new_references.empty()) {
                        const rich_line_type &prev = new_references.back();
                        if (not prev.empty()) {
                            const rich_line_type &next = next_route._rich_lines.front();
                            if (not next.empty()) {
                                const auto &prev_point = prev.back();
                                const auto &next_point = next.front();
                                if (not geometry::equals_points(prev_point, next_point)) {
                                    if (not connect) {
                                        if (non_connect_except) {
                                            throw route_merge_exception{};
                                        }
                                    } else {
                                        // connect both routes with a new segment inbetween
                                        rich_line_type connection{line_type{prev_point, next_point}};
                                        const auto &_rich_line = new_rich_lines.emplace_back(std::move(connection));
                                        new_references.emplace_back(_rich_line);
                                    }
                                }
                            }
                        }
                    }

                    auto next_rich_it = next_route._own_rich_lines.begin();
                    for (auto next_it = next_route._rich_lines.begin();
                         next_it != next_route._rich_lines.end(); next_it++) {
                        const rich_line_type &next = *next_it;

                        // local data movement
                        bool data_moved = false;
                        if (next_rich_it != next_route._own_rich_lines.end()) {
                            rich_line_type &next_rich_line = *next_rich_it;
                            if (&next_rich_line == &next) {
                                // reference has local data, move it
                                const auto &_rich_line = new_rich_lines.emplace_back(std::move(next_rich_line));
                                new_references.emplace_back(_rich_line);
                                next_rich_it++;
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

            return route{std::move(new_rich_lines), std::move(new_references)};
        }

        static void join(route &a, route &b, bool retain_reversal = false) {
            if (a.is_invalid() or b.is_invalid() or a._rich_lines.empty() or b._rich_lines.empty()) {
                // cannot join invalid or empty routes
                return;
            }

            const rich_line_type &a_back_ref = a._rich_lines.back();
            const rich_line_type &b_front_ref = b._rich_lines.front();

            if (a_back_ref.rich_segments().empty() or b_front_ref.rich_segments().empty()) {
                // cannot join empty segments
                return;
            }

            if (retain_reversal) {
                const rich_segment_type &a_back_segment = a_back_ref.rich_segments().back();
                const rich_segment_type &b_front_segment = b_front_ref.rich_segments().front();
                if (a_back_segment.has_azimuth() and b_front_segment.has_azimuth()) {
                    const auto direction =
                            std::fabs(geometry::direction_deg(a_back_segment.azimuth(), b_front_segment.azimuth())) /
                            180.0;
                    if (direction > 0.99) {
                        // reversal detected, shall be retained, so stop join
                        return;
                    }
                }
            }

            using segment_type = typename rich_segment_traits<rich_segment_type>::segment_type;

            std::vector<rich_segment_type> a_back_segments{a_back_ref.rich_segments().cbegin(),
                                                           std::prev(a_back_ref.rich_segments().cend())};
            a_back_segments.emplace_back(rich_segment_type{segment_type{a_back_ref.rich_segments().back().first(),
                                                                        b_front_ref.rich_segments().front().second()}});
            rich_line_type a_back{std::move(a_back_segments)};
            std::vector<rich_segment_type> b_front_segments{std::next(b_front_ref.rich_segments().cbegin()),
                                                            b_front_ref.rich_segments().cend()};
            rich_line_type b_front{std::move(b_front_segments)};

            if (not a._own_rich_lines.empty() and &a_back_ref == &a._own_rich_lines.back()) {
                a._own_rich_lines.pop_back();
            }

            if (not b._own_rich_lines.empty() and &b_front_ref == &b._own_rich_lines.front()) {
                b._own_rich_lines.pop_front();
            }

            if (a_back.rich_segments().empty()) {
                a._rich_lines.pop_back();
            } else {
                const rich_line_type &new_a_back_ref = a._own_rich_lines.emplace_back(std::move(a_back));
                a._rich_lines.back() = std::cref(new_a_back_ref);
            }
            a._computed_length = false;
            a._computed_azimuth = false;
            a._computed_directions = false;

            if (b_front.rich_segments().empty()) {
                b._rich_lines.erase(b._rich_lines.begin());
            } else {
                const rich_line_type &new_b_front_ref = b._own_rich_lines.emplace_front(std::move(b_front));
                b._rich_lines.front() = std::cref(new_b_front_ref);
            }
            b._computed_length = false;
            b._computed_azimuth = false;
            b._computed_directions = false;
        }

        [[nodiscard]] std::string wkt() const {
            return get_multi_rich_line().wkt();
        }

        [[nodiscard]] std::vector<std::string> header() const {
            return {"geometry", "length"};
        }

        [[nodiscard]] std::vector<std::string> row() const {
            std::vector<std::string> row;
            row.reserve(2);
            row.emplace_back(wkt());
            if (has_length()) {
                row.emplace_back(std::to_string(length()));
            }
            return row;
        }

        [[nodiscard]] std::string str() const {
            const auto &fields = row();
            std::string str = "Route(";
            bool first = true;
            for (const auto &field: fields) {
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

    private:
        mutable bool _computed_length, _computed_azimuth, _computed_directions;
        mutable bool _is_invalid, _is_connected, _has_length, _has_azimuth, _has_directions;
        mutable length_type _length;
        mutable angle_type _azimuth, _directions, _absolute_directions;
        std::deque<rich_line_type> _own_rich_lines;
        std::vector<std::reference_wrapper<const rich_line_type>> _rich_lines;

        void _transfer(const route &other) {
            _is_invalid = other._is_invalid;
            _computed_length = other._computed_length;
            _computed_azimuth = other._computed_azimuth;
            _computed_directions = other._computed_directions;
            if (other._computed_length) {
                _has_length = other._has_length;
                if (other._has_length) {
                    _length = other._length;
                }
            }
            if (other._computed_azimuth) {
                _has_azimuth = other._has_azimuth;
                if (other._has_azimuth) {
                    _azimuth = other._azimuth;
                }
            }
            if (other._computed_directions) {
                _is_connected = other._is_connected;
                _has_directions = other._has_directions;
                if (other._has_directions) {
                    _directions = other._directions;
                    _absolute_directions = other._absolute_directions;
                }
            }
        }

        void _copy(const route &other) {
            _transfer(other);

            _own_rich_lines.clear();
            _rich_lines.clear();

            _rich_lines.reserve(other._rich_lines.size());

            auto rich_it = other._own_rich_lines.cbegin();
            for (auto reference_it = other._rich_lines.cbegin();
                 reference_it != other._rich_lines.cend(); reference_it++) {
                const rich_line_type &reference = *reference_it;
                if (rich_it != other._own_rich_lines.cend()) {
                    const rich_line_type &rich_line = *rich_it;
                    if (&rich_line == &reference) {
                        // reference has local data
                        const auto &_rich_line = _own_rich_lines.emplace_back(rich_line);
                        _rich_lines.emplace_back(_rich_line);
                        rich_it++;
                        continue;
                    }
                }

                _rich_lines.emplace_back(reference);
            }
        }

        void _move(route &&other) noexcept {
            _transfer(other);

            _own_rich_lines.clear();
            _rich_lines.clear();

            _rich_lines.reserve(other._rich_lines.size());

            auto rich_it = other._own_rich_lines.begin();
            for (auto reference_it = other._rich_lines.begin();
                 reference_it != other._rich_lines.end(); reference_it++) {
                if (rich_it != other._own_rich_lines.end()) {
                    const rich_line_type &reference = *reference_it;
                    rich_line_type &rich_line = *rich_it;
                    if (&rich_line == &reference) {
                        // reference has local data
                        const auto &_rich_line = _own_rich_lines.emplace_back(std::move(rich_line));
                        _rich_lines.emplace_back(_rich_line);
                        rich_it++;
                        continue;
                    }
                }

                _rich_lines.emplace_back(std::move(*reference_it));
            }

            decltype(other._own_rich_lines) _other_rich_lines;
            other._own_rich_lines.swap(_other_rich_lines);
            decltype(other._rich_lines) other_rich_lines;
            other._rich_lines.swap(other_rich_lines);
        }

        void _check_references() const {
            // check references being correct
            std::size_t found = 0;
            auto reference_it = _rich_lines.cbegin();
            for (auto rich_it = _own_rich_lines.cbegin(); rich_it != _own_rich_lines.cend(); rich_it++) {
                const rich_line_type &rich_line = *rich_it;
                while (reference_it != _rich_lines.cend()) {
                    const rich_line_type &reference = *reference_it++;
                    if (&reference == &rich_line) {
                        found++;
                        break;
                    }
                }
            }

            if (found != _own_rich_lines.size()) {
                throw std::logic_error("route references are incorrect");
            }
        }

        void _compute_length() const {
            if (not _computed_length) {
                _has_length = false;
                if (_rich_lines.empty()) {
                    return;
                } else if (_rich_lines.size() == 1) {
                    const rich_line_type &rich_line = _rich_lines.front();
                    if (rich_line.has_length()) {
                        _length = rich_line.length();
                        _has_length = true;
                    }
                } else {
                    _length = default_float_type<length_type>::v0;
                    for (std::size_t i = 0, j = 1; j < _rich_lines.size(); ++i, ++j) {
                        const rich_line_type &curr_rich_line = _rich_lines[i];
                        const rich_line_type &next_rich_line = _rich_lines[j];

                        // add current length
                        if (curr_rich_line.has_length()) {
                            _length += curr_rich_line.length();
                            _has_length = true;
                        }
                        // in last loop, also add next length
                        if (j + 1 >= _rich_lines.size() and next_rich_line.has_length()) {
                            _length += next_rich_line.length();
                            _has_length = true;
                        }
                    }
                }
                _computed_length = true;
            }
        }

        void _compute_azimuth() const {
            if (not _computed_azimuth) {
                _has_azimuth = false;
                if (_rich_lines.empty()) {
                    return;
                } else if (_rich_lines.size() == 1) {
                    const rich_line_type &rich_line = _rich_lines.front();
                    if (rich_line.has_azimuth()) {
                        _azimuth = rich_line.azimuth();
                        _has_azimuth = true;
                    }
                } else {
                    for (auto first_it = _rich_lines.cbegin(); first_it != _rich_lines.cend(); first_it++) {
                        const rich_line_type &first_rich_line = *first_it;

                        if (not first_rich_line.empty()) {
                            for (auto last_it = _rich_lines.crbegin(); last_it != _rich_lines.crend(); last_it++) {
                                const rich_line_type &last_rich_line = *last_it;

                                if (not last_rich_line.empty()) {
                                    const auto &first_point = first_rich_line.front();
                                    const auto &last_point = last_rich_line.back();

                                    if (not geometry::equals_points(first_point, last_point)) {
                                        _azimuth = geometry::azimuth_deg(first_point, last_point);
                                        _has_azimuth = true;
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                _computed_azimuth = true;
            }
        }

        void _compute_directions() const {
            if (not _computed_directions) {
                _is_connected = true;
                _has_directions = false;
                if (_rich_lines.empty()) {
                    return;
                } else if (_rich_lines.size() == 1) {
                    const rich_line_type &rich_line = _rich_lines.front();
                    if (rich_line.has_directions()) {
                        _directions = rich_line.directions();
                        _absolute_directions = rich_line.absolute_directions();
                        _has_directions = true;
                    }
                } else {
                    _directions = default_float_type<angle_type>::v0;
                    _absolute_directions = default_float_type<angle_type>::v0;

                    for (std::size_t i = 0, j = 1; j < _rich_lines.size(); ++i, ++j) {
                        const rich_line_type &curr_rich_line = _rich_lines[i];
                        const rich_line_type &next_rich_line = _rich_lines[j];
                        if (not curr_rich_line.empty() and not next_rich_line.empty()) {
                            const auto &prev_point = curr_rich_line.back();
                            const auto &next_point = next_rich_line.front();
                            // check if rich lines are attached
                            if (geometry::equals_points(prev_point, next_point)) {
                                // add direction inbetween only when attached
                                if (curr_rich_line.has_azimuth() and next_rich_line.has_azimuth()) {
                                    const auto direction = geometry::direction_deg(
                                            curr_rich_line.rich_segments().back().azimuth(),
                                            next_rich_line.rich_segments().front().azimuth());
                                    _directions += direction;
                                    _absolute_directions += std::fabs(direction);
                                    _has_directions = true;
                                }
                            } else {
                                _is_connected = false;
                            }
                        }

                        // add remaining directions
                        if (curr_rich_line.has_directions()) {
                            _directions += curr_rich_line.directions();
                            _absolute_directions += curr_rich_line.absolute_directions();
                            _has_directions = true;
                        }
                        // in last loop, also add next directions
                        if (j + 1 >= _rich_lines.size() and next_rich_line.has_directions()) {
                            _directions += next_rich_line.directions();
                            _absolute_directions += next_rich_line.absolute_directions();
                            _has_directions = true;
                        }
                    }
                }
                _computed_directions = true;
            }
        }

    };

    template<typename Point>
    using route_type = route<rich_referring_line_type<Point>>;

    template<typename Point>
    using eager_route_type = route<eager_rich_referring_line_type<Point>>;

    template<typename Point>
    using lazy_route_type = route<lazy_rich_referring_line_type<Point>>;

}

#endif //MAP_MATCHING_2_ROUTE_HPP
