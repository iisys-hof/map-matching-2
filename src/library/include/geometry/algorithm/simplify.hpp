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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_SIMPLIFY_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_SIMPLIFY_HPP

#include <vector>
#include <list>

#include "geometry/common.hpp"

#include "geometry/rich_type/concepts.hpp"
#include "geometry/rich_type/traits/rich_segment.hpp"
#include "geometry/rich_type/traits/rich_line.hpp"

namespace map_matching_2::geometry {

    template<typename Line>
    [[nodiscard]] std::vector<std::size_t> simplify_indices(
            const Line &line, std::size_t from, std::size_t to, double tolerance) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));

        std::vector<std::size_t> indices{};

        if (from >= to or to - from < 2 or from >= line.size() or to > line.size()) {
            return indices;
        }

        using line_type = Line;
        using point_type = point_type_t<line_type>;
        using referring_segment_type = typename models<point_type>::referring_segment_type;
        using distance_type = typename data<point_type>::distance_type;

        std::size_t max{};
        distance_type max_distance = geometry::default_float_type<distance_type>::v0;

        referring_segment_type segment{line[from], line[to - 1]};

        for (std::size_t i = from + 1; i < to - 1; ++i) {
            const point_type &point = line[i];
            const distance_type distance = geometry::distance(point, segment);

            if (distance > max_distance) {
                max_distance = distance;
                max = i;
            }
        }

        if (max_distance > tolerance) {
            indices = simplify_indices(line, from, max + 1, tolerance);
            auto _indices = simplify_indices(line, max, to, tolerance);
            indices.reserve(indices.size() + _indices.size());
            std::move(std::begin(_indices), std::end(_indices), std::back_inserter(indices));
        } else {
            indices.reserve((to - 1) - (from + 1));
            for (std::size_t i = from + 1; i < to - 1; ++i) {
                indices.emplace_back(i);
            }
        }

        return indices;
    }

    template<typename Line>
    void simplify(Line &line, std::size_t from, std::size_t to, double tolerance) {
        auto indices = simplify_indices(line, from, to, tolerance);
        if (not indices.empty()) {
            for (std::size_t i = indices.size(); i-- > 0;) {
                std::size_t index = indices[i];
                if (index < line.size()) {
                    line.erase(std::next(line.begin(), index));
                }
            }
        }
    }

    template<is_rich_line RichLine>
    [[nodiscard]] std::vector<std::size_t> simplify_rich_line_indices(
            RichLine &rich_line,
            bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
        using rich_line_type = RichLine;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;

        std::vector<std::size_t> indices{};

        if (not retain_reversals) {
            indices = simplify_indices<line_type>(rich_line.line(), 0, rich_line.size(), tolerance);
        } else {
            indices.reserve(rich_line.size());
            std::size_t index = 0;
            if (rich_line.size() > 2) {
                for (std::size_t i = 0; i + 2 < rich_line.size(); i++) {
                    const auto _segment = rich_line.rich_referring_segment(i);
                    const auto _next_segment = rich_line.rich_referring_segment(i + 1);
                    if (_segment.has_azimuth() and _next_segment.has_azimuth()) {
                        const auto direction = std::fabs(
                                geometry::direction_deg(_segment.azimuth(), _next_segment.azimuth()));
                        if (direction > (180.0 - reverse_tolerance)) {
                            auto _indices = simplify_indices<line_type>(rich_line.line(), index, i + 2, tolerance);
                            std::move(std::begin(_indices), std::end(_indices), std::back_inserter(indices));
                            index = i + 1;
                        }
                    }
                }
            }
            auto _indices = simplify_indices<line_type>(rich_line.line(), index, rich_line.size(), tolerance);
            std::move(std::begin(_indices), std::end(_indices), std::back_inserter(indices));
        }

        return indices;
    }

    template<is_rich_line RichLine>
    void simplify_rich_line(RichLine &rich_line,
            bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
        auto indices = simplify_rich_line_indices(rich_line, retain_reversals, tolerance, reverse_tolerance);
        rich_line.erase(indices);
    }

    template<is_rich_segment RichSegment>
    void _simplify_rich_segments(
            std::pmr::list<RichSegment> &segments,
            typename std::pmr::list<RichSegment>::iterator begin,
            typename std::pmr::list<RichSegment>::iterator end,
            double tolerance) {
        if (std::distance(begin, end) < 2) {
            return;
        }

        using rich_segment_type = RichSegment;
        using segment_type = typename rich_segment_traits<rich_segment_type>::segment_type;
        using distance_type = typename rich_segment_traits<rich_segment_type>::distance_type;

        typename std::pmr::list<rich_segment_type>::iterator max_it;
        distance_type max_distance = geometry::default_float_type<distance_type>::v0;

        rich_segment_type complete_segment{segment_type{(*begin).first(), (*std::prev(end)).second()}};

        for (auto it = std::next(begin); it != end; it++) {
            const rich_segment_type &segment = *it;
            const auto distance = geometry::distance(segment.segment().first, complete_segment.segment());

            if (distance > max_distance) {
                max_distance = distance;
                max_it = it;
            }
        }

        if (max_distance > tolerance) {
            _simplify_rich_segments<rich_segment_type>(segments, begin, max_it, tolerance);
            _simplify_rich_segments<rich_segment_type>(segments, max_it, end, tolerance);
        } else {
            auto last_it = segments.erase(begin, end);
            segments.insert(last_it, complete_segment);
        }
    }

    template<is_rich_segment RichSegment>
    void simplify_rich_segments(
            std::pmr::list<RichSegment> &segments,
            bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
        using rich_segment_type = RichSegment;

        if (not retain_reversals) {
            _simplify_rich_segments<rich_segment_type>(segments, segments.begin(), segments.end(), tolerance);
        } else {
            auto index = segments.begin();
            if (segments.size() > 1) {
                auto it = segments.begin();
                while (std::next(it) != segments.end()) {
                    auto next_it = std::next(it);
                    const auto &_segment = *it;
                    const auto &_next_segment = *next_it;
                    if (_segment.has_azimuth() and _next_segment.has_azimuth()) {
                        const auto direction =
                                std::fabs(geometry::direction_deg(_segment.azimuth(), _next_segment.azimuth()));
                        if (direction > (180.0 - reverse_tolerance)) {
                            // reverse movement detected
                            const auto position = std::distance(segments.begin(), next_it);
                            const auto size = segments.size();
                            _simplify_rich_segments<rich_segment_type>(segments, index, next_it, tolerance);
                            const auto diff = size - segments.size();
                            it = std::next(segments.begin(), position - diff);
                            index = it;
                        } else {
                            ++it;
                        }
                    } else {
                        ++it;
                    }
                }
            }
            // remaining
            _simplify_rich_segments<rich_segment_type>(segments, index, segments.end(), tolerance);
        }
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_SIMPLIFY_HPP
