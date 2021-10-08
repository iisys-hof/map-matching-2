// Copyright (C) 2021-2021 Adrian Wöltche
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

#include "rich_segment.hpp"

#include "util.hpp"
#include "types.hpp"

namespace map_matching_2::geometry {

    template<typename Segment>
    rich_segment<Segment>::rich_segment(Segment segment) : segment{std::move(segment)} {
        _complete();
    }

    template<typename Segment>
    rich_segment<Segment>::rich_segment(point_type first, point_type second)
            : rich_segment{Segment{std::move(first), std::move(second)}} {}

    template<typename Segment>
    rich_segment<Segment>::rich_segment(Segment segment, length_type length, angle_type azimuth)
            : segment{std::move(segment)}, length{length}, azimuth{azimuth} {
        this->has_length = false;
        this->has_azimuth = false;
        if (not geometry::equals_points(this->segment.first, this->segment.second)) {
            this->has_length = true;
            this->has_azimuth = true;
        }
    }

    template<typename Segment>
    void rich_segment<Segment>::_complete() {
        this->has_length = false;
        this->has_azimuth = false;
        if (not geometry::equals_points(this->segment.first, this->segment.second)) {
            this->length = geometry::length_segment(this->segment);
            this->has_length = true;
            this->azimuth = geometry::azimuth_segment_deg(this->segment);
            this->has_azimuth = true;
        }
    }

    template<typename Segment>
    template<template<typename> class Container>
    Container<rich_segment<Segment>>
    rich_segment<Segment>::line2rich_segments(const line_type &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        Container <rich_segment> rich_segments;
        if (line.size() >= 2) {
            if constexpr(std::is_same_v<Container < rich_segment>, std::vector<rich_segment> >) {
                rich_segments.reserve(line.size() - 1);
            }
            for (std::size_t i = 0; i < line.size() - 1; ++i) {
                rich_segments.emplace_back(rich_segment{line[i], line[i + 1]});
            }
        }
        return rich_segments;
    }

    template<typename Segment>
    template<template<typename> class Container>
    typename rich_segment<Segment>::line_type
    rich_segment<Segment>::rich_segments2line(const Container<rich_segment> &rich_segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        line_type line;
        line.reserve(rich_segments.size() + 1);
        for (const auto &rich_segment: rich_segments) {
            const auto &segment = rich_segment.segment;
            if (line.empty()) {
                line.emplace_back(segment.first);
            }
            line.emplace_back(segment.second);
        }
        return line;
    }

    template<typename Segment>
    template<template<typename> class Container>
    Container<typename rich_segment<Segment>::angle_type>
    rich_segment<Segment>::rich_segments_directions_deg(const Container<rich_segment> &rich_segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        Container <angle_type> directions;
        if constexpr(std::is_same_v<Container < angle_type>, std::vector<angle_type>>) {
            directions.reserve(rich_segments.size() - 1);
        }
        for (auto it = rich_segments.cbegin(); std::next(it) != rich_segments.cend(); ++it) {
            const auto &a = *it;
            const auto &b = *std::next(it);
            angle_type direction = geometry::default_float_type<angle_type>::v0;
            if (geometry::equals_points(a.segment.second, b.segment.first)) {
                if (a.has_azimuth and b.has_azimuth) {
                    direction = direction_deg(a.azimuth, b.azimuth);
                } else if (a.has_azimuth and not b.has_azimuth) {
                    direction = direction_deg(a.azimuth, direction);
                } else if (not a.has_azimuth and b.has_azimuth) {
                    direction = direction_deg(direction, b.azimuth);
                }
            }
            directions.emplace_back(direction);
        }
        return directions;
    }

    template<typename Segment>
    void rich_segment<Segment>::simplify(std::list<rich_segment> &segments, const bool retain_reversals,
                                         const double tolerance, const double reverse_tolerance) {
        if (not retain_reversals) {
            _simplify(segments, segments.begin(), segments.end(), tolerance);
        } else {
            auto index = segments.begin();
            for (auto it = segments.begin(); it != segments.end(); it++) {
                if (std::next(it) != segments.end()) {
                    auto next_it = std::next(it);
                    const auto &segment = *it;
                    const auto &next_segment = *next_it;
                    if (segment.has_azimuth and next_segment.has_azimuth) {
                        const auto direction =
                                std::fabs(geometry::direction_deg(segment.azimuth, next_segment.azimuth));
                        if (direction > (180.0 - reverse_tolerance)) {
                            // reverse movement detected
                            const auto position = std::distance(segments.begin(), next_it);
                            const auto size = segments.size();
                            _simplify(segments, index, next_it, tolerance);
                            const auto diff = size - segments.size();
                            it = std::next(segments.begin(), position - diff);
                            index = it;
                        }
                    }
                }
            }
            // remaining
            _simplify(segments, index, segments.end(), tolerance);
        }
    }

    template<typename Segment>
    void rich_segment<Segment>::_simplify(std::list<rich_segment> &segments,
                                          typename std::list<rich_segment>::iterator begin,
                                          typename std::list<rich_segment>::iterator end,
                                          const double tolerance) {
        if (std::distance(begin, end) < 2) {
            return;
        }

        typename std::list<rich_segment>::iterator max_it;
        distance_type max_distance = geometry::default_float_type<distance_type>::v0;

        rich_segment complete_segment{(*begin).segment.first, (*std::prev(end)).segment.second};
//        std::cout << complete_segment.wkt() << std::endl;

        for (auto it = std::next(begin); it != end; it++) {
            const rich_segment &segment = *it;
            const auto distance = geometry::distance(segment.segment.first, complete_segment.segment);

            if (distance > max_distance) {
                max_distance = distance;
                max_it = it;
            }
        }

        if (max_distance > tolerance) {
            _simplify(segments, begin, max_it, tolerance);
            _simplify(segments, max_it, end, tolerance);
        } else {
            auto last_it = segments.erase(begin, end);
            segments.insert(last_it, complete_segment);
        }
    }

    template<typename Segment>
    typename rich_segment<Segment>::line_type rich_segment<Segment>::line() const {
        return line_type{segment.first, segment.second};
    }

    template<typename Segment>
    std::string rich_segment<Segment>::wkt() const {
        return geometry::to_wkt(segment);
    }

    template<typename Segment>
    std::string rich_segment<Segment>::str() const {
        std::string str = "RichSegment(";
        str.append(wkt());
        str.append(",");
        str.append(std::to_string(length));
        str.append(",");
        str.append(std::to_string(azimuth));
        str.append(")");
        return str;
    }

    template<typename Segment>
    bool operator==(const rich_segment<Segment> &left, const rich_segment<Segment> &right) {
        return geometry::equals_points(left.segment.first, right.segment.first) and
               geometry::equals_points(left.segment.second, right.segment.second);
    }

    template<typename Segment>
    bool operator!=(const rich_segment<Segment> &left, const rich_segment<Segment> &right) {
        return not(left == right);
    }

    template<typename Segment>
    std::ostream &operator<<(std::ostream &out, const rich_segment<Segment> &rich_line) {
        out << rich_line.str();
        return out;
    }

    template
    class rich_segment<geometry::types_geographic::segment_type>;

    template
    class rich_segment<geometry::types_cartesian::segment_type>;

    template
    std::vector<rich_segment<geometry::types_geographic::segment_type>>
    rich_segment<geometry::types_geographic::segment_type>::line2rich_segments(const line_type &line);

    template
    std::vector<rich_segment<geometry::types_cartesian::segment_type>>
    rich_segment<geometry::types_cartesian::segment_type>::line2rich_segments(const line_type &line);

    template
    std::list<rich_segment<geometry::types_geographic::segment_type>>
    rich_segment<geometry::types_geographic::segment_type>::line2rich_segments(const line_type &line);

    template
    std::list<rich_segment<geometry::types_cartesian::segment_type>>
    rich_segment<geometry::types_cartesian::segment_type>::line2rich_segments(const line_type &line);

    template
    typename rich_segment<geometry::types_geographic::segment_type>::line_type
    rich_segment<geometry::types_geographic::segment_type>::rich_segments2line(
            const std::vector<rich_segment> &rich_segments);

    template
    typename rich_segment<geometry::types_cartesian::segment_type>::line_type
    rich_segment<geometry::types_cartesian::segment_type>::rich_segments2line(
            const std::vector<rich_segment> &rich_segments);

    template
    typename rich_segment<geometry::types_geographic::segment_type>::line_type
    rich_segment<geometry::types_geographic::segment_type>::rich_segments2line(
            const std::list<rich_segment> &rich_segments);

    template
    typename rich_segment<geometry::types_cartesian::segment_type>::line_type
    rich_segment<geometry::types_cartesian::segment_type>::rich_segments2line(
            const std::list<rich_segment> &rich_segments);

    template
    std::vector<typename rich_segment<geometry::types_geographic::segment_type>::angle_type>
    rich_segment<geometry::types_geographic::segment_type>::rich_segments_directions_deg(
            const std::vector<rich_segment> &rich_segments);

    template
    std::vector<typename rich_segment<geometry::types_cartesian::segment_type>::angle_type>
    rich_segment<geometry::types_cartesian::segment_type>::rich_segments_directions_deg(
            const std::vector<rich_segment> &rich_segments);

    template
    std::list<typename rich_segment<geometry::types_geographic::segment_type>::angle_type>
    rich_segment<geometry::types_geographic::segment_type>::rich_segments_directions_deg(
            const std::list<rich_segment> &rich_segments);

    template
    std::list<typename rich_segment<geometry::types_cartesian::segment_type>::angle_type>
    rich_segment<geometry::types_cartesian::segment_type>::rich_segments_directions_deg(
            const std::list<rich_segment> &rich_segments);

    template
    bool operator==(const rich_segment<geometry::types_geographic::segment_type> &left,
                    const rich_segment<geometry::types_geographic::segment_type> &right);

    template
    bool operator==(const rich_segment<geometry::types_cartesian::segment_type> &left,
                    const rich_segment<geometry::types_cartesian::segment_type> &right);

    template
    bool operator!=(const rich_segment<geometry::types_geographic::segment_type> &left,
                    const rich_segment<geometry::types_geographic::segment_type> &right);

    template
    bool operator!=(const rich_segment<geometry::types_cartesian::segment_type> &left,
                    const rich_segment<geometry::types_cartesian::segment_type> &right);

    template
    std::ostream &
    operator<<(std::ostream &out, const rich_segment<geometry::types_geographic::segment_type> &rich_segment);

    template
    std::ostream &
    operator<<(std::ostream &out, const rich_segment <geometry::types_cartesian::segment_type> &rich_segment);

}
