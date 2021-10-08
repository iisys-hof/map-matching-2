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

#ifndef MAP_MATCHING_2_RICH_SEGMENT_HPP
#define MAP_MATCHING_2_RICH_SEGMENT_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace map_matching_2::geometry {

    template<typename Segment>
    struct rich_segment {

        using point_type = typename boost::geometry::point_type<Segment>::type;
        using line_type = boost::geometry::model::linestring<point_type>;
        using segment_type = boost::geometry::model::segment<point_type>;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using length_type = typename boost::geometry::default_length_result<Segment>::type;
        using angle_type = typename boost::geometry::coordinate_type<Segment>::type;

        bool has_length;
        bool has_azimuth;
        length_type length;
        angle_type azimuth;
        segment_type segment;

        rich_segment(Segment segment);

        rich_segment(point_type first, point_type second);

        rich_segment(Segment segment, length_type length, angle_type azimuth);

        ~rich_segment() = default;

        rich_segment(const rich_segment &other) = default;

        rich_segment(rich_segment &&other) noexcept = default;

        rich_segment &operator=(const rich_segment &other) = default;

        rich_segment &operator=(rich_segment &&other) noexcept = default;

        void _complete();

        template<template<typename> class Container>
        [[nodiscard]] static Container<rich_segment>
        line2rich_segments(const line_type &line);

        template<template<typename> class Container>
        [[nodiscard]] static line_type
        rich_segments2line(const Container<rich_segment> &rich_segments);

        template<template<typename> class Container>
        [[nodiscard]] static Container<angle_type>
        rich_segments_directions_deg(const Container<rich_segment> &rich_segments);

        static void simplify(std::list<rich_segment> &segments, bool retain_reversals = true, double tolerance = 1e-3,
                             double reverse_tolerance = 1e-3);

        static void _simplify(std::list<rich_segment> &segments, typename std::list<rich_segment>::iterator begin,
                              typename std::list<rich_segment>::iterator end, double tolerance);

        [[nodiscard]] line_type line() const;

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::string str() const;

        template<typename SegmentT>
        friend bool operator==(const rich_segment<SegmentT> &left, const rich_segment<SegmentT> &right);

        template<typename SegmentT>
        friend bool operator!=(const rich_segment<SegmentT> &left, const rich_segment<SegmentT> &right);

        template<typename SegmentT>
        friend std::ostream &operator<<(std::ostream &out, const rich_segment<SegmentT> &rich_segment);

    };

}

#endif //MAP_MATCHING_2_RICH_SEGMENT_HPP
