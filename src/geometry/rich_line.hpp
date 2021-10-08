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

#ifndef MAP_MATCHING_2_RICH_LINE_HPP
#define MAP_MATCHING_2_RICH_LINE_HPP

#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "rich_segment.hpp"

namespace map_matching_2::geometry {

    struct rich_line_merge_exception : public std::exception {
    };

    template<typename Line>
    struct rich_line {

        using point_type = typename boost::geometry::point_type<Line>::type;
        using line_type = Line;
        using segment_type = boost::geometry::model::segment<point_type>;
        using rich_segment_type = rich_segment<segment_type>;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using length_type = typename boost::geometry::default_length_result<Line>::type;
        using angle_type = typename boost::geometry::coordinate_type<Line>::type;

        bool has_length;
        bool has_azimuth;
        bool has_directions;
        length_type length;
        angle_type azimuth;
        angle_type directions;
        angle_type absolute_directions;
        std::vector<angle_type> segments_directions;
        std::vector<angle_type> segments_absolute_directions;
        Line line;
        std::vector<rich_segment_type> rich_segments;

        rich_line();

        rich_line(Line line);

        rich_line(std::vector<rich_segment_type> rich_segments);

        rich_line(Line line, std::vector<rich_segment_type> rich_segments);

        ~rich_line() = default;

        rich_line(const rich_line &other) = default;

        rich_line(rich_line &&other) noexcept = default;

        rich_line &operator=(const rich_line &other) = default;

        rich_line &operator=(rich_line &&other) noexcept = default;

        void _complete();

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3);

        template<typename Network>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr);

        [[nodiscard]] static rich_line merge(const std::vector<std::reference_wrapper<rich_line>> &rich_lines);

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::string str() const;

        template<typename LineT>
        friend bool operator==(const rich_line<LineT> &left, const rich_line<LineT> &right);

        template<typename LineT>
        friend bool operator!=(const rich_line<LineT> &left, const rich_line<LineT> &right);

        template<typename LineT>
        friend std::ostream &operator<<(std::ostream &out, const rich_line<LineT> &rich_line);

    };

}

#endif //MAP_MATCHING_2_RICH_LINE_HPP
