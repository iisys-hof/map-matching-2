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

#ifndef MAP_MATCHING_2_MULTI_RICH_LINE_HPP
#define MAP_MATCHING_2_MULTI_RICH_LINE_HPP

#include "rich_line.hpp"

namespace map_matching_2::geometry {

    template<typename Line>
    struct multi_rich_line {

        using point_type = typename boost::geometry::point_type<Line>::type;
        using line_type = Line;
        using multi_line_type = boost::geometry::model::multi_linestring<Line>;
        using segment_type = boost::geometry::model::segment<point_type>;
        using rich_segment_type = rich_segment<segment_type>;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using length_type = typename boost::geometry::default_length_result<Line>::type;
        using angle_type = typename boost::geometry::coordinate_type<Line>::type;
        using rich_line_type = rich_line<Line>;

        bool has_length;
        bool has_azimuth;
        bool has_directions;
        length_type length;
        angle_type azimuth;
        angle_type directions;
        angle_type absolute_directions;
        multi_line_type multi_line;
        std::vector<rich_line_type> rich_lines;

        multi_rich_line();

        multi_rich_line(multi_line_type multi_line);

        multi_rich_line(rich_line_type rich_line);

        multi_rich_line(std::vector<line_type> lines);

        multi_rich_line(std::vector<rich_line_type> rich_lines);

        multi_rich_line(multi_line_type multi_line, std::vector<rich_line_type> rich_lines);

        ~multi_rich_line() = default;

        multi_rich_line(const multi_rich_line &other) = default;

        multi_rich_line(multi_rich_line &&other) noexcept = default;

        multi_rich_line &operator=(const multi_rich_line &other) = default;

        multi_rich_line &operator=(multi_rich_line &&other) noexcept = default;

        void _complete();

        [[nodiscard]] std::size_t sizes() const;

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3);

        template<typename Network>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr);

        [[nodiscard]] static multi_rich_line
        merge(const std::vector<std::reference_wrapper<rich_line_type>> &rich_lines);

        [[nodiscard]] static multi_line_type lines2multi_lines(const std::vector<line_type> &lines);

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::string str() const;

        template<typename LineT>
        friend bool operator==(const multi_rich_line<LineT> &left, const multi_rich_line<LineT> &right);

        template<typename LineT>
        friend bool operator!=(const multi_rich_line<LineT> &left, const multi_rich_line<LineT> &right);

        template<typename LineT>
        friend std::ostream &operator<<(std::ostream &out, const multi_rich_line<LineT> &multi_rich_line);

    };

}

#endif //MAP_MATCHING_2_MULTI_RICH_LINE_HPP
