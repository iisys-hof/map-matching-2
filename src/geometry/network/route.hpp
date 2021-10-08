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

    template<typename Line>
    struct route {

        using line_type = Line;
        using rich_line_type = rich_line<Line>;
        using multi_rich_line_type = multi_rich_line<Line>;
        using point_type = typename boost::geometry::point_type<line_type>::type;
        using segment_type = boost::geometry::model::segment<point_type>;
        using rich_segment_type = typename rich_line_type::rich_segment_type;
        using length_type = typename boost::geometry::default_length_result<line_type>::type;
        using angle_type = typename boost::geometry::coordinate_type<line_type>::type;

        bool is_invalid;
        bool is_connected;
        bool has_length;
        bool has_azimuth;
        bool has_directions;
        length_type length;
        angle_type azimuth;
        angle_type directions;
        angle_type absolute_directions;
        std::deque<rich_line_type> _rich_lines;
        std::vector<std::reference_wrapper<const rich_line_type>> rich_lines;

        route();

        route(bool is_invalid);

        route(line_type line);

        route(rich_line_type rich_line);

        route(std::deque<rich_line_type> rich_lines);

        route(std::reference_wrapper<const rich_line_type> rich_line_reference);

        route(std::vector<std::reference_wrapper<const rich_line_type>> rich_lines_references);

        route(std::deque<rich_line_type> rich_lines,
              std::vector<std::reference_wrapper<const rich_line_type>> rich_lines_references);

        ~route() = default;

        route(const route &other);

        route(route &&other);

        route &operator=(const route &other);

        route &operator=(route &&other);

        void _copy(const route &other);

        void _move(route &&other);

        void _complete();

        [[nodiscard]] bool attaches(const route &other) const;

        [[nodiscard]] rich_line_type extract_return_line(const route &other) const;

        [[nodiscard]] Line get_line() const;

        [[nodiscard]] rich_line_type get_rich_line() const;

        [[nodiscard]] multi_rich_line_type get_multi_rich_line() const;

        [[nodiscard]] std::vector<std::reference_wrapper<const edge <Line>>> get_edges() const;

        [[nodiscard]] static route merge(const std::vector<std::reference_wrapper<route>> &routes,
                                         bool connect = false, bool non_connect_except = false);

        static void join(route &a, route &b, bool retain_reversal = false);

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row() const;

        [[nodiscard]] std::string str() const;

        template<typename LineT>
        friend std::ostream &operator<<(std::ostream &out, const route<LineT> &route);

    };

}

#endif //MAP_MATCHING_2_ROUTE_HPP
