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
    class route {

    public:
        using line_type = Line;
        using rich_line_type = rich_line<Line>;
        using multi_rich_line_type = multi_rich_line<Line>;
        using point_type = typename boost::geometry::point_type<line_type>::type;
        using segment_type = boost::geometry::model::segment<point_type>;
        using rich_segment_type = typename rich_line_type::rich_segment_type;
        using length_type = typename boost::geometry::default_length_result<line_type>::type;
        using angle_type = typename boost::geometry::coordinate_type<line_type>::type;

        route();

        explicit route(bool is_invalid);

        explicit route(line_type line);

        explicit route(rich_line_type rich_line);

        explicit route(std::deque<rich_line_type> rich_lines);

        explicit route(std::reference_wrapper<const rich_line_type> rich_line_reference);

        explicit route(std::vector<std::reference_wrapper<const rich_line_type>> rich_lines_references);

        route(std::deque<rich_line_type> rich_lines,
              std::vector<std::reference_wrapper<const rich_line_type>> rich_lines_references);

        ~route() = default;

        route(const route &other);

        route(route &&other) noexcept;

        route &operator=(const route &other);

        route &operator=(route &&other) noexcept;

        [[nodiscard]] bool is_invalid() const;

        [[nodiscard]] bool is_connected() const;

        [[nodiscard]] bool has_length() const;

        [[nodiscard]] length_type length() const;

        [[nodiscard]] bool has_azimuth() const;

        [[nodiscard]] angle_type azimuth() const;

        [[nodiscard]] bool has_directions() const;

        [[nodiscard]] angle_type directions() const;

        [[nodiscard]] angle_type absolute_directions() const;

        [[nodiscard]] const std::vector<std::reference_wrapper<const rich_line_type>> &rich_lines() const;

        [[nodiscard]] bool attaches(const route &other) const;

        [[nodiscard]] std::array<std::int64_t, 8> find_return_line(const route &other) const;

        [[nodiscard]] rich_line_type extract_return_line(const route &other, bool from_self = true) const;

        [[nodiscard]] route trim_merge(const route &other) const;

        [[nodiscard]] line_type get_line() const;

        [[nodiscard]] rich_line_type get_rich_line() const;

        [[nodiscard]] multi_rich_line_type get_multi_rich_line() const;

        [[nodiscard]] static route merge(const std::vector<std::reference_wrapper<route>> &routes,
                                         bool connect = false, bool non_connect_except = false);

        static void join(route &a, route &b, bool retain_reversal = false);

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row() const;

        [[nodiscard]] std::string str() const;

        template<typename LineT>
        friend std::ostream &operator<<(std::ostream &out, const route<LineT> &route);

    private:
        mutable bool _computed_length, _computed_azimuth, _computed_directions;
        mutable bool _is_invalid, _is_connected, _has_length, _has_azimuth, _has_directions;
        mutable length_type _length;
        mutable angle_type _azimuth, _directions, _absolute_directions;
        std::deque<rich_line_type> _own_rich_lines;
        std::vector<std::reference_wrapper<const rich_line_type>> _rich_lines;

        void _transfer(const route &other);

        void _copy(const route &other);

        void _move(route &&other) noexcept;

        void _check_references() const;

        void _compute_length() const;

        void _compute_azimuth() const;

        void _compute_directions() const;

    };

}

#endif //MAP_MATCHING_2_ROUTE_HPP
