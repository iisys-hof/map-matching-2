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

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "rich_segment.hpp"

namespace map_matching_2::geometry {

    struct rich_line_merge_exception : public std::exception {
    };

    template<typename Line>
    class rich_line {

    public:
        using point_type = typename boost::geometry::point_type<Line>::type;
        using line_type = Line;
        using segment_type = boost::geometry::model::segment<point_type>;
        using rich_segment_type = rich_segment<segment_type>;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using length_type = typename boost::geometry::default_length_result<Line>::type;
        using angle_type = typename boost::geometry::coordinate_type<Line>::type;

        rich_line();

        explicit rich_line(Line line);

        explicit rich_line(std::vector<rich_segment_type> rich_segments);

        ~rich_line() = default;

        rich_line(const rich_line &other);

        rich_line(rich_line &&other) noexcept;

        rich_line &operator=(const rich_line &other);

        rich_line &operator=(rich_line &&other) noexcept;

        [[nodiscard]] bool has_length() const;

        [[nodiscard]] length_type length() const;

        [[nodiscard]] bool has_azimuth() const;

        [[nodiscard]] angle_type azimuth() const;

        [[nodiscard]] bool has_directions() const;

        [[nodiscard]] angle_type directions() const;

        [[nodiscard]] angle_type absolute_directions() const;

        [[nodiscard]] const std::vector<rich_segment_type> &rich_segments() const;

        [[nodiscard]] const line_type &line() const;

        [[nodiscard]] angle_type segments_direction(std::size_t index) const;

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

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _line;
        }

    private:
        mutable std::mutex _mutex_length, _mutex_azimuth, _mutex_directions, _mutex_rich_segments;
        mutable std::atomic<bool> _computed_length, _computed_azimuth, _computed_directions, _computed_rich_segments;
        mutable bool _has_length, _has_azimuth, _has_directions;
        mutable length_type _length;
        mutable angle_type _azimuth, _directions, _absolute_directions;
        mutable std::vector<rich_segment_type> _rich_segments;
        line_type _line;

        void _transfer(const rich_line &other);

        void _copy(const rich_line &other);

        void _move(rich_line &&other) noexcept;

        void _compute_length() const;

        void _compute_azimuth() const;

        void _compute_directions() const;

        void _compute_rich_segments() const;

    };

}

namespace boost::serialization {

    template<typename Archive, typename Point>
    void serialize(Archive &ar, boost::geometry::model::linestring<Point> &line, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP((static_cast<std::vector<Point> &>(line)));
    }

}

#endif //MAP_MATCHING_2_RICH_LINE_HPP
