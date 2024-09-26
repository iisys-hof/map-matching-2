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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_BUFFER_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_BUFFER_HPP

#include "util/concepts.hpp"

#include "types/geometry/point.hpp"

#include "geometry/common.hpp"
#include "geometry/types.hpp"
#include "geometry/traits.hpp"

namespace map_matching_2::geometry {

    using _buffer_side_strategy = boost::geometry::strategy::buffer::side_straight;
    using _buffer_join_strategy = boost::geometry::strategy::buffer::join_round;
    using _buffer_end_strategy = boost::geometry::strategy::buffer::end_round;

    const _buffer_side_strategy _buffer_side;
    const _buffer_join_strategy _buffer_join;
    const _buffer_end_strategy _buffer_end;

    template<typename Point>
    [[nodiscard]] typename models<Point>::template multi_polygon_type<>
    buffer(const Point &point, const double buffer_radius, const std::size_t buffer_points) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if (buffer_radius <= 0.0) {
            throw std::invalid_argument("buffer_radius in buffer method must be greater 0.0");
        }

        using multi_polygon_type = typename models<Point>::template multi_polygon_type<>;
        multi_polygon_type buffer;

        using buffer_distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<
            typename data<Point>::coordinate_type>;
        const buffer_distance_strategy buffer_distance{buffer_radius};

        using coordinate_system_type = typename data<Point>::coordinate_system_type;

        if constexpr (std::is_same_v<coordinate_system_type, cs_geographic> or
            std::is_same_v<coordinate_system_type, cs_spherical_equatorial>) {
            using buffer_point_strategy = boost::geometry::strategy::buffer::geographic_point_circle<>;
            const buffer_point_strategy buffer_point{buffer_points};

            boost::geometry::buffer(
                    point, buffer, buffer_distance, _buffer_side, _buffer_join, _buffer_end, buffer_point);
        } else if constexpr (std::is_same_v<coordinate_system_type, cs_cartesian>) {
            using buffer_point_strategy = boost::geometry::strategy::buffer::point_circle;
            const buffer_point_strategy buffer_point{buffer_points};

            boost::geometry::buffer(
                    point, buffer, buffer_distance, _buffer_side, _buffer_join, _buffer_end, buffer_point);
        } else {
            static_assert(util::dependent_false_v<coordinate_system_type>, "Invalid coordinate system");
            throw std::invalid_argument{"Invalid coordinate system"};
        }

        return buffer;
    }

    template<typename Point>
    [[nodiscard]] typename models<Point>::template multi_polygon_type<>
    buffer(const Point &point, const double buffer_radius) {
        std::size_t buffer_points = next_pow2(static_cast<std::uint64_t>(buffer_radius / 4));
        buffer_points = std::max(buffer_points, std::size_t{8ul});

        return geometry::buffer<Point>(point, buffer_radius, buffer_points);
    }

    template<typename Point>
    [[nodiscard]] typename models<Point>::box_type
    buffer_box(const Point &point, const double buffer_radius) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        using multi_polygon_type = typename models<Point>::template multi_polygon_type<>;
        using box_type = typename models<Point>::box_type;

        const multi_polygon_type buffer_polygon = geometry::buffer<Point>(point, buffer_radius, 4ul);

        box_type buffer_box;
        boost::geometry::envelope(buffer_polygon, buffer_box);

        return buffer_box;
    }

    template<typename Segment>
    [[nodiscard]] typename models<point_type_t<Segment>>::box_type
    segment_buffer_box(const Segment &segment, const double buffer_radius) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using point_type = point_type_t<Segment>;
        using box_type = typename models<point_type>::box_type;
        using coordinate_type = typename data<point_type>::coordinate_type;

        coordinate_type min_x = std::numeric_limits<coordinate_type>::infinity();
        coordinate_type min_y = std::numeric_limits<coordinate_type>::infinity();
        coordinate_type max_x = -std::numeric_limits<coordinate_type>::infinity();
        coordinate_type max_y = -std::numeric_limits<coordinate_type>::infinity();

        const auto coordinate_finder = [&](const point_type &point,
                coordinate_type &_min_x, coordinate_type &_min_y,
                coordinate_type &_max_x, coordinate_type &_max_y) {
            box_type box = buffer_box(point, buffer_radius);

            const point_type &min_corner = box.min_corner();
            const point_type &max_corner = box.max_corner();

            _min_x = std::min(_min_x, min_corner.template get<0>());
            _min_y = std::min(_min_y, min_corner.template get<1>());
            _max_x = std::max(_max_x, max_corner.template get<0>());
            _max_y = std::max(_max_y, max_corner.template get<1>());
        };

        coordinate_finder(segment.first, min_x, min_y, max_x, max_y);
        coordinate_finder(segment.second, min_x, min_y, max_x, max_y);

        return box_type{point_type{min_x, min_y}, point_type{max_x, max_y}};
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_BUFFER_HPP
