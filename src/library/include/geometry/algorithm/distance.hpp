// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_DISTANCE_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_DISTANCE_HPP

#include "util/concepts.hpp"

#include "geometry/common.hpp"
#include "geometry/cache.hpp"

namespace map_matching_2::geometry {

    // https://en.wikipedia.org/wiki/Earth_radius
    constexpr double EARTH_RADIUS_METER = 6378137.0;

    template<typename GeometryA, typename GeometryB>
    [[nodiscard]] typename boost::geometry::default_distance_result<GeometryA, GeometryB>::type
    distance(const GeometryA &a, const GeometryB &b) {
        using coordinate_system_type_a = typename boost::geometry::coordinate_system<GeometryA>::type;
        using coordinate_system_type_b = typename boost::geometry::coordinate_system<GeometryB>::type;
        if constexpr (std::same_as<coordinate_system_type_a, cs_spherical_equatorial> and
            std::same_as<coordinate_system_type_b, cs_spherical_equatorial>) {
            return boost::geometry::distance(a, b,
                    boost::geometry::strategy::distance::haversine(EARTH_RADIUS_METER));
        } else {
            return boost::geometry::distance(a, b);
        }
    }

    template<typename PointA, typename PointB>
    [[nodiscard]] typename boost::geometry::default_distance_result<PointA, PointB>::type
    point_distance(const PointA &a, const PointB &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointA>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointB>));

        if (use_distance_cache) {
            if (not distance_cache) {
                reset_distance_cache();
            }

            const auto key = _cache_key_type{
                    boost::geometry::get<0>(a), boost::geometry::get<1>(a),
                    boost::geometry::get<0>(b), boost::geometry::get<1>(b)
            };
            const auto search = distance_cache->find(key);
            const bool exists = search != distance_cache->end();
            if (exists) {
                return search->second;
            } else {
                const auto distance = geometry::distance(a, b);
                distance_cache->emplace(key, distance);
                return distance;
            }
        } else {
            return geometry::distance(a, b);
        }
    }

    template<typename Point, std::size_t Dimension = boost::geometry::dimension<Point>::value>
    [[nodiscard]] std::array<typename data<Point>::coordinate_type, Dimension>
    degrees2meters(const Point &point, typename data<Point>::coordinate_type latitude =
            std::numeric_limits<typename data<Point>::coordinate_type>::signaling_NaN()) {
        using coordinate_type = typename data<Point>::coordinate_type;
        using coordinate_system_type = typename data<Point>::coordinate_system_type;

        constexpr std::size_t dimension = boost::geometry::dimension<Point>::value;
        static_assert(dimension >= 1 and dimension <= 3, "invalid dimension");

        if constexpr (std::same_as<coordinate_system_type, cs_geographic> or
            std::same_as<coordinate_system_type, cs_spherical_equatorial>) {
            // we only use degree coordinate systems, no further check required

            constexpr coordinate_type PI = std::numbers::pi_v<coordinate_type>;
            constexpr coordinate_type EARTH_PERIMETER = 2 * PI * EARTH_RADIUS_METER;

            if (std::isnan(latitude)) {
                if constexpr (dimension == 1) {
                    // assume latitude to be zero
                    latitude = default_float_type<coordinate_type>::v0;
                } else if constexpr (dimension >= 2) {
                    // get latitude from point
                    latitude = boost::geometry::get<1>(point);
                }
            }

            const coordinate_type longitude = boost::geometry::get<0>(point);

            // convert latitude to meters
            const coordinate_type latitude_meters = latitude * EARTH_PERIMETER / 360.0;

            // length of one degree longitude at latitude (spherical earth)
            const coordinate_type longitude_length = EARTH_PERIMETER * std::cos(latitude * PI / 180.0) / 360.0;

            // convert longitude to meters
            const coordinate_type longitude_meters = longitude * longitude_length;

            if constexpr (dimension == 1) {
                return {longitude_meters};
            } else if constexpr (dimension == 2) {
                return {longitude_meters, latitude_meters};
            } else {
                // altitude is usually already in meters
                return {longitude_meters, latitude_meters, boost::geometry::get<2>(point)};
            }
        } else if constexpr (std::same_as<coordinate_system_type, cs_cartesian>) {
            // keep it, already in meters
            if constexpr (dimension == 1) {
                return {boost::geometry::get<0>(point)};
            } else if constexpr (dimension == 2) {
                return {boost::geometry::get<0>(point), boost::geometry::get<1>(point)};
            } else {
                return {boost::geometry::get<0>(point), boost::geometry::get<1>(point), boost::geometry::get<2>(point)};
            }
        } else {
            static_assert(util::dependent_false_v<coordinate_system_type>, "invalid coordinate system");
            throw std::invalid_argument{"invalid coordinate system"};
        }
    }

    template<typename PointA, typename PointB>
    [[nodiscard]] typename boost::geometry::default_distance_result<PointA, PointB>::type
    euclidean_distance(const PointA &a, const PointB &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointA>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointB>));

        constexpr std::size_t dimension_a = boost::geometry::dimension<PointA>::value;
        constexpr std::size_t dimension_b = boost::geometry::dimension<PointB>::value;

        static_assert(dimension_a == dimension_b, "dimensions do not match");

        using coordinate_system_type_a = typename data<PointA>::coordinate_system_type;
        using coordinate_system_type_b = typename data<PointB>::coordinate_system_type;

        static_assert(std::same_as<coordinate_system_type_a, coordinate_system_type_b>,
                "coordinate systems do not match");

        constexpr auto TWO = default_float_type<
            typename boost::geometry::default_distance_result<PointA, PointB>::type>::v2;

        auto sum = default_float_type<typename boost::geometry::default_distance_result<PointA, PointB>::type>::v0;

        // only converts if not already in meters
        const auto meters_a = degrees2meters<PointA, dimension_a>(a);
        const auto meters_b = degrees2meters<PointB, dimension_b>(b);

        if constexpr (dimension_a == 1) {
            sum += std::pow(meters_a[0] - meters_b[0], TWO);
        } else if constexpr (dimension_a == 2) {
            sum += std::pow(meters_a[0] - meters_b[0], TWO);
            sum += std::pow(meters_a[1] - meters_b[1], TWO);
        } else {
            sum += std::pow(meters_a[0] - meters_b[0], TWO);
            sum += std::pow(meters_a[1] - meters_b[1], TWO);
            sum += std::pow(meters_a[2] - meters_b[2], TWO);
        }

        return std::sqrt(sum);
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_DISTANCE_HPP
