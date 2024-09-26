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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_DISTANCE_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_DISTANCE_HPP

#include "geometry/common.hpp"
#include "geometry/cache.hpp"

namespace map_matching_2::geometry {

    template<typename GeometryA, typename GeometryB>
    [[nodiscard]] typename boost::geometry::default_distance_result<GeometryA, GeometryB>::type
    distance(const GeometryA &a, const GeometryB &b) {
        using coordinate_system_type_a = typename boost::geometry::coordinate_system<GeometryA>::type;
        using coordinate_system_type_b = typename boost::geometry::coordinate_system<GeometryB>::type;
        if constexpr (std::same_as<coordinate_system_type_a, cs_spherical_equatorial> and
            std::same_as<coordinate_system_type_b, cs_spherical_equatorial>) {
            // mean radius of earth is 6371 km, and we want the result in meter
            return boost::geometry::distance(a, b, boost::geometry::strategy::distance::haversine(6371.0 * 1000));
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

    template<typename PointA, typename PointB>
    [[nodiscard]] typename boost::geometry::default_distance_result<PointA, PointB>::type
    euclidean_distance(const PointA &a, const PointB &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointA>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointB>));

        assert(boost::geometry::dimension<PointA>::value ==
                boost::geometry::dimension<PointB>::value);

        auto sum = default_float_type<typename boost::geometry::default_distance_result<PointA, PointB>::type>::v0;
        auto two = default_float_type<typename boost::geometry::default_distance_result<PointA, PointB>::type>::v2;

        if constexpr (boost::geometry::dimension<PointA>::value == 1) {
            sum += std::pow(boost::geometry::get<0>(a) - boost::geometry::get<0>(b), two);
        } else if constexpr (boost::geometry::dimension<PointA>::value == 2) {
            sum += std::pow(boost::geometry::get<0>(a) - boost::geometry::get<0>(b), two);
            sum += std::pow(boost::geometry::get<1>(a) - boost::geometry::get<1>(b), two);
        } else {
            sum += std::pow(boost::geometry::get<0>(a) - boost::geometry::get<0>(b), two);
            sum += std::pow(boost::geometry::get<1>(a) - boost::geometry::get<1>(b), two);
            sum += std::pow(boost::geometry::get<2>(a) - boost::geometry::get<2>(b), two);
        }

        return std::sqrt(sum);
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_DISTANCE_HPP
