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

#ifndef MAP_MATCHING_2_GEOMETRY_POINT_HPP
#define MAP_MATCHING_2_GEOMETRY_POINT_HPP

#include <cstddef>
#include <functional>

#include <boost/container_hash/hash.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

namespace boost::geometry::model {

    template<typename Point, std::size_t... Is>
    std::size_t hash_value_impl(const Point &data, std::index_sequence<Is...>) {
        std::size_t seed = 0;
        ((boost::hash_combine(seed, geometry::get<Is>(data))), ...);
        return seed;
    }

    template<typename Point, std::size_t... Is>
    bool equals_impl(const Point &left, const Point &right, std::index_sequence<Is...>) {
        return ((geometry::get<Is>(left) == geometry::get<Is>(right)) && ...);
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    std::size_t hash_value(const point<CoordinateType, DimensionCount, CoordinateSystem> &data) {
        return hash_value_impl(data, std::make_index_sequence<DimensionCount>{});
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator==(const point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return equals_impl(left, right, std::make_index_sequence<DimensionCount>{});
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator!=(const point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return not(left == right);
    }

    template<typename Point, std::size_t I = 0>
    bool less_impl(const Point &left, const Point &right) {
        if constexpr (I < geometry::dimension<Point>::value) {
            return geometry::get<I>(left) < geometry::get<I>(right) ||
            (geometry::get<I>(left) == geometry::get<I>(right) &&
                less_impl<Point, I + 1>(left, right));
        }
        return false;
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator<(const point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return less_impl(left, right);
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator<=(const point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return not(right < left); // equivalent to left <= right
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator>(const point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return right < left; // equivalent to left > right
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator>=(const point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return not(left < right); // equivalent to left >= right
    }

}

namespace std {

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    struct hash<boost::geometry::model::point<CoordinateType, DimensionCount, CoordinateSystem>> {
        using type = boost::geometry::model::point<CoordinateType, DimensionCount, CoordinateSystem>;

        std::size_t operator()(const type &data) const noexcept {
            return hash_value(data);
        }
    };

}

namespace map_matching_2::geometry {

    template<typename Geometry>
    using point_type_t = typename boost::geometry::point_type<Geometry>::type;

    template<typename Point>
    concept is_point = std::is_same_v<typename boost::geometry::tag<Point>::type, boost::geometry::point_tag>;

}

#endif //MAP_MATCHING_2_GEOMETRY_POINT_HPP
