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

#include <cstdint>
#include <cstddef>
#include <functional>

#include <boost/container_hash/hash.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

namespace map_matching_2::geometry {

    template<typename Geometry>
    using point_type_t = typename boost::geometry::point_type<Geometry>::type;

    template<typename Point>
    concept is_point = std::is_same_v<typename boost::geometry::tag<Point>::type, boost::geometry::point_tag>;

    template<typename TimePoint>
    concept is_time_point = is_point<TimePoint> &&
            requires(const TimePoint &time_point) {
                { time_point.timestamp() } -> std::same_as<std::uint64_t>;
            };

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    class time_point : public boost::geometry::model::point<CoordinateType, DimensionCount, CoordinateSystem> {
    public:
        using base_type = boost::geometry::model::point<CoordinateType, DimensionCount, CoordinateSystem>;
        using timestamp_type = std::uint64_t;

        constexpr time_point() = default;

        constexpr explicit time_point(const CoordinateType &v0,
                const timestamp_type time = 0) requires
            (boost::geometry::detail::is_coordinates_number_leq<CoordinateType, 1, DimensionCount>::value)
            : base_type{v0}, _time{time} {}

        constexpr time_point(const CoordinateType &v0, const CoordinateType &v1,
                const timestamp_type time = 0) requires
            (boost::geometry::detail::is_coordinates_number_leq<CoordinateType, 2, DimensionCount>::value)
            : base_type{v0, v1}, _time{time} {}

        constexpr time_point(const CoordinateType &v0, const CoordinateType &v1, const CoordinateType &v2,
                const timestamp_type time = 0) requires
            (boost::geometry::detail::is_coordinates_number_leq<CoordinateType, 3, DimensionCount>::value)
            : base_type{v0, v1, v2}, _time{time} {}

        constexpr time_point(const time_point &other) = default;

        constexpr time_point(const time_point &other, const timestamp_type time)
            : base_type(other), _time{time} {}

        template<typename Point> requires
            (not std::same_as<Point, time_point> and std::derived_from<Point, base_type>)
        constexpr time_point(const Point &other)
            : base_type(other) {}

        constexpr time_point(time_point &&other) noexcept = default;

        constexpr time_point(time_point &&other, const timestamp_type time) noexcept
            : base_type(std::move(other)), _time{time} {}

        template<typename Point> requires
            (not std::same_as<Point, time_point> and std::derived_from<Point, base_type>)
        constexpr time_point(Point &&other) noexcept
            : base_type(std::forward<base_type>(other)) {}

        constexpr time_point &operator=(const time_point &other) = default;

        template<typename Point> requires
            (not std::same_as<Point, time_point> and std::derived_from<Point, base_type>)
        constexpr time_point &operator=(const Point &other) {
            if (this != &other) {
                base_type::operator=(other);
            }
            return *this;
        }

        constexpr time_point &operator=(time_point &&other) noexcept = default;

        template<typename Point> requires
            (not std::same_as<Point, time_point> and std::derived_from<Point, base_type>)
        constexpr time_point &operator=(Point &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::forward<base_type>(other));
            }
            return *this;
        }

        constexpr ~time_point() = default;

        [[nodiscard]] constexpr timestamp_type timestamp() const {
            return _time;
        }

        constexpr void timestamp(const timestamp_type time) {
            _time = time;
        }

    private:
        timestamp_type _time{0};

    };

}

namespace boost::geometry::traits {

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    struct tag<map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem>> {
        using type = point_tag;
    };

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    struct coordinate_type<map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem>> {
        using type = CoordinateType;
    };

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    struct coordinate_system<map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem>> {
        using type = CoordinateSystem;
    };

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    struct dimension<map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem>>
            : std::integral_constant<std::size_t, DimensionCount> {};

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem, std::size_t Dimension>
    struct access<map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem>, Dimension> {
        static constexpr CoordinateType get(
                const map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem> &p) {
            return p.template get<Dimension>();
        }

        static void set(map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem> &p,
                const CoordinateType &value) {
            p.template set<Dimension>(value);
        }
    };

}

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

namespace map_matching_2::geometry {

    template<is_time_point TimePoint, std::size_t... Is>
    std::size_t hash_value_impl(const TimePoint &data, std::index_sequence<Is...>) {
        std::size_t seed = 0;
        boost::hash_combine(seed, data.timestamp());
        ((boost::hash_combine(seed, boost::geometry::get<Is>(data))), ...);
        return seed;
    }

    template<is_time_point TimePoint, std::size_t... Is>
    bool equals_impl(const TimePoint &left, const TimePoint &right, std::index_sequence<Is...>) {
        return left.timestamp() == right.timestamp() &&
                ((boost::geometry::get<Is>(left) == boost::geometry::get<Is>(right)) && ...);
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    std::size_t hash_value(const time_point<CoordinateType, DimensionCount, CoordinateSystem> &data) {
        return hash_value_impl(data, std::make_index_sequence<DimensionCount>{});
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator==(const time_point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const time_point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return equals_impl(left, right, std::make_index_sequence<DimensionCount>{});
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator!=(const time_point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const time_point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return not(left == right);
    }

    template<is_time_point TimePoint, std::size_t I = 0>
    bool less_impl(const TimePoint &left, const TimePoint &right) {
        if constexpr (I < boost::geometry::dimension<TimePoint>::value) {
            return left.timestamp() < right.timestamp() ||
                    left.timestamp() == right.timestamp() && (
                        boost::geometry::get<I>(left) < boost::geometry::get<I>(right) ||
                        (boost::geometry::get<I>(left) == boost::geometry::get<I>(right) &&
                            less_impl<TimePoint, I + 1>(left, right)));
        }
        return false;
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator<(const time_point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const time_point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return less_impl(left, right);
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator<=(const time_point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const time_point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return not(right < left); // equivalent to left <= right
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator>(const time_point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const time_point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
        return right < left; // equivalent to left > right
    }

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    bool operator>=(const time_point<CoordinateType, DimensionCount, CoordinateSystem> &left,
            const time_point<CoordinateType, DimensionCount, CoordinateSystem> &right) {
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

    template<typename CoordinateType, std::size_t DimensionCount, typename CoordinateSystem>
    struct hash<map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem>> {
        using type = map_matching_2::geometry::time_point<CoordinateType, DimensionCount, CoordinateSystem>;

        std::size_t operator()(const type &data) const noexcept {
            return hash_value(data);
        }
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_POINT_HPP
