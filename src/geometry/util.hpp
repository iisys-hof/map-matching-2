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

#ifndef MAP_MATCHING_2_GEOMETRY_UTIL_HPP
#define MAP_MATCHING_2_GEOMETRY_UTIL_HPP

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <algorithm>
#include <string>
#include <sstream>
#include <tuple>
#include <vector>

#include <absl/container/flat_hash_map.h>

#ifndef BOOST_THREAD_VERSION
#define BOOST_THREAD_VERSION 5
#endif

#include <boost/thread.hpp>

#include <boost/serialization/serialization.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "traits.hpp"

namespace map_matching_2::geometry {

    template<typename Float>
    struct default_float_type {
    };

    template<>
    struct default_float_type<float> {
        static constexpr float v0 = 0.0f;
        static constexpr float v1 = 1.0f;
        static constexpr float v2 = 2.0f;
        static constexpr float v180 = 180.0f;
        static constexpr float v360 = 360.0f;
    };

    template<>
    struct default_float_type<double> {
        static constexpr double v0 = 0.0;
        static constexpr double v1 = 1.0;
        static constexpr double v2 = 2.0;
        static constexpr double v180 = 180.0;
        static constexpr double v360 = 360.0;
    };

    template<>
    struct default_float_type<long double> {
        static constexpr long double v0 = 0.0L;
        static constexpr long double v1 = 1.0L;
        static constexpr long double v2 = 2.0L;
        static constexpr long double v180 = 180.0L;
        static constexpr long double v360 = 360.0L;
    };

    [[nodiscard]] constexpr std::uint64_t next_pow2(std::uint64_t x) {
        x--;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        x |= x >> 32;
        x++;
        return x;
    }

    template<typename Float>
    [[nodiscard]] constexpr Float rad2deg(const Float rad) {
        return rad * (default_float_type<Float>::v180 / M_PI);
    }

    template<typename Float>
    [[nodiscard]] constexpr Float deg2rad(const Float deg) {
        return deg * (M_PI / default_float_type<Float>::v180);
    }

    template<typename Float>
    [[nodiscard]] constexpr Float angle_diff(const Float a, const Float b) {
        Float d = a - b;
        return (d >= default_float_type<Float>::v180
                ? d - default_float_type<Float>::v360
                : (d <= -default_float_type<Float>::v180
                   ? d + default_float_type<Float>::v360
                   : d));
    }

    template<typename Geometry>
    [[nodiscard]] constexpr bool equals(const Geometry &a, const Geometry &b) {
        return boost::geometry::equals(a, b);
    }

    template<typename Point>
    [[nodiscard]] constexpr bool equals_points(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            return boost::geometry::get<0>(a) == boost::geometry::get<0>(b);
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            return boost::geometry::get<0>(a) == boost::geometry::get<0>(b) and
                   boost::geometry::get<1>(a) == boost::geometry::get<1>(b);
        } else {
            return boost::geometry::get<0>(a) == boost::geometry::get<0>(b) and
                   boost::geometry::get<1>(a) == boost::geometry::get<1>(b) and
                   boost::geometry::get<2>(a) == boost::geometry::get<2>(b);
        }
    }

    template<typename Point>
    [[nodiscard]] constexpr bool point_set_comparator(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            return boost::geometry::get<0>(a) < boost::geometry::get<0>(b);
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            return boost::geometry::get<0>(a) < boost::geometry::get<0>(b) or
                   (boost::geometry::get<0>(a) == boost::geometry::get<0>(b) and
                    boost::geometry::get<1>(a) < boost::geometry::get<1>(b));
        } else {
            return boost::geometry::get<0>(a) < boost::geometry::get<0>(b) or
                   ((boost::geometry::get<0>(a) == boost::geometry::get<0>(b) and
                     boost::geometry::get<1>(a) < boost::geometry::get<1>(b)) or
                    (boost::geometry::get<0>(a) == boost::geometry::get<0>(b) and
                     boost::geometry::get<1>(a) == boost::geometry::get<1>(b) and
                     boost::geometry::get<2>(a) < boost::geometry::get<2>(b)));
        }
    }

    using _cs_geographic = boost::geometry::cs::geographic<boost::geometry::degree>;
    using _cs_spherical_equatorial = boost::geometry::cs::spherical_equatorial<boost::geometry::degree>;
    using _cs_cartesian = boost::geometry::cs::cartesian;

    using _point_type = boost::geometry::model::point<double, 2, _cs_cartesian>;

    using _cache_key_type = std::tuple<double, double, double, double>;
    using distance_cache_type = absl::flat_hash_map<_cache_key_type, typename data<_point_type>::distance_type>;
    using azimuth_cache_type = absl::flat_hash_map<_cache_key_type, typename data<_point_type>::angle_type>;

    extern boost::thread_specific_ptr<distance_cache_type> distance_cache;
    extern boost::thread_specific_ptr<azimuth_cache_type> azimuth_cache;

    template<typename GeometryA, typename GeometryB>
    [[nodiscard]] constexpr typename boost::geometry::default_distance_result<GeometryA, GeometryB>::type
    distance(const GeometryA &a, const GeometryB &b) {
        return boost::geometry::distance(a, b);
    }

    template<typename PointA, typename PointB>
    [[nodiscard]] typename boost::geometry::default_distance_result<PointA, PointB>::type
    point_distance(const PointA &a, const PointB &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointA>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointB>));

        if (not distance_cache.get()) {
            distance_cache.reset(new distance_cache_type{});
        }

        const auto key = _cache_key_type{
                boost::geometry::get<0>(a), boost::geometry::get<1>(a),
                boost::geometry::get<0>(b), boost::geometry::get<1>(b)};
        const auto search = distance_cache->find(key);
        const bool exists = search != distance_cache->end();
        if (exists) {
            return search->second;
        } else {
            const auto distance = geometry::distance(a, b);
            distance_cache->emplace(key, distance);
            return distance;
        }
    }

    template<typename PointA, typename PointB>
    [[nodiscard]] constexpr typename boost::geometry::default_distance_result<PointA, PointB>::type
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

    template<typename Geometry>
    [[nodiscard]] constexpr typename data<Geometry>::length_type
    length(const Geometry &geometry) {
        return boost::geometry::length(geometry);
    }

    template<typename Segment>
    [[nodiscard]] constexpr typename data<Segment>::length_type
    length_segment(const Segment &segment) {
        if constexpr (geometry::is_segment_v<Segment> or geometry::is_referring_segment_v<Segment>) {
            return geometry::point_distance(segment.first, segment.second);
        } else if constexpr (geometry::is_pointing_segment_v<Segment>) {
            return geometry::point_distance(*(segment.first), *(segment.second));
        } else {
            static_assert(dependent_false_v<Segment>, "Invalid segment type");
            throw std::invalid_argument{"Invalid segment type"};
        }
    }

    inline void clear_distance_cache() {
        distance_cache.reset(new distance_cache_type{});
    }

    template<typename Point>
    [[nodiscard]] typename data<Point>::angle_type
    azimuth_rad(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if (not azimuth_cache.get()) {
            azimuth_cache.reset(new azimuth_cache_type{});
        }

        const auto key = _cache_key_type{
                boost::geometry::get<0>(a), boost::geometry::get<1>(a),
                boost::geometry::get<0>(b), boost::geometry::get<1>(b)};
        const auto search = azimuth_cache->find(key);
        const bool exists = search != azimuth_cache->end();
        if (exists) {
            return search->second;
        } else {
            const auto azimuth = boost::geometry::azimuth(a, b);
            azimuth_cache->emplace(key, azimuth);
            return azimuth;
        }
    }

    inline void clear_azimuth_cache() {
        azimuth_cache.reset(new azimuth_cache_type{});
    }

    template<typename Point>
    [[nodiscard]] constexpr typename data<Point>::angle_type
    azimuth_deg(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        return rad2deg(azimuth_rad(a, b));
    }

    template<typename Line>
    [[nodiscard]] constexpr typename data<Line>::angle_type
    azimuth_line_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        if (line.size() >= 2) {
            return azimuth_deg(line.front(), line.back());
        }
        return default_float_type<typename data<Line>::angle_type>::v0;
    }

    template<typename Segment>
    [[nodiscard]] constexpr typename data<Segment>::angle_type
    azimuth_segment_deg(const Segment &segment) {
        if constexpr (geometry::is_segment_v<Segment> or geometry::is_referring_segment_v<Segment>) {
            return azimuth_deg(segment.first, segment.second);
        } else if constexpr (geometry::is_pointing_segment_v<Segment>) {
            return azimuth_deg(*(segment.first), *(segment.second));
        } else {
            static_assert(dependent_false_v<Segment>, "Invalid segment type");
            throw std::invalid_argument{"Invalid segment type"};
        }
    }

    template<typename Float>
    [[nodiscard]] constexpr Float direction_deg(const Float azimuth_deg_a, const Float azimuth_deg_b) {
        return angle_diff(azimuth_deg_a, azimuth_deg_b);
    }

    template<typename Point>
    [[nodiscard]] constexpr typename data<Point>::angle_type
    direction_deg(const Point &a, const Point &b, const Point &c) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        return angle_diff(azimuth_deg(a, b), azimuth_deg(b, c));
    }

    template<typename Line>
    [[nodiscard]] constexpr typename data<Line>::angle_type
    directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename data<Line>::angle_type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            directions += direction_deg(line[i], line[j], line[k]);
        }
        return directions;
    }

    template<typename Line>
    [[nodiscard]] constexpr typename data<Line>::angle_type
    absolute_directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename data<Line>::angle_type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            directions += std::fabs(direction_deg(line[i], line[j], line[k]));
        }
        return directions;
    }

    template<typename Point, typename PointNew>
    [[nodiscard]] constexpr PointNew convert_point(const Point &point) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointNew>));

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            return PointNew{
                    (typename data<PointNew>::coordinate_type) boost::geometry::get<0>(point)};
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            return PointNew{
                    (typename data<PointNew>::coordinate_type) boost::geometry::get<0>(point),
                    (typename data<PointNew>::coordinate_type) boost::geometry::get<1>(point)};
        } else {
            return PointNew{
                    (typename data<PointNew>::coordinate_type) boost::geometry::get<0>(point),
                    (typename data<PointNew>::coordinate_type) boost::geometry::get<1>(point),
                    (typename data<PointNew>::coordinate_type) boost::geometry::get<2>(point)};
        }
    }

    template<typename Line, typename LineNew>
    [[nodiscard]] constexpr LineNew convert_line(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<LineNew>));

        using point_type = point_type_t<Line>;
        using point_type_new = point_type_t<LineNew>;

        LineNew line_new;
        line_new.reserve(line.size());
        for (const auto &point: line) {
            line_new.emplace_back(convert_point<point_type, point_type_new>(point));
        }
        return line_new;
    }

    template<typename Line>
    [[nodiscard]] constexpr std::pair<typename data<Line>::angle_type, typename data<Line>::angle_type>
    combined_directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename data<Line>::angle_type>::v0;
        auto absolute_directions = default_float_type<typename data<Line>::angle_type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            const auto direction = direction_deg(line[i], line[j], line[k]);
            directions += direction;
            absolute_directions += std::fabs(direction);
        }
        return std::pair{directions, absolute_directions};
    }

    template<typename Geometry>
    [[nodiscard]] inline std::string to_wkt(const Geometry &geometry, const std::int32_t precision = 12) {
        std::ostringstream wkt;
        wkt << std::setprecision(precision) << boost::geometry::wkt(geometry);
        return wkt.str();
    }

    using _buffer_side_strategy = boost::geometry::strategy::buffer::side_straight;
    using _buffer_join_strategy = boost::geometry::strategy::buffer::join_round;
    using _buffer_end_strategy = boost::geometry::strategy::buffer::end_round;

    const _buffer_side_strategy _buffer_side;
    const _buffer_join_strategy _buffer_join;
    const _buffer_end_strategy _buffer_end;

    template<typename Point>
    [[nodiscard]] typename models<Point>::multi_polygon_type
    buffer(const Point &point, double buffer_radius) {
        std::size_t buffer_points = geometry::next_pow2((std::uint64_t) (buffer_radius / 4));
        buffer_points = std::max(buffer_points, 8ul);

        return buffer(point, buffer_radius, buffer_points);
    }

    template<typename Point>
    [[nodiscard]] typename models<Point>::multi_polygon_type
    buffer(const Point &point, double buffer_radius, std::size_t buffer_points) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if (buffer_radius <= 0.0) {
            throw std::invalid_argument("buffer_radius in buffer method must be greater 0.0");
        }

        using multi_polygon_type = typename models<Point>::multi_polygon_type;
        multi_polygon_type buffer;

        using buffer_distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<
                typename data<Point>::coordinate_type>;
        const buffer_distance_strategy buffer_distance{buffer_radius};

        if constexpr (std::is_same_v<typename data<Point>::coordinate_system_type, _cs_geographic> or
                      std::is_same_v<typename data<Point>::coordinate_system_type, _cs_spherical_equatorial>) {
            using buffer_point_strategy = boost::geometry::strategy::buffer::geographic_point_circle<>;
            const buffer_point_strategy buffer_point{buffer_points};

            boost::geometry::buffer(point, buffer, buffer_distance, _buffer_side, _buffer_join, _buffer_end,
                                    buffer_point);
        } else if constexpr (std::is_same_v<typename data<Point>::coordinate_system_type, _cs_cartesian>) {
            using buffer_point_strategy = boost::geometry::strategy::buffer::point_circle;
            const buffer_point_strategy buffer_point{buffer_points};

            boost::geometry::buffer(point, buffer, buffer_distance, _buffer_side, _buffer_join, _buffer_end,
                                    buffer_point);
        } else {
            static_assert(dependent_false_v<typename data<Point>::coordinate_system_type>, "Invalid coordinate system");
        }

        return buffer;
    }

    template<typename Point>
    [[nodiscard]] typename models<Point>::box_type
    buffer_box(const Point &point, double buffer_radius) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        using multi_polygon_type = typename models<Point>::multi_polygon_type;
        using box_type = typename models<Point>::box_type;

        const multi_polygon_type buffer_polygon = buffer(point, buffer_radius, 4ul);

        box_type buffer_box;
        boost::geometry::envelope(buffer_polygon, buffer_box);

        return buffer_box;
    }

    template<typename Segment>
    [[nodiscard]] typename models<point_type_t<Segment>>::box_type
    segment_buffer_box(const Segment &segment, std::size_t buffer_points, double buffer_radius) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using point_type = point_type_t<Segment>;
        using box_type = typename models<point_type>::box_type;
        using coordinate_type = typename data<point_type>::coordinate_type;

        coordinate_type min_x = std::numeric_limits<coordinate_type>::infinity();
        coordinate_type min_y = std::numeric_limits<coordinate_type>::infinity();
        coordinate_type max_x = -std::numeric_limits<coordinate_type>::infinity();
        coordinate_type max_y = -std::numeric_limits<coordinate_type>::infinity();

        const auto coordinate_finder = [&](const point_type &point, coordinate_type &min_x, coordinate_type &min_y,
                                           coordinate_type &max_x, coordinate_type &max_y) {
            box_type box = buffer_box(point, buffer_radius);

            const point_type &min_corner = box.min_corner();
            const point_type &max_corner = box.max_corner();

            min_x = std::min(min_x, min_corner.template get<0>());
            min_y = std::min(min_y, min_corner.template get<1>());
            max_x = std::max(max_x, max_corner.template get<0>());
            max_y = std::max(max_y, max_corner.template get<1>());
        };

        coordinate_finder(segment.first, min_x, min_y, max_x, max_y);
        coordinate_finder(segment.second, min_x, min_y, max_x, max_y);

        return box_type{point_type{min_x, min_y}, point_type{max_x, max_y}};
    }

    template<template<typename> class Container, typename Line>
    [[nodiscard]] Container<typename models<point_type_t<Line>>::segment_type>
    line2segments(const Line &line) {
        using segment_type = typename models<point_type_t<Line>>::segment_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<segment_type>));

        Container<segment_type> segments;
        if (line.size() >= 2) {
            if constexpr (std::is_void_v<decltype(&Container<segment_type>::reserve)>) {
                segments.reserve(line.size() - 1);
            }
            for (std::size_t i = 0; i < line.size() - 1; ++i) {
                segments.emplace_back(segment_type{line[i], line[i + 1]});
            }
        }
        return segments;
    }

    template<typename Segment, template<typename> class Container>
    [[nodiscard]] typename models<point_type_t<Segment>>::line_type
    segments2line(const Container<Segment> &segments) {
        using line_type = typename models<point_type_t<Segment>>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        line_type line;
        if (not segments.empty()) {
            line.reserve(segments.size() + 1);
            for (const Segment &segment: segments) {
                if (line.empty()) {
                    line.emplace_back(segment.first);
                }
                line.emplace_back(segment.second);
            }
        }
        return line;
    }

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename data<Segment>::length_type>
    segments_lengths(const Container<Segment> &segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using length_type = typename data<Segment>::length_type;

        Container<length_type> lengths;
        if constexpr (std::is_void_v<decltype(&Container<length_type>::reserve)>) {
            lengths.reserve(segments.size());
        }
        for (const auto &segment: segments) {
            lengths.emplace_back(geometry::length_segment(segment));
        }
        return lengths;
    }

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename data<Segment>::angle_type>
    segments_azimuths_deg(const Container<Segment> &segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using angle_type = typename data<Segment>::angle_type;

        Container<angle_type> azimuths;
        if constexpr (std::is_void_v<decltype(&Container<angle_type>::reserve)>) {
            azimuths.reserve(segments.size());
        }
        for (const auto &segment: segments) {
            azimuths.emplace_back(azimuth_segment_deg(segment));
        }
        return azimuths;
    }

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename data<Segment>::angle_type>
    segments_directions_deg(const Container<Segment> &segments,
                            const Container<typename data<Segment>::angle_type> &segments_azimuths = {}) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename data<Segment>::angle_type;

        Container<angle_type> directions;
        if constexpr (std::is_void_v<decltype(&Container<angle_type>::reserve)>) {
            directions.reserve(segments.size() - 1);
        }
        if (segments_azimuths.empty()) {
            for (std::size_t i = 0, j = 1; j < segments.size(); ++i, ++j) {
                const auto &a = segments[i];
                const auto &b = segments[j];
                assert(geometry::equals_points(a.second, b.first));
                directions.emplace_back(direction_deg(a.first, a.second, b.second));
            }
        } else {
            for (std::size_t i = 0, j = 1; j < segments_azimuths.size(); ++i, ++j) {
                directions.emplace_back(direction_deg(segments_azimuths[i], segments_azimuths[j]));
            }
        }
        return directions;
    }

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename data<Segment>::angle_type>
    segments_absolute_directions_deg(const Container<Segment> &segments,
                                     const Container<typename data<Segment>::angle_type> &segments_azimuths = {}) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename data<Segment>::angle_type;

        Container<angle_type> directions;
        if constexpr (std::is_void_v<decltype(&Container<angle_type>::reserve)>) {
            directions.reserve(segments.size() - 1);
        }
        if (segments_azimuths.empty()) {
            for (std::size_t i = 0, j = 1; j < segments.size(); ++i, ++j) {
                const auto &a = segments[i];
                const auto &b = segments[j];
                assert(geometry::equals_points(a.second, b.first));
                directions.emplace_back(std::fabs(direction_deg(a.first, a.second, b.second)));
            }
        } else {
            for (std::size_t i = 0, j = 1; j < segments_azimuths.size(); ++i, ++j) {
                directions.emplace_back(std::fabs(direction_deg(segments_azimuths[i], segments_azimuths[j])));
            }
        }
        return directions;
    }

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] std::pair<Container<typename data<Segment>::angle_type>,
            Container<typename data<Segment>::angle_type>>
    segments_combined_directions_deg(const Container<Segment> &segments,
                                     const Container<typename data<Segment>::angle_type> &segments_azimuths = {}) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename data<Segment>::angle_type;

        Container<angle_type> directions;
        Container<angle_type> absolute_directions;
        if constexpr (std::is_void_v<decltype(&Container<angle_type>::reserve)>) {
            directions.reserve(segments.size() - 1);
            absolute_directions.reserve(segments.size() - 1);
        }
        if (segments_azimuths.empty()) {
            for (std::size_t i = 0, j = 1; j < segments.size(); ++i, ++j) {
                const auto &a = segments[i];
                const auto &b = segments[j];
                assert(geometry::equals_points(a.second, b.first));
                const auto direction = direction_deg(a.first, a.second, b.second);
                directions.emplace_back(direction);
                absolute_directions.emplace_back(std::fabs(direction));
            }
        } else {
            for (std::size_t i = 0, j = 1; j < segments_azimuths.size(); ++i, ++j) {
                const auto direction = direction_deg(segments_azimuths[i], segments_azimuths[j]);
                directions.emplace_back(direction);
                absolute_directions.emplace_back(std::fabs(direction));
            }
        }
        return std::pair{std::move(directions), std::move(absolute_directions)};
    }

    template<typename Line>
    [[nodiscard]] bool lines_are_reversed(const Line &a, const Line &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));

        if (a.empty() or b.empty()) {
            // when one line is empty, they cannot be reverse to each other
            return false;
        }

        if (a.size() != b.size()) {
            // if sizes are not equal, they cannot be reverse to each other
            return false;
        }

        if (not(equals_points(a.front(), b.back()) and equals_points(a.back(), b.front()))) {
            // back and front are not equal, we can already quit
            return false;
        }

        // check the rest of the lines when front and back were equal
        for (std::size_t i = 1, j = b.size() - 2; i < a.size(); ++i, --j) {
            if (not equals_points(a[i], b[j])) {
                // if only one point pair is not equal, they cannot be reverse
                return false;
            }
        }

        // all points were equal concerning reverse lines
        return true;
    }

    template<template<typename> class Container, typename Element>
    [[nodiscard]] Element median(Container<Element> &container) {
        assert(not container.empty());

        std::size_t middle = container.size() / 2;
        std::nth_element(container.begin(), container.begin() + middle, container.end());
        if (container.size() % 2 == 1) {
            return container[middle];
        } else {
            std::nth_element(container.begin(), container.begin() + middle - 1, container.end());
            return (container[middle - 1] + container[middle]) / static_cast<Element>(2);
        }
    }

    template<template<typename> class Container, typename Element>
    [[nodiscard]] Element median_absolute_deviation(Element center, Container<Element> &container) {
        assert(not container.empty());

        for (Element &element: container) {
            element = std::abs(element - center);
        }
        return median(container);
    }

    template<template<typename> class Container, typename Element>
    [[nodiscard]] Element median_absolute_deviation(Container<Element> &container) {
        return median_absolute_deviation(median(container), container);
    }

    template<template<typename> class Container, typename Point>
    [[nodiscard]] Point median_point(const Container<Point> &points) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        assert(not points.empty());
        assert(boost::geometry::dimension<Point>::value >= 1 and boost::geometry::dimension<Point>::value <= 3);

        using coordinate_type = typename data<Point>::coordinate_type;

        std::vector<coordinate_type> x, y, z;
        if constexpr (boost::geometry::dimension<Point>::value >= 1) {
            x.reserve(points.size());
        }
        if constexpr (boost::geometry::dimension<Point>::value >= 2) {
            y.reserve(points.size());
        }
        if constexpr (boost::geometry::dimension<Point>::value == 3) {
            z.reserve(points.size());
        }

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            for (const auto &point: points) {
                x.emplace_back(boost::geometry::get<0>(point));
            }
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            for (const auto &point: points) {
                x.emplace_back(boost::geometry::get<0>(point));
                y.emplace_back(boost::geometry::get<1>(point));
            }
        } else {
            for (const auto &point: points) {
                x.emplace_back(boost::geometry::get<0>(point));
                y.emplace_back(boost::geometry::get<1>(point));
                z.emplace_back(boost::geometry::get<2>(point));
            }
        }

        coordinate_type median_x, median_y, median_z;
        if constexpr (boost::geometry::dimension<Point>::value >= 1) {
            median_x = median(x);
        }
        if constexpr (boost::geometry::dimension<Point>::value >= 2) {
            median_y = median(y);
        }
        if constexpr (boost::geometry::dimension<Point>::value == 3) {
            median_z = median(z);
        }

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            return Point{median_x};
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            return Point{median_x, median_y};
        } else {
            return Point{median_x, median_y, median_z};
        }
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_UTIL_HPP
