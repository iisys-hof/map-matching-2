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

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

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

        if constexpr(boost::geometry::traits::dimension<Point>::value == 1) {
            return boost::geometry::get<0>(a) == boost::geometry::get<0>(b);
        } else if constexpr(boost::geometry::traits::dimension<Point>::value == 2) {
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

        if constexpr(boost::geometry::traits::dimension<Point>::value == 1) {
            return boost::geometry::get<0>(a) < boost::geometry::get<0>(b);
        } else if constexpr(boost::geometry::traits::dimension<Point>::value == 2) {
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

    template<typename GeometryA, typename GeometryB>
    [[nodiscard]] constexpr typename boost::geometry::default_distance_result<GeometryA, GeometryB>::type
    distance(const GeometryA &a, const GeometryB &b) {
        return boost::geometry::distance(a, b);
    }

    template<typename PointA, typename PointB>
    [[nodiscard]] typename boost::geometry::default_distance_result<PointA, PointB>::type
    point_distance(const PointA &a, const PointB &b);

    template<typename PointA, typename PointB>
    [[nodiscard]] constexpr typename boost::geometry::default_distance_result<PointA, PointB>::type
    euclidean_distance(const PointA &a, const PointB &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointA>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointB>));

        assert(boost::geometry::traits::dimension<PointA>::value ==
               boost::geometry::traits::dimension<PointB>::value);

        auto sum = default_float_type<typename boost::geometry::default_distance_result<PointA, PointB>::type>::v0;
        auto two = default_float_type<typename boost::geometry::default_distance_result<PointA, PointB>::type>::v2;

        if constexpr(boost::geometry::traits::dimension<PointA>::value == 1) {
            sum += std::pow(boost::geometry::get<0>(a) - boost::geometry::get<0>(b), two);
        } else if constexpr(boost::geometry::traits::dimension<PointA>::value == 2) {
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
    [[nodiscard]] constexpr typename boost::geometry::default_length_result<Geometry>::type
    length(const Geometry &geometry) {
        return boost::geometry::length(geometry);
    }

    template<typename Segment>
    [[nodiscard]] constexpr typename boost::geometry::default_length_result<Segment>::type
    length_segment(const Segment &segment) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        return geometry::point_distance(segment.first, segment.second);
    }

    void clear_distance_cache();

    template<typename Point>
    [[nodiscard]] typename boost::geometry::coordinate_type<Point>::type
    azimuth_rad(const Point &a, const Point &b);

    void clear_azimuth_cache();

    template<typename Point>
    [[nodiscard]] constexpr typename boost::geometry::coordinate_type<Point>::type
    azimuth_deg(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        return rad2deg(azimuth_rad(a, b));
    }

    template<typename Line>
    [[nodiscard]] constexpr typename boost::geometry::coordinate_type<Line>::type
    azimuth_line_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        if (line.size() >= 2) {
            return azimuth_deg(line.front(), line.back());
        }
        return default_float_type<typename boost::geometry::coordinate_type<Line>::type>::v0;
    }

    template<typename Segment>
    [[nodiscard]] constexpr typename boost::geometry::coordinate_type<Segment>::type
    azimuth_segment_deg(const Segment &segment) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        return azimuth_deg(segment.first, segment.second);
    }

    template<typename Float>
    [[nodiscard]] constexpr Float direction_deg(const Float azimuth_deg_a, const Float azimuth_deg_b) {
        return angle_diff(azimuth_deg_a, azimuth_deg_b);
    }

    template<typename Point>
    [[nodiscard]] constexpr typename boost::geometry::coordinate_type<Point>::type
    direction_deg(const Point &a, const Point &b, const Point &c) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        return angle_diff(azimuth_deg(a, b), azimuth_deg(b, c));
    }

    template<typename Line>
    [[nodiscard]] constexpr typename boost::geometry::coordinate_type<Line>::type
    directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename boost::geometry::coordinate_type<Line>::type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            directions += direction_deg(line[i], line[j], line[k]);
        }
        return directions;
    }

    template<typename Line>
    [[nodiscard]] constexpr typename boost::geometry::coordinate_type<Line>::type
    absolute_directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename boost::geometry::coordinate_type<Line>::type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            directions += std::fabs(direction_deg(line[i], line[j], line[k]));
        }
        return directions;
    }

    template<typename Point, typename PointNew>
    [[nodiscard]] constexpr PointNew convert_point(const Point &point) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointNew>));

        if constexpr(boost::geometry::traits::dimension<Point>::value == 1) {
            return PointNew{
                    (typename boost::geometry::traits::coordinate_type<PointNew>::type) boost::geometry::get<0>(point)};
        } else if constexpr(boost::geometry::traits::dimension<Point>::value == 2) {
            return PointNew{
                    (typename boost::geometry::traits::coordinate_type<PointNew>::type) boost::geometry::get<0>(point),
                    (typename boost::geometry::traits::coordinate_type<PointNew>::type) boost::geometry::get<1>(point)};
        } else {
            return PointNew{
                    (typename boost::geometry::traits::coordinate_type<PointNew>::type) boost::geometry::get<0>(point),
                    (typename boost::geometry::traits::coordinate_type<PointNew>::type) boost::geometry::get<1>(point),
                    (typename boost::geometry::traits::coordinate_type<PointNew>::type) boost::geometry::get<2>(point)};
        }
    }

    template<typename Line, typename LineNew>
    [[nodiscard]] constexpr LineNew convert_line(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<LineNew>));

        using point_type = typename boost::geometry::point_type<Line>::type;
        using point_type_new = typename boost::geometry::point_type<LineNew>::type;

        LineNew line_new;
        line_new.reserve(line.size());
        for (const auto &point: line) {
            line_new.emplace_back(convert_point<point_type, point_type_new>(point));
        }
        return line_new;
    }

    template<typename Line>
    [[nodiscard]] constexpr std::pair<typename boost::geometry::coordinate_type<Line>::type,
            typename boost::geometry::coordinate_type<Line>::type>
    combined_directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename boost::geometry::coordinate_type<Line>::type>::v0;
        auto absolute_directions = default_float_type<typename boost::geometry::coordinate_type<Line>::type>::v0;
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

    template<typename Point>
    [[nodiscard]] boost::geometry::model::multi_polygon<boost::geometry::model::polygon<Point>>
    buffer(const Point &point, std::size_t buffer_points, double buffer_radius);

    template<typename Segment>
    [[nodiscard]] boost::geometry::model::box<typename boost::geometry::point_type<Segment>::type>
    segment_buffer_box(const Segment &segment, std::size_t buffer_points, double buffer_radius);

    template<template<typename> class Container, typename Line>
    [[nodiscard]] Container<boost::geometry::model::segment<typename boost::geometry::point_type<Line>::type>>
    line2segments(const Line &line);

    template<typename Segment, template<typename> class Container>
    [[nodiscard]] boost::geometry::model::linestring<typename boost::geometry::point_type<Segment>::type>
    segments2line(const Container<Segment> &segments);

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename boost::geometry::default_length_result<Segment>::type>
    segments_lengths(const Container<Segment> &segments);

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename boost::geometry::coordinate_type<Segment>::type>
    segments_azimuths_deg(const Container<Segment> &segments);

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename boost::geometry::coordinate_type<Segment>::type>
    segments_directions_deg(const Container<Segment> &segments,
                            const Container<typename boost::geometry::coordinate_type<Segment>::type> &segments_azimuths = {});

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] Container<typename boost::geometry::coordinate_type<Segment>::type>
    segments_absolute_directions_deg(const Container<Segment> &segments,
                                     const Container<typename boost::geometry::coordinate_type<Segment>::type> &segments_azimuths = {});

    template<template<typename> class Container, typename Segment>
    [[nodiscard]] std::pair<Container<typename boost::geometry::coordinate_type<Segment>::type>,
            Container<typename boost::geometry::coordinate_type<Segment>::type>>
    segments_combined_directions_deg(const Container<Segment> &segments,
                                     const Container<typename boost::geometry::coordinate_type<Segment>::type> &segments_azimuths = {});

    template<typename Line>
    [[nodiscard]] bool lines_are_reversed(const Line &a, const Line &b);

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
    [[nodiscard]] Point median_point(const Container<Point> &points);

}

#endif //MAP_MATCHING_2_GEOMETRY_UTIL_HPP
