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

#include "util.hpp"

#ifndef BOOST_THREAD_VERSION
#define BOOST_THREAD_VERSION 5
#endif

#include <unordered_map>

#include <boost/thread.hpp>

#include <boost/geometry/algorithms/detail/azimuth.hpp>

#include "types.hpp"

namespace map_matching_2::geometry {

    using _cache_key_type = std::tuple<double, double, double, double>;
    using _distance_cache_type = std::unordered_map<_cache_key_type,
            typename boost::geometry::default_distance_result<geometry::types_geographic::point_type, geometry::types_geographic::point_type>::type,
            boost::hash<_cache_key_type>>;
    using _azimuth_cache_type = std::unordered_map<_cache_key_type,
            typename boost::geometry::coordinate_type<geometry::types_geographic::point_type>::type,
            boost::hash<_cache_key_type>>;

    boost::thread_specific_ptr<_distance_cache_type> _distance_cache;
    boost::thread_specific_ptr<_azimuth_cache_type> _azimuth_cache;

    template<typename PointA, typename PointB>
    typename boost::geometry::default_distance_result<PointA, PointB>::type
    point_distance(const PointA &a, const PointB &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointA>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointB>));

        if (not _distance_cache.get()) {
            _distance_cache.reset(new _distance_cache_type{});
        }

        const auto key = _cache_key_type{
                boost::geometry::get<0>(a), boost::geometry::get<1>(a),
                boost::geometry::get<0>(b), boost::geometry::get<1>(b)};
        const auto search = _distance_cache->find(key);
        const bool exists = search != _distance_cache->end();
        if (exists) {
            return search->second;
        } else {
            const auto distance = geometry::distance(a, b);
            _distance_cache->emplace(key, distance);
            return distance;
        }
    }

    template
    typename boost::geometry::default_distance_result<geometry::types_geographic::point_type, geometry::types_geographic::point_type>::type
    point_distance(const geometry::types_geographic::point_type &a, const geometry::types_geographic::point_type &b);

    template
    typename boost::geometry::default_distance_result<geometry::types_cartesian::point_type, geometry::types_cartesian::point_type>::type
    point_distance(const geometry::types_cartesian::point_type &a, const geometry::types_cartesian::point_type &b);

    void clear_distance_cache() {
        _distance_cache.reset(new _distance_cache_type{});
    }

    template<typename Point>
    typename boost::geometry::coordinate_type<Point>::type
    azimuth_rad(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if (not _azimuth_cache.get()) {
            _azimuth_cache.reset(new _azimuth_cache_type{});
        }

        const auto key = _cache_key_type{
                boost::geometry::get<0>(a), boost::geometry::get<1>(a),
                boost::geometry::get<0>(b), boost::geometry::get<1>(b)};
        const auto search = _azimuth_cache->find(key);
        const bool exists = search != _azimuth_cache->end();
        if (exists) {
            return search->second;
        } else {
            const auto azimuth = boost::geometry::detail::azimuth<
                    typename boost::geometry::coordinate_type<Point>::type>(a, b);
            _azimuth_cache->emplace(key, azimuth);
            return azimuth;
        }
    }

    template
    typename boost::geometry::coordinate_type<geometry::types_geographic::point_type>::type
    azimuth_rad(const geometry::types_geographic::point_type &a, const geometry::types_geographic::point_type &b);

    template
    typename boost::geometry::coordinate_type<geometry::types_cartesian::point_type>::type
    azimuth_rad(const geometry::types_cartesian::point_type &a, const geometry::types_cartesian::point_type &b);

    void clear_azimuth_cache() {
        _azimuth_cache.reset(new _azimuth_cache_type{});
    }

    template<typename Point>
    boost::geometry::model::multi_polygon<boost::geometry::model::polygon<Point>>
    buffer(const Point &point, std::size_t buffer_points, double buffer_radius) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        using multi_polygon_type = boost::geometry::model::multi_polygon<boost::geometry::model::polygon<Point>>;
        using buffer_distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<
                typename boost::geometry::coordinate_type<Point>::type>;
        using buffer_side_strategy = boost::geometry::strategy::buffer::side_straight;
        using buffer_join_strategy = boost::geometry::strategy::buffer::join_round;
        using buffer_end_strategy = boost::geometry::strategy::buffer::end_round;

        const buffer_side_strategy buffer_side;
        const buffer_join_strategy buffer_join;
        const buffer_end_strategy buffer_end;

        auto current_buffer_radius = buffer_radius;

        multi_polygon_type buffer;
        while (buffer.empty()) {
            const buffer_distance_strategy buffer_distance{current_buffer_radius};

            if constexpr(std::is_same_v<typename boost::geometry::coordinate_system<Point>::type,
                    geometry::cs_geographic>) {
                using buffer_point_strategy = boost::geometry::strategy::buffer::geographic_point_circle<>;
                const buffer_point_strategy buffer_point{buffer_points};

                boost::geometry::buffer(point, buffer, buffer_distance, buffer_side, buffer_join, buffer_end,
                                        buffer_point);
            } else if constexpr(std::is_same_v<typename boost::geometry::coordinate_system<Point>::type,
                    geometry::cs_cartesian>) {
                using buffer_point_strategy = boost::geometry::strategy::buffer::point_circle;
                const buffer_point_strategy buffer_point{buffer_points};

                boost::geometry::buffer(point, buffer, buffer_distance, buffer_side, buffer_join, buffer_end,
                                        buffer_point);
            } else {
                static_assert(not std::is_same_v<typename boost::geometry::coordinate_system<Point>::type,
                        geometry::cs_geographic> and
                              not std::is_same_v<typename boost::geometry::coordinate_system<Point>::type,
                                      geometry::cs_cartesian>, "Invalid coordinate system");
            }

            current_buffer_radius *= 2;
        }

        return buffer;
    }

    template
    boost::geometry::model::multi_polygon<boost::geometry::model::polygon<geometry::types_geographic::point_type>>
    buffer(const geometry::types_geographic::point_type &point, std::size_t buffer_points, double buffer_radius);

    template
    boost::geometry::model::multi_polygon<boost::geometry::model::polygon<geometry::types_cartesian::point_type>>
    buffer(const geometry::types_cartesian::point_type &point, std::size_t buffer_points, double buffer_radius);

    template<typename Segment>
    boost::geometry::model::box<typename boost::geometry::point_type<Segment>::type>
    segment_buffer_box(const Segment &segment, std::size_t buffer_points, double buffer_radius) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using point_type = typename boost::geometry::point_type<Segment>::type;
        using box_type = boost::geometry::model::box<point_type>;
        using multi_polygon_type = boost::geometry::model::multi_polygon<boost::geometry::model::polygon<point_type>>;
        using coordinate_type = typename boost::geometry::coordinate_type<point_type>::type;
        using buffer_distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<
                typename boost::geometry::coordinate_type<point_type>::type>;
        using buffer_side_strategy = boost::geometry::strategy::buffer::side_straight;
        using buffer_join_strategy = boost::geometry::strategy::buffer::join_round;
        using buffer_end_strategy = boost::geometry::strategy::buffer::end_round;

        const buffer_side_strategy buffer_side;
        const buffer_join_strategy buffer_join;
        const buffer_end_strategy buffer_end;

        coordinate_type min_x = std::numeric_limits<coordinate_type>::infinity();
        coordinate_type min_y = std::numeric_limits<coordinate_type>::infinity();
        coordinate_type max_x = -std::numeric_limits<coordinate_type>::infinity();
        coordinate_type max_y = -std::numeric_limits<coordinate_type>::infinity();

        const auto coordinate_finder = [&](const point_type &point, coordinate_type &min_x, coordinate_type &min_y,
                                           coordinate_type &max_x, coordinate_type &max_y) {
            auto current_buffer_radius = buffer_radius;

            multi_polygon_type buffer;
            while (buffer.empty()) {
                const buffer_distance_strategy buffer_distance{current_buffer_radius};

                if constexpr(std::is_same_v<typename boost::geometry::coordinate_system<point_type>::type,
                        geometry::cs_geographic>) {
                    using buffer_point_strategy = boost::geometry::strategy::buffer::geographic_point_circle<>;
                    const buffer_point_strategy buffer_point{buffer_points};

                    boost::geometry::buffer(point, buffer, buffer_distance, buffer_side, buffer_join, buffer_end,
                                            buffer_point);
                } else if constexpr(std::is_same_v<typename boost::geometry::coordinate_system<point_type>::type,
                        geometry::cs_cartesian>) {
                    using buffer_point_strategy = boost::geometry::strategy::buffer::point_circle;
                    const buffer_point_strategy buffer_point{buffer_points};

                    boost::geometry::buffer(point, buffer, buffer_distance, buffer_side, buffer_join, buffer_end,
                                            buffer_point);
                } else {
                    static_assert(not std::is_same_v<typename boost::geometry::coordinate_system<point_type>::type,
                            geometry::cs_geographic> and
                                  not std::is_same_v<typename boost::geometry::coordinate_system<point_type>::type,
                                          geometry::cs_cartesian>, "Invalid coordinate system");
                }

                current_buffer_radius *= 2;
            }

            box_type box;
            boost::geometry::envelope(buffer, box);

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

    template
    boost::geometry::model::box<typename boost::geometry::point_type<geometry::types_geographic::segment_type>::type>
    segment_buffer_box(const geometry::types_geographic::segment_type &segment,
                       std::size_t buffer_points, double buffer_radius);

    template
    boost::geometry::model::box<typename boost::geometry::point_type<geometry::types_cartesian::segment_type>::type>
    segment_buffer_box(const geometry::types_cartesian::segment_type &segment,
                       std::size_t buffer_points, double buffer_radius);

    template<template<typename> class Container, typename Line>
    Container<boost::geometry::model::segment<typename boost::geometry::point_type<Line>::type>>
    line2segments(const Line &line) {
        using segment_type = boost::geometry::model::segment<typename boost::geometry::point_type<Line>::type>;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<segment_type>));

        Container<segment_type> segments;
        if (line.size() >= 2) {
            if constexpr(std::is_same_v<Container<segment_type>, std::vector<segment_type>>) {
                segments.reserve(line.size() - 1);
            }
            for (std::size_t i = 0; i < line.size() - 1; ++i) {
                segments.emplace_back(segment_type{line[i], line[i + 1]});
            }
        }
        return segments;
    }

    template
    std::vector<boost::geometry::model::segment<typename boost::geometry::point_type<geometry::types_geographic::line_type>::type>>
    line2segments(const geometry::types_geographic::line_type &line);

    template
    std::list<boost::geometry::model::segment<typename boost::geometry::point_type<geometry::types_geographic::line_type>::type>>
    line2segments(const geometry::types_geographic::line_type &line);

    template
    std::vector<boost::geometry::model::segment<typename boost::geometry::point_type<geometry::types_cartesian::line_type>::type>>
    line2segments(const geometry::types_cartesian::line_type &line);

    template
    std::list<boost::geometry::model::segment<typename boost::geometry::point_type<geometry::types_cartesian::line_type>::type>>
    line2segments(const geometry::types_cartesian::line_type &line);

    template<typename Segment, template<typename> class Container>
    boost::geometry::model::linestring<typename boost::geometry::point_type<Segment>::type>
    segments2line(const Container<Segment> &segments) {
        using line_type = boost::geometry::model::linestring<typename boost::geometry::point_type<Segment>::type>;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

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

    template
    boost::geometry::model::linestring<typename boost::geometry::point_type<geometry::types_geographic::segment_type>::type>
    segments2line(const std::vector<geometry::types_geographic::segment_type> &segments);

    template
    boost::geometry::model::linestring<typename boost::geometry::point_type<geometry::types_geographic::segment_type>::type>
    segments2line(const std::list<geometry::types_geographic::segment_type> &segments);

    template
    boost::geometry::model::linestring<typename boost::geometry::point_type<geometry::types_cartesian::segment_type>::type>
    segments2line(const std::vector<geometry::types_cartesian::segment_type> &segments);

    template
    boost::geometry::model::linestring<typename boost::geometry::point_type<geometry::types_cartesian::segment_type>::type>
    segments2line(const std::list<geometry::types_cartesian::segment_type> &segments);

    template<template<typename> class Container, typename Segment>
    Container<typename boost::geometry::default_length_result<Segment>::type>
    segments_lengths(const Container<Segment> &segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using length_type = typename boost::geometry::default_length_result<Segment>::type;

        Container<length_type> lengths;
        if constexpr(std::is_same_v<Container<length_type>, std::vector<length_type>>) {
            lengths.reserve(segments.size());
        }
        for (const auto &segment: segments) {
            lengths.emplace_back(geometry::length_segment(segment));
        }
        return lengths;
    }

    template
    std::vector<typename boost::geometry::default_length_result<geometry::types_geographic::segment_type>::type>
    segments_lengths(const std::vector<geometry::types_geographic::segment_type> &segments);

    template
    std::vector<typename boost::geometry::default_length_result<geometry::types_cartesian::segment_type>::type>
    segments_lengths(const std::vector<geometry::types_cartesian::segment_type> &segments);

    template<template<typename> class Container, typename Segment>
    Container<typename boost::geometry::coordinate_type<Segment>::type>
    segments_azimuths_deg(const Container<Segment> &segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using angle_type = typename boost::geometry::coordinate_type<Segment>::type;

        Container<angle_type> azimuths;
        if constexpr(std::is_same_v<Container<angle_type>, std::vector<angle_type>>) {
            azimuths.reserve(segments.size());
        }
        for (const auto &segment: segments) {
            azimuths.emplace_back(azimuth_segment_deg(segment));
        }
        return azimuths;
    }

    template
    std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type>
    segments_azimuths_deg(const std::vector<geometry::types_geographic::segment_type> &segments);

    template
    std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type>
    segments_azimuths_deg(const std::vector<geometry::types_cartesian::segment_type> &segments);

    template<template<typename> class Container, typename Segment>
    Container<typename boost::geometry::coordinate_type<Segment>::type>
    segments_directions_deg(const Container<Segment> &segments,
                            const Container<typename boost::geometry::coordinate_type<Segment>::type> &segments_azimuths) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename boost::geometry::coordinate_type<Segment>::type;

        Container<angle_type> directions;
        if constexpr(std::is_same_v<Container<angle_type>, std::vector<angle_type>>) {
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

    template
    std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type>
    segments_directions_deg(const std::vector<geometry::types_geographic::segment_type> &segments,
                            const std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type> &segments_azimuths);

    template
    std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type>
    segments_directions_deg(const std::vector<geometry::types_cartesian::segment_type> &segments,
                            const std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type> &segments_azimuths);

    template<template<typename> class Container, typename Segment>
    Container<typename boost::geometry::coordinate_type<Segment>::type>
    segments_absolute_directions_deg(const Container<Segment> &segments,
                                     const Container<typename boost::geometry::coordinate_type<Segment>::type> &segments_azimuths) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename boost::geometry::coordinate_type<Segment>::type;

        Container<angle_type> directions;
        if constexpr(std::is_same_v<Container<angle_type>, std::vector<angle_type>>) {
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

    template
    std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type>
    segments_absolute_directions_deg(const std::vector<geometry::types_geographic::segment_type> &segments,
                                     const std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type> &segments_azimuths);

    template
    std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type>
    segments_absolute_directions_deg(const std::vector<geometry::types_cartesian::segment_type> &segments,
                                     const std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type> &segments_azimuths);

    template<template<typename> class Container, typename Segment>
    std::pair<Container<typename boost::geometry::coordinate_type<Segment>::type>,
            Container<typename boost::geometry::coordinate_type<Segment>::type>>
    segments_combined_directions_deg(const Container<Segment> &segments,
                                     const Container<typename boost::geometry::coordinate_type<Segment>::type> &segments_azimuths) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename boost::geometry::coordinate_type<Segment>::type;

        Container<angle_type> directions;
        Container<angle_type> absolute_directions;
        if constexpr(std::is_same_v<Container<angle_type>, std::vector<angle_type>>) {
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

    template
    std::pair<std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type>,
            std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type>>
    segments_combined_directions_deg(const std::vector<geometry::types_geographic::segment_type> &segments,
                                     const std::vector<typename boost::geometry::coordinate_type<geometry::types_geographic::segment_type>::type> &segments_azimuths);

    template
    std::pair<std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type>,
            std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type>>
    segments_combined_directions_deg(const std::vector<geometry::types_cartesian::segment_type> &segments,
                                     const std::vector<typename boost::geometry::coordinate_type<geometry::types_cartesian::segment_type>::type> &segments_azimuths);

    template<typename Line>
    bool lines_are_reversed(const Line &a, const Line &b) {
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

    template
    bool lines_are_reversed(const boost::geometry::model::linestring<geometry::types_geographic::point_type> &a,
                            const boost::geometry::model::linestring<geometry::types_geographic::point_type> &b);

    template
    bool lines_are_reversed(const boost::geometry::model::linestring<geometry::types_cartesian::point_type> &a,
                            const boost::geometry::model::linestring<geometry::types_cartesian::point_type> &b);

    template<template<typename> class Container, typename Point>
    Point median_point(const Container<Point> &points) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        assert(not points.empty());
        assert(boost::geometry::dimension<Point>::value >= 1 and boost::geometry::dimension<Point>::value <= 3);

        using coordinate_type = typename boost::geometry::coordinate_type<Point>::type;

        std::vector<coordinate_type> x, y, z;
        if constexpr(boost::geometry::dimension<Point>::value >= 1) {
            x.reserve(points.size());
        }
        if constexpr(boost::geometry::dimension<Point>::value >= 2) {
            y.reserve(points.size());
        }
        if constexpr(boost::geometry::dimension<Point>::value == 3) {
            z.reserve(points.size());
        }

        if constexpr(boost::geometry::dimension<Point>::value == 1) {
            for (const auto &point: points) {
                x.emplace_back(boost::geometry::get<0>(point));
            }
        } else if constexpr(boost::geometry::dimension<Point>::value == 2) {
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
        if constexpr(boost::geometry::dimension<Point>::value >= 1) {
            median_x = median(x);
        }
        if constexpr(boost::geometry::dimension<Point>::value >= 2) {
            median_y = median(y);
        }
        if constexpr(boost::geometry::dimension<Point>::value == 3) {
            median_z = median(z);
        }

        if constexpr(boost::geometry::dimension<Point>::value == 1) {
            return Point{median_x};
        } else if constexpr(boost::geometry::dimension<Point>::value == 2) {
            return Point{median_x, median_y};
        } else {
            return Point{median_x, median_y, median_z};
        }
    }

    template
    geometry::types_geographic::point_type
    median_point(const std::vector<geometry::types_geographic::point_type> &points);

    template
    geometry::types_cartesian::point_type
    median_point(const std::vector<geometry::types_cartesian::point_type> &points);

    template
    geometry::types_geographic::point_type
    median_point(const std::list<geometry::types_geographic::point_type> &points);

    template
    geometry::types_cartesian::point_type
    median_point(const std::list<geometry::types_cartesian::point_type> &points);

}
