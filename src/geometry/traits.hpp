// Copyright (C) 2022 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TRAITS_HPP
#define MAP_MATCHING_2_TRAITS_HPP

#include <type_traits>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace map_matching_2::geometry {

    template<typename Geometry>
    using point_type_t = typename boost::geometry::point_type<Geometry>::type;

    template<typename Point, typename = void>
    struct models {
    };

    template<typename Point>
    struct models<Point, std::enable_if_t<std::is_same_v<typename boost::geometry::tag<Point>::type, boost::geometry::point_tag>>> {
        using point_type = Point;
        using multi_point_type = boost::geometry::model::multi_point<point_type>;
        using line_type = boost::geometry::model::linestring<point_type>;
        using multi_line_type = boost::geometry::model::multi_linestring<line_type>;
        using segment_type = boost::geometry::model::segment<point_type>;
        using referring_segment_type = boost::geometry::model::referring_segment<const point_type>;
        using pointing_segment_type = boost::geometry::model::pointing_segment<const point_type>;
        using box_type = boost::geometry::model::box<point_type>;
        using polygon_type = boost::geometry::model::polygon<point_type>;
        using multi_polygon_type = boost::geometry::model::multi_polygon<polygon_type>;
    };

    template<typename Geometry>
    struct data {
        using geometry_type = Geometry;
        using coordinate_system_type = typename boost::geometry::coordinate_system<geometry_type>::type;
        using coordinate_type = typename boost::geometry::coordinate_type<geometry_type>::type;
        using distance_type = typename boost::geometry::default_distance_result<geometry_type, geometry_type>::type;
        using length_type = typename boost::geometry::default_length_result<geometry_type>::type;
        using angle_type = typename boost::geometry::coordinate_type<geometry_type>::type;
    };

    template<typename RichSegment, typename = void>
    struct rich_segment_traits {
    };

    template<typename RichSegment>
    struct rich_segment_traits<RichSegment, std::void_t<typename RichSegment::segment_type>> {
        using rich_segment_type = RichSegment;
        using segment_type = typename rich_segment_type::segment_type;
        using point_type = point_type_t<segment_type>;
        using line_type = typename models<point_type>::line_type;
        using coordinate_system_type = typename data<segment_type>::coordinate_system_type;
        using coordinate_type = typename data<segment_type>::coordinate_type;
        using distance_type = typename data<point_type>::distance_type;
        using length_type = typename data<segment_type>::length_type;
        using angle_type = typename data<segment_type>::angle_type;
    };

    template<typename RichLine, typename = void>
    struct rich_line_traits {
    };

    template<typename RichLine>
    struct rich_line_traits<RichLine, std::void_t<typename RichLine::rich_segment_type>> {
        using rich_line_type = RichLine;
        using rich_segment_type = typename rich_line_type::rich_segment_type;
        using segment_type = typename rich_segment_traits<rich_segment_type>::segment_type;
        using point_type = typename rich_segment_traits<rich_segment_type>::point_type;
        using line_type = typename rich_segment_traits<rich_segment_type>::line_type;
        using coordinate_system_type = typename rich_segment_traits<rich_segment_type>::coordinate_system_type;
        using coordinate_type = typename rich_segment_traits<rich_segment_type>::coordinate_type;
        using distance_type = typename rich_segment_traits<rich_segment_type>::distance_type;
        using length_type = typename rich_segment_traits<rich_segment_type>::length_type;
        using angle_type = typename rich_segment_traits<rich_segment_type>::angle_type;
        using multi_line_type = typename models<point_type>::multi_line_type;
    };

    template<typename MultiRichLine, typename = void>
    struct multi_rich_line_traits {
    };

    template<typename MultiRichLine>
    struct multi_rich_line_traits<MultiRichLine, std::void_t<typename MultiRichLine::rich_line_type>> {
        using multi_rich_line_type = MultiRichLine;
        using rich_line_type = typename MultiRichLine::rich_line_type;
        using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using multi_line_type = typename rich_line_traits<rich_line_type>::multi_line_type;
        using coordinate_system_type = typename rich_line_traits<rich_line_type>::coordinate_system_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;
    };

    template<typename Route, typename = void>
    struct route_traits {
    };

    template<typename Route>
    struct route_traits<Route, std::void_t<typename Route::rich_line_type>> {
        using route_type = Route;
        using rich_line_type = typename Route::rich_line_type;
        using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_system_type = typename rich_line_traits<rich_line_type>::coordinate_system_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;
        using multi_line_type = typename rich_line_traits<rich_line_type>::multi_line_type;
    };

    template<typename Network, typename = void>
    struct network_traits {
    };

    template<typename Network>
    struct network_traits<Network, std::void_t<typename Network::graph_storage_type>> {
        using network_type = Network;
        using graph_storage_type = typename network_type::graph_storage_type;
        using graph_type = typename graph_storage_type::graph_type;
        using node_type = typename graph_storage_type::node_type;
        using edge_type = typename graph_storage_type::edge_type;
        using graph_traits = typename boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;
        using rich_line_type = typename edge_type::rich_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
    };

    template<typename>
    inline constexpr bool dependent_false_v = false;

    template<typename RichSegment, typename = void>
    struct is_rich_segment : public std::false_type {
    };

    template<typename RichSegment>
    struct is_rich_segment<RichSegment,
            std::void_t<decltype(&RichSegment::segment),
                    decltype(&RichSegment::has_length), decltype(&RichSegment::length),
                    decltype(&RichSegment::has_azimuth), decltype(&RichSegment::azimuth)>>
            : public std::true_type {
    };

    template<typename Segment, typename = void>
    struct is_segment : public std::false_type {
    };

    template<typename Segment>
    struct is_segment<Segment,
            std::enable_if_t<std::is_same_v<Segment, typename models<point_type_t<Segment>>::segment_type>>>
            : public std::true_type {
    };

    template<typename Segment, typename Enable = void>
    inline constexpr bool is_segment_v = is_segment<Segment, Enable>::value;

    template<typename Segment, typename = void>
    struct is_referring_segment : public std::false_type {
    };

    template<typename Segment>
    struct is_referring_segment<Segment,
            std::enable_if_t<std::is_same_v<Segment, typename models<point_type_t<Segment>>::referring_segment_type>>>
            : public std::true_type {
    };

    template<typename Segment, typename Enable = void>
    inline constexpr bool is_referring_segment_v = is_referring_segment<Segment, Enable>::value;

    template<typename Segment, typename = void>
    struct is_pointing_segment : public std::false_type {
    };

    template<typename Segment>
    struct is_pointing_segment<Segment,
            std::enable_if_t<std::is_same_v<Segment, typename models<point_type_t<Segment>>::pointing_segment_type>>>
            : public std::true_type {
    };

    template<typename Segment, typename Enable = void>
    inline constexpr bool is_pointing_segment_v = is_pointing_segment<Segment, Enable>::value;

    template<typename RichLine, typename = void>
    struct is_rich_line : public std::false_type {
    };

    template<typename RichLine>
    struct is_rich_line<RichLine,
            std::void_t<decltype(&RichLine::rich_segments),
                    decltype(&RichLine::has_length), decltype(&RichLine::length),
                    decltype(&RichLine::has_azimuth), decltype(&RichLine::azimuth),
                    decltype(&RichLine::has_directions), decltype(&RichLine::directions),
                    decltype(&RichLine::absolute_directions)>>
            : public std::true_type {
    };

    template<typename RichLine, typename Enable = void>
    inline constexpr bool is_rich_line_v = is_rich_line<RichLine, Enable>::value;

    template<typename RichLine, typename = void>
    struct is_multi_rich_line : public std::false_type {
    };

    template<typename RichLine>
    struct is_multi_rich_line<RichLine,
            std::void_t<decltype(&RichLine::rich_lines),
                    decltype(&RichLine::has_length), decltype(&RichLine::length),
                    decltype(&RichLine::has_azimuth), decltype(&RichLine::azimuth),
                    decltype(&RichLine::has_directions), decltype(&RichLine::directions),
                    decltype(&RichLine::absolute_directions)>>
            : public std::true_type {
    };

    template<typename RichLine, typename Enable = void>
    inline constexpr bool is_multi_rich_line_v = is_multi_rich_line<RichLine, Enable>::value;

}

#endif //MAP_MATCHING_2_TRAITS_HPP
