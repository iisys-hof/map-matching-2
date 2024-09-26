// Copyright (C) 2022-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TESTS_HELPER_GEOMETRY_HELPER_HPP
#define MAP_MATCHING_2_TESTS_HELPER_GEOMETRY_HELPER_HPP

#include <initializer_list>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

template<typename Point, typename Line = boost::geometry::model::linestring<Point>>
Line add_to_line(Line &line, const std::initializer_list<Point> &points) {
    line.reserve(line.size() + points.size());
    for (const auto &point : points) {
        line.emplace_back(point);
    }

    return line;
}

template<typename Point, typename Line = boost::geometry::model::linestring<Point>>
Line add_to_line(Line &line,
        const std::initializer_list<typename boost::geometry::traits::coordinate_type<Point>::type> &coordinates) {
    assert(coordinates.size() % 2 == 0);

    line.reserve(line.size() + coordinates.size() / 2);
    for (auto it = coordinates.begin(); it != coordinates.end(); ++it) {
        const auto x = *it;
        const auto y = *(++it);
        line.emplace_back(Point{x, y});
    }

    return line;
}

template<typename Point, typename Line = boost::geometry::model::linestring<Point>>
Line create_line(const std::initializer_list<Point> &points) {
    return Line{points};
}

template<typename Point, typename Line = boost::geometry::model::linestring<Point>>
Line create_line(
        const std::initializer_list<typename boost::geometry::traits::coordinate_type<Point>::type> &coordinates) {
    assert(coordinates.size() % 2 == 0);

    Line line;
    return add_to_line<Point>(line, coordinates);
}

#endif //MAP_MATCHING_2_TESTS_HELPER_GEOMETRY_HELPER_HPP
