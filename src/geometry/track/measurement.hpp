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

#ifndef MAP_MATCHING_2_MEASUREMENT_HPP
#define MAP_MATCHING_2_MEASUREMENT_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace map_matching_2::geometry::track {

    template<typename Point>
    struct measurement {

        using point_type = Point;
        using line_type = boost::geometry::model::linestring<Point>;

        std::uint64_t timestamp;
        Point point;

        measurement(std::uint64_t timestamp, Point point);

        ~measurement() = default;

        measurement(const measurement &other) = default;

        measurement(measurement &&other) noexcept = default;

        measurement &operator=(const measurement &other) = default;

        measurement &operator=(measurement &&other) noexcept = default;

        static line_type measurements2line(const std::vector<measurement> &measurements);

        static std::vector<measurement> line2measurements(const line_type &line);

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row() const;

        [[nodiscard]] std::string str() const;

        template<typename T>
        friend std::ostream &operator<<(std::ostream &out, const measurement<T> &measurement);

    };

}

#endif //MAP_MATCHING_2_MEASUREMENT_HPP
