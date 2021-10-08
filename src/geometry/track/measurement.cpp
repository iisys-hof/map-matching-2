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

#include "measurement.hpp"

#include "../util.hpp"
#include "../types.hpp"

namespace map_matching_2::geometry::track {

    template<typename Point>
    measurement<Point>::measurement(std::uint64_t timestamp, Point point)
            : timestamp{timestamp}, point{std::move(point)} {}


    template<typename Point>
    typename measurement<Point>::line_type
    measurement<Point>::measurements2line(const std::vector<measurement> &measurements) {
        line_type line;
        line.reserve(measurements.size());
        for (const measurement &measurement: measurements) {
            line.emplace_back(measurement.point);
        }
        return line;
    }

    template<typename Point>
    std::vector<measurement<Point>>
    measurement<Point>::line2measurements(const line_type &line) {
        std::vector<measurement> measurements;
        measurements.reserve(line.size());
        std::uint64_t _timestamp = 0;
        for (const point_type &_point: line) {
            measurements.emplace_back(measurement{_timestamp++, _point});
        }
        return measurements;
    }

    template<typename Point>
    std::string measurement<Point>::wkt() const {
        return geometry::to_wkt(point);
    }

    template<typename Point>
    std::vector<std::string> measurement<Point>::header() const {
        return {"timestamp", "geometry"};
    }

    template<typename Point>
    std::vector<std::string> measurement<Point>::row() const {
        std::vector<std::string> row;
        row.reserve(2);
        row.emplace_back(std::to_string(timestamp));
        row.emplace_back(wkt());
        return row;
    }

    template<typename Point>
    std::string measurement<Point>::str() const {
        const auto &fields = row();
        std::string str = "Measurement(";
        bool first = true;
        for (const auto &field: fields) {
            if (!first) {
                str.append(",");
            }
            first = false;
            str.append(field);
        }
        str.append(")");
        return str;
    }

    template<typename Point>
    std::ostream &operator<<(std::ostream &out, const measurement<Point> &measurement) {
        out << measurement.str();
        return out;
    }

    template
    class measurement<geometry::types_geographic::point_type>;

    template
    class measurement<geometry::types_cartesian::point_type>;

    template
    std::ostream &
    operator<<(std::ostream &out, const measurement <geometry::types_geographic::point_type> &measurement);

    template
    std::ostream &operator<<(std::ostream &out, const measurement <geometry::types_cartesian::point_type> &measurement);

}
