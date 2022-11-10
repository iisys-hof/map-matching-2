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

#include <boost/serialization/serialization.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "../util.hpp"

namespace map_matching_2::geometry::track {

    template<typename Point>
    struct measurement {

        using point_type = Point;

        std::uint64_t timestamp;
        point_type point;

        measurement(std::uint64_t timestamp, point_type point)
                : timestamp{timestamp}, point{std::move(point)} {}

        ~measurement() = default;

        measurement(const measurement &other) = default;

        measurement(measurement &&other) noexcept = default;

        measurement &operator=(const measurement &other) = default;

        measurement &operator=(measurement &&other) noexcept = default;

        template<typename Line>
        [[nodiscard]] static Line measurements2line(const std::vector<measurement> &measurements) {
            BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));

            Line line;
            line.reserve(measurements.size());
            for (const measurement &measurement: measurements) {
                line.emplace_back(measurement.point);
            }
            return line;
        }

        template<typename Line>
        [[nodiscard]] static std::vector<measurement> line2measurements(const Line &line) {
            BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));

            std::vector<measurement> measurements;
            measurements.reserve(line.size());
            std::uint64_t _timestamp = 0;
            for (const point_type &_point: line) {
                measurements.emplace_back(measurement{_timestamp++, _point});
            }
            return measurements;
        }

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(point);
        }

        [[nodiscard]] std::vector<std::string> header() const {
            return {"timestamp", "geometry"};
        }

        [[nodiscard]] std::vector<std::string> row() const {
            std::vector<std::string> row;
            row.reserve(2);
            row.emplace_back(std::to_string(timestamp));
            row.emplace_back(wkt());
            return row;
        }

        [[nodiscard]] std::string str() const {
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


        friend std::ostream &operator<<(std::ostream &out, const measurement &measurement) {
            return out << measurement.str();
        }

    };

}

#endif //MAP_MATCHING_2_MEASUREMENT_HPP
