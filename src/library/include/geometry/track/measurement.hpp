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

#ifndef MAP_MATCHING_2_GEOMETRY_TRACK_MEASUREMENT_HPP
#define MAP_MATCHING_2_GEOMETRY_TRACK_MEASUREMENT_HPP

#include "types/geometry/reprojector.hpp"

#include "geometry/types.hpp"

#include "geometry/algorithm/reproject.hpp"
#include "geometry/algorithm/wkt.hpp"

namespace map_matching_2::geometry::track {

    template<typename Point>
    struct measurement {

        using point_type = Point;

        std::uint64_t timestamp;
        point_type point;

        constexpr measurement(const std::uint64_t timestamp, point_type point)
            : timestamp{timestamp}, point{std::move(point)} {}

        template<typename InPoint>
        measurement(const std::uint64_t timestamp, InPoint input_point,
                const geometry::point_reprojector_variant &reprojector_variant)
            : timestamp{timestamp} {
            reproject_point(input_point, point, reprojector_variant);
        }

        constexpr measurement(const measurement &other) = default;

        constexpr measurement(measurement &&other) noexcept = default;

        constexpr measurement &operator=(const measurement &other) = default;

        constexpr measurement &operator=(measurement &&other) noexcept = default;

        constexpr ~measurement() = default;

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(point);
        }

        [[nodiscard]] static std::vector<std::string> header() {
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
            for (const auto &field : fields) {
                if (!first) {
                    str.append(",");
                }
                first = false;
                str.append(field);
            }
            str.append(")");
            return str;
        }

        friend constexpr bool operator==(const measurement &left, const measurement &right) {
            return left.timestamp == right.timestamp;
        }

        friend constexpr bool operator!=(const measurement &left, const measurement &right) {
            return not(left == right);
        }

        friend constexpr bool operator<(const measurement &left, const measurement &right) {
            return left.timestamp < right.timestamp;
        }

        friend constexpr bool operator<=(const measurement &left, const measurement &right) {
            return not(right < left);
        }

        friend constexpr bool operator>(const measurement &left, const measurement &right) {
            return right < left;
        }

        friend constexpr bool operator>=(const measurement &left, const measurement &right) {
            return not(left < right);
        }

        friend std::ostream &operator<<(std::ostream &out, const measurement &measurement) {
            return out << measurement.str();
        }

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_TRACK_MEASUREMENT_HPP
