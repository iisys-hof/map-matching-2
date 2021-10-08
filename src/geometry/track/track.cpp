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

#include "track.hpp"

#include "../network/types.hpp"
#include "../types.hpp"
#include "../util.hpp"

namespace map_matching_2::geometry::track {


    template<typename Measurement>
    track<Measurement>::track() : track{"", {}, {}} {}

    template<typename Measurement>
    track<Measurement>::track(std::string id, line_type line)
            : rich_line_type{std::move(line)}, id{std::move(id)},
              measurements{Measurement::line2measurements(this->line)} {}

    template<typename Measurement>
    track<Measurement>::track(std::string id, rich_line_type rich_line)
            : rich_line_type{std::move(rich_line)}, id{std::move(id)},
              measurements{Measurement::line2measurements(this->line)} {}

    template<typename Measurement>
    track<Measurement>::track(std::string id, std::vector<measurement_type> measurements)
            : rich_line_type{Measurement::measurements2line(measurements)}, id{std::move(id)},
              measurements{std::move(measurements)} {}

    template<typename Measurement>
    track<Measurement>::track(std::string id, line_type line, std::vector<measurement_type> measurements)
            : rich_line_type{std::move(line)}, id{std::move(id)}, measurements{std::move(measurements)} {
        assert(this->line.size() == this->measurements.size());
    }

    template<typename Measurement>
    track<Measurement>
    track<Measurement>::thin_out(const std::set<std::size_t> &indices_to_remove) const {
        line_type new_line;
        new_line.reserve(this->line.size() - indices_to_remove.size());
        std::vector<measurement_type> new_measurements;
        new_measurements.reserve(measurements.size() - indices_to_remove.size());

        for (std::size_t i = 0; i < measurements.size(); ++i) {
            if (not indices_to_remove.contains(i)) {
                new_line.emplace_back(this->line[i]);
                new_measurements.emplace_back(measurements[i]);
            }
        }

        return track{id, new_line, new_measurements};
    }

    template<typename Measurement>
    void track<Measurement>::_correct_measurements() {
        if (this->line.empty()) {
            measurements.clear();
        } else {
            std::uint64_t time = not measurements.empty() ? measurements.front().timestamp : 0;
            measurements.clear();
            measurements.reserve(this->line.size());
            for (const auto &point: this->line) {
                measurements.emplace_back(measurement_type{time++, point});
            }

            // does not work because median might create new points
//            std::vector<measurement_type> new_measurements;
//            new_measurements.reserve(this->line.size());
//            std::size_t i = 0;
//            for (auto &measurement : measurements) {
//                if (geometry::equals_points(measurement.point, this->line[i])) {
//                    new_measurements.emplace_back(std::move(measurement));
//                    i++;
//                }
//
//                if (i == this->line.size()) {
//                    break;
//                }
//            }
//
//            measurements = new_measurements;
        }
    }

    template<typename Measurement>
    void
    track<Measurement>::simplify(const bool retain_reversals, const double tolerance, const double reverse_tolerance) {
        static_cast<rich_line_type *>(this)->simplify(retain_reversals, tolerance, reverse_tolerance);
        _correct_measurements();
    }

    template<typename Measurement>
    template<typename Network>
    void track<Measurement>::median_merge(const double tolerance, const bool adaptive, const Network *network) {
        static_cast<rich_line_type *>(this)->median_merge(tolerance, adaptive, network);
        _correct_measurements();
    }

    template<typename Measurement>
    std::vector<std::string> track<Measurement>::header() const {
        return {"id", "geometry", "measurements"};
    }

    template<typename Measurement>
    std::vector<std::string> track<Measurement>::row() const {
        std::vector<std::string> row;
        row.reserve(3);
        row.emplace_back(this->id);
        row.emplace_back(this->wkt());
        std::string measurements_str;
        measurements_str.append("[");
        bool first = true;
        for (const auto &measurement: this->measurements) {
            if (!first) {
                measurements_str.append(",");
            }
            first = false;
            measurements_str.append(measurement.str());
        }
        measurements_str.append("]");
        row.emplace_back(std::move(measurements_str));
        return row;
    }

    template<typename Measurement>
    std::string track<Measurement>::str() const {
        const auto &fields = row();
        std::string str = "Track(";
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

    template<typename Measurement>
    track<Measurement>
    track<Measurement>::create_track(
            const std::initializer_list<typename boost::geometry::coordinate_type<point_type>::type> coordinates) {
        std::vector<measurement_type> new_measurements;
        new_measurements.reserve(coordinates.size() / 2);
        line_type new_line;
        new_line.reserve(coordinates.size() / 2);
        std::uint64_t timestamp = 0;
        for (auto it = coordinates.begin(); it != coordinates.end(); ++it) {
            const auto x = *it;
            const auto y = *(++it);
            point_type point{x, y};
            new_measurements.emplace_back(measurement_type{timestamp++, point});
            new_line.emplace_back(std::move(point));
        }

        return track{"", std::move(new_line), std::move(new_measurements)};
    }

    template<typename Measurement>
    void track<Measurement>::sort_tracks(std::vector<track> &tracks, const bool ascending) {
        if (ascending) {
            std::sort(tracks.begin(), tracks.end(), [](const auto &left, const auto &right) {
                return left.measurements.size() < right.measurements.size();
            });
        } else {
            std::sort(tracks.begin(), tracks.end(), [](const auto &left, const auto &right) {
                return left.measurements.size() > right.measurements.size();
            });
        }
    }

    template<typename Measurement>
    std::ostream &operator<<(std::ostream &out, const track<Measurement> &track) {
        out << track.str();
        return out;
    }

    template
    class track<measurement<geometry::types_geographic::point_type>>;

    template
    class track<measurement<geometry::types_cartesian::point_type>>;

    template
    void track<measurement<geometry::types_geographic::point_type>>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_geographic::network_static *network);

    template
    void track<measurement<geometry::types_cartesian::point_type>>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_cartesian::network_static *network);

    template
    std::ostream &
    operator<<(std::ostream &out, const track <measurement<geometry::types_geographic::point_type>> &track);

    template
    std::ostream &
    operator<<(std::ostream &out, const track <measurement<geometry::types_cartesian::point_type>> &track);

}
