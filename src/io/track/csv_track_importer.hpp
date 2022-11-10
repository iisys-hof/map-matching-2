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

#ifndef MAP_MATCHING_2_CSV_TRACK_IMPORTER_HPP
#define MAP_MATCHING_2_CSV_TRACK_IMPORTER_HPP

#include "../csv_importer.hpp"

#include <boost/serialization/serialization.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <absl/container/flat_hash_map.h>

namespace map_matching_2::io::track {

    struct csv_track_importer_settings {

        char delimiter = ',';
        bool no_header = false;
        bool no_id = false;
        bool wkt = false;
        bool no_parse_time = false;
        std::vector<std::string> field_id = {"id"};
        std::string id_aggregator = "_";
        std::string field_x = "x";
        std::string field_y = "y";
        std::string field_time = "time";
        std::string time_format = "%FT%T%Oz";
        std::string field_geometry = "geometry";

    };

    template<typename MultiTrack>
    class csv_track_importer : public csv_importer {

    public:
        using measurement_type = typename MultiTrack::measurement_type;
        using point_type = typename MultiTrack::point_type;
        using line_type = typename MultiTrack::line_type;
        using multi_line_type = typename MultiTrack::multi_line_type;

        csv_track_importer_settings settings;

        csv_track_importer(std::string filename, absl::flat_hash_map<std::string, MultiTrack> &tracks)
                : csv_importer{std::move(filename)}, _tracks{tracks} {}

    protected:
        absl::flat_hash_map<std::string, MultiTrack> &_tracks;

        void configure_format(csv::CSVFormat &format) override {
            if (settings.no_header) {
                format.no_header();
            }
            format.delimiter(settings.delimiter);
        }

        void process_row(std::size_t n, const csv::CSVRow &row) override {
            using coordinate_type = typename boost::geometry::coordinate_type<point_type>::type;

            std::string id;
            if (settings.no_id) {
                id = "0";
            } else {
                for (const auto &_field_id: settings.field_id) {
                    if (not id.empty()) {
                        id.append(settings.id_aggregator);
                    }
                    if (settings.no_header and _is_number(_field_id)) {
                        id.append(row[std::stoul(_field_id)].get<std::string>());
                    } else {
                        id.append(row[_field_id].get<std::string>());
                    }
                }
            }

            std::uint64_t timestamp = 0;
            std::string time;
            if (settings.no_header and _is_number(settings.field_time)) {
                time = row[std::stoul(settings.field_time)].get<std::string>();
            } else {
                const auto &columns = row.get_col_names();
                if (std::find(columns.cbegin(), columns.cend(), settings.field_time) != columns.cend()) {
                    time = row[settings.field_time].get<std::string>();
                }
            }

            if (not time.empty()) {
                if (settings.no_parse_time) {
                    if (_is_number(time)) {
                        timestamp = std::stoul(time);
                    } else if (_measurements.contains(id)) {
                        timestamp = _measurements[id].size();
                    }
                } else {
                    timestamp = this->parse_time(time, settings.time_format);
                }
            } else if (_measurements.contains(id)) {
                timestamp = _measurements[id].size();
            }

            if (settings.wkt) {
                std::string wkt_string;
                if (settings.no_header and _is_number(settings.field_geometry)) {
                    wkt_string = row[std::stoul(settings.field_geometry)].get<std::string>();
                } else {
                    wkt_string = row[settings.field_geometry].get<std::string>();
                }
                boost::to_upper(wkt_string);
                if (wkt_string.starts_with("POINT")) {
                    point_type point;
                    boost::geometry::read_wkt(wkt_string, point);

                    if (!_measurements.contains(id)) {
                        _measurements.emplace(id, std::vector<measurement_type>{});
                    }
                    _measurements[id].emplace_back(measurement_type{timestamp, std::move(point)});
                } else if (wkt_string.starts_with("LINESTRING")) {
                    if (settings.no_id) {
                        id = std::to_string(n);
                    }
                    line_type line;
                    boost::geometry::read_wkt(wkt_string, line);
                    _tracks.emplace(id, MultiTrack{id, std::move(line)});
                } else if (wkt_string.starts_with("MULTILINESTRING")) {
                    if (settings.no_id) {
                        id = std::to_string(n);
                    }
                    multi_line_type multi_line;
                    boost::geometry::read_wkt(wkt_string, multi_line);
                    _tracks.emplace(id, MultiTrack{id, std::move(multi_line)});
                } else if (wkt_string.starts_with("GEOMETRYCOLLECTION EMPTY")) {
                    if (settings.no_id) {
                        id = std::to_string(n);
                    }
                    _tracks.emplace(id, MultiTrack{id, line_type{}});
                } else {
                    std::clog << "Could not parse the following track with id "
                              << id << ", skipping ...\n" << wkt_string << std::endl;
                }
            } else {
                coordinate_type x, y;
                if (settings.no_header and _is_number(settings.field_x)) {
                    x = row[std::stoul(settings.field_x)].get<coordinate_type>();
                } else {
                    x = row[settings.field_x].get<coordinate_type>();
                }
                if (settings.no_header and _is_number(settings.field_y)) {
                    y = row[std::stoul(settings.field_y)].get<coordinate_type>();
                } else {
                    y = row[settings.field_y].get<coordinate_type>();
                }

                if (!_measurements.contains(id)) {
                    _measurements.emplace(id, std::vector<measurement_type>{});
                }
                _measurements[id].emplace_back(measurement_type{timestamp, point_type{x, y}});
            }
        }

        void finish_import() override {
            const auto timestamp_comparator = [](const auto &left, const auto &right) {
                return left.timestamp < right.timestamp;
            };

            for (auto &measurement_pair: _measurements) {
                const auto &id = measurement_pair.first;
                auto &measurements = measurement_pair.second;
                std::sort(measurements.begin(), measurements.end(), timestamp_comparator);
                _tracks.emplace(id, MultiTrack{id, measurement_type::template measurements2line<line_type>(
                        std::move(measurements))});
            }
        }

    private:
        absl::flat_hash_map<std::string, std::vector<measurement_type>> _measurements;

        [[nodiscard]] bool _is_number(const std::string &str) {
            if (not str.empty()) {
                for (char c: str) {
                    if (not std::isdigit(c)) {
                        return false;
                    }
                }
                return true;
            }
            return false;
        }

    };

}

#endif //MAP_MATCHING_2_CSV_TRACK_IMPORTER_HPP
