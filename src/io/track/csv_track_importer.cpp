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

#include "csv_track_importer.hpp"

#include <geometry/track/types.hpp>

namespace map_matching_2::io::track {

    template<typename MultiTrack>
    csv_track_importer<MultiTrack>::csv_track_importer(std::string filename,
                                                       std::unordered_map<std::string, MultiTrack> &tracks)
            : csv_importer{std::move(filename)}, _tracks{tracks} {}

    template<typename MultiTrack>
    void csv_track_importer<MultiTrack>::configure_format(csv::CSVFormat &format) {
        if (settings.no_header) {
            format.no_header();
        }
        format.delimiter(settings.delimiter);
    }

    template<typename MultiTrack>
    void csv_track_importer<MultiTrack>::process_row(const std::size_t n, const csv::CSVRow &row) {
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

    template<typename MultiTrack>
    void csv_track_importer<MultiTrack>::finish_import() {
        const auto timestamp_comparator = [](const auto &left, const auto &right) {
            return left.timestamp < right.timestamp;
        };

        for (auto &measurement_pair: _measurements) {
            const auto &id = measurement_pair.first;
            auto &measurements = measurement_pair.second;
            std::sort(measurements.begin(), measurements.end(), timestamp_comparator);
            _tracks.emplace(id, MultiTrack{id, std::move(measurements)});
        }
    }

    template<typename MultiTrack>
    bool csv_track_importer<MultiTrack>::_is_number(const std::string &str) {
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

    template
    class csv_track_importer<geometry::track::types_geographic::multi_track_type>;

    template
    class csv_track_importer<geometry::track::types_cartesian::multi_track_type>;

}