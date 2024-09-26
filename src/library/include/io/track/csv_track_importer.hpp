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

#ifndef MAP_MATCHING_2_IO_TRACK_CSV_TRACK_IMPORTER_HPP
#define MAP_MATCHING_2_IO_TRACK_CSV_TRACK_IMPORTER_HPP

#include <any>

#include <boost/algorithm/string.hpp>

#include <boost/geometry/io/wkt/read.hpp>

#include <boost/unordered/unordered_flat_map.hpp>

#include "types/geometry/track/multi_track.hpp"
#include "types/geometry/reprojector.hpp"

#include "geometry/algorithm/convert.hpp"
#include "geometry/algorithm/srs_dispatch.hpp"

#include "../csv_importer.hpp"
#include "../csv_settings.hpp"

namespace map_matching_2::io::track {

    template<typename Forwarder>
    class csv_track_importer : public csv_importer {

    public:
        using forwarder_type = Forwarder;

        using multi_track_variant_type = typename forwarder_type::multi_track_variant_type;

        constexpr csv_track_importer(std::vector<std::string> filenames, std::vector<std::string> selectors,
                io::csv_settings csv_settings, forwarder_type &forwarder, const geometry::srs_transform &srs_transform,
                const geometry::point_reprojector_variant &reprojector_variant)
            : csv_importer{std::move(filenames), csv_settings.skip_lines},
            _forwarder{forwarder}, _srs_transform{srs_transform}, _reprojector_variant{reprojector_variant},
            _csv_settings{std::move(csv_settings)}, _selectors{std::move(selectors)} {}

    protected:
        forwarder_type &_forwarder;
        const geometry::srs_transform &_srs_transform;
        const geometry::point_reprojector_variant &_reprojector_variant;
        io::csv_settings _csv_settings;
        std::vector<std::string> _selectors;

        void configure_format(csv::CSVFormat &format) override {
            if (_csv_settings.no_header) {
                format.no_header();
            }
            format.delimiter(parse_delimiter(_csv_settings.delimiter));
        }

        void process_row(const std::size_t file_num, const std::size_t row_num, const csv::CSVRow &row) override {
            geometry::srs_dispatch(_srs_transform,
                    [this, file_num, row_num, &row]<typename InputCS, typename OutputCS>() {
                        using point_type_in = geometry::point_type<InputCS>;
                        using point_type_out = geometry::point_type<OutputCS>;

                        using coordinate_type_in = typename geometry::data<point_type_in>::coordinate_type;
                        using line_type_in = typename geometry::models<point_type_in>::template line_type<>;
                        using multi_line_type_in = typename geometry::models<point_type_in>::
                                template multi_line_type<line_type_in>;

                        using multi_track_type = geometry::track::multi_track_type<point_type_out>;
                        using measurement_type_out = typename multi_track_type::measurement_type;

                        std::string id;
                        if (_csv_settings.no_id) {
                            if (_csv_settings.no_id_whole_file) {
                                id = std::to_string(file_num);
                            } else {
                                id = std::to_string(row_num);
                            }
                        } else {
                            for (const auto &_field_id : _csv_settings.field_id) {
                                if (not id.empty()) {
                                    id.append(_csv_settings.id_aggregator);
                                }
                                if (_csv_settings.no_header and is_number(_field_id)) {
                                    id.append(row[std::stoul(_field_id)].get<std::string>());
                                } else {
                                    id.append(row[_field_id].get<std::string>());
                                }
                            }
                        }

                        if (_id_set and _csv_settings.grouped and id != _current_id) {
                            finish_import();
                        }

                        std::uint64_t timestamp = 0;
                        std::string time;
                        for (const auto &_field_time : _csv_settings.field_time) {
                            if (not time.empty()) {
                                time.append(_csv_settings.time_aggregator);
                            }
                            if (_csv_settings.no_header and is_number(_field_time)) {
                                time.append(row[std::stoul(_field_time)].get<std::string>());
                            } else {
                                const auto &columns = row.get_col_names();
                                if (std::find(columns.cbegin(), columns.cend(), _field_time) != columns.cend()) {
                                    time.append(row[_field_time].get<std::string>());
                                }
                            }
                        }

                        if (not time.empty()) {
                            if (_csv_settings.no_parse_time) {
                                if (is_number(time)) {
                                    timestamp = std::stoul(time);
                                } else if (_measurements.contains(id)) {
                                    const auto &measurements =
                                            std::any_cast<std::vector<measurement_type_out>>(_measurements[id]);
                                    timestamp = measurements.size();
                                }
                            } else {
                                timestamp = this->parse_time(time, _csv_settings.time_format);
                            }
                        } else if (_measurements.contains(id)) {
                            const auto &measurements =
                                    std::any_cast<std::vector<measurement_type_out>>(_measurements[id]);
                            timestamp = measurements.size();
                        }

                        if (_csv_settings.wkt) {
                            std::string wkt_string;
                            if (_csv_settings.no_header and is_number(_csv_settings.field_geometry)) {
                                wkt_string = row[std::stoul(_csv_settings.field_geometry)].get<std::string>();
                            } else {
                                wkt_string = row[_csv_settings.field_geometry].get<std::string>();
                            }
                            boost::to_upper(wkt_string);
                            if (wkt_string.starts_with("POINT")) {
                                point_type_in point;
                                boost::geometry::read_wkt(wkt_string, point);

                                if (not _measurements.contains(id)) {
                                    _measurements.emplace(id, std::vector<measurement_type_out>{});
                                }
                                auto &measurements = std::any_cast<std::vector<measurement_type_out> &>(
                                        _measurements[id]);
                                measurements.emplace_back(measurement_type_out{
                                        timestamp, std::move(point), _reprojector_variant
                                });
                            } else if (wkt_string.starts_with("LINESTRING")) {
                                if (_csv_settings.no_id) {
                                    id = std::to_string(row_num);
                                }
                                line_type_in line;
                                boost::geometry::read_wkt(wkt_string, line);
                                _forwarder.pass(multi_track_type{id, std::move(line), _reprojector_variant});
                            } else if (wkt_string.starts_with("MULTILINESTRING")) {
                                if (_csv_settings.no_id) {
                                    id = std::to_string(row_num);
                                }
                                multi_line_type_in multi_line;
                                boost::geometry::read_wkt(wkt_string, multi_line);
                                _forwarder.pass(multi_track_type{id, std::move(multi_line), _reprojector_variant});
                            } else if (wkt_string.starts_with("GEOMETRYCOLLECTION EMPTY")) {
                                if (_csv_settings.no_id) {
                                    id = std::to_string(row_num);
                                }
                                _forwarder.pass(multi_track_type{id, line_type_in{}, _reprojector_variant});
                            } else {
                                std::clog << "Could not parse the following track with id "
                                        << id << ", skipping ...\n" << wkt_string << std::endl;
                            }
                        } else {
                            coordinate_type_in x, y;
                            if (_csv_settings.no_header and is_number(_csv_settings.field_x)) {
                                x = row[std::stoul(_csv_settings.field_x)].get<coordinate_type_in>();
                            } else {
                                x = row[_csv_settings.field_x].get<coordinate_type_in>();
                            }
                            if (_csv_settings.no_header and is_number(_csv_settings.field_y)) {
                                y = row[std::stoul(_csv_settings.field_y)].get<coordinate_type_in>();
                            } else {
                                y = row[_csv_settings.field_y].get<coordinate_type_in>();
                            }

                            if (not _measurements.contains(id)) {
                                _measurements.emplace(id, std::vector<measurement_type_out>{});
                            }
                            auto &measurements = std::any_cast<std::vector<measurement_type_out> &>(_measurements[id]);
                            measurements.emplace_back(measurement_type_out{
                                    timestamp, point_type_in{x, y}, _reprojector_variant
                            });
                        }

                        _current_id = id;
                        _id_set = true;
                    });
        }

        void finish_import() override {
            geometry::srs_dispatch(_srs_transform, [this]<typename InputCS, typename OutputCS>() {
                using point_type_out = geometry::point_type<OutputCS>;

                using multi_track_type = geometry::track::multi_track_type<point_type_out>;
                using measurement_type_out = typename multi_track_type::measurement_type;
                using line_type_out = typename multi_track_type::line_type;

                for (auto &measurement_pair : _measurements) {
                    const auto &id = measurement_pair.first;
                    auto &measurements = std::any_cast<std::vector<measurement_type_out> &>(measurement_pair.second);
                    std::sort(std::begin(measurements), std::end(measurements));
                    _forwarder.pass(multi_track_type{
                            id,
                            geometry::measurements2line<measurement_type_out, line_type_out>(std::move(measurements))
                    });
                }
                _measurements.clear();
            });
        }

    private:
        std::string _current_id{};
        bool _id_set{false};
        boost::unordered::unordered_flat_map<std::string, std::any> _measurements{};

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_CSV_TRACK_IMPORTER_HPP
