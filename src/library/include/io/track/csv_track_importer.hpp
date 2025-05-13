// Copyright (C) 2020-2025 Adrian Wöltche
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

#include <boost/unordered/unordered_flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

#include "types/geometry/track/multi_track.hpp"
#include "types/geometry/reprojector.hpp"

#include "geometry/algorithm/convert.hpp"
#include "geometry/algorithm/srs_dispatch.hpp"

#include "util/checks.hpp"
#include "util/chrono.hpp"

#include "../csv_importer.hpp"
#include "../csv_settings.hpp"

namespace map_matching_2::io::track {

    template<typename Forwarder>
    class csv_track_importer : public csv_importer {

    public:
        using forwarder_type = Forwarder;

        using multi_track_variant_type = typename forwarder_type::multi_track_variant_type;

        constexpr csv_track_importer(std::vector<std::string> filenames, const io::csv_settings &csv_settings,
                forwarder_type &forwarder, const geometry::srs_transform &srs_transform,
                const geometry::point_reprojector_variant &reprojector_variant)
            : csv_importer{std::move(filenames), csv_settings.skip_lines}, _forwarder{forwarder},
            _srs_transform{srs_transform}, _reprojector_variant{reprojector_variant}, _csv_settings{csv_settings},
            _selectors{std::cbegin(csv_settings.selectors), std::cend(csv_settings.selectors)} {}

    protected:
        forwarder_type &_forwarder;
        const geometry::srs_transform &_srs_transform;
        const geometry::point_reprojector_variant &_reprojector_variant;
        io::csv_settings _csv_settings;
        boost::unordered_flat_set<std::string> _selectors;

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
                        using point_type_out = geometry::time_point_type<OutputCS>;

                        using coordinate_type_in = typename geometry::data<point_type_in>::coordinate_type;
                        using line_type_in = typename geometry::models<point_type_in>::template line_type<>;
                        using multi_line_type_in = typename geometry::models<point_type_in>::
                                template multi_line_type<line_type_in>;

                        using multi_track_type = geometry::track::multi_track_type<point_type_out>;
                        using multi_line_type_out = typename multi_track_type::multi_line_type;
                        using line_type_out = typename multi_track_type::line_type;

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
                                if (_csv_settings.no_header and util::is_number(_field_id)) {
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
                            if (_csv_settings.no_header and util::is_number(_field_time)) {
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
                                if (util::is_number(time)) {
                                    timestamp = std::stoul(time);
                                } else if (_points.contains(id)) {
                                    const auto &out_line = std::any_cast<line_type_out>(_points[id]);
                                    timestamp = out_line.size();
                                }
                            } else {
                                timestamp = util::parse_time(time, _csv_settings.time_format);
                            }
                        } else if (_points.contains(id)) {
                            const auto &out_line = std::any_cast<line_type_out>(_points[id]);
                            timestamp = out_line.size();
                        }

                        if (_csv_settings.wkt) {
                            std::string wkt_string;
                            if (_csv_settings.no_header and util::is_number(_csv_settings.field_geometry)) {
                                wkt_string = row[std::stoul(_csv_settings.field_geometry)].get<std::string>();
                            } else {
                                wkt_string = row[_csv_settings.field_geometry].get<std::string>();
                            }
                            boost::to_upper(wkt_string);
                            if (wkt_string.starts_with("POINT")) {
                                point_type_in in_point;
                                boost::geometry::read_wkt(wkt_string, in_point);

                                if (not _points.contains(id)) {
                                    _points.emplace(id, line_type_out{});
                                }
                                auto &out_line = std::any_cast<line_type_out &>(_points[id]);
                                point_type_out out_point;
                                geometry::reproject_point(in_point, out_point, _reprojector_variant);
                                out_line.emplace_back(std::move(out_point), timestamp);
                            } else if (wkt_string.starts_with("LINESTRING")) {
                                if (_csv_settings.no_id) {
                                    id = std::to_string(row_num);
                                }
                                if (_selectors.empty() or _selectors.contains(id)) {
                                    line_type_in in_line;
                                    boost::geometry::read_wkt(wkt_string, in_line);
                                    _forwarder.pass(multi_track_type{id, std::move(in_line), _reprojector_variant});
                                }
                            } else if (wkt_string.starts_with("MULTILINESTRING")) {
                                if (_csv_settings.no_id) {
                                    id = std::to_string(row_num);
                                }
                                if (_selectors.empty() or _selectors.contains(id)) {
                                    multi_line_type_in multi_in_line;
                                    boost::geometry::read_wkt(wkt_string, multi_in_line);
                                    _forwarder.pass(
                                            multi_track_type{id, std::move(multi_in_line), _reprojector_variant});
                                }
                            } else if (wkt_string.starts_with("GEOMETRYCOLLECTION EMPTY")) {
                                if (_csv_settings.no_id) {
                                    id = std::to_string(row_num);
                                }
                                if (_selectors.empty() or _selectors.contains(id)) {
                                    _forwarder.pass(multi_track_type{id, line_type_in{}, _reprojector_variant});
                                }
                            } else {
                                std::clog << "Could not parse the following track with id "
                                        << id << ", skipping ...\n" << wkt_string << std::endl;
                            }
                        } else {
                            coordinate_type_in x, y;
                            if (_csv_settings.no_header and util::is_number(_csv_settings.field_x)) {
                                x = row[std::stoul(_csv_settings.field_x)].get<coordinate_type_in>();
                            } else {
                                x = row[_csv_settings.field_x].get<coordinate_type_in>();
                            }
                            if (_csv_settings.no_header and util::is_number(_csv_settings.field_y)) {
                                y = row[std::stoul(_csv_settings.field_y)].get<coordinate_type_in>();
                            } else {
                                y = row[_csv_settings.field_y].get<coordinate_type_in>();
                            }

                            if (not _points.contains(id)) {
                                _points.emplace(id, line_type_out{});
                            }
                            auto &out_line = std::any_cast<line_type_out &>(_points[id]);
                            point_type_out out_point;
                            geometry::reproject_point(point_type_in{x, y}, out_point, _reprojector_variant);
                            out_line.emplace_back(std::move(out_point), timestamp);
                        }

                        _current_id = id;
                        _id_set = true;
                    });
        }

        void finish_import() override {
            geometry::srs_dispatch(_srs_transform, [this]<typename InputCS, typename OutputCS>() {
                using point_type_out = geometry::time_point_type<OutputCS>;

                using multi_track_type = geometry::track::multi_track_type<point_type_out>;
                using line_type_out = typename multi_track_type::line_type;

                for (auto &pair : _points) {
                    const auto &id = pair.first;
                    if (_selectors.empty() or _selectors.contains(id)) {
                        auto &out_line = std::any_cast<line_type_out &>(pair.second);
                        std::stable_sort(std::begin(out_line), std::end(out_line));
                        _forwarder.pass(multi_track_type{id, std::move(out_line)});
                    }
                }
                _points.clear();
            });
        }

    private:
        std::string _current_id{};
        bool _id_set{false};
        boost::unordered::unordered_flat_map<std::string, std::any> _points{};

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_CSV_TRACK_IMPORTER_HPP
