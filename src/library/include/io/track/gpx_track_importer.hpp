// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_TRACK_GPX_TRACK_IMPORTER_HPP
#define MAP_MATCHING_2_IO_TRACK_GPX_TRACK_IMPORTER_HPP

#include <string>
#include <fstream>
#include <format>
#include <iostream>

#include <gpx/Parser.h>

#include <boost/unordered/unordered_flat_set.hpp>

#include "types/geometry/track/multi_track.hpp"
#include "types/geometry/reprojector.hpp"

#include "geometry/algorithm/convert.hpp"
#include "geometry/algorithm/srs_dispatch.hpp"

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename Forwarder>
    class gpx_track_importer : public importer {

    public:
        using forwarder_type = Forwarder;

        using multi_track_variant_type = typename forwarder_type::multi_track_variant_type;

        constexpr gpx_track_importer(std::vector<std::string> filenames, const std::vector<std::string> &selectors,
                const bool no_id, std::string time_format, const bool no_parse_time, forwarder_type &forwarder,
                const geometry::srs_transform &srs_transform,
                const geometry::point_reprojector_variant &reprojector_variant)
            : importer{std::move(filenames)},
            _forwarder{forwarder}, _srs_transform{srs_transform}, _reprojector_variant{reprojector_variant},
            _selectors{std::cbegin(selectors), std::cend(selectors)}, _no_id{no_id},
            _no_parse_time{no_parse_time}, _time_format{std::move(time_format)} {}

        void read() override {
            const auto &filenames = this->filenames();
            for (std::size_t i = 0; i < filenames.size(); ++i) {
                if (not std::filesystem::exists(filenames[i])) {
                    std::cerr << std::format("file '{}' does not exist, skipping ...", filenames[i]) << std::endl;
                    continue;
                }

                const auto &filename = filenames[i];

                std::ifstream gpx{filename, std::ios::in};
                if (gpx.is_open() and gpx.good()) {
                    _read_gpx(gpx, filename);
                } else {
                    std::clog << std::format("Could not open gpx file '{}', skipping ...\n", filename) << std::endl;
                }
            }
        }

    protected:
        forwarder_type &_forwarder;
        const geometry::srs_transform &_srs_transform;
        const geometry::point_reprojector_variant &_reprojector_variant;
        boost::unordered_flat_set<std::string> _selectors;

        bool _no_id, _no_parse_time;
        std::string _time_format;
        std::size_t _counter{0};

        void _read_gpx(std::istream &gpx, const std::string &filename) {
            gpx::Parser parser(nullptr);
            gpx::GPX *root = parser.parse(gpx);

            if (root == nullptr) {
                std::clog << std::format(
                        "Parsing of gpx file '{}' failed due to {} on line {} and column {}, skipping ...\n",
                        filename, parser.errorText(), parser.errorLineNumber(),
                        parser.errorColumnNumber()) << std::endl;
            } else {
                _parse_dispatch(root);
            }
        }

        void _parse_dispatch(gpx::GPX *root) {
            geometry::srs_dispatch(_srs_transform, [this, root]<typename InputCS, typename OutputCS>() {
                using point_type_out = geometry::point_type<OutputCS>;

                using multi_track_type = geometry::track::multi_track_type<point_type_out>;
                using multi_line_type_out = typename multi_track_type::multi_line_type;

                auto &trks = root->trks().list();
                for (auto trk : trks) {
                    if (trk != nullptr) {
                        auto &trksegs = trk->trksegs().list();

                        multi_line_type_out multi_line;
                        multi_line.reserve(trksegs.size());
                        for (auto seg : trksegs) {
                            if (seg != nullptr) {
                                multi_line.emplace_back(_parse_track_segment<InputCS, OutputCS>(seg));
                            }
                        }

                        std::string id_str;
                        if (_no_id) {
                            id_str = std::to_string(_counter++);
                        } else {
                            id_str = trk->name().getValue();
                            if (id_str.empty()) {
                                // no track name, try metadata name
                                id_str = root->metadata().name().getValue();
                            }
                            if (id_str.empty()) {
                                // still empty, use counter
                                id_str = std::to_string(_counter++);
                                std::clog << std::format("GPX track id is empty, using '{}' ...", id_str) << std::endl;
                            }
                        }

                        if (_selectors.empty() or _selectors.contains(id_str)) {
                            _forwarder.pass(multi_track_type{id_str, std::move(multi_line)});
                        }
                    }
                }
            });
        }

        template<typename InputCS, typename OutputCS>
        [[nodiscard]] typename geometry::track::multi_track_type<geometry::point_type<OutputCS>>::line_type
        _parse_track_segment(gpx::TRKSeg *seg) {
            using point_type_in = geometry::point_type<InputCS>;
            using point_type_out = geometry::point_type<OutputCS>;

            using coordinate_type_in = typename geometry::data<point_type_in>::coordinate_type;

            using multi_track_type = geometry::track::multi_track_type<point_type_out>;
            using measurement_type_out = typename multi_track_type::measurement_type;
            using line_type_out = typename multi_track_type::line_type;

            auto &trkpts = seg->trkpts().list();

            std::vector<measurement_type_out> measurements;
            measurements.reserve(trkpts.size());

            for (auto trkpt : trkpts) {
                if (trkpt != nullptr) {
                    const auto time = trkpt->time().getValue();
                    std::uint64_t timestamp = measurements.size();

                    if (not time.empty()) {
                        if (_no_parse_time) {
                            if (is_number(time)) {
                                timestamp = std::stoul(time);
                            }
                        } else {
                            timestamp = this->parse_time(time, _time_format);
                        }
                    }

                    const coordinate_type_in x = std::stod(trkpt->lon().getValue());
                    const coordinate_type_in y = std::stod(trkpt->lat().getValue());
                    measurements.emplace_back(measurement_type_out{
                            timestamp, point_type_in{x, y}, _reprojector_variant
                    });
                }
            }

            std::sort(std::begin(measurements), std::end(measurements));

            return geometry::measurements2line<measurement_type_out, line_type_out>(measurements);
        };

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_GPX_TRACK_IMPORTER_HPP
