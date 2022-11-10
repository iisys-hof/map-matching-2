// Copyright (C) 2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GPX_TRACK_IMPORTER_HPP
#define MAP_MATCHING_2_GPX_TRACK_IMPORTER_HPP

#include <string>
#include <fstream>

#include <gpx/Parser.h>
#include <gpx/ReportCerr.h>

#include <absl/container/flat_hash_map.h>

#include <boost/serialization/serialization.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename MultiTrack>
    class gpx_track_importer : public importer {

    protected:
        absl::flat_hash_map<std::string, MultiTrack> &_tracks;

    public:
        using measurement_type = typename MultiTrack::measurement_type;
        using point_type = typename MultiTrack::point_type;
        using line_type = typename MultiTrack::line_type;

        explicit gpx_track_importer(std::string filename, absl::flat_hash_map<std::string, MultiTrack> &tracks)
                : importer{std::move(filename)}, _tracks{tracks} {}

        void read() override {
            std::ifstream gpx;
            gpx.open(this->filename());

            if (gpx.is_open() and gpx.good()) {
                gpx::Parser parser(nullptr);
                gpx::GPX *root = parser.parse(gpx);

                if (root == nullptr) {
                    std::clog << "Parsing of gpx file '" << this->filename() << "' failed due to "
                              << parser.errorText() << " on line " << parser.errorLineNumber() << " and column "
                              << parser.errorColumnNumber() << ", skipping ...\n" << std::endl;
                } else {
                    using coordinate_type = typename boost::geometry::coordinate_type<point_type>::type;

                    const auto parse_track_segment = [this](gpx::TRKSeg *seg) {
                        const auto &trkpts = seg->trkpts().list();
                        std::vector<measurement_type> measurements;
                        for (const auto trkpt: trkpts) {
                            if (trkpt != nullptr) {
                                const auto time = trkpt->time().getValue();
                                std::uint64_t timestamp;
                                if (time.ends_with('Z')) {
                                    timestamp = this->parse_time(trkpt->time().getValue(), "%FT%TZ");
                                } else {
                                    timestamp = this->parse_time(trkpt->time().getValue(), "%FT%T%Oz");
                                }
                                const coordinate_type x = std::stod(trkpt->lon().getValue());
                                const coordinate_type y = std::stod(trkpt->lat().getValue());
                                measurements.emplace_back(measurement_type{timestamp, point_type{x, y}});
                            }
                        }

                        const auto timestamp_comparator = [](const auto &left, const auto &right) {
                            return left.timestamp < right.timestamp;
                        };
                        std::sort(measurements.begin(), measurements.end(), timestamp_comparator);

                        return measurement_type::template measurements2line<line_type>(measurements);
                    };

                    const auto &trks = root->trks().list();
                    for (const auto trk: trks) {
                        if (trk != nullptr) {
                            const auto &trksegs = trk->trksegs().list();

                            std::vector<line_type> lines;
                            lines.reserve(trksegs.size());
                            for (const auto seg: trksegs) {
                                if (seg != nullptr) {
                                    lines.emplace_back(parse_track_segment(seg));
                                }
                            }

                            std::string id_str = std::to_string(_tracks.size());
                            if (lines.size() <= 1) {
                                // Track
                                _tracks.emplace(id_str, MultiTrack{id_str, lines.front()});
                            } else {
                                // MultiTrack
                                _tracks.emplace(id_str, MultiTrack{id_str, lines});
                            }
                        }
                    }
                }
            } else {
                std::clog << "Could not open gpx file '" << this->filename() << "', skipping ...\n" << std::endl;
            }
        }

    };

}

#endif //MAP_MATCHING_2_GPX_TRACK_IMPORTER_HPP
