// Copyright (C) 2022-2024 Adrian Wöltche
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

#include "app/match.hpp"

#include "io/track/seattle_ground_truth_importer.hpp"

namespace map_matching_2::app {

    void _read_seattle(const match_data &data, const geometry::point_reprojector_variant &reprojector_variant) {
        const auto output_file = std::filesystem::path{data.match_output.output} / data.match_output.filename;
        const auto output_filename = output_file.string();

        io::track::track_exporter track_exporter{output_filename, data.console.console, global.verbose};
        io::track::seattle_ground_truth_importer seattle_ground_truth_importer{
                data.tracks.files, track_exporter, data.srs_tracks.srs_transform, reprojector_variant
        };
        seattle_ground_truth_importer.read();
    }

    io::track::filter::FILTER_METHOD get_filter_method(const std::string &method) {
        if (method == FILTER_METHOD_WITHIN) {
            return io::track::filter::FILTER_METHOD::WITHIN;
        } else if (method == FILTER_METHOD_INTERSECTS) {
            return io::track::filter::FILTER_METHOD::INTERSECTS;
        }
        return io::track::filter::FILTER_METHOD::NONE;
    }

}
