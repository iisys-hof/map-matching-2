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

#include "gpx_track_importer.hpp"

#include <fstream>

#include <geometry/track/types.hpp>

#include <gpx/Parser.h>
#include <gpx/ReportCerr.h>

namespace map_matching_2::io::track {

    template<typename MultiTrack>
    gpx_track_importer<MultiTrack>::gpx_track_importer(
            std::string filename, std::unordered_map<std::string, MultiTrack> &tracks)
            : importer{std::move(filename)}, _tracks{tracks} {}

    template<typename MultiTrack>
    void gpx_track_importer<MultiTrack>::read() {
        std::ifstream gpx;
        gpx.open(this->filename());

        if (gpx.is_open()) {
            gpx::ReportCerr cerr;
            gpx::Parser parser(&cerr);

            gpx::GPX *root = parser.parse(gpx);

            if (root == nullptr) {
                std::clog << "Parsing of gpx file '" << this->filename() << "' failed due to "
                          << parser.errorText() << " on line " << parser.errorLineNumber() << " and column "
                          << parser.errorColumnNumber() << "\n" << std::endl;
            } else {
                std::cout << "Version: " << root->version().getValue() << std::endl;
            }
        } else {
            std::clog << "Could not open gpx file '" << this->filename() << "', skipping ...\n" << std::endl;
        }
    }

    template
    class gpx_track_importer<geometry::track::types_geographic::multi_track_type>;

    template
    class gpx_track_importer<geometry::track::types_cartesian::multi_track_type>;

}
