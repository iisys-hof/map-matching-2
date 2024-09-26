// Copyright (C) 2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_TRACK_TRACK_EXPORTER_HPP
#define MAP_MATCHING_2_IO_TRACK_TRACK_EXPORTER_HPP

#include <string>

#include "types/geometry/track/multi_track.hpp"

#include "io/results/track/csv_track_exporter.hpp"
#include "io/results/track/track_output_printer.hpp"

namespace map_matching_2::io::track {

    class track_exporter {

    public:
        using multi_track_variant_type = geometry::track::multi_track_variant_type;

        track_exporter(std::string filename, bool console, bool verbose);

        ~track_exporter();

        void pass(const multi_track_variant_type &multi_track);

    private:
        io::results::csv_track_exporter<> _exporter;
        io::results::track_output_printer _output_printer;

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_TRACK_EXPORTER_HPP
