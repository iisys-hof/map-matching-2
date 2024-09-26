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

#include "io/track/track_exporter.hpp"

namespace map_matching_2::io::track {

    track_exporter::track_exporter(std::string filename, const bool console, const bool verbose)
        : _exporter{std::move(filename)}, _output_printer{console, verbose} {
        _output_printer.start();
        _exporter.start();
    }

    track_exporter::~track_exporter() {
        _output_printer.stop();
        _exporter.stop();

        _output_printer.join();
        _exporter.join();
    }

    void track_exporter::pass(const multi_track_variant_type &multi_track) {
        std::visit([this]<typename MultiTrack>(MultiTrack &&_multi_track) {
            using multi_track_type = std::remove_reference_t<MultiTrack>;

            _output_printer.pass(_output_printer.parse_result<multi_track_type>(_multi_track));
            if (not _output_printer.is_console()) {
                _exporter.pass(_exporter.parse_result<multi_track_type>(_multi_track));
            }
        }, multi_track);
    }

}
