// Copyright (C) 2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_TRACK_IMPORTER_SETTINGS_HPP
#define MAP_MATCHING_2_IO_TRACK_IMPORTER_SETTINGS_HPP

#include <string>

#include "io/csv_settings.hpp"

namespace map_matching_2::io::track {

    struct importer_settings {

        enum FILE_TYPE {
            UNKNOWN,
            CSV,
            GPX
        };

        bool quiet{false}, verbose{false};

        geometry::srs_transform tracks_srs_transform{};

        FILE_TYPE file_extension{UNKNOWN};
        std::vector<std::string> files;

        io::csv_settings csv_settings{};

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_IMPORTER_SETTINGS_HPP
