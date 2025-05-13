// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_CSV_SETTINGS_HPP
#define MAP_MATCHING_2_IO_CSV_SETTINGS_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace map_matching_2::io {

    struct csv_settings {

        bool grouped{true}, no_header{false}, no_id{false}, no_id_whole_file{false}, wkt{false}, no_parse_time{false};

        std::size_t skip_lines{0};

        std::string delimiter{","}, id_aggregator{"_"}, field_x{"x"}, field_y{"y"}, field_geometry{"geometry"},
                time_aggregator{"_"}, time_format{"%FT%T%Oz"};

        std::vector<std::string> field_id{"id"}, field_time{"time"}, selectors{};

    };

}

#endif //MAP_MATCHING_2_IO_CSV_SETTINGS_HPP
