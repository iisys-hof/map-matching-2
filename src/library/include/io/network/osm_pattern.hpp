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

#ifndef MAP_MATCHING_2_IO_NETWORK_OSM_PATTERN_HPP
#define MAP_MATCHING_2_IO_NETWORK_OSM_PATTERN_HPP

#include <regex>

namespace map_matching_2::io::network {

    const std::regex OSM_TAGS_FILTER_PATTERN{"^(true|false)(\\|(true|false),[^,=]+(=[^,|]+)?)*$"};

}

#endif //MAP_MATCHING_2_IO_NETWORK_OSM_PATTERN_HPP
