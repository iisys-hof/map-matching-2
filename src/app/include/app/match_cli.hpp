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

#ifndef MAP_MATCHING_2_APP_MATCH_CLI_HPP
#define MAP_MATCHING_2_APP_MATCH_CLI_HPP

#include "match.hpp"

namespace map_matching_2::app {

    void _match_cli(const match_data &data);

    int match_cli(int argc, char *argv[]);

}

#endif //MAP_MATCHING_2_APP_MATCH_CLI_HPP
