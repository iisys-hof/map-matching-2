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

#ifndef MAP_MATCHING_2_APP_SETTINGS_HPP
#define MAP_MATCHING_2_APP_SETTINGS_HPP

#include "options.hpp"

#include "matching/settings.hpp"
#include "compare/settings.hpp"
#include "io/track/importer_settings.hpp"
#include "io/csv_settings.hpp"

namespace map_matching_2::app {

    matching::settings _match_settings(const match_data &data);

    compare::settings _compare_settings(const compare_data &data);

    io::track::importer_settings _importer_settings(const match_data &data);

    io::track::importer_settings _matches_importer_settings(const compare_data &data);

    io::track::importer_settings _compares_importer_settings(const compare_data &data);

    io::csv_settings _csv_settings(const csv_data &csv);

    io::csv_settings _csv_tracks_settings(const csv_tracks_data &csv);

}

#endif //MAP_MATCHING_2_APP_SETTINGS_HPP
