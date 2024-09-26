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

#ifndef MAP_MATCHING_2_MATCHING_MATCHER_MATCHER_OUTPUT_SETTINGS_HPP
#define MAP_MATCHING_2_MATCHING_MATCHER_MATCHER_OUTPUT_SETTINGS_HPP

#include <string>

namespace map_matching_2::matching {

    static const std::string DEFAULT_COLUMNS{
            "id,duration,track,prepared,match,track_length,prepared_length,match_length,edge_ids"
    };

    static const std::string DEFAULT_CANDIDATE_COLUMNS{
            "id,part_index,set_index,candidate_index,projection,distance,from,to,edge_id,is_result"
    };

    struct matcher_output_settings {
        std::string filename, candidates_filename;
        std::string columns, candidates_columns;
        bool console, verbose;

        constexpr matcher_output_settings(std::string filename, std::string columns,
                std::string candidates_filename, std::string candidates_columns,
                const bool console, const bool verbose)
            : filename{std::move(filename)}, columns{std::move(columns)},
            candidates_filename{std::move(candidates_filename)}, candidates_columns{std::move(candidates_columns)},
            console{console}, verbose{verbose} {}
    };

}

#endif //MAP_MATCHING_2_MATCHING_MATCHER_MATCHER_OUTPUT_SETTINGS_HPP
