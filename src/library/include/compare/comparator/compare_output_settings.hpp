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

#ifndef MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARE_OUTPUT_SETTINGS_HPP
#define MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARE_OUTPUT_SETTINGS_HPP

#include <string>

namespace map_matching_2::compare {

    static const std::string DEFAULT_COLUMNS{
            "id,duration,match,ground_truth,match_length,ground_truth_length,"
            "correct_fraction,error_fraction,correct,error_added,error_missed,corrects,error_adds,error_misses"
    };

    struct comparator_output_settings {
        std::string filename;
        std::string columns;
        bool console, verbose;

        constexpr comparator_output_settings(std::string filename, std::string columns,
                const bool console, const bool verbose)
            : filename{std::move(filename)}, columns{std::move(columns)},
            console{console}, verbose{verbose} {}
    };

}

#endif //MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARE_OUTPUT_SETTINGS_HPP
