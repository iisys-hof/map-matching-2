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

#ifndef MAP_MATCHING_2_COMPARE_SETTINGS_HPP
#define MAP_MATCHING_2_COMPARE_SETTINGS_HPP

#include <string>

#include "geometry/srs.hpp"

namespace map_matching_2::compare {

    struct settings {

        bool verbose{false}, quiet{false};

        geometry::srs_transform matches_srs_transform{}, ground_truth_srs_transform{};

        std::string output{}, filename{};

        bool console{false}, ignore_non_existent{false};

        double simplifying_tolerance{0.1}, simplifying_reverse_tolerance{0.1}, adoption_distance_tolerance{0.3},
                split_distance_tolerance{1.0}, split_direction_tolerance{90.0};

    };

}

#endif //MAP_MATCHING_2_COMPARE_SETTINGS_HPP
