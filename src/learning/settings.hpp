// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_LEARNING_SETTINGS_HPP
#define MAP_MATCHING_2_LEARNING_SETTINGS_HPP

namespace map_matching_2::learning {

    struct settings {

        double threshold = 1e-3;
        double discount = 0.99;

        std::size_t episodes = 1000000000;
        double learning_rate = 0.8;
        double epsilon = 0.2;

    };

}

#endif //MAP_MATCHING_2_LEARNING_SETTINGS_HPP
