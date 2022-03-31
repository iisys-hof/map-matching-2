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

#ifndef MAP_MATCHING_2_MATCHING_SETTINGS_HPP
#define MAP_MATCHING_2_MATCHING_SETTINGS_HPP

namespace map_matching_2::matching {

    struct settings {

        std::size_t state_size = 2;
        bool skip_errors = true;
        bool filter_duplicates = true;
        bool filter_defects = false;
        bool simplify = true;
        double simplify_distance_tolerance = 5.0;
        bool median_merge = true;
        double median_merge_distance_tolerance = 10.0;
        bool adaptive_median_merge = true;
        bool adaptive_radius = true;
        bool candidate_adoption_siblings = true;
        bool candidate_adoption_nearby = true;
        bool candidate_adoption_reverse = false;
        double routing_max_distance_factor = 10.0;
        bool export_edges = false;
        bool join_merges = true;

        double mdp_distance_factor = 0.8;
        double mdp_length_factor = 0.3;
        double mdp_azimuth_factor = 0.5;
        double mdp_direction_factor = 0.6;
        double mdp_turn_factor = 0.5;

        double hmm_distance_factor = 0.8;
        double hmm_length_factor = 0.1;
        double hmm_azimuth_factor = 0.2;
        double hmm_direction_factor = 0.8;
        double hmm_turn_factor = 0.6;

        bool k_nearest_candidate_search = false;
        bool k_nearest_reverse = false;
        bool k_nearest_adjacent = false;
        std::size_t k_nearest = 16;

        std::size_t buffer_points = 16;
        double buffer_radius = 200.0;
        double buffer_upper_radius = 10000.0;
        double buffer_lower_radius = 200.0;

    };

}

#endif //MAP_MATCHING_2_MATCHING_SETTINGS_HPP
