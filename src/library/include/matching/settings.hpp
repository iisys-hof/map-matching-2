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

#ifndef MAP_MATCHING_2_MATCHING_SETTINGS_HPP
#define MAP_MATCHING_2_MATCHING_SETTINGS_HPP

#include <cstddef>
#include <string>

#include "geometry/srs.hpp"

namespace map_matching_2::matching {

    struct settings {

        enum CANDIDATE_SEARCH {
            CIRCLE,
            NEAREST,
            COMBINED,
            NEXT
        };

        enum MODEL {
            POLICY_ITERATION,
            VALUE_ITERATION,
            Q_LEARNING,
            VITERBI
        };

        bool verbose{false}, quiet{false};

        geometry::srs_transform tracks_srs_transform{};

        bool export_timestamps{false}, export_edges{false}, export_edge_ids{false}, join_merges{true};
        std::string output{}, filename{}, export_time_zone{"UTC"}, export_candidates{};

        bool console{false};

        bool filter_duplicates{true}, simplify_track{true}, median_merge{true}, adaptive_median_merge{true},
                within_edge_turns{false}, a_star{false}, candidate_adoption_siblings{true},
                candidate_adoption_nearby{true}, candidate_adoption_reverse{false}, adaptive_radius{true};
        double max_time{0.0}, routing_max_distance_factor{5.0}, simplify_track_distance_tolerance{5.0},
                median_merge_distance_tolerance{10.0}, radius{200.0}, radius_upper_limit{10000.0},
                radius_lower_limit{200.0};
        std::size_t k_nearest{16};
        CANDIDATE_SEARCH candidate_search{COMBINED};

        bool skip_errors{true};
        std::size_t state_size{2};
        double discount{0.99};
        MODEL model{VALUE_ITERATION};

        double mdp_distance_factor{0.8}, mdp_length_factor{0.3}, mdp_azimuth_factor{0.5}, mdp_direction_factor{0.6},
                mdp_turn_factor{0.5};
        double hmm_distance_factor{0.8}, hmm_length_factor{0.1}, hmm_azimuth_factor{0.2}, hmm_direction_factor{0.8},
                hmm_turn_factor{0.6};

        double threshold{1e-3};

        double learning_rate{0.9}, epsilon{0.5}, early_stop_factor{1.0};
        std::size_t episodes{1000000000};

    };

}

#endif //MAP_MATCHING_2_MATCHING_SETTINGS_HPP
