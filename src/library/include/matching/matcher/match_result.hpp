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

#ifndef MAP_MATCHING_2_MATCHING_MATCHER_MATCH_RESULT_HPP
#define MAP_MATCHING_2_MATCHING_MATCHER_MATCH_RESULT_HPP

#include <vector>

#include "matching/traits/network.hpp"

namespace map_matching_2::matching {

    template<typename Learner>
    struct match_result {
        using learner_type = Learner;
        using environment_type = typename learner_type::environment_type;
        using network_type = typename environment_type::network_type;

        using multi_track_type = typename network_traits<network_type>::multi_track_type;
        using route_type = typename network_traits<network_type>::route_type;
        using policy_type = typename network_traits<network_type>::policy_type;

        std::vector<environment_type> environments;
        std::vector<learner_type> learner;
        multi_track_type track;
        multi_track_type prepared;
        std::vector<policy_type> policies;
        route_type match;
        std::vector<osmium::object_id_type> edge_ids;
        double duration;
    };

}

#endif //MAP_MATCHING_2_MATCHING_MATCHER_MATCH_RESULT_HPP
