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

#ifndef MAP_MATCHING_2_MATCHING_ALGORITHM_ALGORITHMS_HPP
#define MAP_MATCHING_2_MATCHING_ALGORITHM_ALGORITHMS_HPP

#include "types/matching/algorithm/candidate_search.hpp"
#include "types/matching/algorithm/detect.hpp"
#include "types/matching/algorithm/route.hpp"

#include "matching/traits/network.hpp"

namespace map_matching_2::matching {

    template<typename Network>
    class algorithms {

    public:
        using network_type = Network;
        using track_rich_line_type = typename matching::network_traits<network_type>::track_rich_line_type;

        constexpr explicit algorithms(const network_type &network)
            : searcher{network}, router{network} {}

        detector_type<track_rich_line_type> detector{};
        candidate_searcher<network_type> searcher;
        class router<network_type> router;

    };

}

#endif //MAP_MATCHING_2_MATCHING_ALGORITHM_ALGORITHMS_HPP
