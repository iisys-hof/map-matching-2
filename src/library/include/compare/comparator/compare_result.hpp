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

#ifndef MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARE_RESULT_HPP
#define MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARE_RESULT_HPP

#include "geometry/algorithm/compare/comparison.hpp"

#include "../settings.hpp"

namespace map_matching_2::compare {

    template<typename MultiTrack>
    struct compare_result {
        using multi_track_type = MultiTrack;
        using multi_rich_line_type = typename multi_track_type::multi_rich_line_type;
        using rich_segment_type = typename multi_track_type::rich_segment_type;

        const compare::settings &compare_settings;
        multi_track_type match;
        multi_track_type ground_truth;
        geometry::comparison<rich_segment_type> comparison;
        double duration{0.0};
    };

}

#endif //MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARE_RESULT_HPP
