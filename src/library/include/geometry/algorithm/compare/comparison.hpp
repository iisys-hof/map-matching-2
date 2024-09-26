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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_COMPARE_COMPARISON_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_COMPARE_COMPARISON_HPP

#include <vector>
#include <list>

#include "geometry/rich_type/traits/rich_segment.hpp"

namespace map_matching_2::geometry {

    template<typename RichSegment>
    struct comparison {
        using rich_segment_type = RichSegment;
        using length_type = typename rich_segment_traits<RichSegment>::length_type;

        constexpr comparison() = default;

        constexpr comparison(
                const length_type correct_fraction,
                const length_type error_fraction,
                const length_type correct,
                const length_type error_added,
                const length_type error_missed,
                std::pmr::list<rich_segment_type> corrects,
                std::pmr::list<rich_segment_type> error_adds,
                std::pmr::list<rich_segment_type> error_misses)
            : correct_fraction{correct_fraction},
            error_fraction{error_fraction},
            correct{correct},
            error_added{error_added},
            error_missed{error_missed},
            corrects{
                    std::make_move_iterator(std::begin(corrects)),
                    std::make_move_iterator(std::end(corrects))
            },
            error_adds{
                    std::make_move_iterator(std::begin(error_adds)),
                    std::make_move_iterator(std::end(error_adds))
            },
            error_misses{
                    std::make_move_iterator(std::begin(error_misses)),
                    std::make_move_iterator(std::end(error_misses))
            } {}

        length_type correct_fraction{};
        length_type error_fraction{};
        length_type correct{};
        length_type error_added{};
        length_type error_missed{};
        std::vector<rich_segment_type> corrects{};
        std::vector<rich_segment_type> error_adds{};
        std::vector<rich_segment_type> error_misses{};

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_COMPARE_COMPARISON_HPP
