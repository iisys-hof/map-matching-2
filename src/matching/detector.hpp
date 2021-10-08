// Copyright (C) 2021-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_DETECTOR_HPP
#define MAP_MATCHING_2_DETECTOR_HPP

#include <iostream>
#include <set>
#include <vector>

namespace map_matching_2::matching {

    enum defect {
        none,
        equal,
        forward_backward
    };

    std::ostream &operator<<(std::ostream &out, const std::set<defect> &defect_set);

    template<typename Track>
    class detector {

    public:

        using measurement_type = typename Track::measurement_type;

        [[nodiscard]] std::vector<std::set<defect>> detect(
                const Track &track, bool detect_duplicates = true, bool detect_forward_backward = true) const;

        [[nodiscard]] std::set<defect> detect(
                const Track &track, std::size_t from, std::size_t to, bool detect_forward_backward = true) const;

    private:
        bool _equal(const Track &track, std::size_t from, std::size_t to) const;

        bool _forward_backward(const Track &track, std::size_t from, std::size_t to,
                               double relative_difference = 0.8) const;

    };

}

#endif //MAP_MATCHING_2_DETECTOR_HPP
