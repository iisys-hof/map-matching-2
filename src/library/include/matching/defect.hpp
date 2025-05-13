// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MATCHING_DEFECT_HPP
#define MAP_MATCHING_2_MATCHING_DEFECT_HPP

#include <boost/unordered/unordered_flat_set.hpp>

namespace map_matching_2::matching {

    enum defect {
        none,
        equal,
        forward_backward
    };

    std::ostream &operator<<(std::ostream &out, const boost::unordered_flat_set<defect> &defect_set);

}

#endif //MAP_MATCHING_2_MATCHING_DEFECT_HPP
