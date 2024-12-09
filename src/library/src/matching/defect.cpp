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

#include "matching/defect.hpp"

#include <string>
#include <iostream>

namespace map_matching_2::matching {

    std::ostream &operator<<(std::ostream &out, const boost::unordered_flat_set<defect> &defect_set) {
        std::string defect_str;
        if (defect_set.contains(defect::none)) {
            defect_str.append("none, ");
        }
        if (defect_set.contains(defect::equal)) {
            defect_str.append("equal, ");
        }
        if (defect_set.contains(defect::forward_backward)) {
            defect_str.append("forward_backward, ");
        }
        if (not defect_str.empty() and defect_str.ends_with(", ")) {
            defect_str.erase(defect_str.length() - 2);
        }
        out << defect_str;
        return out;
    }

}
