// Copyright (C) 2025 Adrian Wöltche
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

#include "util/checks.hpp"

#include <cctype>

namespace map_matching_2::util {

    bool is_number(const std::string &str) {
        if (not str.empty()) {
            for (auto it = std::cbegin(str); it != std::cend(str); ++it) {
                const char c = *it;

                if (not std::isdigit(c)) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }

}
