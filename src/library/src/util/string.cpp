// Copyright (C) 2026 Adrian Wöltche
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

#include "util/string.hpp"

namespace map_matching_2::util {

    bool parse_bool(const std::string_view str) {
        // from highest probability to lowest probability to hopefully reduce check time
        if (str == "1") {
            return true;
        }
        if (str == "0") {
            return false;
        }
        if (str == "true") {
            return true;
        }
        if (str == "false") {
            return false;
        }
        if (str == "True") {
            return true;
        }
        if (str == "False") {
            return false;
        }

        throw std::invalid_argument(std::format("invalid string cannot be converted to bool: {}", str));
    }

}
