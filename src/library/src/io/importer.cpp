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

#include "io/importer.hpp"

namespace map_matching_2::io {

    [[nodiscard]] std::vector<char> importer::parse_delimiter(const std::string &delimiter) {
        std::vector<char> delimiters;
        delimiters.reserve(delimiter.size());
        bool escaped = false;
        for (auto it = std::cbegin(delimiter); it != std::cend(delimiter); ++it) {
            const char c = *it;

            if (escaped) {
                if (c == 't') {
                    delimiters.emplace_back('\t');
                } else if (c == 's') {
                    delimiters.emplace_back(' ');
                } else {
                    delimiters.emplace_back(c);
                }
                escaped = false;
            } else {
                if (c == '\\') {
                    escaped = true;
                } else {
                    delimiters.emplace_back(c);
                }
            }
        }
        return delimiters;
    }

}
