// Copyright (C) 2020-2024 Adrian Wöltche
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

#include <cassert>

// __clang_major__ >= 18 # chrono not yet supported
#if (__cplusplus >= 202002L and (defined(__GNUC__) and __GNUC__ >= 14)) or \
    (defined(_MSC_VER) and _MSC_VER >= 1939)
#define HAS_CHRONO_PARSE 1
#include <chrono>
#else
#define HAS_CHRONO_PARSE 0
#include <date/date.h>
#endif

namespace map_matching_2::io {

    [[nodiscard]] bool importer::is_number(const std::string &str) {
        if (not str.empty()) {
            for (char c : str) {
                if (not std::isdigit(c)) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    [[nodiscard]] std::vector<char> importer::parse_delimiter(const std::string &delimiter) {
        std::vector<char> delimiters;
        delimiters.reserve(delimiter.size());
        bool escaped = false;
        for (const char &c : delimiter) {
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

    [[nodiscard]] static std::vector<char> parse_delimiter(const std::string &delimiter) {
        std::vector<char> delimiters;
        delimiters.reserve(delimiter.size());
        bool escaped = false;
        for (const char &c : delimiter) {
            if (c == '\\') {
                escaped = true;
                continue;
            }

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
                delimiters.emplace_back(c);
            }
        }
        return delimiters;
    }

    std::uint64_t importer::parse_time(const std::string &time_str, const std::string &format) {
        std::istringstream time_str_stream{time_str};
#if HAS_CHRONO_PARSE
        std::chrono::sys_time<std::chrono::milliseconds> time;
        time_str_stream >> std::chrono::parse(format, time);
#else
        date::sys_time<std::chrono::milliseconds> time;
        time_str_stream >> date::parse(format, time);
#endif
        const auto count = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
        assert(count >= 0);
        return static_cast<std::uint64_t>(count);
    }

}
