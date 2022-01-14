// Copyright (C) 2020-2021 Adrian Wöltche
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

#include "importer.hpp"

#include <date/date.h>

namespace map_matching_2::io {

    importer::importer(std::string filename)
            : _filename{std::move(filename)} {}

    std::uint64_t importer::parse_time(const std::string &time_str, const std::string &format) {
        date::sys_time<std::chrono::milliseconds> time;
        std::istringstream time_str_stream{time_str};
        time_str_stream >> date::parse(format, time);
        const auto count = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
        assert(count >= 0);
        return (std::uint64_t) count;
    }

    const std::string &importer::filename() const {
        return _filename;
    }

}
