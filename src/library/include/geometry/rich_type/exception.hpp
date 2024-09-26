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

#ifndef MAP_MATCHING_2_RICH_TYPE_EXCEPTION_HPP
#define MAP_MATCHING_2_RICH_TYPE_EXCEPTION_HPP

#include <stdexcept>

namespace map_matching_2::geometry {

    struct computation_exception : std::logic_error {

        explicit computation_exception(const std::string &err)
            : std::logic_error(err) {}

    };

    struct length_computation_exception : computation_exception {

        explicit length_computation_exception(const std::string &err)
            : computation_exception(err) {}

    };

    struct azimuth_computation_exception : computation_exception {

        explicit azimuth_computation_exception(const std::string &err)
            : computation_exception(err) {}

    };

    struct rich_line_merge_exception : std::logic_error {

        explicit rich_line_merge_exception(const std::string &err)
            : std::logic_error(err) {}

    };

    struct directions_computation_exception : computation_exception {

        explicit directions_computation_exception(const std::string &err)
            : computation_exception(err) {}

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_EXCEPTION_HPP
