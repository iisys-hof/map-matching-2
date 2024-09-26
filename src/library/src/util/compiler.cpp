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

#include "util/compiler.hpp"

namespace map_matching_2::util {

    std::string get_compiler_info() {
        std::string result;
#ifdef __clang__
        result.append("Clang version ")
              .append(std::to_string(__clang_major__)).append(".")
              .append(std::to_string(__clang_minor__)).append(".")
              .append(std::to_string(__clang_patchlevel__));
#elif __GNUC__
        result.append("GCC version ")
              .append(std::to_string(__GNUC__)).append(".")
              .append(std::to_string(__GNUC_MINOR__)).append(".")
              .append(std::to_string(__GNUC_PATCHLEVEL__));
#elif _MSC_VER
        result.append("MSVC version ").append(std::to_string(_MSC_VER));
#else
        result.append("Unknown compiler");
#endif
        return result;
    }

}
