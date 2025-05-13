// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_UTIL_MACROS_HPP
#define MAP_MATCHING_2_UTIL_MACROS_HPP

#if defined(_WIN32) || defined(_WIN64)
#define MM2_WINDOWS
#elif defined(__linux__)
#define MM2_LINUX
#endif

#define CONCATENATE_IMPL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_IMPL(x, y)

#define UNPACK(...) __VA_ARGS__

#endif //MAP_MATCHING_2_UTIL_MACROS_HPP
