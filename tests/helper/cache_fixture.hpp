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

#ifndef MAP_MATCHING_2_TESTS_HELPER_CACHE_FIXTURE_HPP
#define MAP_MATCHING_2_TESTS_HELPER_CACHE_FIXTURE_HPP

#include "geometry/cache.hpp"

struct cache_fixture {

    cache_fixture() {
        map_matching_2::geometry::enable_cache();
    }

    ~cache_fixture() = default;

};

#endif //MAP_MATCHING_2_TESTS_HELPER_CACHE_FIXTURE_HPP
