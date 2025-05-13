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

#include "geometry/cache.hpp"

namespace map_matching_2::geometry {

    thread_local std::unique_ptr<distance_cache_type> distance_cache{};
    thread_local std::unique_ptr<azimuth_cache_type> azimuth_cache{};
    bool use_distance_cache{false};
    bool use_azimuth_cache{false};

    void enable_cache() {
        use_distance_cache = true;
        use_azimuth_cache = true;
    }

    void disable_cache() {
        use_distance_cache = false;
        use_azimuth_cache = false;
    }

    void reset_distance_cache() {
        distance_cache = std::make_unique<distance_cache_type>();
    }

    void reset_azimuth_cache() {
        azimuth_cache = std::make_unique<azimuth_cache_type>();
    }

}
