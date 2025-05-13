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

#ifndef MAP_MATCHING_2_GEOMETRY_CACHE_HPP
#define MAP_MATCHING_2_GEOMETRY_CACHE_HPP

#include <memory>
#include <tuple>

#include <boost/unordered/unordered_flat_map.hpp>

#include "types/geometry/point.hpp"

#include "traits.hpp"

namespace map_matching_2::geometry {

    using _coordinate_type = std::common_type_t<
        data<point_type<cs_geographic>>::coordinate_type,
        data<point_type<cs_spherical_equatorial>>::coordinate_type,
        data<point_type<cs_cartesian>>::coordinate_type>;

    using _distance_type = std::common_type_t<
        data<point_type<cs_geographic>>::distance_type,
        data<point_type<cs_spherical_equatorial>>::distance_type,
        data<point_type<cs_cartesian>>::distance_type>;

    using _angle_type = std::common_type_t<
        data<point_type<cs_geographic>>::angle_type,
        data<point_type<cs_spherical_equatorial>>::angle_type,
        data<point_type<cs_cartesian>>::angle_type>;

    using _cache_key_type = std::tuple<_coordinate_type, _coordinate_type, _coordinate_type, _coordinate_type>;
    using distance_cache_type = boost::unordered::unordered_flat_map<_cache_key_type, _distance_type>;
    using azimuth_cache_type = boost::unordered::unordered_flat_map<_cache_key_type, _angle_type>;

    // these are declared in the corresponding cpp file
    extern thread_local std::unique_ptr<distance_cache_type> distance_cache;
    extern thread_local std::unique_ptr<azimuth_cache_type> azimuth_cache;
    extern bool use_distance_cache;
    extern bool use_azimuth_cache;

    void enable_cache();

    void disable_cache();

    void reset_distance_cache();

    void reset_azimuth_cache();

}

#endif //MAP_MATCHING_2_GEOMETRY_CACHE_HPP
