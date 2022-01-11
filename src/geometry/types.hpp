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

#ifndef MAP_MATCHING_2_TYPES_HPP
#define MAP_MATCHING_2_TYPES_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/srs/epsg.hpp>
#include <boost/geometry/srs/projection.hpp>

namespace map_matching_2::geometry {

    template<typename CoordinateSystem>
    struct types {
        using coordinate_system_type = CoordinateSystem;
        using point_type = boost::geometry::model::point<double, 2, coordinate_system_type>;
        using line_type = boost::geometry::model::linestring<point_type>;
        using segment_type = boost::geometry::model::segment<point_type>;
    };

    using cs_geographic = boost::geometry::cs::geographic<boost::geometry::degree>;
    using cs_cartesian = boost::geometry::cs::cartesian;

    using types_geographic = types<cs_geographic>;
    using types_cartesian = types<cs_cartesian>;

}

#endif //MAP_MATCHING_2_TYPES_HPP
