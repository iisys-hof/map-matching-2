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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_SRS_DISPATCH_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_SRS_DISPATCH_HPP

#include "types/geometry/point.hpp"

#include "../srs.hpp"

namespace map_matching_2::geometry {

    template<typename Function>
    auto srs_dispatch(const srs_transform &srs_transform, const Function &func) {
        if (srs_transform.srs_type == SRS::CS_GEOGRAPHIC and
            srs_transform.transform_srs_type == SRS::CS_GEOGRAPHIC) {
            return func.template operator()<geometry::cs_geographic, geometry::cs_geographic>();
        } else if (srs_transform.srs_type == SRS::CS_GEOGRAPHIC and
            srs_transform.transform_srs_type == SRS::CS_SPHERICAL_EQUATORIAL) {
            return func.template operator()<geometry::cs_geographic, geometry::cs_spherical_equatorial>();
        } else if (srs_transform.srs_type == SRS::CS_GEOGRAPHIC and
            srs_transform.transform_srs_type == SRS::CS_CARTESIAN) {
            return func.template operator()<geometry::cs_geographic, geometry::cs_cartesian>();
        } else if (srs_transform.srs_type == SRS::CS_SPHERICAL_EQUATORIAL and
            srs_transform.transform_srs_type == SRS::CS_GEOGRAPHIC) {
            return func.template operator()<geometry::cs_spherical_equatorial, geometry::cs_geographic>();
        } else if (srs_transform.srs_type == SRS::CS_SPHERICAL_EQUATORIAL and
            srs_transform.transform_srs_type == SRS::CS_SPHERICAL_EQUATORIAL) {
            return func.template operator()<geometry::cs_spherical_equatorial, geometry::cs_spherical_equatorial>();
        } else if (srs_transform.srs_type == SRS::CS_SPHERICAL_EQUATORIAL and
            srs_transform.transform_srs_type == SRS::CS_CARTESIAN) {
            return func.template operator()<geometry::cs_spherical_equatorial, geometry::cs_cartesian>();
        } else if (srs_transform.srs_type == SRS::CS_CARTESIAN and
            srs_transform.transform_srs_type == SRS::CS_GEOGRAPHIC) {
            return func.template operator()<geometry::cs_cartesian, geometry::cs_geographic>();
        } else if (srs_transform.srs_type == SRS::CS_CARTESIAN and
            srs_transform.transform_srs_type == SRS::CS_SPHERICAL_EQUATORIAL) {
            return func.template operator()<geometry::cs_cartesian, geometry::cs_spherical_equatorial>();
        } else if (srs_transform.srs_type == SRS::CS_CARTESIAN and
            srs_transform.transform_srs_type == SRS::CS_CARTESIAN) {
            return func.template operator()<geometry::cs_cartesian, geometry::cs_cartesian>();
        } else {
            throw std::invalid_argument{"srs_transform has no correct reprojection information."};
        }
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_SRS_DISPATCH_HPP
