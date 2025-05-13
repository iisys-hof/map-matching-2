// Copyright (C) 2022-2025 Adrian Wöltche
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

#include "geometry/algorithm/reproject.hpp"
#include "geometry/algorithm/srs_dispatch.hpp"

namespace map_matching_2::geometry {

    point_reprojector_variant create_point_reprojector(const srs_transform &srs_transform) {
        return srs_dispatch(srs_transform, [&srs_transform]<typename InputCS, typename OutputCS>() {
            return point_reprojector_variant{
                    point_reprojector_type<point_type<InputCS>, point_type<OutputCS>>{srs_transform}
            };
        });
    }

}
