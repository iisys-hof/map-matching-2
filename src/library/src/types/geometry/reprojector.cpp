// Copyright (C) 2022-2024 Adrian Wöltche
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

#include "types/geometry/reprojector.hpp"

#include "types/extern_undefine.hpp"

MM2_POINT_REPROJECTOR_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_POINT_REPROJECTOR_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_POINT_REPROJECTOR_TEMPLATE_CS(MM2_CARTESIAN)

MM2_POINT_REPROJECTOR_VARIANT_TEMPLATE