// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_OSM_HANDLER_STORAGE_HPP
#define MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_OSM_HANDLER_STORAGE_HPP

#include "types/extern_define.hpp"

#include "types/geometry/network/node.hpp"

#include "io/memory_mapped/storage/osm_handler_storage.hpp"

#define MM2_OSM_HANDLER_STORAGE(CS, TYPES) map_matching_2::io::memory_mapped::storage::osm_handler_storage< \
    MM2_IMPORT_NODE(CS, TYPES), TYPES>

#ifdef EXPLICIT_TEMPLATES

#define MM2_OSM_HANDLER_STORAGE_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_OSM_HANDLER_STORAGE(CS, TYPES);

#define MM2_OSM_HANDLER_STORAGE_TEMPLATE_CS(CS) \
    MM2_OSM_HANDLER_STORAGE_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_OSM_HANDLER_STORAGE_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_OSM_HANDLER_STORAGE_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_OSM_HANDLER_STORAGE_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_OSM_HANDLER_STORAGE_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::io::memory_mapped::storage {

    template<geometry::is_point Point, is_types Types = memory_types>
    using osm_handler_storage_type = osm_handler_storage<geometry::network::import_node_type<Point, Types>, Types>;

}

#endif //MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_OSM_HANDLER_STORAGE_HPP
