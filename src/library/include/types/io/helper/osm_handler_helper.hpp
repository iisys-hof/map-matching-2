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

#ifndef MAP_MATCHING_2_TYPES_IO_HELPER_OSM_HANDLER_HELPER_HPP
#define MAP_MATCHING_2_TYPES_IO_HELPER_OSM_HANDLER_HELPER_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/storage/osm_handler_storage.hpp"
#include "types/io/helper/tag_helper.hpp"

#include "io/helper/osm_handler_helper.hpp"

#define MM2_OSM_HANDLER_HELPER(CS, TYPES, TAG_HELPER_TYPE) map_matching_2::io::helper::osm_handler_helper< \
    MM2_OSM_HANDLER_STORAGE(CS, TYPES), TAG_HELPER_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_OSM_HANDLER_HELPER_TEMPLATE(CS, TYPES, TAG_HELPER_TYPE) \
    MM2_EXTERN template class MM2_OSM_HANDLER_HELPER(CS, TYPES, UNPACK TAG_HELPER_TYPE);

#define MM2_OSM_HANDLER_HELPER_TEMPLATE_CS_TYPES(CS, TYPES) \
    MM2_OSM_HANDLER_HELPER_TEMPLATE(CS, TYPES, (MM2_IMPORT_TAG_HELPER(MM2_MEMORY_TYPES))) \
    MM2_OSM_HANDLER_HELPER_TEMPLATE(CS, TYPES, (MM2_IMPORT_TAG_HELPER(MM2_MMAP_TYPES)))

#define MM2_OSM_HANDLER_HELPER_TEMPLATE_CS(CS) \
    MM2_OSM_HANDLER_HELPER_TEMPLATE_CS_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_OSM_HANDLER_HELPER_TEMPLATE_CS_TYPES(CS, MM2_MMAP_TYPES)

MM2_OSM_HANDLER_HELPER_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_OSM_HANDLER_HELPER_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_OSM_HANDLER_HELPER_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::io::helper {

    template<geometry::is_point Point,
        memory_mapped::is_types Types = memory_mapped::memory_types,
        typename TagHelper = import_tag_helper_type<>>
    using osm_handler_helper_type = osm_handler_helper<
        memory_mapped::storage::osm_handler_storage_type<Point, Types>, TagHelper>;

}

#endif //MAP_MATCHING_2_TYPES_IO_HELPER_OSM_HANDLER_HELPER_HPP
