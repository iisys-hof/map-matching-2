// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_IO_NETWORK_OSM_EXPORTER_HPP
#define MAP_MATCHING_2_TYPES_IO_NETWORK_OSM_EXPORTER_HPP

#include "types/extern_define.hpp"

#include "types/graph/compressed_sparse_row.hpp"
#include "types/io/helper/tag_helper.hpp"

#include "io/network/osm_exporter.hpp"

#define OSM_EXPORTER(GRAPH_TYPE, TAG_HELPER_TYPE) \
    map_matching_2::io::network::osm_exporter<GRAPH_TYPE, TAG_HELPER_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define OSM_EXPORTER_TEMPLATE(GRAPH_TYPE, TAG_HELPER_TYPE) \
    MM2_EXTERN template class OSM_EXPORTER(UNPACK GRAPH_TYPE, UNPACK TAG_HELPER_TYPE);

#define OSM_EXPORTER_TEMPLATE_CS_GRAPH_TYPE(GRAPH_TYPE) \
    OSM_EXPORTER_TEMPLATE(GRAPH_TYPE, (MM2_TAG_HELPER(MM2_MEMORY_TYPES))) \
    OSM_EXPORTER_TEMPLATE(GRAPH_TYPE, (MM2_TAG_HELPER(MM2_MMAP_TYPES)))

#define OSM_EXPORTER_TEMPLATE_CS_TYPES(CS, TYPES) \
    OSM_EXPORTER_TEMPLATE_CS_GRAPH_TYPE((MM2_EAGER_COMPRESSED_SPARSE_ROW(CS, TYPES)))

#define OSM_EXPORTER_TEMPLATE_CS(CS) \
    OSM_EXPORTER_TEMPLATE_CS_TYPES(CS, MM2_MEMORY_TYPES) \
    OSM_EXPORTER_TEMPLATE_CS_TYPES(CS, MM2_MMAP_TYPES)

OSM_EXPORTER_TEMPLATE_CS(MM2_GEOGRAPHIC)
OSM_EXPORTER_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
OSM_EXPORTER_TEMPLATE_CS(MM2_CARTESIAN)

#endif

#endif //MAP_MATCHING_2_TYPES_IO_NETWORK_OSM_EXPORTER_HPP
