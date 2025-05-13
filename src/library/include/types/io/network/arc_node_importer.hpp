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

#ifndef MAP_MATCHING_2_TYPES_IO_NETWORK_ARC_NODE_IMPORTER_HPP
#define MAP_MATCHING_2_TYPES_IO_NETWORK_ARC_NODE_IMPORTER_HPP

#include "types/extern_define.hpp"

#include "types/io/helper/graph_helper.hpp"

#include "io/network/arc_node_importer.hpp"

#define MM2_ARC_NODE_IMPORTER(GRAPH_HELPER_TYPE) map_matching_2::io::network::arc_node_importer<GRAPH_HELPER_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_ARC_NODE_IMPORTER_TEMPLATE(GRAPH_HELPER_TYPE) \
    MM2_EXTERN template class MM2_ARC_NODE_IMPORTER(UNPACK GRAPH_HELPER_TYPE);

#define MM2_ARC_NODE_IMPORTER_TEMPLATE_CS_GRAPH_TYPES(CS, GRAPH_TYPES) \
    MM2_ARC_NODE_IMPORTER_TEMPLATE((MM2_GRAPH_HELPER_IMPORT(CS, GRAPH_TYPES, MM2_IMPORT_TAG_HELPER(MM2_MEMORY_TYPES)))) \
    MM2_ARC_NODE_IMPORTER_TEMPLATE((MM2_GRAPH_HELPER_IMPORT(CS, GRAPH_TYPES, MM2_IMPORT_TAG_HELPER(MM2_MMAP_TYPES))))

#define MM2_ARC_NODE_IMPORTER_TEMPLATE_CS(CS) \
    MM2_ARC_NODE_IMPORTER_TEMPLATE_CS_GRAPH_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_ARC_NODE_IMPORTER_TEMPLATE_CS_GRAPH_TYPES(CS, MM2_MMAP_TYPES)

MM2_ARC_NODE_IMPORTER_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_ARC_NODE_IMPORTER_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_ARC_NODE_IMPORTER_TEMPLATE_CS(MM2_CARTESIAN)

#endif


#endif //MAP_MATCHING_2_TYPES_IO_NETWORK_ARC_NODE_IMPORTER_HPP
