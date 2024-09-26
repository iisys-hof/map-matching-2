// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_IO_TRACK_EDGES_LIST_IMPORTER_HPP
#define MAP_MATCHING_2_TYPES_IO_TRACK_EDGES_LIST_IMPORTER_HPP

#include "types/extern_define.hpp"

#include "types/graph/compressed_sparse_row.hpp"

#include "io/track/track_exporter.hpp"

#include "io/track/edges_list_importer.hpp"

#define MM2_TRACK_EXPORTER map_matching_2::io::track::track_exporter

#define MM2_EDGES_LIST_IMPORTER(FORWARDER_TYPE, GRAPH_TYPE) \
    map_matching_2::io::track::edges_list_importer<FORWARDER_TYPE, GRAPH_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_EDGES_LIST_IMPORTER_TEMPLATE(FORWARDER_TYPE, GRAPH_TYPE) \
    MM2_EXTERN template class MM2_EDGES_LIST_IMPORTER(UNPACK FORWARDER_TYPE, UNPACK GRAPH_TYPE);

#define MM2_EDGES_LIST_IMPORTER_TEMPLATE_CS_GRAPH_TYPES(CS, GRAPH_TYPES) \
    MM2_EDGES_LIST_IMPORTER_TEMPLATE((MM2_TRACK_EXPORTER), (MM2_EAGER_COMPRESSED_SPARSE_ROW(CS, GRAPH_TYPES)))

#define MM2_EDGES_LIST_IMPORTER_TEMPLATE_CS(CS) \
    MM2_EDGES_LIST_IMPORTER_TEMPLATE_CS_GRAPH_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_EDGES_LIST_IMPORTER_TEMPLATE_CS_GRAPH_TYPES(CS, MM2_MMAP_TYPES)

MM2_EDGES_LIST_IMPORTER_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_EDGES_LIST_IMPORTER_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_EDGES_LIST_IMPORTER_TEMPLATE_CS(MM2_CARTESIAN)

#endif

#endif //MAP_MATCHING_2_TYPES_IO_TRACK_EDGES_LIST_IMPORTER_HPP
