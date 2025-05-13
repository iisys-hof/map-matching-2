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

#ifndef MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_GRAPH_STORAGE_HPP
#define MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_GRAPH_STORAGE_HPP

#include "types/extern_define.hpp"

#include "types/graph/adjacency_list.hpp"
#include "types/graph/compressed_sparse_row.hpp"

#include "io/memory_mapped/storage/graph_storage.hpp"

#define MM2_GRAPH_STORAGE_IMPORT(CS, TYPES) map_matching_2::io::memory_mapped::storage::graph_storage< \
    MM2_ADJACENCY_LIST(CS, TYPES), TYPES>

#define MM2_GRAPH_STORAGE(CS, TYPES) map_matching_2::io::memory_mapped::storage::graph_storage< \
    MM2_EAGER_COMPRESSED_SPARSE_ROW(CS, TYPES), TYPES>

#ifdef EXPLICIT_TEMPLATES

#define MM2_GRAPH_STORAGE_IMPORT_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_GRAPH_STORAGE_IMPORT(CS, TYPES);

#define MM2_GRAPH_STORAGE_IMPORT_TEMPLATE_CS(CS) \
    MM2_GRAPH_STORAGE_IMPORT_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_GRAPH_STORAGE_IMPORT_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_GRAPH_STORAGE_IMPORT_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_GRAPH_STORAGE_IMPORT_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_GRAPH_STORAGE_IMPORT_TEMPLATE_CS(MM2_CARTESIAN)

#define MM2_GRAPH_STORAGE_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_GRAPH_STORAGE(CS, TYPES);

#define MM2_GRAPH_STORAGE_TEMPLATE_CS(CS) \
    MM2_GRAPH_STORAGE_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_GRAPH_STORAGE_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_GRAPH_STORAGE_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_GRAPH_STORAGE_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_GRAPH_STORAGE_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::io::memory_mapped::storage {

    template<geometry::is_point Point, is_types Types = memory_types>
    using graph_storage_import_type = graph_storage<graph::network_adjacency_list_type<Point, Types>, Types>;

    template<geometry::is_point Point, is_types Types = memory_types>
    using graph_storage_type = graph_storage<graph::network_compressed_sparse_row_type<Point, Types>, Types>;

}

#endif //MAP_MATCHING_2_TYPES_IO_MEMORY_MAPPED_STORAGE_GRAPH_STORAGE_HPP
