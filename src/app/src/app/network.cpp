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

#include "app/network.hpp"

namespace map_matching_2::app {

    template<typename CoordinateSystem, typename Graph, typename Tags>
    [[nodiscard]] geometry::network::network_import_variant_type
    _create_network_import_tags(const prepare_data &data) {
        return _create_network_import<geometry::network::network_import<
            io::helper::graph_helper_import_type<geometry::point_type<CoordinateSystem>,
                Graph, Tags>>>(data);
    }

    template<typename CoordinateSystem, typename Graph>
    [[nodiscard]] geometry::network::network_import_variant_type
    _create_network_import_graph(const prepare_data &data) {
        if (data.memory.mmap_tags_import) {
            return _create_network_import_tags<CoordinateSystem, Graph,
                io::helper::import_tag_helper_type<io::memory_mapped::file_types>>(data);
        } else {
            return _create_network_import_tags<CoordinateSystem, Graph,
                io::helper::import_tag_helper_type<io::memory_mapped::memory_types>>(data);
        }
    }

    template<typename CoordinateSystem>
    [[nodiscard]] geometry::network::network_import_variant_type
    _create_network_import_coordinate_system(const prepare_data &data) {
        if (data.memory.mmap_graph_import) {
            return _create_network_import_graph<CoordinateSystem, io::memory_mapped::file_types>(data);
        } else {
            return _create_network_import_graph<CoordinateSystem, io::memory_mapped::memory_types>(data);
        }
    }

    geometry::network::network_import_variant_type create_network_import(const prepare_data &data) {
        if (data.srs.srs_transform.transform_srs_type == geometry::SRS::CS_GEOGRAPHIC) {
            return _create_network_import_coordinate_system<geometry::cs_geographic>(data);
        } else if (data.srs.srs_transform.transform_srs_type == geometry::SRS::CS_SPHERICAL_EQUATORIAL) {
            return _create_network_import_coordinate_system<geometry::cs_spherical_equatorial>(data);
        } else if (data.srs.srs_transform.transform_srs_type == geometry::SRS::CS_CARTESIAN) {
            return _create_network_import_coordinate_system<geometry::cs_cartesian>(data);
        } else {
            throw std::invalid_argument{"invalid srs-type: " + data.srs._srs_type};
        }
    }

    template<typename CoordinateSystem, typename Graph, typename Tags, typename Indices>
    [[nodiscard]] geometry::network::network_variant_type
    _open_network_indices(const match_data &data) {
        return _open_network<geometry::network::network<
            io::helper::graph_helper_type<geometry::point_type<CoordinateSystem>,
                Graph, Tags>,
            io::helper::index_helper_read_only_type<
                geometry::point_type<CoordinateSystem>,
                Indices>>>(data);
    }

    template<typename CoordinateSystem, typename Graph, typename Tags>
    [[nodiscard]] geometry::network::network_variant_type
    _open_network_tags(const match_data &data) {
        if (data.memory.mmap_indices) {
            return _open_network_indices<CoordinateSystem, Graph, Tags, io::memory_mapped::file_types>(data);
        } else {
            return _open_network_indices<CoordinateSystem, Graph, Tags, io::memory_mapped::memory_types>(data);
        }
    }

    template<typename CoordinateSystem, typename Graph>
    [[nodiscard]] geometry::network::network_variant_type
    _open_network_graph(const match_data &data) {
        if (data.memory.mmap_tags) {
            return _open_network_tags<CoordinateSystem, Graph,
                io::helper::tag_helper_type<io::memory_mapped::file_types>>(data);
        } else {
            return _open_network_tags<CoordinateSystem, Graph,
                io::helper::tag_helper_type<io::memory_mapped::memory_types>>(data);
        }
    }

    template<typename CoordinateSystem>
    [[nodiscard]] geometry::network::network_variant_type
    _open_network_coordinate_system(const match_data &data) {
        if (data.memory.mmap_graph) {
            return _open_network_graph<CoordinateSystem, io::memory_mapped::file_types>(data);
        } else {
            return _open_network_graph<CoordinateSystem, io::memory_mapped::memory_types>(data);
        }
    }

    geometry::network::network_variant_type open_network(const match_data &data) {
        if (data.srs.srs.srs_type == geometry::SRS::CS_GEOGRAPHIC) {
            return _open_network_coordinate_system<geometry::cs_geographic>(data);
        } else if (data.srs.srs.srs_type == geometry::SRS::CS_SPHERICAL_EQUATORIAL) {
            return _open_network_coordinate_system<geometry::cs_spherical_equatorial>(data);
        } else if (data.srs.srs.srs_type == geometry::SRS::CS_CARTESIAN) {
            return _open_network_coordinate_system<geometry::cs_cartesian>(data);
        } else {
            throw std::invalid_argument{"invalid srs-type: " + data.srs._srs_type};
        }
    }

}
