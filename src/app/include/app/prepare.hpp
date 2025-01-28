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


#ifndef MAP_MATCHING_2_APP_PREPARE_HPP
#define MAP_MATCHING_2_APP_PREPARE_HPP

#include "global.hpp"

#include <utility>
#include <format>

#include "types/io/network/arc_node_importer.hpp"
#include "types/io/network/melbourne_importer.hpp"
#include "types/io/network/seattle_importer.hpp"
#include "types/io/network/giscup_importer.hpp"
#include "types/io/network/osm_exporter.hpp"

#include "io/network/osm_car_importer.hpp"
#include "io/network/osm_manual_importer.hpp"

#include "options.hpp"
#include "network.hpp"

namespace map_matching_2::app {

    template<typename GraphHelper>
    void _import_osm(GraphHelper &graph_helper, const prepare_data &data) {
        auto osm_handler_variant = create_osm_handler(graph_helper, data);

        std::visit([&](auto &&osm_handler) {
            if (data.osm_import.profile == OSM_TAGS_PROFILE_CAR) {
                io::network::osm_car_importer osm_car_importer{data.network.files, *osm_handler};
                osm_car_importer.read();
            } else if (data.osm_import.profile == OSM_TAGS_PROFILE_MANUAL) {
                io::network::osm_manual_importer osm_manual_importer{
                        data.network.files, *osm_handler,
                        data.osm_import.query, data.osm_import.filter,
                        data.osm_import.oneway, data.osm_import.oneway_reverse
                };
                osm_manual_importer.read();
            }
            osm_handler.reset();
        }, std::move(osm_handler_variant));
    }

    template<typename GraphHelper>
    void _import_arc_node(GraphHelper &graph_helper, const prepare_data &data) {
        const auto reprojector_variant = geometry::create_point_reprojector(data.srs.srs_transform);
        io::network::arc_node_importer arc_node_importer{
                data.network.files, graph_helper, data.srs.srs_transform, reprojector_variant
        };
        arc_node_importer.read();
    }

    template<typename GraphHelper>
    void _import_seattle(GraphHelper &graph_helper, const prepare_data &data) {
        const auto reprojector_variant = geometry::create_point_reprojector(data.srs.srs_transform);
        io::network::seattle_importer seattle_importer{
                data.network.files, graph_helper, data.srs.srs_transform, reprojector_variant
        };
        seattle_importer.read();
    }

    template<typename GraphHelper>
    void _import_melbourne(GraphHelper &graph_helper, const prepare_data &data) {
        const auto reprojector_variant = geometry::create_point_reprojector(data.srs.srs_transform);
        io::network::melbourne_importer melbourne_importer{
                data.network.files, graph_helper, data.srs.srs_transform, reprojector_variant
        };
        melbourne_importer.read();
    }

    template<typename GraphHelper>
    void _import_giscup(GraphHelper &graph_helper, const prepare_data &data) {
        const auto reprojector_variant = geometry::create_point_reprojector(data.srs.srs_transform);
        io::network::giscup_importer giscup_importer{
                data.network.files, graph_helper, data.srs.srs_transform, reprojector_variant
        };
        giscup_importer.read();
    }

    template<typename GraphHelper>
    void _execute_import(GraphHelper &graph_helper, const prepare_data &data) {
        verbose_frame("Import network", [&]() {
            if (data.network.format == FORMAT_OSM) {
                _import_osm(graph_helper, data);
            } else if (data.network.format == FORMAT_ARC_NODE) {
                _import_arc_node(graph_helper, data);
            } else if (data.network.format == FORMAT_SEATTLE) {
                _import_seattle(graph_helper, data);
            } else if (data.network.format == FORMAT_MELBOURNE) {
                _import_melbourne(graph_helper, data);
            } else if (data.network.format == FORMAT_GISCUP) {
                _import_giscup(graph_helper, data);
            }
        });
    }

    template<typename NetworkImport>
    void _simplify(std::unique_ptr<NetworkImport> &network_import, const prepare_data &data) {
        if (data.process.simplify != SIMPLIFY_NONE and
            (data.process.simplify == SIMPLIFY_ALL or data.process.simplify == SIMPLIFY_OSM_AWARE)) {
            verbose_frame("Simplify network", [&]() {
                network_import->graph_helper().simplify(*network_import, data.process.simplify == SIMPLIFY_ALL);
            });
        }
    }

    template<typename NetworkImport>
    void _remove_unconnected(std::unique_ptr<NetworkImport> &network_import, const prepare_data &data) {
        if (data.process.remove_unconnected) {
            verbose_frame("Remove unconnected", [&]() {
                network_import->remove_unconnected_components();
            });
        }
    }

    template<typename NetworkImport>
    void _remove_duplicate_nodes(std::unique_ptr<NetworkImport> &network_import, const prepare_data &data) {
        if (data.process.remove_duplicate_nodes) {
            verbose_frame("Remove duplicate nodes", [&]() {
                network_import->remove_duplicate_nodes(
                        data.process.simplify != SIMPLIFY_NONE,
                        data.process.simplify == SIMPLIFY_ALL);
            });
        }
    }

    template<typename NetworkImport>
    void _remove_duplicate_edges(std::unique_ptr<NetworkImport> &network_import, const prepare_data &data) {
        if (data.process.remove_duplicate_edges) {
            verbose_frame("Remove duplicate edges", [&]() {
                network_import->remove_duplicate_edges();
            });
        }
    }

    template<typename Network, typename NetworkImport>
    void _compress(std::unique_ptr<Network> &network, std::unique_ptr<NetworkImport> &network_import) {
        verbose_frame("Compress network", [&]() {
            network->graph_helper().from_adjacency_list(network_import->graph_helper(), network_import->tag_helper());
        });
    }

    template<typename Network, typename NetworkImport>
    void _finish_import(std::unique_ptr<Network> &network, std::unique_ptr<NetworkImport> &network_import,
            const prepare_data &data) {
        _compress(network, network_import);
        network_import.reset();
    }

    template<typename Network>
    void _build_spatial_indices(std::unique_ptr<Network> &network) {
        verbose_frame("Build indices", [&]() {
            network->build_spatial_indices();
        });
    }

    template<typename Network>
    void _serialize(std::unique_ptr<Network> &network, const prepare_data &data) {
        if (not data.memory.mmap_tags or not data.memory.mmap_indices or not data.memory.mmap_graph) {
            verbose_frame("Serialize", [&]() {
                save_network(*network, data);
            });
        }
    }

    template<typename Network>
    void _export(std::unique_ptr<Network> &network, const prepare_data &data) {
        if (not data.expo.export_nodes_csv_file.empty()) {
            verbose_frame("Exporting Nodes", [&]() {
                std::filesystem::path output{data.network.output};
                network->save_nodes_csv((output / data.expo.export_nodes_csv_file).string());
            });
        }

        if (not data.expo.export_edges_csv_file.empty()) {
            verbose_frame("Exporting Edges", [&]() {
                std::filesystem::path output{data.network.output};
                network->save_edges_csv((output / data.expo.export_edges_csv_file).string());
            });
        }

        if (not data.expo.export_network_osm_file.empty()) {
            verbose_frame("Exporting OSM", [&]() {
                std::filesystem::path output{data.network.output};

                // back-converting srs-transform
                geometry::srs_transform _srs_transform = geometry::compute_next_srs_transform(
                        data.srs.srs_transform, 4326, geometry::SRS::CS_GEOGRAPHIC);

                io::network::osm_exporter osm_exporter{
                        (output / data.expo.export_network_osm_file).string(),
                        network->graph(), network->tag_helper(),
                        geometry::create_point_reprojector(_srs_transform)
                };
                osm_exporter.write();
            });
        }
    }

    template<typename Network>
    void _finish_preparation(std::unique_ptr<Network> &network, const prepare_data &data) {
        _build_spatial_indices(network);
        _serialize(network, data);
        _export(network, data);
        network.reset();
    }

    template<typename NetworkImport>
    void _import_network(std::unique_ptr<NetworkImport> &network_import, const prepare_data &data) {
        _execute_import(network_import->graph_helper(), data);
        _simplify(network_import, data);
        _remove_unconnected(network_import, data);
        _remove_duplicate_nodes(network_import, data);
        _remove_duplicate_edges(network_import, data);

        using coordinate_system_type =
                typename geometry::network::network_traits<NetworkImport>::coordinate_system_type;
        auto network_variant = create_compatible_network<coordinate_system_type>(data);

        std::visit([&](auto &&network) {
            _finish_import(network, network_import, data);
            _finish_preparation(network, data);
        }, std::move(network_variant));
    }

    void _prepare(const prepare_data &data);

    int prepare(int argc, char *argv[]);

}

#endif //MAP_MATCHING_2_APP_PREPARE_HPP
