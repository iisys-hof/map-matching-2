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

#ifndef MAP_MATCHING_2_IO_NETWORK_OSM_HANDLER_HPP
#define MAP_MATCHING_2_IO_NETWORK_OSM_HANDLER_HPP

#include <osmium/osm.hpp>
#include <osmium/handler.hpp>
#include <osmium/index/map/all.hpp>
#include <osmium/tags/tags_filter.hpp>

namespace map_matching_2::io::network {

    template<typename GraphHelper, typename Index, typename OSMHandlerHelper>
    class osm_handler : public osmium::handler::Handler {

    public:
        using graph_helper_type = GraphHelper;
        using index_type = Index;
        using osm_handler_helper_type = OSMHandlerHelper;

        static constexpr std::size_t empty_value = osmium::index::empty_value<std::size_t>();

        osm_handler(graph_helper_type &graph_helper, geometry::point_reprojector_variant reprojector_variant,
                osm_handler_helper_type osm_handler_helper = {})
            : _graph_helper{graph_helper}, _reprojector_variant{std::move(reprojector_variant)},
            _osm_handler_helper{std::move(osm_handler_helper)} {}

        void set_tags_filters(osmium::TagsFilter query, osmium::TagsFilter filter, osmium::TagsFilter oneway,
                osmium::TagsFilter oneway_reverse) {
            _query = std::move(query);
            _filter = std::move(filter);
            _oneway = std::move(oneway);
            _oneway_reverse = std::move(oneway_reverse);
        }

        void node(const osmium::Node &osm_node) {
            if (osm_node.visible()) {
                const auto node_id = osm_node.id();
                if (_node_index.get_noexcept(static_cast<osmium::unsigned_object_id_type>(node_id)) == empty_value) {
                    const auto index = _osm_handler_helper.add_node(osm_node, _reprojector_variant);
                    _node_index.set(static_cast<osmium::unsigned_object_id_type>(node_id), index);
                }
            }
        }

        void way(const osmium::Way &osm_way) noexcept {
            if (osm_way.visible() and keep(osm_way)) {
                const auto &osm_nodes = osm_way.nodes();

                auto &osm_vertices = _osm_handler_helper.vertices();

                for (const auto &osm_node : osm_nodes) {
                    const auto osm_node_id = osm_node.ref();

                    try {
                        const auto index = _node_index.get(
                                static_cast<osmium::unsigned_object_id_type>(osm_node_id));

                        auto &osm_vertex = osm_vertices[index];

                        _osm_handler_helper.add_vertex(_graph_helper, osm_vertex);
                    } catch (osmium::not_found &ex) {
                        // std::cerr << ex.what() << std::endl;
                    }
                }

                for (std::size_t i = 0; i < osm_nodes.size() - 1; ++i) {
                    const auto node_ref_from = osm_nodes[i].ref();
                    const auto node_ref_to = osm_nodes[i + 1].ref();

                    try {
                        const auto index_from = _node_index.get(
                                static_cast<osmium::unsigned_object_id_type>(node_ref_from));
                        const auto index_to = _node_index.get(
                                static_cast<osmium::unsigned_object_id_type>(node_ref_to));

                        const auto &osm_vertex_from = osm_vertices[index_from];
                        const auto &osm_vertex_to = osm_vertices[index_to];

                        const auto [oneway, reverse] = detect_oneway(osm_way);

                        _osm_handler_helper.add_edge(
                                _graph_helper, osm_vertex_from, osm_vertex_to, oneway, reverse, osm_way);
                    } catch (osmium::not_found &ex) {
                        // std::cerr << ex.what() << std::endl;
                    }
                }
            }
        }

        void relation(const osmium::Relation &relation) noexcept {
            // std::cout << "relation: " << relation.id() << "; ";
        }

    private:
        graph_helper_type &_graph_helper;
        geometry::point_reprojector_variant _reprojector_variant;

        index_type _node_index{};
        osm_handler_helper_type _osm_handler_helper;

        osmium::TagsFilter _query, _filter, _oneway, _oneway_reverse;

        [[nodiscard]] bool keep(const osmium::Way &way) const {
            for (const auto &query_tag : way.tags()) {
                if (_query(query_tag)) {
                    for (const auto &filter_tag : way.tags()) {
                        if (not _filter(filter_tag)) {
                            return false;
                        }
                    }
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] std::tuple<bool, bool> detect_oneway(const osmium::Way &way) const {
            bool oneway = false;
            bool reverse = false;
            for (const auto &tag : way.tags()) {
                if (_oneway(tag)) {
                    oneway = true;
                }
                if (_oneway_reverse(tag)) {
                    reverse = true;
                }
            }
            return std::make_tuple(oneway, reverse);
        }

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_OSM_HANDLER_HPP