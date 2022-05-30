// Copyright (C) 2020-2021 Adrian Wöltche
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

#include "osm_handler.hpp"

#include <geometry/network/types.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    bool osm_handler<Network>::keep(const osmium::Way &way) const {
        for (const auto &query_tag: way.tags()) {
            if (_query(query_tag)) {
                for (const auto &filter_tag: way.tags()) {
                    if (not _filter(filter_tag)) {
                        return false;
                    }
                }
                return true;
            }
        }

        return false;
    }

    template<typename Network>
    std::tuple<bool, bool> osm_handler<Network>::detect_oneway(const osmium::Way &way) const {
        bool oneway = false;
        bool reverse = false;
        for (const auto &tag: way.tags()) {
            if (_oneway(tag)) {
                oneway = true;
            }
            if (_oneway_reverse(tag)) {
                reverse = true;
            }
        }
        return std::make_tuple(oneway, reverse);
    }

    template<typename Network>
    std::vector<std::uint64_t> osm_handler<Network>::tags(const osmium::TagList &tag_list) {
        std::vector<std::uint64_t> tag_vector;
        tag_vector.reserve(tag_list.size());
        for (const auto &tag: tag_list) {
            tag_vector.emplace_back(_network.tag_helper.id(tag.key(), tag.value()));
        }
        std::sort(tag_vector.begin(), tag_vector.end());
        return tag_vector;
    }

    template<typename Network>
    osm_handler<Network>::osm_handler(Network &network, osmium::TagsFilter query, osmium::TagsFilter filter,
                                      osmium::TagsFilter oneway,
                                      osmium::TagsFilter oneway_reverse)
            : _network{network}, _query{std::move(query)}, _filter{std::move(filter)}, _oneway{std::move(oneway)},
              _oneway_reverse{std::move(oneway_reverse)} {}

    template<typename Network>
    void osm_handler<Network>::node(const osmium::Node &osm_node) noexcept {
        if (osm_node.visible()) {
            const auto node_id = osm_node.id();
            if (_node_index.get_noexcept(static_cast<osmium::unsigned_object_id_type>(node_id)) == empty_value) {
                const auto &node_location = osm_node.location();
                std::size_t index = _vertices.size();
                _vertices.emplace_back(vertex_type{node_type{node_id, {node_location.lon(), node_location.lat()},
                                                             this->tags(osm_node.tags())}});
                _node_index.set(static_cast<osmium::unsigned_object_id_type>(node_id), index);
            }
        }
    }

    template<typename Network>
    void osm_handler<Network>::way(const osmium::Way &osm_way) noexcept {
        if (osm_way.visible() and keep(osm_way)) {
            const auto &osm_nodes = osm_way.nodes();
            for (const auto &osm_node: osm_nodes) {
                const auto osm_node_id = osm_node.ref();
                const auto index = _node_index.get(
                        static_cast<osmium::unsigned_object_id_type>(osm_node_id));
                auto &vertex = _vertices[index];
                if (not vertex.added) {
                    vertex.vertex = _network.add_vertex(vertex.node);
                    vertex.added = true;
                }
            }

            const auto tags = this->tags(osm_way.tags());

            for (std::size_t i = 0; i < osm_nodes.size() - 1; ++i) {
                const auto node_ref_a = osm_nodes[i].ref();
                const auto node_ref_b = osm_nodes[i + 1].ref();

                const auto index_a = _node_index.get(
                        static_cast<osmium::unsigned_object_id_type>(node_ref_a));
                const auto index_b = _node_index.get(
                        static_cast<osmium::unsigned_object_id_type>(node_ref_b));

                const auto &vertex_a = _vertices[index_a];
                const auto &vertex_b = _vertices[index_b];

                const auto &node_a = vertex_a.node;
                const auto &node_b = vertex_b.node;

                const auto [oneway, reverse] = detect_oneway(osm_way);

                if (oneway) {
                    if (not reverse) {
                        typename Network::edge_type edge_a{
                                osm_way.id(), {node_a.id, node_b.id}, {node_a.point, node_b.point}, tags};
                        _network.add_edge(vertex_a.vertex, vertex_b.vertex, edge_a);
                    } else {
                        typename Network::edge_type edge_b{
                                osm_way.id(), {node_b.id, node_a.id}, {node_b.point, node_a.point}, tags};
                        _network.add_edge(vertex_b.vertex, vertex_a.vertex, edge_b);
                    }
                } else {
                    typename Network::edge_type edge_a{
                            osm_way.id(), {node_a.id, node_b.id}, {node_a.point, node_b.point}, tags};
                    _network.add_edge(vertex_a.vertex, vertex_b.vertex, edge_a);

                    typename Network::edge_type edge_b{
                            osm_way.id(), {node_b.id, node_a.id}, {node_b.point, node_a.point}, tags};
                    _network.add_edge(vertex_b.vertex, vertex_a.vertex, edge_b);
                }
            }
        }
    }

    template<typename Network>
    void osm_handler<Network>::relation(const osmium::Relation &relation) noexcept {
//            std::cout << "relation: " << relation.id() << "; ";
    }

    template
    class osm_handler<geometry::network::types_geographic::network_modifiable>;

    template
    class osm_handler<geometry::network::types_cartesian::network_modifiable>;

}
