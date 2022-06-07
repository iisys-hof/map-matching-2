// Copyright (C) 2021 Adrian Wöltche
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

#include "osm_exporter.hpp"

#include <osmium/builder/osm_object_builder.hpp>

#include <geometry/network/types.hpp>
#include <geometry/util.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    osm_exporter<Network>::osm_exporter(std::string filename, const Network &network)
            : exporter(std::move(filename)), _network{network} {}

    template<typename Network>
    void osm_exporter<Network>::write() {
        osmium::memory::Buffer buffer{10240, osmium::memory::Buffer::auto_grow::yes};

        // index all points
        osmium::object_id_type index = 0;
        for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(_network.graph))) {
            const auto &edge = _network.graph[edge_descriptor];
            for (const auto &point: edge.line()) {
                const auto x = boost::geometry::get<0>(point);
                const auto y = boost::geometry::get<1>(point);
                point_map_key_type point_map_key{x, y};
                if (not _point_map.contains(point_map_key)) {
                    _point_map.emplace(point_map_key, index++);
                }
            }
        }

        // add nodes
        point_set_type points_added;
        for (const auto vertex_descriptor: boost::make_iterator_range(boost::vertices(_network.graph))) {
            const auto &node = _network.graph[vertex_descriptor];
            add_node(buffer, node, points_added);
            buffer.commit();
        }

        // add remaining points
        for (const auto &point_pair: _point_map) {
            if (not points_added.contains(point_pair.first)) {
                add_point(buffer, point_pair);
                buffer.commit();
            }
        }

        // add ways
        absl::flat_hash_set<std::size_t> edges_reversed;
        for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(_network.graph))) {
            const auto &edge = _network.graph[edge_descriptor];
            bool is_oneway = true;
            if (not edges_reversed.contains(edge.index)) {
                const auto target = boost::target(edge_descriptor, _network.graph);
                for (const auto &out_descriptor: boost::make_iterator_range(boost::out_edges(target, _network.graph))) {
                    const auto &out_edge = _network.graph[out_descriptor];
                    if (geometry::lines_are_reversed(edge.line(), out_edge.line())) {
                        edges_reversed.emplace(out_edge.index);
                        is_oneway = false;
                    }
                }
                add_way(buffer, edge, is_oneway, index++);
                buffer.commit();
            }
        }

        osmium::io::File output_file{filename()};

        osmium::io::Header header;
        header.set("generator", "Map-Matching 2");

        osmium::io::Writer writer{output_file, header, osmium::io::overwrite::allow};
        writer(std::move(buffer));
        writer.close();

        // reset point map with empty one
        _point_map = point_map_type{};
    }

    template<typename Network>
    void osm_exporter<Network>::add_node(osmium::memory::Buffer &buffer, const node_type &node,
                                         point_set_type &points_added) {
        osmium::builder::NodeBuilder node_builder{buffer};

        const auto x = boost::geometry::get<0>(node.point);
        const auto y = boost::geometry::get<1>(node.point);
        point_map_key_type point_map_key{x, y};
        const auto index = _point_map[point_map_key];

        osmium::Node &osm_node = node_builder.object();
        osm_node.set_id(index);
        osmium::Location location{x, y};
        osm_node.set_location(location);

        {
            osmium::builder::TagListBuilder node_tag_builder{node_builder};
            for (const auto &tag: node.tags) {
                const auto tags = _network.tag_helper.tag(tag);
                node_tag_builder.add_tag(tags.first, tags.second);
            }
        }

        points_added.emplace(point_map_key);
    }

    template<typename Network>
    void osm_exporter<Network>::add_point(osmium::memory::Buffer &buffer,
                                          const std::pair<point_map_key_type, std::size_t> &point) {
        osmium::builder::NodeBuilder node_builder{buffer};

        osmium::Node &osm_node = node_builder.object();
        osm_node.set_id(point.second);
        osmium::Location location{point.first.first, point.first.second};
        osm_node.set_location(location);
    }

    template<typename Network>
    void osm_exporter<Network>::add_way(osmium::memory::Buffer &buffer, const edge_type &edge,
                                        const bool is_oneway, const osmium::object_id_type index) {
        osmium::builder::WayBuilder way_builder{buffer};

        osmium::Way &osm_way = way_builder.object();
        osm_way.set_id(index);

        {
            osmium::builder::WayNodeListBuilder way_node_list_builder{way_builder};
            for (const auto &point: edge.line()) {
                const auto x = boost::geometry::get<0>(point);
                const auto y = boost::geometry::get<1>(point);
                point_map_key_type point_map_key{x, y};
                const auto point_index = _point_map[point_map_key];

                osmium::Location location{x, y};
                osmium::NodeRef node_ref{point_index, location};
                way_node_list_builder.add_node_ref(node_ref);
            }
        }

        {
            osmium::builder::TagListBuilder way_tag_builder{way_builder};
            bool has_oneway = false;
            for (const auto &tag: edge.tags) {
                const auto tags = _network.tag_helper.tag(tag);
                if (tags.first == "oneway") {
                    if (tags.second == "yes") {
                        // one way detected, don't add again later
                        has_oneway = true;
                    } else if (tags.second == "-1" or tags.second == "reverse") {
                        // skip reverse tags as they were corrected on import
                        continue;
                    }
                }
                way_tag_builder.add_tag(tags.first, tags.second);
            }
            if (not has_oneway and is_oneway) {
                way_tag_builder.add_tag("oneway", "yes");
            }
        }
    }

    template
    class osm_exporter<geometry::network::types_geographic::network_static>;

    template
    class osm_exporter<geometry::network::types_cartesian::network_static>;

}
