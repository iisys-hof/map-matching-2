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

#ifndef MAP_MATCHING_2_IO_NETWORK_OSM_EXPORTER_HPP
#define MAP_MATCHING_2_IO_NETWORK_OSM_EXPORTER_HPP

#include <utility>

#include <boost/unordered/unordered_flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/any_output.hpp>

#include "types/geometry/reprojector.hpp"

#include "geometry/algorithm/reverse.hpp"
#include "geometry/traits.hpp"

#include "graph/traits/graph.hpp"

#include "../exporter.hpp"

namespace map_matching_2::io::network {

    template<typename Graph, typename TagHelper>
    class osm_exporter : public exporter {

    public:
        using graph_type = Graph;
        using tag_helper_type = TagHelper;

        osm_exporter(std::string filename, const graph_type &graph, const tag_helper_type &tag_helper,
                geometry::point_reprojector_variant reprojector_variant)
            : exporter(std::move(filename)), _graph{graph}, _tag_helper{tag_helper},
            _reprojector_variant{std::move(reprojector_variant)} {}

        void write() {
            osmium::memory::Buffer buffer{10240, osmium::memory::Buffer::auto_grow::yes};

            // index all points
            std::uint64_t index = 0;
            const auto edges_v = _graph.edges_view();
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                for (const auto &point : edge.rich_line.line()) {
                    point_type _point;
                    geometry::reproject_point(point, _point, _reprojector_variant);
                    const auto x = boost::geometry::get<0>(_point);
                    const auto y = boost::geometry::get<1>(_point);
                    point_map_key_type point_map_key{x, y};
                    if (not _point_map.contains(point_map_key)) {
                        _point_map.emplace(point_map_key, index++);
                    }
                }
            }

            // add nodes
            point_set_type points_added;
            const auto vertices_v = _graph.vertices_view();
            for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                const vertex_data_type &vertex = vertices_v[it].get();
                add_node(buffer, vertex, points_added);
                buffer.commit();
            }

            // add remaining points
            for (const auto &point_pair : _point_map) {
                if (not points_added.contains(point_pair.first)) {
                    add_point(buffer, point_pair);
                    buffer.commit();
                }
            }

            // add ways
            boost::unordered::unordered_flat_set<std::size_t> edges_reversed;
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                bool is_oneway = true;
                if (not edges_reversed.contains(_graph.edge_index(it))) {
                    const auto target = _graph.target(it);
                    const auto out_edges_v = _graph.out_edges_view(target);
                    for (auto out_it = out_edges_v.begin();
                         out_it != out_edges_v.end(); out_it = out_edges_v.next(out_it)) {
                        const edge_data_type &out_edge = out_edges_v[out_it].get();
                        if (geometry::lines_are_reversed(edge.rich_line.line(), out_edge.rich_line.line())) {
                            edges_reversed.emplace(_graph.edge_index(out_it));
                            is_oneway = false;
                        }
                    }
                    add_way(buffer, edge, is_oneway, index++);
                    buffer.commit();
                }
            }

            write_buffer(std::move(buffer));

            // reset point map with empty one
            _point_map = point_map_type{};
        }

    private:
        using vertex_data_type = typename graph::graph_traits<graph_type>::vertex_data_type;
        using edge_data_type = typename graph::graph_traits<graph_type>::edge_data_type;

        using point_type = geometry::point_type<geometry::cs_geographic>;
        using coordinate_type = geometry::data<point_type>::coordinate_type;

        using point_map_key_type = std::pair<coordinate_type, coordinate_type>;
        using point_map_type = boost::unordered::unordered_flat_map<point_map_key_type, std::uint64_t>;
        using point_set_type = boost::unordered::unordered_flat_set<point_map_key_type>;

        const graph_type &_graph;
        const tag_helper_type &_tag_helper;
        point_map_type _point_map{};
        geometry::point_reprojector_variant _reprojector_variant;

        void add_node(osmium::memory::Buffer &buffer, const vertex_data_type &node, point_set_type &points_added) {
            osmium::builder::NodeBuilder node_builder{buffer};

            point_type _point;
            geometry::reproject_point(node.point, _point, _reprojector_variant);
            const auto x = boost::geometry::get<0>(_point);
            const auto y = boost::geometry::get<1>(_point);
            point_map_key_type point_map_key{x, y};
            const auto index = _point_map[point_map_key];

            osmium::Node &osm_node = node_builder.object();
            osm_node.set_id(index);
            osmium::Location location{x, y};
            osm_node.set_location(location);

            {
                const auto &tags = _tag_helper.node_tags(node.id);
                if (not tags.empty()) {
                    osmium::builder::TagListBuilder node_tag_builder{node_builder};
                    for (const auto &tag : tags) {
                        const auto pair = _tag_helper.tag(tag);
                        node_tag_builder.add_tag(pair.first, pair.second);
                    }
                }
            }

            points_added.emplace(point_map_key);
        }

        void add_point(osmium::memory::Buffer &buffer, const std::pair<point_map_key_type, std::size_t> &point) {
            osmium::builder::NodeBuilder node_builder{buffer};

            osmium::Node &osm_node = node_builder.object();
            osm_node.set_id(point.second);
            osmium::Location location{point.first.first, point.first.second};
            osm_node.set_location(location);
        }

        void add_way(osmium::memory::Buffer &buffer, const edge_data_type &edge, bool is_oneway, std::uint64_t index) {
            osmium::builder::WayBuilder way_builder{buffer};

            osmium::Way &osm_way = way_builder.object();
            osm_way.set_id(static_cast<osmium::object_id_type>(index));

            {
                osmium::builder::WayNodeListBuilder way_node_list_builder{way_builder};
                for (const auto &point : edge.rich_line.line()) {
                    point_type _point;
                    geometry::reproject_point(point, _point, _reprojector_variant);
                    const auto x = boost::geometry::get<0>(_point);
                    const auto y = boost::geometry::get<1>(_point);
                    point_map_key_type point_map_key{x, y};
                    const auto point_index = _point_map[point_map_key];

                    osmium::Location location{x, y};
                    osmium::NodeRef node_ref{static_cast<osmium::object_id_type>(point_index), location};
                    way_node_list_builder.add_node_ref(node_ref);
                }
            }

            {
                osmium::builder::TagListBuilder way_tag_builder{way_builder};
                bool has_oneway = false;
                const auto &tags = _tag_helper.edge_tags(edge.id);
                for (const auto &tag : tags) {
                    const auto pair = _tag_helper.tag(tag);
                    if (pair.first == "oneway") {
                        if (pair.second == "yes") {
                            // one way detected, don't add again later
                            has_oneway = true;
                        } else if (pair.second == "-1" or pair.second == "reverse") {
                            // skip reverse tags as they were corrected on import
                            continue;
                        }
                    }
                    way_tag_builder.add_tag(pair.first, pair.second);
                }
                if (not has_oneway and is_oneway) {
                    way_tag_builder.add_tag("oneway", "yes");
                }
            }
        }

        void write_buffer(osmium::memory::Buffer &&buffer) const {
            osmium::io::File output_file{filename()};

            osmium::io::Header header;
            header.set("generator", "Map-Matching 2");

            osmium::io::Writer writer{output_file, header, osmium::io::overwrite::allow};
            writer(std::move(buffer));
            writer.close();
        }

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_OSM_EXPORTER_HPP
