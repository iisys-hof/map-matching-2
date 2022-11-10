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

#ifndef MAP_MATCHING_2_NETWORK_HPP
#define MAP_MATCHING_2_NETWORK_HPP

#include <absl/container/flat_hash_set.h>
#include <absl/container/flat_hash_map.h>
#include <absl/container/btree_set.h>

#ifndef BOOST_THREAD_VERSION
#define BOOST_THREAD_VERSION 5
#endif

#include <boost/thread.hpp>

#include <boost/serialization/serialization.hpp>

#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/srs/epsg.hpp>
#include <boost/geometry/srs/projection.hpp>
#include <boost/geometry/srs/transformation.hpp>


#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <geometry/util.hpp>
#include <io/csv_exporter.hpp>

#include "graph.hpp"
#include "node.hpp"
#include "edge.hpp"
#include "route.hpp"

namespace map_matching_2::geometry::network {

    template<typename GraphStorage>
    class network_base {

    public:
        using graph_storage_type = GraphStorage;
        using graph_type = typename graph_storage_type::graph_type;
        using node_type = typename graph_storage_type::node_type;
        using edge_type = typename graph_storage_type::edge_type;
        using graph_traits = typename boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        using rich_line_type = typename edge_type::rich_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        network_base()
                : network_base{graph_storage_type{}} {}

        explicit network_base(graph_storage_type graph_storage)
                : _graph_storage{std::move(graph_storage)} {}

        ~network_base() = default;

        network_base(const network_base &other) = default;

        network_base(network_base &&other) noexcept = default;

        network_base &operator=(const network_base &other) = default;

        network_base &operator=(network_base &&other) noexcept = default;

        [[nodiscard]] graph_storage_type &graph_storage() const {
            return _graph_storage;
        }

        [[nodiscard]] graph_type &graph() const {
            return _graph_storage.graph();
        }

        [[nodiscard]] node_type &vertex(const vertex_descriptor &v) const {
            return _graph_storage.vertex(v);
        }

        [[nodiscard]] edge_type &edge(const edge_descriptor &e) const {
            return _graph_storage.edge(e);
        }

        [[nodiscard]] class tag_helper &tag_helper() const {
            return _tag_helper;
        }

        [[nodiscard]] length_type length() const {
            length_type length = geometry::default_float_type<length_type>::v0;
            for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(graph()))) {
                const auto &edge = this->edge(edge_descriptor);
                if (edge.has_length()) {
                    length += edge.length();
                }
            }
            return length;
        }

        [[nodiscard]] std::vector<vertex_descriptor> make_predecessors() const {
            std::vector<vertex_descriptor> predecessors;
            predecessors.reserve(boost::num_vertices(graph()));
            for (const auto &vertex: boost::make_iterator_range(boost::vertices(graph()))) {
                predecessors.emplace_back(vertex);
            }
            return predecessors;
        }

        [[nodiscard]] std::vector<length_type> make_distances() const {
            std::vector<length_type> distances;
            distances.reserve(boost::num_vertices(graph()));
            for (const auto &vertex: boost::make_iterator_range(boost::vertices(graph()))) {
                distances.emplace_back(std::numeric_limits<length_type>::infinity());
            }
            return distances;
        }

        [[nodiscard]] std::vector<typename boost::default_color_type> make_colors() const {
            std::vector<typename boost::default_color_type> colors;
            colors.reserve(boost::num_vertices(graph()));
            for (const auto &vertex: boost::make_iterator_range(boost::vertices(graph()))) {
                colors.emplace_back(boost::white_color);
            }
            return colors;
        }

        [[nodiscard]] std::vector<std::size_t> make_heap_index() const {
            std::vector<std::size_t> heap_index;
            heap_index.reserve(boost::num_vertices(graph()));
            for (const auto &vertex: boost::make_iterator_range(boost::vertices(graph()))) {
                heap_index.emplace_back(0);
            }
            return heap_index;
        }

        [[nodiscard]] std::vector<vertex_descriptor> make_queue() const {
            std::vector<vertex_descriptor> queue;
            queue.reserve(boost::num_vertices(graph()));
            return queue;
        }

        template<typename NetworkConverted>
        void convert(NetworkConverted &network_converted) {
            network_converted.graph_storage() = graph_storage();
            network_converted.tag_helper() = tag_helper();
        }

        template<typename NetworkReprojected>
        void reproject(NetworkReprojected &network_reprojected,
                       const boost::geometry::srs::proj4 &from, const boost::geometry::srs::proj4 &to) {
            boost::geometry::srs::transformation<> transformation{from, to};

            vertex_reprojector<typename NetworkReprojected::graph_type>
                    _vertex_reprojector{graph(), network_reprojected.graph(), transformation};
            edge_reprojector<typename NetworkReprojected::graph_type>
                    _edge_reprojector{graph(), network_reprojected.graph(), transformation};

            boost::copy_graph(graph(), network_reprojected.graph(),
                              boost::vertex_index_map(_graph_storage.vertex_index_map()).
                                      vertex_copy(_vertex_reprojector).
                                      edge_copy(_edge_reprojector));
            network_reprojected.tag_helper() = tag_helper();
            network_reprojected.graph_storage().set_maps();
            network_reprojected.graph_storage().remap();
        }

        void save_nodes_csv(std::string filename) const {
            BOOST_CONCEPT_ASSERT((boost::VertexListGraphConcept<graph_type>));

            io::csv_exporter<',', '"'> csv{std::move(filename)};
            bool first = true;
            for (const auto &vertex: boost::make_iterator_range(boost::vertices(graph()))) {
                const auto &node = this->vertex(vertex);
                if (first) {
                    csv << node.header();
                    first = false;
                }
                csv << node.row(_tag_helper);
            }
        }

        void save_edges_csv(std::string filename) const {
            BOOST_CONCEPT_ASSERT((boost::EdgeListGraphConcept<graph_type>));

            io::csv_exporter<',', '"'> csv{std::move(filename)};
            bool first = true;
            for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(graph()))) {
                const auto &edge = this->edge(edge_descriptor);
                if (first) {
                    csv << edge.header();
                    first = false;
                }
                csv << edge.row(_tag_helper);
            }
        }

        void save_csv(std::string nodes, std::string edges) const {
            save_nodes_csv(std::move(nodes));
            save_edges_csv(std::move(edges));
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void save(Archive &ar, const unsigned int version) const {
            ar << _graph_storage;
            ar << _tag_helper;
        }

        template<typename Archive>
        void load(Archive &ar, const unsigned int version) {
            ar >> _graph_storage;
            ar >> _tag_helper;
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()

    protected:
        mutable graph_storage_type _graph_storage;

        mutable class tag_helper _tag_helper;

        template<typename GraphReprojected>
        struct vertex_reprojector {
            using original_vertex_descriptor = typename graph_type::vertex_descriptor;
            using original_point_type = typename graph_type::vertex_property_type::point_type;
            using reprojected_vertex_descriptor = typename GraphReprojected::vertex_descriptor;
            using reprojected_point_type = typename GraphReprojected::vertex_property_type::point_type;

            vertex_reprojector(const graph_type &graph_original, GraphReprojected &graph_reprojected,
                               const boost::geometry::srs::transformation<> &transformation)
                    : vertex_all_map_original(boost::get(boost::vertex_all, graph_original)),
                      vertex_all_map_reprojected(boost::get(boost::vertex_all, graph_reprojected)),
                      transformation{transformation} {}

            void operator()(const original_vertex_descriptor &vertex_original,
                            reprojected_vertex_descriptor &vertex_reprojected) const {
                const auto &node_original = boost::get(vertex_all_map_original, vertex_original);

                const original_point_type &point_original = node_original.point;
                reprojected_point_type point_reprojected;

                transformation.forward(point_original, point_reprojected);

                typename GraphReprojected::vertex_property_type node_reprojected
                        {node_original.id, std::move(point_reprojected), node_original.tags};
                node_reprojected.index = node_original.index;

                put(vertex_all_map_reprojected, vertex_reprojected, node_reprojected);
            }

            typename boost::property_map<graph_type, boost::vertex_all_t>::const_type vertex_all_map_original;
            mutable typename boost::property_map<GraphReprojected, boost::vertex_all_t>::type vertex_all_map_reprojected;
            const boost::geometry::srs::transformation<> &transformation;
        };

        template<typename GraphReprojected>
        struct edge_reprojector {
            using original_edge_descriptor = typename graph_type::edge_descriptor;
            using original_line_type = typename graph_type::edge_property_type::line_type;
            using reprojected_edge_descriptor = typename GraphReprojected::edge_descriptor;
            using reprojected_line_type = typename GraphReprojected::edge_property_type::line_type;

            edge_reprojector(const graph_type &graph_original, GraphReprojected &graph_reprojected,
                             const boost::geometry::srs::transformation<> &transformation)
                    : edge_all_map_original{boost::get(boost::edge_all, graph_original)},
                      edge_all_map_reprojected{boost::get(boost::edge_all, graph_reprojected)},
                      transformation{transformation} {}

            void operator()(const original_edge_descriptor &edge_descriptor_original,
                            reprojected_edge_descriptor &edge_descriptor_reprojected) const {
                const auto &edge_original = boost::get(
                        edge_all_map_original,
                        boost::detail::add_reverse_edge_descriptor<original_edge_descriptor, graph_type>::convert(
                                edge_descriptor_original));

                const original_line_type &line_original = edge_original.line();
                reprojected_line_type line_reprojected;
                line_reprojected.reserve(line_original.size());

                transformation.forward(line_original, line_reprojected);

                typename GraphReprojected::edge_property_type edge_reprojected
                        {edge_original.id, std::move(line_reprojected), edge_original.tags};
                edge_reprojected.index = edge_original.index;

                boost::put(edge_all_map_reprojected, edge_descriptor_reprojected, edge_reprojected);
            }

            typename boost::property_map<graph_type, boost::edge_all_t>::const_type edge_all_map_original;
            mutable typename boost::property_map<GraphReprojected, boost::edge_all_t>::type edge_all_map_reprojected;
            const boost::geometry::srs::transformation<> &transformation;
        };

    };

    template<typename GraphStorage>
    class network_import : public network_base<GraphStorage> {

    public:
        using graph_storage_type = GraphStorage;
        using base_type = network_base<graph_storage_type>;
        using graph_type = typename graph_storage_type::graph_type;
        using node_type = typename graph_storage_type::node_type;
        using edge_type = typename graph_storage_type::edge_type;
        using graph_traits = typename boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        using rich_line_type = typename edge_type::rich_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        network_import()
                : base_type{} {}

        explicit network_import(graph_storage_type graph_storage)
                : base_type{std::move(graph_storage)} {}

        ~network_import() = default;

        network_import(const network_import &other) = default;

        network_import(network_import &&other) noexcept = default;

        network_import &operator=(const network_import &other) = default;

        network_import &operator=(network_import &&other) noexcept = default;

        vertex_descriptor add_vertex(const node_type &node) {
            return this->_graph_storage.add_vertex(node);
        }

        std::pair<edge_descriptor, bool>
        add_edge(const vertex_descriptor &u, const vertex_descriptor &v, const edge_type &edge) {
            return this->_graph_storage.add_edge(u, v, edge);
        }

        void simplify(bool simplify_network_complete = true) {
            BOOST_CONCEPT_ASSERT((boost::VertexAndEdgeListGraphConcept<graph_type>));
            BOOST_CONCEPT_ASSERT((boost::BidirectionalGraphConcept<graph_type>));

            typename graph_traits::vertex_iterator vertex_it, vertex_next, vertex_end;
            std::tie(vertex_it, vertex_end) = boost::vertices(this->graph());
            for (vertex_next = vertex_it; vertex_it != vertex_end; vertex_it = vertex_next) {
                vertex_next++;
                const auto &vertex = *vertex_it;

                const auto degree = boost::degree(vertex, this->graph());
                const auto in_degree = boost::in_degree(vertex, this->graph());
                const auto out_degree = boost::out_degree(vertex, this->graph());

                if ((degree == 2 or degree == 4) and in_degree == out_degree) {
                    // we can only merge when the in and out degree is equal
                    const auto adj_vertices = _adjacent_vertices(vertex);
                    if (_adjacent_mergable(adj_vertices, vertex)) {
                        // only continue if we are within a road or at a dead end
                        std::vector<std::pair<edge_descriptor, edge_descriptor>> merge_edges;
                        merge_edges.reserve(out_degree);

                        for (const auto &in_edge: boost::make_iterator_range(boost::in_edges(vertex, this->graph()))) {
                            for (const auto &out_edge: boost::make_iterator_range(
                                    boost::out_edges(vertex, this->graph()))) {
                                if (degree == 2) {
                                    merge_edges.emplace_back(std::pair{in_edge, out_edge});
                                } else {
                                    if (boost::source(in_edge, this->graph()) !=
                                        boost::target(out_edge, this->graph())) {
                                        merge_edges.emplace_back(std::pair{in_edge, out_edge});
                                    }
                                }
                            }
                        }

                        for (const auto &merge_edge: merge_edges) {
                            _merge(merge_edge.first, merge_edge.second, simplify_network_complete);
                        }
                    }
                }
            }

            this->_graph_storage.reindex();
        }

        void remove_unconnected_components() {
            auto index_map = this->_graph_storage.vertex_index_map();

            auto colors = this->make_colors();
            auto color_map = boost::make_iterator_property_map(colors.begin(), index_map);

            absl::flat_hash_set<vertex_descriptor> visited;
            std::vector<absl::flat_hash_set<vertex_descriptor>> components;

            for (const auto &vertex: boost::make_iterator_range(boost::vertices(this->graph()))) {
                if (not visited.contains(vertex)) {
                    auto &discovered = components.emplace_back(absl::flat_hash_set<vertex_descriptor>{});
                    vertex_visitor vertex_visitor{visited, discovered};

                    boost::breadth_first_visit(this->graph(), vertex,
                                               boost::vertex_index_map(index_map)
                                                       .color_map(color_map)
                                                       .visitor(vertex_visitor));
                }
            }

            const auto largest_component_it = std::max_element(components.cbegin(), components.cend(),
                                                               [](const auto &a, const auto &b) {
                                                                   return a.size() < b.size();
                                                               });
            const auto &largest_component = *largest_component_it;

            // found largest weakly component, remove all remaining nodes and edges
            for (const auto &component_nodes: components) {
                if (&component_nodes != &largest_component) {
                    // only remove nodes not from the largest component
                    for (const auto &vertex_descriptor: component_nodes) {
                        this->_graph_storage.clear_vertex(vertex_descriptor);
                        this->_graph_storage.remove_vertex(vertex_descriptor);
                    }
                }
            }

            this->_graph_storage.reindex();
        }

    private:
        void _merge(edge_descriptor in_edge, edge_descriptor out_edge, bool simplify_network_complete = true) {
            BOOST_CONCEPT_ASSERT((boost::VertexMutablePropertyGraphConcept<graph_type>));
            BOOST_CONCEPT_ASSERT((boost::EdgeMutablePropertyGraphConcept<graph_type>));
            BOOST_CONCEPT_ASSERT((boost::MutableBidirectionalGraphConcept<graph_type>));

            if (in_edge != out_edge) {
                // only merge if edges are not the same
                const auto &source_vertex_in = boost::source(in_edge, this->graph());
                const auto &target_vertex_in = boost::target(in_edge, this->graph());
                const auto &source_vertex_out = boost::source(out_edge, this->graph());
                const auto &target_vertex_out = boost::target(out_edge, this->graph());

                if (target_vertex_in == source_vertex_out and source_vertex_in != target_vertex_out) {
                    // check that edges can be merged in same vertex
                    // a oneway loop could end in itself, only continue if we have not this situation
                    auto &source_edge = this->edge(in_edge);
                    auto &target_edge = this->edge(out_edge);

                    if (simplify_network_complete or
                        (source_edge.id == target_edge.id and source_edge.tags == target_edge.tags)) {
                        // only merge edges when they fit together concerning their id and tag_helper_ref
                        // this means we split when the tag_helper_ref change, for example maxspeed change within road
                        if (not source_edge.empty() and not target_edge.empty()) {
                            // only continue if there is something to merge
                            edge_type new_edge = edge_type::merge(source_edge, target_edge);

                            this->_graph_storage.remove_edge(in_edge);
                            this->_graph_storage.remove_edge(out_edge);
                            add_edge(source_vertex_in, target_vertex_out, new_edge);

                            if (boost::degree(target_vertex_in, this->graph()) == 0) {
                                // only remove vertex if all edges were previously removed
                                this->_graph_storage.remove_vertex(target_vertex_in);
                            }
                        }
                    }
                }
            }
        }

        [[nodiscard]] absl::flat_hash_map<osmium::object_id_type, std::size_t>
        _adjacent_vertices(const vertex_descriptor &vertex) const {
            BOOST_CONCEPT_ASSERT((boost::VertexAndEdgeListGraphConcept<graph_type>));
            BOOST_CONCEPT_ASSERT((boost::BidirectionalGraphConcept<graph_type>));

            absl::flat_hash_map<osmium::object_id_type, std::size_t> adj_vertices_map;
            const auto emplace = [&adj_vertices_map](const osmium::object_id_type id) {
                auto search = adj_vertices_map.find(id);
                if (search == adj_vertices_map.end()) {
                    adj_vertices_map.emplace(id, 1);
                } else {
                    search->second++;
                }
            };

            for (const auto &in_edge: boost::make_iterator_range(boost::in_edges(vertex, this->graph()))) {
                const auto &source_vertex = boost::source(in_edge, this->graph());
                emplace(this->vertex(source_vertex).id);
            }

            for (const auto &out_edge: boost::make_iterator_range(boost::out_edges(vertex, this->graph()))) {
                const auto &target_vertex = boost::target(out_edge, this->graph());
                emplace(this->vertex(target_vertex).id);
            }

            return adj_vertices_map;
        }

        [[nodiscard]] bool _adjacent_mergable(
                const absl::flat_hash_map<osmium::object_id_type, std::size_t> &adjacent_vertices,
                const vertex_descriptor &vertex) const {
            bool adj_vertices_okay = adjacent_vertices.size() <= 2;

            if (adj_vertices_okay and adjacent_vertices.size() > 1) {
                for (const auto &adj: adjacent_vertices) {
                    if (adj.second > 2) {
                        // special situation when a vertex has two out and two in but three of these edges point to the same vertex
                        adj_vertices_okay = false;
                        break;
                    }
                }
            }

            if (adj_vertices_okay and adjacent_vertices.size() > 1) {
                // special situation with degree 4 where both sources have the same source vertex
                absl::flat_hash_set<vertex_descriptor> source_vertices;
                for (const auto &in_edge: boost::make_iterator_range(boost::in_edges(vertex, this->graph()))) {
                    const auto &source_vertex = boost::source(in_edge, this->graph());
                    if (source_vertices.contains(source_vertex)) {
                        adj_vertices_okay = false;
                        break;
                    } else {
                        source_vertices.emplace(source_vertex);
                    }
                }
            }

            if (adj_vertices_okay and adjacent_vertices.size() > 1) {
                // special situation with degree 4 where both targets have the same target vertex
                absl::flat_hash_set<vertex_descriptor> target_vertices;
                for (const auto &out_edge: boost::make_iterator_range(boost::out_edges(vertex, this->graph()))) {
                    const auto &target_vertex = boost::target(out_edge, this->graph());
                    if (target_vertices.contains(target_vertex)) {
                        adj_vertices_okay = false;
                        break;
                    } else {
                        target_vertices.emplace(target_vertex);
                    }
                }
            }

            return adj_vertices_okay;
        }

        class vertex_visitor : public boost::default_bfs_visitor {

        private:
            absl::flat_hash_set<vertex_descriptor> &_visited;
            absl::flat_hash_set<vertex_descriptor> &_discovered;

        public:
            vertex_visitor(absl::flat_hash_set<vertex_descriptor> &visited,
                           absl::flat_hash_set<vertex_descriptor> &discovered)
                    : _visited{visited}, _discovered{discovered} {}

            template<typename graph_type_inner>
            void discover_vertex(vertex_descriptor &vertex, const graph_type_inner &_graph) {
                _visited.emplace(vertex);
                _discovered.emplace(vertex);
            }

        };

    };

    template<typename GraphStorage>
    class network : public network_base<GraphStorage> {

    public:
        using graph_storage_type = GraphStorage;
        using base_type = network_base<graph_storage_type>;
        using graph_type = typename graph_storage_type::graph_type;
        using node_type = typename graph_storage_type::node_type;
        using edge_type = typename graph_storage_type::edge_type;
        using graph_traits = typename boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        using rich_line_type = typename edge_type::rich_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        using route_type = route<rich_line_type>;

        using node_pair_type = std::pair<vertex_descriptor, node_type>;
        using edge_pair_type = std::pair<edge_descriptor, edge_type>;

        using rtree_points_type = boost::geometry::index::rtree<
                std::pair<point_type, std::size_t>, boost::geometry::index::rstar<16>>;
        using rtree_segments_type = boost::geometry::index::rtree<
                std::pair<segment_type, std::pair<std::size_t, std::size_t>>, boost::geometry::index::rstar<16>>;

        network()
                : base_type{} {}

        explicit network(graph_storage_type graph_storage)
                : base_type{std::move(graph_storage)} {};

        ~network() = default;

        network(const network &other) = default;

        network(network &&other) noexcept = default;

        network &operator=(const network &other) = default;

        network &operator=(network &&other) noexcept = default;

        void rebuild_spatial_indices() {
            rebuild_maps();

            const auto build_points_index = [&]() {
                std::vector<std::pair<point_type, std::size_t>> points;
                points.reserve(boost::num_vertices(this->graph()));
                for (const auto &vertex: boost::make_iterator_range(boost::vertices(this->graph()))) {
                    points.emplace_back(
                            std::pair{this->vertex(vertex).point, this->graph_storage().vertex_index(vertex)});
                }

                _points_index = rtree_points_type{points};
            };

            const auto build_segments_index = [&]() {
                std::vector<std::pair<segment_type, std::pair<std::size_t, std::size_t>>> segments;
                segments.reserve(boost::num_edges(this->graph()));
                for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(this->graph()))) {
                    const auto &edge = this->edge(edge_descriptor);
                    for (std::size_t i = 0; i < edge.rich_segments().size(); ++i) {
                        const auto &rich_segment = edge.rich_segments()[i];
                        segments.emplace_back(std::pair{
                                rich_segment.segment(),
                                std::pair{this->graph_storage().edge_index(edge_descriptor), i}});
                    }
                }

                _segments_index = rtree_segments_type{segments};
            };

            std::uint32_t threads = boost::thread::hardware_concurrency();
            if (threads >= 2) {
                boost::thread points_index_thread{build_points_index};
                boost::thread segments_index_thread{build_segments_index};

                points_index_thread.join();
                segments_index_thread.join();
            } else {
                build_points_index();
                build_segments_index();
            }
        }

        void rebuild_maps() {
            std::size_t vertices = boost::num_vertices(this->graph());
            _vertex_map.clear();
            _vertex_map.resize(vertices);
            for (const auto &vertex: boost::make_iterator_range(boost::vertices(this->graph()))) {
                _vertex_map[this->graph_storage().vertex_index(vertex)] = vertex;
            }

            std::size_t edges = boost::num_edges(this->graph());
            _edge_map.clear();
            _edge_map.resize(edges);
            for (const auto &edge: boost::make_iterator_range(boost::edges(this->graph()))) {
                _edge_map[this->graph_storage().edge_index(edge)] = edge;
            }
        }

        [[nodiscard]] std::vector<vertex_descriptor> find_node(osmium::object_id_type node_id) const {
            std::vector<vertex_descriptor> nodes;
            for (const auto &vertex: boost::make_iterator_range(boost::vertices(this->graph()))) {
                const auto &node = this->vertex(vertex);
                if (node.id == node_id) {
                    nodes.emplace_back(vertex);
                }
            }
            return nodes;
        }

        [[nodiscard]] std::vector<edge_descriptor> find_edge(osmium::object_id_type edge_id) const {
            std::vector<edge_descriptor> edges;
            for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(this->graph()))) {
                const auto &edge = this->edge(edge_descriptor);
                if (edge.id == edge_id) {
                    edges.emplace_back(edge_descriptor);
                }
            }
            return edges;
        }

        [[nodiscard]] std::pair<bool, vertex_descriptor> point_in_nodes(const point_type &point) const {
            const auto nearest_points = query_points(boost::geometry::index::nearest(point, 1));
            if (not nearest_points.empty()) {
                const auto &nearest_point_pair = nearest_points.front();
                const auto &nearest_point = nearest_point_pair.first;
                bool found = geometry::equals_points(nearest_point, point);
                const auto &vertex = _vertex_map[nearest_point_pair.second];
                return std::pair{found, vertex};
            }
            return std::pair{false, vertex_descriptor{}};
        }

        [[nodiscard]] std::pair<bool, std::pair<std::size_t, std::size_t>>
        point_in_edges(const point_type &point) const {
            const auto nearest_segments = query_segments(boost::geometry::index::nearest(point, 1));
            if (not nearest_segments.empty()) {
                const auto &nearest_segment_pair = nearest_segments.front();
                const auto &nearest_segment = nearest_segment_pair.first;
                bool found = geometry::equals_points(nearest_segment.first, point) or
                             geometry::equals_points(nearest_segment.second, point);
                const auto edge_pair = nearest_segment_pair.second;
                return std::pair{found, edge_pair};
            }
            return std::pair{false, std::pair<std::size_t, std::size_t>{}};
        }

        [[nodiscard]] std::pair<bool, edge_descriptor> fitting_edge(const segment_type &segment) const {
            using angle_type = typename edge_type::angle_type;
            angle_type best_angle = geometry::default_float_type<angle_type>::v180;
            const auto azimuth = geometry::azimuth_segment_deg(segment);
            const edge_descriptor *edge;
            const segment_type *best_segment = nullptr;

            const auto segments = query_segments(boost::geometry::index::intersects(segment));
            for (const auto &segment_pair: segments) {
                const segment_type &near_segment = segment_pair.first;
                const auto near_azimuth = geometry::azimuth_segment_deg(near_segment);
                const auto angle_diff = std::fabs(geometry::angle_diff(azimuth, near_azimuth));
                if (best_segment == nullptr or
                    (geometry::equals_points(segment.first, near_segment.first) and
                     geometry::equals_points(segment.second, near_segment.second)) or angle_diff < best_angle) {
                    best_angle = angle_diff;
                    best_segment = &near_segment;
                    edge = &_edge_map[segment_pair.second.first];
                }
            }

            if (best_segment != nullptr) {
                return std::pair{true, *edge};
            }
            return std::pair{false, edge_descriptor{}};
        }

        [[nodiscard]] std::vector<std::list<vertex_descriptor>>
        dijkstra_shortest_paths(const vertex_descriptor &start, const absl::flat_hash_set<vertex_descriptor> &goals,
                                double max_distance_factor, distance_type max_distance,
                                std::vector<vertex_descriptor> &vertices, std::vector<vertex_descriptor> &predecessors,
                                std::vector<length_type> &distances,
                                std::vector<typename boost::default_color_type> &colors,
                                std::vector<std::size_t> &heap_index,
                                std::vector<vertex_descriptor> &queue) const {
            auto index_map = this->graph_storage().vertex_index_map();

            auto weight_map = boost::make_transform_value_property_map(
                    [](const edge_type &edge) -> double { return edge.length(); }, this->graph_storage().edge_map());

            auto predecessor_map = boost::make_iterator_property_map(predecessors.begin(), index_map);
            auto distance_map = boost::make_iterator_property_map(distances.begin(), index_map);
            auto color_map = boost::make_iterator_property_map(colors.begin(), index_map);
            auto heap_index_map = boost::make_iterator_property_map(&(*heap_index.begin()), index_map);

            absl::flat_hash_set<vertex_descriptor> goals_found;
            goals_visitor<decltype(index_map), decltype(weight_map), decltype(predecessor_map),
                    decltype(distance_map), decltype(color_map)>
                    goals_visitor{max_distance_factor, max_distance, goals, goals_found, vertices,
                                  index_map, weight_map, predecessor_map, distance_map, color_map};

            auto compare = std::less<length_type>();
            auto combine = std::plus<length_type>();
            auto zero = geometry::default_float_type<length_type>::v0;

            boost::put(distance_map, start, geometry::default_float_type<length_type>::v0);

            try {
                // adapted from boost::dijkstra_shortest_paths_no_init
                boost::d_ary_heap_indirect<vertex_descriptor, 4,
                        decltype(heap_index_map), decltype(distance_map), decltype(compare)>
                        Q(distance_map, heap_index_map, compare, queue);

                boost::detail::dijkstra_bfs_visitor<decltype(goals_visitor), decltype(Q), decltype(weight_map),
                        decltype(predecessor_map), decltype(distance_map), decltype(combine), decltype(compare)>
                        bfs_vis(goals_visitor, Q, weight_map, predecessor_map, distance_map, combine, compare, zero);

                boost::breadth_first_visit(this->graph(), &start, &start + 1, Q, bfs_vis, color_map);
            } catch (goals_found_exception &) {}

            std::vector<std::list<vertex_descriptor>> shortest_paths;
            shortest_paths.reserve(goals_found.size());
            for (const auto &goal: goals_found) {
                std::list<vertex_descriptor> shortest_path;
                for (vertex_descriptor vertex = goal;; vertex = predecessor_map[vertex]) {
                    shortest_path.emplace_front(vertex);
                    if (predecessor_map[vertex] == vertex)
                        break;
                }
                shortest_paths.emplace_back(std::move(shortest_path));
            }

            for (const auto &vertex: vertices) {
                boost::put(predecessor_map, vertex, vertex);
                boost::put(distance_map, vertex, std::numeric_limits<length_type>::infinity());
                boost::put(color_map, vertex, boost::white_color);
                boost::put(heap_index_map, vertex, 0);
            }
            vertices.clear();

            return shortest_paths;
        }

        [[nodiscard]] route_type extract_route(const std::list<vertex_descriptor> &shortest_path) const {
            if (shortest_path.empty()) {
                // return invalid route
                return route_type{true};
            }

            std::vector<std::reference_wrapper<const rich_line_type>> edges;
            edges.reserve(std::distance(shortest_path.cbegin(), shortest_path.cend()) - 1);

            for (auto vertex_it = shortest_path.cbegin(); vertex_it != shortest_path.cend(); ++vertex_it) {
                const auto next_vertex_it = std::next(vertex_it);
                if (next_vertex_it != shortest_path.cend()) {
                    const rich_line_type *edge_ptr = nullptr;
                    length_type shortest_length = std::numeric_limits<length_type>::infinity();
                    for (const auto &out_edge: boost::make_iterator_range(
                            boost::out_edges(*vertex_it, this->graph()))) {
                        if (boost::target(out_edge, this->graph()) == *next_vertex_it) {
                            const auto &edge = this->edge(out_edge);
                            if (not edge.has_length() or edge.length() < shortest_length) {
                                if (edge.has_length()) {
                                    shortest_length = edge.length();
                                }
                                edge_ptr = &edge;
                            }
                        }
                    }

                    if (edge_ptr != nullptr) {
                        edges.emplace_back(std::cref(*edge_ptr));
                    }
                }
            }

            if (edges.empty()) {
                // only one node, so no paths found, empty route for staying here
                return route_type{false};
            }

            if (std::all_of(edges.cbegin(), edges.cend(), [](const rich_line_type &edge) {
                return edge.empty();
            })) {
                // still no line, empty route for staying here
                return route_type{false};
            }

            return route_type{edges};
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<std::pair<point_type, std::size_t>>
        query_points(const Predicate &predicate) const {
            std::vector<std::pair<point_type, std::size_t>> results;
            _points_index.query(predicate, std::back_inserter(results));
            return results;
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<std::pair<segment_type, std::pair<std::size_t, std::size_t>>>
        query_segments(const Predicate &predicate) const {
            std::vector<std::pair<segment_type, std::pair<std::size_t, std::size_t>>> results;
            _segments_index.query(predicate, std::back_inserter(results));
            return results;
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<point_type>
        query_segments_points(const Predicate &predicate) const {
            return _process_segments_points_result(query_segments(predicate));
        }

        template<typename Predicate>
        [[nodiscard]] absl::btree_set<edge_descriptor>
        query_segments_unique(const Predicate &predicate) const {
            return _process_segments_unique_result(query_segments(predicate));
        }

        [[nodiscard]] typename rtree_points_type::bounds_type bounds() const {
            return _points_index.bounds();
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void save(Archive &ar, const unsigned int version) const {
            ar << boost::serialization::base_object<base_type>(*this);
            ar << _points_index;
            ar << _segments_index;
        }

        template<typename Archive>
        void load(Archive &ar, const unsigned int version) {
            ar >> boost::serialization::base_object<base_type>(*this);
            ar >> _points_index;
            ar >> _segments_index;

            rebuild_maps();
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()

    private:
        std::vector<point_type> _points;

        rtree_points_type _points_index;
        rtree_segments_type _segments_index;

        std::vector<vertex_descriptor> _vertex_map;
        std::vector<edge_descriptor> _edge_map;

        [[nodiscard]] std::vector<point_type> _process_segments_points_result(
                std::vector<std::pair<segment_type, std::pair<std::size_t, std::size_t>>> &&pre_results) const {
            std::vector<point_type> results;
            results.reserve(pre_results.size() * 2);
            for (std::size_t i = 0; i < pre_results.size(); ++i) {
                auto &pre_result = pre_results[i];
                results.emplace_back(std::move(pre_result.first.first));
                results.emplace_back(std::move(pre_result.first.second));
            }
            return results;
        }

        [[nodiscard]] absl::btree_set<edge_descriptor> _process_segments_unique_result(
                std::vector<std::pair<segment_type, std::pair<std::size_t, std::size_t>>> &&pre_results) const {
            absl::btree_set<edge_descriptor> results;
            for (const auto &pre_result: pre_results) {
                results.emplace(_edge_map[pre_result.second.first]);
            }
            return results;
        }

        struct goal_found_exception : public std::exception {
        };

        struct goals_found_exception : public std::exception {
        };

        template<typename IndexMap, typename WeightMap, typename PredecessorMap, typename DistanceMap, typename ColorMap>
        class goals_visitor : public boost::default_dijkstra_visitor {

        private:
            const distance_type _max_distance;
            const double _max_distance_factor;

            const absl::flat_hash_set<vertex_descriptor> &_goals;
            absl::flat_hash_set<vertex_descriptor> &_goals_found;
            std::vector<vertex_descriptor> &_vertices;

            const IndexMap &_index_map;
            const WeightMap &_weight_map;
            PredecessorMap &_predecessor_map;
            DistanceMap &_distance_map;
            ColorMap &_color_map;

        public:
            explicit goals_visitor(const distance_type max_distance, const double max_distance_factor,
                                   const absl::flat_hash_set<vertex_descriptor> &goals,
                                   absl::flat_hash_set<vertex_descriptor> &goals_found,
                                   std::vector<vertex_descriptor> &vertices, const IndexMap &index_map,
                                   const WeightMap &weight_map, PredecessorMap &predecessor_map,
                                   DistanceMap &distance_map, ColorMap &color_map)
                    : _max_distance{max_distance}, _max_distance_factor{max_distance_factor}, _goals{goals},
                      _goals_found{goals_found}, _vertices{vertices}, _index_map{index_map}, _weight_map{weight_map},
                      _predecessor_map{predecessor_map}, _distance_map{distance_map}, _color_map{color_map} {}

            void discover_vertex(vertex_descriptor &vertex, const graph_type &_graph) {
                _vertices.emplace_back(vertex);
            }

            void examine_vertex(vertex_descriptor &vertex, const graph_type &_graph) {
                if (_goals.contains(vertex)) {
                    _goals_found.emplace(vertex);
                }

                if (_goals_found.size() >= _goals.size()) {
                    throw goals_found_exception();
                }
            }

            void examine_edge(edge_descriptor edge, const graph_type &_graph) {
                const vertex_descriptor source = boost::source(edge, _graph);
                const auto distance_source = boost::get(_distance_map, source);
                const auto weight = boost::get(_weight_map, edge);
                const auto distance_target = distance_source + weight;
                if (distance_target > _max_distance_factor * _max_distance) {
                    // disable target
                    const vertex_descriptor target = boost::target(edge, _graph);
                    _vertices.emplace_back(target);
                    boost::put(_color_map, target, boost::black_color);
                }
            }

        };

    };

}

#endif //MAP_MATCHING_2_NETWORK_HPP
