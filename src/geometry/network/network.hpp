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

#include <boost/serialization/serialization.hpp>

#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/srs/epsg.hpp>
#include <boost/geometry/srs/projection.hpp>
#include <boost/geometry/srs/transformation.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "node.hpp"
#include "edge.hpp"
#include "route.hpp"

namespace map_matching_2::geometry::network {

    template<typename Graph>
    class network {

    public:
        using graph_type = Graph;
        using graph_traits = typename boost::graph_traits<Graph>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;
        using node_type = typename Graph::vertex_property_type;
        using edge_type = typename Graph::edge_property_type;
        using point_type = typename node_type::point_type;
        using segment_type = typename edge_type::segment_type;
        using line_type = typename edge_type::line_type;
        using route_type = route<line_type>;
        using rich_line_type = typename edge_type::rich_line_type;
        using coordinate_type = typename boost::geometry::coordinate_type<point_type>::type;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using length_type = typename boost::geometry::default_length_result<line_type>::type;

        using rtree_points_type = boost::geometry::index::rtree<
                std::pair<point_type, vertex_descriptor>, boost::geometry::index::rstar<16>>;
        using rtree_segments_type = boost::geometry::index::rtree<
                std::pair<segment_type, std::pair<std::size_t, edge_descriptor>>, boost::geometry::index::rstar<16>>;

        Graph graph;

        rtree_points_type points_index;
        rtree_segments_type segments_index;

        class tag_helper tag_helper;

        network() = default;

        ~network() = default;

        network(const network &other) = default;

        network(network &&other) noexcept = default;

        network &operator=(const network &other) = default;

        network &operator=(network &&other) noexcept = default;

        vertex_descriptor add_vertex(node_type &node);

        edge_descriptor add_edge(vertex_descriptor u, vertex_descriptor v, edge_type &edge);

        void simplify(bool simplify_network_complete = true);

        void remove_unconnected_components();

        void reindex();

        void rebuild_spatial_indices();

        [[nodiscard]] std::vector<vertex_descriptor> find_node(osmium::object_id_type node_id) const;

        [[nodiscard]] std::vector<edge_descriptor> find_edge(osmium::object_id_type edge_id) const;

        [[nodiscard]] std::pair<bool, vertex_descriptor> point_in_nodes(const point_type &point) const;

        [[nodiscard]] std::pair<bool, std::pair<std::size_t, edge_descriptor>>
        point_in_edges(const point_type &point) const;

        [[nodiscard]] std::pair<bool, edge_descriptor> fitting_edge(const segment_type &segment) const;

        [[nodiscard]] std::vector<vertex_descriptor> make_precedessors() const;

        [[nodiscard]] std::vector<length_type> make_distances() const;

        [[nodiscard]] std::vector<typename boost::default_color_type> make_colors() const;

        [[nodiscard]] std::vector<std::size_t> make_heap_index() const;

        [[nodiscard]] std::vector<vertex_descriptor> make_queue() const;

        [[nodiscard]] std::vector<std::list<vertex_descriptor>>
        dijkstra_shortest_paths(const vertex_descriptor &start, const std::unordered_set<vertex_descriptor> &goals,
                                double max_distance_factor, distance_type max_distance,
                                std::vector<vertex_descriptor> &vertices, std::vector<vertex_descriptor> &predecessors,
                                std::vector<length_type> &distances,
                                std::vector<typename boost::default_color_type> &colors,
                                std::vector<std::size_t> &heap_index,
                                std::vector<vertex_descriptor> &queue) const;

        [[nodiscard]] route_type extract_route(const std::list<vertex_descriptor> &shortest_path) const;

        template<typename Predicate>
        [[nodiscard]] std::vector<std::pair<point_type, vertex_descriptor>>
        query_points(const Predicate &predicate) const {
            std::vector<std::pair<point_type, vertex_descriptor>> results;
            points_index.query(predicate, std::back_inserter(results));
            return results;
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor>>>
        query_segments(const Predicate &predicate) const {
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor>>> results;
            segments_index.query(predicate, std::back_inserter(results));
            return results;
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<point_type>
        query_segments_points(const Predicate &predicate) const {
            return _process_segments_points_result(query_segments(predicate));
        }

        template<typename Predicate>
        [[nodiscard]] std::set<edge_descriptor>
        query_segments_unique(const Predicate &predicate) const {
            return _process_segments_unique_result(query_segments(predicate));
        }

        [[nodiscard]] typename rtree_points_type::bounds_type bounds() const;

        template<typename NetworkConverted>
        void convert(NetworkConverted &network_converted) const;

        template<typename NetworkReprojected>
        void reproject(NetworkReprojected &network_reprojected,
                       const boost::geometry::srs::proj4 &from, const boost::geometry::srs::proj4 &to) const;

        void save_nodes(std::string filename) const;

        void save_edges(std::string filename) const;

        void save(std::string nodes, std::string edges) const;

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & graph;
            ar & tag_helper;
        }

    private:
        void _merge(edge_descriptor in_edge, edge_descriptor out_edge, bool simplify_network_complete = true);

        [[nodiscard]] std::unordered_map<osmium::object_id_type, std::size_t>
        _adjacent_vertices(const vertex_descriptor &vertex) const;

        [[nodiscard]] bool _adjacent_mergable(
                const std::unordered_map<osmium::object_id_type, std::size_t> &adjacent_vertices,
                const vertex_descriptor &vertex) const;

        [[nodiscard]] std::vector<point_type> _process_segments_points_result(
                std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor>>> &&pre_results) const;

        [[nodiscard]] std::set<edge_descriptor> _process_segments_unique_result(
                std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor>>> &&pre_results) const;

        template<typename GraphReprojected>
        struct vertex_reprojector {
            using original_vertex_descriptor = typename graph_type::vertex_descriptor;
            using original_point_type = typename graph_type::vertex_property_type::point_type;
            using reprojected_vertex_descriptor = typename GraphReprojected::vertex_descriptor;
            using reprojected_point_type = typename GraphReprojected::vertex_property_type::point_type;

            vertex_reprojector(const graph_type &graph_original, GraphReprojected &graph_reprojected,
                               const boost::geometry::srs::transformation<> &transformation);

            void operator()(const original_vertex_descriptor &vertex_original,
                            reprojected_vertex_descriptor &vertex_reprojected) const;

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
                             const boost::geometry::srs::transformation<> &transformation);

            void operator()(const original_edge_descriptor &edge_descriptor_original,
                            reprojected_edge_descriptor &edge_descriptor_reprojected) const;

            typename boost::property_map<graph_type, boost::edge_all_t>::const_type edge_all_map_original;
            mutable typename boost::property_map<GraphReprojected, boost::edge_all_t>::type edge_all_map_reprojected;
            const boost::geometry::srs::transformation<> &transformation;
        };

        class vertex_visitor : public boost::default_bfs_visitor {

        private:
            std::unordered_set<vertex_descriptor> &_visited;
            std::unordered_set<vertex_descriptor> &_discovered;

        public:
            vertex_visitor(std::unordered_set<vertex_descriptor> &visited,
                           std::unordered_set<vertex_descriptor> &discovered);

            template<typename graph_type_inner>
            void discover_vertex(vertex_descriptor vertex, const graph_type_inner &_graph);

        };

        struct goal_found_exception : public std::exception {
        };

        struct goals_found_exception : public std::exception {
        };

        template<typename IndexMap, typename WeightMap, typename PredecessorMap, typename DistanceMap, typename ColorMap>
        class goals_visitor : public boost::default_dijkstra_visitor {

        private:
            const distance_type _max_distance;
            const double _max_distance_factor;

            const std::unordered_set<vertex_descriptor> &_goals;
            std::unordered_set<vertex_descriptor> &_goals_found;
            std::vector<vertex_descriptor> &_vertices;

            const IndexMap &_index_map;
            const WeightMap &_weight_map;
            PredecessorMap &_predecessor_map;
            DistanceMap &_distance_map;
            ColorMap &_color_map;

        public:
            explicit goals_visitor(const distance_type max_distance, const double max_distance_factor,
                                   const std::unordered_set<vertex_descriptor> &goals,
                                   std::unordered_set<vertex_descriptor> &goals_found,
                                   std::vector<vertex_descriptor> &vertices, const IndexMap &index_map,
                                   const WeightMap &weight_map, PredecessorMap &predecessor_map,
                                   DistanceMap &distance_map, ColorMap &color_map)
                    : _max_distance{max_distance}, _max_distance_factor{max_distance_factor}, _goals{goals},
                      _goals_found{goals_found}, _vertices{vertices}, _index_map{index_map}, _weight_map{weight_map},
                      _predecessor_map{predecessor_map}, _distance_map{distance_map}, _color_map{color_map} {}

            void discover_vertex(vertex_descriptor vertex, const Graph &_graph) {
                _vertices.emplace_back(vertex);
            }

            void examine_vertex(vertex_descriptor vertex, const Graph &_graph) {
                if (_goals.contains(vertex)) {
                    _goals_found.emplace(vertex);
                }

                if (_goals_found.size() >= _goals.size()) {
                    throw goals_found_exception();
                }
            }

            void examine_edge(edge_descriptor edge, const Graph &_graph) {
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
